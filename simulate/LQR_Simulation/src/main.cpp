#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>
#endif

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "balance_controller.h"
#include "constants.h"
#include "state_reader.h"
#include "system_identification.h"

#ifndef DEFAULT_MODEL_PATH
#define DEFAULT_MODEL_PATH "models/scene.xml"
#endif

namespace {

mjvCamera g_camera;
mjvOption g_option;
mjvScene g_scene;
mjrContext g_context;
mjModel* g_model = nullptr;
mjData* g_data = nullptr;
BalanceController* g_controller = nullptr;
SystemIdentificationExperiment* g_sysid = nullptr;
std::atomic<bool> g_stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free);

void StopSignalHandler(int) { g_stop_requested.store(true, std::memory_order_relaxed); }
#ifdef _WIN32
BOOL WINAPI ConsoleStopHandler(DWORD event) {
    if (event != CTRL_C_EVENT && event != CTRL_BREAK_EVENT) return FALSE;
    g_stop_requested.store(true, std::memory_order_relaxed);
    return TRUE;
}
#endif

bool g_paused = false;
bool g_show_help = true;
bool g_console_output = false;
bool g_zero_gravity = false;
bool g_control_enabled = false;
bool g_control_output_enabled = true;
bool g_reset_state_reader = false;
bool g_reset_motion_state = false;
bool g_reset_display_pending = false;
bool g_keyboard_motion_active = false;
bool g_stop_latched = false;
int g_tp_impulse_count = 0;
FeedbackSource g_feedback_source = FeedbackSource::Truth;
double g_next_control_time = 0.0;
double g_next_print_time = 0.0;
double g_print_period = kDefaultPrintPeriod;
ControlCommand g_last_command;
RobotState g_last_state;
std::vector<double> g_initial_qpos;
std::vector<double> g_initial_qvel;
std::array<double, 3> g_initial_gravity{};

bool g_mouse_left = false;
bool g_mouse_middle = false;
bool g_mouse_right = false;
double g_mouse_last_x = 0.0;
double g_mouse_last_y = 0.0;

double g_fps = 0.0;
int g_frame_counter = 0;
auto g_fps_window_start = std::chrono::steady_clock::now();

double ParseDouble(const std::string& text, double fallback) {
    try {
        return std::stod(text);
    } catch (...) {
        return fallback;
    }
}

std::filesystem::path ParseOutputPath(int index, const char* argument) {
#ifdef _WIN32
    // Windows 的窄 argv 可能是本地代码页；保存路径直接读取 Unicode 命令行。
    int count = 0;
    auto arguments = CommandLineToArgvW(GetCommandLineW(), &count);
    if (!arguments) throw std::runtime_error("Could not read Unicode command line");
    std::filesystem::path result;
    if (index < count) result = arguments[index];
    LocalFree(arguments);
    if (result.empty()) throw std::runtime_error("Missing output directory");
    return result;
#else
    (void)index;
    return std::filesystem::path(argument);
#endif
}

ControlMode ParseControlMode(const std::string& text) {
    if (text == "notail" || text == "NoTail") return ControlMode::NoTail;
    if (text == "bipedal" || text == "Bipedal") return ControlMode::Bipedal;
    if (text == "sysid" || text == "SystemIdentification") return ControlMode::SystemIdentification;
    if (text == "proxtripod" || text == "ProXTripod" || text == "proX") {
        return ControlMode::ProXTripod;
    }
    return ControlMode::Tripod;
}

std::string ResolveModelPath(const std::string& requested) {
    const std::vector<std::string> candidates = {
        requested,
        DEFAULT_MODEL_PATH,
        "../models/scene.xml",
        "../../models/scene.xml",
    };
    for (const auto& path : candidates) {
        std::ifstream file(path);
        if (!path.empty() && file.good()) return path;
    }
    return requested;
}

void SelectInitialState() {
    std::cout << "Initial state: XML default state\n";
}

void ApplyInitialState(mjModel* model, mjData* data) {
    // Always restore MuJoCo's XML default qpos/qvel. Keyframes are not used
    // for startup or reset, and no extra initial tilt is injected.
    mj_resetData(model, data);
    mj_forward(model, data);
}

void CaptureInitialState(const mjModel* model, const mjData* data) {
    g_initial_qpos.assign(data->qpos, data->qpos + model->nq);
    g_initial_qvel.assign(data->qvel, data->qvel + model->nv);
    for (int i = 0; i < 3; ++i) g_initial_gravity[i] = model->opt.gravity[i];
}

void RestoreInitialState() {
    if (!g_model || !g_data || g_initial_qpos.size() != static_cast<size_t>(g_model->nq) ||
        g_initial_qvel.size() != static_cast<size_t>(g_model->nv)) {
        return;
    }
    mj_resetData(g_model, g_data);
    std::copy(g_initial_qpos.begin(), g_initial_qpos.end(), g_data->qpos);
    std::copy(g_initial_qvel.begin(), g_initial_qvel.end(), g_data->qvel);
    for (int i = 0; i < 3; ++i) g_model->opt.gravity[i] = g_initial_gravity[i];
    mj_forward(g_model, g_data);
}

void ResetSimulation() {
    if (!g_model || !g_data) return;
    RestoreInitialState();
    g_zero_gravity = std::fabs(g_initial_gravity[2]) < 1e-9;
    g_data->time = 0.0;
    // Hold one rendered frame at the exact XML-default pose before control
    // resumes, so the reset state is observable before the first LQR step.
    g_paused = true;
    g_control_enabled = true;
    g_control_output_enabled = false;
    g_reset_state_reader = true;
    g_reset_display_pending = true;
    g_next_control_time = 0.0;
    g_next_print_time = g_print_period;
    g_keyboard_motion_active = false;
    g_stop_latched = false;
    g_reset_motion_state = false;
    g_tp_impulse_count = 0;
    g_last_command = {};
    g_last_state = {};
    if (g_controller) g_controller->Reset();
    g_fps = 0.0;
    g_frame_counter = 0;
    g_fps_window_start = std::chrono::steady_clock::now();
}

void PrintStateSnapshot(double time, const RobotState& state, const ControlCommand& command) {
    std::cout << std::fixed << std::setprecision(4)
              << "t=" << time
              << " mode=" << (g_controller ? g_controller->ModeName() : "unknown")
              << " pitch=" << state.body.pitch
              << " pitch_rate=" << state.body.pitch_rate
              << " x=" << state.body.x
              << " x_dot=" << state.body.x_dot
              << " speed_ref=" << command.speed_ref
              << " yaw=" << state.body.yaw
              << " yaw_ref=" << command.yaw_ref
              << " yaw_rate_ref=" << command.yaw_rate_ref
              << " beta=" << state.beta
              << " beta_ref=" << command.ref_beta
              << " l0=" << state.left_leg.l0 << "," << state.right_leg.l0
              << " wheel=" << command.actuator_torque[4] << "," << command.actuator_torque[5]
              << " tail=" << command.actuator_torque[6]
              << " enabled=" << (g_control_enabled && g_control_output_enabled ? "yes" : "no")
              << "\n";
}

void FormatOverlay(char* left, size_t left_size, char* right, size_t right_size,
                   const BalanceController& controller) {
    if (g_sysid) {
        std::snprintf(left, left_size, "Mode\nPhase\nFeedback\nPitch\nTail\nLeg L / R\n\nEsc: stop and save\nP/Space: pause\nMotion/reset/output keys locked");
        std::snprintf(right, right_size, "%s\n%s\n%s\n%+.3f rad\n%+.3f rad\n%.3f / %.3f m",
                      controller.ModeName().c_str(), g_sysid->PhaseName(),
                      g_feedback_source == FeedbackSource::Truth ? "truth" : "MCU observer",
                      g_last_state.body.pitch, g_last_state.beta, g_last_state.left_leg.l0, g_last_state.right_leg.l0);
        return;
    }
    std::snprintf(left, left_size,
                  "LQR mode\nFPS\nFeedback\nSpeed cmd / ref\nYaw rate cmd\nYaw ref / actual\nLeg target L / R\nTail cmd / ref\nTail actual\n\nKeys\nW/S: move (hold)\nA/D: turn (hold)\nShift: slower\nQ/E: leg shorter/longer\nZ/C: tail angle -/+\nX: stop motion\nHome: neutral posture\nT: Tp disturbance\nM/N: mode\nP/Space: pause\nR: reset\nK: output\nG: gravity\nH: overlay\nEsc: exit\n\nMouse: drag/scroll");
    std::snprintf(right, right_size,
                  "%s%s%s\n%.1f\n%s\n%+.2f / %+.2f m/s\n%+.2f rad/s\n%+.2f / %+.2f rad\n%.3f / %.3f m\n%+.2f / %+.2f rad\n%+.2f rad",
                  controller.ModeName().c_str(),
                  g_paused ? " (PAUSED)" : (!g_control_output_enabled ? " (OUTPUT OFF)" : ""),
                  g_zero_gravity ? " (ZERO GRAVITY)" : "",
                  g_fps, g_feedback_source == FeedbackSource::Truth ? "truth" : "MCU observer",
                  controller.speed_ref(), g_last_command.speed_ref, controller.yaw_rate_ref(),
                  g_last_command.yaw_ref, g_last_state.body.yaw,
                  g_last_command.ref_l0[0], g_last_command.ref_l0[1],
                  controller.tail_angle_ref(), g_last_command.ref_beta, g_last_state.beta);
}

void UpdateKeyboardReferences(GLFWwindow* window, BalanceController& controller) {
    if (controller.mode() == ControlMode::SystemIdentification) return;
    if (!window) return;
    if (!glfwGetWindowAttrib(window, GLFW_FOCUSED)) {
        controller.StopMotion();
        g_keyboard_motion_active = false;
        return;
    }
    const auto pressed = [window](int key) { return glfwGetKey(window, key) == GLFW_PRESS; };
    const int forward = static_cast<int>(pressed(GLFW_KEY_W)) - static_cast<int>(pressed(GLFW_KEY_S));
    const int turn = static_cast<int>(pressed(GLFW_KEY_A)) - static_cast<int>(pressed(GLFW_KEY_D));
    const bool movement_keys_held = pressed(GLFW_KEY_W) || pressed(GLFW_KEY_S)
                          || pressed(GLFW_KEY_A) || pressed(GLFW_KEY_D);
    if (g_stop_latched && !movement_keys_held) g_stop_latched = false;
    if (g_stop_latched) {
        controller.StopMotion();
    } else if (movement_keys_held || g_keyboard_motion_active) {
        const bool slow = pressed(GLFW_KEY_LEFT_SHIFT) || pressed(GLFW_KEY_RIGHT_SHIFT);
        controller.SetSpeedRef(forward * (slow ? kKeyboardSlowSpeed : kKeyboardSpeed));
        controller.SetYawRateRef(turn * (slow ? kKeyboardSlowYawRate : kKeyboardYawRate));
        g_keyboard_motion_active = movement_keys_held;
    }
    if (controller.mode() == ControlMode::Bipedal) {
        const int leg = static_cast<int>(pressed(GLFW_KEY_E)) - static_cast<int>(pressed(GLFW_KEY_Q));
        const int tail = static_cast<int>(pressed(GLFW_KEY_C)) - static_cast<int>(pressed(GLFW_KEY_Z));
        if (leg) controller.SetLegLengthRef(controller.leg_length_ref() + leg * kKeyboardLegLengthRate * kControlPeriod);
        if (tail) controller.SetTailAngleRef(controller.tail_angle_ref() + tail * kKeyboardTailAngleRate * kControlPeriod);
    }
}

void KeyboardCallback(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        return;
    }
    if (key == GLFW_KEY_P || key == GLFW_KEY_SPACE) {
        g_paused = !g_paused;
        return;
    }
    if (key == GLFW_KEY_H) {
        g_show_help = !g_show_help;
        return;
    }
    if (g_sysid) {
        // 数据采集中不允许运动、改工作点、重置时钟或插入额外扰动。
        return;
    }
    if (key == GLFW_KEY_K) {
        g_control_output_enabled = !g_control_output_enabled;
        if (!g_control_output_enabled && g_model && g_data) ClearCtrl(g_model, g_data);
        return;
    }
    if (key == GLFW_KEY_G && g_model) {
        g_zero_gravity = !g_zero_gravity;
        g_model->opt.gravity[2] = g_zero_gravity ? 0.0 : -kDefaultGravity;
        return;
    }
    if ((key == GLFW_KEY_M || key == GLFW_KEY_N) && g_controller) {
        const int next = (static_cast<int>(g_controller->mode()) + 1) % 4;
        g_controller->SetMode(static_cast<ControlMode>(next));
        g_controller->StopMotion();
        g_reset_motion_state = true;
        return;
    }
    if (key == GLFW_KEY_X && g_controller) {
        g_controller->StopMotion();
        g_stop_latched = true;
        return;
    }
    if (key == GLFW_KEY_HOME && g_controller) {
        g_controller->StopMotion();
        g_controller->SetLegLengthRef(0.18);
        g_controller->SetTailAngleRef(0.0);
        g_stop_latched = true;
        return;
    }
    if (key == GLFW_KEY_W || key == GLFW_KEY_S || key == GLFW_KEY_A || key == GLFW_KEY_D) {
        g_keyboard_motion_active = true;
        return;
    }
    if (key == GLFW_KEY_T) {
        g_tp_impulse_count = 25;
        return;
    }
    if (key == GLFW_KEY_R) {
        ResetSimulation();
    }
}

void MouseButtonCallback(GLFWwindow* window, int, int, int) {
    g_mouse_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    g_mouse_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    g_mouse_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &g_mouse_last_x, &g_mouse_last_y);
}

void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    if (!g_mouse_left && !g_mouse_middle && !g_mouse_right) return;
    const double dx = xpos - g_mouse_last_x;
    const double dy = ypos - g_mouse_last_y;
    g_mouse_last_x = xpos;
    g_mouse_last_y = ypos;
    int width = 1;
    int height = 1;
    glfwGetWindowSize(window, &width, &height);
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    mjtMouse action = mjMOUSE_ZOOM;
    if (g_mouse_right) action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    if (g_mouse_left) action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    mjv_moveCamera(g_model, action, dx / std::max(1, width),
                   dy / std::max(1, height), &g_camera);
}

void ScrollCallback(GLFWwindow*, double, double yoffset) {
    mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &g_camera);
}

}  // namespace

int main(int argc, char** argv) {
    std::string model_path = DEFAULT_MODEL_PATH;
    std::string pulse_actuator;
    double pulse_torque = kDefaultPulseTorque;
    double pulse_start = kDefaultPulseStart;
    double pulse_duration = kDefaultPulseDuration;
    double render_fps = kDefaultRenderFps;
    double duration = kDefaultDuration;
    ControlMode start_mode = ControlMode::Bipedal;
    bool headless = false;
    double initial_speed = 0.0;
    double initial_yaw_rate = 0.0;
    double initial_leg_length = 0.18;
    double initial_tail_angle = 0.0;
    std::filesystem::path sysid_outdir;

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--model" && i + 1 < argc) {
            model_path = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = ParseDouble(argv[++i], duration);
        } else if (arg == "--render-fps" && i + 1 < argc) {
            render_fps = ParseDouble(argv[++i], render_fps);
        } else if (arg == "--print-period" && i + 1 < argc) {
            g_print_period = ParseDouble(argv[++i], g_print_period);
        } else if (arg == "--console-output") {
            g_console_output = true;
        } else if (arg == "--headless") {
            headless = true;
        } else if (arg == "--feedback" && i + 1 < argc) {
            const std::string source = argv[++i];
            if (source == "truth") g_feedback_source = FeedbackSource::Truth;
            else if (source == "mcu") g_feedback_source = FeedbackSource::McuObserver;
            else { std::cerr << "--feedback must be truth or mcu\n"; return 1; }
        } else if (arg == "--speed" && i + 1 < argc) {
            initial_speed = ParseDouble(argv[++i], 0.0);
        } else if (arg == "--yaw-rate" && i + 1 < argc) {
            initial_yaw_rate = ParseDouble(argv[++i], 0.0);
        } else if (arg == "--leg-length" && i + 1 < argc) {
            initial_leg_length = ParseDouble(argv[++i], 0.18);
        } else if (arg == "--tail-angle" && i + 1 < argc) {
            initial_tail_angle = ParseDouble(argv[++i], 0.0);
        } else if (arg == "--zero-gravity") {
            g_zero_gravity = true;
        } else if (arg == "--mode" && i + 1 < argc) {
            start_mode = ParseControlMode(argv[++i]);
        } else if (arg == "--outdir" && i + 1 < argc) {
            ++i;
            try { sysid_outdir = ParseOutputPath(i, argv[i]); }
            catch (const std::exception& e) { std::cerr << "Invalid output path: " << e.what() << "\n"; return 1; }
        } else if (arg == "--pulse" && i + 2 < argc) {
            pulse_actuator = argv[++i];
            pulse_torque = ParseDouble(argv[++i], pulse_torque);
        } else if (arg == "--pulse-start" && i + 1 < argc) {
            pulse_start = ParseDouble(argv[++i], pulse_start);
        } else if (arg == "--pulse-duration" && i + 1 < argc) {
            pulse_duration = ParseDouble(argv[++i], pulse_duration);
        } else if (arg == "--help") {
            std::cout << "Usage: lqr_sim [--model path] [--duration seconds] [--mode notail|bipedal|tripod|proxtripod|sysid]\n"
                         "                 [--render-fps hz] [--print-period seconds] [--console-output]\n"
                         "                 [--headless] [--feedback truth|mcu]\n"
                         "                 [--speed m/s] [--yaw-rate rad/s] [--leg-length m] [--tail-angle rad]\n"
                         "                 [--zero-gravity] [--pulse actuator torque_nm]\n"
                         "                 [--outdir directory] (sysid CSV output)\n"
                         "SYSID parameters: src/system_identification_param.h (rebuild after editing).\n"
                         "SYSID: wait for stability, record baseline, excite, capture post-response; auto-save and exit after "
                      << SYS_ID_AUTO_EXIT_DELAY_SECONDS << " more simulation seconds.\n"
                      << "Esc / Ctrl+C can stop and save early.\n"
                         "--duration is total simulation time; stopping early produces an incomplete capture.\n";
            return 0;
        }
    }

    if (start_mode == ControlMode::SystemIdentification) {
        if (g_zero_gravity || !pulse_actuator.empty() || initial_speed != 0.0 || initial_yaw_rate != 0.0 ||
            initial_leg_length != 0.18 || initial_tail_angle != 0.0) {
            std::cerr << "SYSID uses fixed references from system_identification_param.h; motion, posture, gravity and pulse overrides are not allowed.\n";
            return 1;
        }
        if (sysid_outdir.empty()) {
            const auto now = std::chrono::system_clock::now();
            const auto epoch = std::chrono::system_clock::to_time_t(now);
            char stamp[32] = {};
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&epoch));
            const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
            sysid_outdir = std::filesystem::path("logs") / (std::string("sysid_") + stamp + "_" + std::to_string(millis));
        }
    } else if (!sysid_outdir.empty()) {
        std::cerr << "--outdir requires --mode sysid\n";
        return 1;
    }
    std::signal(SIGINT, StopSignalHandler);
    std::signal(SIGTERM, StopSignalHandler);
#ifdef SIGBREAK
    std::signal(SIGBREAK, StopSignalHandler);
#endif
#ifdef _WIN32
    // 某些终端会继承“忽略 Ctrl+C”，显式恢复并安装 Windows 控制事件处理器。
    SetConsoleCtrlHandler(nullptr, FALSE);
    SetConsoleCtrlHandler(ConsoleStopHandler, TRUE);
#endif

    model_path = ResolveModelPath(model_path);
    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!model) {
        std::cerr << "Failed to load model: " << model_path << "\n" << error << "\n";
        return 1;
    }
    g_model = model;
    model->opt.timestep = kPhysicsPeriod;
    if (g_zero_gravity) {
        model->opt.gravity[0] = 0.0;
        model->opt.gravity[1] = 0.0;
        model->opt.gravity[2] = 0.0;
    }

    mjData* data = mj_makeData(model);
    if (!data) {
        mj_deleteModel(model);
        return 1;
    }
    g_data = data;
    if (headless && duration <= 0.0 && start_mode != ControlMode::SystemIdentification) duration = 5.0;
    GLFWwindow* window = nullptr;
    bool glfw_initialized = false;
    bool render_initialized = false;

    try {
        SelectInitialState();
        if (!headless) {
            if (!glfwInit()) throw std::runtime_error("Could not initialize GLFW");
            glfw_initialized = true;
            window = glfwCreateWindow(1280, 900, "LQR MuJoCo Simulation", nullptr, nullptr);
            if (!window) throw std::runtime_error("Could not create GLFW window");
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1);

            mjv_defaultCamera(&g_camera);
            mjv_defaultOption(&g_option);
            mjv_defaultScene(&g_scene);
            mjr_defaultContext(&g_context);
            mjv_makeScene(model, &g_scene, 2000);
            mjr_makeContext(model, &g_context, mjFONTSCALE_150);
            render_initialized = true;
            g_camera.azimuth = kDefaultCameraAzimuth;
            g_camera.elevation = kDefaultCameraElevation;
            g_camera.distance = kDefaultCameraDistance;
            g_camera.lookat[0] = kDefaultCameraLookatX;
            g_camera.lookat[1] = kDefaultCameraLookatY;
            g_camera.lookat[2] = kDefaultCameraLookatZ;
            glfwSetKeyCallback(window, KeyboardCallback);
            glfwSetCursorPosCallback(window, MouseMoveCallback);
            glfwSetMouseButtonCallback(window, MouseButtonCallback);
            glfwSetScrollCallback(window, ScrollCallback);
        }

        RobotMap map = MakeDefaultRobotMap();
        ResolveRobotMap(model, &map);
        ApplyInitialState(model, data);
        CaptureInitialState(model, data);
        mj_forward(model, data);

        StateReader state_reader(g_feedback_source);
        BalanceController controller;
        controller.SetMode(start_mode);
        controller.SetSpeedRef(initial_speed);
        controller.SetYawRateRef(initial_yaw_rate);
        if (start_mode == ControlMode::Bipedal) {
            controller.SetLegLengthRef(initial_leg_length);
            controller.SetTailAngleRef(initial_tail_angle);
        }
        g_controller = &controller;
        std::unique_ptr<SystemIdentificationExperiment> sysid;
        if (start_mode == ControlMode::SystemIdentification) {
            sysid = std::make_unique<SystemIdentificationExperiment>(model, map, sysid_outdir, g_feedback_source);
            g_sysid = sysid.get();
        }
        g_control_enabled = true;
        std::cout << "Feedback: " << (g_feedback_source == FeedbackSource::Truth ? "truth" : "MCU observer (ideal zero bias)")
                  << "; physics=1 ms, control=2 ms; no startup calibration\n";
        const int pulse_index = pulse_actuator.empty() ? -1 : FindActuatorByName(map, pulse_actuator);
        if (!pulse_actuator.empty() && pulse_index < 0) {
            throw std::runtime_error("Unknown actuator for --pulse: " + pulse_actuator);
        }

        g_next_control_time = data->time;
        g_next_print_time = data->time + g_print_period;
        const double render_period = 1.0 / std::max(1.0, render_fps);
        double next_render_time = data->time;

        const auto run_wall_start = std::chrono::steady_clock::now();
        while (!g_stop_requested && (!window || !glfwWindowShouldClose(window)) &&
               (duration <= 0.0 || data->time < duration) && (!sysid || !sysid->ShouldAutoExit(data->time))) {
            bool render_reset_frame = false;
            if (g_reset_state_reader) {
                state_reader.Reset();
                g_reset_state_reader = false;
            }
            if (g_reset_motion_state) {
                state_reader.ResetMotionStateForModeSwitch();
                g_reset_motion_state = false;
            }
            if (g_reset_display_pending) {
                g_reset_display_pending = false;
                render_reset_frame = true;
            }

            if (!g_paused && data->time + 0.5 * kPhysicsPeriod >= g_next_control_time) {
                ClearCtrl(model, data);
                g_last_state = state_reader.Read(data, map, kControlPeriod);
                if (g_control_enabled) {
                    UpdateKeyboardReferences(window, controller);
                    const auto excitation = sysid ? sysid->Advance(data->time, g_last_state) : std::array<double, 5>{};
                    g_last_command = controller.Compute(g_last_state, excitation);
                    state_reader.SetSpeedReference(g_last_command.speed_ref);

                    if (pulse_index >= 0 && data->time >= pulse_start &&
                        data->time < pulse_start + pulse_duration) {
                        g_last_command.actuator_torque[pulse_index] += pulse_torque;
                    }
                    if (g_tp_impulse_count > 0) {
                        g_last_command.vmc_Tp[0] += 20.0;
                        g_last_command.vmc_Tp[1] += 20.0;
                        double left[2] = {};
                        double right[2] = {};
                        CalcVmc(g_last_command.vmc_F[0], g_last_command.vmc_Tp[0],
                                g_last_state.left_leg.jacobian, left);
                        CalcVmc(g_last_command.vmc_F[1], g_last_command.vmc_Tp[1],
                                g_last_state.right_leg.jacobian, right);
                        g_last_command.actuator_torque[0] = left[1];
                        g_last_command.actuator_torque[1] = left[0];
                        g_last_command.actuator_torque[2] = right[1];
                        g_last_command.actuator_torque[3] = right[0];
                        --g_tp_impulse_count;
                    }
                    for (int i = 0; i < static_cast<int>(map.actuators.size()); ++i) {
                        // 扰动同样经过最终限幅，输出关闭时记录零力矩。
                        const double limit = i < 4 ? kDefaultMaxJointTorque
                            : (i < 6 ? kDefaultMaxWheelTorque : kDefaultMaxTailTorque);
                        g_last_command.actuator_torque[i] = g_control_output_enabled
                            ? std::clamp(g_last_command.actuator_torque[i], -limit, limit) : 0.0;
                        SetActuatorTorque(map, data, i, g_last_command.actuator_torque[i]);
                    }
                    if (sysid) {
                        // 当前 XML 是无动态纯力矩电机；同步计算本拍实际输出，避免记录上一拍 actuator_force。
                        mj_fwdActuation(model, data);
                        sysid->Record(data->time, g_last_state, g_last_command, data);
                    }
                }
                // 以整数控制拍确定下次时间，避免长实验累计浮点误差造成 1 ms 错位。
                g_next_control_time = (std::llround(data->time / kControlPeriod) + 1) * kControlPeriod;
            }

            if (!g_paused) {
                mj_step(model, data);
                state_reader.UpdateImuRoll(data, map);
                if (headless && sysid && duration <= 0.0) {
                    // 未指定时长的无窗口实验按实时运行，支持手动停止或采集完成后自动退出。
                    std::this_thread::sleep_until(run_wall_start +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(data->time)));
                }
            }

            if (data->time + 1e-12 >= g_next_print_time) {
                if (g_console_output) PrintStateSnapshot(data->time, g_last_state, g_last_command);
                g_next_print_time += g_print_period;
            }

            if (window && (g_paused || data->time + 1e-12 >= next_render_time)) {
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
                mjv_updateScene(model, data, &g_option, nullptr, &g_camera, mjCAT_ALL, &g_scene);
                mjr_render(viewport, &g_scene, &g_context);
                ++g_frame_counter;
                const auto now = std::chrono::steady_clock::now();
                const double elapsed = std::chrono::duration<double>(now - g_fps_window_start).count();
                if (elapsed >= 0.5) {
                    g_fps = static_cast<double>(g_frame_counter) / elapsed;
                    g_frame_counter = 0;
                    g_fps_window_start = now;
                }
                if (g_show_help) {
                    char left[1024] = {};
                    char right[1024] = {};
                    FormatOverlay(left, sizeof(left), right, sizeof(right), controller);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left, right, &g_context);
                }
                glfwSwapBuffers(window);
                next_render_time = data->time + render_period;
                if (render_reset_frame) {
                    g_paused = false;
                    g_control_output_enabled = true;
                    g_control_enabled = true;
                    g_next_control_time = data->time;
                }
            }
            if (window) glfwPollEvents();
        }

        if (sysid) {
            const std::string reason = g_stop_requested ? "signal" :
                (window && glfwWindowShouldClose(window) ? "manual_window_stop" :
                 (sysid->ShouldAutoExit(data->time) ? "auto_exit_after_capture" : "duration_limit"));
            sysid->Finish(reason);
        }
        g_sysid = nullptr;
        g_controller = nullptr;

        if (render_initialized) {
            mjr_freeContext(&g_context);
            mjv_freeScene(&g_scene);
        }
        if (window) glfwDestroyWindow(window);
        if (glfw_initialized) glfwTerminate();
    } catch (const std::exception& ex) {
        g_sysid = nullptr;
        g_controller = nullptr;
        std::cerr << "Error: " << ex.what() << "\n";
        if (render_initialized) {
            mjr_freeContext(&g_context);
            mjv_freeScene(&g_scene);
        }
        if (window) glfwDestroyWindow(window);
        if (glfw_initialized) glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
        g_data = nullptr;
        g_model = nullptr;
        return 1;
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    g_data = nullptr;
    g_model = nullptr;
    return 0;
}
