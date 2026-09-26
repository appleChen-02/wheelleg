#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "balance_controller.h"
#include "constants.h"
#include "state_reader.h"

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

bool g_paused = false;
bool g_show_help = true;
bool g_console_output = false;
bool g_zero_gravity = false;
bool g_control_enabled = false;
bool g_control_output_enabled = true;
bool g_reset_state_reader = false;
bool g_reset_display_pending = false;
double g_manual_yaw_offset = 0.0;
int g_tp_impulse_count = 0;
double g_stable_accum = 0.0;
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

ControlMode ParseControlMode(const std::string& text) {
    if (text == "notail" || text == "NoTail") return ControlMode::NoTail;
    if (text == "bipedal" || text == "Bipedal") return ControlMode::Bipedal;
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

void SelectInitialState(const mjModel* model) {
    if (model->nkey > 0) {
        const char* name = mj_id2name(model, mjOBJ_KEY, 0);
        std::cout << "Initial state: XML keyframe " << (name ? name : "unnamed") << "\n";
    } else {
        std::cout << "Initial state: XML default state\n";
    }
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
    g_data->time = 0.0;
    // Hold one rendered frame at the exact XML-default pose before control
    // resumes, so the reset state is observable before the first LQR step.
    g_paused = true;
    g_control_enabled = true;
    g_control_output_enabled = false;
    g_reset_state_reader = true;
    g_reset_display_pending = true;
    g_stable_accum = kStableRequired;
    g_next_control_time = 0.0;
    g_next_print_time = g_print_period;
    g_manual_yaw_offset = 0.0;
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
              << " l0=" << state.left_leg.l0 << "," << state.right_leg.l0
              << " wheel=" << command.actuator_torque[4] << "," << command.actuator_torque[5]
              << " tail=" << command.actuator_torque[6]
              << " enabled=" << (g_control_enabled ? "yes" : "no")
              << "\n";
}

void FormatOverlay(char* left, size_t left_size, char* right, size_t right_size,
                   const BalanceController& controller) {
    std::snprintf(left, left_size,
                  "LQR mode\nFPS\nSpeed ref\n\nMouse\nDrag: rotate\nShift+Drag: move\nScroll: zoom\n\nKeys\nM/N: mode\nV: speed\nLeft/Right: yaw\nT: Tp impulse\nP/Space: pause\nR: reset\nE: enable\nK: output\nG: gravity\nH: overlay\nEsc: exit");
    std::snprintf(right, right_size,
                  "%s%s\n%.1f\n%+.2f m/s\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n%s\n%s",
                  controller.ModeName().c_str(),
                  g_paused ? " (PAUSED)" : (!g_control_output_enabled ? " (OUTPUT OFF)" : ""),
                  g_fps, controller.speed_ref(),
                  g_control_enabled ? "controller enabled" : "waiting for stable state",
                  g_zero_gravity ? "zero gravity" : "normal gravity");
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
    if (key == GLFW_KEY_K) {
        g_control_output_enabled = !g_control_output_enabled;
        if (!g_control_output_enabled && g_model && g_data) ClearCtrl(g_model, g_data);
        return;
    }
    if (key == GLFW_KEY_E) {
        g_control_enabled = true;
        g_stable_accum = kStableRequired;
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
        g_reset_state_reader = true;
        return;
    }
    if (key == GLFW_KEY_V && g_controller) {
        g_controller->SetSpeedRef(g_controller->speed_ref() == 0.0 ? 1.0 : 0.0);
        return;
    }
    if (key == GLFW_KEY_LEFT) {
        g_manual_yaw_offset = g_manual_yaw_offset == 2.0 ? 0.0 : 2.0;
        return;
    }
    if (key == GLFW_KEY_RIGHT) {
        g_manual_yaw_offset = g_manual_yaw_offset == -2.0 ? 0.0 : -2.0;
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
        } else if (arg == "--zero-gravity") {
            g_zero_gravity = true;
        } else if (arg == "--mode" && i + 1 < argc) {
            start_mode = ParseControlMode(argv[++i]);
        } else if (arg == "--pulse" && i + 2 < argc) {
            pulse_actuator = argv[++i];
            pulse_torque = ParseDouble(argv[++i], pulse_torque);
        } else if (arg == "--pulse-start" && i + 1 < argc) {
            pulse_start = ParseDouble(argv[++i], pulse_start);
        } else if (arg == "--pulse-duration" && i + 1 < argc) {
            pulse_duration = ParseDouble(argv[++i], pulse_duration);
        } else if (arg == "--help") {
            std::cout << "Usage: lqr_sim [--model path] [--duration seconds] [--mode notail|bipedal|tripod|proxtripod]\n"
                         "                 [--render-fps hz] [--print-period seconds] [--console-output]\n"
                         "                 [--zero-gravity] [--pulse actuator torque_nm]\n";
            return 0;
        }
    }

    model_path = ResolveModelPath(model_path);
    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!model) {
        std::cerr << "Failed to load model: " << model_path << "\n" << error << "\n";
        return 1;
    }
    g_model = model;
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

    try {
        SelectInitialState(model);
        if (!glfwInit()) throw std::runtime_error("Could not initialize GLFW");
        GLFWwindow* window = glfwCreateWindow(1280, 900, "LQR MuJoCo Simulation", nullptr, nullptr);
        if (!window) throw std::runtime_error("Could not create GLFW window");
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        mjv_defaultCamera(&g_camera);
        mjv_defaultOption(&g_option);
        mjv_defaultScene(&g_scene);
        mjr_defaultContext(&g_context);
        mjv_makeScene(model, &g_scene, 2000);
        mjr_makeContext(model, &g_context, mjFONTSCALE_150);
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

        RobotMap map = MakeDefaultRobotMap();
        ResolveRobotMap(model, &map);
        ApplyInitialState(model, data);
        CaptureInitialState(model, data);
        mj_forward(model, data);

        StateReader state_reader;
        BalanceController controller;
        controller.SetMode(start_mode);
        g_controller = &controller;
        g_control_enabled = true;
        g_stable_accum = kStableRequired;
        const int pulse_index = pulse_actuator.empty() ? -1 : FindActuatorByName(map, pulse_actuator);
        if (!pulse_actuator.empty() && pulse_index < 0) {
            throw std::runtime_error("Unknown actuator for --pulse: " + pulse_actuator);
        }

        g_next_control_time = data->time;
        g_next_print_time = data->time + g_print_period;
        const double render_period = 1.0 / std::max(1.0, render_fps);
        double next_render_time = data->time;

        while (!glfwWindowShouldClose(window) && (duration <= 0.0 || data->time < duration)) {
            bool render_reset_frame = false;
            if (g_reset_state_reader) {
                state_reader.Reset();
                g_reset_state_reader = false;
            }
            if (g_reset_display_pending) {
                g_reset_display_pending = false;
                render_reset_frame = true;
            }

            if (!g_paused && data->time + 1e-12 >= g_next_control_time) {
                ClearCtrl(model, data);
                if (g_control_enabled) {
                    g_last_state = state_reader.Read(data, map, kControlPeriod);
                    g_last_command = controller.Compute(g_last_state);

                    if (pulse_index >= 0 && data->time >= pulse_start &&
                        data->time < pulse_start + pulse_duration) {
                        g_last_command.actuator_torque[pulse_index] += pulse_torque;
                    }
                    if (g_manual_yaw_offset != 0.0) {
                        g_last_command.actuator_torque[4] = std::clamp(
                            g_last_command.actuator_torque[4] + g_manual_yaw_offset, -20.0, 20.0);
                        g_last_command.actuator_torque[5] = std::clamp(
                            g_last_command.actuator_torque[5] - g_manual_yaw_offset, -20.0, 20.0);
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
                    if (g_control_output_enabled) {
                        for (int i = 0; i < static_cast<int>(map.actuators.size()); ++i) {
                            SetActuatorTorque(map, data, i, g_last_command.actuator_torque[i]);
                        }
                    }
                }
                g_next_control_time += kControlPeriod;
            }

            if (!g_paused) mj_step(model, data);

            if (data->time + 1e-12 >= g_next_print_time) {
                if (g_console_output) PrintStateSnapshot(data->time, g_last_state, g_last_command);
                g_next_print_time += g_print_period;
            }

            if (g_paused || data->time + 1e-12 >= next_render_time) {
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
            glfwPollEvents();
        }

        mjr_freeContext(&g_context);
        mjv_freeScene(&g_scene);
        glfwDestroyWindow(window);
        glfwTerminate();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        mjr_freeContext(&g_context);
        mjv_freeScene(&g_scene);
        glfwTerminate();
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
