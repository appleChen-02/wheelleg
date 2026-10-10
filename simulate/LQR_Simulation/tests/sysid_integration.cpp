#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "system_identification.h"

namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

RobotState StableState() {
    RobotState s;
    s.left_leg.l0 = s.right_leg.l0 = 0.18;
    for (auto* leg : {&s.left_leg, &s.right_leg}) {
        leg->jacobian[0][0] = 1.0;
        leg->jacobian[1][1] = 1.0;
    }
    return s;
}

void CheckLogSweep(mjModel* model, const RobotMap& map, const std::filesystem::path& outdir) {
    SystemIdentificationParameters p;
    p.excitation_enable = true;
    p.excitation_mode = SystemIdentificationExcitation::SineSweep;
    p.input_mask = 0x01;
    p.amplitude[0] = 0.1;
    p.sweep_f0 = 0.2;
    p.sweep_f1 = 20.0;
    p.sweep_duration = 240.0;
    p.min_settle_seconds = p.pre_seconds = p.post_seconds = 0.0;
    p.stable_seconds = kControlPeriod;
    const auto state = StableState();
    SystemIdentificationExperiment e(model, map, outdir / "log_sweep", FeedbackSource::Truth, p);
    e.Advance(0.0, state);
    Require(e.Advance(kControlPeriod, state) == std::array<double, 5>{}, "Sweep must start at zero phase");
    double previous = 0.0;
    unsigned crossings = 0;
    unsigned low_frequency_crossings = 0;
    const unsigned samples = static_cast<unsigned>(std::llround(p.sweep_duration / kControlPeriod));
    for (unsigned n = 1; n < samples; ++n) {
        const double tau = n * kControlPeriod;
        const auto input = e.Advance((n + 1) * kControlPeriod, state);
        Require(std::isfinite(input[0]) && std::abs(input[0]) <= p.amplitude[0], "Sweep amplitude is invalid");
        for (int i = 1; i < 5; ++i) Require(input[i] == 0.0, "Sweep leaked into an unselected channel");
        if (previous < 0.0 && input[0] >= 0.0) {
            ++crossings;
            // Invert the integrated frequency at each completed cycle to verify the entire sweep.
            const double expected = 240.0 / std::log(100.0) *
                std::log(1.0 + crossings * std::log(100.0) / 48.0);
            const double observed = tau - kControlPeriod +
                kControlPeriod * (-previous) / (input[0] - previous);
            Require(std::abs(observed - expected) < 0.00002, "Log sweep zero crossing timing differs");
            if (observed < 240.0 * std::log(5.0) / std::log(100.0)) ++low_frequency_crossings;
        }
        previous = input[0];
    }
    Require(crossings == 1031, "Unexpected total log sweep cycle count");
    Require(low_frequency_crossings == 41, "Log sweep did not provide 41 complete cycles below 1 Hz");
    Require(e.Advance(p.sweep_duration + kControlPeriod, state) == std::array<double, 5>{}, "Sweep did not stop at T");
    Require(e.phase() == SystemIdentificationExperiment::Phase::Complete, "Sweep did not complete");
    e.Finish("log_sweep_test");

    p.sweep_f0 = 0.0;
    bool rejected = false;
    try {
        SystemIdentificationExperiment invalid(model, map, outdir / "zero_start", FeedbackSource::Truth, p);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected, "Log sweep accepted a zero start frequency");
}

void RunPhysicalCase(mjModel* model, const RobotMap& map, const std::filesystem::path& outdir,
                     SystemIdentificationParameters p) {
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model), mj_deleteData);
    mj_forward(model, data.get());
    StateReader reader;
    BalanceControllerConfig config;
    config.sys_id = p;
    BalanceController controller(config);
    controller.SetMode(ControlMode::SystemIdentification);
    SystemIdentificationExperiment experiment(model, map, outdir, FeedbackSource::Truth, p);
    double next_control = 0.0;
    RobotState last_state;
    while (data->time < 35.0) {
        if (data->time + 0.5 * kPhysicsPeriod >= next_control) {
            last_state = reader.Read(data.get(), map, kControlPeriod);
            const auto excitation = experiment.Advance(data->time, last_state);
            const auto command = controller.Compute(last_state, excitation);
            for (int i = 0; i < 7; ++i) SetActuatorTorque(map, data.get(), i, command.actuator_torque[i]);
            mj_fwdActuation(model, data.get());
            experiment.Record(data->time, last_state, command, data.get());
            next_control = (std::llround(data->time / kControlPeriod) + 1) * kControlPeriod;
        }
        mj_step(model, data.get());
        reader.UpdateImuRoll(data.get(), map);
    }
    Require(experiment.phase() == SystemIdentificationExperiment::Phase::Complete, "Physical experiment did not finish");
    experiment.Finish("integration_test");

    // 固定 K/theta_eq/T0 应不随实际腿长改变；腿长 PID 的径向力允许改变。
    const auto c1 = controller.Compute(last_state);
    auto changed_length = last_state;
    changed_length.left_leg.l0 += 0.01;
    changed_length.right_leg.l0 -= 0.01;
    const auto c2 = controller.Compute(changed_length);
    Require(c1.controller_output == c2.controller_output, "SYSID gain work point follows actual leg length");
    Require(c1.reference_state == c2.reference_state, "SYSID reference changes with actual leg length");
    for (int i = 0; i < 5; ++i) Require(c1.equilibrium_input[i] == c2.equilibrium_input[i], "SYSID equilibrium follows actual length");

    // 用大髋/轮/尾激励校验广义输入未被日志限幅，电机确实被限幅且饱和位正确。
    const std::array<double, 5> large = {1000.0, -1000.0, 100.0, -100.0, 100.0};
    const auto saturated = controller.Compute(last_state, large);
    Require(saturated.motor_saturation_mask == 0x7F, "Motor saturation bits are incorrect");
    Require(std::abs(saturated.generalized_command[4]) > kDefaultMaxTailTorque, "Raw generalized command was clipped");
    Require(std::abs(saturated.actuator_torque[6]) == kDefaultMaxTailTorque, "Tail actuator was not clipped");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 3, "Usage: SysIdIntegration model.xml output_parent");
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        model->opt.timestep = kPhysicsPeriod;
        auto map = MakeDefaultRobotMap();
        ResolveRobotMap(model.get(), &map);
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto root = std::filesystem::path(argv[2]) / std::to_string(stamp);

        CheckLogSweep(model.get(), map, root);
        SystemIdentificationParameters p;
        p.excitation_mode = SystemIdentificationExcitation::Prbs;
        p.input_mask = 0x1F;
        p.amplitude = {0.03, 0.03, 0.03, 0.03, 0.03};
        p.prbs_duration = 2.0;
        p.pre_seconds = p.post_seconds = 0.1;
        RunPhysicalCase(model.get(), map, root / "prbs", p);
        p.excitation_enable = false;
        p.baseline_duration = 2.0;
        RunPhysicalCase(model.get(), map, root / "no_excitation", p);
        p.excitation_enable = true;
        p.excitation_mode = SystemIdentificationExcitation::SineSweep;
        p.input_mask = 0x10;
        p.sweep_f0 = 0.2;
        p.sweep_f1 = 20.0;
        p.sweep_duration = 2.0;
        RunPhysicalCase(model.get(), map, root / "sine_sweep", p);

        // 稳定确认被运动打断时不得提前启动；超时无激励，不伪报完成。
        p.min_settle_seconds = 0.0;
        p.stable_seconds = 0.02;
        p.settle_timeout_seconds = 0.1;
        auto s = StableState();
        {
            SystemIdentificationExperiment e(model.get(), map, root / "timeout", FeedbackSource::Truth, p);
            e.Advance(0.0, s);
            s.body.x_dot = 1.0;
            Require(e.Advance(0.01, s) == std::array<double, 5>{}, "Excitation during settling");
            s.body.x_dot = 0.0;
            e.Advance(0.03, s);
            e.Advance(0.04, s);
            Require(e.phase() == SystemIdentificationExperiment::Phase::Waiting, "Interrupted stability timer was not reset");
            e.Advance(0.1, s);
            Require(e.phase() == SystemIdentificationExperiment::Phase::Aborted, "Stability timeout did not abort");
            e.Finish("timeout_test");
        }
        // 提前停止保留完整 CSV 表头和未完成标志；姿态越界立即关闭激励。
        p.excitation_enable = true;
        p.excitation_mode = SystemIdentificationExcitation::Prbs;
        p.input_mask = 0x1F;
        p.pre_seconds = 0.0;
        p.stable_seconds = 0.002;
        {
            SystemIdentificationExperiment e(model.get(), map, root / "early_stop", FeedbackSource::Truth, p);
            e.Advance(0.0, s);
            const auto input = e.Advance(0.002, s);
            Require(input[0] == 0.03 && input[1] == 0.03 && input[3] == -0.03, "PRBS first codeword differs from MCU seeds");
            Require(e.phase() == SystemIdentificationExperiment::Phase::Exciting, "PRBS did not start");
            e.Finish("manual_early_stop");
        }
        {
            SystemIdentificationExperiment e(model.get(), map, root / "pose_abort", FeedbackSource::Truth, p);
            e.Advance(0.0, s);
            e.Advance(0.002, s);
            s.body.pitch = 0.2;
            Require(e.Advance(0.004, s) == std::array<double, 5>{}, "Excitation persisted after pose limit");
            Require(e.phase() == SystemIdentificationExperiment::Phase::Aborted, "Pose limit did not abort");
            e.Finish("pose_abort_test");
        }
        std::cout << "SYSID integration checks passed. CSV output: " << root.string() << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "SYSID test failed: " << e.what() << '\n';
        return 1;
    }
}
