#include "system_identification.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace {
constexpr const char* kStateNames[] = {
    "body_x", "body_x_dot", "yaw", "yaw_dot", "left_theta", "left_theta_dot",
    "right_theta", "right_theta_dot", "pitch", "pitch_dot", "tail_beta", "tail_beta_dot",
    "roll", "roll_dot", "left_leg_length", "left_leg_length_dot", "right_leg_length", "right_leg_length_dot"
};
constexpr const char* kInputNames[] = {
    "right_hip_tp", "left_hip_tp", "right_wheel_t", "left_wheel_t", "tail_t", "left_leg_force", "right_leg_force"
};
constexpr const char* kMotorNames[] = {
    "J0_feedback_torque", "J1_feedback_torque", "J2_feedback_torque", "J3_feedback_torque",
    "W0_feedback_torque", "W1_feedback_torque", "Tail_feedback_torque"
};
// MCU 的 J0/J2 是 phi1，J1/J3 是 phi4；仿真的这两组 actuator 顺序相反。
constexpr int kFirmwareMotorIndices[] = {1, 0, 3, 2, 4, 5, 6};
constexpr double kFirmwareMotorDirections[] = {-1, 1, 1, -1, 1, -1, -1};

std::array<double, 18> StateValues(const RobotState& s) {
    return {s.body.x, s.body.x_dot, s.body.yaw, s.body.yaw_rate,
            s.leg_state[0].theta, s.leg_state[0].theta_dot,
            s.leg_state[1].theta, s.leg_state[1].theta_dot,
            s.body.pitch, s.body.pitch_rate, s.beta, s.dbeta, s.body.roll, s.body.roll_rate,
            s.left_leg.l0, s.left_leg.dl0, s.right_leg.l0, s.right_leg.dl0};
}

std::uint64_t Tick(double time) { return static_cast<std::uint64_t>(std::llround(time * 1000.0)); }

std::string PathText(const std::filesystem::path& path) {
    const auto utf8 = path.u8string();
    return std::string(utf8.begin(), utf8.end());
}

bool InverseVmc(const LegKinematics& leg, double t_phi1, double t_phi4, double& tp) {
    const auto& j = leg.jacobian;
    const double det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    if (!std::isfinite(det) || std::abs(det) < SYS_ID_JACOBIAN_DET_MIN) {
        tp = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    // [t_phi1,t_phi4]' = J' [F,-Tp]'.
    tp = (j[0][1] * t_phi1 - j[0][0] * t_phi4) / det;
    return std::isfinite(tp);
}
}  // namespace

SystemIdentificationExperiment::SystemIdentificationExperiment(
    const mjModel* model, const RobotMap& map, std::filesystem::path outdir,
    FeedbackSource feedback, SystemIdentificationParameters parameters)
    : model_(model), map_(map), outdir_(std::filesystem::absolute(outdir)),
      feedback_(feedback), parameters_(parameters), lfsr_(parameters.prbs_seeds) {
    const auto& p = parameters_;
    for (double v : p.reference) if (!std::isfinite(v)) throw std::invalid_argument("SYS_ID_REFERENCE must be finite");
    for (int i : {1, 3, 5, 7, 9, 11, 13, 15, 17}) {
        if (p.reference[i] != 0.0) throw std::invalid_argument("SYS_ID requires a static reference (zero reference velocities)");
    }
    for (int i : {14, 16}) {
        if (p.reference[i] < kMinLegLength || p.reference[i] > kMaxLegLength)
            throw std::invalid_argument("SYS_ID reference leg length is outside limits");
    }
    for (double v : p.amplitude) if (!std::isfinite(v) || v < 0.0) throw std::invalid_argument("SYS_ID amplitudes must be finite and nonnegative");
    for (double v : {p.sweep_duration, p.prbs_duration, p.baseline_duration, p.stable_seconds, p.settle_timeout_seconds}) {
        if (!std::isfinite(v) || v <= 0.0) throw std::invalid_argument("SYS_ID durations must be finite and positive");
    }
    for (double v : {p.pre_seconds, p.post_seconds, p.min_settle_seconds, p.auto_exit_delay_seconds}) {
        if (!std::isfinite(v) || v < 0.0) throw std::invalid_argument("SYS_ID wait times must be finite and nonnegative");
    }
    if (!std::isfinite(p.sweep_f0) || !std::isfinite(p.sweep_f1) || p.sweep_f0 <= 0.0 ||
        p.sweep_f1 <= p.sweep_f0 || p.sweep_f1 >= 125.0 || p.prbs_hold_samples == 0 ||
        (p.input_mask & ~0x1F) != 0 || static_cast<int>(p.excitation_mode) < 0 || static_cast<int>(p.excitation_mode) > 2)
        throw std::invalid_argument("Invalid SYS_ID frequency (require 0 < f0 < f1 < 125 Hz), PRBS hold, mode or input mask");
    if (HasExcitation()) {
        bool active = false;
        for (int i = 0; i < 5; ++i) {
            active |= (p.input_mask & (1U << i)) && p.amplitude[i] > 0.0;
            if (p.prbs_seeds[i] == 0) throw std::invalid_argument("SYS_ID PRBS seeds must be nonzero");
        }
        if (!active) throw std::invalid_argument("SYS_ID enabled but no nonzero excitation channel selected");
    }
    // 同步读取本拍 actuator_force 依赖当前模型使用 gear=1 的无动态纯力矩电机。
    for (const auto& a : map_.actuators) {
        if (model_->actuator_trntype[a.id] != mjTRN_JOINT || model_->actuator_gear[6 * a.id] != 1.0 ||
            model_->actuator_dyntype[a.id] != mjDYN_NONE || model_->actuator_gaintype[a.id] != mjGAIN_FIXED ||
            model_->actuator_gainprm[mjNGAIN * a.id] != 1.0 || model_->actuator_biastype[a.id] != mjBIAS_NONE)
            throw std::invalid_argument("SYS_ID synchronous torque logging requires ideal gear=1 torque actuators");
    }
    for (const char* name : {"system_identification_12.csv", "system_identification_reference.csv", "simulation_experiment.txt"}) {
        if (std::filesystem::exists(outdir_ / name)) throw std::runtime_error("SYS_ID output already exists: " + PathText(outdir_ / name));
    }
    std::filesystem::create_directories(outdir_);
    host_epoch_ = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout << "[SYSID] WAITING_FOR_STABILITY: fixed references, control=2 ms, CSV=4 ms.\n"
              << "[SYSID] Output: " << PathText(outdir_) << "\n" << std::flush;
    PrintProgress(0.0, true);
}

SystemIdentificationExperiment::~SystemIdentificationExperiment() {
    try { Finish("scope_exit"); }
    catch (const std::exception& e) { std::cerr << "[SYSID] Save error: " << e.what() << "\n"; }
}

bool SystemIdentificationExperiment::HasExcitation() const {
    return parameters_.excitation_enable && parameters_.excitation_mode != SystemIdentificationExcitation::Disabled;
}

double SystemIdentificationExperiment::ExcitationDuration() const {
    if (!HasExcitation()) return parameters_.baseline_duration;
    return parameters_.excitation_mode == SystemIdentificationExcitation::SineSweep
        ? parameters_.sweep_duration : parameters_.prbs_duration;
}

bool SystemIdentificationExperiment::ShouldAutoExit(double time) const {
    return phase_ == Phase::Complete &&
           time - capture_complete_time_ + 1e-9 >= parameters_.auto_exit_delay_seconds;
}

void SystemIdentificationExperiment::PrintProgress(double time, bool force) {
    if (!force && (phase_ == Phase::Aborted || time + 1e-9 < next_progress_time_)) return;
    next_progress_time_ = time + SYS_ID_PROGRESS_PERIOD_SECONDS;
    const double total = parameters_.pre_seconds + ExcitationDuration() + parameters_.post_seconds;
    const double recorded = recording_start_ >= 0.0 ? std::max(0.0, time - recording_start_) : 0.0;
    const double elapsed = std::min(recorded, total);
    const char* phase = phase_ == Phase::Exciting && !HasExcitation() ? "baseline_capture" : PhaseName();
    // 使用独立流格式化，不改变其他控制状态打印的精度。
    std::ostringstream line;
    line.imbue(std::locale::classic());
    line << std::fixed << std::setprecision(1)
         << "[SYSID] PROGRESS: collected=" << elapsed << '/' << total << " s ("
         << (total > 0.0 ? 100.0 * elapsed / total : 100.0) << "%); phase=" << phase;
    if (phase_ == Phase::Waiting) {
        const double stable = stable_since_ >= 0.0 ? std::max(0.0, time - stable_since_) : 0.0;
        line << "; waited=" << time << '/' << parameters_.settle_timeout_seconds
             << " s; continuous_stable=" << stable << '/' << parameters_.stable_seconds << " s";
    } else if (phase_ == Phase::Complete) {
        line << "; extra_recorded=" << std::max(0.0, recorded - total)
             << " s; auto_exit_in=" << std::max(0.0, parameters_.auto_exit_delay_seconds - (time - capture_complete_time_)) << " s";
    } else if (phase_ == Phase::Aborted) {
        line << "; capture aborted; stop manually to save";
    } else {
        line << "; remaining=" << std::max(0.0, total - recorded) << " s";
    }
    std::cout << line.str() << '\n' << std::flush;
}

bool SystemIdentificationExperiment::IsStable(const RobotState& s) const {
    const auto x = StateValues(s);
    if (!std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); }) || last_saturation_ != 0) return false;
    if (std::abs(x[1]) > SYS_ID_STABLE_SPEED || std::abs(x[12] - parameters_.reference[12]) > SYS_ID_ABORT_ROLL ||
        std::abs(x[8] - parameters_.reference[8]) > 0.15) return false;
    for (int i : {3, 5, 7, 9, 11, 13}) if (std::abs(x[i]) > SYS_ID_STABLE_ANGULAR_RATE) return false;
    for (int i : {14, 16}) {
        if (std::abs(x[i] - parameters_.reference[i]) > SYS_ID_STABLE_LEG_ERROR ||
            std::abs(x[i + 1]) > SYS_ID_STABLE_LEG_SPEED) return false;
    }
    for (const auto* leg : {&s.left_leg, &s.right_leg}) {
        double unused = 0.0;
        if (!InverseVmc(*leg, 0.0, 0.0, unused)) return false;
    }
    if (stable_since_ >= 0.0) {
        if (std::abs(x[0] - stable_anchor_[0]) > SYS_ID_STABLE_POSITION_CHANGE) return false;
        for (int i : {2, 4, 6, 8, 10, 12}) {
            if (std::abs(std::remainder(x[i] - stable_anchor_[i], 2.0 * std::acos(-1.0))) > SYS_ID_STABLE_ANGLE_CHANGE) return false;
        }
    }
    return true;
}

void SystemIdentificationExperiment::Abort(double time, const std::string& reason) {
    phase_ = Phase::Aborted;
    abort_reason_ = reason;
    std::cout << "[SYSID] ABORTED at t=" << time << ": " << reason
              << ". Excitation OFF; LQR remains ON. Stop manually to save.\n" << std::flush;
}

std::array<double, 5> SystemIdentificationExperiment::Advance(double time, const RobotState& state) {
    const Phase previous_phase = phase_;
    ++sequence_;
    last_time_ = time;
    std::array<double, 5> excitation{};
    if (phase_ == Phase::Waiting) {
        if (time >= parameters_.settle_timeout_seconds) {
            Abort(time, "stability timeout");
        } else if (time >= parameters_.min_settle_seconds && IsStable(state)) {
            if (stable_since_ < 0.0) { stable_since_ = time; stable_anchor_ = StateValues(state); }
            if (time - stable_since_ + 1e-9 >= parameters_.stable_seconds) {
                OpenLogs(time);
                recording_start_ = time;
                phase_ = Phase::Baseline;
                std::cout << "[SYSID] STABLE: recording pre-excitation baseline for " << parameters_.pre_seconds << " s.\n" << std::flush;
            }
        } else {
            stable_since_ = -1.0;
        }
    }
    if (phase_ == Phase::Baseline) {
        if (!IsStable(state)) Abort(time, "pre-excitation baseline lost stability");
        else if (time - recording_start_ + 1e-9 >= parameters_.pre_seconds) {
            operating_point_ = StateValues(state);
            excitation_start_ = time;
            excitation_end_ = time + ExcitationDuration();
            phase_ = Phase::Exciting;
            std::cout << "[SYSID] " << (HasExcitation() ? "EXCITATION_STARTED" : "BASELINE_CAPTURE_STARTED")
                      << " at t=" << time << "; duration=" << ExcitationDuration() << " s.\n" << std::flush;
        }
    }
    if (phase_ == Phase::Exciting) {
        const auto x = StateValues(state);
        if (!std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); }) ||
            std::abs(x[8] - operating_point_[8]) > SYS_ID_ABORT_PITCH_CHANGE ||
            std::abs(x[12] - parameters_.reference[12]) > SYS_ID_ABORT_ROLL ||
            std::abs(x[4] - operating_point_[4]) > SYS_ID_ABORT_THETA_CHANGE ||
            std::abs(x[6] - operating_point_[6]) > SYS_ID_ABORT_THETA_CHANGE) {
            Abort(time, "state left the experiment operating region");
        } else if (time + 1e-9 >= excitation_end_) {
            phase_ = Phase::Post;
            std::cout << "[SYSID] EXCITATION_FINISHED: excitation OFF; recording at least " << parameters_.post_seconds
                      << " s of post-excitation response.\n" << std::flush;
        } else if (HasExcitation()) {
            const double tau = static_cast<double>(excitation_sample_) * kControlPeriod;
            for (int i = 0; i < 5; ++i) {
                if (!(parameters_.input_mask & (1U << i))) continue;
                if (parameters_.excitation_mode == SystemIdentificationExcitation::SineSweep) {
                    // f(tau) = f0 * exp(log(f1/f0) * tau/T); phase is its time integral.
                    // expm1 avoids cancellation when tau is small or f1 is close to f0.
                    const double log_ratio = std::log1p((parameters_.sweep_f1 - parameters_.sweep_f0) /
                                                       parameters_.sweep_f0);
                    const double phase = 2.0 * std::acos(-1.0) * parameters_.sweep_f0 *
                        parameters_.sweep_duration / log_ratio *
                        std::expm1(log_ratio * (tau / parameters_.sweep_duration));
                    excitation[i] = parameters_.amplitude[i] * std::sin(phase);
                } else {
                    excitation[i] = (lfsr_[i] & 1U) ? parameters_.amplitude[i] : -parameters_.amplitude[i];
                    if ((excitation_sample_ + 1) % parameters_.prbs_hold_samples == 0) {
                        const auto lsb = lfsr_[i] & 1U;
                        lfsr_[i] >>= 1U;
                        if (lsb) lfsr_[i] ^= 0xB400U;
                    }
                }
            }
            ++excitation_sample_;
        }
    }
    if (phase_ == Phase::Post && time - excitation_end_ + 1e-9 >= parameters_.post_seconds) {
        phase_ = Phase::Complete;
        capture_complete_time_ = time;
        std::cout << "[SYSID] IDENTIFICATION_CAPTURE_FINISHED: excitation and post-response capture complete.\n"
                  << "[SYSID] LQR and recording remain ON; automatically saving CSV and exiting in "
                  << parameters_.auto_exit_delay_seconds << " s of simulation time.\n" << std::flush;
    }
    PrintProgress(time, phase_ != previous_phase);
    return excitation;
}

void SystemIdentificationExperiment::OpenLogs(double time) {
    samples_.exceptions(std::ios::badbit | std::ios::failbit);
    references_.exceptions(std::ios::badbit | std::ios::failbit);
    samples_.open(outdir_ / "system_identification_12.csv");
    references_.open(outdir_ / "system_identification_reference.csv");
    samples_.imbue(std::locale::classic());
    references_.imbue(std::locale::classic());
    samples_ << std::setprecision(17);
    references_ << std::setprecision(17);
    samples_ << "host_time_s,control_tick_ms,sequence,experiment_id,reference_sequence,protocol_version,profile_id,state_dimension,input_dimension,excitation_config,excitation_input_mask,status_flags,motor_saturation_mask";
    for (int i = 0; i < 12; ++i) samples_ << ',' << kStateNames[i];
    for (const char* prefix : {"controller_", "excitation_", "command_", "actual_"}) {
        for (int i = 0; i < 5; ++i) samples_ << ',' << prefix << kInputNames[i];
    }
    for (const char* name : kMotorNames) samples_ << ',' << name;
    samples_ << '\n';
    references_ << "host_time_s,experiment_id,reference_sequence,protocol_version,profile_id,state_dimension,input_dimension,excitation_config,excitation_input_mask";
    for (const char* name : kStateNames) references_ << ",reference_" << name;
    for (const char* name : kInputNames) references_ << ",equilibrium_" << name;
    references_ << '\n';
    next_sample_tick_ = ((Tick(time) + SYS_ID_SAMPLE_SEND_PERIOD_MS - 1) / SYS_ID_SAMPLE_SEND_PERIOD_MS) * SYS_ID_SAMPLE_SEND_PERIOD_MS;
    next_reference_tick_ = next_sample_tick_;
}

void SystemIdentificationExperiment::Record(double time, const RobotState& state,
                                             const ControlCommand& command, const mjData* data) {
    last_saturation_ = command.motor_saturation_mask;
    if (!samples_.is_open() || finished_ || Tick(time) < next_sample_tick_) return;
    const auto tick = Tick(time);
    next_sample_tick_ = tick + SYS_ID_SAMPLE_SEND_PERIOD_MS;
    const auto x = StateValues(state);
    std::array<double, 7> torques{};
    for (int i = 0; i < 7; ++i) {
        const auto& a = map_.actuators[i];
        torques[i] = data->actuator_force[a.id] * a.direction;
    }
    std::array<double, 5> actual = {0.0, 0.0, torques[5], torques[4], torques[6]};
    unsigned status = 0x01 | 0x80;  // MODE_VALID | REFERENCE_VALID
    const bool left_ok = InverseVmc(state.left_leg, torques[1], torques[0], actual[1]);
    const bool right_ok = InverseVmc(state.right_leg, torques[3], torques[2], actual[0]);
    if (!left_ok) status |= 0x10;
    if (!right_ok) status |= 0x20;
    if (left_ok && right_ok && std::all_of(actual.begin(), actual.end(), [](double v) { return std::isfinite(v); })) status |= 0x40;
    if (phase_ == Phase::Exciting && HasExcitation()) status |= 0x02;
    if ((phase_ == Phase::Post || phase_ == Phase::Complete) && HasExcitation() &&
        parameters_.excitation_mode == SystemIdentificationExcitation::SineSweep) status |= 0x04;
    const unsigned config = (parameters_.excitation_enable ? 1U : 0U) | (static_cast<unsigned>(parameters_.excitation_mode) << 1U);
    // 加速无窗口运行时使用逻辑主机时间，物理时间仍以 control_tick_ms 为准。
    const double host_time = host_epoch_ + time;
    samples_ << host_time << ',' << tick << ',' << sequence_ << ",1,1,1,1,12,5," << config << ','
             << unsigned(parameters_.input_mask) << ',' << status << ',' << unsigned(command.motor_saturation_mask);
    for (int i = 0; i < 12; ++i) samples_ << ',' << x[i];
    const std::array<const std::array<double, 5>*, 4> input_groups = {
        &command.controller_output, &command.excitation, &command.generalized_command, &actual
    };
    for (const auto* values : input_groups) {
        for (double v : *values) samples_ << ',' << v;
    }
    for (int i = 0; i < 7; ++i) samples_ << ',' << torques[kFirmwareMotorIndices[i]] * kFirmwareMotorDirections[i];
    samples_ << '\n';
    ++rows_;
    if (tick >= next_reference_tick_) {
        next_reference_tick_ = tick + SYS_ID_REFERENCE_SEND_PERIOD_MS;
        references_ << host_time << ",1,1,1,1,12,5," << config << ',' << unsigned(parameters_.input_mask);
        for (double v : command.reference_state) references_ << ',' << v;
        for (double v : command.equilibrium_input) references_ << ',' << v;
        references_ << '\n';
        samples_.flush();
        references_.flush();
    }
}

const char* SystemIdentificationExperiment::PhaseName() const {
    switch (phase_) {
        case Phase::Waiting: return "waiting";
        case Phase::Baseline: return "pre_baseline";
        case Phase::Exciting: return "exciting";
        case Phase::Post: return "post_response";
        case Phase::Complete: return "complete";
        case Phase::Aborted: return "aborted";
    }
    return "unknown";
}

void SystemIdentificationExperiment::WriteMetadata(const std::string& reason) {
    std::ofstream out;
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out.open(outdir_ / "simulation_experiment.txt");
    out.imbue(std::locale::classic());
    out << std::setprecision(17)
        << "source=MuJoCo simulation; this is data capture, not A/B estimation\n"
        << "feedback=" << (feedback_ == FeedbackSource::Truth ? "truth" : "mcu")
        << "\nphase=" << PhaseName() << "\nstop_reason=" << reason << "\nabort_reason=" << abort_reason_
        << "\nrows=" << rows_ << "\nlast_sim_time_s=" << last_time_
        << "\nrecording_start_s=" << recording_start_ << "\nexcitation_start_s=" << excitation_start_
        << "\nplanned_excitation_end_s=" << excitation_end_
        << "\ncapture_complete_s=" << capture_complete_time_
        << "\nauto_exit_delay_s=" << parameters_.auto_exit_delay_seconds
        << "\ncontrol_period_s=" << kControlPeriod << "\nsample_period_s=0.004\nreference_period_s=1\n"
        << "host_time_s=logical epoch plus simulation time, not wall-clock arrival time\n"
        << "timing=x[k] before stepping; u[k] newly computed; actual[k] from mj_fwdActuation at x[k],u[k]\n"
        << "motor_columns=MCU raw feedback frame; J0/J1/J2/J3 map to simulation actuator indices 1/0/3/2\n"
        << "excitation_enable=" << parameters_.excitation_enable << "\nexcitation_mode=" << static_cast<int>(parameters_.excitation_mode)
        << "\ninput_mask=" << unsigned(parameters_.input_mask)
        << "\nsweep_law=logarithmic"
        << "\nsweep_f0_hz=" << parameters_.sweep_f0 << "\nsweep_f1_hz=" << parameters_.sweep_f1
        << "\nsweep_duration_s=" << parameters_.sweep_duration << "\nprbs_duration_s=" << parameters_.prbs_duration
        << "\nbaseline_duration_s=" << parameters_.baseline_duration
        << "\nprbs_hold_samples=" << parameters_.prbs_hold_samples
        << "\npre_seconds=" << parameters_.pre_seconds << "\npost_seconds=" << parameters_.post_seconds
        << "\nstable_seconds=" << parameters_.stable_seconds
        << "\nmin_settle_seconds=" << parameters_.min_settle_seconds
        << "\nsettle_timeout_seconds=" << parameters_.settle_timeout_seconds << '\n';
    for (int i = 0; i < 5; ++i) out << "amplitude_" << kInputNames[i] << '=' << parameters_.amplitude[i]
        << "\nseed_" << kInputNames[i] << '=' << parameters_.prbs_seeds[i] << '\n';
    for (int i = 0; i < 18; ++i) out << "configured_reference_" << kStateNames[i] << '=' << parameters_.reference[i] << '\n';
}

void SystemIdentificationExperiment::Finish(const std::string& reason) {
    if (finished_) return;
    if (samples_.is_open()) { samples_.flush(); samples_.close(); }
    if (references_.is_open()) { references_.flush(); references_.close(); }
    WriteMetadata(reason);
    finished_ = true;
    std::cout << "[SYSID] DATA_SAVED: " << rows_ << " rows; phase=" << PhaseName()
              << "; stop=" << reason << "; directory=" << PathText(outdir_) << "\n" << std::flush;
    if (phase_ != Phase::Complete) std::cout << "[SYSID] Capture is incomplete; see simulation_experiment.txt.\n" << std::flush;
}
