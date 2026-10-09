#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include "balance_controller.h"
#include "system_identification_param.h"

// 只负责实验调度与采集；A/B 的拟合仍由 MATLAB 完成。
class SystemIdentificationExperiment {
public:
    enum class Phase { Waiting, Baseline, Exciting, Post, Complete, Aborted };
    SystemIdentificationExperiment(const mjModel* model, const RobotMap& map,
                                   std::filesystem::path outdir, FeedbackSource feedback,
                                   SystemIdentificationParameters parameters = {});
    ~SystemIdentificationExperiment();
    // 每个控制周期在 Compute 之前调用一次；返回该周期的广义输入激励。
    std::array<double, 5> Advance(double time, const RobotState& state);
    // 在新 ctrl 写入后、mj_fwdActuation 后记录 x[k],u[k],actual[k]。
    void Record(double time, const RobotState& state, const ControlCommand& command,
                const mjData* data);
    void Finish(const std::string& reason);
    const char* PhaseName() const;
    Phase phase() const { return phase_; }

private:
    bool IsStable(const RobotState& state) const;
    void OpenLogs(double time);
    void WriteMetadata(const std::string& reason);
    void Abort(double time, const std::string& reason);
    bool HasExcitation() const;
    double ExcitationDuration() const;
    const mjModel* model_;
    RobotMap map_;
    std::filesystem::path outdir_;
    FeedbackSource feedback_;
    SystemIdentificationParameters parameters_;
    Phase phase_ = Phase::Waiting;
    std::array<std::uint16_t, 5> lfsr_{};
    std::array<double, 18> stable_anchor_{};
    std::array<double, 18> operating_point_{};
    double stable_since_ = -1.0;
    double recording_start_ = -1.0;
    double excitation_start_ = -1.0;
    double excitation_end_ = -1.0;
    double last_time_ = 0.0;
    double host_epoch_ = 0.0;
    std::uint64_t sequence_ = 0;
    std::uint64_t excitation_sample_ = 0;
    std::uint64_t rows_ = 0;
    std::uint64_t next_sample_tick_ = 0;
    std::uint64_t next_reference_tick_ = 0;
    std::uint8_t last_saturation_ = 0;
    bool finished_ = false;
    std::string abort_reason_;
    std::ofstream samples_;
    std::ofstream references_;
};
