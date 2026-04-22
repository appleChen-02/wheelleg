function log = read_usb_log(logDir)
%READ_USB_LOG Load USB logger CSV/MAT files into MATLAB.
%
% Usage:
%   log = read_usb_log(".")
%   log = read_usb_log("C:\\path\\to\\log")
%
% The script expects the Python logger output files in the directory:
%   - imu.csv
%   - robot_motion.csv
%   - robot_target.csv
%   - unknown_frames.csv
%   - usb_log.mat (optional, if exported)
%
% Returned data:
%   log.imu, log.robot_motion, log.robot_target, log.unknown_frames: tables
%   log.meta: folder and source information
%
% If usb_log.mat exists, this function also loads the MAT file and exposes it
% in log.mat for convenience.

    if nargin < 1 || strlength(string(logDir)) == 0
        logDir = pwd;
    end
    logDir = char(logDir);

    log = struct();
    log.meta = struct('folder', logDir);

    imuFile = fullfile(logDir, 'imu.csv');
    motionFile = fullfile(logDir, 'robot_motion.csv');
    targetFile = fullfile(logDir, 'robot_target.csv');
    unknownFile = fullfile(logDir, 'unknown_frames.csv');
    matFile = fullfile(logDir, 'usb_log.mat');

    if isfile(imuFile)
        log.imu = readtable(imuFile, 'VariableNamingRule', 'preserve');
    else
        log.imu = table();
    end

    if isfile(motionFile)
        log.robot_motion = readtable(motionFile, 'VariableNamingRule', 'preserve');
    else
        log.robot_motion = table();
    end

    if isfile(targetFile)
        log.robot_target = readtable(targetFile, 'VariableNamingRule', 'preserve');
    else
        log.robot_target = table();
    end

    if isfile(unknownFile)
        log.unknown_frames = readtable(unknownFile, 'VariableNamingRule', 'preserve');
    else
        log.unknown_frames = table();
    end

    if isfile(matFile)
        log.mat = load(matFile);
    else
        log.mat = struct();
    end

    % 自动输出目标量与状态量关于时间的图像（vx/vy/wz）
    plot_target_state_vs_time(log);
end

function plot_target_state_vs_time(log)
    if ~isfield(log, 'robot_motion') || ~isfield(log, 'robot_target')
        return;
    end
    if isempty(log.robot_motion) || isempty(log.robot_target)
        return;
    end

    requiredVars = {'host_time_s', 'vx', 'vy', 'wz'};
    if ~all(ismember(requiredVars, log.robot_motion.Properties.VariableNames))
        return;
    end
    if ~all(ismember(requiredVars, log.robot_target.Properties.VariableNames))
        return;
    end

    f = figure('Name', 'Target vs State Speed', 'Color', 'w');
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Chassis Target vs State (Time Series)');

    nexttile;
    plot(log.robot_motion.host_time_s, log.robot_motion.vx, 'b-', 'LineWidth', 1.0); hold on;
    plot(log.robot_target.host_time_s, log.robot_target.vx, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('vx (m/s)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(log.robot_motion.host_time_s, log.robot_motion.vy, 'b-', 'LineWidth', 1.0); hold on;
    plot(log.robot_target.host_time_s, log.robot_target.vy, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('vy (m/s)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(log.robot_motion.host_time_s, log.robot_motion.wz, 'b-', 'LineWidth', 1.0); hold on;
    plot(log.robot_target.host_time_s, log.robot_target.wz, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('wz (rad/s)'); xlabel('host\_time\_s'); legend('state', 'target', 'Location', 'best');
end
