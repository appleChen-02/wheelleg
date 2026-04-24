function log = read_usb_log(logDir, playbackSpeed)
%READ_USB_LOG Load USB logger CSV/MAT files into MATLAB.
%
% Usage:
%   log = read_usb_log(".")
%   log = read_usb_log("C:\\path\\to\\log")
%   log = read_usb_log("C:\\path\\to\\log", 2.0)  % 2x playback speed
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
%   robot_motion/robot_target now include full snapshot columns from payload <I21f>
%
% If usb_log.mat exists, this function also loads the MAT file and exposes it
% in log.mat for convenience.

    if nargin < 1 || strlength(string(logDir)) == 0
        logDir = pwd;
    end
    logDir = char(logDir);

    if nargin < 2 || ~isfinite(playbackSpeed) || playbackSpeed <= 0
        playbackSpeed = 1.0;
    end

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

    % 姿态：roll / yaw / pitch 三图同窗
    plot_attitude_vs_time(log);

    % 左腿：腿长 / theta / phi 三图同窗
    plot_leg_vs_time(log, 0);

    % 右腿：腿长 / theta / phi 三图同窗
    plot_leg_vs_time(log, 1);

    % 尾巴：摆角 / 摆角速度 / 机体位移x 三图同窗
    plot_tail_body_vs_time(log);

    % 先按时间播放实际运动轨迹（默认 1x），播放结束后再显示静态轨迹图
    animate_actual_trajectory(log, playbackSpeed);

    % 根据线速度/角速度积分得到平面轨迹，并对比目标与实际
    plot_target_state_trajectory(log);
end

function plot_target_state_vs_time(log)
    if ~isfield(log, 'robot_motion') || ~isfield(log, 'robot_target')
        return;
    end
    if isempty(log.robot_motion) || isempty(log.robot_target)
        return;
    end

    requiredVars = {'host_time_s', 'vx', 'wz'};
    if ~all(ismember(requiredVars, log.robot_motion.Properties.VariableNames))
        return;
    end
    if ~all(ismember(requiredVars, log.robot_target.Properties.VariableNames))
        return;
    end

    [tMotion, tTarget] = build_common_relative_time(log.robot_motion.host_time_s, log.robot_target.host_time_s);

    f = figure('Name', 'Target vs State Speed', 'Color', 'w');
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Chassis Target vs State (Time Series)');

    nexttile;
    plot(tMotion, log.robot_motion.vx, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.vx, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('vx (m/s)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.vy, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.vy, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('vy (m/s)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.wz, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.wz, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('wz (rad/s)'); xlabel('time (s, t0 = 0)'); legend('state', 'target', 'Location', 'best');
end

function plot_attitude_vs_time(log)
    if ~isfield(log, 'robot_motion') || ~isfield(log, 'robot_target')
        return;
    end
    if isempty(log.robot_motion) || isempty(log.robot_target)
        return;
    end

    requiredMotion = {'host_time_s', 'body_roll', 'body_yaw', 'body_pitch'};
    requiredTarget = {'host_time_s', 'body_roll', 'body_yaw', 'body_pitch'};

    if ~all(ismember(requiredMotion, log.robot_motion.Properties.VariableNames))
        return;
    end
    if ~all(ismember(requiredTarget, log.robot_target.Properties.VariableNames))
        return;
    end

    [tMotion, tTarget] = build_common_relative_time(log.robot_motion.host_time_s, log.robot_target.host_time_s);

    f = figure('Name', 'Attitude (Roll/Yaw/Pitch)', 'Color', 'w');
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Robot Attitude: State vs Target');

    nexttile;
    plot(tMotion, log.robot_motion.body_roll, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_roll, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('roll (rad)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.body_yaw, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_yaw, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('yaw (rad)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.body_pitch, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_pitch, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('pitch (rad)'); xlabel('time (s, t0 = 0)'); legend('state', 'target', 'Location', 'best');
end

function plot_leg_vs_time(log, legIdx)
    if ~isfield(log, 'robot_motion') || ~isfield(log, 'robot_target')
        return;
    end
    if isempty(log.robot_motion) || isempty(log.robot_target)
        return;
    end

    legxName = sprintf('leg%d_legx', legIdx);
    thetaName = sprintf('leg%d_theta', legIdx);
    phiName = sprintf('leg%d_phi', legIdx);

    requiredMotion = {'host_time_s', legxName, thetaName, phiName};
    requiredTarget = {'host_time_s', legxName, thetaName, phiName};

    if ~all(ismember(requiredMotion, log.robot_motion.Properties.VariableNames))
        return;
    end
    if ~all(ismember(requiredTarget, log.robot_target.Properties.VariableNames))
        return;
    end

    [tMotion, tTarget] = build_common_relative_time(log.robot_motion.host_time_s, log.robot_target.host_time_s);

    legTitle = 'Left Leg';
    if legIdx == 1
        legTitle = 'Right Leg';
    end

    f = figure('Name', sprintf('%s (legx/theta/phi)', legTitle), 'Color', 'w');
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, sprintf('%s: State vs Target', legTitle));

    nexttile;
    plot(tMotion, log.robot_motion.(legxName), 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.(legxName), 'r--', 'LineWidth', 1.0);
    grid on; ylabel('legx (m)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.(thetaName), 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.(thetaName), 'r--', 'LineWidth', 1.0);
    grid on; ylabel('theta (rad)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.(phiName), 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.(phiName), 'r--', 'LineWidth', 1.0);
    grid on; ylabel('phi (rad)'); xlabel('time (s, t0 = 0)'); legend('state', 'target', 'Location', 'best');
end

function plot_tail_body_vs_time(log)
    if ~isfield(log, 'robot_motion') || ~isfield(log, 'robot_target')
        return;
    end
    if isempty(log.robot_motion) || isempty(log.robot_target)
        return;
    end

    requiredMotion = {'host_time_s', 'tail_beta', 'tail_beta_dot', 'body_x'};
    requiredTarget = {'host_time_s', 'tail_beta', 'tail_beta_dot', 'body_x'};

    if ~all(ismember(requiredMotion, log.robot_motion.Properties.VariableNames))
        return;
    end
    if ~all(ismember(requiredTarget, log.robot_target.Properties.VariableNames))
        return;
    end

    [tMotion, tTarget] = build_common_relative_time(log.robot_motion.host_time_s, log.robot_target.host_time_s);

    f = figure('Name', 'Tail and Body X', 'Color', 'w');
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Tail and Body Displacement: State vs Target');

    nexttile;
    plot(tMotion, log.robot_motion.tail_beta, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.tail_beta, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('tail\_beta (rad)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.tail_beta_dot, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.tail_beta_dot, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('tail\_beta\_dot (rad/s)'); legend('state', 'target', 'Location', 'best');

    nexttile;
    plot(tMotion, log.robot_motion.body_x, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_x, 'r--', 'LineWidth', 1.0);
    grid on; ylabel('body\_x (m)'); xlabel('time (s, t0 = 0)'); legend('state', 'target', 'Location', 'best');
end

function plot_target_state_trajectory(log)
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

    [xState, yState] = integrate_body_velocity_to_xy(log.robot_motion);
    [xTarget, yTarget] = integrate_body_velocity_to_xy(log.robot_target);

    f = figure('Name', 'Target vs State Trajectory', 'Color', 'w');
    ax = axes(f);
    plot(ax, xState, yState, 'b-', 'LineWidth', 1.2); hold(ax, 'on');
    plot(ax, xTarget, yTarget, 'r--', 'LineWidth', 1.2);
    grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'x (m)');
    ylabel(ax, 'y (m)');
    title(ax, 'Chassis Target vs State Trajectory (Integrated from vx/wz)');
    legend(ax, 'state', 'target', 'Location', 'best');
end

function animate_actual_trajectory(log, playbackSpeed)
    if ~isfield(log, 'robot_motion') || isempty(log.robot_motion)
        return;
    end

    requiredVars = {'host_time_s', 'vx', 'wz'};
    if ~all(ismember(requiredVars, log.robot_motion.Properties.VariableNames))
        return;
    end

    [xState, yState, tState] = integrate_body_velocity_to_xy(log.robot_motion);
    if isempty(tState) || numel(tState) < 2
        return;
    end

    f = figure('Name', 'Actual Motion Trajectory Playback', 'Color', 'w');
    ax = axes(f);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'x (m)');
    ylabel(ax, 'y (m)');
    title(ax, sprintf('Actual Motion Playback (%.2fx)', playbackSpeed));

    xMin = min(xState);
    xMax = max(xState);
    yMin = min(yState);
    yMax = max(yState);
    span = max([xMax - xMin, yMax - yMin, 1e-3]);
    margin = 0.1 * span;
    xlim(ax, [xMin - margin, xMax + margin]);
    ylim(ax, [yMin - margin, yMax + margin]);

    trajLine = plot(ax, xState(1), yState(1), 'b-', 'LineWidth', 1.5);
    carDot = plot(ax, xState(1), yState(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

    for k = 2:numel(tState)
        dt = tState(k) - tState(k - 1);
        if ~isfinite(dt) || dt < 0
            dt = 0;
        end

        set(trajLine, 'XData', xState(1:k), 'YData', yState(1:k));
        set(carDot, 'XData', xState(k), 'YData', yState(k));
        title(ax, sprintf('Actual Motion Playback (%.2fx), t = %.2f s', playbackSpeed, tState(k) - tState(1)));
        drawnow;
        pause(dt / playbackSpeed);
    end
end

function [x, y, t] = integrate_body_velocity_to_xy(tbl)
    t = double(tbl.host_time_s(:));
    vx = double(tbl.vx(:));
    wz = double(tbl.wz(:));

    n = min([numel(t), numel(vx), numel(wz)]);
    t = t(1:n);
    vx = vx(1:n);
    wz = wz(1:n);

    if n == 0
        x = [];
        y = [];
        t = [];
        return;
    end

    t(~isfinite(t)) = 0;
    vx(~isfinite(vx)) = 0;
    wz(~isfinite(wz)) = 0;

    dt = [0; diff(t)];
    dt(dt < 0) = 0;

    yaw = cumsum(wz .* dt);

    % 基于地面速度模型：仅使用前向线速度 vx 和角速度 wz
    vwx = vx .* cos(yaw);
    vwy = vx .* sin(yaw);

    x = cumsum(vwx .* dt);
    y = cumsum(vwy .* dt);
end

function [tA, tB] = build_common_relative_time(timeA, timeB)
    tA = double(timeA(:));
    tB = double(timeB(:));

    t0A = first_finite_time(tA);
    t0B = first_finite_time(tB);

    if isfinite(t0A) && isfinite(t0B)
        t0 = min(t0A, t0B);
    elseif isfinite(t0A)
        t0 = t0A;
    elseif isfinite(t0B)
        t0 = t0B;
    else
        t0 = 0;
    end

    tA = tA - t0;
    tB = tB - t0;
end

function t0 = first_finite_time(t)
    idx = find(isfinite(t), 1, 'first');
    if isempty(idx)
        t0 = NaN;
    else
        t0 = t(idx);
    end
end
