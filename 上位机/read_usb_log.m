function log = read_usb_log(logDir, playbackSpeed, timeRange)
%READ_USB_LOG Load USB logger CSV/MAT files into MATLAB.
%
% Usage:
%   log = read_usb_log(".")
%   log = read_usb_log("C:\\path\\to\\log")
%   log = read_usb_log("C:\\path\\to\\log", 2.0)  % 2x playback speed
%   log = read_usb_log("C:\\path\\to\\log", 1.0, [0 90])  % time window in seconds
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

    if nargin < 3 || isempty(timeRange)
        timeRange = [];
    else
        validate_time_range(timeRange);
    end

    log = struct();
    log.meta = struct('folder', logDir);
    axisLabelFontSize = 14;
    tickFontSize = 11;
    baseTickCount = 6;
    figurePosition = [100, 100, 1400, 980];

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

    if ~isempty(timeRange)
        timeOrigin = get_log_time_origin(log);
        log = apply_time_range_filter(log, timeRange, timeOrigin);
        log.meta.timeRange = timeRange;
        log.meta.timeOrigin = timeOrigin;
    end

    % 自动输出目标量与状态量关于时间的图像（vx/vy/wz）
    plot_target_state_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 姿态：roll / yaw / pitch 三图同窗
    plot_attitude_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 左腿：腿长 / theta / phi 三图同窗
    plot_leg_vs_time(log, 0, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 右腿：腿长 / theta / phi 三图同窗
    plot_leg_vs_time(log, 1, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 尾巴：摆角 / 摆角速度 / 机体位移x 三图同窗
    plot_tail_body_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 先按时间播放实际运动轨迹（默认 1x），播放结束后再显示静态轨迹图
    %animate_actual_trajectory(log, playbackSpeed, axisLabelFontSize, tickFontSize);

    % 根据线速度/角速度积分得到平面轨迹，并对比目标与实际
    plot_target_state_trajectory(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);
end

function plot_target_state_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
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

    f = create_plot_figure('Target vs State Speed', figurePosition, baseTickCount);
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Chassis Target vs State (Time Series)');

    ax = nexttile;
    plot(tMotion, log.robot_motion.vx, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.vx, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('vx (m/s)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.vy, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.vy, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('vy (m/s)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.wz, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.wz, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('wz (rad/s)', 'FontSize', axisLabelFontSize); xlabel('time (s, t0 = 0)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');
end

function plot_attitude_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
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

    f = create_plot_figure('Attitude (Roll/Yaw/Pitch)', figurePosition, baseTickCount);
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Robot Attitude: State vs Target');

    ax = nexttile;
    plot(tMotion, log.robot_motion.body_roll, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_roll, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('roll (rad)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.body_yaw, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_yaw, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('yaw (rad)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.body_pitch, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_pitch, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('pitch (rad)', 'FontSize', axisLabelFontSize); xlabel('time (s, t0 = 0)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');
end

function plot_leg_vs_time(log, legIdx, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
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

    f = create_plot_figure(sprintf('%s (legx/theta/phi)', legTitle), figurePosition, baseTickCount);
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, sprintf('%s: State vs Target', legTitle));

    ax = nexttile;
    plot(tMotion, log.robot_motion.(legxName), 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.(legxName), 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('legx (m)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.(thetaName), 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.(thetaName), 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('theta (rad)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.(phiName), 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.(phiName), 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('phi (rad)', 'FontSize', axisLabelFontSize); xlabel('time (s, t0 = 0)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');
end

function plot_tail_body_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
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

    f = create_plot_figure('Tail and Body X', figurePosition, baseTickCount);
    tlo = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Tail and Body Displacement: State vs Target');

    ax = nexttile;
    plot(tMotion, log.robot_motion.tail_beta, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.tail_beta, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('tail\_beta (rad)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.tail_beta_dot, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.tail_beta_dot, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('tail\_beta\_dot (rad/s)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');

    ax = nexttile;
    plot(tMotion, log.robot_motion.body_x, 'b-', 'LineWidth', 1.0); hold on;
    plot(tTarget, log.robot_target.body_x, 'r--', 'LineWidth', 1.0);
    grid on; style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true); ylabel('body\_x (m)', 'FontSize', axisLabelFontSize); xlabel('time (s, t0 = 0)', 'FontSize', axisLabelFontSize); legend('state', 'target', 'Location', 'best');
end

function plot_target_state_trajectory(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
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

    f = create_plot_figure('Target vs State Trajectory', figurePosition, baseTickCount);
    ax = axes(f);
    plot(ax, xState, yState, 'b-', 'LineWidth', 1.2); hold(ax, 'on');
    plot(ax, xTarget, yTarget, 'r--', 'LineWidth', 1.2);
    grid(ax, 'on'); axis(ax, 'equal');
    style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, false);
    xlabel(ax, 'x (m)', 'FontSize', axisLabelFontSize);
    ylabel(ax, 'y (m)', 'FontSize', axisLabelFontSize);
    title(ax, 'Chassis Target vs State Trajectory (Integrated from vx/wz)', 'FontSize', axisLabelFontSize);
    legend(ax, 'state', 'target', 'Location', 'best');
end

function animate_actual_trajectory(log, playbackSpeed, axisLabelFontSize, tickFontSize)
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
    set(ax, 'FontSize', tickFontSize);
    xlabel(ax, 'x (m)', 'FontSize', axisLabelFontSize);
    ylabel(ax, 'y (m)', 'FontSize', axisLabelFontSize);
    title(ax, sprintf('Actual Motion Playback (%.2fx)', playbackSpeed), 'FontSize', axisLabelFontSize);

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
        title(ax, sprintf('Actual Motion Playback (%.2fx), t = %.2f s', playbackSpeed, tState(k) - tState(1)), 'FontSize', axisLabelFontSize);
        drawnow;
        pause(dt / playbackSpeed);
    end
end

function f = create_plot_figure(name, figurePosition, baseTickCount)
    f = figure('Name', name, 'Color', 'w', 'Position', figurePosition);
    f.SizeChangedFcn = @(src, event) refresh_figure_ticks(src, baseTickCount);
end

function style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, isTimeXAxis)
    set(ax, 'FontSize', tickFontSize);
    if nargin < 5
        isTimeXAxis = false;
    end
    setappdata(ax, 'BaseTickCount', baseTickCount);
    setappdata(ax, 'IsTimeXAxis', isTimeXAxis);
    apply_adaptive_ticks(ax, baseTickCount, isTimeXAxis);
end

function apply_adaptive_ticks(ax, baseTickCount, isTimeXAxis)
    if nargin < 2 || isempty(baseTickCount)
        baseTickCount = 6;
    end
    if nargin < 3
        isTimeXAxis = false;
    end

    drawnow limitrate;

    fig = ancestor(ax, 'figure');
    if isempty(fig) || ~isvalid(fig)
        return;
    end

    oldFigUnits = fig.Units;
    oldAxUnits = ax.Units;
    fig.Units = 'pixels';
    ax.Units = 'pixels';
    axPos = ax.Position;
    fig.Units = oldFigUnits;
    ax.Units = oldAxUnits;

    xLimits = xlim(ax);
    yLimits = ylim(ax);

    xTickCount = adaptive_tick_count(axPos(3), baseTickCount);
    yTickCount = adaptive_tick_count(axPos(4), baseTickCount);

    if all(isfinite(xLimits)) && diff(xLimits) > 0
        [xTicks, xFmt] = build_axis_ticks(xLimits, xTickCount, isTimeXAxis);
        if ~isempty(xTicks)
            xticks(ax, xTicks);
            xtickformat(ax, xFmt);
        end
    end

    if all(isfinite(yLimits)) && diff(yLimits) > 0
        [yTicks, yFmt] = build_axis_ticks(yLimits, yTickCount, false, true);
        if ~isempty(yTicks)
            yticks(ax, yTicks);
            ytickformat(ax, yFmt);
        end
    end
end

function count = adaptive_tick_count(axisLengthPixels, baseTickCount)
    count = max(4, min(12, round(baseTickCount * axisLengthPixels / 280)));
end

function [ticks, fmt] = build_axis_ticks(limits, desiredCount, isTimeAxis, preferDensity)
    ticks = [];
    fmt = '%.0f';

    if nargin < 4
        preferDensity = false;
    end

    minValue = limits(1);
    maxValue = limits(2);
    span = maxValue - minValue;
    if ~isfinite(span) || span <= 0
        return;
    end

    if isTimeAxis
        if span >= 20
            rawStep = span / max(desiredCount, 2);
            step = max(10, 10 * ceil(rawStep / 10));
        else
            step = 1;
        end
        startValue = floor(minValue / step) * step;
        stopValue = ceil(maxValue / step) * step;
        ticks = startValue:step:stopValue;
        fmt = '%.0f';
        return;
    end

    if preferDensity
        ticks = linspace(minValue, maxValue, desiredCount);
        if numel(ticks) >= 2
            fmt = tick_label_format(diff(ticks(1:2)));
        else
            fmt = '%.0f';
        end
        return;
    end

    if span >= 1
        rawStep = span / max(desiredCount, 2);
        step = nice_integer_step(rawStep);
        startValue = floor(minValue / step) * step;
        stopValue = ceil(maxValue / step) * step;
        ticks = startValue:step:stopValue;
        fmt = '%.0f';
    else
        rawStep = span / max(desiredCount, 2);
        step = nice_decimal_step(rawStep);
        startValue = floor(minValue / step) * step;
        stopValue = ceil(maxValue / step) * step;
        ticks = startValue:step:stopValue;
        fmt = tick_label_format(step);
    end
end

function step = nice_integer_step(rawStep)
    if ~isfinite(rawStep) || rawStep <= 0
        step = 1;
        return;
    end

    magnitude = 10 ^ floor(log10(rawStep));
    normalized = rawStep / magnitude;
    if normalized <= 1
        niceNormalized = 1;
    elseif normalized <= 2
        niceNormalized = 2;
    elseif normalized <= 5
        niceNormalized = 5;
    else
        niceNormalized = 10;
    end
    step = max(1, niceNormalized * magnitude);
end

function step = nice_decimal_step(rawStep)
    if ~isfinite(rawStep) || rawStep <= 0
        step = 0.1;
        return;
    end

    magnitude = 10 ^ floor(log10(rawStep));
    normalized = rawStep / magnitude;
    if normalized <= 1
        niceNormalized = 1;
    elseif normalized <= 2
        niceNormalized = 2;
    elseif normalized <= 5
        niceNormalized = 5;
    else
        niceNormalized = 10;
    end
    step = niceNormalized * magnitude;
end

function fmt = tick_label_format(tickSpacing)
    if ~isfinite(tickSpacing) || tickSpacing <= 0
        fmt = '%.0f';
        return;
    end

    if tickSpacing >= 1
        fmt = '%.0f';
    elseif tickSpacing >= 0.1
        fmt = '%.1f';
    else
        fmt = '%.2f';
    end
end

function refresh_figure_ticks(fig, baseTickCount)
    axesList = findall(fig, 'Type', 'axes');
    for idx = 1:numel(axesList)
        thisBaseTickCount = baseTickCount;
        if isappdata(axesList(idx), 'BaseTickCount')
            thisBaseTickCount = getappdata(axesList(idx), 'BaseTickCount');
        end

        isTimeXAxis = false;
        if isappdata(axesList(idx), 'IsTimeXAxis')
            isTimeXAxis = getappdata(axesList(idx), 'IsTimeXAxis');
        end

        apply_adaptive_ticks(axesList(idx), thisBaseTickCount, isTimeXAxis);
    end
end

function validate_time_range(timeRange)
    if ~isnumeric(timeRange) || ~isvector(timeRange) || numel(timeRange) ~= 2
        error('read_usb_log:InvalidTimeRange', 'timeRange must be a numeric vector with two elements.');
    end
    if any(~isfinite(timeRange))
        error('read_usb_log:InvalidTimeRange', 'timeRange must contain finite values.');
    end
    if timeRange(2) < timeRange(1)
        error('read_usb_log:InvalidTimeRange', 'timeRange must be increasing: [start end].');
    end
end

function timeOrigin = get_log_time_origin(log)
    timeOrigin = NaN;
    candidateFields = {'robot_motion', 'robot_target', 'imu', 'unknown_frames'};
    for idx = 1:numel(candidateFields)
        fieldName = candidateFields{idx};
        if isfield(log, fieldName)
            tbl = log.(fieldName);
            if istable(tbl) && ismember('host_time_s', tbl.Properties.VariableNames)
                currentOrigin = first_finite_time(double(tbl.host_time_s(:)));
                if isfinite(currentOrigin)
                    if ~isfinite(timeOrigin)
                        timeOrigin = currentOrigin;
                    else
                        timeOrigin = min(timeOrigin, currentOrigin);
                    end
                end
            end
        end
    end

    if ~isfinite(timeOrigin)
        timeOrigin = 0;
    end
end

function log = apply_time_range_filter(log, timeRange, timeOrigin)
    filterFields = {'imu', 'robot_motion', 'robot_target', 'unknown_frames'};
    for idx = 1:numel(filterFields)
        fieldName = filterFields{idx};
        if ~isfield(log, fieldName)
            continue;
        end

        tbl = log.(fieldName);
        if ~istable(tbl) || ~ismember('host_time_s', tbl.Properties.VariableNames) || isempty(tbl)
            continue;
        end

        relativeTime = double(tbl.host_time_s(:)) - timeOrigin;
        keepMask = isfinite(relativeTime) & relativeTime >= timeRange(1) & relativeTime <= timeRange(2);
        log.(fieldName) = tbl(keepMask, :);
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
