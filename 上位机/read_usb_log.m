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
%   - torque.csv
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
    torqueFile = fullfile(logDir, 'torque.csv');
    motorAngleFile = fullfile(logDir, 'motor_angle.csv');
    motorErrorFile = fullfile(logDir, 'motor_error.csv');
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

    if isfile(torqueFile)
        log.torque = readtable(torqueFile, 'VariableNamingRule', 'preserve');
    else
        log.torque = table();
    end

    if isfile(motorAngleFile)
        log.motor_angle = readtable(motorAngleFile, 'VariableNamingRule', 'preserve');
    else
        log.motor_angle = table();
    end

    if isfile(motorErrorFile)
        log.motor_error = readtable(motorErrorFile, 'VariableNamingRule', 'preserve');
    else
        log.motor_error = table();
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

    % 控制力矩：右髋/左髋/右轮/左轮/尾巴 五图同窗
    plot_torque_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 电机角度：J0~J3髋关节 + W0~W1轮子 + Tail尾巴 七图同窗
    plot_motor_angles_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 先按时间播放实际运动轨迹（默认 1x），播放结束后再显示静态轨迹图
    %animate_actual_trajectory(log, playbackSpeed, axisLabelFontSize, tickFontSize);

    % 根据线速度/角速度积分得到平面轨迹，并对比目标与实际
    plot_target_state_trajectory(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);

    % 绘制各通道 Bode 图（闭环频率响应：目标 → 状态）
    plot_bode_channels(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition);
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

function plot_torque_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
    if ~isfield(log, 'torque') || isempty(log.torque)
        return;
    end

    requiredVars = {'host_time_s', 'T_r_to_b', 'T_l_to_b', 'T_wr_to_r', 'T_wl_to_l', 'T_t_to_b'};
    if ~all(ismember(requiredVars, log.torque.Properties.VariableNames))
        return;
    end

    tTorque = double(log.torque.host_time_s(:));
    t0 = first_finite_time(tTorque);
    if ~isfinite(t0)
        t0 = 0;
    end
    tTorque = tTorque - t0;

    f = create_plot_figure('Control Torques', figurePosition, baseTickCount);
    tlo = tiledlayout(f, 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Control Torques (K[0]..K[4])');

    torqueNames = {'T_r_to_b (right hip)', 'T_l_to_b (left hip)', 'T_wr_to_r (right wheel)', 'T_wl_to_l (left wheel)', 'T_t_to_b (tail)'};
    torqueFields = {'T_r_to_b', 'T_l_to_b', 'T_wr_to_r', 'T_wl_to_l', 'T_t_to_b'};

    for i = 1:5
        ax = nexttile;
        plot(tTorque, log.torque.(torqueFields{i}), 'b-', 'LineWidth', 1.0);
        grid on;
        style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true);
        ylabel(sprintf('%s (N*m)', torqueNames{i}), 'FontSize', axisLabelFontSize);
        if i == 5
            xlabel('time (s, t0 = 0)', 'FontSize', axisLabelFontSize);
        end
    end
end

function plot_motor_angles_vs_time(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
    if ~isfield(log, 'motor_angle') || isempty(log.motor_angle)
        return;
    end

    requiredVars = {'host_time_s', 'J0_pos', 'J1_pos', 'J2_pos', 'J3_pos', ...
                    'W0_pos', 'W1_pos', 'Tail_pos'};
    if ~all(ismember(requiredVars, log.motor_angle.Properties.VariableNames))
        return;
    end

    tAngle = double(log.motor_angle.host_time_s(:));
    t0 = first_finite_time(tAngle);
    if ~isfinite(t0)
        t0 = 0;
    end
    tAngle = tAngle - t0;

    f = create_plot_figure('Motor Angles (J0-J3 / W0-W1 / Tail)', figurePosition, baseTickCount);
    tlo = tiledlayout(f, 7, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, 'Motor Shaft Angles (raw encoder)');

    angleFields = {'J0_pos', 'J1_pos', 'J2_pos', 'J3_pos', 'W0_pos', 'W1_pos', 'Tail_pos'};
    angleLabels = {'J0 (left front hip) rad', 'J1 (left rear hip) rad', ...
                   'J2 (right front hip) rad', 'J3 (right rear hip) rad', ...
                   'W0 (left wheel) rad', 'W1 (right wheel) rad', ...
                   'Tail rad'};
    lineColors = lines(7);

    for i = 1:7
        ax = nexttile;
        plot(tAngle, log.motor_angle.(angleFields{i}), ...
             'Color', lineColors(i,:), 'LineWidth', 1.0);
        grid on;
        style_plot_axes(ax, axisLabelFontSize, tickFontSize, baseTickCount, true);
        ylabel(angleLabels{i}, 'FontSize', axisLabelFontSize);
        if i == 7
            xlabel('time (s, t0 = 0)', 'FontSize', axisLabelFontSize);
        end
    end

    % 叠加离线标记（若有 motor_error.csv）
    if isfield(log, 'motor_error') && ~isempty(log.motor_error) && ...
       ismember('host_time_s', log.motor_error.Properties.VariableNames)
        tErr = double(log.motor_error.host_time_s(:)) - t0;
        for i = 1:7
            ax = nexttile(i);
            hold(ax, 'on');
            yl = ylim(ax);
            for k = 1:length(tErr)
                xline(ax, tErr(k), 'r-', 'LineWidth', 0.3, 'Alpha', 0.3);
            end
            ylim(ax, yl);
        end
    end
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
    candidateFields = {'robot_motion', 'robot_target', 'imu', 'torque', 'motor_angle', 'motor_error', 'unknown_frames'};
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
    filterFields = {'imu', 'robot_motion', 'robot_target', 'torque', 'motor_angle', 'motor_error', 'unknown_frames'};
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

% =========================================================================
%  Bode 图绘制（闭环频率响应：target → motion）
% =========================================================================

function plot_bode_channels(log, axisLabelFontSize, tickFontSize, baseTickCount, figurePosition)
    %PLOT_BODE_CHANNELS 绘制各通道闭环 Bode 图（目标 → 状态）
    %
    % 使用 MCU 端 tick_ms (HAL_GetTick) 作为时间基准，重采样至均匀网格后，
    % 通过 tfestimate (Welch 平均周期图法) 估计各通道的闭环频率响应。
    %
    % 绘制通道分组：
    %   Group 1 - 速度跟踪  : vx, vy, wz
    %   Group 2 - 姿态跟踪  : body_roll, body_pitch, body_yaw
    %   Group 3 - 腿长与尾巴: leg0_legx, leg1_legx, tail_beta
    %
    % 不建议绘制 Bode 图的通道（内部运动学状态 / 导数冗余量 / 积分量）：
    %   tail_beta_dot, body_x,
    %   leg0_phi, leg0_phi_dot, leg0_legx_dot, leg0_theta, leg0_theta_dot,
    %   leg1_phi, leg1_phi_dot, leg1_legx_dot, leg1_theta, leg1_theta_dot

    if ~exist('tfestimate', 'file')
        warning('Bode: Signal Processing Toolbox required for tfestimate. Skipping Bode plots.');
        return;
    end

    if ~isfield(log, 'robot_motion') || ~isfield(log, 'robot_target')
        return;
    end
    if isempty(log.robot_motion) || isempty(log.robot_target)
        return;
    end

    % ---- 参数 ----
    Fs = 100;            % 重采样频率 (Hz)，匹配 MCU 10 ms 发送间隔
    Nfft = 512;          % FFT 点数，频率分辨率 ≈ 0.2 Hz，窗口 ≈ 5.1 s
    window = hann(Nfft);
    noverlap = Nfft / 2; % 50% overlap

    % ---- 获取时间戳：优先 MCU tick_ms (1 ms 精度)，备选 host_time_s ----
    motionVars = log.robot_motion.Properties.VariableNames;
    targetVars = log.robot_target.Properties.VariableNames;
    useTickMs = ismember('tick_ms', motionVars) && ismember('tick_ms', targetVars);

    if useTickMs
        t_target_raw = double(log.robot_target.tick_ms(:));
        t_motion_raw = double(log.robot_motion.tick_ms(:));
    else
        t_target_raw = double(log.robot_target.host_time_s(:));
        t_motion_raw = double(log.robot_motion.host_time_s(:));
        Fs = 50;  % host_time_s 存在 USB/OS 抖动，降低重采样率
    end

    % ---- 构建均匀时间网格 ----
    [t_uniform, t_target_s, t_motion_s] = bode_build_uniform_grid(...
        t_target_raw, t_motion_raw, Fs, useTickMs);

    if isempty(t_uniform) || length(t_uniform) < Nfft
        warning('Bode: insufficient data length for tfestimate (have %d pts, need >%d).', ...
            length(t_uniform), Nfft);
        return;
    end

    % 有效分析频段提示
    fprintf('[Bode] 时间基准: %s, Fs=%.0f Hz, Nfft=%d, 有效分析频段 ≈ 0.2–%.0f Hz\n', ...
        condstr(useTickMs, 'tick_ms (MCU)', 'host_time_s (PC)'), Fs, Nfft, Fs/3);

    % ---- 通道分组定义 ----
    groups = {...
        struct('channels', {{'vx', 'vy', 'wz'}}, ...
               'labels',   {{'v_x (m/s)', 'v_y (m/s)', '\omega_z (rad/s)'}}, ...
               'title',    'Velocity Tracking Bode (target \rightarrow state)'), ...
        struct('channels', {{'body_roll', 'body_pitch', 'body_yaw'}}, ...
               'labels',   {{'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)'}}, ...
               'title',    'Attitude Tracking Bode (target \rightarrow state)'), ...
        struct('channels', {{'leg0_legx', 'leg1_legx', 'tail_beta'}}, ...
               'labels',   {{'L_0 leg length (m)', 'L_1 leg length (m)', '\beta_{tail} (rad)'}}, ...
               'title',    'Leg & Tail Tracking Bode (target \rightarrow state)') ...
    };

    % Bode 图 3 列，需要比默认更宽的图窗
    bodePos = figurePosition;
    bodePos(3) = max(bodePos(3), 1800);

    for g = 1:length(groups)
        group = groups{g};
        bode_plot_one_group(log, group, t_target_s, t_motion_s, t_uniform, ...
            Fs, Nfft, window, noverlap, bodePos, ...
            axisLabelFontSize, tickFontSize, baseTickCount, useTickMs);
    end
end

% -------------------------------------------------------------------------
function [t_uniform, t_target_s, t_motion_s] = ...
        bode_build_uniform_grid(t_target_raw, t_motion_raw, Fs, useTickMs)
    % 从两个非均匀时间向量构建统一的均匀时间网格

    % 取交集时间窗口
    t0 = max(first_finite_time(t_target_raw), first_finite_time(t_motion_raw));
    t_end_raw = min(max(t_target_raw), max(t_motion_raw));

    if ~isfinite(t0) || ~isfinite(t_end_raw) || t_end_raw <= t0
        t_uniform = [];
        t_target_s = [];
        t_motion_s = [];
        return;
    end

    % 转为秒并相对 t0
    if useTickMs
        % tick_ms: HAL_GetTick() 单位是 ms → 转为 s
        scale = 1000;
    else
        % host_time_s: 已经是 s
        scale = 1;
    end

    t_target_s = (t_target_raw - t0) / scale;
    t_motion_s = (t_motion_raw - t0) / scale;

    t_max = min(max(t_target_s), max(t_motion_s));
    t_uniform = (0 : 1/Fs : t_max)';
end

% -------------------------------------------------------------------------
function bode_plot_one_group(log, group, t_target_s, t_motion_s, t_uniform, ...
        Fs, Nfft, window, noverlap, figurePosition, ...
        axisLabelFontSize, tickFontSize, baseTickCount, useTickMs)

    nCh = length(group.channels);

    f = figure('Name', group.title, 'Color', 'w', 'Position', figurePosition);

    timeLabel = condstr(useTickMs, 'tick_{ms}', 'host\_time_s');
    tlo = tiledlayout(f, 2, nCh, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tlo, sprintf('%s  [Fs=%.0f Hz, N_{fft}=%d, t_{src}=%s]', ...
        group.title, Fs, Nfft, timeLabel), 'FontSize', axisLabelFontSize);

    for ch = 1:nCh
        chName = group.channels{ch};
        chLabel = group.labels{ch};

        % ---- 检查列名是否存在 ----
        if ~ismember(chName, log.robot_motion.Properties.VariableNames) || ...
           ~ismember(chName, log.robot_target.Properties.VariableNames)
            ax = nexttile; text(0.5, 0.5, sprintf('%s: missing', chName), ...
                'HorizontalAlign', 'center', 'FontSize', tickFontSize); axis(ax, 'off');
            ax = nexttile; text(0.5, 0.5, sprintf('%s: missing', chName), ...
                'HorizontalAlign', 'center', 'FontSize', tickFontSize); axis(ax, 'off');
            continue;
        end

        u_raw = double(log.robot_target.(chName)(:));
        y_raw = double(log.robot_motion.(chName)(:));

        % ---- 分别插值到同一均匀网格 ----
        u_uniform = interp1(t_target_s, u_raw, t_uniform, 'linear');
        y_uniform = interp1(t_motion_s, y_raw, t_uniform, 'linear');

        % ---- 清理 NaN / Inf ----
        valid = isfinite(u_uniform) & isfinite(y_uniform);
        if sum(valid) < Nfft
            ax = nexttile; text(0.5, 0.5, sprintf('data < %d pts', Nfft), ...
                'HorizontalAlign', 'center', 'FontSize', tickFontSize); axis(ax, 'off');
            ax = nexttile; text(0.5, 0.5, sprintf('data < %d pts', Nfft), ...
                'HorizontalAlign', 'center', 'FontSize', tickFontSize); axis(ax, 'off');
            continue;
        end

        u_clean = u_uniform(valid);
        y_clean = y_uniform(valid);

        % ---- tfestimate: Welch 平均周期图法 ----
        [Txy, f] = tfestimate(u_clean, y_clean, window, noverlap, Nfft, Fs);

        mag_dB = 20 * log10(max(abs(Txy), eps));
        phase_deg = unwrap(angle(Txy)) * 180 / pi;

        % ---- 提取 -3 dB 带宽 ----
        idx_3dB = find(mag_dB <= -3, 1, 'first');
        bw_str = '';
        bw_freq = NaN;
        if ~isempty(idx_3dB) && idx_3dB > 1
            bw_freq = f(idx_3dB);
            if isfinite(bw_freq) && bw_freq > f(2) && bw_freq < Fs/3
                bw_str = sprintf('  |  -3dB @ %.1f Hz', bw_freq);
            end
        end

        % ---- 低频平均增益 (0.2–1.0 Hz) ----
        mask_low = f > 0.2 & f < 1.0;
        low_gain_str = '';
        if any(mask_low)
            avg_low = mean(mag_dB(mask_low & isfinite(mag_dB)));
            if isfinite(avg_low)
                low_gain_str = sprintf('G_0≈%.1f dB  ', avg_low);
            end
        end

        % ---- 幅频子图 (上排) ----
        ax_mag = nexttile;
        semilogx(ax_mag, f, mag_dB, 'b-', 'LineWidth', 1.2);
        hold(ax_mag, 'on');
        yline(ax_mag, 0,  'k--', 'LineWidth', 0.6);   % 0 dB 参考线
        yline(ax_mag, -3, 'r--', 'LineWidth', 0.6);   % -3 dB 带宽线

        if ~isnan(bw_freq)
            xline(ax_mag, bw_freq, 'r--', 'LineWidth', 0.8);
        end

        grid(ax_mag, 'on');
        ylabel(ax_mag, 'Magnitude (dB)', 'FontSize', axisLabelFontSize);
        title(ax_mag, sprintf('%s  [%s%s]', chLabel, low_gain_str, bw_str), ...
            'FontSize', axisLabelFontSize - 2);
        set(ax_mag, 'FontSize', tickFontSize);

        % 限制 y 范围避免受极低频噪声影响
        yl_mag = ylim(ax_mag);
        ylim(ax_mag, [max(yl_mag(1), -40), min(yl_mag(2), 10)]);

        % ---- 相频子图 (下排) ----
        ax_phase = nexttile;
        semilogx(ax_phase, f, phase_deg, 'b-', 'LineWidth', 1.2);
        hold(ax_phase, 'on');
        yline(ax_phase, 0,   'k--', 'LineWidth', 0.6);
        yline(ax_phase, -90,  'k:', 'LineWidth', 0.5);
        yline(ax_phase, -180, 'k:', 'LineWidth', 0.5);

        % 自动扩展 y 范围，至少覆盖 [-360, 90]
        yl_phase = ylim(ax_phase);
        ylim(ax_phase, [min(yl_phase(1), -360), max(yl_phase(2), 90)]);

        grid(ax_phase, 'on');
        xlabel(ax_phase, 'Frequency (Hz)', 'FontSize', axisLabelFontSize);
        ylabel(ax_phase, 'Phase (deg)', 'FontSize', axisLabelFontSize);
        set(ax_phase, 'FontSize', tickFontSize);
    end

    % 链接所有子图的 x 轴 (频率轴)
    if isgraphics(f, 'figure')
        allAxes = findobj(f, 'Type', 'Axes');
        if numel(allAxes) >= 2
            linkaxes(allAxes, 'x');
        end
    end
end

% -------------------------------------------------------------------------
function s = condstr(condition, trueStr, falseStr)
    if condition
        s = trueStr;
    else
        s = falseStr;
    end
end
