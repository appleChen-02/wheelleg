function theoretical_data = plot_theoretical_bode(A_num, B_num, K, Fs, figurePosition)
%PLOT_THEORETICAL_BODE 绘制 LQR 闭环理论 Bode 图
%
% 理论闭环模型：
%   控制律: u = -K (x - r)
%   闭环:   dx/dt = (A - BK) x + (BK) r
%   对通道 i: G_i(s) = c_i * (sI - Acl)^(-1) * b_i
%     其中 Acl = A - BK, b_i = (BK) 第i列, c_i = 第i位=1的行向量
%
% Usage:    
%   % 方式1：自动查找最新的 lqr_linear_model_*.mat
%   theoretical_data = plot_theoretical_bode();
%
%   % 方式2：指定 .mat 文件路径
%   theoretical_data = plot_theoretical_bode('lqr_linear_model_20260714_1.mat');
%
%   % 方式3：先运行 compute_lqr_0706.m，从 workspace 读取（无 .mat 文件时）
%   theoretical_data = plot_theoretical_bode('workspace');
%
%   % 方式4：手动传入矩阵
%   theoretical_data = plot_theoretical_bode(A_num, B_num, K);
%
%   % 方式5：传入全部参数
%   theoretical_data = plot_theoretical_bode(A_num, B_num, K, 100, [100 100 1800 900]);
%
% 返回值:
%   theoretical_data  — struct，包含各通道的频率响应数据
%     .f_Hz        — 频率向量 (Hz)
%     .channels{i} — struct: .name, .label, .mag_dB, .phase_deg, .sys (ss object)
%     .sourceFile  — 数据来源文件（如有）
%
% 依赖:
%   Control System Toolbox (ss, freqresp, eig)

    %% ---- 参数解析：支持多种输入方式 ----
    sourceFile = '';
    leg_length_val = NaN;
    lqr_Q = [];
    lqr_R = [];
    if nargin == 0
        % 无参数：自动查找最新的 lqr_linear_model_*.mat
        files = dir('lqr_linear_model_*.mat');
        if ~isempty(files)
            [~, idx] = max([files.datenum]);
            sourceFile = files(idx).name;
            fprintf('[plot_theoretical_bode] 自动加载: %s\n', sourceFile);
            ld = load(sourceFile);
            A_num = ld.A_num;
            B_num = ld.B_num;
            K     = ld.K;
            lqr_Q = ld.lqr_Q;
            lqr_R = ld.lqr_R;
            if isfield(ld, 'leg_length_val')
                leg_length_val = ld.leg_length_val;
            end

        else
            % 回退到 workspace
            fprintf('[plot_theoretical_bode] 未找到 .mat 文件，回退到 workspace\n');
            sourceFile = 'workspace';
            A_num = [];
        end
    elseif ischar(A_num) || isstring(A_num)
        % 第一个参数是字符串 → 文件路径 或 'workspace'
        src = char(A_num);
        if strcmpi(src, 'workspace')
            sourceFile = 'workspace';
            A_num = [];
        else
            if ~isfile(src)
                error('文件不存在: %s', src);
            end
            sourceFile = src;
            ld = load(sourceFile);
            A_num = ld.A_num;
            B_num = ld.B_num;
            K     = ld.K;
            lqr_Q = ld.lqr_Q;
            lqr_R = ld.lqr_R;
            if isfield(ld, 'leg_length_val')
                leg_length_val = ld.leg_length_val;
            end
        end
    end

    % 如果上面没有拿到矩阵（workspace 回退），从 workspace 取
    if isempty(A_num)
        if ~evalin('base', 'exist(''A_num'',''var'') && exist(''B_num'',''var'') && exist(''K'',''var'')')
            error(['未找到 A_num / B_num / K。' newline ...
                   '请先运行 compute_lqr_0706.m，或传入 .mat 文件路径，或直接传入矩阵。']);
        end
        A_num = evalin('base', 'A_num');
        B_num = evalin('base', 'B_num');
        K     = evalin('base', 'K');
        if evalin('base', 'exist(''lqr_Q'',''var'')')
            lqr_Q = evalin('base', 'lqr_Q');
        else
            lqr_Q = eye(size(A_num, 1));
        end
        if evalin('base', 'exist(''lqr_R'',''var'')')
            lqr_R = evalin('base', 'lqr_R');
        else
            lqr_R = eye(size(B_num, 2));
        end
        if evalin('base', 'exist(''leg_length_val'',''var'')')
            leg_length_val = evalin('base', 'leg_length_val');
        end
    end

    if nargin < 4 || isempty(Fs)
        Fs = 100;
    end
    if nargin < 5 || isempty(figurePosition)
        figurePosition = [100, 100, 1800, 900];
    end
    if isfinite(leg_length_val)
        legLabel = sprintf('l=%.3f m', leg_length_val);
    else
        legLabel = 'l=unknown';
    end
    fprintf('Q矩阵 (状态权重):\n');
    disp(lqr_Q);
    
    fprintf('R矩阵 (控制权重):\n');
    disp(lqr_R);

    %% ---- 检查闭环稳定性 ----
    Acl = A_num - B_num * K;
    e_cl = eig(Acl);
    fprintf('[Theory Bode] 闭环特征值 max(Re): %.4e\n', max(real(e_cl)));
    if any(real(e_cl) > 1e-6)
        warning('闭环系统存在不稳定极点！Bode 图可能无意义。');
    end

    %% ---- 参考输入增益矩阵 ----
    Bref = B_num * K;  % 12×12: 输入为参考状态 r，Bref=r 如何进入动力学

    %% ---- 通道定义 ----
    % 每个 struct: .state_idx (1-based), .name, .label, .group
    % 与实验 Bode 的三个 group 对齐
    channels = {
        struct('state_idx', 2,  'name', 'vx',         'label', 'v_x (m/s)',             'group', 1);
        struct('state_idx', -1, 'name', 'vy',          'label', 'v_y (m/s)',             'group', 1);
        struct('state_idx', 4,  'name', 'wz',          'label', '\omega_z (rad/s)',     'group', 1);
        struct('state_idx', -1, 'name', 'body_roll',    'label', 'Roll (rad)',            'group', 2);
        struct('state_idx', 9,  'name', 'body_pitch',  'label', '\theta_b pitch (rad)',  'group', 2);
        struct('state_idx', 3,  'name', 'body_yaw',    'label', '\phi yaw (rad)',        'group', 2);
        struct('state_idx', -1, 'name', 'leg0_legx',    'label', 'L_0 leg length (m)',   'group', 3);
        struct('state_idx', -1, 'name', 'leg1_legx',    'label', 'L_1 leg length (m)',   'group', 3);
        struct('state_idx', 11, 'name', 'tail_beta',   'label', '\beta_{tail} (rad)',   'group', 3);
    };

    nTotal = length(channels);
    state_dim = size(A_num, 1);
    axisLabelFontSize = 14;
    tickFontSize = 11;

    %% ---- 频率向量 (Hz) ----
    % 与实验 Bode 保持一致的频率范围
    f_Hz = logspace(log10(0.05), log10(Fs/2), 500)';  % 0.05 Hz → Fs/2 Hz
    w_rad = 2 * pi * f_Hz;  % MATLAB bode 用 rad/s

    %% ---- 为每个通道计算闭环传函 ----
    theoretical_data = struct();
    theoretical_data.sourceFile = sourceFile;
    theoretical_data.f_Hz = f_Hz;
    theoretical_data.channels = cell(nTotal, 1);

    for ch = 1:nTotal
        chInfo = channels{ch};
        data = struct();
        data.name = chInfo.name;
        data.label = chInfo.label;
        data.group = chInfo.group;
        data.hasModel = (chInfo.state_idx > 0);
        data.state_idx = chInfo.state_idx;
        data.gainMargin = NaN;
        data.gainMargin_dB = NaN;
        data.phaseMargin_deg = NaN;
        data.gainMarginFreq_rad_s = NaN;
        data.phaseMarginFreq_rad_s = NaN;

        if data.hasModel
            i = chInfo.state_idx;
            % b_i = 参考 r 的第 i 个分量如何进入动力学
            b_i = Bref(:, i);
            % c_i = 提取第 i 个状态作为输出
            c_i = zeros(1, state_dim);
            c_i(i) = 1;

            % 构造 SISO 状态空间
            try
                sys_i = ss(Acl, b_i, c_i, 0);
                data.sys = sys_i;
                data.poles = pole(sys_i);
                data.zeros = zero(sys_i);

                % 频率响应 (用 freqresp 而非 bode，以便精确控制频率点)
                H = freqresp(sys_i, w_rad);
                H = H(:);  % SISO → 列向量

                data.mag_dB = 20 * log10(max(abs(H), eps));
                data.phase_deg = unwrap(angle(H)) * 180 / pi;

                % Gain/phase margins of the modeled SISO transfer function.
                [data.gainMargin, data.phaseMargin_deg, ...
                    data.gainMarginFreq_rad_s, data.phaseMarginFreq_rad_s] = margin(sys_i);
                if data.gainMargin > 0
                    data.gainMargin_dB = 20 * log10(data.gainMargin);
                end

                % -3dB 带宽
                idx_3dB = find(data.mag_dB <= -3, 1, 'first');
                if ~isempty(idx_3dB) && idx_3dB > 1
                    data.bw_Hz = f_Hz(idx_3dB);
                else
                    data.bw_Hz = NaN;
                end

                % 低频增益
                mask_low = f_Hz >= 0.1 & f_Hz <= 1.0;
                if any(mask_low)
                    data.g0_dB = mean(data.mag_dB(mask_low & isfinite(data.mag_dB)));
                else
                    data.g0_dB = NaN;
                end

                fprintf('[Theory Bode] %-14s: G_0=%.1f dB, -3dB @ %s, PM=%.2f deg, GM=%.2f dB\n', ...
                    data.name, data.g0_dB, ...
                    condstr2(isfinite(data.bw_Hz), sprintf('%.2f Hz', data.bw_Hz), 'out of band'), ...
                    data.phaseMargin_deg, data.gainMargin_dB);
            catch ME
                warning('频道 %s 传函计算失败: %s', data.name, ME.message);
                data.hasModel = false;
            end
        end

        theoretical_data.channels{ch} = data;
    end

    %% ---- 分组绘制 ----
    groupDefs = {
        struct('title', 'Velocity Tracking  —  Theoretical (LQR closed-loop, l=0.17 m)');
        struct('title', 'Attitude Tracking  —  Theoretical (LQR closed-loop, l=0.17 m)');
        struct('title', 'Leg & Tail Tracking —  Theoretical (LQR closed-loop, l=0.17 m)');
    };

    % 新版模型文件保存了目标腿长；旧文件则显示为 unknown。
    for g = 1:3
        groupDefs{g}.title = strrep(groupDefs{g}.title, 'l=0.17 m', legLabel);
    end

    for g = 1:3
        groupChs = {};
        for ch = 1:nTotal
            if channels{ch}.group == g
                groupChs{end+1} = ch; %#ok<AGROW>
            end
        end

        nCh = length(groupChs);
        f = figure('Name', groupDefs{g}.title, 'Color', 'w', 'Position', figurePosition);
        sgtitle(groupDefs{g}.title, 'FontSize', axisLabelFontSize);

        for k = 1:nCh
            ch = groupChs{k};
            data = theoretical_data.channels{ch};

            if data.hasModel
                % ==== 幅频图 ====
                ax_mag = subplot(2, nCh, k);
                semilogx(ax_mag, f_Hz, data.mag_dB, 'm-', 'LineWidth', 1.5);
                hold(ax_mag, 'on');
                yline(ax_mag, 0,  'k--', 'LineWidth', 0.6);
                yline(ax_mag, -3, 'r--', 'LineWidth', 0.6);

                bw_str = '';
                if isfinite(data.bw_Hz)
                    xline(ax_mag, data.bw_Hz, 'r--', 'LineWidth', 0.8);
                    bw_str = sprintf('  |  -3dB @ %.2f Hz', data.bw_Hz);
                end
                g0_str = '';
                if isfinite(data.g0_dB)
                    g0_str = sprintf('G_0≈%.1f dB  ', data.g0_dB);
                end

                grid(ax_mag, 'on');
                ylabel(ax_mag, 'Magnitude (dB)', 'FontSize', axisLabelFontSize);
                title(ax_mag, sprintf('%s  [%s%s]', data.label, g0_str, bw_str), ...
                    'FontSize', axisLabelFontSize - 2);
                set(ax_mag, 'FontSize', tickFontSize);
                yl_mag = ylim(ax_mag);
                ylim(ax_mag, [max(yl_mag(1), -60), min(yl_mag(2), 20)]);

                % ==== 相频图 ====
                ax_phase = subplot(2, nCh, nCh + k);
                semilogx(ax_phase, f_Hz, data.phase_deg, 'm-', 'LineWidth', 1.5);
                hold(ax_phase, 'on');
                yline(ax_phase, 0,   'k--', 'LineWidth', 0.6);
                yline(ax_phase, -90,  'k:', 'LineWidth', 0.5);
                yline(ax_phase, -180, 'k:', 'LineWidth', 0.5);
                yl_phase = ylim(ax_phase);
                ylim(ax_phase, [min(yl_phase(1), -360), max(yl_phase(2), 90)]);

                grid(ax_phase, 'on');
                xlabel(ax_phase, 'Frequency (Hz)', 'FontSize', axisLabelFontSize);
                ylabel(ax_phase, 'Phase (deg)', 'FontSize', axisLabelFontSize);
                set(ax_phase, 'FontSize', tickFontSize);

            else
                % ==== 无模型的占位通道 ====
                ax = subplot(2, nCh, k);
                text(0.5, 0.5, sprintf('%s\n(not in LQR model)', data.name), ...
                    'HorizontalAlign', 'center', 'FontSize', tickFontSize, ...
                    'Color', [0.5 0.5 0.5]);
                axis(ax, 'off');
                ax = subplot(2, nCh, nCh + k);
                text(0.5, 0.5, sprintf('%s\n(not in LQR model)', data.name), ...
                    'HorizontalAlign', 'center', 'FontSize', tickFontSize, ...
                    'Color', [0.5 0.5 0.5]);
                axis(ax, 'off');
            end
        end

        % 链接频率轴并限制显示范围至 Nyquist 频率 Fs/2
        allAxes = findobj(f, 'Type', 'Axes');
        if numel(allAxes) >= 2
            linkaxes(allAxes, 'x');
            xlim(allAxes(1), [0.05, Fs/2]);
        end
    end

    %% ---- One zero-pole figure for each modeled tracking channel ----
    modeledChs = find(cellfun(@(data) data.hasModel, theoretical_data.channels));
    for k = 1:length(modeledChs)
        data = theoretical_data.channels{modeledChs(k)};
        zpTitle = sprintf('Zero-Pole Map  -  %s', data.name);
        zpFigure = figure('Name', zpTitle, 'Color', 'w', 'Position', figurePosition);
        ax = axes('Parent', zpFigure);
        hold(ax, 'on');
        xline(ax, 0, 'k:', 'LineWidth', 0.6);
        yline(ax, 0, 'k:', 'LineWidth', 0.6);
        hPoles = plot(ax, real(data.poles), imag(data.poles), 'rx', ...
            'MarkerSize', 8, 'LineWidth', 1.5);
        hZeros = plot(ax, real(data.zeros), imag(data.zeros), 'bo', ...
            'MarkerSize', 7, 'LineWidth', 1.5);

        locations = [data.poles(:); data.zeros(:)];
        locations = locations(isfinite(locations));
        axisLimit = max(abs([real(locations); imag(locations)]));
        if isempty(axisLimit) || axisLimit == 0
            axisLimit = 1;
        else
            axisLimit = 1.1 * axisLimit;
        end

        grid(ax, 'on');
        axis(ax, 'equal');
        xlim(ax, [-axisLimit, axisLimit]);
        ylim(ax, [-axisLimit, axisLimit]);
        title(ax, sprintf('%s  -  LQR Closed-loop', data.label), ...
            'FontSize', axisLabelFontSize);
        xlabel(ax, 'Real Axis (rad/s)', 'FontSize', axisLabelFontSize - 2);
        ylabel(ax, 'Imaginary Axis (rad/s)', 'FontSize', axisLabelFontSize - 2);
        set(ax, 'FontSize', tickFontSize);
        legend(ax, [hPoles, hZeros], {'Poles', 'Zeros'}, 'Location', 'best');
    end

    %% ---- 打印汇总 ----
    fprintf('\n[Theory Bode] ====== 汇总 ======\n');
    fprintf('%-14s  %-10s  %-12s  %-10s  %-10s  %s\n', ...
        'Channel', 'G_0 (dB)', '-3dB (Hz)', 'PM (deg)', 'GM (dB)', 'Model?');
    fprintf('%-14s  %-10s  %-12s  %-10s  %-10s  %s\n', ...
        '-------', '--------', '---------', '--------', '-------', '------');
    for ch = 1:nTotal
        data = theoretical_data.channels{ch};
        if data.hasModel
            fprintf('%-14s  %8.2f     %-12s  %8.2f  %8.2f  %s\n', ...
                data.name, data.g0_dB, ...
                condstr2(isfinite(data.bw_Hz), sprintf('%.2f', data.bw_Hz), 'out-of-band'), ...
                data.phaseMargin_deg, data.gainMargin_dB, 'yes');
        else
            fprintf('%-14s  %-10s  %-12s  %-10s  %-10s  %s\n', ...
                data.name, 'N/A', 'N/A', 'N/A', 'N/A', 'no');
        end
    end
    fprintf('========================================\n');

    fprintf('\n[Theory Bode] ====== Stability Margins ======\n');
    fprintf('%-14s  %-14s  %-14s  %-14s  %-14s\n', ...
        'Channel', 'PM (deg)', 'PM freq (Hz)', 'GM (dB)', 'GM freq (Hz)');
    fprintf('%-14s  %-14s  %-14s  %-14s  %-14s\n', ...
        '-------', '-------', '------------', '-------', '------------');
    for ch = 1:nTotal
        data = theoretical_data.channels{ch};
        if data.hasModel
            fprintf('%-14s  %14.2f  %14.4g  %14.2f  %14.4g\n', ...
                data.name, data.phaseMargin_deg, ...
                data.phaseMarginFreq_rad_s / (2 * pi), data.gainMargin_dB, ...
                data.gainMarginFreq_rad_s / (2 * pi));
        else
            fprintf('%-14s  %-14s  %-14s  %-14s  %-14s\n', ...
                data.name, 'N/A', 'N/A', 'N/A', 'N/A');
        end
    end
    fprintf('===============================================================\n');
end

function s = condstr2(condition, trueStr, falseStr)
    if condition
        s = trueStr;
    else
        s = falseStr;
    end
end
