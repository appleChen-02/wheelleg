function report = review_right_hip_sweep(rawDir, resultDir, showFigures)
%REVIEW_RIGHT_HIP_SWEEP 右髋单通道扫频案例的逐项检查与聚焦绘图。
% 通常由 analyze_sysid_excitation 在主分析完成后自动调用，使用本次数据/结果目录。
% 单独无参数运行时，读取下方配置的历史案例目录。
% 只生成 case_* 补充结果，不修改原始 CSV 或 analyze_sysid_excitation 的输出。
% 此处频响沿用原结果的输入定义；command 模式的曲线不是独立开环对象列。
% 用法：cd matlab/系统辨识; review_right_hip_sweep
% 可选：review_right_hip_sweep(rawDir, resultDir, false) 不保留图窗。
% 主分析中 cfg.autoReviewRightHip 控制自动调用，cfg.reviewShowFigures 控制图窗显示。

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(fileparts(scriptDir));
if nargin < 1 || isempty(rawDir)
    rawDir = fullfile(projectDir, 'simulate', 'LQR_Simulation', 'logs', 'sine01101001');
end
if nargin < 2 || isempty(resultDir)
    resultDir = fullfile(scriptDir, 'results', '20261010_111911');
end
if nargin < 3, showFigures = true; end

T = readtable(fullfile(rawDir, 'system_identification_12.csv'));
R = readtable(fullfile(rawDir, 'system_identification_reference.csv'));
S = load(fullfile(resultDir, 'frf_data.mat'));
assert(isscalar(S.activeIdx) && S.activeIdx == 1, 'This worksheet requires right-hip-only excitation.');
assert(all(T.motor_saturation_mask == 0), 'This case worksheet requires an unsaturated recording.');

states = {'body_x','body_x_dot','yaw','yaw_dot','left_theta','left_theta_dot', ...
          'right_theta','right_theta_dot','pitch','pitch_dot','tail_beta','tail_beta_dot'};
units = {'m','m/s','rad','rad/s','rad','rad/s','rad','rad/s','rad','rad/s','rad','rad/s'};
inputs = {'right_hip_tp','left_hip_tp','right_wheel_t','left_wheel_t','tail_t'};
X = T{:, states};
assert(all(isfinite(X), 'all'), 'State data contain NaN/Inf.');
t = (double(T.control_tick_ms) - double(T.control_tick_ms(1))) / 1000;
exc = bitand(T.status_flags, 2) ~= 0;  % 连续控制阶段，包含正弦过零点。
assert(any(exc), 'No excitation-active samples.');
pre = t < t(find(exc, 1, 'first')) - S.cfg.guardSec;
post = t > t(find(exc, 1, 'last')) + S.cfg.guardSec;
assert(nnz(pre) > 1 && nnz(post) > 1, 'Missing pre/post baseline.');

xOp = mean(X(pre, :), 1);
preStd = std(X(pre, :), 0, 1);
excStd = std(X(exc, :), 0, 1);
peak = max(abs(X(exc, :) - xOp), [], 1);
shiftExc = mean(X(exc, :), 1) - xOp;
shiftPost = mean(X(post, :), 1) - xOp;
ratio = excStd ./ preStd;
stateMetrics = table(states', units', xOp', preStd', excStd', peak', shiftExc', shiftPost', ratio', ...
    'VariableNames', {'state','unit','mean_pre','std_pre','std_excitation', ...
    'peak_abs_delta_from_pre','mean_shift_excitation','mean_shift_post','std_ratio_excitation_to_pre'});
writetable(stateMetrics, fullfile(resultDir, 'case_state_metrics.csv'));

motors = {'J0','J1','J2','J3','W0','W1','Tail'};
% 本案例对应当前仿真模型的限幅；改变模型限幅后须同步这里。
limits = [40,40,40,40,4.5,4.5,7]';
M = T{:, strcat(motors, '_feedback_torque')};
motorPeak = max(abs(M(exc, :)), [], 1)';
motorMargin = table(motors', limits, motorPeak, 100 * motorPeak ./ limits, limits - motorPeak, ...
    'VariableNames', {'motor','limit_Nm','peak_abs_excitation_Nm','peak_percent_of_limit','remaining_to_limit_Nm'});
writetable(motorMargin, fullfile(resultDir, 'case_motor_margin.csv'));

C = T{:, strcat('command_', inputs)};
U = T{:, strcat('controller_', inputs)};
E = T{:, strcat('excitation_', inputs)};
A = T{:, strcat('actual_', inputs)};
eq = R{1, strcat('equilibrium_', inputs)};
residual = max(abs(C - U - E - eq), [], 'all');
actualError = max(abs(A - C), [], 'all');
assert(all(diff(double(T.control_tick_ms)) == 4), 'Sampling is not uniformly 4 ms.');
assert(all(diff(double(T.sequence)) == 2), 'Sequence gap.');

fprintf('\n[1] Data: %d samples, 250 Hz, pre/active/post %.3f / %.3f / %.3f s\n', ...
    height(T), nnz(pre)*0.004, nnz(exc)*0.004, nnz(post)*0.004);
fprintf('[2] Max |command-controller-equilibrium-excitation| = %.3g N*m\n', residual);
fprintf('    Max |actual-command| = %.3g N*m\n', actualError);
fprintf('[3] State metrics: deviations are relative to the pre-excitation MEAN, not the reference.\n');
disp(stateMetrics(ismember(stateMetrics.state, {'body_x','left_theta','right_theta','pitch','tail_beta'}), :));
fprintf('[4] Motor margins:\n'); disp(motorMargin);

% 聚焦四个角度状态，避免原图中 12 条曲线叠在一起无法区分。
f = S.f(:);
band = f >= S.band(1) & f <= S.band(2) & S.valid(:);
selected = [7, 9, 5, 11];  % 右腿摆角、pitch、左腿摆角、尾角。
visibility = 'off';
if showFigures, visibility = 'on'; end
fig = figure('Visible', visibility, 'Position', [100,100,1250,1050], 'Color', 'w');
layout = tiledlayout(fig, 4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:numel(selected)
    k = selected(row);
    G = squeeze(S.G(k, 1, :));
    % 单输入时两种相干度应一致。旧分析输出的 cohPair 曾因绝对 eps
    % 下限而低估低能量频点；本例使用保存的多重相干度，原结果保持不变。
    coherence = S.coh(k, :)';
    freq = f(band);
    values = {abs(G(band)), unwrap(angle(G(band)))*180/pi, coherence(band)};
    low = coherence(band) < 0.8;
    labels = {'Magnitude [rad/(N m)]','Phase [deg]','Coherence'};
    for column = 1:3
        ax = nexttile(layout, (row-1)*3 + column);
        if column == 1
            loglog(ax, freq, values{column}, 'LineWidth', 1.2);
        else
            semilogx(ax, freq, values{column}, 'LineWidth', 1.2);
        end
        hold(ax, 'on');
        plot(ax, freq(low), values{column}(low), 'o', 'Color', [0.85,0.35,0.1], 'MarkerSize', 4);
        if column == 3
            yline(ax, 0.8, '--'); yline(ax, 0.5, ':'); ylim(ax, [0,1.05]);
        end
        grid(ax, 'on'); xlim(ax, S.band); ylabel(ax, labels{column});
        title(ax, states{k}, 'Interpreter', 'none');
        if row == 4, xlabel(ax, 'Frequency [Hz]'); end
    end
end
title(layout, sprintf('Right hip sweep: input=%s; orange points: coherence <0.8\nClosed-loop experiment; command curves are not an independent open-loop plant column', S.cfg.frfInput), ...
    'Interpreter', 'none');
exportgraphics(fig, fullfile(resultDir, 'case_frf_focus.png'), 'Resolution', 160);
if ~showFigures, close(fig); end

% 用同一套逐窗去均值/Hann/FFT，演示“外部激励”与“右髋总输入”的区别。
% 不需要额外工具箱。此处三个信号顺序为 [pitch, command_right_hip, excitation_right_hip]。
Z = [T.pitch(exc), T.command_right_hip_tp(exc), T.excitation_right_hip_tp(exc)];
nwin = max(64, round(S.cfg.winSec * S.fs));
nfft = 2^nextpow2(nwin);
nf = nfft/2 + 1;
step = max(1, floor(nwin * (1-S.cfg.overlapFrac)));
window = 0.5 * (1-cos(2*pi*(0:nwin-1)'/(nwin-1)));
cross = complex(zeros(nf, 3, 3));
count = 0;
for first = 1:step:size(Z,1)-nwin+1
    segment = Z(first:first+nwin-1, :);
    segment = (segment - mean(segment, 1)) .* window;
    fftValues = fft(segment, nfft);
    fftValues = fftValues(1:nf, :);
    for i = 1:3
        for j = 1:3
            cross(:,i,j) = cross(:,i,j) + fftValues(:,i).*conj(fftValues(:,j));
        end
    end
    count = count + 1;
end
assert(count > 0, 'Recording is too short for the selected window.');
freq = (0:nf-1)' * S.fs / nfft;
HcommandExcitation = cross(:,2,3) ./ cross(:,3,3);
HpitchExcitation = cross(:,1,3) ./ cross(:,3,3);
HpitchCommand = cross(:,1,2) ./ cross(:,2,2);
requested = [0.5, 1, 2, 5, 10]';
indices = zeros(size(requested));
for k = 1:numel(requested), [~,indices(k)] = min(abs(freq-requested(k))); end
frequencyMetrics = table(freq(indices), abs(HcommandExcitation(indices)), abs(HpitchCommand(indices)), ...
    abs(HpitchExcitation(indices)), 'VariableNames', {'frequency_hz','command_over_excitation', ...
    'pitch_over_command_rad_per_Nm','pitch_over_excitation_rad_per_Nm'});
writetable(frequencyMetrics, fullfile(resultDir, 'case_pitch_frequency_metrics.csv'));
fprintf('[5] Frequency examples: input ratios are AC frequency-domain ratios, not mean torque ratios.\n');
disp(frequencyMetrics);
fprintf('Outputs: case_state_metrics.csv, case_motor_margin.csv, case_pitch_frequency_metrics.csv, case_frf_focus.png\n');
fprintf('One amplitude proves one usable operating condition; linearity still requires amplitude comparisons.\n');

report = struct('stateMetrics', stateMetrics, 'motorMargin', motorMargin, ...
    'frequencyMetrics', frequencyMetrics, 'inputResidualMax', residual, 'actualErrorMax', actualError);
end
