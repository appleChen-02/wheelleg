function analyze_sysid_excitation()
%ANALYZE_SYSID_EXCITATION 系统辨识模式（激励）闭环频响分析（单通道优先，天然支持多通道）
%
% 数据来源：usb_data_logger.py 采集的
%   - system_identification_12.csv          (0x10 帧, 12状态/5输入, 250Hz)
%   - system_identification_reference.csv   (0x12 参考配置帧, 1Hz)
%
% 与 analyze_sysid_no_excitation.m 的分工：
%   - 无激励脚本：固定工作点闭环稳态统计（工作点/漂移/波动/K一致性），作为基线；
%   - 本脚本：激励段的 输入->状态 非参数频响估计（H1）+ 多重相干 + 激励信号验证，
%     用于检查线性范围、共振频段，并输出 frf_data.mat 供后续参数模型拟合。
%
% 方法说明（闭环直接法辨识）：
%   - 激励叠加在 LQR 输出之后、VMC 之前，闭环内辨识采用直接法：
%     输入 u 取"总广义输入" command_*（= LQR + 平衡前馈 + 激励）中被激励的通道
%     （cfg.frfInput 可改为 'excitation' 或 'controller'，默认不要改）。
%     H1 估计 G(f) = S_yu * inv(S_uu)，多重相干 gamma^2 = 1 - res/S_yy。
%   - 单通道激励时 G 退化为普通 H1 频响、gamma^2 退化为普通相干；
%     多通道独立 PRBS 时同一套代码直接给出 MIMO FRF 矩阵与多重相干。
%   - 闭环直接法存在反馈噪声引入的偏差 inv(S_uu)*S_uv，激励幅值越大偏差越小；
%     因此只解读相干度可接受频段内的结果，且不要解读激励频带以外的频响。
%
% 前提约定（与固件 system_identification_param.h 对应）：
%   - profile_id = 1（12状态/5输入），双足 12 状态 LQR；
%   - SYS_ID_EXCITATION_ENABLE = 1，至少一个 excitation 通道非零；
%   - 时间轴一律使用 control_tick_ms，不使用 host_time_s（仅作参考对比）。
%
% 输出（results/<时间戳>/ 目录）：
%   summary.csv / excitation_check.csv / operating_point.csv /
%   frf_input_<通道>.csv / frf_data.mat / data_quality.json / fig_*.png，
%   并在命令行打印最终结论。
%
% 使用：把采集目录放到本脚本同目录的 data/ 下，或将 cfg.dataDir 指向该目录，
% 然后运行 analyze_sysid_excitation

clc; close all;

%% ============================== 配置 ==============================
cfg = struct();
cfg.dataDir  = 'C:\Users\0\Desktop\wheelleg\main_github\wheelleg\simulate\LQR_Simulation\logs\sine01100901';        % '' = 自动搜索（脚本目录、脚本目录/data、上位机/log）
cfg.file12   = 'system_identification_12.csv';
cfg.fileRef  = 'system_identification_reference.csv';
cfg.outRoot  = 'results';

cfg.tRange   = [];        % 手动截取 [t0 t1] (s, 相对首样本)；[] = 全部使用

cfg.expectedProfileId = 1;
cfg.expectedStateDim  = 12;
cfg.expectedInputDim  = 5;

cfg.excitationAbsTol  = 1e-4;   % N*m，excitation 通道视为零的阈值
cfg.ampRelTol         = 0.15;   % 激励幅值与期望值的相对偏差容限

% ---- 以下激励参数必须与 system_identification_param.h 保持一致 ----
cfg.ampExpected = [0.1, 0, 0, 0, 0];  % 右髋 左髋 右轮 左轮 尾巴 (N*m)
cfg.sweepF0 = 0.2;              % chirp 起始频率 Hz
cfg.sweepF1 = 20.0;             % chirp 终止频率 Hz
cfg.sweepT  = 240.0;             % chirp 时长 s
cfg.prbsHoldS = 10 * 0.002;     % PRBS 码元保持时间 s (BIT_HOLD_SAMPLES * 2ms)

cfg.minPreSec  = 5.0;     % 激励前稳态段最短长度，用于估计工作点
cfg.guardSec   = 0.5;     % 激励段与前后稳态段之间的保护间隔
cfg.winSec     = 10.0;    % Welch 窗长 -> 频率分辨率 0.1 Hz
cfg.overlapFrac = 0.5;    % Welch 重叠率

cfg.legLength = 0.18;     % SYS_ID_LEG_LENGTH

% 可选：无激励基线的 operating_point.csv（结果目录里），提供后工作点直接取基线值
cfg.baselineOpCsv = '';
% 可选：含名义模型 A(12x12)/B(12x5)/C(可选) 的 mat 文件，用于叠加模型预测频响
cfg.modelMatFile  = '';
% 辨识输入选择：'command'(总广义输入,推荐) | 'excitation'(外部激励) | 'controller'(仅LQR)
cfg.frfInput = 'command';

stateNames = {'body_x','body_x_dot','yaw','yaw_dot', ...
              'left_theta','left_theta_dot','right_theta','right_theta_dot', ...
              'pitch','pitch_dot','tail_beta','tail_beta_dot'};
inputNames = {'right_hip_tp','left_hip_tp','right_wheel_t','left_wheel_t','tail_t'};
motorNames = {'J0','J1','J2','J3','W0','W1','Tail'};
flagNames  = {'MODE_VALID','EXCITATION_ACTIVE','SWEEP_FINISHED','MOTOR_OFFLINE', ...
              'LEFT_JACOBIAN_BAD','RIGHT_JACOBIAN_BAD','ACTUAL_INPUT_VALID','REFERENCE_VALID'};
satNames   = {'J0','J1','J2','J3','W0','W1','Tail'};
units12    = {'m','m/s','rad','rad/s','rad','rad/s','rad','rad/s', ...
              'rad','rad/s','rad','rad/s'};

issues = {};   % 警告级问题列表

%% ============================ 数据定位 ============================
[dataDir, pathNote] = locateDataDir(cfg);
f12  = fullfile(dataDir, cfg.file12);
fref = fullfile(dataDir, cfg.fileRef);
fprintf('[数据] 目录: %s (%s)\n', dataDir, pathNote);
if ~exist(f12, 'file')
    error(['找不到 %s。\n请先用 usb_data_logger.py 采集系统辨识模式数据，' ...
           '并将采集目录放到本脚本同目录的 data/ 下，或修改 cfg.dataDir。'], f12);
end

fprintf('[数据] 读取 %s ...\n', cfg.file12);
T12 = readtable(f12);
hasRef = exist(fref, 'file') ~= 0;
if hasRef
    fprintf('[数据] 读取 %s ...\n', cfg.fileRef);
    Tref = readtable(fref);
else
    warning('未找到参考帧文件 %s，参考工作点相关检查将降级。', cfg.fileRef);
    Tref = table();
end

%% ============================ 列名校验 ============================
commonCols = {'control_tick_ms','sequence','experiment_id','reference_sequence', ...
    'protocol_version','profile_id','state_dimension','input_dimension', ...
    'excitation_config','excitation_input_mask','status_flags','motor_saturation_mask'};
stateCols = stateNames;
ctrlCols  = strcat('controller_', inputNames);
excCols   = strcat('excitation_', inputNames);
cmdCols   = strcat('command_', inputNames);
actCols   = strcat('actual_', inputNames);
motorCols = strcat(motorNames, '_feedback_torque');
needCols  = [commonCols, stateCols, ctrlCols, excCols, cmdCols, actCols, motorCols];
missing   = needCols(~ismember(needCols, T12.Properties.VariableNames));
if ~isempty(missing)
    error('CSV 缺少必需列: %s', strjoin(missing, ', '));
end

%% ============================ 基本校验 ============================
profileId = unique(T12.profile_id);
stateDim  = unique(T12.state_dimension);
inputDim  = unique(T12.input_dimension);
if numel(profileId) ~= 1 || profileId ~= cfg.expectedProfileId
    error('profile_id 不为 %d（实际: %s），数据不是 12状态/5输入 档位，终止。', ...
        cfg.expectedProfileId, mat2str(profileId));
end
if numel(stateDim) ~= 1 || stateDim ~= cfg.expectedStateDim || ...
        numel(inputDim) ~= 1 || inputDim ~= cfg.expectedInputDim
    error('state/input dimension 与 %d/%d 不符，终止。', ...
        cfg.expectedStateDim, cfg.expectedInputDim);
end

expIds = unique(T12.experiment_id);
if numel(expIds) > 1
    issues{end+1} = sprintf('experiment_id 不唯一: %s。', mat2str(expIds)); %#ok<AGROW>
end
expIdMain = mode(T12.experiment_id);

% ---- 激励存在性检查（与无激励脚本相反：这里必须有激励）----
excCfgVals = unique(T12.excitation_config);
excEnabled = any(bitand(T12.excitation_config, 1) == 1);
EXCall = [T12.excitation_right_hip_tp, T12.excitation_left_hip_tp, ...
          T12.excitation_right_wheel_t, T12.excitation_left_wheel_t, T12.excitation_tail_t];
excRmsAll = sqrt(mean(EXCall.^2, 1));
activeIdx = find(excRmsAll > 5 * cfg.excitationAbsTol);
if isempty(activeIdx) && ~excEnabled
    error(['数据中没有任何激励（使能位=0 且所有 excitation 通道低于阈值）。\n' ...
           '这份数据应使用 analyze_sysid_no_excitation.m 分析。']);
end
if isempty(activeIdx)
    error('激励使能位为 1 但所有 excitation 通道均低于阈值 %.1e，请检查固件与日志定义。', ...
        cfg.excitationAbsTol);
end
if ~excEnabled
    issues{end+1} = 'excitation 通道有信号但 excitation_config 使能位为 0，日志定义可能不一致。'; %#ok<AGROW>
end

% 激励掩码一致性：bit0~4 对应 5 个广义输入
excMaskVals = unique(T12.excitation_input_mask);
maskMain = mode(T12.excitation_input_mask);
maskBits = find(bitget(double(maskMain), 1:7));
if numel(excMaskVals) > 1
    issues{end+1} = sprintf('excitation_input_mask 不唯一: %s。', mat2str(excMaskVals)); %#ok<AGROW>
end
if ~isequal(sort(activeIdx(:)), sort(maskBits(:)))
    issues{end+1} = sprintf(['实际有信号的激励通道 [%s] 与掩码 0x%s 的位 [%s] 不一致，' ...
        '以实际信号为准。'], vecToText('%d', activeIdx), dec2hex(double(maskMain)), ...
        vecToText('%d', maskBits)); %#ok<AGROW>
end

% status_flags 统计
SF = T12.status_flags;
flagCounts = zeros(1, 8);
for b = 1:8
    flagCounts(b) = sum(bitand(SF, 2^(b-1)) ~= 0);
end
badStatusMask = bitand(SF, bitor(bitor(8, 16), 32)) ~= 0;  % OFFLINE|JAC_L|JAC_R
nBadStatus = sum(badStatusMask);
if nBadStatus > 0
    issues{end+1} = sprintf('%d/%d 帧带 MOTOR_OFFLINE 或 JACOBIAN_BAD 标志（对应段已被剔除/分段）。', ...
        nBadStatus, height(T12)); %#ok<AGROW>
end
sweepFinished = any(bitand(SF, 4) ~= 0);   % SWEEP_FINISHED

% 饱和掩码统计（激励数据中分段处理，不直接剔帧）
SAT = T12.motor_saturation_mask;
nSat = sum(SAT ~= 0);
satCounts = zeros(1, 7);
for b = 1:7
    satCounts(b) = sum(bitand(SAT, 2^(b-1)) ~= 0);
end

%% ============================ 时间轴 ==============================
tick = double(T12.control_tick_ms);
tAll = (tick - tick(1)) / 1000;
seq  = double(T12.sequence);

dt = diff(tAll);
if any(dt <= 0)
    issues{end+1} = sprintf('control_tick_ms 出现 %d 次非递增（回绕或异常）。', sum(dt <= 0)); %#ok<AGROW>
end
nLostSamples = max(1 + sum(max(round(max(dt, 0) / 0.004), 1)) - numel(tAll), 0);
dropRate = nLostSamples / max(nLostSamples + numel(tAll), 1);

dseq = diff(seq);
expectedSeqStep = max(round(max(dt, 0) / 0.002), 1);   % 固件控制周期 2ms
nSeqGap  = sum(dseq ~= expectedSeqStep);
nSeqLost = sum(max(dseq - expectedSeqStep, 0));
if nSeqGap > 0
    issues{end+1} = sprintf('sequence 有 %d 处不连续（累计缺 %d 帧）。', nSeqGap, nSeqLost); %#ok<AGROW>
end

hostDev = NaN;
if ismember('host_time_s', T12.Properties.VariableNames)
    h = T12.host_time_s;
    dev = (h - h(1)) - tAll;
    hostDev = max(abs(dev - median(dev)));
end

%% ========================== 手动时间截取 ==========================
useMask = true(height(T12), 1);
if ~isempty(cfg.tRange)
    useMask = tAll >= cfg.tRange(1) & tAll <= cfg.tRange(2);
    if nnz(useMask) < 100
        error('cfg.tRange 截取后仅剩 %d 帧，过少，终止。', nnz(useMask));
    end
end

t = tAll(useMask);  t = t - t(1);
S = [T12.body_x, T12.body_x_dot, T12.yaw, T12.yaw_dot, ...
     T12.left_theta, T12.left_theta_dot, T12.right_theta, T12.right_theta_dot, ...
     T12.pitch, T12.pitch_dot, T12.tail_beta, T12.tail_beta_dot];
U = [T12.controller_right_hip_tp, T12.controller_left_hip_tp, ...
     T12.controller_right_wheel_t, T12.controller_left_wheel_t, T12.controller_tail_t];
C = [T12.command_right_hip_tp, T12.command_left_hip_tp, ...
     T12.command_right_wheel_t, T12.command_left_wheel_t, T12.command_tail_t];
A = [T12.actual_right_hip_tp, T12.actual_left_hip_tp, ...
     T12.actual_right_wheel_t, T12.actual_left_wheel_t, T12.actual_tail_t];
MOTOR = [T12.J0_feedback_torque, T12.J1_feedback_torque, T12.J2_feedback_torque, ...
         T12.J3_feedback_torque, T12.W0_feedback_torque, T12.W1_feedback_torque, ...
         T12.Tail_feedback_torque];
EXC = EXCall(useMask, :);
S = S(useMask,:);  U = U(useMask,:);  C = C(useMask,:);  A = A(useMask,:);
MOTOR = MOTOR(useMask,:);  SF = SF(useMask);  SAT = SAT(useMask);
n0 = nnz(useMask);
badStatus = badStatusMask(useMask);
Sraw = S;   % 保留原始角（未 unwrap），仅用于绘图

% NaN/Inf 掩码（参与均值统计；Welch 分段时还会再分段）
finiteMask = all(isfinite([S U C A MOTOR EXC]), 2);
nNonFinite = n0 - sum(finiteMask);
if nNonFinite > 0
    issues{end+1} = sprintf('%d 帧含 NaN/Inf。', nNonFinite); %#ok<AGROW>
end
okRow = finiteMask & ~badStatus;

%% ============================ 角度处理 ============================
angleIdx = [3 5 7 9 11];      % yaw, left_theta, right_theta, pitch, tail_beta
wrapEvents = zeros(1, 12);
for k = angleIdx
    d = diff(Sraw(:, k));
    wrapEvents(k) = sum(abs(d) > pi);
    if wrapEvents(k) > 0
        issues{end+1} = sprintf('%s 跨越 ±pi %d 次，已做 unwrap。', ...
            stateNames{k}, wrapEvents(k)); %#ok<AGROW>
    end
    S(:, k) = unwrap(Sraw(:, k));
end

%% ======================== 激励段自动定位 ==========================
fs = 1 / median(diff(t(okRow)));
guard = round(cfg.guardSec * fs);

% 激励段以信号本身为准；信号检测失败时退回 EXCITATION_ACTIVE 标志
excSig = max(abs(EXC), [], 2) > cfg.excitationAbsTol;
i0 = find(excSig, 1, 'first');
i1 = find(excSig, 1, 'last');
if isempty(i0)
    excSig = bitand(SF, 2) ~= 0;
    i0 = find(excSig, 1, 'first');
    i1 = find(excSig, 1, 'last');
end
if isempty(i0)
    error('未检测到激励段（excitation 通道与 EXCITATION_ACTIVE 标志均未置位）。');
end
excT0 = t(i0);  excT1 = t(i1);
excDur = excT1 - excT0;

% 激励类型启发式：两能带(±幅值)判为 PRBS，否则扫频
x0 = EXC(:, activeIdx(1));
x0ok = x0(okRow);
ampPeak0 = max(abs(x0ok));
fracHigh = mean(abs(x0ok) > 0.7 * ampPeak0);
if fracHigh > 0.5
    excType = 'prbs';
    excTypeLabel = 'PRBS (两能带)';
else
    excType = 'sweep';
    excTypeLabel = '正弦扫频';
end

% 激励频带（用于相干度解读范围）
if strcmp(excType, 'sweep')
    band = [cfg.sweepF0, cfg.sweepF1];
else
    band = [0.2, min(0.5 / cfg.prbsHoldS, 0.45 * fs / 2)];
end

fprintf('[激励] 类型: %s, 通道: [%s], 区间 [%.2f, %.2f] s (%.1f s), 频带 [%.2f, %.2f] Hz\n', ...
    excTypeLabel, strjoin(inputNames(activeIdx), ','), excT0, excT1, excDur, band(1), band(2));

%% ============================ 工作点 ==============================
% 优先级：基线 operating_point.csv > 激励前稳态段均值 > 激励段均值(降级)
preMask = false(n0, 1);
if i0 > 1
    preMask(1:max(i0 - 1 - guard, 0)) = true;
end
postMask = false(n0, 1);
if i1 < n0
    postMask(min(i1 + 1 + guard, n0):n0) = true;
end

xPre = mean(S(preMask & okRow, :), 1);
xPost = mean(S(postMask & okRow, :), 1);
xExc = mean(S(excSig & okRow, :), 1);
if ~isempty(cfg.baselineOpCsv) && exist(cfg.baselineOpCsv, 'file') == 2
    Top = readtable(cfg.baselineOpCsv);
    xOp = nan(1, 12);
    for k = 1:12
        sel = strcmp(Top.channel, stateNames{k});
        if any(sel), xOp(k) = Top.mean(find(sel, 1)); end
    end
    if any(isnan(xOp))
        issues{end+1} = '基线 operating_point.csv 缺少部分状态，缺项用激励前段均值补齐。'; %#ok<AGROW>
        xOp(isnan(xOp)) = xPre(isnan(xOp));
    end
    opSource = 'baseline operating_point.csv';
elseif nnz(preMask & okRow) >= cfg.minPreSec * fs
    xOp = xPre;
    opSource = 'pre-excitation steady segment';
else
    xOp = xExc;
    opSource = 'excitation segment mean (downgraded)';
    issues{end+1} = '激励前稳态段不足 5 s 或缺失，工作点退化为激励段均值，低频频响解读需谨慎。'; %#ok<AGROW>
end

opShift = xExc - xOp;
[~, maxShiftIdx] = max(abs(opShift));

%% ======================== 激励信号验证 ============================
excCheck = verifyExcitation(EXC, okRow, excSig, t, i0, i1, excType, activeIdx, cfg, inputNames);
issues = [issues, excCheck.warnings];

%% ======================= 参考帧（对照用） =========================
ref12 = nan(1, 12);
equil5 = nan(1, 5);
refSource = '(reference frame missing)';
refSeqUsed = NaN;
if hasRef && height(Tref) > 0
    refCols = {'experiment_id','reference_sequence', ...
        'reference_body_x','reference_body_x_dot','reference_yaw','reference_yaw_dot', ...
        'reference_left_theta','reference_left_theta_dot','reference_right_theta', ...
        'reference_right_theta_dot','reference_pitch','reference_pitch_dot', ...
        'reference_tail_beta','reference_tail_beta_dot'};
    eqCols = {'equilibrium_right_hip_tp','equilibrium_left_hip_tp', ...
        'equilibrium_right_wheel_t','equilibrium_left_wheel_t','equilibrium_tail_t'};
    if all(ismember(refCols, Tref.Properties.VariableNames))
        refRows = Tref(Tref.experiment_id == expIdMain, :);
        if isempty(refRows), refRows = Tref(end, :); end
        r = refRows(end, :);
        ref12 = [r.reference_body_x, r.reference_body_x_dot, r.reference_yaw, r.reference_yaw_dot, ...
                 r.reference_left_theta, r.reference_left_theta_dot, r.reference_right_theta, ...
                 r.reference_right_theta_dot, r.reference_pitch, r.reference_pitch_dot, ...
                 r.reference_tail_beta, r.reference_tail_beta_dot];
        refSeqUsed = r.reference_sequence;
        refSource = '0x12 reference frame (last matched)';
        if all(ismember(eqCols, Tref.Properties.VariableNames))
            equil5 = [r.equilibrium_right_hip_tp, r.equilibrium_left_hip_tp, ...
                      r.equilibrium_right_wheel_t, r.equilibrium_left_wheel_t, ...
                      r.equilibrium_tail_t];
        end
    end
end

%% ====================== 干净分段（Welch 输入） ====================
% 饱和/异常/非有限帧作为分段边界，不删除帧，避免时间轴空洞
Nwin = max(64, round(cfg.winSec * fs));
segMask = okRow & (SAT == 0);
runs = findRuns(segMask);
% 与激励段求交
for rr = numel(runs):-1:1
    runs{rr} = [max(runs{rr}(1), i0), min(runs{rr}(2), i1)];
    if runs{rr}(2) - runs{rr}(1) + 1 < Nwin
        runs(rr) = [];
    end
end
if isempty(runs)
    error(['激励段内没有长度 >= %.1f s 的连续干净分段（无饱和/离线/雅可比异常/NaN）。\n' ...
           '请检查饱和掩码（%d 帧饱和）或加长采集。'], cfg.winSec, nSat);
end
nFramesUsed = sum(cellfun(@(r) r(2) - r(1) + 1, runs));
nFramesDropped = (i1 - i0 + 1) - nFramesUsed;

%% ====================== 频响估计（H1 + 相干） =====================
switch lower(cfg.frfInput)
    case 'command'
        uMat = C(:, activeIdx);
    case 'excitation'
        uMat = EXC(:, activeIdx);
    case 'controller'
        uMat = U(:, activeIdx);
    otherwise
        error('cfg.frfInput 只能为 command / excitation / controller。');
end
activeNames = inputNames(activeIdx);
nU = numel(activeIdx);

Z = [S, uMat];                          % 12 状态 + nU 输入（Welch 内部逐窗去均值）
[Sblk, f, nsegTot] = crossSpectraRuns(runs, Z, fs, cfg.winSec, cfg.overlapFrac);
df = f(2) - f(1);
Nf = numel(f);

% 每个频点的 H1 估计与相干
G  = complex(zeros(12, nU, Nf));        % y ≈ G(f) u
coh = zeros(12, Nf);                    % 多重相干（给定当前输入集合）
cohPair = zeros(12, nU, Nf);            % 普通(逐输入-输出对)相干
valid = false(1, Nf);
for kf = 1:Nf
    M = Sblk(:, :, kf);                 % m×m 互谱矩阵，S_ab = E[X_a conj(X_b)]
    Syy = M(1:12, 1:12);                % S_ab = E[X_a conj(X_b)]
    Suu = M(12+1:end, 12+1:end);
    Syu = M(1:12, 12+1:end);            % E[Y_a conj(U_b)]
    if nU == 1
        ok = real(Suu(1,1)) > eps;
    else
        try
            ok = rcond(Suu) > 1e-10;
        catch
            ok = false;
        end
    end
    if ~ok, continue; end
    valid(kf) = true;
    G(:, :, kf) = Syu / Suu;
    for ii = 1:12
        powerY = real(Syy(ii, ii));
        res = powerY - real(Syu(ii, :) * (Suu \ Syu(ii, :)'));
        % PSD 有物理单位，不能把小于机器 eps 的正功率抬高到 eps。
        % 理想仿真输出很小，固定绝对下限会使相干度随量纲/幅值而改变。
        if powerY > 0
            coh(ii, kf) = min(max(1 - res / powerY, 0), 1);
        end
        for jj = 1:nU
            powerU = real(Suu(jj, jj));
            if powerY > 0 && powerU > 0
                % 先按两个功率的平方根归一化，再平方，避免功率乘积过小。
                normalizedCross = abs(Syu(ii, jj)) / sqrt(powerY) / sqrt(powerU);
                cohPair(ii, jj, kf) = min(max(normalizedCross^2, 0), 1);
            end
        end
    end
end

% 激励频带内、有效频点上的平均相干
bandSel = (f >= band(1) & f <= band(2) & valid(:));
bandCoh = mean(coh(:, bandSel), 2, 'omitnan');

% 相干度不足提示
poorStates = find(bandCoh < 0.5);
if ~isempty(poorStates)
    issues{end+1} = sprintf('激励频带内相干度 < 0.5 的状态: %s（这些状态的频响不可信，考虑加大幅值或延长采集）。', ...
        strjoin(stateNames(poorStates), ',')); %#ok<AGROW>
end

% 可选：名义模型频响叠加
Gmodel = [];
if ~isempty(cfg.modelMatFile) && exist(cfg.modelMatFile, 'file') == 2
    mload = load(cfg.modelMatFile);
    if isfield(mload, 'A') && isfield(mload, 'B')
        Am = mload.A;  Bm = mload.B;
        if isfield(mload, 'C'), Cm = mload.C; else, Cm = eye(12); end
        Gmodel = complex(zeros(12, nU, Nf));
        for kf = 1:Nf
            if ~valid(kf), continue; end
            Gmodel(:, :, kf) = Cm * ((1j * 2 * pi * f(kf) * eye(12) - Am) \ Bm(:, activeIdx));
        end
        fprintf('[模型] 已叠加名义模型频响: %s\n', cfg.modelMatFile);
    else
        warning('modelMatFile 中缺少 A/B，跳过模型叠加。');
    end
end

%% ============================ 结果输出 ===========================
outDir = fullfile(cfg.outRoot, datestr(now, 'yyyymmdd_HHMMSS'));
if ~exist(outDir, 'dir'); mkdir(outDir); end
fprintf('[输出] 结果目录: %s\n', outDir);

% ---- summary.csv ----
summaryRows = {
    'data_dir', dataDir
    'profile_id', num2str(profileId)
    'experiment_id', num2str(expIdMain)
    'n_samples_raw', num2str(height(T12))
    'n_samples_used', num2str(n0)
    'fs_hz', num2str(fs)
    'lost_samples_from_tick', num2str(nLostSamples)
    'drop_rate', num2str(dropRate)
    'sequence_gaps', num2str(nSeqGap)
    'sequence_lost_frames', num2str(nSeqLost)
    'host_time_max_dev_s', num2str(hostDev)
    'excitation_type_detected', excType
    'excitation_channels', strjoin(activeNames, ',')
    'excitation_mask', sprintf('0x%s', dec2hex(double(maskMain)))
    'excitation_window_s', sprintf('[%.3f, %.3f]', excT0, excT1)
    'excitation_duration_s', num2str(excDur)
    'analysis_band_hz', sprintf('[%.2f, %.2f]', band(1), band(2))
    'frf_input_source', cfg.frfInput
    'welch_window_s', num2str(cfg.winSec)
    'freq_resolution_hz', num2str(df)
    'welch_segments', num2str(nsegTot)
    'clean_runs', num2str(numel(runs))
    'frames_dropped_in_window', num2str(nFramesDropped)
    'motor_saturation_frames_total', num2str(nSat)
    'saturation_counts', vecToText('%s=%d', {satNames; satCounts})
    'operating_point_source', opSource
    'max_operating_point_shift', sprintf('%s: %.5f', char(stateNames(maxShiftIdx)), opShift(maxShiftIdx))
    'ref_source', refSource
    'sweep_finished_flag', num2str(double(sweepFinished))
    'issues_count', num2str(numel(issues))
    };
summaryT = table(summaryRows(:,1), summaryRows(:,2), 'VariableNames', {'key','value'});
writeCsv(summaryT, fullfile(outDir, 'summary.csv'));

% ---- excitation_check.csv ----
writeCsv(excCheck.csv, fullfile(outDir, 'excitation_check.csv'));

% ---- operating_point.csv ----
opT = table(stateNames(:), xExc(:), xPre(:), xPost(:), xOp(:), ...
    repmat({opSource}, 12, 1), ...
    'VariableNames', {'state','mean_excitation_window','mean_pre','mean_post','x_op_used','source'});
writeCsv(opT, fullfile(outDir, 'operating_point.csv'));

% ---- frf_input_<通道>.csv ----
for jj = 1:nU
    magD = abs(squeeze(G(:, jj, :)));
    phD = unwrap(angle(squeeze(G(:, jj, :))), [], 2) * 180 / pi;
    cohD = squeeze(cohPair(:, jj, :));   % 12×Nf 普通相干
    Tfrf = table(f, 'VariableNames', {'freq_hz'});
    for ii = 1:12
        Tfrf.(['mag_' stateNames{ii}]) = magD(ii, :)';
        Tfrf.(['phase_deg_' stateNames{ii}]) = phD(ii, :)';
        Tfrf.(['coh_' stateNames{ii}]) = cohD(ii, :)';
    end
    writeCsv(Tfrf, fullfile(outDir, sprintf('frf_input_%s.csv', activeNames{jj})));
end

% ---- frf_data.mat（供后续参数模型拟合） ----
save(fullfile(outDir, 'frf_data.mat'), 'f', 'G', 'coh', 'cohPair', 'valid', 'band', ...
     'activeIdx', 'activeNames', 'stateNames', 'inputNames', 'fs', 'df', ...
     'nsegTot', 'cfg', 'Gmodel', '-v7');

% ---- data_quality.json ----
dq = struct();
dq.data_dir = dataDir;
dq.n_samples_raw = height(T12);
dq.n_samples_used = n0;
dq.fs_hz = fs;
dq.lost_samples_from_tick = nLostSamples;
dq.drop_rate = dropRate;
dq.sequence_gaps = nSeqGap;
dq.sequence_lost_frames = nSeqLost;
dq.host_time_max_dev_s = hostDev;
dq.experiment_id = expIdMain;
dq.excitation = struct('type_detected', excType, 'channels', {activeNames}, ...
    'mask', sprintf('0x%s', dec2hex(double(maskMain))), ...
    'window_s', [excT0 excT1], 'duration_s', excDur, ...
    'band_hz', band, 'sweep_finished_flag', double(sweepFinished));
dq.status_flag_counts = names2struct(flagNames, flagCounts);
dq.motor_saturation_frames = nSat;
dq.saturation_counts = names2struct(satNames, satCounts);
dq.angle_wrap_events = names2struct(stateNames, wrapEvents);
dq.n_nonfinite_frames = nNonFinite;
dq.n_bad_status_frames = nBadStatus;
dq.operating_point = struct('source', opSource, 'x_op_used', xOp, ...
    'mean_excitation_window', xExc, 'mean_pre', xPre, 'mean_post', xPost, ...
    'shift_exc_minus_op', opShift, 'state_names', {stateNames});
dq.reference = struct('source', refSource, 'reference_sequence', refSeqUsed, ...
    'state_reference', ref12, 'equilibrium_input', equil5);
dq.welch = struct('window_s', cfg.winSec, 'overlap', cfg.overlapFrac, ...
    'freq_resolution_hz', df, 'segments', nsegTot, 'clean_runs', numel(runs), ...
    'frames_dropped_in_window', nFramesDropped);
dq.frf = struct('input_source', cfg.frfInput, 'channels', {activeNames}, ...
    'band_avg_coherence', names2struct(stateNames, bandCoh));
dq.issues = issues;
dqJson = jsonencode(dq, 'PrettyPrint', true);
fq = fopen(fullfile(outDir, 'data_quality.json'), 'w');
fwrite(fq, dqJson, 'char');
fclose(fq);

%% ============================== 绘图 ==============================
tWin = t(i0:i1);

% 图1: 激励通道时域（叠加 chirp 理论重建）+ 第一通道三种力矩对比
fh1 = figure('Visible','off','Position',[40 40 1400 300 + 260*nU]);
for jj = 1:nU
    subplot(nU + 1, 1, jj);
    ch = activeIdx(jj);
    plot(tWin, EXC(i0:i1, ch), 'Color', [.1 .35 .75]); hold on; grid on;
    ylabel(inputNames{ch}, 'Interpreter', 'none');
    if strcmp(excType, 'sweep') && ~isempty(excCheck.recon{ch})
        plot(tWin, excCheck.recon{ch}, 'Color', [.85 .5 .1]);
        legend({'logged', 'chirp recon'}, 'Interpreter', 'none', 'Location', 'best');
    end
    if jj == 1
        title(sprintf('Excitation channels (%s)', excTypeLabel), 'Interpreter', 'none');
    end
end
subplot(nU + 1, 1, nU + 1);
plot(tWin, U(i0:i1, activeIdx(1)), 'Color', [.1 .35 .75]); hold on;
plot(tWin, C(i0:i1, activeIdx(1)), 'Color', [.85 .5 .1]);
plot(tWin, A(i0:i1, activeIdx(1)), 'Color', [.2 .6 .2]);
grid on; ylabel(inputNames{activeIdx(1)}, 'Interpreter', 'none');
xlabel('t (s)');
legend({'controller', 'command', 'actual'}, 'Interpreter', 'none', 'Location', 'best');
title('First excited channel: controller vs command vs actual', 'Interpreter', 'none');
savefigPng(fh1, fullfile(outDir, 'fig1_excitation_time.png'));

% 图2: 激励段状态时域（相对工作点偏差）
fh2 = figure('Visible','off','Position',[60 60 1400 900]);
for k = 1:12
    subplot(4, 3, k);
    plot(tWin, S(i0:i1, k) - xOp(k), 'Color', [.1 .35 .75]);
    grid on;
    ylabel(units12{k}, 'Interpreter', 'none');
    title(sprintf('%s - x_{op}', stateNames{k}), 'Interpreter', 'none');
end
sgtitle('States during excitation (deviation from operating point)', 'Interpreter', 'none');
savefigPng(fh2, fullfile(outDir, 'fig2_states_time.png'));

% 图3/4: 频响幅值、相位（每个激励输入一张）
for jj = 1:nU
    magD = abs(squeeze(G(:, jj, :)));
    phD  = unwrap(angle(squeeze(G(:, jj, :))), [], 2) * 180 / pi;
    sel = f > 0 & valid(:);

    fh3 = figure('Visible','off','Position',[80 80 1100 800]);
    for k = 1:12
        loglog(f(sel), magD(k, sel), 'Color', [.1 .35 .75]); hold on;
    end
    if ~isempty(Gmodel)
        Gm = abs(squeeze(Gmodel(:, jj, :)));
        for k = 1:12
            loglog(f(sel), Gm(k, sel), 'k--');
        end
    end
    xline(band(1), ':k'); xline(band(2), ':k');
    grid on; ylabel('|G|'); xlabel('f (Hz)');
    title(sprintf('FRF magnitude: %s -> states (dashed = model)', activeNames{jj}), ...
        'Interpreter', 'none');
    savefigPng(fh3, fullfile(outDir, sprintf('fig3_frf_mag_%s.png', activeNames{jj})));

    fh4 = figure('Visible','off','Position',[100 100 1100 800]);
    for k = 1:12
        semilogx(f(sel), phD(k, sel), 'Color', [.1 .35 .75]); hold on;
    end
    xline(band(1), ':k'); xline(band(2), ':k');
    grid on; ylabel('phase (deg)'); xlabel('f (Hz)');
    title(sprintf('FRF phase: %s -> states', activeNames{jj}), 'Interpreter', 'none');
    savefigPng(fh4, fullfile(outDir, sprintf('fig4_frf_phase_%s.png', activeNames{jj})));
end

% 图5: 多重相干 + 输入 PSD（频带覆盖）
fh5 = figure('Visible','off','Position',[120 120 1100 800]);
subplot(2, 1, 1);
sel = f > 0 & valid(:);
for k = 1:12
    semilogx(f(sel), coh(k, sel), 'Color', [.1 .35 .75]); hold on;
end
yline(0.8, '--r'); yline(0.5, ':r');
xline(band(1), ':k'); xline(band(2), ':k');
grid on; ylabel('\gamma^2'); xlabel('f (Hz)');
ylim([0 1]);
title('Multiple coherence (given excited input set)', 'Interpreter', 'none');
subplot(2, 1, 2);
for jj = 1:nU
    loglog(f(sel), squeeze(real(Sblk(12 + jj, 12 + jj, sel))), 'Color', [.85 .5 .1]); hold on;
end
xline(band(1), ':k'); xline(band(2), ':k');
grid on; ylabel('PSD(u command)'); xlabel('f (Hz)');
title('Input command PSD (excitation band coverage)', 'Interpreter', 'none');
legend(activeNames, 'Interpreter', 'none', 'Location', 'best');
savefigPng(fh5, fullfile(outDir, 'fig5_coherence.png'));

%% ========================== 最终结论打印 ==========================
fprintf('\n');
fprintf('================ 系统辨识激励分析结论 ================\n');
fprintf('数据: %s, %.1f Hz, 全部 %d 帧, 使用 %d 帧\n', dataDir, fs, height(T12), n0);
fprintf('激励: %s, 通道 [%s], 区间 [%.2f, %.2f] s, 时长 %.1f s\n', ...
    excTypeLabel, strjoin(activeNames, ','), excT0, excT1, excDur);
fprintf('丢帧率 %.3f%% (tick), sequence 缺帧 %d, 饱和 %d 帧\n', 100*dropRate, nSeqLost, nSat);
fprintf('\n--- 1) 激励信号验证 ---\n');
for jj = 1:numel(excCheck.rows)
    fprintf('  %s\n', excCheck.rows{jj});
end
fprintf('\n--- 2) 工作点（来源: %s）---\n', opSource);
fprintf('%-16s %12s %12s %12s %12s\n', 'state', 'x_op', 'exc_mean', 'pre_mean', 'shift');
for k = 1:12
    fprintf('%-16s %12.5f %12.5f %12.5f %+12.5f\n', ...
        stateNames{k}, xOp(k), xExc(k), xPre(k), xExc(k) - xOp(k));
end
fprintf('\n--- 3) 激励频带内平均相干度 (band %.2f-%.2f Hz) ---\n', band(1), band(2));
[~, order] = sort(bandCoh, 'descend');
for k = order(:)'
    fprintf('%-16s gamma2=%.3f %s\n', stateNames{k}, bandCoh(k), ...
        ternary(bandCoh(k) >= 0.8, '(good)', ...
        ternary(bandCoh(k) >= 0.5, '(acceptable)', '(poor)')));
end
fprintf('\n--- 4) 频响估计设置 ---\n');
fprintf('输入源: %s (%s), Welch 窗 %.1f s, 分辨率 %.3f Hz, %d 段 / %d 个干净分段\n', ...
    cfg.frfInput, strjoin(activeNames, ','), cfg.winSec, df, nsegTot, numel(runs));
fprintf('激励段内因饱和/异常丢弃 %d 帧（分段处理，未删帧）\n', nFramesDropped);
fprintf('\n--- 5) 解读提示 ---\n');
fprintf('1. 只解读频带 [%.2f, %.2f] Hz 内且 gamma2 >= 0.5 的频响；\n', band(1), band(2));
fprintf('2. 低于 %.2f Hz（约 2/窗长）的频响受窗长限制，不可信；\n', 2/cfg.winSec);
fprintf('3. 闭环直接法频响含反馈噪声偏差，激励幅值越大偏差越小；\n');
fprintf('4. 线性范围检查：用不同幅值重复实验，对比 FRF 是否重合；\n');
fprintf('5. frf_data.mat 已保存（f/G/coh），可直接用于参数模型拟合。\n');
if ~isempty(issues)
    fprintf('\n*** 数据质量问题 (%d 条, 详见 data_quality.json) ***\n', numel(issues));
    for ii = 1:numel(issues)
        fprintf('  [%d] %s\n', ii, issues{ii});
    end
end
fprintf('============================================================\n');
fprintf('全部结果已写入: %s\n', outDir);

end

%% =================================================================
%%                           局部函数
%% =================================================================
function [dataDir, note] = locateDataDir(cfg)
% 自动搜索数据目录：cfg.dataDir > 脚本目录/data > 脚本目录 > 上位机/log
scriptDir = fileparts(mfilename('fullpath'));
candidates = {};
if ~isempty(cfg.dataDir)
    candidates{end+1} = cfg.dataDir;
end
candidates{end+1} = fullfile(scriptDir, 'data');
candidates{end+1} = scriptDir;
candidates{end+1} = fullfile(scriptDir, '..', '..', '上位机', 'log');
for i = 1:numel(candidates)
    if exist(fullfile(candidates{i}, cfg.file12), 'file')
        dataDir = candidates{i};
        note = sprintf('auto-detected (candidate %d)', i);
        return;
    end
end
dataDir = candidates{1};
note = 'NOT FOUND (run will fail)';
end

function runs = findRuns(mask)
% 连续 true 段 -> {[i0 i1], ...}
d = diff([false; mask(:); false]);
i0 = find(d == 1);
i1 = find(d == -1) - 1;
runs = arrayfun(@(a, b) [a b], i0, i1, 'UniformOutput', false);
end

function [Savg, f, nsegTot] = crossSpectraRuns(runs, Z, fs, winSec, overlapFrac)
% 分段 Welch 互谱矩阵估计。
% Z: n×m；runs: cell of [i0 i1]。逐窗去均值 + Hann 窗。
% 返回 Savg: m×m×Nf，S_ij(f) = E[X_i conj(X_j)]（单边谱）；f；总段数。
% 注意频率维必须放在最后：MATLAB 隐式扩展沿尾部维度对齐，
% m×m×1 才能与 m×m×Nf 逐频点相加（若频率维在第一维会把全频段之和广播到每个频点）。
m = size(Z, 2);
Nwin = max(64, round(winSec * fs));
Nfft = 2 ^ nextpow2(Nwin);
Nf = Nfft/2 + 1;
w = 0.5 * (1 - cos(2 * pi * (0:Nwin-1)' / (Nwin - 1)));
Uw = sum(w.^2);
scale = 2 / (fs * Uw);
acc = zeros(m, m, Nf);
nsegTot = 0;
step = max(floor(Nwin * (1 - overlapFrac)), 1);
for rr = 1:numel(runs)
    idx = runs{rr}(1):runs{rr}(2);
    n = numel(idx);
    if n < Nwin, continue; end
    X = Z(idx, :);
    for s0 = 1:step:(n - Nwin + 1)
        seg = X(s0:s0+Nwin-1, :);
        seg = seg - mean(seg, 1);
        seg = seg .* w;
        F = fft(seg, Nfft);
        Fh = F(1:Nf, :).';   % m×Nf，Fh(a,k) = 第 k 个频点上通道 a 的谱
        % 逐频点互谱张量: T(a,b,k) = F_ka * conj(F_kb)
        % 注意不能用 F'*F：那会把所有频点求和收缩成单一矩阵
        T2 = reshape(Fh, m, 1, Nf) .* reshape(conj(Fh), 1, m, Nf);
        acc = acc + scale * T2;
        nsegTot = nsegTot + 1;
    end
end
if nsegTot == 0
    error('Welch 互谱估计没有任何完整窗口（分段过短或窗长过大）。');
end
acc(:,:,1) = 0.5 * acc(:,:,1);        % DC 不做单边补偿
acc(:,:,end) = 0.5 * acc(:,:,end);    % Nyquist 同理
Savg = acc / nsegTot;
f = (0:Nfft/2)' * fs / Nfft;
end

function excCheck = verifyExcitation(EXC, okRow, excSig, t, i0, i1, excType, activeIdx, cfg, inputNames)
% 激励信号验证：幅值、扫频瞬时频率 / PRBS 电平与码元保持时间
excCheck = struct();
excCheck.rows = {};
excCheck.warnings = {};
excCheck.recon = cell(1, numel(EXC(1, :)));
checkTable = {};

tWin = t(i0:i1);
for jj = 1:numel(activeIdx)
    ch = activeIdx(jj);
    x = EXC(:, ch);
    xw = x(excSig & okRow);
    tw = t(excSig & okRow);
    ampExp = cfg.ampExpected(ch);
    % 幅值：取中段 98 百分位，避开首尾不完整周期
    interior = tw >= t(i0) + 0.15 * (t(i1) - t(i0)) & tw <= t(i1) - 0.15 * (t(i1) - t(i0));
    if nnz(interior) < 50, interior = true(size(tw)); end
    ampEst = localPrctile(abs(xw(interior)), 98);
    ampOK = abs(ampEst - ampExp) <= cfg.ampRelTol * max(ampExp, eps);

    if strcmp(excType, 'sweep')
        % 瞬时频率：上穿零点间隔
        xs = xw(:);
        up = find(xs(1:end-1) < 0 & xs(2:end) >= 0);
        recon = [];
        if numel(up) >= 3
            tUp = tw(up);
            per = diff(tUp);
            tIf = (tUp(1:end-1) + tUp(2:end)) / 2;
            ifreq = 1 ./ per;
            % 线性拟合 f(t) = a*t + b，斜率应 ≈ (f1-f0)/T
            pFit = polyfit(tIf, ifreq, 1);
            slopeExp = (cfg.sweepF1 - cfg.sweepF0) / cfg.sweepT;
            fAtStart = polyval(pFit, t(i0));
            fAtEnd = polyval(pFit, t(i1));
            slopeOK = abs(pFit(1) - slopeExp) <= 0.3 * max(slopeExp, eps);
            excCheck.rows{end+1} = sprintf( ...
                '%-14s sweep: amp=%.3f/%.3f N*m %s, f(%.1fs)=%.2f Hz, f(%.1fs)=%.2f Hz, slope=%.3f (期望 %.3f) %s', ...
                inputNames{ch}, ampEst, ampExp, ternary(ampOK, 'OK', 'MISMATCH'), ...
                t(i0), fAtStart, t(i1), fAtEnd, pFit(1), slopeExp, ...
                ternary(slopeOK, 'OK', 'MISMATCH')); %#ok<AGROW>
            if abs(fAtStart - cfg.sweepF0) > 1.0
                excCheck.warnings{end+1} = sprintf( ...
                    '%s: 扫频起始时刻瞬时频率 %.2f Hz 与 %.2f Hz 偏差过大，录制可能在扫频开始之后才开始，或参数不一致。', ...
                    inputNames{ch}, fAtStart, cfg.sweepF0); %#ok<AGROW>
            end
            if abs((t(i1) - t(i0)) - cfg.sweepT) > 0.1 * cfg.sweepT
                excCheck.warnings{end+1} = sprintf( ...
                    '%s: 激励段时长 %.1f s 与扫频时长 %.1f s 偏差超过 10%%。', ...
                    inputNames{ch}, t(i1) - t(i0), cfg.sweepT); %#ok<AGROW>
            end
            % 理论 chirp 重建（用于图 1 叠加对比）
            tau = tWin - t(i0);
            recon = ampEst .* sin(2 * pi * (cfg.sweepF0 .* tau + ...
                0.5 * (cfg.sweepF1 - cfg.sweepF0) / cfg.sweepT .* tau.^2)) .* ...
                (tau >= 0 & tau <= cfg.sweepT);
        else
            excCheck.rows{end+1} = sprintf( ...
                '%-14s sweep: 上穿零点不足，无法估计瞬时频率（段太短或幅值太小）', ...
                inputNames{ch}); %#ok<AGROW>
            excCheck.warnings{end+1} = sprintf('%s: 扫频信号零交叉不足，无法验证瞬时频率。', ...
                inputNames{ch}); %#ok<AGROW>
        end
        excCheck.recon{ch} = recon;
        row = {inputNames{ch}, 'sine_sweep', ampEst, ampExp, double(ampOK), ...
               NaN, NaN, NaN};
    else
        % PRBS：两能带 + 码元保持时间
        pk = max(abs(xw)) + eps;
        lvlPos = mean(xw > 0.7 * pk);
        lvlNeg = mean(xw < -0.7 * pk);
        sg = sign(xw);
        sg(abs(xw) < 0.2 * pk) = 0;
        sgNz = sg(sg ~= 0);
        dsg = diff(sgNz);
        runBounds = find(dsg ~= 0);
        if numel(runBounds) >= 2
            frameDt = median(diff(t));   % USB 帧周期 ~4ms（激励按控制周期生成、按帧采样）
            runLen = diff([0; runBounds; numel(sgNz)]) * frameDt;
            holdEst = median(runLen);
        else
            holdEst = NaN;
        end
        holdOK = ~isnan(holdEst) && abs(holdEst - cfg.prbsHoldS) <= 0.5 * cfg.prbsHoldS;
        excCheck.rows{end+1} = sprintf( ...
            '%-14s prbs: amp=%.3f/%.3f N*m %s, +lvl=%.0f%% -lvl=%.0f%%, hold=%.3f s (期望 %.3f s) %s', ...
            inputNames{ch}, ampEst, ampExp, ternary(ampOK, 'OK', 'MISMATCH'), ...
            100*lvlPos, 100*lvlNeg, holdEst, cfg.prbsHoldS, ...
            ternary(holdOK, 'OK', 'MISMATCH')); %#ok<AGROW>
        if ~holdOK
            excCheck.warnings{end+1} = sprintf('%s: PRBS 码元保持时间 %.3f s 与参数 %.3f s 不符。', ...
                inputNames{ch}, holdEst, cfg.prbsHoldS); %#ok<AGROW>
        end
        row = {inputNames{ch}, 'prbs', ampEst, ampExp, double(ampOK), ...
               lvlPos, lvlNeg, holdEst};
    end
    checkTable(end+1, :) = row; %#ok<AGROW>
end
excCheck.csv = cell2table(checkTable, 'VariableNames', ...
    {'channel', 'type', 'amp_est_Nm', 'amp_expected_Nm', 'amp_ok', ...
     'prbs_pos_level_frac', 'prbs_neg_level_frac', 'prbs_hold_s'});
end

function q = localPrctile(X, p)
% 无工具箱时的百分位（对每列做线性插值）
X = X(:);
q = zeros(1, size(X, 2));
for c = 1:size(X, 2)
    xs = sort(X(:, c));
    m = numel(xs);
    idx = (p/100) * (m - 1) + 1;
    lo = floor(idx); hi = ceil(idx);
    if lo < 1; lo = 1; end
    if hi > m; hi = m; end
    frac = idx - lo;
    q(c) = xs(lo) * (1 - frac) + xs(hi) * frac;
end
end

function writeCsv(T, fname)
writetable(T, fname, 'Encoding', 'UTF-8');
end

function s = names2struct(names, vals)
s = struct();
for i = 1:numel(names)
    fname = regexprep(names{i}, '[^A-Za-z0-9_]', '_');
    s.(fname) = vals(i);
end
end

function txt = vecToText(fmt, args)
% args 为 {names; vals} 二元 cell 或数值向量
if iscell(args) && numel(args) == 2 && iscell(args{1})
    names = args{1}; vals = args{2};
    parts = cell(1, numel(names));
    for i = 1:numel(names)
        parts{i} = sprintf(fmt, names{i}, vals(i));
    end
else
    parts = cellfun(@(v) sprintf(fmt, v), num2cell(args), 'UniformOutput', false);
end
txt = strjoin(parts, ' ');
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function savefigPng(fh, fname)
try
    exportgraphics(fh, fname, 'Resolution', 150);
catch
    print(fh, fname, '-dpng', '-r150');
end
close(fh);
end
