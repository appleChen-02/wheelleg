function analyze_sysid_no_excitation()
%ANALYZE_SYSID_NO_EXCITATION 系统辨识模式（无激励）闭环稳态数据分析
%
% 数据来源：上位机 usb_data_logger.py 采集的
%   - system_identification_12.csv          (0x10 帧, 12状态/5输入, 250Hz)
%   - system_identification_reference.csv   (0x12 参考配置帧, 1Hz)
%
% 前提约定（与固件 system_identification_param.h 对应）：
%   - profile_id = 1（12状态/5输入），双足模式 ConsoleBipedal /
%     LocomotionController_Pro_Bipedal 的 12 状态 LQR
%   - SYS_ID_EXCITATION_ENABLE = 0，excitation 通道应全零
%   - 记录内容已经是选定的稳态段：脚本不再做预热切除和稳态自动检测；
%     如需手动截取可设置 cfg.tRange = [t0 t1]（单位 s，相对文件首样本）
%   - 时间轴一律使用 control_tick_ms，不使用 host_time_s（仅作参考对比）
%   - uint32 回绕按约定不处理；若出现 tick 倒退会以 issue 形式暴露
%
% 输出（results/<时间戳>/ 目录）：
%   summary.csv / operating_point.csv / state_bias.csv /
%   state_statistics.csv / input_statistics.csv / drift_statistics.csv /
%   state_covariance.csv / input_covariance.csv / cross_covariance.csv /
%   k_check.csv / command_residual.csv / psd_band_rms.csv /
%   data_quality.json / fig_*.png，并在命令行打印最终结论
%
% 使用：把采集目录（含两个 csv）放到本脚本同目录的 data/ 下，
% 或将 cfg.dataDir 指向该目录，然后运行 analyze_sysid_no_excitation

clc; close all;

%% ============================== 配置 ==============================
cfg = struct();
cfg.dataDir  = 'C:\Users\0\Desktop\轮足\单片机\matlab\读取测试\test0910_1';        % '' = 自动搜索（脚本目录、脚本目录/data、上位机/log）
cfg.file12   = 'system_identification_12.csv';
cfg.fileRef  = 'system_identification_reference.csv';
cfg.outRoot  = 'results';

cfg.tRange   = [];        % 手动截取 [t0 t1] (s, 相对首样本)；[] = 全部使用

cfg.expectedProfileId = 1;
cfg.expectedStateDim  = 12;
cfg.expectedInputDim  = 5;
cfg.excitationAbsTol  = 1e-4;   % N*m，excitation 通道视为全零的阈值

cfg.nominalDt  = 0.004;         % 250Hz 采样帧周期
cfg.controlPeriodS = 0.002;     % 固件控制周期(2ms)：sequence 按该周期递增
cfg.dropFactor = 1.5;           % dt > 1.5*nominal 判为丢帧

cfg.psdWindowSec = 10;                        % Welch 窗长（s）
cfg.psdBands     = [0 1; 1 20; 20 Inf];       % 三个分析频段 (Hz)
bandLabels = {'0-1 Hz', '1-20 Hz', '20 Hz-Nyquist'};

cfg.legLength = 0.18;           % SYS_ID_LEG_LENGTH，K 矩阵插值工作点

% 名义参考工作点（system_identification_param.h 的 SYS_ID_REF_*，12维）
cfg.nominalRef = [-0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

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
    warning('未找到参考帧文件 %s，参考工作点/平衡输入相关检查将降级。', cfg.fileRef);
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

issues = {};   % 质量问题列表（warning 级；致命问题直接 error）

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
    issues{end+1} = sprintf('experiment_id 不唯一: %s（参考帧匹配取出现最多的 %d）。', ...
        mat2str(expIds), mode(T12.experiment_id)); %#ok<AGROW>
end
expIdMain = mode(T12.experiment_id);

% 激励校验：excitation_config bit0 = 使能位；同时检查 5 个 excitation 通道
excCfgVals = unique(T12.excitation_config);
excEnabled = any(bitand(T12.excitation_config, 1) == 1);
EXC = [T12.excitation_right_hip_tp, T12.excitation_left_hip_tp, ...
       T12.excitation_right_wheel_t, T12.excitation_left_wheel_t, T12.excitation_tail_t];
excMax = max(abs(EXC(:)));
if excEnabled
    issues{end+1} = sprintf('excitation_config=0x%s 的使能位为 1，激励处于使能状态！本数据不是无激励基线。', ...
        dec2hex(double(T12.excitation_config(1)))); %#ok<AGROW>
end
if excMax >= cfg.excitationAbsTol
    issues{end+1} = sprintf('excitation 通道最大绝对值 %.3e 超过阈值 %.1e，存在非零激励。', ...
        excMax, cfg.excitationAbsTol); %#ok<AGROW>
end

% status_flags 逐位统计
SF = T12.status_flags;
flagCounts = zeros(1, 8);
for b = 1:8
    flagCounts(b) = sum(bitand(SF, 2^(b-1)) ~= 0);
end
badStatusMask = bitand(SF, bitor(bitor(8, 16), 32)) ~= 0;  % OFFLINE|JAC_L|JAC_R
nBadStatus = sum(badStatusMask);
if nBadStatus > 0
    issues{end+1} = sprintf('%d/%d 帧带 MOTOR_OFFLINE 或 JACOBIAN_BAD 标志。', ...
        nBadStatus, height(T12)); %#ok<AGROW>
end
if any(bitand(SF, 2) ~= 0)
    issues{end+1} = '存在 EXCITATION_ACTIVE 标志帧，与"无激励"前提冲突。'; %#ok<AGROW>
end
nActualValid = sum(bitand(SF, 64) ~= 0);
nRefValid    = sum(bitand(SF, 128) ~= 0);

% 饱和掩码（bit0..6 = J0,J1,J2,J3,W0,W1,Tail）
SAT = T12.motor_saturation_mask;
nSat = sum(SAT ~= 0);
satCounts = zeros(1, 7);
for b = 1:7
    satCounts(b) = sum(bitand(SAT, 2^(b-1)) ~= 0);
end
if nSat > 0
    issues{end+1} = sprintf('%d/%d 帧出现电机输出饱和（%s）。', ...
        nSat, height(T12), vecToText(satNames, satCounts)); %#ok<AGROW>
end

%% ============================ 时间轴 ==============================
% 以 control_tick_ms 为主时间轴；host_time_s 仅作偏差参考
tick = double(T12.control_tick_ms);
tAll = (tick - tick(1)) / 1000;
seq  = double(T12.sequence);

dt = diff(tAll);
if any(dt <= 0)
    issues{end+1} = sprintf('control_tick_ms 出现 %d 次非递增（回绕或异常），请检查数据。', ...
        sum(dt <= 0)); %#ok<AGROW>
end
expectedFromDt = 1 + sum(max(round(max(dt, 0) / cfg.nominalDt), 1));
nLostSamples   = max(expectedFromDt - numel(tAll), 0);
dropRate = nLostSamples / max(expectedFromDt, 1);

dseq = diff(seq);
% 固件 sequence 按控制周期(2ms)递增，USB 帧按 250Hz 发送，相邻帧差为 2；
% 期望差值必须用控制周期 cfg.controlPeriodS 推算（dt/0.002），
% 不能用 USB 帧周期 nominalDt（dt/0.004 会得到 1，把正常差值 2 全部误报为缺帧）
expectedSeqStep = max(round(max(dt, 0) / cfg.controlPeriodS), 1);
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

% 组装分析矩阵（截取后）
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
S = S(useMask,:);  U = U(useMask,:);  C = C(useMask,:);  A = A(useMask,:);
MOTOR = MOTOR(useMask,:);  EXC = EXC(useMask,:);
SF = SF(useMask);  SAT = SAT(useMask);
Sraw = S;                    % 保留原始角（未 unwrap），仅用于绘图和圆周统计
n0 = nnz(useMask);

% NaN/Inf 剔除
finiteMask = all(isfinite([S U C A MOTOR]), 2);
nNonFinite = n0 - sum(finiteMask);
if nNonFinite > 0
    issues{end+1} = sprintf('%d 帧含 NaN/Inf，已从统计中剔除。', nNonFinite); %#ok<AGROW>
end

% 最终有效样本掩码：剔除 NaN/离线/雅可比异常/饱和
gMask = finiteMask & ~badStatusMask(useMask) & (SAT(useMask) == 0);
nGood = sum(gMask);
if nGood < 100
    error('有效样本仅 %d 帧（剔除非有限值/异常状态/饱和后），不足以做统计，终止。', nGood);
end
if nGood < n0
    issues{end+1} = sprintf('统计使用 %d/%d 帧（剔除异常状态/饱和/非有限值）。', nGood, n0); %#ok<AGROW>
end
tD = t(gMask);
nD = nGood;

%% ============================ 角度处理 ============================
% yaw 一律 unwrap；其余圆周量检查是否跨越 ±pi，跨越则同样 unwrap 并记录
angleIdx = [3 5 7 9 11];      % yaw, left_theta, right_theta, pitch, tail_beta
wrapEvents = zeros(1, 12);
for k = angleIdx
    d = diff(Sraw(gMask, k));
    wrapEvents(k) = sum(abs(d) > pi);
    if wrapEvents(k) > 0
        issues{end+1} = sprintf('%s 跨越 ±pi %d 次，已做 unwrap。', ...
            stateNames{k}, wrapEvents(k)); %#ok<AGROW>
    end
    S(:, k) = unwrap(Sraw(:, k));
end
Sg = S(gMask, :);

% 圆周均值（对原始角，与 unwrap 均值对照用）
circMean = nan(1, 12);
for k = angleIdx
    circMean(k) = angle(mean(exp(1i * Sraw(gMask, k))));
end

%% =========================== 参考帧 ===============================
ref12 = cfg.nominalRef;      % 兜底：名义参考（param 头文件）
equil5 = [0 0 0 0 0];
refSource = 'nominal (system_identification_param.h)';
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
        if isempty(refRows)
            refRows = Tref(end, :);
            issues{end+1} = sprintf('参考帧 experiment_id 与数据帧不匹配，使用最后一帧（id=%d）。', ...
                Tref.experiment_id(end)); %#ok<AGROW>
        end
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
        else
            issues{end+1} = '参考帧缺少 equilibrium_* 列，command_residual 将包含平衡项偏差。'; %#ok<AGROW>
        end
    else
        issues{end+1} = '参考帧 CSV 缺少 reference_* 列，使用名义参考值。'; %#ok<AGROW>
    end
end

%% =========================== 漂移分析 =============================
% 对 body_x、yaw（积分状态）及 body_x_dot、yaw_dot 拟合 z = a*t + b
driftIdx = [1 3 2 4];
drift = struct('name', {}, 'slope', {}, 'ciLo', {}, 'ciHi', {}, 'intercept', {}, ...
               'r2', {}, 'quadCoef', {}, 'quadSig', {}, 'verdict', {});
driftCurves = cell(1, numel(driftIdx));
for kk = 1:numel(driftIdx)
    ch = driftIdx(kk);
    z = Sg(:, ch);
    X1 = [tD, ones(nD, 1)];
    [b1, se1, res, r2] = olsFit(X1, z);
    tc  = tcrit975(nD - 2);
    ciLo = b1(1) - tc * se1(1);
    ciHi = b1(1) + tc * se1(1);
    % 残差二次项显著性：判断线性去趋势是否足够（显著则建议分段/二次趋势）
    tcC = tD - mean(tD);
    X2 = [tcC.^2, ones(nD, 1)];
    [b2, se2] = olsFit(X2, res);
    quadSig = abs(b2(1)) > 2 * se2(1);
    if quadSig
        verdict = 'residual has significant quadratic term: use piecewise/low-order trend';
    else
        verdict = 'linear detrend OK';
    end
    drift(kk) = struct('name', stateNames{ch}, 'slope', b1(1), 'ciLo', ciLo, 'ciHi', ciHi, ...
        'intercept', b1(2), 'r2', r2, 'quadCoef', b2(1), 'quadSig', double(quadSig), ...
        'verdict', verdict);
    driftCurves{kk} = struct('t', tD, 'z', z, 'fit', X1 * b1, 'name', stateNames{ch});
end

%% ============================ 工作点 ==============================
x_eq  = mean(Sg, 1);                    % 实测闭环状态工作点（12维）
u_eq  = mean(U(gMask, :), 1);           % LQR 输出偏置（5维）
uc_eq = mean(C(gMask, :), 1);           % 广义指令工作点
ua_eq = mean(A(gMask, :), 1);           % 广义实际力矩工作点
state_bias = x_eq - ref12;              % 实测工作点偏差

% 日志一致性检查：command = controller + equilibrium + excitation（应≈0）
% 固件中 generalized_command = LQR输出 + T0_eq + 激励，无激励时残差应≈数值零
resid = C(gMask, :) - U(gMask, :) - repmat(equil5, nD, 1) - EXC(gMask, :);
residRms = sqrt(mean(resid.^2, 1));
residMax = max(abs(resid), [], 1);

%% ======================== 噪声与稳态波动 ==========================
% 积分状态 body_x / yaw 用线性去趋势残差；其余直接去均值
Xd = Sg;
Xd(:, 1) = Sg(:, 1) - (drift(1).slope * tD + drift(1).intercept);
Xd(:, 3) = Sg(:, 3) - (drift(2).slope * tD + drift(2).intercept);
Xd = Xd - mean(Xd, 1);

Ud = U(gMask, :) - u_eq;
Cd = C(gMask, :) - uc_eq;
Ad = A(gMask, :) - ua_eq;

stateStats = statTable(Xd, Sg, stateNames, units12);
inputStats = [statTable(Ud, U(gMask,:), strcat('controller_', inputNames), ...
                        repmat({'N*m'}, 1, 5)); ...
              statTable(Cd, C(gMask,:), strcat('command_', inputNames), ...
                        repmat({'N*m'}, 1, 5)); ...
              statTable(Ad, A(gMask,:), strcat('actual_', inputNames), ...
                        repmat({'N*m'}, 1, 5))];

%% =========================== 多变量统计 ===========================
CovX  = cov(Xd);
CorrX = corrcoef(Xd);
CovU  = cov(Ud);
CorrU = corrcoef(Ud);
CovXU = (Xd' * Ud) / (nD - 1);

% LQR 一致性检验: Cov(u) ≈ Keff * Cov(error) * Keff'
% K 来自 chassis_balance_extras.c 的 K_coef_Bipedal（TAIL_VERSION=1），
% 在 SYS_ID_LEG_LENGTH 处插值；pitch/pitch_dot 两列（9/10）在
% CalcLQR_Pro_Tail 中以负号进入，等效增益需翻转符号。
K     = buildK(cfg.legLength, false);
Koff  = buildK(cfg.legLength, true);
Keff  = K;    Keff(:, [9 10]) = -K(:, [9 10]);
KeffOff = Koff;  KeffOff(:, [9 10]) = -Koff(:, [9 10]);

predU   = Keff    * CovX * Keff';
predUff = KeffOff * CovX * KeffOff';
relErrK    = norm(predU   - CovU, 'fro') / max(norm(CovU, 'fro'), eps);
relErrKoff = norm(predUff - CovU, 'fro') / max(norm(CovU, 'fro'), eps);
actStd = sqrt(diag(CovU));
kCheck = table(inputNames(:), actStd, sqrt(diag(predU)), ...
    sqrt(diag(predU)) ./ max(actStd, eps), ...
    sqrt(diag(predUff)) ./ max(actStd, eps), ...
    'VariableNames', {'channel','actual_std','pred_std_K','ratio_K','ratio_K_takeoff'});

% 平衡点系数对照（chassis_balance_extras.c 的 Offset/T0 Bipedal 系数）
thetaEqModel = thetaEqAt(cfg.legLength);
t0Model = t0EqAt(cfg.legLength);

%% ============================ 频域分析 ===========================
fs = 1 / median(diff(tD));
winLen = min(nD, max(64, round(fs * cfg.psdWindowSec)));
psdCh = [1 3 9 5 7 11];              % body_x yaw pitch Ltheta Rtheta tail_beta
nChU = size(Ud, 2);
Pxx = cell(1, numel(psdCh) + nChU);
psdLabels = cell(1, numel(psdCh) + nChU);
f = [];
for kk = 1:numel(psdCh)
    [Pxx{kk}, f] = welchPsd(Xd(:, psdCh(kk)), fs, winLen);
    psdLabels{kk} = stateNames{psdCh(kk)};
end
for kk = 1:nChU
    [Pxx{numel(psdCh)+kk}, f] = welchPsd(Ud(:, kk), fs, winLen);
    psdLabels{numel(psdCh)+kk} = ['controller_' inputNames{kk}];
end
bandRms = zeros(numel(Pxx), size(cfg.psdBands, 1));
df = f(2) - f(1);
for kk = 1:numel(Pxx)
    for bb = 1:size(cfg.psdBands, 1)
        sel = f >= cfg.psdBands(bb,1) & f < min(cfg.psdBands(bb,2), fs/2);
        bandRms(kk, bb) = sqrt(sum(Pxx{kk}(sel)) * df);
    end
end
bandTable = table(psdLabels(:), ...
    repmat(bandLabels(1), numel(Pxx), 1), bandRms(:,1), ...
    repmat(bandLabels(2), numel(Pxx), 1), bandRms(:,2), ...
    repmat(bandLabels(3), numel(Pxx), 1), bandRms(:,3), ...
    'VariableNames', {'channel','band1','rms1','band2','rms2','band3','rms3'});

%% ============================ 结果输出 ===========================
outDir = fullfile(cfg.outRoot, datestr(now, 'yyyymmdd_HHMMSS'));
if ~exist(outDir, 'dir'); mkdir(outDir); end
fprintf('[输出] 结果目录: %s\n', outDir);

% ---- summary.csv ----
summaryRows = {
    'data_dir', dataDir
    'file_12', cfg.file12
    'file_ref', ternary(hasRef, cfg.fileRef, '(missing)')
    'profile_id', num2str(profileId)
    'experiment_id', num2str(expIdMain)
    'experiment_id_all', mat2str(expIds)
    'n_samples_raw', num2str(height(T12))
    'n_samples_used', num2str(n0)
    'n_samples_good', num2str(nGood)
    'used_time_range_s', sprintf('[%.3f, %.3f]', t(1), t(end))
    'used_source', ternary(isempty(cfg.tRange), 'whole file (pre-selected steady segment)', 'cfg.tRange trim')
    'nominal_dt_s', num2str(cfg.nominalDt)
    'median_dt_s', num2str(median(diff(tD)))
    'fs_hz', num2str(fs)
    'lost_samples_from_tick', num2str(nLostSamples)
    'drop_rate', num2str(dropRate)
    'sequence_gaps', num2str(nSeqGap)
    'sequence_lost_frames', num2str(nSeqLost)
    'host_time_max_dev_s', num2str(hostDev)
    'excitation_config_values', mat2str(excCfgVals)
    'excitation_max_abs_Nm', num2str(excMax)
    'excitation_enabled_bit', num2str(double(excEnabled))
    'status_flags_counts', vecToText(flagNames, flagCounts)
    'motor_saturation_frames', num2str(nSat)
    'saturation_counts', vecToText(satNames, satCounts)
    'angle_wrap_events', vecToText(stateNames, wrapEvents)
    'ref_source', refSource
    'reference_sequence_used', num2str(refSeqUsed)
    'theta_eq_model_rad', num2str(thetaEqModel)
    't0_hip_model_Nm', num2str(t0Model(1))
    't0_tail_model_Nm', num2str(t0Model(2))
    'k_rel_err_normal', num2str(relErrK)
    'k_rel_err_takeoff', num2str(relErrKoff)
    'issues_count', num2str(numel(issues))
    };
summaryT = table(summaryRows(:,1), summaryRows(:,2), 'VariableNames', {'key','value'});
writeCsv(summaryT, fullfile(outDir, 'summary.csv'));

% ---- 工作点与偏差 ----
opT = table([stateNames(:); strcat('u_eq_', inputNames(:)); ...
             strcat('uc_eq_', inputNames(:)); strcat('ua_eq_', inputNames(:))], ...
            [x_eq(:); u_eq(:); uc_eq(:); ua_eq(:)], ...
            'VariableNames', {'channel','mean'});
writeCsv(opT, fullfile(outDir, 'operating_point.csv'));

biasT = table(stateNames(:), x_eq(:), ref12(:), state_bias(:), ...
    repmat({'reference = 0x12 frame if available, else nominal'}, 12, 1), ...
    'VariableNames', {'state','x_eq','reference','state_bias','ref_note'});
writeCsv(biasT, fullfile(outDir, 'state_bias.csv'));

% ---- 统计/漂移/协方差 ----
writeCsv(stateStats, fullfile(outDir, 'state_statistics.csv'));
writeCsv(inputStats, fullfile(outDir, 'input_statistics.csv'));
driftT = table({drift.name}.', [drift.slope]', [drift.ciLo]', [drift.ciHi]', ...
    [drift.intercept]', [drift.r2]', [drift.quadCoef]', [drift.quadSig]', {drift.verdict}.', ...
    'VariableNames', {'channel','slope_per_s','ci95_lo','ci95_hi','intercept', ...
                      'r2_linear','quad_coef','quad_significant','verdict'});
writeCsv(driftT, fullfile(outDir, 'drift_statistics.csv'));

writeMat(CovX,  stateNames, stateNames, fullfile(outDir, 'state_covariance.csv'));
writeMat(CovU,  inputNames, inputNames, fullfile(outDir, 'input_covariance.csv'));
writeMat(CovXU, stateNames, inputNames, fullfile(outDir, 'cross_covariance.csv'));
writeCsv(kCheck, fullfile(outDir, 'k_check.csv'));
writeCsv(bandTable, fullfile(outDir, 'psd_band_rms.csv'));

resT = table(strcat('residual_', inputNames(:)), residRms(:), residMax(:), ...
    'VariableNames', {'channel','rms','max'});
writeCsv(resT, fullfile(outDir, 'command_residual.csv'));

% ---- data_quality.json ----
dq = struct();
dq.data_dir = dataDir;
dq.n_samples_raw = height(T12);
dq.n_samples_used = n0;
dq.n_samples_good = nGood;
dq.used_time_range_s = [t(1) t(end)];
dq.median_dt_s = median(diff(tD));
dq.fs_hz = fs;
dq.lost_samples_from_tick = nLostSamples;
dq.drop_rate = dropRate;
dq.sequence_gaps = nSeqGap;
dq.sequence_lost_frames = nSeqLost;
dq.host_time_max_dev_s = hostDev;
dq.experiment_id = expIdMain;
dq.experiment_id_all = expIds(:)';
dq.excitation_config_values = excCfgVals(:)';
dq.excitation_max_abs = excMax;
dq.excitation_enabled_bit = double(excEnabled);
dq.status_flag_counts = names2struct(flagNames, flagCounts);
dq.motor_saturation_frames = nSat;
dq.saturation_counts = names2struct(satNames, satCounts);
dq.angle_wrap_events = names2struct(stateNames, wrapEvents);
dq.circmean = names2struct(stateNames, circMean);
dq.n_nonfinite_removed = nNonFinite;
dq.n_bad_status_frames = nBadStatus;
dq.flags_actual_input_valid = nActualValid;
dq.flags_reference_valid = nRefValid;
dq.reference = struct('source', refSource, 'reference_sequence', refSeqUsed, ...
    'state_reference', ref12, 'equilibrium_input', equil5, ...
    'theta_eq_model_rad', thetaEqModel, 't0_model_Nm', t0Model);
dq.k_check = struct('rel_err_normal', relErrK, 'rel_err_takeoff', relErrKoff, ...
    'leg_length', cfg.legLength);
dq.command_residual = struct('rms', residRms, 'max', residMax);
dq.issues = issues;
dqJson = jsonencode(dq, 'PrettyPrint', true);
fq = fopen(fullfile(outDir, 'data_quality.json'), 'w');
fwrite(fq, dqJson, 'char');
fclose(fq);

%% ============================== 绘图 ==============================
% 图1: 12 个状态时域（灰=剔除样本，蓝=有效样本）
fh1 = figure('Visible','off','Position',[40 40 1400 900]);
for k = 1:12
    subplot(4, 3, k);
    plot(t, Sraw(:,k), 'Color', [.75 .75 .75]); hold on;
    plot(tD, Sg(:,k), 'Color', [.1 .35 .75]);
    grid on;
    title(sprintf('%s [%s]', stateNames{k}, units12{k}), 'Interpreter','none');
    xlabel('t (s)');
end
sgtitle('State feedback (gray = excluded samples)', 'Interpreter', 'none');
savefigPng(fh1, fullfile(outDir, 'fig1_states_time.png'));

% 图2: 5 个输入通道（controller / command / actual）
fh2 = figure('Visible','off','Position',[60 60 1400 900]);
for k = 1:5
    subplot(3, 2, k);
    plot(tD, U(gMask,k), 'Color', [.1 .35 .75]); hold on;
    plot(tD, C(gMask,k), 'Color', [.85 .5 .1]);
    plot(tD, A(gMask,k), 'Color', [.2 .6 .2]);
    grid on;
    title(sprintf('%s [N*m]', inputNames{k}), 'Interpreter','none');
    xlabel('t (s)');
end
subplot(3, 2, 6); axis off;
text(0.1, 0.75, 'blue:   controller_output', 'Color', [.1 .35 .75]);
text(0.1, 0.50, 'orange: command',            'Color', [.85 .5 .1]);
text(0.1, 0.25, 'green:  actual',             'Color', [.2 .6 .2]);
sgtitle('Generalized torque: LQR output vs command vs actual', 'Interpreter', 'none');
savefigPng(fh2, fullfile(outDir, 'fig2_inputs_time.png'));

% 图3: 漂移拟合
fh3 = figure('Visible','off','Position',[80 80 1200 800]);
for kk = 1:numel(driftCurves)
    dc = driftCurves{kk};
    subplot(2, 2, kk);
    plot(dc.t, dc.z, 'Color', [.6 .6 .6]); hold on;
    plot(dc.t, dc.fit, 'r', 'LineWidth', 1.2);
    grid on;
    title(sprintf('%s: slope=%.3e [%.3e, %.3e]/s', dc.name, ...
        drift(kk).slope, drift(kk).ciLo, drift(kk).ciHi), 'Interpreter','none');
    xlabel('t (s)');
end
sgtitle('Integral state drift: raw curve + linear trend', 'Interpreter', 'none');
savefigPng(fh3, fullfile(outDir, 'fig3_drift.png'));

% 图4: PSD
fh4 = figure('Visible','off','Position',[100 100 1200 800]);
subplot(2, 1, 1);
for kk = 1:numel(psdCh)
    loglog(f(2:end), Pxx{kk}(2:end)); hold on;
end
xline(cfg.psdBands(1,2)); xline(cfg.psdBands(2,2));
grid on;
ylabel('PSD'); title(sprintf('State PSD (fs=%.1f Hz)', fs));
legend(psdLabels(1:numel(psdCh)), 'Interpreter','none', 'Location','best');
subplot(2, 1, 2);
for kk = 1:nChU
    loglog(f(2:end), Pxx{numel(psdCh)+kk}(2:end)); hold on;
end
xline(cfg.psdBands(1,2)); xline(cfg.psdBands(2,2));
grid on;
ylabel('PSD'); xlabel('f (Hz)');
title('LQR output PSD');
legend(psdLabels(numel(psdCh)+1:end), 'Interpreter','none', 'Location','best');
savefigPng(fh4, fullfile(outDir, 'fig4_psd.png'));

% 图5: 相关矩阵
fh5 = figure('Visible','off','Position',[120 120 1400 500]);
subplot(1, 3, 1);
imagesc(CorrX, [-1 1]); colorbar; axis square;
xticks(1:12); xticklabels(stateNames); xtickangle(90);
yticks(1:12); yticklabels(stateNames);
title('Corr(state)');
subplot(1, 3, 2);
imagesc(CorrU, [-1 1]); colorbar; axis square;
xticks(1:5); xticklabels(inputNames); xtickangle(90);
yticks(1:5); yticklabels(inputNames);
title('Corr(u lqr)');
subplot(1, 3, 3);
cl = max(abs(CovXU(:)));
imagesc(CovXU, [-cl cl]); colorbar; axis square;
xticks(1:5); xticklabels(inputNames); xtickangle(90);
yticks(1:12); yticklabels(stateNames);
title('Cov(state, u lqr)');
savefigPng(fh5, fullfile(outDir, 'fig5_correlation.png'));

%% ========================== 最终结论打印 ==========================
fprintf('\n');
fprintf('================ 系统辨识无激励基线分析结论 ================\n');
fprintf('数据: %s, 有效 %d 帧 (%.1f Hz), 使用区间 [%.2f, %.2f] s\n', ...
    dataDir, nGood, fs, t(1), t(end));
fprintf('丢帧率 %.3f%% (tick), sequence 缺帧 %d\n', 100*dropRate, nSeqLost);
fprintf('\n--- 1) 名义参考工作点 vs 实测闭环工作点 ---\n');
fprintf('%-16s %12s %12s %12s %10s\n', 'state', 'x_eq', 'reference', 'bias', 'unit');
for k = 1:12
    fprintf('%-16s %12.5f %12.5f %12.5f %10s\n', ...
        stateNames{k}, x_eq(k), ref12(k), state_bias(k), units12{k});
end
fprintf('\n--- 2) 积分状态漂移率 ---\n');
for kk = 1:numel(drift)
    fprintf('%-12s 斜率 %+.3e  95%%CI [%+.3e, %+.3e] /s, R2=%.3f, %s\n', ...
        drift(kk).name, drift(kk).slope, drift(kk).ciLo, drift(kk).ciHi, ...
        drift(kk).r2, drift(kk).verdict);
end
fprintf('\n--- 3) 闭环稳态波动（样本标准差, body_x/yaw 为去趋势残差）---\n');
for k = 1:12
    fprintf('%-16s std=%.5f %-5s  AC_RMS=%.5f  P99-P1=%.5f\n', ...
        stateNames{k}, stateStats.std_sample(k), units12{k}, ...
        stateStats.ac_rms(k), stateStats.p99_p1(k));
end
fprintf('\n--- 4) LQR 输出偏置与波动 ---\n');
for k = 1:5
    fprintf('%-16s 偏置=%+.4f Nm, std=%.4f Nm (command std=%.4f, actual std=%.4f)\n', ...
        inputNames{k}, u_eq(k), sqrt(CovU(k,k)), sqrt(cov(Cd(:,k))), sqrt(cov(Ad(:,k))));
end
fprintf('\n--- 5) 日志一致性: command - controller - equilibrium - excitation ---\n');
for k = 1:5
    fprintf('%-16s RMS=%.2e Nm, max=%.2e Nm %s\n', inputNames{k}, ...
        residRms(k), residMax(k), ternary(residMax(k) < 1e-3, '(OK)', '(偏离数值零, 检查限幅/日志定义)'));
end
fprintf('\n--- 6) Cov(u_lqr) ≈ K*Cov(error)*K'' 一致性 (leg=%.2f m) ---\n', cfg.legLength);
for k = 1:5
    fprintf('%-16s 实测std=%.4f, K预测std=%.4f (比例 %.2f), 离地K预测比例 %.2f\n', ...
        inputNames{k}, actStd(k), sqrt(predU(k,k)), sqrt(predU(k,k))/max(actStd(k),eps), ...
        sqrt(predUff(k,k))/max(actStd(k),eps));
end
fprintf('整体相对误差: 正常K=%.1f%%, 离地K=%.1f%%; 差异大通常意味着限幅/附加控制器/日志定义不一致\n', ...
    100*relErrK, 100*relErrKoff);
fprintf('\n--- 7) 平衡点对照 (模型系数 @ l=%.2f m) ---\n', cfg.legLength);
fprintf('theta_eq(模型)=%.4f rad, 参考帧 equilibrium_hip=%.4f Nm (模型 %.4f), tail=%.4f Nm (模型 %.4f)\n', ...
    thetaEqModel, equil5(1), t0Model(1), equil5(5), t0Model(2));
fprintf('\n--- 8) 注意事项 ---\n');
fprintf('本套无激励分析可作为后续激励实验的基线，但不能单独估计纯传感器测量噪声；\n');
fprintf('如需估计纯测量噪声，请补充一组"机器人固定 + 执行器关闭/机械锁止"的对照数据。\n');
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

function [beta, seb, res, r2] = olsFit(X, y)
% 普通最小二乘: 返回系数、各系数标准误、残差、R^2
beta = X \ y;
res = y - X * beta;
n = size(X, 1);
p = size(X, 2);
s2 = (res' * res) / max(n - p, 1);
covb = s2 * pinv(X' * X);
seb = sqrt(max(diag(covb), 0));
ssTot = sum((y - mean(y)).^2);
r2 = 1 - (res' * res) / max(ssTot, eps);
end

function tc = tcrit975(df)
% t 分布 0.975 分位数；无统计工具箱时用大样本近似
if df > 0 && exist('tinv', 'file') == 2
    tc = tinv(0.975, df);
else
    tc = 1.96;
end
end

function T = statTable(P, R, names, units)
% P: 预处理后（去均值/去趋势）矩阵；R: 原始矩阵
% 标准差一律为样本标准差 (ddof=1)
n = size(P, 1);
meanv  = mean(R, 1)';
medianv = median(R, 1)';
stdv   = std(P, 0, 1)';
acRms  = sqrt(mean(P.^2, 1))';
rawRms = sqrt(mean(R.^2, 1))';
p2p    = (max(P, [], 1) - min(P, [], 1))';
p99p1  = (localPrctile(P, 99) - localPrctile(P, 1))';
madv   = median(abs(P - median(P, 1)), 1)';
minv   = min(P, [], 1)';
maxv   = max(P, [], 1)';
T = table(names(:), units(:), repmat(n, numel(names), 1), ...
    meanv, medianv, stdv, acRms, rawRms, p2p, p99p1, madv, minv, maxv, ...
    'VariableNames', {'channel','unit','n','mean','median','std_sample', ...
                      'ac_rms','raw_rms','peak_to_peak','p99_p1','mad','min','max'});
end

function q = localPrctile(X, p)
% 无工具箱时的百分位（对每列做线性插值）
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
% 写 CSV（UTF-8）
writetable(T, fname, 'Encoding', 'UTF-8');
end

function writeMat(M, rowNames, colNames, fname)
% 逐列展开成 table，保证 CSV 列头正确
T = table(rowNames(:), 'VariableNames', {'name'});
for c = 1:size(M, 2)
    T.(colNames{c}) = M(:, c);
end
writeCsv(T, fname);
end

function s = names2struct(names, vals)
s = struct();
for i = 1:numel(names)
    fname = regexprep(names{i}, '[^A-Za-z0-9_]', '_');
    s.(fname) = vals(i);
end
end

function txt = vecToText(names, vals)
parts = cell(1, numel(names));
for i = 1:numel(names)
    parts{i} = sprintf('%s=%d', names{i}, vals(i));
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

function K = buildK(l, isTakeOff)
% K_coef_Bipedal（TAIL_VERSION=1）来自 chassis_balance_extras.c
% K_n = p1 + p2*l_l + p3*l_r + p4*l_l^2 + p5*l_l*l_r + p6*l_r^2
% 行: 1=右髋Tp 2=左髋Tp 3=右轮T 4=左轮T 5=尾Tt；列: 12状态误差
Kcoef = [ ...
    -7.69121,     34.1657,    -13.0083,    -26.6936,    -37.1583,     44.7063;
    -22.2912,     98.8947,    -26.1953,     -74.363,    -122.286,     116.432;
    -9.82062,    -26.9227,     -56.037,     42.0536,     -57.446,     90.8396;
    -1.25073,    -1.66179,    -12.2444,     0.26186,    -4.97422,     16.0727;
     1.86587,     16.4443,     41.3567,    -13.8793,     35.3998,    -45.6473;
  -0.0120036,     1.23822,     2.45042,    0.0429883,    -1.23274,    -1.60082;
    -48.0955,     7.56688,     44.1354,      -15.704,    -37.9995,     1.33148;
    -2.76632,     0.572425,     4.34901,     -1.15136,    -1.57923,    -4.47867;
     32.1994,    -27.4829,     121.614,      12.557,     -18.884,    -117.651;
    0.479561,     1.91591,     5.63873,     -1.95413,    -6.09211,    -1.99342;
     1.65203,    -2.91606,     8.09073,      2.49341,    -2.79406,    -5.90478;
  -0.0227486,     0.920985,     1.53882,    -0.864576,     -2.3731,   -0.109468;
    -7.69121,    -13.0083,     34.1657,     44.7063,    -37.1583,    -26.6936;
    -22.2912,    -26.1953,     98.8947,     116.432,    -122.286,     -74.363;
    9.82062,      56.037,     26.9227,     -90.8396,     57.446,     -42.0536;
    1.25073,     12.2444,     1.66179,     -16.0727,     4.97422,    -0.26186;
    -48.0955,     44.1354,     7.56688,      1.33148,    -37.9995,    -15.704;
    -2.76632,     4.34901,     0.572425,    -4.47867,    -1.57923,    -1.15136;
    1.86587,      41.3567,     16.4443,     -45.6473,     35.3998,    -13.8793;
  -0.0120036,     2.45042,     1.23822,     -1.60082,    -1.23274,    0.0429883;
    32.1994,     121.614,     -27.4829,    -117.651,     -18.884,     12.557;
    0.479561,     5.63873,     1.91591,     -1.99342,    -6.09211,    -1.95413;
    1.65203,      8.09073,    -2.91606,     -5.90478,    -2.79406,     2.49341;
  -0.0227486,     1.53882,     0.920985,    -0.109468,     -2.3731,   -0.864576;
    0.949593,     -4.01736,     10.1595,      3.35388,    0.184543,   -11.9639;
    3.27707,     -13.2102,     24.5198,       8.9654,     8.78357,    -33.6662;
   -12.8755,     -18.7749,     28.3428,      22.9898,     3.31849,    -27.2187;
   -1.70363,     -4.54028,     5.04443,      5.99638,    -1.07701,     -3.0655;
    3.59757,      14.1722,    -0.881715,     -2.14798,    -1.24472,    -13.8333;
    0.4173,     0.00588123,   -0.0304268,     1.69146,     1.0199,     -1.01673;
    6.87829,     -10.7342,     49.6134,      11.2583,    -7.36869,    -39.1467;
    0.586485,    -0.830777,     1.9412,      0.65299,     0.492003,   -0.103434;
    11.0536,     -18.4572,     -8.7737,      18.7122,     13.6328,    -14.4018;
    1.08845,     -2.30585,    -0.257286,     2.03758,     1.77641,     -1.511;
    1.626,       -3.12078,     -1.0815,      2.738,       2.32552,    -1.16344;
    0.402964,    -0.900548,    -0.137939,    0.779487,    0.714547,   -0.432484;
    0.949593,     10.1595,     -4.01736,    -11.9639,     0.184543,    3.35388;
    3.27707,      24.5198,    -13.2102,     -33.6662,     8.78357,     8.9654;
    12.8755,     -28.3428,     18.7749,      27.2187,    -3.31849,    -22.9898;
    1.70363,     -5.04443,     4.54028,      3.0655,      1.07701,    -5.99638;
    6.87829,      49.6134,    -10.7342,     -39.1467,    -7.36869,     11.2583;
    0.586485,     1.9412,     -0.830777,    -0.103434,    0.492003,    0.65299;
    3.59757,     -0.881715,    14.1722,     -13.8333,    -1.24472,    -2.14798;
    0.4173,     -0.0304268,   0.00588123,   -1.01673,     1.0199,      1.69146;
    11.0536,      -8.7737,    -18.4572,     -14.4018,     13.6328,     18.7122;
    1.08845,     -0.257286,    -2.30585,     -1.511,      1.77641,     2.03758;
    1.626,       -1.0815,     -3.12078,     -1.16344,     2.32552,     2.738;
    0.402964,    -0.137939,    -0.900548,   -0.432484,    0.714547,    0.779487;
    0.712792,    -0.302708,   -0.302708,    -0.875097,    1.21945,    -0.875097;
    2.09691,     -1.75297,    -1.75297,     -2.2752,      6.30632,    -2.2752;
   -9.85347e-12, -3.31079,     3.31079,     5.64841,   -1.07581e-11, -5.64841;
   -1.5745e-12,  -0.739633,    0.739633,    0.994938,   -1.25026e-12, -0.994938;
    2.56228,      2.0858,     -4.93812,     -4.79944,    1.5661,      3.26878;
    0.180309,    -0.0790601,  -0.308406,     0.0586868,  0.236778,    0.167071;
    2.56228,     -4.93812,     2.0858,       3.26878,    1.5661,     -4.79944;
    0.180309,    -0.308406,   -0.0790601,    0.167071,   0.236778,    0.0586868;
    12.1293,     -12.3115,    -12.3115,     11.2491,     10.2684,     11.2491;
    0.559952,    -0.785471,   -0.785471,    0.572642,    0.907656,    0.572642;
    -5.3191,     -1.16875,    -1.16875,     0.97677,     1.16785,     0.97677;
   -0.855165,    -0.233108,   -0.233108,    0.157056,    0.300975,    0.157056];
K = zeros(5, 12);
for nRow = 1:60
    r = ceil(nRow / 12);
    c = mod(nRow - 1, 12) + 1;
    p = Kcoef(nRow, :);
    K(r, c) = p(1) + p(2)*l + p(3)*l + p(4)*l^2 + p(5)*l^2 + p(6)*l^2;
end
if isTakeOff
    % GetK_Pro_Bipedal 的 is_take_off 分支（行0/1仅保留列4..7；行2/3全零；
    % 行4清零列0..3和8..9）
    K(1:2, [1:4 9:12]) = 0;
    K(3:4, :) = 0;
    K(5, [1:4 9 10]) = 0;
end
end

function thetaEq = thetaEqAt(l)
% Offset_coef_Bipedal（l_l = l_r = l）: theta_l_eq 与 theta_r_eq 相同
thetaEq = 0.867123 + (-4.62177) * l + 7.26101 * l^2;
end

function T0 = t0EqAt(l)
% T0_coef_Bipedal（l_l = l_r = l）: [Tp_eq, Tt_eq]
Tp = 1.01133 + (-2.47908e-14)*2*l + (3.25579e-14 + 4.02904e-14 + 3.25579e-14) * l^2;
Tt = -1.44252 + (-6.19769e-15)*2*l + (8.13946e-15 + 1.00726e-14 + 8.13946e-15) * l^2;
T0 = [Tp, Tt];
end

function [Pxx, f] = welchPsd(x, fs, winLen)
% Welch 谱估计（Hann 窗, 50% 重叠, 分段去均值），单边谱，不依赖工具箱
x = x(:);
N = min(winLen, numel(x));
if mod(N, 2) == 1; N = N - 1; end
w = 0.5 * (1 - cos(2 * pi * (0:N-1)' / (N - 1)));   % hann 窗
Uw = sum(w.^2);
Nfft = 2 ^ nextpow2(N);
step = max(floor(N / 2), 1);
nseg = max(floor((numel(x) - N) / step) + 1, 1);
acc = zeros(Nfft/2 + 1, 1);
for s = 1:nseg
    seg = x((s-1)*step + (1:N));
    seg = seg - mean(seg);
    Xf = fft(seg .* w, Nfft);
    P = abs(Xf).^2 / (fs * Uw);
    P = P(1:Nfft/2+1);
    P(2:end-1) = 2 * P(2:end-1);   % 单边谱补偿
    acc = acc + P;
end
Pxx = acc / nseg;
f = (0:Nfft/2)' * fs / Nfft;
end

function savefigPng(fh, fname)
try
    exportgraphics(fh, fname, 'Resolution', 150);
catch
    print(fh, fname, '-dpng', '-r150');
end
close(fh);
end
