% compute_lqr_0821.m
% LQR calculation for dynamic0821.py.
% Python supplies the corrected dynamics, equilibrium, and legacy-ordered
% state-space matrices. This script selects Lag/NE and computes LQR.

clear; clc;

%% Configuration
input_file = 'lqr_input_0821.mat';
model_choice = 'Lag';       % 'Lag' or 'NE'
enable_fitting = false;     % requires Python-generated K_sample_2d
fit_file = 'lqr_fit_samples_0821_Lag.mat';

lqr_Q = diag([120, 400, 4000, 1, 2000, 10, 2000, 10, 6000, 1, 300, 5]);
lqr_R = diag([1, 1, 12, 12, 8]);

%% Load/select model
if ~isfile(input_file)
    error('找不到 %s。请先运行 dynamic0821.py。', input_file);
end
data = load(input_file);
switch lower(model_choice)
    case 'lag'
        if ~isfield(data, 'A_Lag_int') || ~isfield(data, 'B_ss_Lag_int')
            error('输入文件缺少 Lagrange 的 A_Lag_int/B_ss_Lag_int。请重新运行 dynamic0821.py。');
        end
        A_num = data.A_Lag_int;
        B_num = data.B_ss_Lag_int;
        model_name = 'Lagrange';
    case 'ne'
        if ~isfield(data, 'A_NE_int') || ~isfield(data, 'B_ss_NE_int')
            error('输入文件缺少 NE 的 A_NE_int/B_ss_NE_int。请重新运行 dynamic0821.py。');
        end
        A_num = data.A_NE_int;
        B_num = data.B_ss_NE_int;
        model_name = 'Newton-Euler';
    otherwise
        error('model_choice 必须为 ''Lag'' 或 ''NE''。');
end
if ~isequal(size(A_num), [12, 12]) || ~isequal(size(B_num), [12, 5])
    error('A/B 尺寸错误：A 应为 12x12，B 应为 12x5。');
end
if isfield(data, 'X_eq'), X_eq = data.X_eq(:); else, X_eq = zeros(12,1); end
if isfield(data, 'U_eq'), U_eq = data.U_eq(:); else, U_eq = zeros(5,1); end
if numel(X_eq) ~= 12 || numel(U_eq) ~= 5
    error('X_eq/U_eq 尺寸错误。');
end
if any(~isfinite(A_num(:))) || any(~isfinite(B_num(:)))
    error('A/B 含 NaN 或 Inf。');
end

fprintf('模型: %s\n', model_name);
fprintf('状态顺序: [X,dX,phi,dphi,theta_l,dtheta_l,theta_r,dtheta_r,theta_b,dtheta_b,theta_t,dtheta_t]\n');
fprintf('可控性秩: %d/12\n', rank(ctrb(A_num, B_num)));

%% LQR
[K, S, poles] = lqr(A_num, B_num, lqr_Q, lqr_R);
fprintf('K 尺寸: %dx%d，闭环极点最大实部: %.6e\n', ...
        size(K,1), size(K,2), max(real(poles)));
disp(K);

%% Firmware output
fprintf('// model: %s\n', model_name);
fprintf('// control law: U = U_eq - K * (X - X_eq)\n');
fprintf('float K[5][12] = {\n');
for i = 1:5
    fprintf('    {');
    for j = 1:12
        if j < 12, fprintf('%.9gf, ', K(i,j));
        else, fprintf('%.9gf', K(i,j)); end
    end
    if i < 5, fprintf('},\n'); else, fprintf('}\n'); end
end
fprintf('};\n');
fprintf('X_eq and U_eq are saved in the MAT file.\n');

%% Optional 60x6 polynomial fit
% One linearization cannot determine a 60x6 table. The table is generated
% only from K_sample_2d exported by the same Python model.
K_Fit_Coefficients = [];
Offset_Fit_Coefficients = [];
InputEq_Fit_Coefficients = [];
if enable_fitting
    if ~strcmpi(model_choice, 'Lag')
        error('60x6 拟合当前只允许使用 Lagrange 扫描结果。');
    end
    fit_data = load(fit_file);
    fields = {'K_sample_2d', 'offset_sample_2d', 'input_eq_sample_2d'};
    for k = 1:numel(fields)
        if ~isfield(fit_data, fields{k})
            error('enable_fitting=true 需要字段 %s。', fields{k});
        end
    end
    K_samples = fit_data.K_sample_2d;
    offset_samples = fit_data.offset_sample_2d;
    input_samples = fit_data.input_eq_sample_2d;
    valid = all(isfinite(K_samples),2) & all(isfinite(offset_samples),2) & all(isfinite(input_samples),2);
    K_samples = K_samples(valid,:);
    offset_samples = offset_samples(valid,:);
    input_samples = input_samples(valid,:);
    if size(K_samples,1) < 20, error('有效拟合样本少于 20。'); end
    ll = K_samples(:,1); lr = K_samples(:,2);
    Phi = [ones(size(ll)), ll, lr, ll.^2, ll.*lr, lr.^2];
    if size(input_samples,2) ~= 7
        error('新模型 input_eq_sample_2d 必须为 N x 7。');
    end
    coeff = Phi \ [K_samples(:,3:62), offset_samples(:,3:6), input_samples(:,3:7)];
    K_Fit_Coefficients = coeff(:,1:60)';
    Offset_Fit_Coefficients = coeff(:,61:64)';
    InputEq_Fit_Coefficients = coeff(:,65:69)';
    fprintf('K_Fit_Coefficients 尺寸: %dx%d\n', size(K_Fit_Coefficients,1), size(K_Fit_Coefficients,2));
    fprintf('InputEq_Fit_Coefficients 尺寸: %dx%d\n', size(InputEq_Fit_Coefficients,1), size(InputEq_Fit_Coefficients,2));
    fprintf('float K_Fit_Coefficients[60][6] = {\n');
    for n = 1:60
        fprintf('    {');
        for c = 1:6
            if c < 6, fprintf('%.9gf, ', K_Fit_Coefficients(n,c));
            else, fprintf('%.9gf', K_Fit_Coefficients(n,c)); end
        end
        if n < 60, fprintf('},\n'); else, fprintf('}\n'); end
    end
    fprintf('};\n');
    fprintf('float InputEq_Fit_Coefficients[5][6] = {\n');
    for n = 1:5
        fprintf('    {');
        for c = 1:6
            if c < 6, fprintf('%.9gf, ', InputEq_Fit_Coefficients(n,c));
            else, fprintf('%.9gf', InputEq_Fit_Coefficients(n,c)); end
        end
        if n < 5, fprintf('},\n'); else, fprintf('}\n'); end
    end
    fprintf('};\n');
end

%% Save
save('lqr_linear_model_0821.mat', 'A_num', 'B_num', 'K', 'S', 'poles', ...
     'X_eq', 'U_eq', 'lqr_Q', 'lqr_R', 'model_name', ...
     'K_Fit_Coefficients', 'Offset_Fit_Coefficients', 'InputEq_Fit_Coefficients');
fprintf('已保存 lqr_linear_model_0821.mat\n');
