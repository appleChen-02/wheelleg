% fit_lqr_corrected.m
% -------------------------------------------------------------------------
% 基于修正后的动力学模型，拟合：
%   K_Fit_Coefficients       : 60x6
%   Offset_Fit_Coefficients   : 4x6
%   InputEq_Fit_Coefficients  : 2x6
%
% 每个 (l_l,l_r) 网格点都重新求解静态平衡点、完整线性化并计算 LQR。
% 二维二次多项式的系数顺序为：
%   [p00, p10, p01, p20, p11, p02]
%
% K 的 60 行顺序与原版 C 输出一致：
%   K[0][0..11], K[1][0..11], ..., K[4][0..11]
%
% 注意：本文件生成的是新版修正模型的系数，不能与原版系数混用。
% -------------------------------------------------------------------------

clear; clc; tic;
fprintf('开始拟合修正模型的 K、平衡角和静态输入...\n');
load('corrected_dynamics.mat');

%% 与模型一致的符号变量
syms g R R_w real
syms m_b I_b l_body_com theta_b0 real
syms m_l m_r I_l I_r l_l l_r l_leg_l_com l_leg_r_com real
syms theta_l0 theta_r0 real
syms m_wl m_wr I_wl I_wr real
syms m_t I_t l_tail_com delta_t l_tail_mount_h l_tail_mount_v real
syms I_yaw real
syms X_b_h phi theta_l theta_r theta_b theta_t real
syms dX_b_h dphi dtheta_l dtheta_r dtheta_b dtheta_t real

q = [X_b_h; phi; theta_l; theta_r; theta_b; theta_t];
dq = [dX_b_h; dphi; dtheta_l; dtheta_r; dtheta_b; dtheta_t];
param_vector = [g; R; R_w; m_b; I_b; l_body_com; theta_b0; ...
    m_l; m_r; I_l; I_r; l_l; l_r; l_leg_l_com; l_leg_r_com; ...
    theta_l0; theta_r0; m_wl; m_wr; I_wl; I_wr; m_t; I_t; ...
    l_tail_com; delta_t; l_tail_mount_h; l_tail_mount_v; I_yaw];

%% 新版默认物理参数
% 这里的重力和角偏置符号与 compare_dynamics_dynamics_corrected_s.py 一致。
base_param = [ ...
    9.81; 0.130; 0.386/2; ...
    6.9; 59035.925e-6; 4.3e-3; 0; ...
    2.2; 2.2; 0.034231929; 0.034231929; 0.17; 0.17; ...
    0.10157; 0.10157; -0.582108261; -0.582108261; ...
    0.823; 0.823; 6311.798e-6; 6311.798e-6; ...
    0.83; 29341.743e-6; 0.17755; 0.06597; ...
    0.07125; 0.1105; 294272.34e-6];

if numel(param_vector) ~= numel(base_param)
    error('参数向量和参数数值长度不一致。');
end

%% 腿长相关参数函数
% 这些函数沿用原拟合代码中的机械参数拟合关系；角度符号按新版模型取反。
theta0_fun = @(x) 277.69*x.^4 - 304.33*x.^3 + 130.52*x.^2 - 27.562*x + 2.7591;
ld_fun     = @(x) 1.4205*x.^4 - 2.0644*x.^3 + 1.2526*x.^2 - 0.059*x + 0.0844;
Ileg_fun   = @(x) -0.3588*x.^4 + 0.3595*x.^3 + 0.0659*x.^2 + 0.0318*x + 0.0255;
Iyaw_fun   = @(x) -6.4624*x.^4 + 6.7339*x.^3 - 3.0029*x.^2 + 0.5202*x + 0.2758;

%% 生成数值动力学函数
% 显式函数返回 M(q)、g(q,dq)、B(q)。参数作为最后一个向量输入，
% 可避免在 961 个网格点内重复进行符号 subs。
dyn_fun = matlabFunction(M_lag_sym,g_lag_sym,B_lag_sym,'Vars', ...
    {q,dq,param_vector});

%% LQR 权重和扫描范围
lqr_Q = diag([120,400,4000,1,2000,10,2000,10,6000,1,300,5]);
lqr_R = diag([1,1,12,12,8]);
l_range = 0.15:0.005:0.30;
num_legs = numel(l_range);
sample_count = num_legs^2;

% 每行前两列为腿长，后面分别保存 60 个 K 元素、4 个平衡角、2 个平衡输入。
K_sample_2d = nan(sample_count,62);
offset_sample_2d = nan(sample_count,6);
input_eq_sample_2d = nan(sample_count,4);

% 使用相邻左腿长度的解作为下一次初值，可显著减少 fsolve 迭代次数。
default_seed = [0.02; 0.02; 1.0; -1.44];
seed_by_right_length = repmat(default_seed,1,num_legs);
relative_step = 1e-5;

if exist('fsolve','file') ~= 2
    error('需要 Optimization Toolbox 中的 fsolve。');
end
if exist('lqr','file') ~= 2
    error('需要 Control System Toolbox 中的 lqr。');
end
solver_options = optimoptions('fsolve','Display','off', ...
    'FunctionTolerance',1e-10,'StepTolerance',1e-10, ...
    'MaxIterations',400,'MaxFunctionEvaluations',2000);

%% 扫描所有左右腿长组合
idx = 0;
for i = 1:num_legs
    l_l_fit = l_range(i);
    for j = 1:num_legs
        idx = idx + 1;
        l_r_fit = l_range(j);
        p = base_param;

        % 参数向量索引：l_l=12, l_r=13, l_leg_l_com=14,
        % l_leg_r_com=15, theta_l0=16, theta_r0=17, I_l=10, I_r=11, I_yaw=28。
        p(10) = Ileg_fun(l_l_fit);
        p(11) = Ileg_fun(l_r_fit);
        p(12) = l_l_fit;
        p(13) = l_r_fit;
        p(14) = ld_fun(l_l_fit);
        p(15) = ld_fun(l_r_fit);
        p(16) = -theta0_fun(l_l_fit);
        p(17) = -theta0_fun(l_r_fit);
        p(28) = Iyaw_fun((l_l_fit+l_r_fit)/2);

        try
            % 静态状态：X_b_h=phi=theta_b=theta_t=0，速度全为零。
            q_static = @(z) [0; 0; z(1); z(2); 0; 0];
            dq_zero = zeros(6,1);
            residual = @(z) equilibrium_residual(dyn_fun,q_static(z), ...
                dq_zero,[z(3);z(3);0;0;z(4)],p);

            % 第一个左右腿长度使用默认初值，后续使用上一列的解。
            seed = seed_by_right_length(:,j);
            [z_eq,fval,exitflag] = fsolve(residual,seed,solver_options);
            if exitflag <= 0 || any(~isfinite(z_eq)) || norm(fval,inf) > 1e-8
                % 失败时再尝试一个固定的通用初值。
                [z_eq,fval,exitflag] = fsolve(residual,default_seed,solver_options);
            end
            if exitflag <= 0 || any(~isfinite(z_eq)) || norm(fval,inf) > 1e-8
                error('平衡点求解失败，残差 %.3e。',norm(fval,inf));
            end
            seed_by_right_length(:,j) = z_eq;

            q_eq = q_static(z_eq);
            u_eq = [z_eq(3);z_eq(3);0;0;z_eq(4)];

            % 完整显式动力学：f=[dq; M\(-g+B*u)]。
            f_state = @(x) [x(7:12); explicit_acceleration( ...
                dyn_fun,x(1:6),x(7:12),u_eq,p)];
            x_eq = [q_eq; dq_zero];
            A_fit = zeros(12,12);
            A_fit(1:6,7:12) = eye(6);
            h = relative_step*max(1,abs(x_eq));
            for col_index = 1:12
                x_plus = x_eq; x_minus = x_eq;
                x_plus(col_index) = x_plus(col_index)+h(col_index);
                x_minus(col_index) = x_minus(col_index)-h(col_index);
                f_plus = f_state(x_plus);
                f_minus = f_state(x_minus);
                A_fit(7:12,col_index) = ...
                    (f_plus(7:12)-f_minus(7:12))/(2*h(col_index));
            end
            [M_eq,~,B_eq] = dyn_fun(q_eq,dq_zero,p);
            B_fit = [zeros(6,5); M_eq\B_eq];
            K_fit = lqr(A_fit,B_fit,lqr_Q,lqr_R);

            K_vector = reshape(K_fit.',1,[]);
            K_sample_2d(idx,:) = [l_l_fit,l_r_fit,K_vector];
            offset_sample_2d(idx,:) = [l_l_fit,l_r_fit,z_eq(1),z_eq(2),0,0];
            input_eq_sample_2d(idx,:) = [l_l_fit,l_r_fit,z_eq(3),z_eq(4)];
        catch ME
            warning('l_l=%.3f, l_r=%.3f 计算失败：%s',l_l_fit,l_r_fit,ME.message);
        end

        if mod(idx,49)==0 || idx==sample_count
            fprintf('进度：%d/%d (%.1f%%)\n',idx,sample_count,100*idx/sample_count);
        end
    end
end

%% 过滤无效样本并进行二维二次拟合
valid_idx = ~any(isnan(K_sample_2d),2) & ...
            ~any(isnan(offset_sample_2d),2) & ...
            ~any(isnan(input_eq_sample_2d),2);
K_sample_2d_valid = K_sample_2d(valid_idx,:);
offset_sample_2d_valid = offset_sample_2d(valid_idx,:);
input_eq_sample_2d_valid = input_eq_sample_2d(valid_idx,:);

fprintf('有效样本数：%d/%d\n',size(K_sample_2d_valid,1),sample_count);
if size(K_sample_2d_valid,1) < 20
    error('有效样本少于 20，停止拟合。');
end

l_l_samples = K_sample_2d_valid(:,1);
l_r_samples = K_sample_2d_valid(:,2);
design = [ones(size(l_l_samples)),l_l_samples,l_r_samples, ...
          l_l_samples.^2,l_l_samples.*l_r_samples,l_r_samples.^2];
fit_values = [K_sample_2d_valid(:,3:62), ...
              offset_sample_2d_valid(:,3:6), ...
              input_eq_sample_2d_valid(:,3:4)];
all_coefficients = design\fit_values;
K_Fit_Coefficients = all_coefficients(:,1:60)';
Offset_Fit_Coefficients = all_coefficients(:,61:64)';
InputEq_Fit_Coefficients = all_coefficients(:,65:66)';

%% 计算拟合误差，便于手动验证
fit_prediction = design*all_coefficients;
fit_error = fit_prediction-fit_values;
fit_error_inf = max(abs(fit_error),[],1);
K_fit_error_inf = max(fit_error_inf(1:60));
Offset_fit_error_inf = max(fit_error_inf(61:64));
Input_fit_error_inf = max(fit_error_inf(65:66));
fprintf('K 拟合最大绝对误差：%.3e\n',K_fit_error_inf);
fprintf('平衡角拟合最大绝对误差：%.3e\n',Offset_fit_error_inf);
fprintf('平衡输入拟合最大绝对误差：%.3e\n',Input_fit_error_inf);

%% 保存结果
save('lqr_fitting_results_corrected.mat', ...
    'K_sample_2d','K_sample_2d_valid', ...
    'K_Fit_Coefficients','Offset_Fit_Coefficients','InputEq_Fit_Coefficients', ...
    'offset_sample_2d','offset_sample_2d_valid', ...
    'input_eq_sample_2d','input_eq_sample_2d_valid', ...
    'l_range','lqr_Q','lqr_R','relative_step','design', ...
    'fit_error_inf','K_fit_error_inf','Offset_fit_error_inf', ...
    'Input_fit_error_inf','-v7.3');
fprintf('拟合结果已保存到 lqr_fitting_results_corrected.mat\n');

%% 局部函数
function residual = equilibrium_residual(dyn_fun,q_value,dq_value,u_value,p)
    [M_value,g_value,B_value] = dyn_fun(q_value,dq_value,p); %#ok<ASGLU>
    % 使用完整六个广义方程；前两个平移/Yaw 方程在静态点应自动为零。
    residual = reshape(double(g_value-B_value*u_value),[],1);
    residual = residual(3:6);
end

function ddq_value = explicit_acceleration(dyn_fun,q_value,dq_value,u_value,p)
    [M_value,g_value,B_value] = dyn_fun(q_value,dq_value,p);
    ddq_value = M_value\(-g_value+B_value*u_value);
    ddq_value = reshape(double(ddq_value),[],1);
end
