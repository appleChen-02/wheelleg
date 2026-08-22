% compute_lqr_corrected.m
% -------------------------------------------------------------------------
% 读取 build_corrected_dynamics.m 生成的修正模型，完成：
%   1. 代入物理参数；
%   2. 搜索 theta_b=theta_t=0 的对称静态平衡点；
%   3. 对完整显式动力学进行数值中心差分线性化；
%   4. 计算 12 状态、5 输入系统的 LQR 增益 K。
%
% 控制律约定：delta_u = -K*delta_x。
% -------------------------------------------------------------------------

clear; clc;
fprintf('读取修正后的动力学模型...\n');
load('corrected_dynamics.mat');

%% 重新声明与模型同名的符号，用于参数代入
syms g R R_w real
syms m_b I_b l_body_com theta_b0 real
syms m_l m_r I_l I_r l_l l_r l_leg_l_com l_leg_r_com real
syms theta_l0 theta_r0 real
syms m_wl m_wr I_wl I_wr real
syms m_t I_t l_tail_com delta_t l_tail_mount_h l_tail_mount_v real
syms I_yaw real
syms X_b_h phi theta_l theta_r theta_b theta_t real
syms dX_b_h dphi dtheta_l dtheta_r dtheta_b dtheta_t real
syms T_hip_r T_hip_l T_motor_wr T_motor_wl T_tail real

param_syms_local = [g; R; R_w; m_b; I_b; l_body_com; theta_b0; ...
    m_l; m_r; I_l; I_r; l_l; l_r; l_leg_l_com; l_leg_r_com; ...
    theta_l0; theta_r0; m_wl; m_wr; I_wl; I_wr; m_t; I_t; ...
    l_tail_com; delta_t; l_tail_mount_h; l_tail_mount_v; I_yaw];

%% 新版 Python 使用的物理参数
% 注意：新版重力大小为正数；theta_l0/theta_r0 的符号也与旧 MATLAB 版本不同。
param_values = [ ...
    9.81; ...                    % g
    0.130; ...                   % R
    0.386/2; ...                 % R_w
    6.9; 59035.925e-6; 4.3e-3; 0; ...
    2.2; 2.2; 0.034231929; 0.034231929; 0.17; 0.17; ...
    0.10157; 0.10157; -0.582108261; -0.582108261; ...
    0.823; 0.823; 6311.798e-6; 6311.798e-6; ...
    0.83; 29341.743e-6; 0.17755; 0.06597; ...
    0.07125; 0.1105; 294272.34e-6];

if numel(param_syms_local) ~= numel(param_values)
    error('参数符号数量与参数数值数量不一致。');
end

M_param = subs(M_lag_sym, param_syms_local, param_values);
g_param = subs(g_lag_sym, param_syms_local, param_values);
B_param = subs(B_lag_sym, param_syms_local, param_values);

%% 搜索对称静态平衡点
% 约束：theta_b=theta_t=0，theta_l=theta_r=theta_s，轮电机力矩为零。
% 未知量为 theta_s、左右相同的髋关节力矩和尾巴力矩。
syms theta_s T_hip_eq T_tail_eq real

static_symbols = [X_b_h; phi; theta_l; theta_r; theta_b; theta_t; ...
    dX_b_h; dphi; dtheta_l; dtheta_r; dtheta_b; dtheta_t];
static_values = [0; 0; theta_s; theta_s; 0; 0; zeros(6,1)];

h_static = subs(g_param, static_symbols, static_values);
B_static = subs(B_param, static_symbols(1:6), static_values(1:6));
u_static = [T_hip_eq; T_hip_eq; 0; 0; T_tail_eq];
full_static_residual = simplify(h_static - B_static*u_static);

% 对称情况下第 3、4 个腿方程相同，因此使用它们的和，加上机体和尾巴方程。
reduced_static_residual = [full_static_residual(3) + full_static_residual(4); ...
                           full_static_residual(5); ...
                           full_static_residual(6)];
static_fun = matlabFunction(reduced_static_residual, ...
    'Vars', {theta_s, T_hip_eq, T_tail_eq});
full_static_fun = matlabFunction(full_static_residual, ...
    'Vars', {theta_s, T_hip_eq, T_tail_eq});

if exist('fsolve','file') ~= 2
    error('需要 Optimization Toolbox 中的 fsolve 来搜索静态平衡点。');
end

opts = optimoptions('fsolve','Display','off', ...
    'FunctionTolerance',1e-10,'StepTolerance',1e-10, ...
    'MaxIterations',400,'MaxFunctionEvaluations',2000);
seed_angles = [0.3, linspace(-2.8,2.8,17)];
candidates = [];
for k = 1:numel(seed_angles)
    x0 = [seed_angles(k); 1; -1];
    try
        residual_handle = @(z) reshape(double(static_fun(z(1),z(2),z(3))),[],1);
        [x_candidate, fval, exitflag] = fsolve(residual_handle,x0,opts);
        x_candidate(1) = mod(x_candidate(1)+pi,2*pi)-pi;
        full_value = reshape(double(full_static_fun(...
            x_candidate(1),x_candidate(2),x_candidate(3))),[],1);
        if exitflag > 0 && all(isfinite(x_candidate)) && ...
                norm(fval,inf) <= 1e-8 && norm(full_value,inf) <= 1e-8 && ...
                x_candidate(1) >= -pi && x_candidate(1) <= pi
            if isempty(candidates) || all(abs(candidates(:,1)-x_candidate(1)) > 1e-6)
                candidates(end+1,:) = x_candidate(:).'; %#ok<AGROW>
            end
        end
    catch
        % 一个初值失败时继续尝试其余初值。
    end
end

if isempty(candidates)
    error('没有找到满足新版模型静态方程的平衡点。');
end
[~, selected_index] = min(abs(candidates(:,1)-0.3));
selected = candidates(selected_index,:).';
theta_l_star = selected(1);
theta_r_star = selected(1);
T_hip_eq_val = selected(2);
T_tail_eq_val = selected(3);

q_eq = [0; 0; theta_l_star; theta_r_star; 0; 0];
dq_eq = zeros(6,1);
u_eq = [T_hip_eq_val; T_hip_eq_val; 0; 0; T_tail_eq_val];

fprintf('平衡点：theta_l=theta_r=%.10f rad，T_hip=%.10f，T_tail=%.10f\n', ...
    theta_l_star,T_hip_eq_val,T_tail_eq_val);
fprintf('完整静态残差无穷范数：%.3e\n', ...
    norm(reshape(double(full_static_fun(theta_l_star,T_hip_eq_val,T_tail_eq_val)),[],1),inf));

%% 生成显式动力学函数
% f(x,u) = [dq; M(q)^(-1)*(-g(q,dq)+B(q)*u)]。
dyn_fun = matlabFunction(M_param,g_param,B_param,'Vars', ...
    {[X_b_h; phi; theta_l; theta_r; theta_b; theta_t], ...
     [dX_b_h; dphi; dtheta_l; dtheta_r; dtheta_b; dtheta_t]});

acceleration = @(q_value,dq_value,u_value) local_acceleration( ...
    dyn_fun,q_value,dq_value,u_value);
state_dynamics = @(x_value,u_value) [x_value(7:12); ...
    acceleration(x_value(1:6),x_value(7:12),u_value)];

%% 在非零输入平衡点对完整加速度场做中心差分
x_eq = [q_eq; dq_eq];
n = 12;
m_ctrl = 5;
A_num = zeros(n,n);
A_num(1:6,7:12) = eye(6);
relative_step = 1e-5;
finite_difference_steps = relative_step*max(1,abs(x_eq));
for i = 1:n
    x_plus = x_eq;
    x_minus = x_eq;
    x_plus(i) = x_plus(i) + finite_difference_steps(i);
    x_minus(i) = x_minus(i) - finite_difference_steps(i);
    f_plus = state_dynamics(x_plus,u_eq);
    f_minus = state_dynamics(x_minus,u_eq);
    A_num(7:12,i) = (f_plus(7:12)-f_minus(7:12)) ...
        /(2*finite_difference_steps(i));
end

[M_eq,g_eq,B_eq] = dyn_fun(q_eq,dq_eq);
B_num = [zeros(6,m_ctrl); M_eq\B_eq];
equilibrium_acceleration = acceleration(q_eq,dq_eq,u_eq);
fprintf('平衡点加速度无穷范数：%.3e\n',norm(equilibrium_acceleration,inf));

%% 可控性和 LQR
Co = ctrb(A_num,B_num);
rank_Co = rank(Co);
fprintf('可控性矩阵秩：%d / %d\n',rank_Co,n);

lqr_Q = diag([120,400,4000,1,2000,10,2000,10,6000,1,300,5]);
lqr_R = diag([1,1,12,12,8]);

if exist('lqr','file') ~= 2
    error('需要 Control System Toolbox 中的 lqr。');
end
[K,S,e] = lqr(A_num,B_num,lqr_Q,lqr_R);
fprintf('LQR K 尺寸：%d x %d\n',size(K,1),size(K,2));
fprintf('闭环特征值最大实部：%.6e\n',max(real(e)));

%% 保存结果
save('lqr_results_corrected.mat','A_num','B_num','K','S','e', ...
    'lqr_Q','lqr_R','q_eq','dq_eq','u_eq','theta_l_star', ...
    'theta_r_star','T_hip_eq_val','T_tail_eq_val','M_eq','g_eq','B_eq', ...
    'rank_Co','finite_difference_steps','relative_step', ...
    'equilibrium_acceleration','-v7.3');

fprintf('结果已保存到 lqr_results_corrected.mat\n');

%% 局部函数：计算显式广义加速度
function ddq_value = local_acceleration(dyn_fun,q_value,dq_value,u_value)
    [M_value,g_value,B_value] = dyn_fun(q_value,dq_value);
    ddq_value = M_value\(-g_value + B_value*u_value);
    ddq_value = reshape(double(ddq_value),[],1);
end
