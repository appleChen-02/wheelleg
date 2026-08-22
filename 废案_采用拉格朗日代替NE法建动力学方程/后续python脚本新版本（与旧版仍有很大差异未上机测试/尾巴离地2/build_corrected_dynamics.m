% build_corrected_dynamics.m
% -------------------------------------------------------------------------
% 根据 compare_dynamics_dynamics_corrected_s.py 重建修正后的符号动力学模型。
%
% 输出 corrected_dynamics.mat，主要变量为：
%   M_lag_sym, g_lag_sym, B_lag_sym : 修正后的 Lagrange 模型
%   eqs_lag                         : M*ddq+g-B*u=0 的六个方程
%   q, dq, ddq, u                   : 状态、速度、加速度和输入符号
%   param_syms                      : 参数符号向量
%
% 约定：g 是正的重力大小（9.81），竖直向下方向由方程中的 -m*g 表示。
% LQR 主流程使用 Lagrange 模型；Newton-Euler 模型建议单独用于验证。
% -------------------------------------------------------------------------

clear; clc;
fprintf('开始构建修正后的 Lagrange 动力学模型...\n');

%% 物理参数、广义坐标和控制输入
syms g R R_w real
syms m_b I_b l_body_com theta_b0 real
syms m_l m_r I_l I_r l_l l_r l_leg_l_com l_leg_r_com real
syms theta_l0 theta_r0 real
syms m_wl m_wr I_wl I_wr real
syms m_t I_t l_tail_com delta_t l_tail_mount_h l_tail_mount_v real
syms I_yaw real

syms X_b_h phi theta_l theta_r theta_b theta_t real
syms dX_b_h dphi dtheta_l dtheta_r dtheta_b dtheta_t real
syms ddX_b_h ddphi ddtheta_l ddtheta_r ddtheta_b ddtheta_t real

syms T_hip_r T_hip_l T_motor_wr T_motor_wl T_tail real

q = [X_b_h; phi; theta_l; theta_r; theta_b; theta_t];
dq = [dX_b_h; dphi; dtheta_l; dtheta_r; dtheta_b; dtheta_t];
ddq = [ddX_b_h; ddphi; ddtheta_l; ddtheta_r; ddtheta_b; ddtheta_t];
u = [T_hip_r; T_hip_l; T_motor_wr; T_motor_wl; T_tail];
param_syms = [g; R; R_w; m_b; I_b; l_body_com; theta_b0; ...
    m_l; m_r; I_l; I_r; l_l; l_r; l_leg_l_com; l_leg_r_com; ...
    theta_l0; theta_r0; m_wl; m_wr; I_wl; I_wr; m_t; I_t; ...
    l_tail_com; delta_t; l_tail_mount_h; l_tail_mount_v; I_yaw];

%% 几何角度和轮-腿约束
beta_l = theta_l + theta_l0;
beta_r = theta_r + theta_r0;
beta_b = theta_b + theta_b0;
alpha_t = theta_t + theta_b - delta_t;
dalpha = dtheta_t + dtheta_b;
ddalpha = ddtheta_t + ddtheta_b;

% 轮心水平速度来自广义坐标约束；竖直速度为零。
v_wl_h = dX_b_h - R_w*dphi - l_l*cos(theta_l)*dtheta_l;
v_wr_h = dX_b_h + R_w*dphi - l_r*cos(theta_r)*dtheta_r;

% 左右髋点速度。代入约束后髋中点水平速度就是 dX_b_h。
v_hl_h = v_wl_h + l_l*cos(theta_l)*dtheta_l;
v_hr_h = v_wr_h + l_r*cos(theta_r)*dtheta_r;
v_hl_v = -l_l*sin(theta_l)*dtheta_l;
v_hr_v = -l_r*sin(theta_r)*dtheta_r;
v_hip_h = simplify((v_hl_h + v_hr_h)/2);
v_hip_v = (v_hl_v + v_hr_v)/2;

%% 各部件质心速度、动能和势能
% 左右腿质心
v_leg_l_h = v_wl_h + l_leg_l_com*cos(beta_l)*dtheta_l;
v_leg_l_v = -l_leg_l_com*sin(beta_l)*dtheta_l;
v_leg_r_h = v_wr_h + l_leg_r_com*cos(beta_r)*dtheta_r;
v_leg_r_v = -l_leg_r_com*sin(beta_r)*dtheta_r;

% 机体质心：相对髋中点的位置为 [-l_body_com*sin(beta_b),
% l_body_com*cos(beta_b)]。
v_body_h = v_hip_h - l_body_com*cos(beta_b)*dtheta_b;
v_body_v = v_hip_v - l_body_com*sin(beta_b)*dtheta_b;

% 尾巴安装点和尾巴质心
r_tail_mount_h = l_tail_mount_h*cos(theta_b) + l_tail_mount_v*sin(theta_b);
r_tail_mount_v = l_tail_mount_h*sin(theta_b) - l_tail_mount_v*cos(theta_b);
v_mount_h = v_hip_h - r_tail_mount_v*dtheta_b;
v_mount_v = v_hip_v + r_tail_mount_h*dtheta_b;
v_tail_h = v_mount_h - l_tail_com*sin(alpha_t)*dalpha;
v_tail_v = v_mount_v + l_tail_com*cos(alpha_t)*dalpha;

K_wl = sym(1)/2*(m_wl + I_wl/R^2)*v_wl_h^2;
K_wr = sym(1)/2*(m_wr + I_wr/R^2)*v_wr_h^2;
K_leg_l = sym(1)/2*m_l*(v_leg_l_h^2 + v_leg_l_v^2) ...
    + sym(1)/2*I_l*dtheta_l^2;
K_leg_r = sym(1)/2*m_r*(v_leg_r_h^2 + v_leg_r_v^2) ...
    + sym(1)/2*I_r*dtheta_r^2;
K_body = sym(1)/2*m_b*(v_body_h^2 + v_body_v^2) ...
    + sym(1)/2*I_b*dtheta_b^2;
K_tail = sym(1)/2*m_t*(v_tail_h^2 + v_tail_v^2) ...
    + sym(1)/2*I_t*dalpha^2;

y_hip = R + (l_l*cos(theta_l) + l_r*cos(theta_r))/2;
V = m_wl*g*R + m_wr*g*R ...
    + m_l*g*(R + l_leg_l_com*cos(beta_l)) ...
    + m_r*g*(R + l_leg_r_com*cos(beta_r)) ...
    + m_b*g*(y_hip + l_body_com*cos(beta_b)) ...
    + m_t*g*(y_hip + r_tail_mount_v + l_tail_com*sin(alpha_t));

T = K_wl + K_wr + K_leg_l + K_leg_r + K_body ...
    + sym(1)/2*I_yaw*dphi^2 + K_tail;
L = T - V;

%% 修正后的广义输入
% 输入顺序必须与 compute_lqr_corrected.m 及控制器保持一致。
Q = [-(T_motor_wl + T_motor_wr)/R;
      R_w/R*(T_motor_wl - T_motor_wr);
      T_motor_wl*(1 + l_l*cos(theta_l)/R) + T_hip_l;
      T_motor_wr*(1 + l_r*cos(theta_r)/R) + T_hip_r;
      T_hip_l + T_hip_r;
      -T_tail];

%% Euler-Lagrange 方程
% 方程形式：E = M(q)*ddq + g(q,dq) - B(q)*u = 0。
eqs_lag = sym(zeros(6,1));
for k = 1:6
    p_k = diff(L, dq(k));
    dpdt_k = 0;
    for j = 1:6
        dpdt_k = dpdt_k + diff(p_k, q(j))*dq(j) ...
            + diff(p_k, dq(j))*ddq(j);
    end
    eqs_lag(k) = expand(dpdt_k - diff(L, q(k)) - Q(k));
end

M_lag_sym = jacobian(eqs_lag, ddq);
B_lag_sym = -jacobian(eqs_lag, u);
zero_dynamic = [ddq; u];
g_lag_sym = subs(eqs_lag, zero_dynamic, zeros(numel(zero_dynamic),1));

% 基本符号一致性检查。
affine_error = simplify(eqs_lag - M_lag_sym*ddq - g_lag_sym + B_lag_sym*u);
if any(arrayfun(@(x) ~isequal(x, sym(0)), affine_error))
    error('修正后的 Lagrange 方程未能整理成 M*ddq+g-B*u=0。');
end

fprintf('检查质量矩阵对称性...\n');
if any(arrayfun(@(x) ~isequal(x, sym(0)), simplify(M_lag_sym-M_lag_sym.')))
    warning('符号质量矩阵不是严格对称，可能只是未完全化简。');
end

%% 保存模型
model_version = 'corrected_lagrange_from_python_2026';
save('corrected_dynamics.mat', ...
    'M_lag_sym','g_lag_sym','B_lag_sym','eqs_lag', ...
    'q','dq','ddq','u','param_syms', ...
    'model_version','-v7.3');

fprintf('修正后的模型已保存到 corrected_dynamics.mat\n');
fprintf('状态维度：6 个广义坐标，LQR 状态维度：12\n');

