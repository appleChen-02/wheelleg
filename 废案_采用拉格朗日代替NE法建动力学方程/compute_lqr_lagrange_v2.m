% compute_lqr_lagrange_v2.m
% =========================================================================
% 轮腿机器人 LQR 控制器计算 — 拉格朗日法版本
%
% 功能: 加载拉格朗日动力学 → 代入数值参数 → 求解平衡点 → 线性化 → LQR
%
% 输入: dynamics_v2_lagrange.mat (M_sym, g_sym, B_sym, eq1~eq6)
% 输出: lqr_results_lagrange.mat, lqr_fitting_results_lagrange.mat
%
% 方法: 与 lqr_lagrange.m (二阶倒立摆) 相同的拉格朗日线性化流程
%       A = [0 I; -M⁻¹·∂g/∂q  -M⁻¹·∂g/∂dq]
%       B = [0; M⁻¹·B_ctrl]
%
% ========== 状态向量 (12维) ==========
%   X = [X_b^h, V_b^h, phi, dphi,
%        theta_l, dtheta_l, theta_r, dtheta_r,
%        theta_b, dtheta_b, theta_t, dtheta_t]^T
%
% ========== 控制向量 (5维) ==========
%   U = [T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail]^T
% =========================================================================

clear; clc;
tic_total = tic;
fprintf('========================================\n');
fprintf('轮腿机器人 LQR 控制器计算 (拉格朗日法)\n');
fprintf('========================================\n\n');

%% ========================================================================
%  Part 1: 定义物理参数数值
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 1: 定义物理参数\n');
fprintf('========================================\n\n');

% ==================== 物理常数 ====================
g_val = -9.81;              % 重力加速度 (m/s^2)

% ==================== 几何参数 ====================
R_val = 0.130;              % 轮子半径 (m)
R_w_val = 0.386 / 2;        % 轮距/2 (m)

% ==================== 机体参数 ====================
m_b_val = 6.9;              % 机体质量 (kg)
I_b_val = 59035.925e-6;     % 机体俯仰转动惯量 (kg·m²)
l_body_com_val = 4.3e-3;     % 机体质心到俯仰轴距离 (m)
I_yaw_val = 294272.34e-6;   % 整体yaw轴转动惯量 (kg·m²)
theta_b0_val = 0;           % 质心偏移角度 (rad)

% ==================== 轮子参数 ====================
m_wl_val = 0.823;           % 左轮质量 (kg)
m_wr_val = 0.823;           % 右轮质量 (kg)
I_wl_val = 6311.798e-6;     % 左轮转动惯量 (kg·m²)
I_wr_val = 6311.798e-6;     % 右轮转动惯量 (kg·m²)

% ==================== 腿部参数 (默认腿长 0.17m) ====================
l_l_val = 0.17;             % 左腿长度 (m)
l_r_val = 0.17;             % 右腿长度 (m)
m_l_val = 2.2;              % 左腿质量 (kg)
m_r_val = 2.2;              % 右腿质量 (kg)
I_l_val = 0.034231929;      % 左腿转动惯量 (kg·m²)
I_r_val = 0.034231929;      % 右腿转动惯量 (kg·m²)
l_leg_l_com_val = 0.10157;   % 左腿质心到轮轴距离 (m)
l_leg_r_com_val = 0.10157;   % 右腿质心到轮轴距离 (m)
theta_l0_val = 0.582108261; % 左腿偏移角度 (rad)
theta_r0_val = 0.582108261; % 右腿偏移角度 (rad)

% ==================== 尾巴参数 v1.1 ====================
m_t_val = 0.83;             % 尾巴质量 (kg)
I_t_val = 29341.743e-6;     % 尾巴绕尾电机轴转动惯量 (kg·m²)
l_tail_mount_h_val = 0.07125;  % 安装点相对机身质心前向距离 (m)
l_tail_mount_v_val = 0.1105;   % 安装点相对机身质心下向距离的绝对值 (m)
l_tail_com_val = 0.17755;      % 尾电机到尾巴质心距离 (m)
delta_t_val = 0.06597;      % 尾巴质心偏置角 (rad)
theta_t_star_val = 0.0;     % 尾巴平衡角 (rad)

% Yaw惯量随腿长变化的拟合函数
Iyaw_fun = @(x) -6.4624*x.^4 + 6.7339*x.^3 - 3.0029*x.^2 + 0.5202*x + 0.2758;

fprintf('✓ 物理参数设置完成\n\n');

%% ========================================================================
%  Part 2: 定义符号变量并加载拉格朗日动力学
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 2: 加载拉格朗日动力学方程\n');
fprintf('========================================\n\n');

% ====== 广义坐标 q (6维) ======
syms X_b_h    phi    theta_l    theta_r    theta_b    theta_t     real

% ====== 广义速度 dq ======
syms dX_b_h   dphi   dtheta_l   dtheta_r   dtheta_b   dtheta_t   real

% ====== 广义加速度 ddq ======
syms ddX_b_h ddphi  ddtheta_l  ddtheta_r  ddtheta_b  ddtheta_t  real

% ====== 控制输入 u (5维) ======
syms T_hip_r T_hip_l T_motor_wr T_motor_wl T_tail real

% ====== 物理参数符号 ======
syms g R R_w real
syms m_b I_b l_body_com theta_b0 real
syms l_tail_mount_h l_tail_mount_v real
syms m_l m_r I_l I_r l_l l_r l_leg_l_com l_leg_r_com theta_l0 theta_r0 real
syms m_wl m_wr I_wl I_wr real
syms m_t I_t l_tail_com delta_t real
syms I_yaw real

q   = [X_b_h;    phi;    theta_l;    theta_r;    theta_b;    theta_t   ];
dq  = [dX_b_h;   dphi;   dtheta_l;   dtheta_r;   dtheta_b;   dtheta_t  ];
ddq = [ddX_b_h;  ddphi;  ddtheta_l;  ddtheta_r;  ddtheta_b;  ddtheta_t ];
u   = [T_hip_r; T_hip_l; T_motor_wr; T_motor_wl; T_tail];

n_q = 6;
n_u = 5;

% 加载拉格朗日动力学
load('dynamics_v2_lagrange.mat');
fprintf('✓ 已加载 dynamics_v2_lagrange.mat\n');
fprintf('  包含: M_sym(%dx%d), g_sym(%dx1), B_sym(%dx%d)\n\n', n_q, n_q, n_q, n_q, n_u);

%% ========================================================================
%  Part 3: 代入物理参数数值
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 3: 代入物理参数数值\n');
fprintf('========================================\n\n');

% 物理参数代换表
param_subs = {
    m_b,    m_b_val;
    m_l,    m_l_val;
    m_r,    m_r_val;
    m_wl,   m_wl_val;
    m_wr,   m_wr_val;
    I_b,    I_b_val;
    I_l,    I_l_val;
    I_r,    I_r_val;
    I_wl,   I_wl_val;
    I_wr,   I_wr_val;
    I_yaw,  I_yaw_val;
    l_l,    l_l_val;
    l_r,    l_r_val;
    l_leg_l_com,  l_leg_l_com_val;
    l_leg_r_com,  l_leg_r_com_val;
    l_body_com,   l_body_com_val;
    R,      R_val;
    R_w,    R_w_val;
    g,      g_val;
    theta_l0, theta_l0_val;
    theta_r0, theta_r0_val;
    theta_b0, theta_b0_val;
    m_t,    m_t_val;
    I_t,    I_t_val;
    l_tail_mount_h,  l_tail_mount_h_val;
    l_tail_mount_v,  l_tail_mount_v_val;
    l_tail_com,  l_tail_com_val;
    delta_t, delta_t_val;
};

% 代入物理参数
fprintf('  正在代入物理参数...\n');
tic_param = tic;
M_param = simplify(subs(M_sym, param_subs(:,1), param_subs(:,2)));
g_param = simplify(subs(g_sym, param_subs(:,1), param_subs(:,2)));
B_param = simplify(subs(B_sym, param_subs(:,1), param_subs(:,2)));
fprintf('  参数代入完成 (耗时 %.1f 秒)\n\n', toc(tic_param));

%% ========================================================================
%  Part 4: 求解平衡点 theta_l*, theta_r*, T_leg_eq, T_t_eq
%  ========================================================================
%
%  平衡条件: 所有速度=0, 加速度=0, theta_b*=0, theta_t*=0
%  静态方程: g(q, dq=0) = B(q) * u_eq
%
%  利用第3~6行 (theta_l, theta_r, theta_b, theta_t) 求解4个未知量

fprintf('========================================\n');
fprintf('Part 4: 求解平衡点\n');
fprintf('========================================\n\n');

% 目标平衡姿态
theta_b_star = 0.0;
theta_t_star = 0.0;

% 未知静态输入 (假设左右腿对机身静态力矩相同)
syms T_leg_eq T_t_eq real

% 静态代换: 固定姿态、零速度、零加速度
static_subs = {
    theta_b,    theta_b_star;
    theta_t,    theta_t_star;
    dtheta_l,   0;
    dtheta_r,   0;
    dtheta_b,   0;
    dtheta_t,   0;
    dX_b_h,     0;
    dphi,       0;
    ddtheta_l,  0;
    ddtheta_r,  0;
    ddtheta_b,  0;
    ddtheta_t,  0;
    ddX_b_h,    0;
    ddphi,      0;
    phi,        0;
    X_b_h,      0;
    T_hip_r,   T_leg_eq;
    T_hip_l,   T_leg_eq;
    T_motor_wr,  0;
    T_motor_wl,  0;
    T_tail,   T_t_eq;
};

% 计算静态方程: g(q, dq=0) - B(q) * u_eq = 0
g_static = simplify(subs(g_param, static_subs(:,1), static_subs(:,2)));
B_static = simplify(subs(B_param, static_subs(:,1), static_subs(:,2)));
u_static = [T_leg_eq; T_leg_eq; 0; 0; T_t_eq];

eq_static_full = g_static - B_static * u_static;  % 6×1

% 选择第3~6个方程 (theta_l, theta_r, theta_b, theta_t 对应行)
% eq(1)=水平动量, eq(2)=yaw — 在静态平衡下自动满足
eq_static = eq_static_full(3:6);

fprintf('静态方程 (eq3~eq6):\n');
for i = 1:4
    fprintf('  eq_static(%d) = ', i+2);
    % 显示简化形式
    disp(eq_static(i));
end

fprintf('\n正在联立求解 [theta_l, theta_r, T_leg_eq, T_t_eq] ...\n');

% 初值
x0 = [0.2; 0.2; 0.4; -1.4];

sol_static_num = vpasolve( ...
    [eq_static(1) == 0, eq_static(2) == 0, eq_static(3) == 0, eq_static(4) == 0], ...
    [theta_l, theta_r, T_leg_eq, T_t_eq], ...
    x0);

if isempty(sol_static_num)
    error('vpasolve 未找到平衡点解，请检查静态方程、参数或初值设置。');
end

theta_l_star = double(sol_static_num.theta_l);
theta_r_star = double(sol_static_num.theta_r);
T_leg_eq_val = double(sol_static_num.T_leg_eq);
T_t_eq_val   = double(sol_static_num.T_t_eq);

% 标量检查
if ~isscalar(theta_l_star) || ~isscalar(theta_r_star) || ...
   ~isscalar(T_leg_eq_val) || ~isscalar(T_t_eq_val)
    error('vpasolve 返回结果不是标量，请检查求解结果。');
end

% 静态方程残差验证
eq_static_check = double(subs(eq_static_full, ...
    [theta_l, theta_r, T_leg_eq, T_t_eq], ...
    [theta_l_star, theta_r_star, T_leg_eq_val, T_t_eq_val]));

fprintf('\n静态方程残差 (全部6个方程):\n');
disp(eq_static_check);
fprintf('静态方程残差 ||eq_static||: %.2e\n', norm(eq_static_check));

fprintf('\n平衡点求解结果:\n');
fprintf('  theta_l* = %.10f rad (%.6f deg)\n', theta_l_star, rad2deg(theta_l_star));
fprintf('  theta_r* = %.10f rad (%.6f deg)\n', theta_r_star, rad2deg(theta_r_star));
fprintf('  theta_b* = %.10f rad (%.6f deg)\n', theta_b_star, rad2deg(theta_b_star));
fprintf('  theta_t* = %.10f rad (%.6f deg)\n', theta_t_star, rad2deg(theta_t_star));
fprintf('  T_leg_eq = %.10f\n', T_leg_eq_val);
fprintf('  T_t_eq   = %.10f\n', T_t_eq_val);
fprintf('  ✓ 平衡点求解完成!\n\n');

%% ========================================================================
%  Part 5: 在平衡点处线性化
%  ========================================================================
%
%  拉格朗日法线性化 (参考 lqr_lagrange.m):
%    动力学: M(q)*ddq + g(q,dq) = B(q)*u
%    在平衡点 (dq=0): M_eq * ddq + (∂g/∂q)*Δq + (∂g/∂dq)*Δdq = B_eq * u
%    ddq = M_eq⁻¹ * (B_eq*u - K_q*Δq - K_dq*Δdq)
%
%    其中 K_q = ∂g/∂q|₀, K_dq = ∂g/∂dq|₀

fprintf('========================================\n');
fprintf('Part 5: 在平衡点处线性化\n');
fprintf('========================================\n\n');

% 平衡点代换表 (含 u_eq)
eq_subs_full = {
    theta_l,    theta_l_star;
    theta_r,    theta_r_star;
    theta_b,    theta_b_star;
    theta_t,    theta_t_star;
    dtheta_l,   0;
    dtheta_r,   0;
    dtheta_b,   0;
    dtheta_t,   0;
    dX_b_h,     0;
    dphi,       0;
    phi,        0;
    X_b_h,      0;
    T_hip_r,   T_leg_eq_val;
    T_hip_l,   T_leg_eq_val;
    T_motor_wr,  0;
    T_motor_wl,  0;
    T_tail,   T_t_eq_val;
};

% ----- M_eq, B_eq (数值) -----
fprintf('  计算 M_eq, B_eq...\n');
M_eq = double(subs(M_param, eq_subs_full(:,1), eq_subs_full(:,2)));
B_eq = double(subs(B_param, eq_subs_full(:,1), eq_subs_full(:,2)));
g_eq = double(subs(g_param, eq_subs_full(:,1), eq_subs_full(:,2)));

fprintf('  平衡点 M 矩阵 (6×6):\n');
disp(M_eq);

fprintf('  平衡点 B 矩阵 (6×5):\n');
disp(B_eq);

fprintf('  平衡点 g 向量 (应为零向量):\n');
disp(g_eq);
fprintf('  ||g_eq|| = %.2e\n\n', norm(g_eq));

% ----- ∂g/∂q (刚度矩阵 K_q) -----
fprintf('  计算刚度矩阵 K_q = ∂g/∂q...\n');
K_q = sym(zeros(n_q, n_q));
for i = 1:n_q
    for j = 1:n_q
        K_q(i,j) = diff(g_param(i), q(j));
    end
end
K_q_eq = double(subs(K_q, eq_subs_full(:,1), eq_subs_full(:,2)));

fprintf('  K_q = ∂g/∂q 在平衡点:\n');
disp(K_q_eq);

% ----- 提取 g 中与 dq 有关的部分，计算 ∂g/∂dq (阻尼矩阵 K_dq) -----
fprintf('  计算阻尼矩阵 K_dq = ∂g/∂dq...\n');
K_dq = sym(zeros(n_q, n_q));
for i = 1:n_q
    for j = 1:n_q
        K_dq(i,j) = diff(g_param(i), dq(j));
    end
end
K_dq_eq = double(subs(K_dq, eq_subs_full(:,1), eq_subs_full(:,2)));

fprintf('  K_dq = ∂g/∂dq 在平衡点:\n');
disp(K_dq_eq);

%% ========================================================================
%  Part 6: 构建 12×12 状态空间矩阵
%  ========================================================================
%
%  状态: X = [X_b^h, dX_b^h, phi, dphi,
%            theta_l, dtheta_l, theta_r, dtheta_r,
%            theta_b, dtheta_b, theta_t, dtheta_t]^T
%
%  dX/dt = A·X + B_ctrl·u

fprintf('========================================\n');
fprintf('Part 6: 构建状态空间 A, B 矩阵\n');
fprintf('========================================\n\n');

n_states = 12;
A_num = zeros(n_states, n_states);
B_num = zeros(n_states, n_u);

% ----- 运动学关系 (位置-速度) -----
A_num(1,2)  = 1;   % dX_b_h/dt  = V_b_h
A_num(3,4)  = 1;   % dphi/dt    = dphi
A_num(5,6)  = 1;   % dtheta_l/dt = dtheta_l
A_num(7,8)  = 1;   % dtheta_r/dt = dtheta_r
A_num(9,10) = 1;   % dtheta_b/dt = dtheta_b
A_num(11,12)= 1;   % dtheta_t/dt = dtheta_t

% ----- 动力学部分 -----
% ddq = M⁻¹ * (B*u - K_q*Δq - K_dq*Δdq)
% 加速度行 = [-M⁻¹*K_q,  -M⁻¹*K_dq]
M_inv = inv(M_eq);

A_dyn_q  = -M_inv * K_q_eq;   % 位置刚度 (6×6)
A_dyn_dq = -M_inv * K_dq_eq;  % 速度阻尼 (6×6)
B_dyn    =  M_inv * B_eq;     % 控制输入 (6×5)

% 填充 A 矩阵: 加速度行
% 行 2:  ddX_b_h  映射到状态 1,3,5,7,9,11 (位置) 和 2,4,6,8,10,12 (速度)
% 行 4:  ddphi
% 行 6:  ddtheta_l
% 行 8:  ddtheta_r
% 行 10: ddtheta_b
% 行 12: ddtheta_t

accel_rows = [2, 4, 6, 8, 10, 12];    % 对应 ddq(1)..ddq(6)
pos_cols   = [1, 3, 5, 7, 9, 11];     % 对应 q(1)..q(6)
vel_cols   = [2, 4, 6, 8, 10, 12];    % 对应 dq(1)..dq(6)

for i = 1:n_q   % 遍历6个加速度方程
    for j = 1:n_q   % 遍历6个广义坐标
        % 位置刚度项
        A_num(accel_rows(i), pos_cols(j)) = A_dyn_q(i,j);
        % 速度阻尼项
        A_num(accel_rows(i), vel_cols(j)) = A_dyn_dq(i,j);
    end
end

% 填充 B 矩阵: 加速度行
for i = 1:n_q
    B_num(accel_rows(i), :) = B_dyn(i, :);
end

fprintf('数值 A 矩阵 (12×12):\n');
disp(A_num);

fprintf('数值 B 矩阵 (12×5):\n');
disp(B_num);

%% ========================================================================
%  Part 7: 检查可控性
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 7: 检查系统可控性\n');
fprintf('========================================\n\n');

Co = ctrb(A_num, B_num);
rank_Co = rank(Co);
fprintf('可控性矩阵秩: %d (系统维度: %d)\n', rank_Co, n_states);

if rank_Co < n_states
    fprintf('  ⚠ 系统不完全可控 (秩=%d < %d)\n', rank_Co, n_states);
    fprintf('  物理原因: X_b^h 和 phi 是积分器状态\n');
    fprintf('  解决方案: 这是正常的! LQR 仍可计算\n\n');
else
    fprintf('  ✓ 系统完全可控\n\n');
end

%% ========================================================================
%  Part 8: 设置 LQR 权重
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 8: 设置 LQR 权重\n');
fprintf('========================================\n\n');

% Q矩阵: 状态权重 (12×12)
% 顺序: [X_b^h, V_b^h, phi, dphi,
%         theta_l, dtheta_l, theta_r, dtheta_r,
%         theta_b, dtheta_b, theta_t, dtheta_t]
lqr_Q = diag([120, 10, ...
              4000, 1, ...
              1000, 10, ...
              1000, 10, ...
              8000, 1, ...
              300, 5]);

% R矩阵: 控制输入权重 (5×5)
% 顺序: [T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail]
lqr_R = diag([1, 1, 10, 10, 10]);

fprintf('Q矩阵 (状态权重):\n');
disp(lqr_Q);

fprintf('R矩阵 (控制权重):\n');
disp(lqr_R);

%% ========================================================================
%  Part 9: 计算 LQR 增益
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 9: 计算 LQR 增益\n');
fprintf('========================================\n\n');

try
    [K, S, e] = lqr(A_num, B_num, lqr_Q, lqr_R);

    fprintf('✓ LQR增益矩阵 K (5×12):\n');
    disp(K);

    fprintf('闭环特征值:\n');
    disp(e);

    if all(real(e) < 1e-6)
        fprintf('✓ 闭环系统稳定!\n\n');
    else
        warning('闭环系统不稳定!');
    end
catch ME
    fprintf('✗ LQR计算失败: %s\n', ME.message);
    K = [];
end

%% ========================================================================
%  Part 10: 格式化输出 (C代码)
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 10: 格式化输出 (C代码)\n');
fprintf('========================================\n\n');

if ~isempty(K)
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// LQR增益矩阵 K[5][12] — 拉格朗日法\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// 状态向量 X[12]:\n');
    fprintf('//   [X_b_h, V_b_h, phi, dphi,\n');
    fprintf('//    theta_l, dtheta_l, theta_r, dtheta_r,\n');
    fprintf('//    theta_b, dtheta_b, theta_t, dtheta_t]\n');
    fprintf('// 控制向量 U[5]:\n');
    fprintf('//   [T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail]\n');
    fprintf('// 控制律: u = u_eq - K * (X - X_eq)\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n\n');

    fprintf('// 平衡点:\n');
    fprintf('float theta_l_eq = %.10ff;  // 左腿平衡点角度 (rad)\n', theta_l_star);
    fprintf('float theta_r_eq = %.10ff;  // 右腿平衡点角度 (rad)\n', theta_r_star);
    fprintf('float theta_b_eq = %.10ff;  // 机体俯仰平衡点角度 (rad)\n', theta_b_star);
    fprintf('float theta_t_eq = %.10ff;  // 尾巴平衡点角度 (rad)\n', theta_t_star);
    fprintf('float T_leg_eq = %.10ff;    // 左右腿静态平衡扭矩\n', T_leg_eq_val);
    fprintf('float T_t_eq = %.10ff;      // 尾巴静态平衡扭矩\n\n', T_t_eq_val);

    fprintf('// 完整平衡点状态向量 X_eq[12]\n');
    fprintf('float X_eq[12] = {0.0f, 0.0f, 0.0f, 0.0f, %.10ff, 0.0f, %.10ff, 0.0f, %.10ff, 0.0f, %.10ff, 0.0f};\n', ...
            theta_l_star, theta_r_star, theta_b_star, theta_t_star);
    fprintf('//                X_b^h  V_b^h  phi   dphi   theta_l  dtheta_l  theta_r  dtheta_r  theta_b  dtheta_b  theta_t  dtheta_t\n\n');

    fprintf('float K[5][12] = {\n');
    control_names = {'T_hip_r', 'T_hip_l', 'T_motor_wr', 'T_motor_wl', 'T_tail'};
    for i = 1:5
        fprintf('    {');
        for j = 1:12
            if j < 12
                fprintf('%11.6ff, ', K(i,j));
            else
                fprintf('%11.6ff', K(i,j));
            end
        end
        if i < 5
            fprintf('},  // %s\n', control_names{i});
        else
            fprintf('}   // %s\n', control_names{i});
        end
    end
    fprintf('};\n\n');

    % 输出平衡点偏移量说明
    fprintf('// 控制律: u = u_eq - K * (X - X_eq)\n');
    fprintf('// u_eq = [T_leg_eq, T_leg_eq, 0.0f, 0.0f, T_t_eq]\n\n');
end

%% ========================================================================
%  Part 11: 腿长拟合功能
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 11: 腿长拟合功能\n');
fprintf('========================================\n\n');

% 腿长参数查找表 (与 NE 代码一致)
theta0_fun = @(x) 277.69*x.^4 - 304.33*x.^3 + 130.52*x.^2 - 27.562*x + 2.7591;
ld_fun     = @(x) 1.4205*x.^4 - 2.0644*x.^3 + 1.2526*x.^2 - 0.059*x + 0.0844;
Ileg_fun   = @(x) -0.3588*x.^4 + 0.3595*x.^3 + 0.0659*x.^2 + 0.0318*x + 0.0255;

enable_fitting = true;

if enable_fitting
    fprintf('正在计算不同腿长下的 K 矩阵、平衡点偏移量和静态输入...\n\n');

    l_range = 0.15:0.005:0.30;
    num_legs = length(l_range);
    sample_size_2d = num_legs^2;

    % K_sample: [l_l, l_r, K(60个元素)]
    K_sample_2d = zeros(sample_size_2d, 62);

    % offset_sample: [l_l, l_r, theta_l*, theta_r*, theta_b*, theta_t*]
    offset_sample_2d = zeros(sample_size_2d, 6);

    % input_eq_sample: [l_l, l_r, T_leg_eq, T_t_eq]
    input_eq_sample_2d = zeros(sample_size_2d, 4);

    tic_fit = tic;
    idx = 0;

    for i = 1:num_legs
        for j = 1:num_legs
            idx = idx + 1;

            % ==================== 左腿参数 ====================
            l_l_fit = l_range(i);
            theta_l0_fit = theta0_fun(l_l_fit);
            l_leg_l_com_fit = ld_fun(l_l_fit);
            I_l_fit      = Ileg_fun(l_l_fit);

            % ==================== 右腿参数 ====================
            l_r_fit = l_range(j);
            theta_r0_fit = theta0_fun(l_r_fit);
            l_leg_r_com_fit = ld_fun(l_r_fit);
            I_r_fit      = Ileg_fun(l_r_fit);

            % yaw 惯量拟合
            I_yaw_fit = Iyaw_fun((l_l_fit + l_r_fit)/2);

            % ==================== 参数代换 ====================
            param_subs_fit = {
                m_b, m_b_val;   m_l, m_l_val;   m_r, m_r_val;
                m_wl, m_wl_val; m_wr, m_wr_val;
                I_b, I_b_val;   I_l, I_l_fit;   I_r, I_r_fit;
                I_wl, I_wl_val; I_wr, I_wr_val; I_yaw, I_yaw_fit;
                l_l, l_l_fit;   l_r, l_r_fit;
                l_leg_l_com, l_leg_l_com_fit; l_leg_r_com, l_leg_r_com_fit;
                l_body_com, l_body_com_val;   R, R_val;       R_w, R_w_val;   g, g_val;
                theta_l0, theta_l0_fit; theta_r0, theta_r0_fit; theta_b0, theta_b0_val;
                m_t, m_t_val;   I_t, I_t_val;
                l_tail_mount_h, l_tail_mount_h_val; l_tail_mount_v, l_tail_mount_v_val;
                l_tail_com, l_tail_com_val; delta_t, delta_t_val;
            };

            try
                % ==================== 参数化后的矩阵 ====================
                M_fit = simplify(subs(M_sym, param_subs_fit(:,1), param_subs_fit(:,2)));
                g_fit = simplify(subs(g_sym, param_subs_fit(:,1), param_subs_fit(:,2)));
                B_fit = simplify(subs(B_sym, param_subs_fit(:,1), param_subs_fit(:,2)));

                % ==================== 固定平衡姿态 ====================
                theta_b_fit = 0.0;
                theta_t_fit = 0.0;

                syms T_leg_eq_fit T_t_eq_fit real

                static_subs_fit = {
                    theta_b,    theta_b_fit;
                    theta_t,    theta_t_fit;
                    dtheta_l,   0;
                    dtheta_r,   0;
                    dtheta_b,   0;
                    dtheta_t,   0;
                    dX_b_h,     0;
                    dphi,       0;
                    ddtheta_l,  0;
                    ddtheta_r,  0;
                    ddtheta_b,  0;
                    ddtheta_t,  0;
                    ddX_b_h,    0;
                    ddphi,      0;
                    phi,        0;
                    X_b_h,      0;
                    T_hip_r,   T_leg_eq_fit;
                    T_hip_l,   T_leg_eq_fit;
                    T_motor_wr,  0;
                    T_motor_wl,  0;
                    T_tail,   T_t_eq_fit;
                };

                % ==================== 静态平衡方程 ====================
                g_static_fit = simplify(subs(g_fit, static_subs_fit(:,1), static_subs_fit(:,2)));
                B_static_fit = simplify(subs(B_fit, static_subs_fit(:,1), static_subs_fit(:,2)));

                u_static_fit = [T_leg_eq_fit; T_leg_eq_fit; 0; 0; T_t_eq_fit];
                eq_static_fit_full = g_static_fit - B_static_fit * u_static_fit;
                eq_static_fit = eq_static_fit_full(3:6);  % theta_l, theta_r, theta_b, theta_t

                % ==================== 数值求解平衡点 ====================
                x0_fit = [0.2; 0.2; 0.4; -1.4];

                sol_fit = vpasolve( ...
                    [eq_static_fit(1) == 0, eq_static_fit(2) == 0, ...
                     eq_static_fit(3) == 0, eq_static_fit(4) == 0], ...
                    [theta_l, theta_r, T_leg_eq_fit, T_t_eq_fit], ...
                    x0_fit);

                if isempty(sol_fit)
                    error('vpasolve 未找到解');
                end

                theta_l_fit_val = double(sol_fit.theta_l);
                theta_r_fit_val = double(sol_fit.theta_r);
                T_leg_fit_val   = double(sol_fit.T_leg_eq_fit);
                T_t_fit_val     = double(sol_fit.T_t_eq_fit);

                if ~isscalar(theta_l_fit_val) || ~isscalar(theta_r_fit_val) || ...
                   ~isscalar(T_leg_fit_val) || ~isscalar(T_t_fit_val)
                    error('平衡点结果不是标量');
                end

                % ==================== 静态方程残差检查 ====================
                eq_static_check_fit = double(subs(eq_static_fit_full, ...
                    [theta_l, theta_r, T_leg_eq_fit, T_t_eq_fit], ...
                    [theta_l_fit_val, theta_r_fit_val, T_leg_fit_val, T_t_fit_val]));

                if norm(eq_static_check_fit) > 1e-5
                    error('静态平衡残差过大: %.3e', norm(eq_static_check_fit));
                end

                % ==================== 线性化点代换 ====================
                eq_subs_fit = {
                    theta_l,    theta_l_fit_val;
                    theta_r,    theta_r_fit_val;
                    theta_b,    theta_b_fit;
                    theta_t,    theta_t_fit;
                    dtheta_l,   0;
                    dtheta_r,   0;
                    dtheta_b,   0;
                    dtheta_t,   0;
                    dX_b_h,     0;
                    dphi,       0;
                    phi,        0;
                    X_b_h,      0;
                    T_hip_r,   T_leg_fit_val;
                    T_hip_l,   T_leg_fit_val;
                    T_motor_wr,  0;
                    T_motor_wl,  0;
                    T_tail,   T_t_fit_val;
                };

                % M_eq, B_eq
                M_eq_fit = double(subs(M_fit, eq_subs_fit(:,1), eq_subs_fit(:,2)));
                B_eq_fit = double(subs(B_fit, eq_subs_fit(:,1), eq_subs_fit(:,2)));

                if size(M_eq_fit,1) ~= 6 || size(M_eq_fit,2) ~= 6
                    error('M_eq_fit 不是 6×6');
                end

                % ∂g/∂q
                K_q_fit = sym(zeros(n_q, n_q));
                for ii = 1:n_q
                    for jj = 1:n_q
                        K_q_fit(ii,jj) = diff(g_fit(ii), q(jj));
                    end
                end
                K_q_eq_fit = double(subs(K_q_fit, eq_subs_fit(:,1), eq_subs_fit(:,2)));

                % ∂g/∂dq
                K_dq_fit = sym(zeros(n_q, n_q));
                for ii = 1:n_q
                    for jj = 1:n_q
                        K_dq_fit(ii,jj) = diff(g_fit(ii), dq(jj));
                    end
                end
                K_dq_eq_fit = double(subs(K_dq_fit, eq_subs_fit(:,1), eq_subs_fit(:,2)));

                % ==================== 构造状态空间 ====================
                M_inv_fit = inv(M_eq_fit);

                A_dyn_q_fit  = -M_inv_fit * K_q_eq_fit;
                A_dyn_dq_fit = -M_inv_fit * K_dq_eq_fit;
                B_dyn_fit    =  M_inv_fit * B_eq_fit;

                A_fit_num = zeros(12, 12);
                B_fit_num = zeros(12, 5);

                % 运动学关系
                A_fit_num(1,2)   = 1;
                A_fit_num(3,4)   = 1;
                A_fit_num(5,6)   = 1;
                A_fit_num(7,8)   = 1;
                A_fit_num(9,10)  = 1;
                A_fit_num(11,12) = 1;

                % 动力学部分
                for ii = 1:n_q
                    for jj = 1:n_q
                        A_fit_num(accel_rows(ii), pos_cols(jj)) = A_dyn_q_fit(ii,jj);
                        A_fit_num(accel_rows(ii), vel_cols(jj)) = A_dyn_dq_fit(ii,jj);
                    end
                    B_fit_num(accel_rows(ii), :) = B_dyn_fit(ii, :);
                end

                % ==================== 计算 LQR ====================
                K_fit = lqr(A_fit_num, B_fit_num, lqr_Q, lqr_R);
                K_fit_trans = K_fit';

                % ==================== 存储样本 ====================
                K_sample_2d(idx, 1) = l_l_fit;
                K_sample_2d(idx, 2) = l_r_fit;
                K_sample_2d(idx, 3:62) = K_fit_trans(:)';

                offset_sample_2d(idx, 1) = l_l_fit;
                offset_sample_2d(idx, 2) = l_r_fit;
                offset_sample_2d(idx, 3) = theta_l_fit_val;
                offset_sample_2d(idx, 4) = theta_r_fit_val;
                offset_sample_2d(idx, 5) = theta_b_fit;
                offset_sample_2d(idx, 6) = theta_t_fit;

                input_eq_sample_2d(idx, 1) = l_l_fit;
                input_eq_sample_2d(idx, 2) = l_r_fit;
                input_eq_sample_2d(idx, 3) = T_leg_fit_val;
                input_eq_sample_2d(idx, 4) = T_t_fit_val;

            catch ME
                warning('计算失败 l_l=%.3f, l_r=%.3f: %s', l_l_fit, l_r_fit, ME.message);
                K_sample_2d(idx, :) = NaN;
                offset_sample_2d(idx, :) = NaN;
                input_eq_sample_2d(idx, :) = NaN;
            end

            if mod(idx, 49) == 0 || idx == sample_size_2d
                fprintf('  进度: %d/%d (%.1f 秒)\n', idx, sample_size_2d, toc(tic_fit));
            end
        end
    end

    fprintf('  ✓ 样本计算完成! 耗时: %.2f 秒\n', toc(tic_fit));

    % ==================== 过滤失败样本 ====================
    valid_idx = ~any(isnan(K_sample_2d), 2) & ...
                ~any(isnan(offset_sample_2d), 2) & ...
                ~any(isnan(input_eq_sample_2d), 2);

    K_sample_2d_valid = K_sample_2d(valid_idx, :);
    offset_sample_2d_valid = offset_sample_2d(valid_idx, :);
    input_eq_sample_2d_valid = input_eq_sample_2d(valid_idx, :);

    fprintf('  有效样本数: %d / %d\n', size(K_sample_2d_valid,1), sample_size_2d);

    if size(K_sample_2d_valid,1) < 20
        warning('有效样本过少，拟合结果可能不可靠');
    end

    % ==================== 二维多项式拟合 - K矩阵 ====================
    fprintf('\n正在进行二维多项式拟合 (K矩阵)...\n');

    K_Fit_Coefficients = zeros(60, 6);
    l_l_samples = K_sample_2d_valid(:, 1);
    l_r_samples = K_sample_2d_valid(:, 2);

    for n_fit = 1:60
        K_values = K_sample_2d_valid(:, n_fit+2);
        try
            K_Surface_Fit = fit([l_l_samples, l_r_samples], K_values, 'poly22');
            K_Fit_Coefficients(n_fit, :) = coeffvalues(K_Surface_Fit);
        catch
            warning('K拟合失败: 元素 %d', n_fit);
        end
    end
    fprintf('  ✓ K矩阵拟合完成\n');

    % ==================== 二维多项式拟合 - 平衡点偏移量 ====================
    fprintf('正在进行二维多项式拟合 (平衡点偏移量)...\n');

    Offset_Fit_Coefficients = zeros(4, 6);
    offset_names = {'theta_l_eq', 'theta_r_eq', 'theta_b_eq', 'theta_t_eq'};

    for n_off = 1:4
        offset_values = offset_sample_2d_valid(:, n_off+2);
        try
            Offset_Surface_Fit = fit([l_l_samples, l_r_samples], offset_values, 'poly22');
            Offset_Fit_Coefficients(n_off, :) = coeffvalues(Offset_Surface_Fit);
        catch
            warning('偏移量拟合失败: %s', offset_names{n_off});
        end
    end
    fprintf('  ✓ 偏移量拟合完成\n');

    % ==================== 二维多项式拟合 - 静态输入 ====================
    fprintf('正在进行二维多项式拟合 (静态输入)...\n');

    InputEq_Fit_Coefficients = zeros(2, 6);
    input_eq_names = {'T_leg_eq', 'T_t_eq'};

    for n_in = 1:2
        input_values = input_eq_sample_2d_valid(:, n_in+2);
        try
            InputEq_Surface_Fit = fit([l_l_samples, l_r_samples], input_values, 'poly22');
            InputEq_Fit_Coefficients(n_in, :) = coeffvalues(InputEq_Surface_Fit);
        catch
            warning('静态输入拟合失败: %s', input_eq_names{n_in});
        end
    end
    fprintf('  ✓ 静态输入拟合完成\n\n');

    % ==================== 输出 K 拟合系数 ====================
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// K矩阵拟合系数 K_Fit_Coefficients[60][6] — 拉格朗日法\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²\n');
    fprintf('// 系数顺序: [p00, p10, p01, p20, p11, p02]\n');
    fprintf('// K 维度: 5×12 = 60 个元素\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n\n');

    fprintf('float K_Fit_Coefficients[60][6] = {\n');
    for n_fit = 1:60
        row = ceil(n_fit/12) - 1;
        col = mod(n_fit-1, 12);
        fprintf('    {');
        for c = 1:6
            if c < 6
                fprintf('%12.6gf, ', K_Fit_Coefficients(n_fit,c));
            else
                fprintf('%12.6gf', K_Fit_Coefficients(n_fit,c));
            end
        end
        if n_fit < 60
            fprintf('},  // K[%d][%d]\n', row, col);
        else
            fprintf('}   // K[%d][%d]\n', row, col);
        end
    end
    fprintf('};\n\n');

    % ==================== 输出平衡点偏移量拟合系数 ====================
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// 平衡点偏移量拟合系数 Offset_Fit_Coefficients[4][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// theta_eq(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²\n');
    fprintf('// 顺序: theta_l_eq, theta_r_eq, theta_b_eq, theta_t_eq\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n\n');

    fprintf('float Offset_Fit_Coefficients[4][6] = {\n');
    for n_off = 1:4
        fprintf('    {');
        for c = 1:6
            if c < 6
                fprintf('%12.6gf, ', Offset_Fit_Coefficients(n_off,c));
            else
                fprintf('%12.6gf', Offset_Fit_Coefficients(n_off,c));
            end
        end
        if n_off < 4
            fprintf('},  // %s\n', offset_names{n_off});
        else
            fprintf('}   // %s\n', offset_names{n_off});
        end
    end
    fprintf('};\n\n');

    % ==================== 输出静态输入拟合系数 ====================
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// 静态输入拟合系数 InputEq_Fit_Coefficients[2][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// T_eq(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²\n');
    fprintf('// 顺序: T_leg_eq, T_t_eq\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n\n');

    fprintf('float InputEq_Fit_Coefficients[2][6] = {\n');
    for n_in = 1:2
        fprintf('    {');
        for c = 1:6
            if c < 6
                fprintf('%12.6gf, ', InputEq_Fit_Coefficients(n_in,c));
            else
                fprintf('%12.6gf', InputEq_Fit_Coefficients(n_in,c));
            end
        end
        if n_in < 2
            fprintf('},  // %s\n', input_eq_names{n_in});
        else
            fprintf('}   // %s\n', input_eq_names{n_in});
        end
    end
    fprintf('};\n\n');

    % ==================== 输出使用说明 ====================
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// 使用方法 (C代码示例)\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n');
    fprintf('// 1. 计算平衡点偏移量\n');
    fprintf('//    theta_l_eq = poly22(Offset_Fit_Coefficients[0], l_l, l_r);\n');
    fprintf('//    theta_r_eq = poly22(Offset_Fit_Coefficients[1], l_l, l_r);\n');
    fprintf('//    theta_b_eq = poly22(Offset_Fit_Coefficients[2], l_l, l_r);\n');
    fprintf('//    theta_t_eq = poly22(Offset_Fit_Coefficients[3], l_l, l_r);\n');
    fprintf('//\n');
    fprintf('// 2. 计算静态输入前馈\n');
    fprintf('//    T_leg_eq = poly22(InputEq_Fit_Coefficients[0], l_l, l_r);\n');
    fprintf('//    T_t_eq   = poly22(InputEq_Fit_Coefficients[1], l_l, l_r);\n');
    fprintf('//\n');
    fprintf('// 3. 计算 K 矩阵\n');
    fprintf('//    for row=0:4, for col=0:11:\n');
    fprintf('//      K[row][col] = poly22(K_Fit_Coefficients[row*12+col], l_l, l_r);\n');
    fprintf('//\n');
    fprintf('// 4. 计算控制输出 (5维)\n');
    fprintf('//    X_err = X - X_eq;\n');
    fprintf('//    u_eq = [T_leg_eq, T_leg_eq, 0, 0, T_t_eq];\n');
    fprintf('//    u = u_eq - K * X_err;\n');
    fprintf('// ═══════════════════════════════════════════════════════════\n\n');

    % 保存拟合结果
    save('lqr_fitting_results_lagrange.mat', ...
         'K_sample_2d', 'K_sample_2d_valid', 'K_Fit_Coefficients', ...
         'offset_sample_2d', 'offset_sample_2d_valid', 'Offset_Fit_Coefficients', ...
         'input_eq_sample_2d', 'input_eq_sample_2d_valid', 'InputEq_Fit_Coefficients');

    fprintf('拟合结果已保存到 lqr_fitting_results_lagrange.mat\n');
end

%% ========================================================================
%  保存主结果
%  ========================================================================

fprintf('\n========================================\n');
fprintf('保存主结果...\n');
fprintf('========================================\n');

save('lqr_results_lagrange.mat', ...
     'A_num', 'B_num', 'K', 'S', 'e', ...
     'M_eq', 'B_eq', 'g_eq', 'K_q_eq', 'K_dq_eq', ...
     'theta_l_star', 'theta_r_star', 'theta_b_star', 'theta_t_star', ...
     'T_leg_eq_val', 'T_t_eq_val', ...
     'lqr_Q', 'lqr_R', 'n_states', 'n_u');

fprintf('  结果已保存到 lqr_results_lagrange.mat\n');
fprintf('\n========================================\n');
fprintf('LQR 计算完成! (总耗时: %.1f 秒)\n', toc(tic_total));
fprintf('========================================\n');
