% compute_lqr_v2.m
% 轮腿机器人 LQR 控制器计算 - 整合版

% ！！！！！！！！！！！！！！！！！！！
% ！！！使用前请务必确认好尾巴版本！！！
% ！！！！！！！！！！！！！！！！！！！

% 功能: 加载动力学方程 → 代入数值参数 → 求解平衡点 → 线性化 → 计算 LQR
%
% 输入: dynamics_new_coords.mat (用广义坐标表示的方程)
% 输出: lqr_results.mat, lqr_fitting_results.mat
%
% ========== 状态向量 (10维, 对应推导 §4.3) ==========
%   X = [X_b^h, V_b^h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]^T
%
% ========== 控制向量 (4维, 对应推导 §4.3) ==========
%   U = [T_{r→b}, T_{l→b}, T_{wr→r}, T_{wl→l}]^T

clear; clc;
tic;
fprintf('========================================\n');
fprintf('轮腿机器人 LQR 控制器计算 (整合版)\n');
fprintf('========================================\n\n');

% 仅在需要人工检查符号表达式时开启。常规 LQR 与拟合不需要全局化简。
enable_symbolic_simplification = false;

%% ========================================
%  Part 1: 定义物理参数 (先于符号计算)
%  ========================================

fprintf('========================================\n');
fprintf('Part 1: 定义物理参数\n');
fprintf('========================================\n\n');

% ==================== 物理常数 ====================
g_val = -9.81;              % 重力加速度 (m/s^2)

% ==================== 几何参数 ====================
R_val = 0.130;              % 轮子半径 (m)
R_w_val = 0.386 / 2;  % 轮距/2 (m)

% ==================== 机体参数 ====================
m_b_val = 6.9;            % 机体质量 (kg)
I_b_val = 59035.925e-6;            % 机体俯仰转动惯量 (kg·m²)
l_body_com_val = 4.3e-3;            % 机体质心到俯仰轴距离 (m)
I_yaw_val = 294272.34e-6;          % 整体yaw轴转动惯量 (kg·m²)
theta_b0_val = 0;           % 质心偏移角度 (rad)

% ==================== 轮子参数 ====================
m_wl_val = 0.823;            % 左轮质量 (kg)
m_wr_val = 0.823;            % 右轮质量 (kg)
I_wl_val = 6311.798e-6;     % 左轮转动惯量 (kg·m²)
I_wr_val = 6311.798e-6;     % 右轮转动惯量 (kg·m²)

% ==================== 腿部参数 (默认腿长 0.17m) ====================
l_l_val = 0.17;             % 左腿长度 (m)
l_r_val = 0.17;             % 右腿长度 (m)
m_l_val = 2.2;             % 左腿质量 (kg)
m_r_val = 2.2;             % 右腿质量 (kg)
I_l_val = 0.034231929;           % 左腿转动惯量 (kg·m²)
I_r_val = 0.034231929;           % 右腿转动惯量 (kg·m²)
l_leg_l_com_val = 0.10157;  % 左腿质心到轮轴距离 (m)
l_leg_r_com_val = 0.10157;  % 右腿质心到轮轴距离 (m)
theta_l0_val = 0.582108261;  % 左腿偏移角度 (rad)
theta_r0_val = 0.582108261;  % 右腿偏移角度 (rad)

% ==================== 尾巴参数 v1.0 ====================
% m_t_val = 0.87;                 % 尾巴质量 (kg)
% I_t_val = 55967.334e-6;         % 尾巴绕尾电机轴转动惯量 (kg·m²)
% a_t_p_val = 0.0;             % 先用旧 lT_real 作为前向距离初值
% b_t_p_val = 0.089;             % 若已有真实下向偏置，请替换
% l_t_c_val = 0.23985;         % 尾电机到尾巴质心距离
% delta_t_val = 0.1034;         % 尾巴质心偏置角 (rad)
% theta_t_star_val = 0.0;         % 尾巴平衡角 (rad), 水平
% Iyaw_fun   = @(x) -22.057*x.^4 + 18.074*x.^3 - 6.79*x.^2 + 0.9921*x + 0.2515;

% ==================== 尾巴参数 v1.1 ====================
m_t_val = 0.83;                 % 尾巴质量 (kg)
I_t_val = 29341.743e-6;         % 尾巴绕尾电机轴转动惯量 (kg·m²)
l_tail_mount_h_val = 0.07125;             % 尾电机安装点相对机体质心前向距离
l_tail_mount_v_val = 0.1105;             % 尾电机安装点相对机体质心下向距离
l_tail_com_val = 0.17755;         % 尾电机到尾巴质心距离
delta_t_val = 0.06597;         % 尾巴质心偏置角 (rad)
theta_t_star_val = 0.0;         % 尾巴平衡角 (rad), 水平
Iyaw_fun   = @(x) -6.4624*x.^4 + 6.7339*x.^3 - 3.0029*x.^2 + 0.5202*x + 0.2758;

fprintf('✓ 物理参数设置完成\n\n');

%% ========================================
%  Part 2: 定义符号变量并加载动力学
%  ========================================

fprintf('========================================\n');
fprintf('Part 2: 加载动力学方程\n');
fprintf('========================================\n\n');

% ========== 广义坐标及其导数 ==========
syms X_b_h dX_b_h ddX_b_h real
syms phi dphi ddphi real
syms theta_l dtheta_l ddtheta_l real
syms theta_r dtheta_r ddtheta_r real
syms theta_b dtheta_b ddtheta_b real
syms theta_t dtheta_t ddtheta_t real

% ========== 轮角加速度 ==========
syms ddtheta_wl ddtheta_wr real

% ========== 控制力矩 ==========
syms T_r_to_b T_l_to_b T_wr_to_r T_wl_to_l real
syms T_t_to_b real

% ========== 物理参数符号 ==========
syms m_b m_l m_r m_wl m_wr real
syms I_b I_l I_r I_wl I_wr I_yaw real
syms l_body_com l_l l_r l_leg_l_com l_leg_r_com real
syms theta_l0 theta_r0 theta_b0 real
syms R R_w g real
syms m_t I_t l_tail_mount_h l_tail_mount_v l_tail_com delta_t real

% 加载动力学方程
load('dynamics_new_coords.mat');
fprintf('✓ 已加载 dynamics_new_coords.mat\n\n');

%% ========================================
%  Part 3: 符号处理 - 轮角加速度代换
%  ========================================

fprintf('========================================\n');
fprintf('Part 3: 轮角加速度代换\n');
fprintf('========================================\n\n');

leg_term = (l_r*cos(theta_r)*ddtheta_r + l_l*cos(theta_l)*ddtheta_l)/2 ...
         - (l_r*sin(theta_r)*dtheta_r^2 + l_l*sin(theta_l)*dtheta_l^2)/2;

ddtheta_wr_sub = (ddX_b_h + R_w*ddphi)/R - leg_term/R;
ddtheta_wl_sub = (ddX_b_h - R_w*ddphi)/R - leg_term/R;

wheel_subs = {ddtheta_wr, ddtheta_wr_sub; ddtheta_wl, ddtheta_wl_sub};

eqs_wheel = subs([eq1_new; eq2_new; eq3_new; eq4_new; eq5_new; eq6_new], ...
    wheel_subs(:,1), wheel_subs(:,2));
if enable_symbolic_simplification
    eqs_wheel = simplify(eqs_wheel);
end

[eq1_new, eq2_new, eq3_new, eq4_new, eq5_new, eq6_new] = ...
    deal(eqs_wheel(1), eqs_wheel(2), eqs_wheel(3), eqs_wheel(4), eqs_wheel(5), eqs_wheel(6));

fprintf('✓ 轮角加速度已代换为广义坐标\n\n');

%% ========================================
%  Part 4: 提取 M, B, g 矩阵 (符号形式)
%  ========================================

fprintf('========================================\n');
fprintf('Part 4: 提取 M, B, g 矩阵\n');
fprintf('========================================\n\n');

ddq = [ddX_b_h; ddphi; ddtheta_l; ddtheta_r; ddtheta_b; ddtheta_t];
u = [T_r_to_b; T_l_to_b; T_wr_to_r; T_wl_to_l; T_t_to_b];
eqs = [eq1_new; eq2_new; eq3_new; eq4_new; eq5_new; eq6_new];

% 方程对加速度和输入均为线性；一次 Jacobian 比逐元素 diff 更高效。
M_sym = jacobian(eqs, ddq);
B_raw = jacobian(eqs, u);
B_sym = -B_raw;

% 提取 g 向量。直接置零广义加速度和输入，避免依赖 simplify
% 消去 eqs - M*ddq - B*u 中可能残留的同类项。
g_sym = -subs(eqs, [ddq; u], zeros(numel(ddq) + numel(u), 1));

fprintf('✓ M, B, g 符号矩阵已提取\n');

% ★ 行序对齐: NE方程顺序 [eq1水平, eq2机体, eq3右腿, eq4左腿, eq5Yaw, eq6尾巴]
%                → ddq顺序 [ddX_b_h, ddphi, ddtheta_l, ddtheta_r, ddtheta_b, ddtheta_t]
reorder_NE = [1, 5, 4, 3, 2, 6];
M_sym = M_sym(reorder_NE, :);
g_sym = g_sym(reorder_NE);
B_sym = B_sym(reorder_NE, :);
fprintf('  ★ 已对齐行序: NE [水平,机体,右腿,左腿,Yaw,尾巴] → [ddX,ddφ,ddθl,ddθr,ddθb,ddθt]\n');

fprintf('\n');

%% ========================================
%  Part 5: 代入物理参数数值
%  ========================================

fprintf('========================================\n');
fprintf('Part 5: 代入物理参数数值\n');
fprintf('========================================\n\n');

% 物理参数代换表
param_subs = {
    m_b, m_b_val;
    m_l, m_l_val;
    m_r, m_r_val;
    m_wl, m_wl_val;
    m_wr, m_wr_val;
    I_b, I_b_val;
    I_l, I_l_val;
    I_r, I_r_val;
    I_wl, I_wl_val;
    I_wr, I_wr_val;
    I_yaw, I_yaw_val;
    l_l, l_l_val;
    l_r, l_r_val;
    l_leg_l_com, l_leg_l_com_val;
    l_leg_r_com, l_leg_r_com_val;
    l_body_com, l_body_com_val;
    R, R_val;
    R_w, R_w_val;
    g, g_val;
    theta_l0, theta_l0_val;
    theta_r0, theta_r0_val;
    theta_b0, theta_b0_val;
    m_t, m_t_val;
    I_t, I_t_val;
    l_tail_mount_h, l_tail_mount_h_val;
    l_tail_mount_v, l_tail_mount_v_val;
    l_tail_com, l_tail_com_val;
    delta_t, delta_t_val;
};

% 代入物理参数。eq1/eq5 不参与静态平衡点求解，不再重复构造。
M_param = subs(M_sym, param_subs(:,1), param_subs(:,2));
B_param = subs(B_sym, param_subs(:,1), param_subs(:,2));
g_param = subs(g_sym, param_subs(:,1), param_subs(:,2));
eq_static_param = subs([eq2_new; eq3_new; eq4_new; eq6_new], ...
    param_subs(:,1), param_subs(:,2));
if enable_symbolic_simplification
    M_param = simplify(M_param);
    B_param = simplify(B_param);
    g_param = simplify(g_param);
    eq_static_param = simplify(eq_static_param);
end
[eq2_param, eq3_param, eq4_param, eq6_param] = ...
    deal(eq_static_param(1), eq_static_param(2), eq_static_param(3), eq_static_param(4));

fprintf('✓ 物理参数已代入 (M, B, g 现在只含状态变量)\n\n');

%% ========================================
%  Part 6: 求解平衡点 theta_l*, theta_r*, theta_b*
%  ========================================

fprintf('========================================\n');
fprintf('Part 6: 求解平衡点\n');
fprintf('========================================\n\n');
%% ========================================
%  Part 6: 求解平衡点
%  ========================================

fprintf('========================================\n');
fprintf('Part 6: 求解平衡点\n');
fprintf('========================================\n\n');

% 目标平衡姿态：强制机身和尾巴都水平
theta_b_star = 0.0;
theta_t_star = 0.0;

fprintf('平衡点条件:\n');
fprintf('  theta_b* = 0\n');
fprintf('  theta_t* = 0\n');
fprintf('  所有速度 = 0, 加速度 = 0\n');
fprintf('  求解 theta_l*, theta_r*, T_leg_eq, T_t_eq 使静态方程成立\n\n');

% 未知静态输入（假设左右腿对机身静态力矩相同）
syms T_leg_eq T_t_eq real

% 静态代换：固定姿态、速度、加速度、非尾轮输入
static_subs = {
    theta_b, theta_b_star;
    theta_t, theta_t_star;
    dtheta_l, 0;
    dtheta_r, 0;
    dtheta_b, 0;
    dtheta_t, 0;
    ddtheta_l, 0;
    ddtheta_r, 0;
    ddtheta_b, 0;
    ddtheta_t, 0;
    ddX_b_h, 0;
    ddphi, 0;
    phi, 0;
    dphi, 0;
    X_b_h, 0;
    dX_b_h, 0;
    T_r_to_b, T_leg_eq;
    T_l_to_b, T_leg_eq;
    T_wr_to_r, 0;
    T_wl_to_l, 0;
    T_t_to_b, T_t_eq
};

% 使用“完整动力学方程”做静态平衡，而不是 g_sym。
eq_static = subs([eq2_param; eq3_param; eq4_param; eq6_param], ...
    static_subs(:,1), static_subs(:,2));
if enable_symbolic_simplification
    eq_static = simplify(eq_static);
end
[eq2_static, eq3_static, eq4_static, eq6_static] = ...
    deal(eq_static(1), eq_static(2), eq_static(3), eq_static(4));

fprintf('静态方程:\n');
fprintf('eq2_static = \n'); disp(eq2_static);
fprintf('eq3_static = \n'); disp(eq3_static);
fprintf('eq4_static = \n'); disp(eq4_static);
fprintf('eq6_static = \n'); disp(eq6_static);

fprintf('正在联立求解 [theta_l, theta_r, T_leg_eq, T_t_eq] ...\n');

fprintf('使用数值求解器求解 [theta_l, theta_r, T_leg_eq, T_t_eq] ...\n');

% 初值非常重要，可按你的机构经验调整
% [theta_l, theta_r, T_leg_eq, T_t_eq]
x0 = [0; 0; 0; 1.44];
static_residual_fun = matlabFunction(eq_static, 'Vars', ...
    {theta_l, theta_r, T_leg_eq, T_t_eq});
[x_static, solve_method] = solve_static_equilibrium( ...
    static_residual_fun, x0, eq_static, ...
    [theta_l; theta_r; T_leg_eq; T_t_eq]);
fprintf('  平衡点求解器: %s\n', solve_method);

theta_l_star = x_static(1);
theta_r_star = x_static(2);
T_leg_eq_val = x_static(3);
T_t_eq_val   = x_static(4);

if ~isscalar(theta_l_star) || ~isscalar(theta_r_star) || ...
   ~isscalar(T_leg_eq_val) || ~isscalar(T_t_eq_val)
    error('vpasolve 返回结果不是标量，请检查求解结果。');
end

% 静态方程残差验证
eq_static_check = double(subs( ...
    [eq2_static; eq3_static; eq4_static; eq6_static], ...
    [theta_l, theta_r, T_leg_eq, T_t_eq], ...
    [theta_l_star, theta_r_star, T_leg_eq_val, T_t_eq_val]));

fprintf('\n静态方程残差:\n');
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

%% ========================================
%  Part 7: 在平衡点处线性化
%  ========================================

fprintf('========================================\n');
fprintf('Part 7: 在平衡点处线性化\n');
fprintf('========================================\n\n');

% 完整的平衡点代换
eq_subs_full = {
    theta_l, theta_l_star;
    theta_r, theta_r_star;
    theta_b, theta_b_star;
    theta_t, theta_t_star;
    ddX_b_h, 0;
    ddphi, 0;
    ddtheta_l, 0;
    ddtheta_r, 0;
    ddtheta_b, 0;
    ddtheta_t, 0;
    dtheta_l, 0;
    dtheta_r, 0;
    dtheta_b, 0;
    dtheta_t, 0;
    phi, 0;
    dphi, 0;
    X_b_h, 0;
    dX_b_h, 0;
    T_r_to_b, T_leg_eq_val;
    T_l_to_b, T_leg_eq_val;
    T_wr_to_r, 0;
    T_wl_to_l, 0;
    T_t_to_b, T_t_eq_val;
};

% 平衡点处的 M, B, g 矩阵 (数值)
M_eq = double(subs(M_param, eq_subs_full(:,1), eq_subs_full(:,2)));
B_eq = double(subs(B_param, eq_subs_full(:,1), eq_subs_full(:,2)));
g_eq = double(subs(g_param, eq_subs_full(:,1), eq_subs_full(:,2)));

fprintf('平衡点处 M 矩阵:\n');
disp(M_eq);

fprintf('平衡点处 B 矩阵:\n');
disp(B_eq);

fprintf('平衡点处 g 向量 (验证: 应为0):\n');
disp(g_eq);

% 一次性求出全部角度偏导，避免后续拟合循环反复做符号微分。
dg_dtheta_param = jacobian(g_param, [theta_l, theta_r, theta_b, theta_t]);
dg_dtheta_eq = double(subs(dg_dtheta_param, eq_subs_full(:,1), eq_subs_full(:,2)));
dg_dtheta_l = dg_dtheta_eq(:,1);
dg_dtheta_r = dg_dtheta_eq(:,2);
dg_dtheta_b = dg_dtheta_eq(:,3);
dg_dtheta_t = dg_dtheta_eq(:,4);

fprintf('dg/d(theta_l) 在平衡点:\n');
disp(dg_dtheta_l);

fprintf('dg/d(theta_r) 在平衡点:\n');
disp(dg_dtheta_r);

fprintf('dg/d(theta_b) 在平衡点:\n');
disp(dg_dtheta_b);

%% ========================================
%  Part 8: 构建 10x10 状态空间矩阵
%  ========================================

fprintf('========================================\n');
fprintf('Part 8: 构建状态空间 A, B 矩阵\n');
fprintf('========================================\n\n');

n = 12;
m_ctrl = 5;

A_num = zeros(n, n);
B_num = zeros(n, m_ctrl);

% 运动学关系 (位置-速度)
A_num(1,2) = 1;   % dX_b^h/dt = V_b^h
A_num(3,4) = 1;   % dphi/dt = dphi
A_num(5,6) = 1;   % dtheta_l/dt = dtheta_l
A_num(7,8) = 1;   % dtheta_r/dt = dtheta_r
A_num(9,10) = 1;  % dtheta_b/dt = dtheta_b
A_num(11,12) = 1;  % dtheta_t/dt = dtheta_t

% 用线性方程求解代替显式求逆，数值更稳定。
M_solve_rhs = [dg_dtheta_l, dg_dtheta_r, dg_dtheta_b, dg_dtheta_t, B_eq];
M_solve_result = M_eq \ M_solve_rhs;
A_dyn_theta_l = M_solve_result(:,1);
A_dyn_theta_r = M_solve_result(:,2);
A_dyn_theta_b = M_solve_result(:,3);
A_dyn_theta_t = M_solve_result(:,4);

% 填充A矩阵 (加速度行)
A_num(2,5)  = A_dyn_theta_l(1);  A_num(2,7)  = A_dyn_theta_r(1);  A_num(2,9)  = A_dyn_theta_b(1);  A_num(2,11) = A_dyn_theta_t(1);
A_num(4,5)  = A_dyn_theta_l(2);  A_num(4,7)  = A_dyn_theta_r(2);  A_num(4,9)  = A_dyn_theta_b(2);  A_num(4,11) = A_dyn_theta_t(2);
A_num(6,5)  = A_dyn_theta_l(3);  A_num(6,7)  = A_dyn_theta_r(3);  A_num(6,9)  = A_dyn_theta_b(3);  A_num(6,11) = A_dyn_theta_t(3);
A_num(8,5)  = A_dyn_theta_l(4);  A_num(8,7)  = A_dyn_theta_r(4);  A_num(8,9)  = A_dyn_theta_b(4);  A_num(8,11) = A_dyn_theta_t(4);
A_num(10,5) = A_dyn_theta_l(5);  A_num(10,7) = A_dyn_theta_r(5);  A_num(10,9) = A_dyn_theta_b(5);  A_num(10,11)= A_dyn_theta_t(5);
A_num(12,5) = A_dyn_theta_l(6);  A_num(12,7) = A_dyn_theta_r(6);  A_num(12,9) = A_dyn_theta_b(6);  A_num(12,11)= A_dyn_theta_t(6);

% B矩阵: M\B
B_dyn = M_solve_result(:,5:end);

B_num(2,:) = B_dyn(1,:);
B_num(4,:) = B_dyn(2,:);
B_num(6,:) = B_dyn(3,:);
B_num(8,:) = B_dyn(4,:);
B_num(10,:) = B_dyn(5,:);
B_num(12,:) = B_dyn(6,:);

fprintf('数值 A 矩阵 (12×12):\n');
disp(A_num);

fprintf('数值 B 矩阵 (12×5):\n');
disp(B_num);

%% ========================================
%  Part 9: 检查可控性
%  ========================================

fprintf('========================================\n');
fprintf('Part 9: 检查系统可控性\n');
fprintf('========================================\n\n');

Co = ctrb(A_num, B_num);
rank_Co = rank(Co);
fprintf('可控性矩阵秩: %d (系统维度: 12)\n', rank_Co);

if rank_Co < 12
    fprintf('  ⚠ 系统不完全可控 (秩=%d < 12)\n', rank_Co);
    fprintf('  物理原因: X_b^h 和 phi 是积分器状态\n');
    fprintf('  解决方案: 这是正常的! LQR仍可计算\n\n');
else
    fprintf('  ✓ 系统完全可控\n\n');
end

%% ========================================
%  Part 10: 设置 LQR 权重
%  ========================================

fprintf('========================================\n');
fprintf('Part 10: 设置 LQR 权重\n');
fprintf('========================================\n\n');

% Q矩阵: 状态权重
lqr_Q = diag([120, 400, ...
              4000, 1, ...
              2000, 10, ...
              2000, 10, ...
              6000, 1, ...
              300, 5]);

% R矩阵: 控制输入权重
lqr_R = diag([1, 1, 12, 12, 8]);

fprintf('Q矩阵 (状态权重):\n');
disp(lqr_Q);

fprintf('R矩阵 (控制权重):\n');
disp(lqr_R);

%% ========================================
%  Part 11: 计算 LQR 增益
%  ========================================

fprintf('========================================\n');
fprintf('Part 11: 计算 LQR 增益\n');
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



%% ========================================
%  Part 12: 格式化输出
%  ========================================

fprintf('========================================\n');
fprintf('Part 12: 格式化输出 (C代码)\n');
fprintf('========================================\n\n');

if ~isempty(K)
    fprintf('// LQR增益矩阵 K[5][12]\n');
    fprintf('// 控制律: u = -K * X\n');
    fprintf('// 平衡点: theta_l*=%.6f, theta_r*=%.6f, theta_b*=%.6f (rad)\n\n', ...
            theta_l_star, theta_r_star, theta_b_star);
    fprintf('float theta_t_eq = %.10ff;  // 尾巴平衡点角度\n', theta_t_star);
    fprintf('float T_t_eq = %.10ff;      // 尾巴静态平衡扭矩\n', T_t_eq_val);
    fprintf('float T_leg_eq = %.10ff;    // 左右腿静态平衡扭矩\n\n', T_leg_eq_val);
    fprintf('float K[5][12] = {\n');
    control_names = {'T_r_to_b', 'T_l_to_b', 'T_wr_to_r', 'T_wl_to_l', 'T_t_to_b'};    
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
    
    % 输出平衡点偏移量
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 平衡点偏移量 (Equilibrium Point Offsets)\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 控制律: u = -K * (X - X_eq)\n');
    fprintf('// 其中 X_eq 是平衡点状态向量\n');
    fprintf('// 状态向量中只有 theta_l, theta_r, theta_b 有非零平衡点\n');
    fprintf('// 对应索引: theta_l -> X[4], theta_r -> X[6], theta_b -> X[8]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');
    
    fprintf('// 平衡点角度 (rad)\n');
    fprintf('float theta_l_eq = %.10ff;  // 左腿平衡点角度\n', theta_l_star);
    fprintf('float theta_r_eq = %.10ff;  // 右腿平衡点角度\n', theta_r_star);
    fprintf('float theta_b_eq = %.10ff;  // 机体俯仰平衡点角度\n\n', theta_b_star);
    
    fprintf('// 平衡点角度 (deg) - 仅供参考\n');
    fprintf('// theta_l_eq = %.6f deg\n', rad2deg(theta_l_star));
    fprintf('// theta_r_eq = %.6f deg\n', rad2deg(theta_r_star));
    fprintf('// theta_b_eq = %.6f deg\n\n', rad2deg(theta_b_star));
    
    fprintf('// 完整平衡点状态向量 X_eq[10]\n');
    fprintf('float X_eq[12] = {0.0f, 0.0f, 0.0f, 0.0f, %.10ff, 0.0f, %.10ff, 0.0f, %.10ff, 0.0f, %.10ff, 0.0f};\n', ...
            theta_l_star, theta_r_star, theta_b_star, theta_t_star);
    fprintf('//                X_b^h  V_b^h  phi   dphi   theta_l  dtheta_l  theta_r  dtheta_r  theta_b  dtheta_b\n\n');
end

%% ========================================
%  Part 13: 腿长拟合功能
%  ========================================

fprintf('========================================\n');
fprintf('Part 13: 腿长拟合功能\n');
fprintf('========================================\n\n');

% 腿长参数查找表
theta0_fun = @(x) 277.69*x.^4 - 304.33*x.^3 + 130.52*x.^2 - 27.562*x + 2.7591;
ld_fun     = @(x) 1.4205*x.^4 - 2.0644*x.^3 + 1.2526*x.^2 - 0.059*x + 0.0844;
Ileg_fun   = @(x) -0.3588*x.^4 + 0.3595*x.^3 + 0.0659*x.^2 + 0.0318*x + 0.0255;

% 单点 LQR 是默认工作流。二维拟合会计算 961 个模型，按需显式开启。
enable_fitting = true;

if enable_fitting
    if exist('fsolve', 'file') ~= 2
        error('腿长拟合需要 Optimization Toolbox 中的 fsolve。请安装该工具箱或关闭 enable_fitting。');
    end

    fprintf('正在计算不同腿长下的 K 矩阵、平衡点偏移量和静态输入...\n\n');

    % 拟合中仅以下 9 个参数随腿长变化。其余参数和静态状态一次代入，
    % 将循环内工作全部转换为数值函数调用。
    syms T_leg_eq_fit T_t_eq_fit real
    fit_param_symbols = [l_l, l_r, I_l, I_r, I_yaw, ...
                         l_leg_l_com, l_leg_r_com, theta_l0, theta_r0];
    fit_constant_subs = {
        m_b, m_b_val; m_l, m_l_val; m_r, m_r_val;
        m_wl, m_wl_val; m_wr, m_wr_val;
        I_b, I_b_val; I_wl, I_wl_val; I_wr, I_wr_val;
        l_body_com, l_body_com_val; R, R_val; R_w, R_w_val; g, g_val;
        theta_b0, theta_b0_val; m_t, m_t_val; I_t, I_t_val;
        l_tail_mount_h, l_tail_mount_h_val; l_tail_mount_v, l_tail_mount_v_val;
        l_tail_com, l_tail_com_val; delta_t, delta_t_val;
    };
    fit_static_subs = {
        theta_b, 0; theta_t, 0;
        dtheta_l, 0; dtheta_r, 0; dtheta_b, 0; dtheta_t, 0;
        ddtheta_l, 0; ddtheta_r, 0; ddtheta_b, 0; ddtheta_t, 0;
        ddX_b_h, 0; ddphi, 0; phi, 0; dphi, 0; X_b_h, 0; dX_b_h, 0;
        T_r_to_b, T_leg_eq_fit; T_l_to_b, T_leg_eq_fit;
        T_wr_to_r, 0; T_wl_to_l, 0; T_t_to_b, T_t_eq_fit;
    };
    fit_linear_subs = [fit_constant_subs; fit_static_subs];

    eq_static_fit_sym = subs([eq2_new; eq3_new; eq4_new; eq6_new], ...
        fit_linear_subs(:,1), fit_linear_subs(:,2));
    M_fit_sym = subs(M_sym, fit_linear_subs(:,1), fit_linear_subs(:,2));
    B_fit_sym = subs(B_sym, fit_linear_subs(:,1), fit_linear_subs(:,2));
    dg_dtheta_fit_sym = subs(jacobian(g_sym, [theta_l, theta_r, theta_b, theta_t]), ...
        fit_linear_subs(:,1), fit_linear_subs(:,2));
    if enable_symbolic_simplification
        eq_static_fit_sym = simplify(eq_static_fit_sym);
        M_fit_sym = simplify(M_fit_sym);
        B_fit_sym = simplify(B_fit_sym);
        dg_dtheta_fit_sym = simplify(dg_dtheta_fit_sym);
    end

    static_fit_fun = matlabFunction(eq_static_fit_sym, 'Vars', ...
        {theta_l, theta_r, T_leg_eq_fit, T_t_eq_fit, l_l, l_r, I_l, I_r, ...
         I_yaw, l_leg_l_com, l_leg_r_com, theta_l0, theta_r0});
    M_fit_fun = matlabFunction(M_fit_sym, 'Vars', ...
        {theta_l, theta_r, l_l, l_r, I_l, I_r, I_yaw, ...
         l_leg_l_com, l_leg_r_com, theta_l0, theta_r0});
    B_fit_fun = matlabFunction(B_fit_sym, 'Vars', ...
        {theta_l, theta_r, l_l, l_r, I_l, I_r, I_yaw, ...
         l_leg_l_com, l_leg_r_com, theta_l0, theta_r0});
    dg_dtheta_fit_fun = matlabFunction(dg_dtheta_fit_sym, 'Vars', ...
        {theta_l, theta_r, l_l, l_r, I_l, I_r, I_yaw, ...
         l_leg_l_com, l_leg_r_com, theta_l0, theta_r0});
    
    l_range = 0.15:0.005:0.30;
    num_legs = length(l_range);
    sample_size_2d = num_legs^2;
    
    % [l_l, l_r, K(5x12=60个)]
    K_sample_2d = zeros(sample_size_2d, 62);
    
    % [l_l, l_r, theta_l*, theta_r*, theta_b*, theta_t*]
    offset_sample_2d = zeros(sample_size_2d, 6);
    
    % [l_l, l_r, T_leg_eq, T_t_eq]
    input_eq_sample_2d = zeros(sample_size_2d, 4);
    
    tic_fit = tic;
    idx = 0;
    % 保持原拟合脚本的首点初值；每列再保存上一个左腿长度的解作热启动。
    fit_default_seed = [0.2; 0.2; 0.4; -1.4];
    main_equilibrium_seed = [theta_l_star; theta_r_star; T_leg_eq_val; T_t_eq_val];
    equilibrium_seeds = repmat(fit_default_seed, 1, num_legs);
    
    for i = 1:num_legs
        for j = 1:num_legs
            idx = idx + 1;
            
            % ==================== 左腿参数 ====================
            l_l_fit = l_range(i);
            theta_l0_fit = theta0_fun(l_l_fit);
            l_leg_l_com_fit    = ld_fun(l_l_fit);
            I_l_fit      = Ileg_fun(l_l_fit);
            
            % ==================== 右腿参数 ====================
            l_r_fit = l_range(j);
            theta_r0_fit = theta0_fun(l_r_fit);
            l_leg_r_com_fit    = ld_fun(l_r_fit);
            I_r_fit      = Ileg_fun(l_r_fit);
            
            % yaw 惯量拟合（沿用你的旧逻辑）
            I_yaw_fit = Iyaw_fun((l_l_fit + l_r_fit)/2);
            
            fit_param_values = [l_l_fit, l_r_fit, I_l_fit, I_r_fit, I_yaw_fit, ...
                                l_leg_l_com_fit, l_leg_r_com_fit, theta_l0_fit, theta_r0_fit];
            
            try
                % 循环内只调用数值函数；不再重复 subs、simplify 与符号微分。
                static_fit_residual = @(theta_l_value, theta_r_value, T_leg_value, T_t_value) ...
                    static_fit_fun(theta_l_value, theta_r_value, T_leg_value, T_t_value, fit_param_values(1), ...
                    fit_param_values(2), fit_param_values(3), fit_param_values(4), fit_param_values(5), ...
                    fit_param_values(6), fit_param_values(7), fit_param_values(8), fit_param_values(9));
                fit_initial_guesses = [equilibrium_seeds(:,j), fit_default_seed, main_equilibrium_seed];
                try
                    [x_fit, ~] = solve_static_equilibrium( ...
                        static_fit_residual, fit_initial_guesses, [], []);
                catch numeric_solve_error
                    % 仅在数值多初值都失败时才构造该点的符号方程回退。
                    eq_static_fit_fallback = subs(eq_static_fit_sym, ...
                        fit_param_symbols, fit_param_values);
                    [x_fit, ~] = solve_static_equilibrium( ...
                        static_fit_residual, fit_initial_guesses, ...
                        eq_static_fit_fallback, ...
                        [theta_l; theta_r; T_leg_eq_fit; T_t_eq_fit]);
                    warning('fsolve 未收敛，已使用符号回退 l_l=%.3f, l_r=%.3f: %s', ...
                        l_l_fit, l_r_fit, numeric_solve_error.message);
                end
                equilibrium_seeds(:,j) = x_fit;

                theta_l_fit_val = x_fit(1);
                theta_r_fit_val = x_fit(2);
                T_leg_fit_val   = x_fit(3);
                T_t_fit_val     = x_fit(4);
                
                if ~isscalar(theta_l_fit_val) || ~isscalar(theta_r_fit_val) || ...
                   ~isscalar(T_leg_fit_val) || ~isscalar(T_t_fit_val)
                    error('平衡点结果不是标量');
                end
                
                % ==================== 静态方程残差检查 ====================
                eq_static_check_fit = static_fit_residual( ...
                    theta_l_fit_val, theta_r_fit_val, T_leg_fit_val, T_t_fit_val);
                
                if norm(eq_static_check_fit) > 1e-5
                    error('静态平衡残差过大: %.3e', norm(eq_static_check_fit));
                end
                
                % 已预处理为数值函数；平衡点处直接评估线性化矩阵。
                M_eq_fit = M_fit_fun(theta_l_fit_val, theta_r_fit_val, fit_param_values(1), ...
                    fit_param_values(2), fit_param_values(3), fit_param_values(4), fit_param_values(5), ...
                    fit_param_values(6), fit_param_values(7), fit_param_values(8), fit_param_values(9));
                B_eq_fit = B_fit_fun(theta_l_fit_val, theta_r_fit_val, fit_param_values(1), ...
                    fit_param_values(2), fit_param_values(3), fit_param_values(4), fit_param_values(5), ...
                    fit_param_values(6), fit_param_values(7), fit_param_values(8), fit_param_values(9));
                
                if size(M_eq_fit,1) ~= 6 || size(M_eq_fit,2) ~= 6
                    error('M_eq_fit 不是 6x6');
                end
                
                dg_dtheta_eq_fit = dg_dtheta_fit_fun(theta_l_fit_val, theta_r_fit_val, fit_param_values(1), ...
                    fit_param_values(2), fit_param_values(3), fit_param_values(4), fit_param_values(5), ...
                    fit_param_values(6), fit_param_values(7), fit_param_values(8), fit_param_values(9));
                
                % ==================== 构造 12x12, 12x5 状态空间 ====================
                M_solve_result_fit = M_eq_fit \ [dg_dtheta_eq_fit, B_eq_fit];
                A_dyn_theta_l_fit = M_solve_result_fit(:,1);
                A_dyn_theta_r_fit = M_solve_result_fit(:,2);
                A_dyn_theta_b_fit = M_solve_result_fit(:,3);
                A_dyn_theta_t_fit = M_solve_result_fit(:,4);
                
                A_fit_num = zeros(12, 12);
                B_fit_num = zeros(12, 5);
                
                % 运动学关系
                A_fit_num(1,2) = 1;
                A_fit_num(3,4) = 1;
                A_fit_num(5,6) = 1;
                A_fit_num(7,8) = 1;
                A_fit_num(9,10) = 1;
                A_fit_num(11,12) = 1;
                
                % 动力学部分
                A_fit_num(2,5)  = A_dyn_theta_l_fit(1);  A_fit_num(2,7)  = A_dyn_theta_r_fit(1);  A_fit_num(2,9)  = A_dyn_theta_b_fit(1);  A_fit_num(2,11) = A_dyn_theta_t_fit(1);
                A_fit_num(4,5)  = A_dyn_theta_l_fit(2);  A_fit_num(4,7)  = A_dyn_theta_r_fit(2);  A_fit_num(4,9)  = A_dyn_theta_b_fit(2);  A_fit_num(4,11) = A_dyn_theta_t_fit(2);
                A_fit_num(6,5)  = A_dyn_theta_l_fit(3);  A_fit_num(6,7)  = A_dyn_theta_r_fit(3);  A_fit_num(6,9)  = A_dyn_theta_b_fit(3);  A_fit_num(6,11) = A_dyn_theta_t_fit(3);
                A_fit_num(8,5)  = A_dyn_theta_l_fit(4);  A_fit_num(8,7)  = A_dyn_theta_r_fit(4);  A_fit_num(8,9)  = A_dyn_theta_b_fit(4);  A_fit_num(8,11) = A_dyn_theta_t_fit(4);
                A_fit_num(10,5) = A_dyn_theta_l_fit(5);  A_fit_num(10,7) = A_dyn_theta_r_fit(5);  A_fit_num(10,9) = A_dyn_theta_b_fit(5);  A_fit_num(10,11)= A_dyn_theta_t_fit(5);
                A_fit_num(12,5) = A_dyn_theta_l_fit(6);  A_fit_num(12,7) = A_dyn_theta_r_fit(6);  A_fit_num(12,9) = A_dyn_theta_b_fit(6);  A_fit_num(12,11)= A_dyn_theta_t_fit(6);
                
                B_dyn_fit = M_solve_result_fit(:,5:end);
                
                B_fit_num(2,:)  = B_dyn_fit(1,:);
                B_fit_num(4,:)  = B_dyn_fit(2,:);
                B_fit_num(6,:)  = B_dyn_fit(3,:);
                B_fit_num(8,:)  = B_dyn_fit(4,:);
                B_fit_num(10,:) = B_dyn_fit(5,:);
                B_fit_num(12,:) = B_dyn_fit(6,:);
                
                % ==================== 计算 LQR ====================
                K_fit = lqr(A_fit_num, B_fit_num, lqr_Q, lqr_R);   % 5x12
                K_fit_trans = K_fit';                              % 12x5
                
                % ==================== 存储 K 样本 ====================
                K_sample_2d(idx, 1) = l_l_fit;
                K_sample_2d(idx, 2) = l_r_fit;
                K_sample_2d(idx, 3:62) = K_fit_trans(:)';
                
                % ==================== 存储平衡点偏移量 ====================
                offset_sample_2d(idx, 1) = l_l_fit;
                offset_sample_2d(idx, 2) = l_r_fit;
                offset_sample_2d(idx, 3) = theta_l_fit_val;
                offset_sample_2d(idx, 4) = theta_r_fit_val;
                offset_sample_2d(idx, 5) = 0;
                offset_sample_2d(idx, 6) = 0;
                
                % ==================== 存储静态输入 ====================
                input_eq_sample_2d(idx, 1) = l_l_fit;
                input_eq_sample_2d(idx, 2) = l_r_fit;
                input_eq_sample_2d(idx, 3) = T_leg_fit_val;
                input_eq_sample_2d(idx, 4) = T_t_fit_val;
                
            catch ME
                warning('计算失败 l_l=%.3f, l_r=%.3f: %s', l_l_fit, l_r_fit, ME.message);
                
                % 失败样本置 NaN，后面拟合前再过滤
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
    
    % 对同一个 poly22 设计矩阵一次求解全部 66 个输出，替代 66 次 fit 调用。
    % 系数列顺序与原 coeffvalues(..., 'poly22') 输出保持一致。
    fprintf('\n正在进行二维多项式拟合...\n');
    l_l_samples = K_sample_2d_valid(:, 1);
    l_r_samples = K_sample_2d_valid(:, 2);
    offset_names = {'theta_l_eq', 'theta_r_eq', 'theta_b_eq', 'theta_t_eq'};
    input_eq_names = {'T_leg_eq', 'T_t_eq'};
    poly22_design = [ones(size(l_l_samples)), l_l_samples, l_r_samples, ...
                     l_l_samples.^2, l_l_samples.*l_r_samples, l_r_samples.^2];
    all_fit_values = [K_sample_2d_valid(:,3:62), ...
                      offset_sample_2d_valid(:,3:6), input_eq_sample_2d_valid(:,3:4)];
    all_fit_coefficients = poly22_design \ all_fit_values;
    K_Fit_Coefficients = all_fit_coefficients(:,1:60)';
    Offset_Fit_Coefficients = all_fit_coefficients(:,61:64)';
    InputEq_Fit_Coefficients = all_fit_coefficients(:,65:66)';
    fprintf('  ✓ K矩阵、平衡点偏移量和静态输入拟合完成\n\n');
    
    % ==================== 输出 K 拟合系数 ====================
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// K矩阵拟合系数 K_Fit_Coefficients[60][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2\n');
    fprintf('// 系数顺序: [p00, p10, p01, p20, p11, p02]\n');
    fprintf('// K 维度: 5x12\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');
    
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
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 平衡点偏移量拟合系数 Offset_Fit_Coefficients[4][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// theta_eq(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2\n');
    fprintf('// 系数顺序: [p00, p10, p01, p20, p11, p02]\n');
    fprintf('// 顺序: theta_l_eq, theta_r_eq, theta_b_eq, theta_t_eq\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');
    
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
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 静态输入拟合系数 InputEq_Fit_Coefficients[2][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// T_eq(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2\n');
    fprintf('// 系数顺序: [p00, p10, p01, p20, p11, p02]\n');
    fprintf('// 顺序: T_leg_eq, T_t_eq\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');
    
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
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 使用方法 (C代码示例)\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 1. 计算平衡点偏移量\n');
    fprintf('//    theta_l_eq = poly22(Offset_Fit_Coefficients[0], l_l, l_r)\n');
    fprintf('//    theta_r_eq = poly22(Offset_Fit_Coefficients[1], l_l, l_r)\n');
    fprintf('//    theta_b_eq = poly22(Offset_Fit_Coefficients[2], l_l, l_r)\n');
    fprintf('//    theta_t_eq = poly22(Offset_Fit_Coefficients[3], l_l, l_r)\n');
    fprintf('//\n');
    fprintf('// 2. 计算静态输入前馈\n');
    fprintf('//    T_leg_eq = poly22(InputEq_Fit_Coefficients[0], l_l, l_r)\n');
    fprintf('//    T_t_eq   = poly22(InputEq_Fit_Coefficients[1], l_l, l_r)\n');
    fprintf('//\n');
    fprintf('// 3. 计算状态误差 (12维)\n');
    fprintf('//    X_err = X - X_eq\n');
    fprintf('//\n');
    fprintf('// 4. 计算控制输出 (5维)\n');
    fprintf('//    u_eq = [T_leg_eq, T_leg_eq, 0, 0, T_t_eq]\n');
    fprintf('//    u = u_eq - K * X_err\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');
    
    save('lqr_fitting_results.mat', ...
         'K_sample_2d', 'K_sample_2d_valid', 'K_Fit_Coefficients', ...
         'offset_sample_2d', 'offset_sample_2d_valid', 'Offset_Fit_Coefficients', ...
         'input_eq_sample_2d', 'input_eq_sample_2d_valid', 'InputEq_Fit_Coefficients');
    
    fprintf('拟合结果已保存到 lqr_fitting_results.mat\n');
    fprintf('  包含: K矩阵拟合系数, 平衡点偏移量拟合系数, 静态输入拟合系数\n');
end

function [solution, method] = solve_static_equilibrium(residual_function, initial_guesses, symbolic_equations, symbolic_variables)
% 优先使用双精度 fsolve 多初值求解；主流程失败时才回退符号 vpasolve。
    residual = @(x) reshape(double(residual_function(x(1), x(2), x(3), x(4))), [], 1);
    tolerance = 1e-8;
    if isvector(initial_guesses)
        initial_guesses = initial_guesses(:);
    end

    if exist('fsolve', 'file') == 2
        options = optimoptions('fsolve', 'Display', 'off', ...
            'FunctionTolerance', 1e-10, 'StepTolerance', 1e-10, ...
            'MaxIterations', 400, 'MaxFunctionEvaluations', 2000);
        for guess_index = 1:size(initial_guesses, 2)
            try
                [candidate, fval] = fsolve(residual, initial_guesses(:,guess_index), options);
                % 残差满足阈值即接受，避免因迭代上限的 exitflag 误弃有效解。
                if all(isfinite(candidate)) && norm(fval, inf) <= tolerance
                    solution = candidate(:);
                    method = sprintf('fsolve (initial guess %d)', guess_index);
                    return;
                end
            catch
                % 尝试下一组初值；所有初值失败后再使用符号回退。
            end
        end
    end

    if isempty(symbolic_equations) || isempty(symbolic_variables)
        error('fsolve 未收敛，且没有可用的符号方程作为回退。');
    end

    symbolic_solution = vpasolve(symbolic_equations == 0, symbolic_variables, initial_guesses(:,1));
    if isempty(symbolic_solution)
        error('未找到静态平衡点，请检查初值、参数和方程。');
    end

    solution = zeros(numel(symbolic_variables), 1);
    for index = 1:numel(symbolic_variables)
        solution(index) = double(symbolic_solution.(char(symbolic_variables(index))));
    end
    if norm(residual(solution), inf) > tolerance
        error('静态平衡点残差过大。');
    end
    method = 'vpasolve fallback';
end

%% ========================================
%  Part 14: 保存线性化模型 (供 plot_theoretical_bode 使用)
%  ========================================

if ~isempty(K)
    % 生成带时间戳的文件名，支持多组参数调试
    dateStr = datestr(now, 'yyyymmdd');
    baseName = sprintf('lqr_linear_model_%s', dateStr);

    % 自动递增编号，避免覆盖
    idx = 1;
    while isfile(sprintf('%s_%d.mat', baseName, idx))
        idx = idx + 1;
    end
    saveName = sprintf('%s_%d.mat', baseName, idx);

    if enable_fitting
        save(saveName, 'A_num', 'B_num', 'K','lqr_Q','lqr_R','InputEq_Fit_Coefficients','K_Fit_Coefficients');
    else
        save(saveName, 'A_num', 'B_num', 'K','lqr_Q','lqr_R');
    end

    save(saveName, 'A_num', 'B_num', 'K','lqr_Q','lqr_R');
    fprintf('✓ 线性化模型已保存: %s\n', saveName);
    fprintf('  包含: A_num (12×12), B_num (12×5), K (5×12)\n\n');
end