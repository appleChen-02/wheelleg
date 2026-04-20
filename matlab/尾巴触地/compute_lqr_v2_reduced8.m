% compute_lqr_v2.m
% 轮腿机器人 LQR 控制器计算 - 整合版
%
% 功能: 加载动力学方程 → 代入数值参数 → 求解平衡点 → 线性化 → 计算 LQR
%
% 输入: dynamics_new_coords.mat (用广义坐标表示的方程)
% 输出: lqr_results.mat, lqr_fitting_results.mat
%
% ========== 最终状态向量 (8维) ==========
%   X = [X_b^h, V_b^h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r]^T
%
% ========== 完整广义加速度 (6维, 仅用于动力学提取) ==========
%   ddq_full = [ddX_b_h, ddphi, ddtheta_l, ddtheta_r, ddtheta_b, ddtheta_t]^T
%
% ========== 控制向量 (5维) ==========
%   U = [T_{r→b}, T_{l→b}, T_{wr→r}, T_{wl→l}, T_{t→b}]^T

clear; clc;
tic;
fprintf('========================================\n');
fprintf('轮腿机器人 LQR 控制器计算 (8维降阶状态空间版)\n');
fprintf('========================================\n\n');

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
R_w_val = 0.386 / 2;        % 轮距/2 (m)

% ==================== 机体参数 ====================
m_b_val = 6.9;              % 机体质量 (kg)
I_b_val = 59035.925e-6;     % 机体俯仰转动惯量 (kg·m²)
l_b_val = 4.3e-3;           % 机体质心到俯仰轴距离 (m)
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
l_l_d_val = 0.10157;        % 左腿质心到轮轴距离 (m)
l_r_d_val = 0.10157;        % 右腿质心到轮轴距离 (m)
theta_l0_val = 0.582108261; % 左腿偏移角度 (rad)
theta_r0_val = 0.582108261; % 右腿偏移角度 (rad)

% ==================== 尾巴参数 ====================
m_t_val = 0.87;             % 尾巴质量 (kg)
I_t_val = 55967.334e-6;     % 尾巴绕尾电机轴转动惯量 (kg·m²)
a_t_p_val = 0.0;            % 尾电机相对机体质心前向距离
b_t_p_val = 0.089;          % 尾电机相对机体质心下向距离
l_t_c_val = 0.23985;        % 尾电机到尾巴质心距离
delta_t_val = 0.1034;       % 尾巴质心偏置角 (rad)
theta_t_star_val = 0.0;     % 尾巴平衡角 (rad), 水平

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
syms l_b l_l l_r l_l_d l_r_d real
syms theta_l0 theta_r0 theta_b0 real
syms R R_w g real
syms m_t I_t a_t_p b_t_p l_t_c delta_t real

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

eq1_new = simplify(subs(eq1_new, wheel_subs(:,1), wheel_subs(:,2)));
eq2_new = simplify(subs(eq2_new, wheel_subs(:,1), wheel_subs(:,2)));
eq3_new = simplify(subs(eq3_new, wheel_subs(:,1), wheel_subs(:,2)));
eq4_new = simplify(subs(eq4_new, wheel_subs(:,1), wheel_subs(:,2)));
eq5_new = simplify(subs(eq5_new, wheel_subs(:,1), wheel_subs(:,2)));
eq6_new = simplify(subs(eq6_new, wheel_subs(:,1), wheel_subs(:,2)));

fprintf('✓ 轮角加速度已代换为广义坐标\n\n');

%% ========================================
%  Part 4: 提取 M, B, g 矩阵 (符号形式)
%  ========================================

fprintf('========================================\n');
fprintf('Part 4: 提取 M, B, g 矩阵\n');
fprintf('========================================\n\n');

% 注意:
% 这里仍然保留完整 6 维广义加速度, 只用于从完整动力学方程中提取 M/B/g。
% 最终 LQR 状态空间会在 Part 8 中降到 8 维, 不显式包含 theta_b / theta_t。
ddq_full = [ddX_b_h; ddphi; ddtheta_l; ddtheta_r; ddtheta_b; ddtheta_t];
u = [T_r_to_b; T_l_to_b; T_wr_to_r; T_wl_to_l; T_t_to_b];
eqs = {eq1_new, eq2_new, eq3_new, eq4_new, eq5_new, eq6_new};

% 提取 M 矩阵
M_sym = sym(zeros(6,6));
for i = 1:6
    for j = 1:6
        M_sym(i,j) = diff(eqs{i}, ddq_full(j));
    end
end

% 提取 B 矩阵
B_raw = sym(zeros(6,5));
for i = 1:6
    for j = 1:5
        B_raw(i,j) = diff(eqs{i}, u(j));
    end
end
B_sym = -B_raw;

% 提取 g 向量
g_sym = sym(zeros(6,1));
for i = 1:6
    g_sym(i) = -(eqs{i} - M_sym(i,:)*ddq_full - B_raw(i,:)*u);
end

fprintf('✓ M, B, g 符号矩阵已提取\n\n');

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
    l_l_d, l_l_d_val;
    l_r_d, l_r_d_val;
    l_b, l_b_val;
    R, R_val;
    R_w, R_w_val;
    g, g_val;
    theta_l0, theta_l0_val;
    theta_r0, theta_r0_val;
    theta_b0, theta_b0_val;
    m_t, m_t_val;
    I_t, I_t_val;
    a_t_p, a_t_p_val;
    b_t_p, b_t_p_val;
    l_t_c, l_t_c_val;
    delta_t, delta_t_val;
};

% 代入物理参数
M_param = simplify(subs(M_sym, param_subs(:,1), param_subs(:,2)));
B_param = simplify(subs(B_sym, param_subs(:,1), param_subs(:,2)));
g_param = simplify(subs(g_sym, param_subs(:,1), param_subs(:,2)));

eq1_param = simplify(subs(eq1_new, param_subs(:,1), param_subs(:,2)));
eq2_param = simplify(subs(eq2_new, param_subs(:,1), param_subs(:,2)));
eq3_param = simplify(subs(eq3_new, param_subs(:,1), param_subs(:,2)));
eq4_param = simplify(subs(eq4_new, param_subs(:,1), param_subs(:,2)));
eq5_param = simplify(subs(eq5_new, param_subs(:,1), param_subs(:,2)));
eq6_param = simplify(subs(eq6_new, param_subs(:,1), param_subs(:,2)));

fprintf('✓ 物理参数已代入 (M, B, g 现在只含状态变量)\n\n');

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

% 使用完整动力学方程做静态平衡
eq2_static = simplify(subs(eq2_param, static_subs(:,1), static_subs(:,2)));
eq3_static = simplify(subs(eq3_param, static_subs(:,1), static_subs(:,2)));
eq4_static = simplify(subs(eq4_param, static_subs(:,1), static_subs(:,2)));
eq6_static = simplify(subs(eq6_param, static_subs(:,1), static_subs(:,2)));

fprintf('静态方程:\n');
fprintf('eq2_static = \n'); disp(eq2_static);
fprintf('eq3_static = \n'); disp(eq3_static);
fprintf('eq4_static = \n'); disp(eq4_static);
fprintf('eq6_static = \n'); disp(eq6_static);

fprintf('正在联立求解 [theta_l, theta_r, T_leg_eq, T_t_eq] ...\n');

% 初值可按机构经验调整
x0 = [0; 0; 0; 1.44];

disp('eq2_static free vars ='); disp(symvar(eq2_static));
disp('eq3_static free vars ='); disp(symvar(eq3_static));
disp('eq4_static free vars ='); disp(symvar(eq4_static));
disp('eq6_static free vars ='); disp(symvar(eq6_static));

sol_static_num = vpasolve( ...
    [eq2_static == 0, eq3_static == 0, eq4_static == 0, eq6_static == 0], ...
    [theta_l, theta_r, T_leg_eq, T_t_eq], ...
    x0);

if isempty(sol_static_num)
    error('vpasolve 未找到平衡点解，请检查静态方程、参数或初值设置。');
end

theta_l_star = double(sol_static_num.theta_l);
theta_r_star = double(sol_static_num.theta_r);
T_leg_eq_val = double(sol_static_num.T_leg_eq);
T_t_eq_val   = double(sol_static_num.T_t_eq);

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

% 计算 dg/d(theta) 在平衡点
dg_dtheta_l = double(subs(diff(g_param, theta_l), eq_subs_full(:,1), eq_subs_full(:,2)));
dg_dtheta_r = double(subs(diff(g_param, theta_r), eq_subs_full(:,1), eq_subs_full(:,2)));
dg_dtheta_b = double(subs(diff(g_param, theta_b), eq_subs_full(:,1), eq_subs_full(:,2)));
dg_dtheta_t = double(subs(diff(g_param, theta_t), eq_subs_full(:,1), eq_subs_full(:,2)));

fprintf('dg/d(theta_l) 在平衡点:\n');
disp(dg_dtheta_l);

fprintf('dg/d(theta_r) 在平衡点:\n');
disp(dg_dtheta_r);

fprintf('dg/d(theta_b) 在平衡点:\n');
disp(dg_dtheta_b);

fprintf('dg/d(theta_t) 在平衡点:\n');
disp(dg_dtheta_t);

%% ========================================
%  Part 8: 构建 8x8 状态空间矩阵
%  ========================================

fprintf('========================================\n');
fprintf('Part 8: 构建降阶状态空间 A, B 矩阵\n');
fprintf('========================================\n\n');

% 最终状态:
% X = [X_b_h, dX_b_h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r]^T
n = 8;
m_ctrl = 5;

A_num = zeros(n, n);
B_num = zeros(n, m_ctrl);

% 运动学关系 (位置-速度)
A_num(1,2) = 1;   % d(X_b_h)/dt   = dX_b_h
A_num(3,4) = 1;   % d(phi)/dt     = dphi
A_num(5,6) = 1;   % d(theta_l)/dt = dtheta_l
A_num(7,8) = 1;   % d(theta_r)/dt = dtheta_r

% 使用完整动力学矩阵求加速度映射
M_inv = inv(M_eq);

A_dyn_theta_l = M_inv * dg_dtheta_l;   % 6x1
A_dyn_theta_r = M_inv * dg_dtheta_r;   % 6x1

% 只取前4个广义加速度:
% [ddX_b_h; ddphi; ddtheta_l; ddtheta_r]
A_dyn_theta_l_red = A_dyn_theta_l(1:4);
A_dyn_theta_r_red = A_dyn_theta_r(1:4);

% 填充 A 矩阵动力学部分
A_num(2,5) = A_dyn_theta_l_red(1);
A_num(2,7) = A_dyn_theta_r_red(1);

A_num(4,5) = A_dyn_theta_l_red(2);
A_num(4,7) = A_dyn_theta_r_red(2);

A_num(6,5) = A_dyn_theta_l_red(3);
A_num(6,7) = A_dyn_theta_r_red(3);

A_num(8,5) = A_dyn_theta_l_red(4);
A_num(8,7) = A_dyn_theta_r_red(4);

% 输入矩阵：仍由完整动力学得到，但只取前4个加速度方程
B_dyn = M_inv * B_eq;        % 6x5
B_dyn_red = B_dyn(1:4, :);   % 对应 [ddX_b_h; ddphi; ddtheta_l; ddtheta_r]

B_num(2,:) = B_dyn_red(1,:);
B_num(4,:) = B_dyn_red(2,:);
B_num(6,:) = B_dyn_red(3,:);
B_num(8,:) = B_dyn_red(4,:);

fprintf('数值 A 矩阵 (8×8):\n');
disp(A_num);

fprintf('数值 B 矩阵 (8×5):\n');
disp(B_num);

%% ========================================
%  Part 9: 检查可控性
%  ========================================

fprintf('========================================\n');
fprintf('Part 9: 检查系统可控性\n');
fprintf('========================================\n\n');

Co = ctrb(A_num, B_num);
rank_Co = rank(Co);
fprintf('可控性矩阵秩: %d (系统维度: 8)\n', rank_Co);

if rank_Co < 8
    fprintf('  ⚠ 系统不完全可控 (秩=%d < 8)\n', rank_Co);
    fprintf('  物理原因: X_b^h 和 phi 可能形成积分器/弱可控模态\n');
    fprintf('  这不一定阻止 LQR 计算, 但需要关注闭环特征值\n\n');
else
    fprintf('  ✓ 系统完全可控\n\n');
end

%% ========================================
%  Part 10: 设置 LQR 权重
%  ========================================

fprintf('========================================\n');
fprintf('Part 10: 设置 LQR 权重\n');
fprintf('========================================\n\n');

% Q矩阵: 状态权重 (8维)
lqr_Q = diag([120, 10, ...
              4000, 1, ...
              1000, 10, ...
              1000, 10]);

% R矩阵: 控制输入权重 (5维)
lqr_R = diag([1, 1, 10, 10, 10]);

fprintf('Q矩阵 (状态权重, 8×8):\n');
disp(lqr_Q);

fprintf('R矩阵 (控制权重, 5×5):\n');
disp(lqr_R);

%% ========================================
%  Part 11: 计算 LQR 增益
%  ========================================

fprintf('========================================\n');
fprintf('Part 11: 计算 LQR 增益\n');
fprintf('========================================\n\n');

try
    [K, S, e] = lqr(A_num, B_num, lqr_Q, lqr_R);

    fprintf('✓ LQR增益矩阵 K (5×8):\n');
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
    S = [];
    e = [];
end

%% ========================================
%  Part 12: 格式化输出
%  ========================================

fprintf('========================================\n');
fprintf('Part 12: 格式化输出 (C代码)\n');
fprintf('========================================\n\n');

if ~isempty(K)
    fprintf('// LQR增益矩阵 K[5][8]\n');
    fprintf('// 控制律: u = u_eq - K * (X - X_eq)\n');
    fprintf('// 8维状态: [X_b_h, V_b_h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r]\n');
    fprintf('// 平衡点: theta_l*=%.6f, theta_r*=%.6f, theta_b*=%.6f, theta_t*=%.6f (rad)\n\n', ...
            theta_l_star, theta_r_star, theta_b_star, theta_t_star);

    fprintf('float theta_t_eq = %.10ff;  // 尾巴平衡点角度\n', theta_t_star);
    fprintf('float T_t_eq = %.10ff;      // 尾巴静态平衡扭矩\n', T_t_eq_val);
    fprintf('float T_leg_eq = %.10ff;    // 左右腿静态平衡扭矩\n\n', T_leg_eq_val);

    fprintf('float K[5][8] = {\n');
    control_names = {'T_r_to_b', 'T_l_to_b', 'T_wr_to_r', 'T_wl_to_l', 'T_t_to_b'};
    for i = 1:5
        fprintf('    {');
        for j = 1:8
            if j < 8
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

    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 平衡点偏移量 (Equilibrium Point Offsets)\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 控制律: u = u_eq - K * (X - X_eq)\n');
    fprintf('// 其中 X_eq 是 8维状态平衡点向量\n');
    fprintf('// 只有 theta_l, theta_r 有非零平衡点\n');
    fprintf('// 对应索引: theta_l -> X[4], theta_r -> X[6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');

    fprintf('float theta_l_eq = %.10ff;  // 左腿平衡点角度\n', theta_l_star);
    fprintf('float theta_r_eq = %.10ff;  // 右腿平衡点角度\n', theta_r_star);
    fprintf('float theta_b_eq = %.10ff;  // 机体俯仰平衡点角度 (不在LQR状态中)\n', theta_b_star);
    fprintf('float theta_t_eq_const = %.10ff;  // 尾巴平衡点角度 (不在LQR状态中)\n\n', theta_t_star);

    fprintf('// 平衡点角度 (deg) - 仅供参考\n');
    fprintf('// theta_l_eq = %.6f deg\n', rad2deg(theta_l_star));
    fprintf('// theta_r_eq = %.6f deg\n', rad2deg(theta_r_star));
    fprintf('// theta_b_eq = %.6f deg\n', rad2deg(theta_b_star));
    fprintf('// theta_t_eq = %.6f deg\n\n', rad2deg(theta_t_star));

    fprintf('// 完整平衡点状态向量 X_eq[8]\n');
    fprintf('float X_eq[8] = {0.0f, 0.0f, 0.0f, 0.0f, %.10ff, 0.0f, %.10ff, 0.0f};\n', ...
            theta_l_star, theta_r_star);
    fprintf('//               X_b^h  V_b^h  phi   dphi   theta_l  dtheta_l  theta_r  dtheta_r\n\n');
end

%% ========================================
%  Part 13: 腿长拟合功能
%  ========================================

fprintf('========================================\n');
fprintf('Part 13: 腿长拟合功能\n');
fprintf('========================================\n\n');

% 腿长参数查找表
% v 1.0
theta0_fun = @(x) 277.69*x.^4 - 304.33*x.^3 + 130.52*x.^2 - 27.562*x + 2.7591;
ld_fun     = @(x) 1.4205*x.^4 - 2.0644*x.^3 + 1.2526*x.^2 - 0.059*x + 0.0844;
Ileg_fun   = @(x) -0.3588*x.^4 + 0.3595*x.^3 + 0.0659*x.^2 + 0.0318*x + 0.0255;
Iyaw_fun   = @(x) -22.057*x.^4 + 18.074*x.^3 - 6.79*x.^2 + 0.9921*x + 0.2515;

enable_fitting = true;

if enable_fitting
    fprintf('正在计算不同腿长下的 K 矩阵、平衡点偏移量和静态输入...\n\n');

    l_range = 0.15:0.005:0.30;
    num_legs = length(l_range);
    sample_size_2d = num_legs^2;

    % [l_l, l_r, K(5x8=40个)]
    K_sample_2d = zeros(sample_size_2d, 42);

    % [l_l, l_r, theta_l*, theta_r*]
    offset_sample_2d = zeros(sample_size_2d, 4);

    % [l_l, l_r, T_leg_eq, T_t_eq]
    input_eq_sample_2d = zeros(sample_size_2d, 4);

    tic_fit = tic;
    idx = 0;

    for i = 1:num_legs
        for j = 1:num_legs
            idx = idx + 1;

            % ==================== 左腿参数 ====================
            l_l_fit = l_range(i);
            theta_l0_fit = theta0_fun(l_l_fit);
            l_l_d_fit    = ld_fun(l_l_fit);
            I_l_fit      = Ileg_fun(l_l_fit);

            % ==================== 右腿参数 ====================
            l_r_fit = l_range(j);
            theta_r0_fit = theta0_fun(l_r_fit);
            l_r_d_fit    = ld_fun(l_r_fit);
            I_r_fit      = Ileg_fun(l_r_fit);

            % yaw 惯量拟合（沿用旧逻辑）
            I_yaw_fit = Iyaw_fun((l_l_fit + l_r_fit)/2);

            % ==================== 参数代换表 ====================
            param_subs_fit = {
                m_b, m_b_val;   m_l, m_l_val;   m_r, m_r_val;
                m_wl, m_wl_val; m_wr, m_wr_val;
                I_b, I_b_val;   I_l, I_l_fit;   I_r, I_r_fit;
                I_wl, I_wl_val; I_wr, I_wr_val; I_yaw, I_yaw_fit;
                l_l, l_l_fit;   l_r, l_r_fit;
                l_l_d, l_l_d_fit; l_r_d, l_r_d_fit;
                l_b, l_b_val;   R, R_val;       R_w, R_w_val;   g, g_val;
                theta_l0, theta_l0_fit; theta_r0, theta_r0_fit; theta_b0, theta_b0_val;
                m_t, m_t_val;   I_t, I_t_val;
                a_t_p, a_t_p_val; b_t_p, b_t_p_val;
                l_t_c, l_t_c_val; delta_t, delta_t_val;
            };

            try
                % ==================== 参数化后的完整方程 ====================
                eq2_fit = simplify(subs(eq2_new, param_subs_fit(:,1), param_subs_fit(:,2)));
                eq3_fit = simplify(subs(eq3_new, param_subs_fit(:,1), param_subs_fit(:,2)));
                eq4_fit = simplify(subs(eq4_new, param_subs_fit(:,1), param_subs_fit(:,2)));
                eq6_fit = simplify(subs(eq6_new, param_subs_fit(:,1), param_subs_fit(:,2)));

                M_fit = simplify(subs(M_sym, param_subs_fit(:,1), param_subs_fit(:,2)));
                B_fit = simplify(subs(B_sym, param_subs_fit(:,1), param_subs_fit(:,2)));
                g_fit = simplify(subs(g_sym, param_subs_fit(:,1), param_subs_fit(:,2)));

                % ==================== 固定平衡姿态 ====================
                theta_b_fit = 0.0;
                theta_t_fit = 0.0;

                syms T_leg_eq_fit T_t_eq_fit real

                static_subs_fit = {
                    theta_b, theta_b_fit;
                    theta_t, theta_t_fit;
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
                    T_r_to_b, T_leg_eq_fit;
                    T_l_to_b, T_leg_eq_fit;
                    T_wr_to_r, 0;
                    T_wl_to_l, 0;
                    T_t_to_b, T_t_eq_fit
                };

                % ==================== 静态平衡方程 ====================
                eq2_static_fit = simplify(subs(eq2_fit, static_subs_fit(:,1), static_subs_fit(:,2)));
                eq3_static_fit = simplify(subs(eq3_fit, static_subs_fit(:,1), static_subs_fit(:,2)));
                eq4_static_fit = simplify(subs(eq4_fit, static_subs_fit(:,1), static_subs_fit(:,2)));
                eq6_static_fit = simplify(subs(eq6_fit, static_subs_fit(:,1), static_subs_fit(:,2)));

                % ==================== 数值求解平衡点 ====================
                x0_fit = [0.2; 0.2; 0.4; -1.4];

                sol_fit = vpasolve( ...
                    [eq2_static_fit == 0, eq3_static_fit == 0, eq4_static_fit == 0, eq6_static_fit == 0], ...
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
                eq_static_check_fit = double(subs( ...
                    [eq2_static_fit; eq3_static_fit; eq4_static_fit; eq6_static_fit], ...
                    [theta_l, theta_r, T_leg_eq_fit, T_t_eq_fit], ...
                    [theta_l_fit_val, theta_r_fit_val, T_leg_fit_val, T_t_fit_val]));

                if norm(eq_static_check_fit) > 1e-5
                    error('静态平衡残差过大: %.3e', norm(eq_static_check_fit));
                end

                % ==================== 线性化点代换 ====================
                eq_subs_fit = {
                    theta_l, theta_l_fit_val;
                    theta_r, theta_r_fit_val;
                    theta_b, theta_b_fit;
                    theta_t, theta_t_fit;
                    dtheta_l, 0;
                    dtheta_r, 0;
                    dtheta_b, 0;
                    dtheta_t, 0;
                    phi, 0;
                    dphi, 0;
                    X_b_h, 0;
                    dX_b_h, 0;
                    T_r_to_b, T_leg_fit_val;
                    T_l_to_b, T_leg_fit_val;
                    T_wr_to_r, 0;
                    T_wl_to_l, 0;
                    T_t_to_b, T_t_fit_val
                };

                M_eq_fit = double(subs(M_fit, eq_subs_fit(:,1), eq_subs_fit(:,2)));
                B_eq_fit = double(subs(B_fit, eq_subs_fit(:,1), eq_subs_fit(:,2)));

                if size(M_eq_fit,1) ~= 6 || size(M_eq_fit,2) ~= 6
                    error('M_eq_fit 不是 6x6');
                end

                dg_dtheta_l_fit = double(subs(diff(g_fit, theta_l), eq_subs_fit(:,1), eq_subs_fit(:,2)));
                dg_dtheta_r_fit = double(subs(diff(g_fit, theta_r), eq_subs_fit(:,1), eq_subs_fit(:,2)));

                % ==================== 构造 8x8, 8x5 状态空间 ====================
                M_inv_fit = inv(M_eq_fit);

                A_dyn_theta_l_fit = M_inv_fit * dg_dtheta_l_fit;
                A_dyn_theta_r_fit = M_inv_fit * dg_dtheta_r_fit;

                A_fit_num = zeros(8, 8);
                B_fit_num = zeros(8, 5);

                % 运动学关系
                A_fit_num(1,2) = 1;
                A_fit_num(3,4) = 1;
                A_fit_num(5,6) = 1;
                A_fit_num(7,8) = 1;

                % 动力学部分（只取前4个广义加速度）
                A_fit_num(2,5) = A_dyn_theta_l_fit(1);
                A_fit_num(2,7) = A_dyn_theta_r_fit(1);

                A_fit_num(4,5) = A_dyn_theta_l_fit(2);
                A_fit_num(4,7) = A_dyn_theta_r_fit(2);

                A_fit_num(6,5) = A_dyn_theta_l_fit(3);
                A_fit_num(6,7) = A_dyn_theta_r_fit(3);

                A_fit_num(8,5) = A_dyn_theta_l_fit(4);
                A_fit_num(8,7) = A_dyn_theta_r_fit(4);

                B_dyn_fit = M_inv_fit * B_eq_fit;

                B_fit_num(2,:) = B_dyn_fit(1,:);
                B_fit_num(4,:) = B_dyn_fit(2,:);
                B_fit_num(6,:) = B_dyn_fit(3,:);
                B_fit_num(8,:) = B_dyn_fit(4,:);

                % ==================== 计算 LQR ====================
                K_fit = lqr(A_fit_num, B_fit_num, lqr_Q, lqr_R);   % 5x8
                K_fit_trans = K_fit';                              % 8x5

                % ==================== 存储 K 样本 ====================
                K_sample_2d(idx, 1) = l_l_fit;
                K_sample_2d(idx, 2) = l_r_fit;
                K_sample_2d(idx, 3:42) = K_fit_trans(:)';

                % ==================== 存储平衡点偏移量 ====================
                offset_sample_2d(idx, 1) = l_l_fit;
                offset_sample_2d(idx, 2) = l_r_fit;
                offset_sample_2d(idx, 3) = theta_l_fit_val;
                offset_sample_2d(idx, 4) = theta_r_fit_val;

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

    % ==================== 二维多项式拟合 - K矩阵 ====================
    fprintf('\n正在进行二维多项式拟合 (K矩阵)...\n');

    K_Fit_Coefficients = zeros(40, 6);
    l_l_samples = K_sample_2d_valid(:, 1);
    l_r_samples = K_sample_2d_valid(:, 2);

    for n_fit = 1:40
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

    Offset_Fit_Coefficients = zeros(2, 6);
    offset_names = {'theta_l_eq', 'theta_r_eq'};

    for n_off = 1:2
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
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// K矩阵拟合系数 K_Fit_Coefficients[40][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2\n');
    fprintf('// 系数顺序: [p00, p10, p01, p20, p11, p02]\n');
    fprintf('// K 维度: 5x8\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');

    fprintf('float K_Fit_Coefficients[40][6] = {\n');
    for n_fit = 1:40
        row = ceil(n_fit/8) - 1;
        col = mod(n_fit-1, 8);
        fprintf('    {');
        for c = 1:6
            if c < 6
                fprintf('%12.6gf, ', K_Fit_Coefficients(n_fit,c));
            else
                fprintf('%12.6gf', K_Fit_Coefficients(n_fit,c));
            end
        end
        if n_fit < 40
            fprintf('},  // K[%d][%d]\n', row, col);
        else
            fprintf('}   // K[%d][%d]\n', row, col);
        end
    end
    fprintf('};\n\n');

    % ==================== 输出平衡点偏移量拟合系数 ====================
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 平衡点偏移量拟合系数 Offset_Fit_Coefficients[2][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// theta_eq(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2\n');
    fprintf('// 系数顺序: [p00, p10, p01, p20, p11, p02]\n');
    fprintf('// 顺序: theta_l_eq, theta_r_eq\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');

    fprintf('float Offset_Fit_Coefficients[2][6] = {\n');
    for n_off = 1:2
        fprintf('    {');
        for c = 1:6
            if c < 6
                fprintf('%12.6gf, ', Offset_Fit_Coefficients(n_off,c));
            else
                fprintf('%12.6gf', Offset_Fit_Coefficients(n_off,c));
            end
        end
        if n_off < 2
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
    fprintf('//\n');
    fprintf('// 2. 计算静态输入前馈\n');
    fprintf('//    T_leg_eq = poly22(InputEq_Fit_Coefficients[0], l_l, l_r)\n');
    fprintf('//    T_t_eq   = poly22(InputEq_Fit_Coefficients[1], l_l, l_r)\n');
    fprintf('//\n');
    fprintf('// 3. 计算状态误差 (8维)\n');
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

%% ========================================
%  Part 14: 保存当前工况结果
%  ========================================

fprintf('========================================\n');
fprintf('Part 14: 保存结果\n');
fprintf('========================================\n\n');

save('lqr_results.mat', ...
     'A_num', 'B_num', 'K', 'S', 'e', ...
     'M_eq', 'B_eq', 'g_eq', ...
     'theta_l_star', 'theta_r_star', 'theta_b_star', 'theta_t_star', ...
     'T_leg_eq_val', 'T_t_eq_val', ...
     'lqr_Q', 'lqr_R');

fprintf('结果已保存到 lqr_results.mat\n');
fprintf('总耗时: %.2f 秒\n', toc);
