% compare_dynamics.m
% =========================================================================
% 统一动力学推导与对比 — 牛顿-欧拉法 vs 拉格朗日法
%
% 广义坐标 q = [X_b_h; phi; theta_l; theta_r; theta_b; theta_t]  (6个)
% 控制输入 u = [T_hip_r; T_hip_l; T_motor_wr; T_motor_wl; T_tail] (5个)
%
% 动力学形式: M(q)*ddq + g(q,dq) = B(q)*u
%   M: 6×6  质量/惯性矩阵 (对称)
%   g: 6×1  重力+科氏力+离心力向量
%   B: 6×5  控制输入映射矩阵
%
% 关键约定:
%   alpha_t = theta_t + theta_b - delta_t  (尾巴质心绝对方向角)
%   g = -9.81 (重力加速度，负值，向下)
%
% 输出: comparison_results.mat
%   M_NE, g_NE, B_NE       — NE法 矩阵 (符号)
%   M_Lag, g_Lag, B_Lag     — 拉格朗日法 矩阵 (符号)
%   M_NE_num, M_Lag_num 等 — 数值矩阵
%   M_diff_sym, g_diff_sym, B_diff_sym — 符号差异
% =========================================================================

clear; clc;
fprintf('========================================\n');
fprintf('统一动力学推导与对比\n');
fprintf('  牛顿-欧拉法  vs  拉格朗日法\n');
fprintf('========================================\n\n');

%% ========================================================================
%  Phase 1: 公共符号定义 (统一使用拉格朗日命名体系)
%  ========================================================================

fprintf('========================================\n');
fprintf('Phase 1: 公共符号定义\n');
fprintf('========================================\n\n');

% ---------- 物理常数 ----------
syms g real

% ---------- 几何参数 ----------
syms R R_w real                 % 轮半径, 半轮距

% ---------- 机体参数 ----------
syms m_b I_b l_body_com theta_b0 real

% ---------- 腿部参数 ----------
syms m_l m_r I_l I_r real
syms l_l l_r real               % 腿长 (髋→轮轴)
syms l_leg_l_com l_leg_r_com real  % 腿质心到轮轴距离
syms theta_l0 theta_r0 real     % 腿质心结构偏置角

% ---------- 轮子参数 ----------
syms m_wl m_wr I_wl I_wr real

% ---------- 尾巴参数 ----------
syms m_t I_t l_tail_com delta_t real
syms l_tail_mount_h l_tail_mount_v real   % 尾电机安装点相对机体质心偏移

% ---------- Yaw转动惯量 ----------
syms I_yaw real

% ---------- 广义坐标 q 及其导数 ----------
syms X_b_h    phi    theta_l    theta_r    theta_b    theta_t     real
syms dX_b_h   dphi   dtheta_l   dtheta_r   dtheta_b   dtheta_t   real
syms ddX_b_h  ddphi  ddtheta_l  ddtheta_r  ddtheta_b  ddtheta_t  real

% ---------- 控制输入 ----------
syms T_hip_r T_hip_l T_motor_wr T_motor_wl T_tail real

% ---------- 轮角加速度 (NE法中间变量) ----------
syms ddtheta_wl ddtheta_wr real

% ---------- 向量形式 ----------
q   = [X_b_h;   phi;    theta_l;   theta_r;   theta_b;   theta_t  ];
dq  = [dX_b_h;  dphi;   dtheta_l;  dtheta_r;  dtheta_b;  dtheta_t ];
ddq = [ddX_b_h; ddphi;  ddtheta_l; ddtheta_r; ddtheta_b; ddtheta_t];
u   = [T_hip_r; T_hip_l; T_motor_wr; T_motor_wl; T_tail];

n_q = 6;
n_u = 5;

fprintf('  广义坐标 (6个): X_b_h, phi, theta_l, theta_r, theta_b, theta_t\n');
fprintf('  控制输入 (5个): T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail\n\n');

%% ========================================================================
%  Phase 2: 牛顿-欧拉法推导
%  ========================================================================

fprintf('========================================\n');
fprintf('Phase 2: 牛顿-欧拉法推导\n');
fprintf('========================================\n\n');

% ----- 2.1 NE法专用中间变量 (内力、加速度) -----
fprintf('  Step 2.1: 定义NE中间变量...\n');

% 加速度 (NE中间变量)
syms a_b_h a_b_v real           % 机体加速度 (hip中点)
syms a_l_h a_l_v a_r_h a_r_v real % 腿转轴加速度
syms a_wl_h a_wl_v a_wr_h a_wr_v real % 轮加速度
syms a_t_h a_t_v real           % 尾巴质心加速度
syms a_tp_h a_tp_v real         % 尾电机安装点加速度

% 内力 (仅力, 力矩使用拉格朗日统一名称)
syms F_l_to_b_h F_l_to_b_v real   % 左腿→机体
syms F_r_to_b_h F_r_to_b_v real   % 右腿→机体
syms F_wl_to_l_h F_wl_to_l_v real % 左轮→左腿
syms F_wr_to_r_h F_wr_to_r_v real % 右轮→右腿
syms F_g_to_wl_h F_g_to_wl_v real % 地→左轮
syms F_g_to_wr_h F_g_to_wr_v real % 地→右轮
syms F_t_to_b_h F_t_to_b_v real   % 尾巴→机体

% 几何中间变量
syms alpha_t r_bp_h r_bp_v real   % 尾巴方向角, 安装点相对质心位置

% 力矩别名 (NE旧名 → 拉格朗日统一名, 保持NE方程可读性)
T_l_to_b  = T_hip_l;
T_r_to_b  = T_hip_r;
T_wl_to_l = T_motor_wl;
T_wr_to_r = T_motor_wr;
T_t_to_b  = T_tail;

fprintf('    完成\n\n');

% ----- 2.2 19个原始牛顿-欧拉方程 -----
fprintf('  Step 2.2: 建立19个原始NE方程...\n');

% 尾巴绝对方向角 (关键约定)
alpha_t = theta_t + theta_b - delta_t;

% 尾电机安装点在机体系中的位置 (旋转到世界系)
r_bp_h = l_tail_mount_h * cos(theta_b) + l_tail_mount_v * sin(theta_b);
r_bp_v = l_tail_mount_h * sin(theta_b) - l_tail_mount_v * cos(theta_b);

% --- 机体 (3个) ---
eq_body_h   = F_l_to_b_h + F_r_to_b_h + F_t_to_b_h - m_b * a_b_h;
eq_body_v   = F_l_to_b_v + F_r_to_b_v + F_t_to_b_v + m_b * g - m_b * a_b_v;
eq_body_rot = T_l_to_b + T_r_to_b + T_t_to_b ...
            + (r_bp_h * F_t_to_b_v - r_bp_v * F_t_to_b_h) ...
            + m_b * g * l_body_com * sin(theta_b + theta_b0) ...
            - I_b * ddtheta_b;

% --- 尾巴 (3个) ---
eq_tail_h   = -F_t_to_b_h - m_t * a_t_h;
eq_tail_v   = -F_t_to_b_v + m_t * g - m_t * a_t_v;
eq_tail_rot = -I_t *(ddtheta_t+ddtheta_b) - T_t_to_b + m_t * g * l_tail_com * cos(alpha_t);

% --- 左腿 (3个) ---
eq_leg_l_h   = -F_l_to_b_h + F_wl_to_l_h - m_l * a_l_h;
eq_leg_l_v   = -F_l_to_b_v + F_wl_to_l_v + m_l * g - m_l * a_l_v;
eq_leg_l_rot = -T_l_to_b + T_wl_to_l ...
             + m_l * g * l_leg_l_com * sin(theta_l + theta_l0) ...
             + F_wl_to_l_v * sin(theta_l) * l_l ...
             - F_wl_to_l_h * cos(theta_l) * l_l ...
             - I_l * ddtheta_l;

% --- 右腿 (3个) ---
eq_leg_r_h   = -F_r_to_b_h + F_wr_to_r_h - m_r * a_r_h;
eq_leg_r_v   = -F_r_to_b_v + F_wr_to_r_v + m_r * g - m_r * a_r_v;
eq_leg_r_rot = -T_r_to_b + T_wr_to_r ...
             + m_r * g * l_leg_r_com * sin(theta_r + theta_r0) ...
             + F_wr_to_r_v * sin(theta_r) * l_r ...
             - F_wr_to_r_h * cos(theta_r) * l_r ...
             - I_r * ddtheta_r;

% --- 左轮 (3个) ---
eq_wl_h   = -F_wl_to_l_h + F_g_to_wl_h - m_wl * a_wl_h;
eq_wl_v   = -F_wl_to_l_v + F_g_to_wl_v + m_wl * g - m_wl * a_wl_v;
eq_wl_rot = -T_wl_to_l - F_g_to_wl_h * R - I_wl * ddtheta_wl;

% --- 右轮 (3个) ---
eq_wr_h   = -F_wr_to_r_h + F_g_to_wr_h - m_wr * a_wr_h;
eq_wr_v   = -F_wr_to_r_v + F_g_to_wr_v + m_wr * g - m_wr * a_wr_v;
eq_wr_rot = -T_wr_to_r - F_g_to_wr_h * R - I_wr * ddtheta_wr;

% --- Yaw (1个) ---
eq_yaw = (F_g_to_wr_h - F_g_to_wl_h) * R_w - I_yaw * ddphi;

fprintf('    共 19 个原始方程\n\n');

% ----- 2.3 消去内力 → 6个方程 -----
fprintf('  Step 2.3: 消去内力...\n');

% 地面力从轮转动方程解出
F_g_to_wl_h_expr = -(T_wl_to_l + I_wl * ddtheta_wl) / R;
F_g_to_wr_h_expr = -(T_wr_to_r + I_wr * ddtheta_wr) / R;

% eq1: 整体水平动量 (所有水平方程相加)
eq1_NE = m_b * a_b_h + m_l * a_l_h + m_r * a_r_h + m_t * a_t_h ...
       + m_wl * a_wl_h + m_wr * a_wr_h ...
       + (T_wl_to_l + T_wr_to_r + I_wl * ddtheta_wl + I_wr * ddtheta_wr) / R;

% eq2: 机体转动 (代入尾巴力表达式)
% 从尾巴方程: F_t_to_b_h = -m_t * a_t_h,  F_t_to_b_v = m_t*g - m_t*a_t_v
F_t_to_b_h_from_tail = -m_t * a_t_h;
F_t_to_b_v_from_tail = m_t * g - m_t * a_t_v;   % ★ 已修正: 之前简写为 -m_t*g - m_t*a_t_v

eq2_NE = I_b * ddtheta_b - T_l_to_b - T_r_to_b - T_t_to_b ...
       - (r_bp_h * F_t_to_b_v_from_tail - r_bp_v * F_t_to_b_h_from_tail) ...
       - m_b * g * l_body_com * sin(theta_b + theta_b0);
eq2_NE = expand(eq2_NE);

% eq3: 右腿转动 (使用竖直力近似)
% F_wr_to_r_h = F_g_to_wr_h - m_wr*a_wr_h
F_wr_to_r_h_expr = F_g_to_wr_h_expr - m_wr * a_wr_h;

% 竖直支持力近似: 两轮平分总重 (不含尾巴, 尾巴力已在eq2中单独处理)
F_g_v_each = (m_b + m_l + m_r + m_wl + m_wr) * g / 2;

eq3_NE = I_r * ddtheta_r - T_wr_to_r + T_r_to_b ...
       - m_r * g * l_leg_r_com * sin(theta_r + theta_r0) ...
       - m_wr * a_wr_h * l_r * cos(theta_r) ...
       - (T_wr_to_r + I_wr * ddtheta_wr) * l_r * cos(theta_r) / R ...
       - (m_b * a_b_v + m_l * a_l_v + m_r * a_r_v) * l_r * sin(theta_r) / 2 ...
       - (m_wr - m_b - m_l - m_r - m_wl) * g * l_r * sin(theta_r) / 2;

% eq4: 左腿转动
eq4_NE = I_l * ddtheta_l - T_wl_to_l + T_l_to_b ...
       - m_l * g * l_leg_l_com * sin(theta_l + theta_l0) ...
       - m_wl * a_wl_h * l_l * cos(theta_l) ...
       - (T_wl_to_l + I_wl * ddtheta_wl) * l_l * cos(theta_l) / R ...
       - (m_b * a_b_v + m_l * a_l_v + m_r * a_r_v) * l_l * sin(theta_l) / 2 ...
       - (m_wl - m_b - m_l - m_r - m_wr) * g * l_l * sin(theta_l) / 2;

% eq5: Yaw转动
eq5_NE = I_yaw * ddphi ...
       - R_w / R * (T_wl_to_l + I_wl * ddtheta_wl - T_wr_to_r - I_wr * ddtheta_wr);

% eq6: 尾巴转动
eq6_NE = I_t * (ddtheta_t+ddtheta_b) + T_t_to_b - m_t * g * l_tail_com * cos(alpha_t);

fprintf('    得到 6 个NE最终方程\n\n');

% ----- 2.4 运动学约束 (加速度→广义坐标) -----
fprintf('  Step 2.4: 施加运动学约束...\n');

% 纯滚动约束
a_wr_h_expr = R * ddtheta_wr;
a_wl_h_expr = R * ddtheta_wl;
a_wr_v_expr = sym(0);
a_wl_v_expr = sym(0);

% 腿转轴加速度
a_r_h_expr = a_wr_h_expr + l_r * cos(theta_r) * ddtheta_r - l_r * sin(theta_r) * dtheta_r^2;
a_r_v_expr = a_wr_v_expr - l_r * sin(theta_r) * ddtheta_r - l_r * cos(theta_r) * dtheta_r^2;
a_l_h_expr = a_wl_h_expr + l_l * cos(theta_l) * ddtheta_l - l_l * sin(theta_l) * dtheta_l^2;
a_l_v_expr = a_wl_v_expr - l_l * sin(theta_l) * ddtheta_l - l_l * cos(theta_l) * dtheta_l^2;

% 机体加速度 (hip中点 = 左右腿转轴平均)
a_b_h_expr = (a_r_h_expr + a_l_h_expr) / 2;
a_b_v_expr = (a_r_v_expr + a_l_v_expr) / 2;

% 尾电机安装点加速度
a_tp_h_expr = a_b_h_expr ...
            - r_bp_v * ddtheta_b - r_bp_h * dtheta_b^2;
a_tp_v_expr = a_b_v_expr ...
            + r_bp_h * ddtheta_b - r_bp_v * dtheta_b^2;

% 尾巴质心加速度 (d(alpha_t)/dt = dtheta_t + dtheta_b)
dalpha_t   = dtheta_t + dtheta_b;
ddalpha_t  = ddtheta_t + ddtheta_b;

a_t_h_expr = a_tp_h_expr ...
           - l_tail_com * (ddalpha_t * sin(alpha_t) + dalpha_t^2 * cos(alpha_t));
a_t_v_expr = a_tp_v_expr ...
           + l_tail_com * (ddalpha_t * cos(alpha_t) - dalpha_t^2 * sin(alpha_t));

% 构建代换列表 (加速度→广义坐标表达式)
kin_subs = {
    a_wr_h, a_wr_h_expr;  a_wl_h, a_wl_h_expr;
    a_wr_v, a_wr_v_expr;  a_wl_v, a_wl_v_expr;
    a_r_h,  a_r_h_expr;   a_r_v,  a_r_v_expr;
    a_l_h,  a_l_h_expr;   a_l_v,  a_l_v_expr;
    a_b_h,  a_b_h_expr;   a_b_v,  a_b_v_expr;
    a_tp_h, a_tp_h_expr;  a_tp_v, a_tp_v_expr;
    a_t_h,  a_t_h_expr;   a_t_v,  a_t_v_expr;
};

% 代入6个NE方程
eq1_NE = simplify(subs(eq1_NE, kin_subs(:,1), kin_subs(:,2)));
eq2_NE = simplify(subs(eq2_NE, kin_subs(:,1), kin_subs(:,2)));
eq3_NE = simplify(subs(eq3_NE, kin_subs(:,1), kin_subs(:,2)));
eq4_NE = simplify(subs(eq4_NE, kin_subs(:,1), kin_subs(:,2)));
eq5_NE = simplify(subs(eq5_NE, kin_subs(:,1), kin_subs(:,2)));
eq6_NE = simplify(subs(eq6_NE, kin_subs(:,1), kin_subs(:,2)));

fprintf('    运动学约束已施加\n\n');

% ----- 2.5 轮角加速度代换 → 广义加速度 -----
fprintf('  Step 2.5: 轮角加速度→广义坐标...\n');

leg_accel_term = (l_r * cos(theta_r) * ddtheta_r + l_l * cos(theta_l) * ddtheta_l) / 2;
leg_vel_sq_term = (l_r * sin(theta_r) * dtheta_r^2 + l_l * sin(theta_l) * dtheta_l^2) / 2;
leg_term = leg_accel_term - leg_vel_sq_term;

ddtheta_wr_from_q = (ddX_b_h + R_w * ddphi) / R - leg_term / R;
ddtheta_wl_from_q = (ddX_b_h - R_w * ddphi) / R - leg_term / R;

wheel_subs = {ddtheta_wr, ddtheta_wr_from_q; ddtheta_wl, ddtheta_wl_from_q};

eq1_NE = simplify(subs(eq1_NE, wheel_subs(:,1), wheel_subs(:,2)));
eq2_NE = simplify(subs(eq2_NE, wheel_subs(:,1), wheel_subs(:,2)));
eq3_NE = simplify(subs(eq3_NE, wheel_subs(:,1), wheel_subs(:,2)));
eq4_NE = simplify(subs(eq4_NE, wheel_subs(:,1), wheel_subs(:,2)));
eq5_NE = simplify(subs(eq5_NE, wheel_subs(:,1), wheel_subs(:,2)));
eq6_NE = simplify(subs(eq6_NE, wheel_subs(:,1), wheel_subs(:,2)));

fprintf('    轮角加速度已代换\n\n');

% ----- 2.6 提取 M, g, B 矩阵 (NE) -----
fprintf('  Step 2.6: 提取 M_NE, g_NE, B_NE...\n');

eqs_NE = {eq1_NE, eq2_NE, eq3_NE, eq4_NE, eq5_NE, eq6_NE};

M_NE_sym = sym(zeros(n_q, n_q));
for i = 1:n_q
    for j = 1:n_q
        M_NE_sym(i,j) = diff(eqs_NE{i}, ddq(j));
    end
end

B_raw_NE = sym(zeros(n_q, n_u));
for i = 1:n_q
    for j = 1:n_u
        B_raw_NE(i,j) = diff(eqs_NE{i}, u(j));
    end
end
B_NE_sym = -B_raw_NE;   % eq = M*ddq + g - B*u = 0  →  B*u = M*ddq + g

g_NE_sym = sym(zeros(n_q, 1));
for i = 1:n_q
    g_NE_sym(i) = eqs_NE{i} - M_NE_sym(i,:) * ddq - B_raw_NE(i,:) * u;
end
g_NE_sym = -g_NE_sym;   % NE法g向量取反，与拉格朗日法g的定义对齐

% 使用NE中间变量名(T_l_to_b等)的M/g/B需要统一为新名
% 注: NE中间变量 T_l_to_b, T_wl_to_l 等在消去内力后可能残留在方程中
% 它们在eq1~eq6中作为自由符号出现，提取M/g/B时应已被正确处理

% ★ 行序对齐: NE方程顺序 [eq1水平, eq2机体, eq3右腿, eq4左腿, eq5Yaw, eq6尾巴]
%                → ddq顺序 [ddX_b_h, ddphi, ddtheta_l, ddtheta_r, ddtheta_b, ddtheta_t]
reorder_NE = [1, 5, 4, 3, 2, 6];  % NE行号→标准行号
M_NE_sym = M_NE_sym(reorder_NE, :);
g_NE_sym = g_NE_sym(reorder_NE);
B_NE_sym = B_NE_sym(reorder_NE, :);
fprintf('    ★ 已对齐行序: NE [水平,机体,右腿,左腿,Yaw,尾巴] → 标准 [X,φ,θl,θr,θb,θt]\n');

fprintf('    M_NE: %dx%d, g_NE: %dx1, B_NE: %dx%d\n', n_q, n_q, n_q, n_q, n_u);
fprintf('    M_NE 非零元: %d / %d\n', ...
    nnz(~logical(M_NE_sym == sym(0))), n_q*n_q);

fprintf('\n  NE法推导完成\n\n');

%% ========================================================================
%  Phase 3: 拉格朗日法推导
%  ========================================================================

fprintf('========================================\n');
fprintf('Phase 3: 拉格朗日法推导\n');
fprintf('========================================\n\n');

% ----- 3.1 动能 T -----
fprintf('  Step 3.1: 动能 T...\n');

% 轮心水平速度
v_wl_h = dX_b_h - R_w * dphi - l_l * cos(theta_l) * dtheta_l;
v_wr_h = dX_b_h + R_w * dphi - l_r * cos(theta_r) * dtheta_r;
K_wl = 1/2 * (m_wl + I_wl / R^2) * v_wl_h^2;
K_wr = 1/2 * (m_wr + I_wr / R^2) * v_wr_h^2;

% 髋中点速度
v_hip_h = (v_wl_h + v_wr_h) / 2;
v_hip_v = -(l_l * sin(theta_l) * dtheta_l + l_r * sin(theta_r) * dtheta_r) / 2;

% 左腿质心速度 (质心在轮轴上方 l_leg_l_com 处, 偏移角 theta_l0)
v_legl_h = v_wl_h + l_leg_l_com * cos(theta_l + theta_l0) * dtheta_l;
v_legl_v = l_leg_l_com * sin(theta_l + theta_l0) * dtheta_l;
K_legl = 1/2 * m_l * (v_legl_h^2 + v_legl_v^2) + 1/2 * I_l * dtheta_l^2;

% 右腿质心速度
v_legr_h = v_wr_h + l_leg_r_com * cos(theta_r + theta_r0) * dtheta_r;
v_legr_v = l_leg_r_com * sin(theta_r + theta_r0) * dtheta_r;
K_legr = 1/2 * m_r * (v_legr_h^2 + v_legr_v^2) + 1/2 * I_r * dtheta_r^2;

% 机体质心速度 (质心在hip上方 l_body_com 处)
v_b_h = v_hip_h + l_body_com * cos(theta_b + theta_b0) * dtheta_b;
v_b_v = v_hip_v - l_body_com * sin(theta_b + theta_b0) * dtheta_b;
K_body = 1/2 * m_b * (v_b_h^2 + v_b_v^2) + 1/2 * I_b * dtheta_b^2;

% Yaw动能
K_yaw = 1/2 * I_yaw * dphi^2;

% 尾巴动能
% 尾电机安装点位置 (相对于机体质心, 在世界系下)
r_T_h = l_tail_mount_h * cos(theta_b) + l_tail_mount_v * sin(theta_b);
r_T_v = l_tail_mount_h * sin(theta_b) - l_tail_mount_v * cos(theta_b);

% 安装点速度 = hip速度 + ω_b × r
v_mount_h = v_hip_h + (-r_T_v) * dtheta_b;
v_mount_v = v_hip_v + (  r_T_h) * dtheta_b;

% 尾巴质心绝对方向角 α_t = θ_t + θ_b - δ_t
alpha_t_L = theta_t + theta_b - delta_t;   % d(α_t)/dt = dθ_t + dθ_b
dalpha_t_L = dtheta_t + dtheta_b;

% 尾巴质心速度 = 安装点速度 + 相对转动贡献
v_tail_h = v_mount_h - l_tail_com * sin(alpha_t_L) * dalpha_t_L;
v_tail_v = v_mount_v + l_tail_com * cos(alpha_t_L) * dalpha_t_L;

% 尾巴转动动能使用绝对角速度 dθ_t + dθ_b
K_tail = 1/2 * m_t * (v_tail_h^2 + v_tail_v^2) ...
       + 1/2 * I_t * dalpha_t_L^2;

% 总动能
T = K_wl + K_wr + K_legl + K_legr + K_body + K_yaw + K_tail;

fprintf('    T 项数: %d\n', length(children(expand(T))));

% ----- 3.2 势能 V -----
fprintf('  Step 3.2: 势能 V...\n');

% 髋中点高度
y_hip = R + (l_l * cos(theta_l) + l_r * cos(theta_r)) / 2;

% 轮子
V_wl = m_wl * g * R;
V_wr = m_wr * g * R;

% 腿质心
V_legl = m_l * g * (R + l_leg_l_com * cos(theta_l + theta_l0));
V_legr = m_r * g * (R + l_leg_r_com * cos(theta_r + theta_r0));

% 机体质心
V_body = m_b * g * (y_hip + l_body_com * cos(theta_b + theta_b0));

% 尾巴
y_mount = y_hip + r_T_v;
V_tail = m_t * g * (y_mount + l_tail_com * sin(alpha_t_L));

% 总势能
V = V_wl + V_wr + V_legl + V_legr + V_body + V_tail;

fprintf('    V 项数: %d\n', length(children(expand(V))));

% ----- 3.3 拉格朗日量 -----
fprintf('  Step 3.3: L = T - V...\n');
L = T - V;

% ----- 3.4 广义力 Q -----
fprintf('  Step 3.4: 广义力 Q...\n');

Q = sym(zeros(n_q, 1));

% Q_X_b_h: 轮力矩通过纯滚动产生水平驱动力
Q(1) = -(T_motor_wl + T_motor_wr) / R;

% Q_phi: 左右轮力矩差产生偏航力矩
Q(2) = R_w / R * (T_motor_wl - T_motor_wr);

% Q_theta_l: 左轮力矩 + 左髋反作用力矩
Q(3) = T_motor_wl*(1 + l_l*cos(theta_l)/R) - T_hip_l;

% Q_theta_r: 右轮力矩 + 右髋反作用力矩
Q(4) = T_motor_wr*(1 + l_r*cos(theta_r)/R) - T_hip_r;

% Q_theta_b: 髋力矩 + 尾力矩 (均正向作用于机体俯仰)
Q(5) = T_hip_l + T_hip_r + T_tail;

% Q_theta_t: 尾电机反作用力矩
Q(6) = -T_tail;

% ----- 3.5 欧拉-拉格朗日方程 -----
fprintf('  Step 3.5: 推导欧拉-拉格朗日方程...\n');

eqs_Lag = sym(zeros(n_q, 1));

for k = 1:n_q
    % 广义动量 p_k = ∂L/∂(dq_k)
    p_k = diff(L, dq(k));

    % 全时间导数 dp_k/dt = Σ ∂p_k/∂q_j * dq_j  +  Σ ∂p_k/∂dq_j * ddq_j
    dpk_dt = sym(0);
    for j = 1:n_q
        dpk_dt = dpk_dt + diff(p_k, q(j)) * dq(j);
        dpk_dt = dpk_dt + diff(p_k, dq(j)) * ddq(j);
    end

    % ∂L/∂q_k
    dL_dqk = diff(L, q(k));

    % 欧拉-拉格朗日: d/dt(∂L/∂q̇_k) - ∂L/∂q_k = Q_k
    eqs_Lag(k) = dpk_dt - dL_dqk - Q(k);

    if mod(k, 3) == 0
        fprintf('    进度: %d/6\n', k);
    end
end

fprintf('    推导完成\n');

% ----- 3.6 提取 M, g, B 矩阵 (Lagrange) -----
fprintf('  Step 3.6: 提取 M_Lag, g_Lag, B_Lag...\n');

M_Lag_sym = sym(zeros(n_q, n_q));
for i = 1:n_q
    for j = 1:n_q
        M_Lag_sym(i,j) = diff(eqs_Lag(i), ddq(j));
    end
end

B_raw_Lag = sym(zeros(n_q, n_u));
for i = 1:n_q
    for j = 1:n_u
        B_raw_Lag(i,j) = diff(eqs_Lag(i), u(j));
    end
end
B_Lag_sym = -B_raw_Lag;

g_Lag_sym = sym(zeros(n_q, 1));
for i = 1:n_q
    g_Lag_sym(i) = eqs_Lag(i) - M_Lag_sym(i,:) * ddq - B_raw_Lag(i,:) * u;
end

fprintf('    M_Lag: %dx%d, g_Lag: %dx1, B_Lag: %dx%d\n', n_q, n_q, n_q, n_q, n_u);
fprintf('    M_Lag 非零元: %d / %d\n', ...
    nnz(~logical(M_Lag_sym == sym(0))), n_q*n_q);

fprintf('\n  拉格朗日法推导完成\n\n');

%% ========================================================================
%  Phase 4: 符号层面对比
%  ========================================================================

fprintf('========================================\n');
fprintf('Phase 4: 符号层面对比 M, g, B\n');
fprintf('========================================\n\n');

fprintf('正在化简差异矩阵 (可能需要几分钟)...\n');

% --- M 矩阵对比 ---
fprintf('\n--- M 矩阵 (质量/惯性) ---\n');
M_diff = simplify(M_NE_sym - M_Lag_sym);
diff_count_M = 0;
M_diff_info = {};
for i = 1:n_q
    for j = 1:n_q
        if ~isequal(M_diff(i,j), sym(0))
            diff_count_M = diff_count_M + 1;
            M_diff_info{end+1} = sprintf('    M(%d,%d): %s', i, j, char(M_diff(i,j)));
        end
    end
end
if diff_count_M == 0
    fprintf('  ✓ M 矩阵完全一致\n');
else
    fprintf('  ✗ M 矩阵存在 %d 处差异 (共 %d 个元素)\n', diff_count_M, n_q*n_q);
    for k = 1:min(10, length(M_diff_info))
        fprintf('%s\n', M_diff_info{k});
    end
    if length(M_diff_info) > 10
        fprintf('    ... (还有 %d 处差异未显示)\n', length(M_diff_info)-10);
    end
end

% --- g 向量对比 ---
fprintf('\n--- g 向量 (重力+科氏力+离心力) ---\n');
g_diff = simplify(g_NE_sym - g_Lag_sym);
diff_count_g = 0;
g_diff_info = {};
for i = 1:n_q
    if ~isequal(g_diff(i), sym(0))
        diff_count_g = diff_count_g + 1;
        g_diff_info{end+1} = sprintf('    g(%d): %s', i, char(g_diff(i)));
    end
end
if diff_count_g == 0
    fprintf('  ✓ g 向量完全一致\n');
else
    fprintf('  ✗ g 向量存在 %d 处差异\n', diff_count_g);
    for k = 1:length(g_diff_info)
        fprintf('%s\n', g_diff_info{k});
    end
end

% --- B 矩阵对比 ---
fprintf('\n--- B 矩阵 (控制输入映射) ---\n');
B_diff = simplify(B_NE_sym - B_Lag_sym);
diff_count_B = 0;
B_diff_info = {};
for i = 1:n_q
    for j = 1:n_u
        if ~isequal(B_diff(i,j), sym(0))
            diff_count_B = diff_count_B + 1;
            B_diff_info{end+1} = sprintf('    B(%d,%d): %s', i, j, char(B_diff(i,j)));
        end
    end
end
if diff_count_B == 0
    fprintf('  ✓ B 矩阵完全一致\n');
else
    fprintf('  ✗ B 矩阵存在 %d 处差异 (共 %d 个元素)\n', diff_count_B, n_q*n_u);
    for k = 1:length(B_diff_info)
        fprintf('%s\n', B_diff_info{k});
    end
end

% --- 汇总 ---
fprintf('\n--- 符号对比汇总 ---\n');
total_diff = diff_count_M + diff_count_g + diff_count_B;
if total_diff == 0
    fprintf('  ✓✓✓ 拉格朗日法与牛顿-欧拉法完全等价!\n');
else
    fprintf('  总计 %d 处差异 (M:%d, g:%d, B:%d)\n', ...
        total_diff, diff_count_M, diff_count_g, diff_count_B);
    fprintf('  (部分差异可能源自NE法腿部方程的竖直力近似)\n');
end
fprintf('\n');

%% ========================================================================
%  Phase 5: 数值代入对比
%  ========================================================================

fprintf('========================================\n');
fprintf('Phase 5: 数值代入对比\n');
fprintf('========================================\n\n');

% ----- 5.1 物理参数数值 -----
param_val = struct( ...
    'g',               -9.81, ...
    'R',               0.130, ...
    'R_w',             0.386 / 2, ...
    'm_b',             6.9, ...
    'I_b',             59035.925e-6, ...
    'l_body_com',      4.3e-3, ...
    'I_yaw',           294272.34e-6, ...
    'theta_b0',        0, ...
    'm_wl',            0.823, ...
    'm_wr',            0.823, ...
    'I_wl',            6311.798e-6, ...
    'I_wr',            6311.798e-6, ...
    'l_l',             0.17, ...
    'l_r',             0.17, ...
    'm_l',             2.2, ...
    'm_r',             2.2, ...
    'I_l',             0.034231929, ...
    'I_r',             0.034231929, ...
    'l_leg_l_com',     0.10157, ...
    'l_leg_r_com',     0.10157, ...
    'theta_l0',        0.582108261, ...
    'theta_r0',        0.582108261, ...
    'm_t',             0.83, ...
    'I_t',             29341.743e-6, ...
    'l_tail_com',      0.17755, ...
    'delta_t',         0.06597, ...
    'l_tail_mount_h',  0.07125, ...
    'l_tail_mount_v',  0.1105 ...
);

% 参数符号→数值 代换
param_names = fieldnames(param_val);
param_subs_num = cell(length(param_names), 2);
for k = 1:length(param_names)
    param_subs_num{k,1} = eval(param_names{k});
    param_subs_num{k,2} = param_val.(param_names{k});
end

% ----- 5.2 测试姿态 -----
theta_l_test = 0.3;    % rad  (~17.2°)
theta_r_test = 0.3;    % rad
theta_b_test = 0.0;    % rad
theta_t_test = 0.0;    % rad
phi_test     = 0.0;

fprintf('测试姿态:\n');
fprintf('  theta_l = %.4f rad (%.2f deg)\n', theta_l_test, rad2deg(theta_l_test));
fprintf('  theta_r = %.4f rad (%.2f deg)\n', theta_r_test, rad2deg(theta_r_test));
fprintf('  theta_b = %.4f rad (%.2f deg)\n', theta_b_test, rad2deg(theta_b_test));
fprintf('  theta_t = %.4f rad (%.2f deg)\n', theta_t_test, rad2deg(theta_t_test));
fprintf('  所有速度 = 0, 所有控制 = 0\n\n');

% 姿态+速度+控制 代换
state_subs_num = {
    theta_l, theta_l_test;  theta_r, theta_r_test;
    theta_b, theta_b_test;  theta_t, theta_t_test;
    phi,     phi_test;      X_b_h, 0;
    dtheta_l, 0;  dtheta_r, 0;  dtheta_b, 0;  dtheta_t, 0;
    dX_b_h, 0;    dphi, 0;
    % 控制=0 (静态对比)
    T_hip_r, 0; T_hip_l, 0;
    T_motor_wr, 0; T_motor_wl, 0;
    T_tail, 0;
    % 加速度=0 (提取g时的兜底)
    ddX_b_h, 0; ddphi, 0;
    ddtheta_l, 0; ddtheta_r, 0; ddtheta_b, 0; ddtheta_t, 0;
};

% ----- 5.3 数值计算 -----
fprintf('正在计算数值矩阵...\n');

% NE 数值
M_NE_num = double(subs(subs(M_NE_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));
g_NE_num = double(subs(subs(g_NE_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));
B_NE_num = double(subs(subs(B_NE_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));

% Lagrange 数值
M_Lag_num = double(subs(subs(M_Lag_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));
g_Lag_num = double(subs(subs(g_Lag_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));
B_Lag_num = double(subs(subs(B_Lag_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));

fprintf('  完成\n\n');

% ----- 5.4 数值对比输出 -----
fprintf('========================================\n');
fprintf('数值结果\n');
fprintf('========================================\n\n');

% --- M 矩阵 ---
fprintf('═══════════════════════════════════════════\n');
fprintf('  M 矩阵 (6×6, 行=X_b_h,phi,θ_l,θ_r,θ_b,θ_t)\n');
fprintf('═══════════════════════════════════════════\n\n');

fprintf('--- NE 法 M 矩阵 ---\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.4f ', M_NE_num(i,j));
    end
    fprintf('\n');
end

fprintf('\n--- 拉格朗日法 M 矩阵 ---\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.4f ', M_Lag_num(i,j));
    end
    fprintf('\n');
end

M_num_diff = M_NE_num - M_Lag_num;
fprintf('\n--- M 差异 (NE - Lagrange) ---\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.2e ', M_num_diff(i,j));
    end
    fprintf('\n');
end
fprintf('  ||M_NE - M_Lag||_inf = %.4e\n', norm(M_num_diff, 'inf'));
fprintf('  ||M_NE - M_Lag||_F   = %.4e\n', norm(M_num_diff, 'fro'));

% --- g 向量 ---
fprintf('\n═══════════════════════════════════════════\n');
fprintf('  g 向量 (6×1)\n');
fprintf('═══════════════════════════════════════════\n\n');

fprintf('  行号 | %12s | %12s | %12s\n', 'NE', 'Lagrange', '差异');
fprintf('  -----|---------------|---------------|-------------\n');
for i = 1:6
    fprintf('  %4d | %12.6f | %12.6f | %12.4e\n', ...
        i, g_NE_num(i), g_Lag_num(i), g_NE_num(i) - g_Lag_num(i));
end
g_num_diff = g_NE_num - g_Lag_num;
fprintf('\n  ||g_NE - g_Lag||_inf = %.4e\n', norm(g_num_diff, 'inf'));

% --- B 矩阵 ---
fprintf('\n═══════════════════════════════════════════\n');
fprintf('  B 矩阵 (6×5, 列=T_hip_r,T_hip_l,T_motor_wr,T_motor_wl,T_tail)\n');
fprintf('═══════════════════════════════════════════\n\n');

fprintf('--- NE 法 B 矩阵 ---\n');
for i = 1:6
    fprintf('  ');
    for j = 1:5
        fprintf('%10.4f ', B_NE_num(i,j));
    end
    fprintf('\n');
end

fprintf('\n--- 拉格朗日法 B 矩阵 ---\n');
for i = 1:6
    fprintf('  ');
    for j = 1:5
        fprintf('%10.4f ', B_Lag_num(i,j));
    end
    fprintf('\n');
end

B_num_diff = B_NE_num - B_Lag_num;
fprintf('\n--- B 差异 (NE - Lagrange) ---\n');
for i = 1:6
    fprintf('  ');
    for j = 1:5
        fprintf('%10.2e ', B_num_diff(i,j));
    end
    fprintf('\n');
end
fprintf('  ||B_NE - B_Lag||_inf = %.4e\n', norm(B_num_diff, 'inf'));
fprintf('  ||B_NE - B_Lag||_F   = %.4e\n', norm(B_num_diff, 'fro'));

% ----- 5.5 综合判断 -----
fprintf('\n========================================\n');
fprintf('数值对比结论\n');
fprintf('========================================\n\n');

threshold = 1e-10;
M_match = norm(M_num_diff, 'inf') < threshold;
g_match = norm(g_num_diff, 'inf') < threshold;
B_match = norm(B_num_diff, 'inf') < threshold;

fprintf('  M 矩阵: ');
if M_match
    fprintf('✓ 一致 (||diff||_inf = %.2e < %.0e)\n', norm(M_num_diff,'inf'), threshold);
else
    fprintf('✗ 不一致 (||diff||_inf = %.2e)\n', norm(M_num_diff,'inf'));
end

fprintf('  g 向量: ');
if g_match
    fprintf('✓ 一致 (||diff||_inf = %.2e < %.0e)\n', norm(g_num_diff,'inf'), threshold);
else
    fprintf('✗ 不一致 (||diff||_inf = %.2e)\n', norm(g_num_diff,'inf'));
end

fprintf('  B 矩阵: ');
if B_match
    fprintf('✓ 一致 (||diff||_inf = %.2e < %.0e)\n', norm(B_num_diff,'inf'), threshold);
else
    fprintf('✗ 不一致 (||diff||_inf = %.2e)\n', norm(B_num_diff,'inf'));
end

if M_match && g_match && B_match
    fprintf('\n  ✓✓✓ 数值验证完全通过!\n');
else
    n_bad = ~M_match + ~g_match + ~B_match;
    fprintf('\n  ⚠ %d 个矩阵/向量存在数值差异\n', n_bad);
    fprintf('  可能原因:\n');
    fprintf('    1. NE法腿部转动方程使用了竖直力平均分配近似\n');
    fprintf('    2. 两种方法对尾巴和腿的质心建模精细度不同\n');
end

%% ========================================================================
%  Phase 6: 自检
%  ========================================================================

fprintf('\n========================================\n');
fprintf('Phase 6: 自检\n');
fprintf('========================================\n\n');

% 检查 M 矩阵对称性 (拉格朗日法的M必须对称)
M_Lag_diff_sym = simplify(M_Lag_sym - M_Lag_sym.');
if isequal(M_Lag_diff_sym, sym(zeros(n_q, n_q)))
    fprintf('  ✓ M_Lag 矩阵对称 (符合拉格朗日力学预期)\n');
else
    fprintf('  ✗ M_Lag 矩阵不对称! 请检查 T 表达式\n');
end

% 检查 NE 的 M 是否对称
M_NE_diff_sym = simplify(M_NE_sym - M_NE_sym.');
if isequal(M_NE_diff_sym, sym(zeros(n_q, n_q)))
    fprintf('  ✓ M_NE  矩阵对称\n');
else
    fprintf('  ✗ M_NE  矩阵不对称 (NE法不保证对称性, 但合理模型应近似对称)\n');
end

% 检查 B*u 与 Q 一致性 (拉格朗日法)
BQ_Lag = simplify(B_Lag_sym * u - Q);
BQ_Lag_num = double(subs(subs(BQ_Lag, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));
fprintf('\n  B_qLag * u - Q  (应为零向量):\n');
for i = 1:n_q
    fprintf('  [%d] 符号: %s\n', i, char(BQ_Lag(i)));
end

%% ========================================================================
%  Phase 7: 状态空间 A, B_ss 矩阵 (dx = A*x + B_ss*u)
%  ========================================================================

fprintf('\n========================================\n');
fprintf('Phase 7: 状态空间 A, B_ss 矩阵\n');
fprintf('========================================\n\n');

fprintf('  状态向量 x = [q; dq] (12x1)\n');
fprintf('  控制输入 u = [T_hip_r; T_hip_l; T_motor_wr; T_motor_wl; T_tail] (5x1)\n');
fprintf('  系统形式: dx = A*x + B_ss*u\n');
fprintf('  线性化原理:\n');
fprintf('    ddq = M^{-1} * (B*u - g)\n');
fprintf('    A = [0_{6x6},       I_{6x6};\n');
fprintf('         -M^{-1}*Jg_q,  -M^{-1}*Jg_dq]\n');
fprintf('    B_ss = [0_{6x5}; M^{-1}*B]\n');
fprintf('  线性化点: dq=0, 测试姿态, u=0\n\n');

% ----- 7.1 符号Jacobian -----
fprintf('  Step 7.1: 计算符号 Jacobian ∂g/∂q 和 ∂g/∂dq...\n');

% 拉格朗日法
Jg_q_Lag = jacobian(g_Lag_sym, q);    % 6×6
Jg_dq_Lag = jacobian(g_Lag_sym, dq);  % 6×6

% NE法
Jg_q_NE = jacobian(g_NE_sym, q);
Jg_dq_NE = jacobian(g_NE_sym, dq);

fprintf('    完成\n\n');

% ----- 7.2 数值代入 -----
fprintf('  Step 7.2: 数值代入...\n');

Jg_q_Lag_num = double(subs(subs(Jg_q_Lag, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));
Jg_dq_Lag_num = double(subs(subs(Jg_dq_Lag, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));

Jg_q_NE_num = double(subs(subs(Jg_q_NE, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));
Jg_dq_NE_num = double(subs(subs(Jg_dq_NE, param_subs_num(:,1), param_subs_num(:,2)), ...
    state_subs_num(:,1), state_subs_num(:,2)));

M_Lag_inv_num = inv(M_Lag_num);
M_NE_inv_num  = inv(M_NE_num);

fprintf('    完成\n\n');

% ----- 7.3 构造 A 矩阵 -----
fprintf('  Step 7.3: 构造 A 矩阵 (12×12)...\n');

% 拉格朗日法
A_Lag = zeros(12, 12);
A_Lag(1:6, 7:12) = eye(6);
A_Lag(7:12, 1:6) = -M_Lag_inv_num * Jg_q_Lag_num;
A_Lag(7:12, 7:12) = -M_Lag_inv_num * Jg_dq_Lag_num;

% NE法
A_NE = zeros(12, 12);
A_NE(1:6, 7:12) = eye(6);
A_NE(7:12, 1:6) = -M_NE_inv_num * Jg_q_NE_num;
A_NE(7:12, 7:12) = -M_NE_inv_num * Jg_dq_NE_num;

% ----- 7.4 构造 B_ss 矩阵 -----
fprintf('  Step 7.4: 构造 B_ss 矩阵 (12×5)...\n');

B_ss_Lag = zeros(12, 5);
B_ss_Lag(7:12, :) = M_Lag_inv_num * B_Lag_num;

B_ss_NE = zeros(12, 5);
B_ss_NE(7:12, :) = M_NE_inv_num * B_NE_num;

fprintf('    完成\n\n');

% ----- 7.5 数值输出 -----
q_labels  = {'X_b_h','phi','theta_l','theta_r','theta_b','theta_t'};
dq_labels = {'dX_b_h','dphi','dtheta_l','dtheta_r','dtheta_b','dtheta_t'};
u_labels  = {'T_hip_r','T_hip_l','T_wr','T_wl','T_tail'};

% --- A 矩阵 ---
fprintf('═══════════════════════════════════════════\n');
fprintf('  A 矩阵 (12×12)\n');
fprintf('  行/列: X_b_h, φ, θ_l, θ_r, θ_b, θ_t | dX_b_h, dφ, dθ_l, dθ_r, dθ_b, dθ_t\n');
fprintf('═══════════════════════════════════════════\n\n');

fprintf('--- 拉格朗日法 A 矩阵 ---\n');
fprintf('  A = [0_{6x6} | I_{6x6}]\n');
fprintf('      [-M\\Jg_q  | -M\\Jg_dq]\n\n');
fprintf('  A(1:6, 1:6) = 0_{6x6}\n');
fprintf('  A(1:6, 7:12) = I_{6x6}\n\n');
fprintf('  A(7:12, 1:6) = -M^{-1} * ∂g/∂q :\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.4f ', A_Lag(i+6, j));
    end
    fprintf('\n');
end
fprintf('\n  A(7:12, 7:12) = -M^{-1} * ∂g/∂dq :\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.4f ', A_Lag(i+6, j+6));
    end
    fprintf('\n');
end

fprintf('\n--- NE 法 A 矩阵 ---\n');
fprintf('  A(7:12, 1:6) = -M^{-1} * ∂g/∂q :\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.4f ', A_NE(i+6, j));
    end
    fprintf('\n');
end
fprintf('\n  A(7:12, 7:12) = -M^{-1} * ∂g/∂dq :\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.4f ', A_NE(i+6, j+6));
    end
    fprintf('\n');
end

A_num_diff = A_Lag - A_NE;
fprintf('\n--- A 差异 (Lagrange - NE), ||diff||_inf = %.4e ---\n', norm(A_num_diff, 'inf'));
fprintf('  A(7:12, 1:6) 块差异:\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.2e ', A_num_diff(i+6, j));
    end
    fprintf('\n');
end
fprintf('  A(7:12, 7:12) 块差异:\n');
for i = 1:6
    fprintf('  ');
    for j = 1:6
        fprintf('%10.2e ', A_num_diff(i+6, j+6));
    end
    fprintf('\n');
end

% --- 特征值 ---
fprintf('\n--- A 矩阵特征值 ---\n');
eig_A_Lag = eig(A_Lag);
eig_A_NE  = eig(A_NE);
fprintf('  %4s | %14s %14s | %14s %14s\n', 'No.', 'Re(Lag)', 'Im(Lag)', 'Re(NE)', 'Im(NE)');
fprintf('  -----|---------------|---------------|---------------|---------------\n');
for i = 1:12
    fprintf('  %4d | %14.6f %14.6f | %14.6f %14.6f\n', ...
        i, real(eig_A_Lag(i)), imag(eig_A_Lag(i)), ...
        real(eig_A_NE(i)), imag(eig_A_NE(i)));
end

% --- B_ss 矩阵 ---
fprintf('\n═══════════════════════════════════════════\n');
fprintf('  B_ss 矩阵 (12×5, 列 = T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail)\n');
fprintf('═══════════════════════════════════════════\n\n');

fprintf('--- 拉格朗日法 B_ss 矩阵 ---\n');
fprintf('  B_ss = [0_{6x5}; M^{-1}*B]\n\n');
fprintf('  B_ss(7:12, :) :\n');
for i = 1:6
    fprintf('  ');
    for j = 1:5
        fprintf('%10.4f ', B_ss_Lag(i+6, j));
    end
    fprintf('\n');
end

fprintf('\n--- NE 法 B_ss 矩阵 ---\n');
fprintf('  B_ss(7:12, :) :\n');
for i = 1:6
    fprintf('  ');
    for j = 1:5
        fprintf('%10.4f ', B_ss_NE(i+6, j));
    end
    fprintf('\n');
end

B_ss_num_diff = B_ss_Lag - B_ss_NE;
fprintf('\n--- B_ss 差异 (Lagrange - NE), ||diff||_inf = %.4e ---\n', norm(B_ss_num_diff, 'inf'));
fprintf('  B_ss(7:12, :) 块差异:\n');
for i = 1:6
    fprintf('  ');
    for j = 1:5
        fprintf('%10.2e ', B_ss_num_diff(i+6, j));
    end
    fprintf('\n');
end

% --- 能控性 ---
fprintf('\n--- 能控性矩阵秩 ---\n');
n_x = 12;
Co_Lag = ctrb(A_Lag, B_ss_Lag);
Co_NE  = ctrb(A_NE,  B_ss_NE);
fprintf('  rank(Co)_Lag = %d / %d\n', rank(Co_Lag), n_x);
fprintf('  rank(Co)_NE  = %d / %d\n', rank(Co_NE),  n_x);
if rank(Co_Lag) == n_x
    fprintf('  ✓ 拉格朗日法系统完全能控\n');
else
    fprintf('  ⚠ 拉格朗日法系统不完全能控 (缺失 %d 维)\n', n_x - rank(Co_Lag));
end
if rank(Co_NE) == n_x
    fprintf('  ✓ NE法系统完全能控\n');
else
    fprintf('  ⚠ NE法系统不完全能控 (缺失 %d 维)\n', n_x - rank(Co_NE));
end

fprintf('\n  Phase 7 完成\n');

%% ========================================================================
%  保存结果
%  ========================================================================

fprintf('\n========================================\n');
fprintf('保存结果...\n');
fprintf('========================================\n\n');

save('comparison_results.mat', ...
    'M_NE_sym', 'g_NE_sym', 'B_NE_sym', ...
    'M_Lag_sym', 'g_Lag_sym', 'B_Lag_sym', ...
    'M_diff', 'g_diff', 'B_diff', ...
    'M_NE_num', 'M_Lag_num', 'M_num_diff', ...
    'g_NE_num', 'g_Lag_num', 'g_num_diff', ...
    'B_NE_num', 'B_Lag_num', 'B_num_diff', ...
    'diff_count_M', 'diff_count_g', 'diff_count_B', ...
    'A_Lag', 'A_NE', 'A_num_diff', ...
    'B_ss_Lag', 'B_ss_NE', 'B_ss_num_diff', ...
    'Jg_q_Lag_num', 'Jg_dq_Lag_num', 'Jg_q_NE_num', 'Jg_dq_NE_num', ...
    'param_val', 'theta_l_test', 'theta_r_test');

fprintf('  结果已保存到 comparison_results.mat\n');
fprintf('  包含:\n');
fprintf('    符号: M_NE_sym, g_NE_sym, B_NE_sym\n');
fprintf('          M_Lag_sym, g_Lag_sym, B_Lag_sym\n');
fprintf('          M_diff, g_diff, B_diff (符号差异)\n');
fprintf('    数值: M_NE_num, M_Lag_num, M_num_diff\n');
fprintf('          g_NE_num, g_Lag_num, g_num_diff\n');
fprintf('          B_NE_num, B_Lag_num, B_num_diff\n');
fprintf('    状态空间: A_Lag, A_NE, B_ss_Lag, B_ss_NE\n');
fprintf('    差异计数: diff_count_M, g, B\n');

fprintf('\n========================================\n');
fprintf('统一对比完成!\n');
fprintf('========================================\n');