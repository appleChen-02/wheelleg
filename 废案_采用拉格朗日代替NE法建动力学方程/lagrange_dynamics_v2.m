% lagrange_dynamics_v2.m
% =========================================================================
% 拉格朗日法动力学推导 — 轮腿机器人 (含尾巴)
%
% 输入: 无需外部文件
%       用户需在 Step 2 中填写系统动能 T 和势能 V
%       广义坐标: q = [X_b_h, phi, theta_l, theta+_r, theta_b, theta_t]
%
% 输出: dynamics_v2_lagrange.mat
%       包含: M_sym, g_sym, B_sym  (符号矩阵)
%             eq1~eq6              (动力学方程, eq_i = 0)
%             ddq, u, symvar_info  (变量信息)
%
% 原理: L = T - V
%       d/dt(∂L/∂q̇_k) - ∂L/∂q_k = Q_k    (k = 1..6)
%
% 对照: lqr_lagrange.m (二阶倒立摆参考代码)
% =========================================================================

clear; clc;
fprintf('========================================\n');
fprintf('拉格朗日法动力学推导\n');
fprintf('广义坐标: q = [X_b^h, phi, theta_l, theta_r, theta_b, theta_t]\n');
fprintf('========================================\n\n');

%% ========================================================================
%  Step 1: 定义全部符号变量
%  ========================================================================

fprintf('Step 1: 定义符号变量...\n');

% ---------- 物理常数 ----------
syms g real                 % 重力加速度 (注意: g = -9.81, 与NE代码一致)
syms R R_w real             % 轮半径, 半轮距

% ---------- 机体参数 ----------
syms m_b I_b l_body_com theta_b0 real
syms l_tail_mount_h l_tail_mount_v real  % 尾电机安装点相对机体质心的前向/下向偏移

% ---------- 腿部参数 ----------
syms m_l m_r I_l I_r l_l l_r l_leg_l_com l_leg_r_com theta_l0 theta_r0 real

% ---------- 轮子参数 ----------
syms m_wl m_wr I_wl I_wr real

% ---------- 尾巴参数 ----------
syms m_t I_t l_tail_com delta_t real

% ---------- Yaw转动惯量 ----------
syms I_yaw real

% ====== 广义坐标 q ======
% q = [X_b_h, phi, theta_l, theta_r, theta_b, theta_t]
syms X_b_h    phi    theta_l    theta_r    theta_b    theta_t     real

% ====== 广义速度 dq ======
syms dX_b_h   dphi   dtheta_l   dtheta_r   dtheta_b   dtheta_t   real

% ====== 广义加速度 ddq ======
syms ddX_b_h ddphi  ddtheta_l  ddtheta_r  ddtheta_b  ddtheta_t  real

% ====== 轮角加速度 (用于动能中的轮转动项) ======
% 注意：如果用户在 T 中直接使用广义速度表达轮动能，则无需这些变量
% 这里保留它们以便用户选择在 T 中包含 "½*I_w*(v_w/R)^2" 形式
syms ddtheta_wl ddtheta_wr real

% ====== 控制输入 u ======
% u = [T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail]
syms T_hip_r T_hip_l T_motor_wr T_motor_wl T_tail real

%%
% 将向量组织为列向量，方便后续矩阵运算
q   = [X_b_h;    phi;    theta_l;    theta_r;    theta_b;    theta_t   ];
dq  = [dX_b_h;   dphi;   dtheta_l;   dtheta_r;   dtheta_b;   dtheta_t  ];
ddq = [ddX_b_h;  ddphi;  ddtheta_l;  ddtheta_r;  ddtheta_b;  ddtheta_t ];
u   = [T_hip_r; T_hip_l; T_motor_wr; T_motor_wl; T_tail];

n_q = length(q);      % 广义坐标数 = 6
n_u = length(u);      % 控制输入数 = 5

fprintf('  广义坐标 (%d 个): ', n_q);
fprintf('%s ', q);
fprintf('\n');
fprintf('  控制输入 (%d 个): ', n_u);
fprintf('%s ', u);
fprintf('\n\n');

%% ========================================================================
%  Step 2: 用户填写区 — 动能 T 和 势能 V
%  ========================================================================
%
%  请用以下变量表达式填写:
%
%  【广义坐标】     X_b_h, phi, theta_l, theta_r, theta_b, theta_t
%  【广义速度】     dX_b_h, dphi, dtheta_l, dtheta_r, dtheta_b, dtheta_t
%  【物理参数】     g(重力加速度=-9.81), R(轮半径), R_w(半轮距)
%                  m_b, I_b, l_body_com, theta_b0 (机体)
%                  m_l, I_l, l_l, l_leg_l_com, theta_l0 (左腿)
%                  m_r, I_r, l_r, l_leg_r_com, theta_r0 (右腿)
%                  m_wl, I_wl, m_wr, I_wr (轮子)
%                  m_t, I_t, l_tail_com, delta_t, l_tail_mount_h, l_tail_mount_v (尾巴)
%                  I_yaw (Yaw转动惯量)
%
%  【坐标约定】(与 NE 代码一致)
%    - h 轴: 水平向前, 原点在平衡状态下 hip 中点在地面的投影
%    - v 轴: 竖直向上
%    - 机体俯仰角 theta_b: 从水平线起算 (>0 = 抬头, 即机身前倾)
%    - 腿角 theta_l, theta_r: 从竖直向上方向起算
%    - 尾巴角 theta_t: 从水平线起算
%    - yaw角 phi: 从上方看逆时针
%
%  【势能约定】
%    V = m * g * (质心高度)   (g = -9.81, 高度向上为正)
%    例如: 一个质心高度为 h 的质量 m, V = m * g * h
%
%  【轮动能建议】
%    轮子有平动动能 ½*m_w*(v_w^h² + v_w^v²) 和转动动能 ½*I_w*ω²
%    纯滚动约束: ω = v_w^h / R
%    建议直接在 T 中将转动和平动合并: ½*(m_w + I_w/R²)*v_w^h²
%    其中 v_w^h 是轮心水平速度 (用广义速度表达)
%
%  【参考: 现有 NE 代码中各刚体加速度表达式 (apply_kinematics_v2.m §4.1)】
%    左轮心水平速度: v_wl^h = dX_b_h - l_l*cos(theta_l)*dtheta_l
%    右轮心水平速度: v_wr^h = dX_b_h - l_r*cos(theta_r)*dtheta_r
%    左腿质心水平速度: v_legl^h = v_wl^h + l_leg_l_com*cos(theta_l+theta_l0)*dtheta_l
%    右腿质心水平速度: v_legr^h = v_wr^h + l_leg_r_com*cos(theta_r+theta_r0)*dtheta_r
%    机体 hip 中点竖直高度:
%      y_hip = R + (l_l*cos(theta_l) + l_r*cos(theta_r))/2
%    机体质心水平速度:
%      v_b^h = (v_wl^h + v_wr^h)/2 + l_body_com*cos(theta_b+theta_b0)*dtheta_b
%    机体质心竖直速度:
%      v_b^v = -(l_l*sin(theta_l)*dtheta_l + l_r*sin(theta_r)*dtheta_r)/2 ...
%              - l_body_com*sin(theta_b+theta_b0)*dtheta_b
%    尾电机安装点位置 (相对 hip 中点):
%      r_T_h = l_tail_mount_h*cos(theta_b) + l_tail_mount_v*sin(theta_b)
%      r_T_v = l_tail_mount_h*sin(theta_b) - l_tail_mount_v*cos(theta_b)
%    Yaw 动能: ½ * I_yaw * dphi²
%
%  ========================================================================

fprintf('Step 2: 动能 T 和 势能 V\n');
fprintf('----------------------------------------\n');
fprintf('⚠  请在本文件中搜索 "USER_FILL" 并填写 T 和 V\n');
fprintf('   当前使用空白占位符 —— 方程将为 0 = 0\n');
fprintf('----------------------------------------\n\n');

% ╔══════════════════════════════════════════════════════════════════════╗
% ║                    USER_FILL: 请在下方填写 T 和 V                    ║
% ╚══════════════════════════════════════════════════════════════════════╝

% --- 动能 T ---
% T 应为 dq 的二次型 + 常数项 (对称正定)
% 形式: T = ½ * Σ (m_i * v_i²) + ½ * Σ (I_i * ω_i²)
% 其中所有速度 v_i, ω_i 必须用广义速度 dq = [dX_b_h, dphi, dtheta_l,
% dtheta_r, dtheta_b, dtheta_t] 表示
%T = sym(0);   % <--- 请替换为你的动能表达式

% 轮心水平速度 (小角度近似 cos(phi)≈1, 与 NE apply_kinematics_v2.m §5.0 一致)
v_wl_h = dX_b_h - R_w * dphi - l_l * cos(theta_l) * dtheta_l;
v_wr_h = dX_b_h + R_w * dphi - l_r * cos(theta_r) * dtheta_r;
K_wl = 1/2 * (m_wl + I_wl/R^2) * v_wl_h^2;
K_wr = 1/2 * (m_wr + I_wr/R^2) * v_wr_h^2;

v_legl_h = v_wl_h + l_leg_l_com * cos(theta_l + theta_l0) * dtheta_l;
v_legl_v = l_leg_l_com * sin(theta_l + theta_l0) * dtheta_l;
K_legl = 1/2 * m_l * (v_legl_h^2 + v_legl_v^2) + 1/2 * I_l * dtheta_l^2;
v_legr_h = v_wr_h + l_leg_r_com * cos(theta_r + theta_r0) * dtheta_r;
v_legr_v = l_leg_r_com * sin(theta_r + theta_r0) * dtheta_r;
K_legr = 1/2 * m_r * (v_legr_h^2 + v_legr_v^2) + 1/2 * I_r * dtheta_r^2;

v_hip_h = (v_wl_h + v_wr_h) / 2;
v_hip_v = -(l_l * sin(theta_l) * dtheta_l + l_r * sin(theta_r) * dtheta_r) / 2;
v_b_h = v_hip_h + l_body_com * cos(theta_b + theta_b0) * dtheta_b;
v_b_v = v_hip_v - l_body_com * sin(theta_b + theta_b0) * dtheta_b;
K_body = 1/2 * m_b * (v_b_h^2 + v_b_v^2) + 1/2 * I_b * dtheta_b^2;

K_yaw = 1/2 * I_yaw * dphi^2;

r_T_h = l_tail_mount_h * cos(theta_b) + l_tail_mount_v * sin(theta_b);
r_T_v = l_tail_mount_h * sin(theta_b) - l_tail_mount_v * cos(theta_b);
v_mount_h = v_hip_h + (-r_T_v) * dtheta_b;
v_mount_v = v_hip_v + (r_T_h) * dtheta_b;
% 尾巴质心绝对角度 (与 NE 代码 alpha_t 一致)
alpha_t = theta_t - theta_b + delta_t;
% 尾巴质心线速度 = 安装点速度 + 尾巴绕安装点转动贡献
% d(alpha_t)/dt = dtheta_t - dtheta_b
v_tail_h = v_mount_h - l_tail_com * sin(alpha_t) * (dtheta_t - dtheta_b);
v_tail_v = v_mount_v + l_tail_com * cos(alpha_t) * (dtheta_t - dtheta_b);
K_tail = 1/2 * m_t * (v_tail_h^2 + v_tail_v^2) + 1/2 * I_t * dtheta_t^2;

T = K_wl + K_wr + K_legl + K_legr + K_body + K_yaw + K_tail;

% --- 势能 V ---
% V 仅依赖于广义坐标 q (不含速度)
% V = Σ (m_i * g * y_i)    (g = -9.81, y_i 为质心高度)
%V = sym(0);   % <--- 请替换为你的势能表达式

% 髋中点高度（左右腿共同决定）
y_hip = R + (l_l * cos(theta_l) + l_r * cos(theta_r)) / 2;

% 轮子
V_wl = m_wl * g * R;
V_wr = m_wr * g * R;

% 腿质心（只需质心相对轮心的偏移）
V_legl = m_l * g * (R + l_leg_l_com * cos(theta_l + theta_l0));
V_legr = m_r * g * (R + l_leg_r_com * cos(theta_r + theta_r0));

% 机体质心
V_body = m_b * g * (y_hip + l_body_com * cos(theta_b + theta_b0));

% 尾巴安装点的竖直偏移（随机体俯仰变化）
% 注意: r_T_v 在上面动能部分已定义，此处重用于势能
% r_T_v = l_tail_mount_h * sin(theta_b) - l_tail_mount_v * cos(theta_b);
% 尾巴安装点高度 = 髋中点高度 + 安装点相对髋中点的竖直偏移
y_mount = y_hip + r_T_v;
% 尾巴质心高度 = 安装点高度 + 尾巴质心相对安装点的竖直偏移
% alpha_t = theta_t - theta_b + delta_t (尾巴质心绝对角度)
V_tail = m_t * g * (y_mount + l_tail_com * sin(alpha_t));

% 总势能
V = V_wl + V_wr + V_legl + V_legr + V_body + V_tail;

% ╔══════════════════════════════════════════════════════════════════════╗
% ║                          USER_FILL 结束                              ║
% ╚══════════════════════════════════════════════════════════════════════╝

% 检查用户是否已填写
if isequal(T, sym(0)) && isequal(V, sym(0))
    fprintf('\n*** 注意: T 和 V 尚未填写, 动力学方程将为空 ***\n');
    fprintf('*** 请在 Step 2 的 USER_FILL 区域填写 T 和 V ***\n\n');
    user_filled = false;
else
    fprintf('✓ T 和 V 已填写\n\n');
    user_filled = true;
end

%% ========================================================================
%  Step 3: 构建拉格朗日量
%  ========================================================================

fprintf('Step 3: 构建拉格朗日量 L = T - V...\n');

L = T - V;

fprintf('  L 表达式包含 %d 项\n', length(children(expand(L))));
fprintf('  L 中的变量: ');
disp(symvar(L)');
fprintf('\n');

%% ========================================================================
%  Step 4: 定义广义力 Q_k
%  ========================================================================
%  由电机力矩通过虚功原理给出。
%  各控制输入对应的关节运动:
%    T_motor_wl: 左轮电机 → 左轮旋转 → 通过纯滚动 → 水平力/偏航力矩
%    T_motor_wr: 右轮电机 → 右轮旋转 → 通过纯滚动 → 水平力/偏航力矩
%    T_hip_l:  左髋电机 → theta_b 增加, theta_l 减少
%    T_hip_r:  右髋电机 → theta_b 增加, theta_r 减少
%    T_tail:  尾电机   → theta_b 增加, theta_t 减少

fprintf('Step 4: 定义广义力 Q_k...\n');

% 广义力 Q 为 6×1 列向量，Q_k = ∂(虚功)/∂q_k  由控制力矩贡献
% 参考 NE 代码 eq1, eq5 中力矩项的形式
Q = sym(zeros(n_q, 1));

% Q_{X_b_h}: 轮力矩通过纯滚动产生水平驱动力
Q(1) = -(T_motor_wl + T_motor_wr) / R;

% Q_{phi}: 左右轮力矩差产生偏航力矩
Q(2) = R_w / R * (T_motor_wl - T_motor_wr);

% Q_{theta_l}: 左轮力矩 + 左髋反作用力矩
Q(3) = T_motor_wl - T_hip_l;

% Q_{theta_r}: 右轮力矩 + 右髋反作用力矩
Q(4) = T_motor_wr - T_hip_r;

% Q_{theta_b}: 左右髋力矩 + 尾电机力矩 (均正向作用于机体俯仰)
Q(5) = T_hip_l + T_hip_r + T_tail;

% Q_{theta_t}: 尾电机反作用力矩
Q(6) = -T_tail;

fprintf('  广义力 Q = \n');
for k = 1:n_q
    fprintf('  Q(%d) = %s\n', k, char(Q(k)));
end
fprintf('\n');

%% ========================================================================
%  Step 5: 欧拉-拉格朗日方程
%  ========================================================================
%  d/dt(∂L/∂q̇_k) - ∂L/∂q_k = Q_k
%
%  实现方法:
%    p_k = ∂L/∂q̇_k                    (广义动量)
%    dp_k/dt = Σ ∂p_k/∂q_j * q̇_j + Σ ∂p_k/∂q̇_j * q̈_j   (全导数)
%    eq_k = dp_k/dt - ∂L/∂q_k - Q_k = 0

fprintf('Step 5: 推导欧拉-拉格朗日方程...\n');

% 声明时间函数: 位置 q(t) 和速度 dq(t)
% 这是为了后续用 diff(f,t) 求全导数 (MATLAB符号工具箱会识别为时间函数)
% 注意: 这里不用 symfun, 而是在计算 d(∂L/∂q̇)/dt 时手动用链式法则
% 这样更可控、更高效

eqs = sym(zeros(n_q, 1));  % 最终动力学方程

for k = 1:n_q
    % ----- 广义动量 p_k = ∂L/∂(dq_k) -----
    p_k = diff(L, dq(k));

    % ----- 全时间导数 dp_k/dt = Σ(∂p_k/∂q_j)*dq_j + Σ(∂p_k/∂dq_j)*ddq_j -----
    % ∂p_k/∂q 部分
    dpk_dt_q = sym(0);
    for j = 1:n_q
        dpk_dt_q = dpk_dt_q + diff(p_k, q(j)) * dq(j);
    end

    % ∂p_k/∂dq 部分
    dpk_dt_dq = sym(0);
    for j = 1:n_q
        dpk_dt_dq = dpk_dt_dq + diff(p_k, dq(j)) * ddq(j);
    end

    dpk_dt = dpk_dt_q + dpk_dt_dq;

    % ----- ∂L/∂q_k -----
    dL_dqk = diff(L, q(k));

    % ----- 欧拉-拉格朗日方程 -----
    eqs(k) = dpk_dt - dL_dqk - Q(k);

    if mod(k, 2) == 0
        fprintf('  进度: 已推导坐标 %d/%d (%s)\n', k, n_q, char(q(k)));
    end
end

fprintf('  欧拉-拉格朗日方程推导完成 (共 %d 个方程)\n\n', n_q);

%% ========================================================================
%  Step 6: 提取 M(q), g(q,q̇), B(q) 矩阵
%  ========================================================================
%
%  动力学方程形式: M(q)*ddq + g(q,q̇) = B(q)*u
%  即: eq_k = M(k,:)*ddq + g_k - B(k,:)*u = 0
%
%  M: n_q × n_q   质量/惯性矩阵  (ddq 的系数)
%  g: n_q × 1     科氏力+离心力+重力项 (不含 ddq 和 u)
%  B: n_q × n_u   控制输入映射矩阵 (u 的系数, 含符号约定)

fprintf('Step 6: 提取 M, g, B 矩阵...\n');

% ----- M 矩阵: M(i,j) = ∂(eq_i)/∂(ddq_j) -----
M_sym = sym(zeros(n_q, n_q));
for i = 1:n_q
    for j = 1:n_q
        M_sym(i,j) = diff(eqs(i), ddq(j));
    end
end

% ----- B_raw: B_raw(i,j) = ∂(eq_i)/∂(u_j) -----
B_raw = sym(zeros(n_q, n_u));
for i = 1:n_q
    for j = 1:n_u
        B_raw(i,j) = diff(eqs(i), u(j));
    end
end
% 从 eqs = M*ddq + g - B*u = 0 得 B*u 项系数: B = -B_raw
B_sym = -B_raw;

% ----- g 向量: g = -(eqs - M*ddq - B_raw*u) -----
% 即从方程中减去加速度项和控制输入项，剩余部分取反
g_sym = sym(zeros(n_q, 1));
for i = 1:n_q
    g_sym(i) = -(eqs(i) - M_sym(i,:)*ddq - B_raw(i,:)*u);
end

% 化简 (可能耗时)
fprintf('  正在简化矩阵...\n');
tic_simplify = tic;
M_sym = simplify(M_sym);
g_sym = simplify(g_sym);
B_sym = simplify(B_sym);
fprintf('  简化完成 (耗时 %.1f 秒)\n\n', toc(tic_simplify));

%% ========================================================================
%  Step 7: 构建 eq1~eq6 (保持与 NE 法兼容的接口)
%  ========================================================================
%
%  eq_i = M(i,:)*ddq + g_sym(i) - B_sym(i,:)*u  =  0
%  注意: eq_i 中用到的轮角加速度 ddtheta_wl, ddtheta_wr 应已由
%  动能 T 的设计直接使用广义坐标表达 (通过纯滚动约束)。
%  如果 T 中仍含 ddtheta_wl, ddtheta_wr, 则此处需额外代换。

fprintf('Step 7: 构建 eq1~eq6 (NE 兼容格式)...\n');

eq1 = M_sym(1,:)*ddq + g_sym(1) - B_sym(1,:)*u;
eq2 = M_sym(2,:)*ddq + g_sym(2) - B_sym(2,:)*u;
eq3 = M_sym(3,:)*ddq + g_sym(3) - B_sym(3,:)*u;
eq4 = M_sym(4,:)*ddq + g_sym(4) - B_sym(4,:)*u;
eq5 = M_sym(5,:)*ddq + g_sym(5) - B_sym(5,:)*u;
eq6 = M_sym(6,:)*ddq + g_sym(6) - B_sym(6,:)*u;

% 检查是否仍需 ddtheta_wl, ddtheta_wr 代换
vars_eq1 = symvar(eq1);
if any(has([eq1;eq2;eq3;eq4;eq5;eq6], [ddtheta_wl, ddtheta_wr]))
    fprintf('  ⚠  检测到方程中仍含 ddtheta_wl / ddtheta_wr\n');
    fprintf('     这表示 T 中未直接用广义速度表达轮动能\n');
    fprintf('     请在 T 中使用 ½*(m_w+I_w/R²)*v_w^h² 形式, 或在此处补充代换\n\n');
else
    fprintf('  ✓ 方程中不含 ddtheta_wl / ddtheta_wr, 纯滚动约束已正确嵌入\n\n');
end

%% ========================================================================
%  Step 8: 打印方程信息
%  ========================================================================

fprintf('========================================\n');
fprintf('方程汇总\n');
fprintf('========================================\n\n');

eq_names = {'eq1: 水平动量 (X_b^h)', 'eq2: Yaw转动 (phi)', ...
            'eq3: 左腿转动 (theta_l)', 'eq4: 右腿转动 (theta_r)', ...
            'eq5: 机体俯仰 (theta_b)', 'eq6: 尾巴转动 (theta_t)'};

for i = 1:6
    fprintf('----------------------------------------\n');
    fprintf('%s\n', eq_names{i});
    fprintf('----------------------------------------\n');

    % 显示紧凑形式
    eq_i = eval(sprintf('eq%d', i));
    fprintf('项数: %d\n', length(children(expand(eq_i))));

    % 分离各项: M*ddq, g, B*u
    M_row = M_sym(i,:);
    g_row = g_sym(i);
    B_row = B_sym(i,:);

    fprintf('惯性项 (M*ddq): ');
    % 显示非零惯性耦合
    for j = 1:n_q
        if ~isequal(M_row(j), sym(0))
            fprintf('[%s]*ddq(%d)  ', char(M_row(j)), j);
        end
    end
    fprintf('\n');

    fprintf('重力/科氏项 (g): 项数=%d\n', length(children(expand(g_row))));

    fprintf('控制项 (B*u): ');
    for j = 1:n_u
        if ~isequal(B_row(j), sym(0))
            fprintf('[%s]*%s  ', char(B_row(j)), char(u(j)));
        end
    end
    fprintf('\n\n');
end

%% ========================================================================
%  Step 9: 保存结果
%  ========================================================================

fprintf('Step 9: 保存结果...\n');

% 组织变量信息
symvar_info = struct();
symvar_info.q   = q;
symvar_info.dq  = dq;
symvar_info.ddq = ddq;
symvar_info.u   = u;
symvar_info.n_q = n_q;
symvar_info.n_u = n_u;

script_dir = fileparts(mfilename('fullpath'));
save(fullfile(script_dir, 'dynamics_v2_lagrange.mat'), ...
     'M_sym', 'g_sym', 'B_sym', ...
     'eq1', 'eq2', 'eq3', 'eq4', 'eq5', 'eq6', ...
     'symvar_info');

fprintf('  已保存到 dynamics_v2_lagrange.mat\n');
fprintf('  包含: M_sym(%dx%d), g_sym(%dx1), B_sym(%dx%d)\n', n_q, n_q, n_q, n_q, n_u);
fprintf('        eq1~eq6 (动力学方程, eq_i = 0)\n');
fprintf('        symvar_info (变量定义)\n');

%% ========================================================================
%  Step 10: 自检
%  ========================================================================

fprintf('\n========================================\n');
fprintf('自检\n');
fprintf('========================================\n\n');

% 检查 M 矩阵对称性
M_diff = simplify(M_sym - M_sym.');
if isequal(M_diff, sym(zeros(n_q, n_q)))
    fprintf('✓ M 矩阵对称 (符合拉格朗日力学预期)\n');
else
    fprintf('✗ M 矩阵不对称! 请检查 T 表达式\n');
    fprintf('  最大不对称元素: \n');
    % 显示一个非零差异
    for i = 1:n_q
        for j = i+1:n_q
            if ~isequal(M_diff(i,j), sym(0))
                fprintf('  M(%d,%d)-M(%d,%d) = %s\n', i, j, j, i, char(M_diff(i,j)));
                break;
            end
        end
    end
end

% 检查 B 矩阵与控制输入对应关系
fprintf('\nB 矩阵 (符号) %dx%d:\n', n_q, n_u);
fprintf('行: [X_b^h, phi, theta_l, theta_r, theta_b, theta_t]\n');
fprintf('列: [T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail]\n');
disp(B_sym);

% 检查广义力 Q 与 B 矩阵的一致性
% B*u = Q (理论上), 验证 B_sym*u == Q
BQ_diff = simplify(B_sym * u - Q);
if isequal(BQ_diff, sym(zeros(n_q, 1)))
    fprintf('\n✓ B*u = Q (广义力与控制输入一致)\n');
else
    fprintf('\n⚠ B*u ≠ Q, 差异:\n');
    disp(BQ_diff);
end

fprintf('\n========================================\n');
if user_filled
    fprintf('拉格朗日法动力学推导完成!\n');
else
    fprintf('框架搭建完成。请在 Step 2 中填写 T 和 V 后重新运行。\n');
end
fprintf('========================================\n');
