% validate_lagrange_vs_ne.m
% =========================================================================
% 交叉验证 — 拉格朗日法 vs 牛顿-欧拉法
%
% 复现步骤:
%   1. 先运行 新尾巴离地/lagrange_dynamics_v2.m (需已填写 T 和 V)
%   2. 再运行本脚本
% =========================================================================

clear; clc;
fprintf('========================================\n');
fprintf('交叉验证: 拉格朗日法 vs 牛顿-欧拉法\n');
fprintf('========================================\n\n');

%% ========================================================================
%  Part 1: 定义符号变量 — 新旧两套命名
%  ========================================================================
%  NE 的 dynamics_v2.mat 保存时使用旧变量名 (l_b, T_wl_to_l, ...)
%  拉格朗日 dynamics_v2_lagrange.mat 使用新变量名 (l_body_com, T_motor_wl, ...)
%  本脚本声明两套变量，再通过 subs 将 NE 方程统一到新命名体系下对比。

fprintf('========================================\n');
fprintf('Part 1: 定义符号变量\n');
fprintf('========================================\n\n');

% ====== A. 物理参数 — 新名 ======
syms g R R_w real
syms m_b I_b l_body_com theta_b0 real
syms l_tail_mount_h l_tail_mount_v real
syms m_l m_r I_l I_r l_l l_r l_leg_l_com l_leg_r_com theta_l0 theta_r0 real
syms m_wl m_wr I_wl I_wr real
syms m_t I_t l_tail_com delta_t real
syms I_yaw real

% ====== B. 物理参数 — 旧名 (仅用于接收 NE .mat 中的旧变量) ======
syms l_b       real   % → l_body_com
syms l_l_d     real   % → l_leg_l_com
syms l_r_d     real   % → l_leg_r_com
syms a_t_p     real   % → l_tail_mount_h
syms b_t_p     real   % → l_tail_mount_v
syms l_t_c     real   % → l_tail_com

% ====== C. 广义坐标和导数 ======
syms X_b_h dX_b_h ddX_b_h real
syms phi dphi ddphi real
syms theta_l dtheta_l ddtheta_l real
syms theta_r dtheta_r ddtheta_r real
syms theta_b dtheta_b ddtheta_b real
syms theta_t dtheta_t ddtheta_t real

% ====== D. 轮角加速度 ======
syms ddtheta_wl ddtheta_wr real

% ====== E. 控制输入 — 新名 ======
syms T_hip_r T_hip_l T_motor_wr T_motor_wl T_tail real

% ====== F. 控制输入 — 旧名 (用于接收 NE 方程) ======
syms T_r_to_b  real   % → T_hip_r
syms T_l_to_b  real   % → T_hip_l
syms T_wr_to_r real   % → T_motor_wr
syms T_wl_to_l real   % → T_motor_wl
syms T_t_to_b  real   % → T_tail

% ====== G. 物理加速度 (NE 方程中使用) ======
syms a_b_h a_b_v a_l_h a_l_v a_r_h a_r_v real
syms a_wl_h a_wl_v a_wr_h a_wr_v real
syms a_t_h a_t_v a_tp_h a_tp_v real
syms alpha_t real
syms r_bp_h r_bp_v real   % NE eq2 中使用

% ====== H. 旧→新 物理参数映射 ======
param_old_to_new = {
    l_b,   l_body_com;
    l_l_d, l_leg_l_com;
    l_r_d, l_leg_r_com;
    a_t_p, l_tail_mount_h;
    b_t_p, l_tail_mount_v;
    l_t_c, l_tail_com;
};

% ====== I. 旧→新 控制输入映射 ======
ctrl_old_to_new = {
    T_r_to_b,  T_hip_r;
    T_l_to_b,  T_hip_l;
    T_wr_to_r, T_motor_wr;
    T_wl_to_l, T_motor_wl;
    T_t_to_b,  T_tail;
};

% ====== J. 新名向量 (后续统一使用) ======
q   = [X_b_h;   phi;    theta_l;   theta_r;   theta_b;   theta_t  ];
dq  = [dX_b_h;  dphi;   dtheta_l;  dtheta_r;  dtheta_b;  dtheta_t ];
ddq = [ddX_b_h; ddphi;  ddtheta_l; ddtheta_r; ddtheta_b; ddtheta_t];
u   = [T_hip_r; T_hip_l; T_motor_wr; T_motor_wl; T_tail];

n_q = 6;
n_u = 5;

fprintf('  新名符号: %d 个\n', n_q + n_u + 28);
fprintf('  旧名映射: 物理参数 %d 条, 控制输入 %d 条\n', ...
        size(param_old_to_new,1), size(ctrl_old_to_new,1));
fprintf('\n');

%% ========================================================================
%  Part 2: 加载两套动力学方程
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 2: 加载两套动力学方程\n');
fprintf('========================================\n\n');

% ----- 加载牛顿-欧拉法 -----
ne_available = false;
try
    load('../尾巴离地/dynamics_v2.mat');
    if exist('eq1', 'var') && exist('eq6', 'var')
        % 关键: 对 NE 方程做旧→新命名转换
        eq1 = subs(eq1, [param_old_to_new(:,1); ctrl_old_to_new(:,1)], ...
                        [param_old_to_new(:,2); ctrl_old_to_new(:,2)]);
        eq2 = subs(eq2, [param_old_to_new(:,1); ctrl_old_to_new(:,1)], ...
                        [param_old_to_new(:,2); ctrl_old_to_new(:,2)]);
        eq3 = subs(eq3, [param_old_to_new(:,1); ctrl_old_to_new(:,1)], ...
                        [param_old_to_new(:,2); ctrl_old_to_new(:,2)]);
        eq4 = subs(eq4, [param_old_to_new(:,1); ctrl_old_to_new(:,1)], ...
                        [param_old_to_new(:,2); ctrl_old_to_new(:,2)]);
        eq5 = subs(eq5, [param_old_to_new(:,1); ctrl_old_to_new(:,1)], ...
                        [param_old_to_new(:,2); ctrl_old_to_new(:,2)]);
        eq6 = subs(eq6, [param_old_to_new(:,1); ctrl_old_to_new(:,1)], ...
                        [param_old_to_new(:,2); ctrl_old_to_new(:,2)]);

        eq_ne = {eq1, eq2, eq3, eq4, eq5, eq6};
        ne_available = true;
        fprintf('✓ NE 法动力学已加载并转换为新命名 (%s)\n', '../尾巴离地/dynamics_v2.mat');
    else
        warning('NE 法文件存在但缺少 eq1~eq6');
    end
catch ME
    warning('无法加载 NE 法动力学: %s', ME.message);
end

% ----- 加载拉格朗日法 -----
lagrange_available = false;
try
    load('dynamics_v2_lagrange.mat');

    if exist('M_sym', 'var') && exist('g_sym', 'var') && exist('B_sym', 'var')
        all_zero_M = isequal(M_sym, sym(zeros(n_q, n_q)));
        all_zero_g = isequal(g_sym, sym(zeros(n_q, 1)));
        all_zero_B = isequal(B_sym, sym(zeros(n_q, n_u)));

        if ~all_zero_M || ~all_zero_g || ~all_zero_B
            lagrange_available = true;
            fprintf('✓ 拉格朗日法动力学已加载 (dynamics_v2_lagrange.mat)\n');
            fprintf('  M 矩阵: %dx%d, g 向量: %dx1, B 矩阵: %dx%d\n', ...
                    n_q, n_q, n_q, n_q, n_u);
        else
            fprintf('⚠ M, g, B 全为零 — 请在 lagrange_dynamics_v2.m 中填写 T 和 V\n');
        end
    else
        warning('拉格朗日法文件缺少 M_sym/g_sym/B_sym');
    end
catch ME
    warning('无法加载拉格朗日法动力学: %s\n请先运行 lagrange_dynamics_v2.m', ME.message);
end

if ~ne_available && ~lagrange_available
    error('两套动力学方程均不可用，无法进行对比');
end

%% ========================================================================
%  Part 3: 构建运动学约束代换
%  ========================================================================

fprintf('\n========================================\n');
fprintf('Part 3: 构建统一运动学代换\n');
fprintf('========================================\n\n');

% 纯滚动约束
a_wr_h_expr = R * ddtheta_wr;
a_wl_h_expr = R * ddtheta_wl;
a_wr_v_expr = sym(0);
a_wl_v_expr = sym(0);

% 腿转轴加速度
a_r_h_expr = a_wr_h_expr + l_r*cos(theta_r)*ddtheta_r - l_r*sin(theta_r)*dtheta_r^2;
a_r_v_expr = a_wr_v_expr - l_r*sin(theta_r)*ddtheta_r - l_r*cos(theta_r)*dtheta_r^2;
a_l_h_expr = a_wl_h_expr + l_l*cos(theta_l)*ddtheta_l - l_l*sin(theta_l)*dtheta_l^2;
a_l_v_expr = a_wl_v_expr - l_l*sin(theta_l)*ddtheta_l - l_l*cos(theta_l)*dtheta_l^2;

% 机体加速度
a_b_h_expr = (a_r_h_expr + a_l_h_expr) / 2;
a_b_v_expr = (a_r_v_expr + a_l_v_expr) / 2;

% 尾电机安装点加速度
alpha_t_expr = theta_t - theta_b + delta_t;
a_tp_h_expr = a_b_h_expr ...
            - (l_tail_mount_h*sin(theta_b) - l_tail_mount_v*cos(theta_b)) * ddtheta_b ...
            - (l_tail_mount_h*cos(theta_b) + l_tail_mount_v*sin(theta_b)) * dtheta_b^2;
a_tp_v_expr = a_b_v_expr ...
            + (l_tail_mount_h*cos(theta_b) + l_tail_mount_v*sin(theta_b)) * ddtheta_b ...
            - (l_tail_mount_h*sin(theta_b) - l_tail_mount_v*cos(theta_b)) * dtheta_b^2;

% 尾巴质心加速度
a_t_h_expr = a_tp_h_expr ...
           - l_tail_com * ((ddtheta_t - ddtheta_b) * sin(alpha_t_expr) ...
                         + (dtheta_t - dtheta_b)^2 * cos(alpha_t_expr));
a_t_v_expr = a_tp_v_expr ...
           - l_tail_com * ((ddtheta_t - ddtheta_b) * cos(alpha_t_expr) ...
                         - (dtheta_t - dtheta_b)^2 * sin(alpha_t_expr));

% 轮角加速度用广义坐标表示
leg_accel_term = (l_r*cos(theta_r)*ddtheta_r + l_l*cos(theta_l)*ddtheta_l)/2;
leg_vel_sq_term = (l_r*sin(theta_r)*dtheta_r^2 + l_l*sin(theta_l)*dtheta_l^2)/2;
leg_term = leg_accel_term - leg_vel_sq_term;
ddtheta_wr_from_q = (ddX_b_h + R_w*ddphi)/R - leg_term/R;
ddtheta_wl_from_q = (ddX_b_h - R_w*ddphi)/R - leg_term/R;

kinematics_subs = {
    a_wr_h, a_wr_h_expr;  a_wl_h, a_wl_h_expr;
    a_wr_v, a_wr_v_expr;  a_wl_v, a_wl_v_expr;
    a_r_h,  a_r_h_expr;   a_r_v,  a_r_v_expr;
    a_l_h,  a_l_h_expr;   a_l_v,  a_l_v_expr;
    a_b_h,  a_b_h_expr;   a_b_v,  a_b_v_expr;
    a_t_h,  a_t_h_expr;   a_t_v,  a_t_v_expr;
    a_tp_h, a_tp_h_expr;  a_tp_v, a_tp_v_expr;
    alpha_t, alpha_t_expr;
};

wheel_subs = {
    ddtheta_wr, ddtheta_wr_from_q;
    ddtheta_wl, ddtheta_wl_from_q;
};

fprintf('  运动学代换: %d 条\n', size(kinematics_subs, 1));
fprintf('  轮角加速度代换: %d 条\n', size(wheel_subs, 1));
fprintf('\n');

%% ========================================================================
%  Part 4: 统一化——将两套方程转换为同一形式
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 4: 统一化两套方程\n');
fprintf('========================================\n\n');

% ----- 4.1 NE 方程代换 -----
if ne_available
    fprintf('  正在代换 NE 方程...\n');
    eq_ne_unified = cell(6, 1);
    for i = 1:6
        tmp = subs(eq_ne{i}, kinematics_subs(:,1), kinematics_subs(:,2));
        tmp = subs(tmp, wheel_subs(:,1), wheel_subs(:,2));
        eq_ne_unified{i} = simplify(tmp);
    end
    fprintf('  NE 方程代换完成\n');

    % 从 NE 统一化方程提取 M, g, B
    M_ne_sym = sym(zeros(n_q, n_q));
    B_raw_ne = sym(zeros(n_q, n_u));
    for i = 1:6
        for j = 1:6
            M_ne_sym(i,j) = diff(eq_ne_unified{i}, ddq(j));
        end
        for j = 1:5
            B_raw_ne(i,j) = diff(eq_ne_unified{i}, u(j));
        end
    end
    g_ne_sym = sym(zeros(n_q, 1));
    for i = 1:6
        g_ne_sym(i) = -(eq_ne_unified{i} - M_ne_sym(i,:)*ddq - B_raw_ne(i,:)*u);
    end
    B_ne_sym = -B_raw_ne;

    % ★ 行序对齐: NE顺序 [X,θ_b,θ_r,θ_l,φ,θ_t] → 拉格朗日顺序 [X,φ,θ_l,θ_r,θ_b,θ_t]
    % 注意: 仅重排行, 不重排列 (列来自 diff(eq, ddq(j)), ddq 已经是拉格朗日顺序)
    reorder_NE = [1, 5, 4, 3, 2, 6];  % NE idx → Lagrange row
    M_ne_sym = M_ne_sym(reorder_NE, :);
    g_ne_sym = g_ne_sym(reorder_NE);
    B_ne_sym = B_ne_sym(reorder_NE, :);

    fprintf('  NE  M 非零元: %d,  g 非零元: %d (已对齐行序)\n', ...
            nnz(~logical(M_ne_sym == sym(0))), nnz(~logical(g_ne_sym == sym(0))));
end

% ----- 4.2 拉格朗日方程 -----
if lagrange_available
    fprintf('  正在处理拉格朗日方程...\n');
    M_lag_unified = M_sym;
    g_lag_unified = g_sym;
    B_lag_unified = B_sym;

    has_wheel = any(has(symvar(M_lag_unified), [ddtheta_wl, ddtheta_wr])) || ...
                any(has(symvar(g_lag_unified), [ddtheta_wl, ddtheta_wr]));
    if has_wheel
        fprintf('  ⚠ 含 ddtheta_wl/ddtheta_wr，正在代换...\n');
        M_lag_unified = simplify(subs(M_lag_unified, wheel_subs(:,1), wheel_subs(:,2)));
        g_lag_unified = simplify(subs(g_lag_unified, wheel_subs(:,1), wheel_subs(:,2)));
        B_lag_unified = simplify(subs(B_lag_unified, wheel_subs(:,1), wheel_subs(:,2)));
    else
        fprintf('  ✓ 已直接用广义坐标表达\n');
    end

    fprintf('  拉格朗日 M 非零元: %d,  g 非零元: %d\n', ...
            nnz(~logical(M_lag_unified == sym(0))), nnz(~logical(g_lag_unified == sym(0))));
end

%% ========================================================================
%  Part 5: 符号层面逐矩阵对比
%  ========================================================================

if ne_available && lagrange_available
    fprintf('\n========================================\n');
    fprintf('Part 5: 符号矩阵对比\n');
    fprintf('========================================\n\n');

    % ----- M 矩阵对比 -----
    fprintf('--- M 矩阵 (质量/惯性) ---\n');
    M_diff = simplify(M_ne_sym - M_lag_unified);
    diff_count_M = sum(arrayfun(@(x) ~isequal(x, sym(0)), M_diff(:)));
    if diff_count_M == 0
        fprintf('  ✓ M 矩阵完全一致\n');
    else
        fprintf('  ⚠ M 矩阵存在 %d 处差异 (共 %d 个元素)\n', diff_count_M, n_q*n_q);
        % 显示前5个
        shown = 0;
        for i = 1:n_q
            for j = 1:n_q
                if ~isequal(M_diff(i,j), sym(0)) && shown < 5
                    shown = shown + 1;
                    fprintf('    M(%d,%d): %s\n', i, j, char(M_diff(i,j)));
                end
            end
        end
    end

    % ----- g 向量对比 -----
    fprintf('\n--- g 向量 (重力+科氏力+离心力) ---\n');
    g_diff = simplify(g_ne_sym - g_lag_unified);
    diff_count_g = sum(arrayfun(@(x) ~isequal(x, sym(0)), g_diff));
    if diff_count_g == 0
        fprintf('  ✓ g 向量完全一致\n');
    else
        fprintf('  ⚠ g 向量存在 %d 处差异\n', diff_count_g);
        for i = 1:n_q
            if ~isequal(g_diff(i), sym(0))
                fprintf('    g(%d): %s\n', i, char(g_diff(i)));
            end
        end
    end

    % ----- B 矩阵对比 -----
    fprintf('\n--- B 矩阵 (控制输入映射) ---\n');
    B_diff = simplify(B_ne_sym - B_lag_unified);
    diff_count_B = sum(arrayfun(@(x) ~isequal(x, sym(0)), B_diff(:)));
    if diff_count_B == 0
        fprintf('  ✓ B 矩阵完全一致\n');
    else
        fprintf('  ⚠ B 矩阵存在 %d 处差异\n', diff_count_B);
        for i = 1:n_q
            for j = 1:n_u
                if ~isequal(B_diff(i,j), sym(0))
                    fprintf('    B(%d,%d): %s\n', i, j, char(B_diff(i,j)));
                end
            end
        end
    end

    % ----- 汇总 -----
    fprintf('\n--- 符号对比汇总 ---\n');
    if diff_count_M == 0 && diff_count_g == 0 && diff_count_B == 0
        fprintf('  ✓✓✓ 拉格朗日法与牛顿-欧拉法完全等价!\n');
    else
        fprintf('  ⚠ 总计 %d 处差异 (M:%d, g:%d, B:%d)\n', ...
                diff_count_M + diff_count_g + diff_count_B, ...
                diff_count_M, diff_count_g, diff_count_B);
    end
    fprintf('\n');
end

%% ========================================================================
%  Part 6: 数值对比 (代入参数)
%  ========================================================================

fprintf('========================================\n');
fprintf('Part 6: 数值参数代入——M, g, B 对比\n');
fprintf('========================================\n\n');

% 物理参数数值 (与 compute_lqr_lagrange_v2.m 保持一致)
param_val = struct( ...
    'g',     -9.81,          'R',     0.130,      'R_w',   0.386/2, ...
    'm_b',   6.9,            'I_b',   59035.925e-6, 'l_body_com', 4.3e-3, ...
    'I_yaw', 294272.34e-6,   'theta_b0', 0, ...
    'm_wl',  0.823,          'm_wr',   0.823, ...
    'I_wl',  6311.798e-6,    'I_wr',   6311.798e-6, ...
    'l_l',   0.17,           'l_r',    0.17, ...
    'm_l',   2.2,            'm_r',    2.2, ...
    'I_l',   0.034231929,    'I_r',    0.034231929, ...
    'l_leg_l_com', 0.10157,  'l_leg_r_com', 0.10157, ...
    'theta_l0', 0.582108261, 'theta_r0', 0.582108261, ...
    'm_t',   0.83,           'I_t',    29341.743e-6, ...
    'l_tail_mount_h', 0.07125, 'l_tail_mount_v', 0.1105, ...
    'l_tail_com', 0.17755,   'delta_t', 0.06597 ...
);

param_names = fieldnames(param_val);
param_subs_num = cell(length(param_names), 2);
for k = 1:length(param_names)
    param_subs_num{k,1} = eval(param_names{k});  % 符号变量
    param_subs_num{k,2} = param_val.(param_names{k});
end

% 平衡点姿态
theta_l_test = 0.3;
theta_r_test = 0.3;
theta_b_test = 0.0;
theta_t_test = 0.0;

eq_subs_num = {
    theta_l, theta_l_test;  theta_r, theta_r_test;
    theta_b, theta_b_test;  theta_t, theta_t_test;
    dtheta_l, 0;  dtheta_r, 0;  dtheta_b, 0;  dtheta_t, 0;
    dX_b_h, 0;    dphi, 0;
    phi, 0;       X_b_h, 0;
    % 零化控制输入 (静态对比)
    T_hip_r, 0; T_hip_l, 0;
    T_motor_wr, 0; T_motor_wl, 0;
    T_tail, 0;
    % 零化加速度 (g 提取残差的兜底)
    ddX_b_h, 0; ddphi, 0;
    ddtheta_l, 0; ddtheta_r, 0; ddtheta_b, 0; ddtheta_t, 0;
};

do_numerical = ne_available && lagrange_available;
if do_numerical
    % ---- 诊断: 检查 g_ne_sym 中残留的未替换符号 ----
    g_ne_after_param = subs(g_ne_sym, param_subs_num(:,1), param_subs_num(:,2));
    g_ne_after_all   = subs(g_ne_after_param, eq_subs_num(:,1), eq_subs_num(:,2));
    leftover_vars = symvar(g_ne_after_all);
    if ~isempty(leftover_vars)
        fprintf('  ⚠ g_ne_sym 残留符号: ');
        disp(leftover_vars');
    end

    M_ne_num = double(subs( ...
        subs(M_ne_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
        eq_subs_num(:,1), eq_subs_num(:,2)));
    g_ne_num = double(subs(g_ne_after_param, eq_subs_num(:,1), eq_subs_num(:,2)));
    B_ne_num = double(subs( ...
        subs(B_ne_sym, param_subs_num(:,1), param_subs_num(:,2)), ...
        eq_subs_num(:,1), eq_subs_num(:,2)));

    M_lag_num = double(subs( ...
        subs(M_lag_unified, param_subs_num(:,1), param_subs_num(:,2)), ...
        eq_subs_num(:,1), eq_subs_num(:,2)));
    g_lag_num = double(subs( ...
        subs(g_lag_unified, param_subs_num(:,1), param_subs_num(:,2)), ...
        eq_subs_num(:,1), eq_subs_num(:,2)));
    B_lag_num = double(subs( ...
        subs(B_lag_unified, param_subs_num(:,1), param_subs_num(:,2)), ...
        eq_subs_num(:,1), eq_subs_num(:,2)));

    fprintf('测试姿态: theta_l=%.1f°, theta_r=%.1f°, theta_b=%.1f°, theta_t=%.1f°\n\n', ...
            rad2deg(theta_l_test), rad2deg(theta_r_test), ...
            rad2deg(theta_b_test), rad2deg(theta_t_test));

    fprintf('--- 数值 M 矩阵对比 ---\n');
    M_num_diff = M_ne_num - M_lag_num;
    fprintf('  ||M_ne - M_lag||_inf = %.4e\n', norm(M_num_diff, 'inf'));
    fprintf('  ||M_ne - M_lag||_2   = %.4e\n', norm(M_num_diff, 'fro'));

    fprintf('\n--- 数值 g 向量对比 ---\n');
    g_num_diff = g_ne_num - g_lag_num;
    fprintf('  ||g_ne - g_lag||_inf = %.4e\n', norm(g_num_diff, 'inf'));

    fprintf('\n--- 数值 B 矩阵对比 ---\n');
    B_num_diff = B_ne_num - B_lag_num;
    fprintf('  ||B_ne - B_lag||_inf = %.4e\n', norm(B_num_diff, 'inf'));

    threshold = 1e-10;
    all_num_match = norm(M_num_diff, 'inf') < threshold && ...
                    norm(g_num_diff, 'inf') < threshold && ...
                    norm(B_num_diff, 'inf') < threshold;

    fprintf('\n--- 数值对比结论 ---\n');
    if all_num_match
        fprintf('  ✓✓✓ 数值验证通过 (阈值: %.0e)\n', threshold);
    else
        fprintf('  ⚠ 数值差异超过阈值 (%.0e)\n', threshold);
        fprintf('\n  详细 M 矩阵对比:\n');
        fprintf('  NE 法 M =\n');  disp(M_ne_num);
        fprintf('  拉格朗日法 M =\n');  disp(M_lag_num);
    end

    save('validation_results.mat', ...
         'M_ne_num', 'M_lag_num', 'M_num_diff', ...
         'g_ne_num', 'g_lag_num', 'g_num_diff', ...
         'B_ne_num', 'B_lag_num', 'B_num_diff', ...
         'M_ne_sym', 'M_lag_unified', 'M_diff', ...
         'g_ne_sym', 'g_lag_unified', 'g_diff', ...
         'B_ne_sym', 'B_lag_unified', 'B_diff');
    fprintf('\n  验证结果已保存到 validation_results.mat\n');
else
    fprintf('  两套方程不可同时用于数值对比 (跳过)\n');
end

%% ========================================================================
%  Part 7: LQR 结果对比 (如有)
%  ========================================================================

fprintf('\n========================================\n');
fprintf('Part 7: LQR 结果对比 (如有)\n');
fprintf('========================================\n\n');

ne_lqr_available  = exist('../尾巴离地/lqr_fitting_results.mat', 'file');
lag_lqr_available = exist('lqr_results_lagrange.mat', 'file');

if ~ne_lqr_available
    fprintf('  未找到 NE 法 LQR 结果\n');
end
if ~lag_lqr_available
    fprintf('  未找到拉格朗日法 LQR 结果\n');
end
if ne_lqr_available && lag_lqr_available
    load('../尾巴离地/lqr_fitting_results.mat', 'K_sample_2d_valid');
    K_ne_valid = K_sample_2d_valid;
    clear K_sample_2d_valid;

    if exist('lqr_fitting_results_lagrange.mat', 'file')
        load('lqr_fitting_results_lagrange.mat', 'K_sample_2d_valid');
        K_lag_valid = K_sample_2d_valid;

        fprintf('  NE 法有效样本: %d\n', size(K_ne_valid, 1));
        fprintf('  拉格朗日法有效样本: %d\n', size(K_lag_valid, 1));

        tol = 1e-6;
        l_target = [0.17, 0.17];
        idx_ne  = find(abs(K_ne_valid(:,1)  - l_target(1)) < tol & ...
                       abs(K_ne_valid(:,2)  - l_target(2)) < tol, 1);
        idx_lag = find(abs(K_lag_valid(:,1) - l_target(1)) < tol & ...
                       abs(K_lag_valid(:,2) - l_target(2)) < tol, 1);

        if ~isempty(idx_ne) && ~isempty(idx_lag)
            K_ne_017  = reshape(K_ne_valid(idx_ne,   3:62), 12, 5)';
            K_lag_017 = reshape(K_lag_valid(idx_lag, 3:62), 12, 5)';
            K_diff_017 = K_ne_017 - K_lag_017;
            fprintf('\n  腿长 (0.17, 0.17) K 矩阵对比:\n');
            fprintf('  ||K_ne - K_lag||_inf = %.4e\n', norm(K_diff_017, 'inf'));
            fprintf('  ||K_ne - K_lag||_2   = %.4e\n', norm(K_diff_017, 'fro'));
        end
    end
end

%% ========================================================================
%  总结
%  ========================================================================

fprintf('\n========================================\n');
fprintf('验证总结\n');
fprintf('========================================\n\n');

if ne_available && lagrange_available
    fprintf('  ✓ 加载 NE  法动力学 (已转换为新命名)\n');
    fprintf('  ✓ 加载拉格朗日法动力学\n');
    fprintf('  ✓ 运动学代换统一化\n');
    fprintf('  ✓ 符号 M/g/B 矩阵对比\n');
    if do_numerical
        fprintf('  ✓ 数值 M/g/B 矩阵对比\n');
    end
    fprintf('  ✓ 比较结果已保存到 validation_results.mat\n');
else
    if ~ne_available
        fprintf('  ✗ NE 法动力学未加载\n');
    end
    if ~lagrange_available
        fprintf('  ✗ 拉格朗日法动力学未加载\n');
    end
end

fprintf('========================================\n');
fprintf('交叉验证完成!\n');
fprintf('========================================\n');
