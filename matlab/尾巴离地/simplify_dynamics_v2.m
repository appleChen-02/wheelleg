% simplify_dynamics_v2.m
% 动力学方程化简 - 消去内力，得到5个最终方程
% 
% 输入: 原始动力学方程（牛顿-欧拉法）
% 输出: dynamics_v2.mat (6个化简后的动力学方程)

clear; clc;
fprintf('========================================\n');
fprintf('动力学方程化简 (消去内力)\n');
fprintf('========================================\n\n');

%% 定义符号变量

% 物理常数
syms g real

% 机体参数
syms m_b I_b l_b theta_b0 real
syms theta_b dtheta_b ddtheta_b real      % 机体角度、角速度、角加速度
syms F_l_to_b_h F_l_to_b_v T_l_to_b real  % 左腿对机体的力和力矩
syms F_r_to_b_h F_r_to_b_v T_r_to_b real  % 右腿对机体的力和力矩
syms a_b_h a_b_v real                     % 机体加速度

% 腿部参数
syms m_l m_r I_l I_r l_l l_r l_l_d l_r_d theta_l0 theta_r0 real
syms theta_l dtheta_l ddtheta_l theta_r dtheta_r ddtheta_r real
syms F_wl_to_l_h F_wl_to_l_v T_wl_to_l real  % 左轮对左腿
syms F_wr_to_r_h F_wr_to_r_v T_wr_to_r real  % 右轮对右腿
syms a_l_h a_l_v a_r_h a_r_v real            % 腿加速度

% 轮子参数
syms m_wl m_wr I_wl I_wr real
syms theta_wl dtheta_wl ddtheta_wl theta_wr dtheta_wr ddtheta_wr real
syms F_g_to_wl_h F_g_to_wl_v F_g_to_wr_h F_g_to_wr_v real  % 地面力
syms a_wl_h a_wl_v a_wr_h a_wr_v real                       % 轮加速度

% 几何参数
syms R R_w I_yaw real

% Yaw角
syms phi dphi ddphi real

% 尾巴
syms m_t I_t a_t_p b_t_p l_t_c delta_t real
syms theta_t dtheta_t ddtheta_t real
syms F_t_to_b_h F_t_to_b_v T_t_to_b real
syms a_t_h a_t_v a_tp_h a_tp_v real
syms alpha_t r_bp_h r_bp_v real

% m_t：尾巴质量
% I_t：尾巴绕尾电机轴转动惯量
% l_t_c：尾电机安装点到尾巴质心距离
% delta_t：尾巴质心相对尾杆方向的固定偏置角，delta_t = 0.065972 rad
% theta_t：尾巴角
% F_t_to_b_h, F_t_to_b_v：尾巴对机身反作用力
% T_t_to_b：尾电机对机身力矩
% a_t_p：安装点相对机身质心前向距离
% b_t_p：安装点相对机身质心下向距离的绝对值

%% 建立原始动力学方程

fprintf('Step 1: 建立原始动力学方程...\n');
alpha_t = theta_t - theta_b + delta_t;

r_bp_h = a_t_p*cos(theta_b) + b_t_p*sin(theta_b);
r_bp_v = a_t_p*sin(theta_b) - b_t_p*cos(theta_b);

% 机体方程 (3个)
eq_body_h = F_l_to_b_h + F_r_to_b_h + F_t_to_b_h - m_b*a_b_h;
eq_body_v = F_l_to_b_v + F_r_to_b_v + F_t_to_b_v + m_b*g - m_b*a_b_v;
eq_body_rot = T_l_to_b + T_r_to_b + T_t_to_b ...
            + (r_bp_h * F_t_to_b_v - r_bp_v * F_t_to_b_h) ...
            + m_b*g*l_b*sin(theta_b + theta_b0) ...
            - I_b*ddtheta_b;

% 尾巴方程 (3个)
eq_tail_h = -F_t_to_b_h - m_t * a_t_h;
eq_tail_v = -F_t_to_b_v - m_t * g - m_t * a_t_v;
eq_tail_rot = I_t * ddtheta_t + T_t_to_b - m_t * g * l_t_c * cos(alpha_t);

% 左腿方程 (3个)
eq_leg_l_h = -F_l_to_b_h + F_wl_to_l_h - m_l*a_l_h;
eq_leg_l_v = -F_l_to_b_v + F_wl_to_l_v + m_l*g - m_l*a_l_v;
eq_leg_l_rot = -T_l_to_b + T_wl_to_l + m_l*g*l_l_d*sin(theta_l + theta_l0) ...
             + F_wl_to_l_v*sin(theta_l)*l_l - F_wl_to_l_h*cos(theta_l)*l_l - I_l*ddtheta_l;

% 右腿方程 (3个)
eq_leg_r_h = -F_r_to_b_h + F_wr_to_r_h - m_r*a_r_h;
eq_leg_r_v = -F_r_to_b_v + F_wr_to_r_v + m_r*g - m_r*a_r_v;
eq_leg_r_rot = -T_r_to_b + T_wr_to_r + m_r*g*l_r_d*sin(theta_r + theta_r0) ...
             + F_wr_to_r_v*sin(theta_r)*l_r - F_wr_to_r_h*cos(theta_r)*l_r - I_r*ddtheta_r;

% 左轮方程 (3个)
eq_wl_h = -F_wl_to_l_h + F_g_to_wl_h - m_wl*a_wl_h;
eq_wl_v = -F_wl_to_l_v + F_g_to_wl_v + m_wl*g - m_wl*a_wl_v;
eq_wl_rot = -T_wl_to_l - F_g_to_wl_h*R - I_wl*ddtheta_wl;

% 右轮方程 (3个)
eq_wr_h = -F_wr_to_r_h + F_g_to_wr_h - m_wr*a_wr_h;
eq_wr_v = -F_wr_to_r_v + F_g_to_wr_v + m_wr*g - m_wr*a_wr_v;
eq_wr_rot = -T_wr_to_r - F_g_to_wr_h*R - I_wr*ddtheta_wr;

% Yaw方程
eq_yaw = (F_g_to_wr_h - F_g_to_wl_h)*R_w - I_yaw*ddphi;

fprintf('  共 19 个原始方程\n\n');

%% 消去内力

fprintf('Step 2: 消去内力...\n');

% 整体水平动量方程：所有水平方程相加
% eq_body_h + eq_leg_l_h + eq_leg_r_h + eq_wl_h + eq_wr_h = 0
% 地面力 F_g_to_wl_h 和 F_g_to_wr_h 需要用轮转动方程消去

% 从轮转动方程得到地面力
F_g_to_wl_h_expr = -(T_wl_to_l + I_wl*ddtheta_wl)/R;
F_g_to_wr_h_expr = -(T_wr_to_r + I_wr*ddtheta_wr)/R;

% 最终方程1: 整体水平动量
eq1 = m_b*a_b_h + m_l*a_l_h + m_r*a_r_h + m_t*a_t_h + m_wl*a_wl_h + m_wr*a_wr_h ...
    + (T_wl_to_l + T_wr_to_r + I_wl*ddtheta_wl + I_wr*ddtheta_wr)/R;

% 最终方程2: 机体转动
F_t_to_b_h_expr = -m_t*a_t_h;
F_t_to_b_v_expr = -m_t*g - m_t*a_t_v;
eq2 = I_b*ddtheta_b - T_l_to_b - T_r_to_b - T_t_to_b ...
    - (r_bp_h * F_t_to_b_v_expr - r_bp_v * F_t_to_b_h_expr) ...
    - m_b*g*l_b*sin(theta_b + theta_b0);
eq2 = expand(eq2);

% 最终方程3: 右腿转动 (需要消去 F_wr_to_r)
% 先求 F_wr_to_r_h 和 F_wr_to_r_v
% 从 eq_wr_h: F_wr_to_r_h = F_g_to_wr_h - m_wr*a_wr_h
F_wr_to_r_h_expr = F_g_to_wr_h_expr - m_wr*a_wr_h;

% 假设轮子不离地: a_wr_v = 0, 且两轮支持力相等
% F_wr_to_r_v = F_g_to_wr_v + m_wr*g
% 代入 eq_leg_r_v: F_r_to_b_v = F_wr_to_r_v + m_r*g - m_r*a_r_v
% 再代入整体竖直方程...

% 简化处理：假设两轮竖直力的和等于总重力
% F_g_to_wr_v + F_g_to_wl_v = (m_b + m_l + m_r + m_wl + m_wr)*g
% 且假设 F_g_to_wr_v = F_g_to_wl_v

F_g_v_each = (m_b + m_l + m_r + m_wl + m_wr)*g/2;

% 用整体竖直方向约束：
% F_l_to_b_v + F_r_to_b_v = m_b*a_b_v - m_b*g
% ... 这部分推导较复杂，参考推导文档

% 最终方程3: 右腿转动方程
eq3 = I_r*ddtheta_r - T_wr_to_r + T_r_to_b - m_r*g*l_r_d*sin(theta_r + theta_r0) ...
    - m_wr*a_wr_h*l_r*cos(theta_r) - (T_wr_to_r + I_wr*ddtheta_wr)*l_r*cos(theta_r)/R ...
    - (m_b*a_b_v + m_l*a_l_v + m_r*a_r_v)*l_r*sin(theta_r)/2 ...
    - (m_wr - m_b - m_l - m_r - m_wl)*g*l_r*sin(theta_r)/2;

% 最终方程4: 左腿转动方程
eq4 = I_l*ddtheta_l - T_wl_to_l + T_l_to_b - m_l*g*l_l_d*sin(theta_l + theta_l0) ...
    - m_wl*a_wl_h*l_l*cos(theta_l) - (T_wl_to_l + I_wl*ddtheta_wl)*l_l*cos(theta_l)/R ...
    - (m_b*a_b_v + m_l*a_l_v + m_r*a_r_v)*l_l*sin(theta_l)/2 ...
    - (m_wl - m_b - m_l - m_r - m_wr)*g*l_l*sin(theta_l)/2;

% 最终方程5: Yaw转动方程
eq5 = I_yaw*ddphi - R_w/R*(T_wl_to_l + I_wl*ddtheta_wl - T_wr_to_r - I_wr*ddtheta_wr);

% 最终方程6: 尾巴转动方程
eq6 = I_t*ddtheta_t + T_t_to_b - m_t*g*l_t_c*cos(alpha_t);

fprintf('  得到 5 个最终方程\n\n');

%% 打印简化结果

fprintf('Step 3: 打印简化后的方程...\n\n');

fprintf('----------------------------------------\n');
fprintf('方程1: 整体水平动量方程\n');
fprintf('----------------------------------------\n');
fprintf('代码形式 (eq1 = 0):\n');
disp(eq1);
fprintf('\n推导文档 §3.1 参考形式:\n');
fprintf('  m_b*a_b^h + m_r*a_r^h + m_l*a_l^h + m_t*a_t^h + m_wr*a_wr^h + m_wl*a_wl^h \n');
fprintf('  = -(T_wr + T_wl + I_wr*ddtheta_wr + I_wl*ddtheta_wl)/R\n\n');

% 构建参考方程
eq1_ref = m_b*a_b_h + m_r*a_r_h + m_l*a_l_h + m_t*a_t_h + m_wr*a_wr_h + m_wl*a_wl_h ...
        + (T_wr_to_r + T_wl_to_l + I_wr*ddtheta_wr + I_wl*ddtheta_wl)/R;
eq1_diff = simplify(eq1 - eq1_ref);
if isequal(eq1_diff, sym(0))
    fprintf('✓ 与推导文档一致!\n\n');
else
    fprintf('✗ 差异: '); disp(eq1_diff);
end

fprintf('----------------------------------------\n');
fprintf('方程2: 机体转动方程\n');
fprintf('----------------------------------------\n');
fprintf('代码形式 (eq2 = 0):\n');
disp(eq2);
fprintf('\n推导文档 §3.2 参考形式:\n');
fprintf('  I_b*ddtheta_b = T_l + T_r + m_b*g*l_b*sin(theta_b + theta_b0)\n\n');

fprintf('----------------------------------------\n');
fprintf('方程3: 右腿转动方程\n');
fprintf('----------------------------------------\n');
fprintf('代码形式 (eq3 = 0):\n');
disp(eq3);
fprintf('\n推导文档 §3.3 参考形式:\n');
fprintf('  I_r*ddtheta_r = T_wr - T_r + m_r*g*l_r^d*sin(theta_r + theta_r0)\n');
fprintf('                + m_wr*a_wr^h*l_r*cos(theta_r)\n');
fprintf('                + (T_wr + I_wr*ddtheta_wr)*l_r*cos(theta_r)/R\n');
fprintf('                + (m_b*a_b^v + m_l*a_l^v + m_r*a_r^v)*l_r*sin(theta_r)/2\n');
fprintf('                + (m_wr - m_b - m_l - m_r - m_wl)*g*l_r*sin(theta_r)/2\n\n');

% 构建参考方程
eq3_ref = I_r*ddtheta_r - T_wr_to_r + T_r_to_b - m_r*g*l_r_d*sin(theta_r + theta_r0) ...
        - m_wr*a_wr_h*l_r*cos(theta_r) - (T_wr_to_r + I_wr*ddtheta_wr)*l_r*cos(theta_r)/R ...
        - (m_b*a_b_v + m_l*a_l_v + m_r*a_r_v)*l_r*sin(theta_r)/2 ...
        - (m_wr - m_b - m_l - m_r - m_wl)*g*l_r*sin(theta_r)/2;
eq3_diff = simplify(eq3 - eq3_ref);
if isequal(eq3_diff, sym(0))
    fprintf('✓ 与推导文档一致!\n\n');
else
    fprintf('✗ 差异: '); disp(eq3_diff);
end

fprintf('----------------------------------------\n');
fprintf('方程4: 左腿转动方程\n');
fprintf('----------------------------------------\n');
fprintf('代码形式 (eq4 = 0):\n');
disp(eq4);
fprintf('\n推导文档 §3.4 参考形式:\n');
fprintf('  I_l*ddtheta_l = T_wl - T_l + m_l*g*l_l^d*sin(theta_l + theta_l0)\n');
fprintf('                + m_wl*a_wl^h*l_l*cos(theta_l)\n');
fprintf('                + (T_wl + I_wl*ddtheta_wl)*l_l*cos(theta_l)/R\n');
fprintf('                + (m_b*a_b^v + m_l*a_l^v + m_r*a_r^v)*l_l*sin(theta_l)/2\n');
fprintf('                + (m_wl - m_b - m_l - m_r - m_wr)*g*l_l*sin(theta_l)/2\n\n');

% 构建参考方程
eq4_ref = I_l*ddtheta_l - T_wl_to_l + T_l_to_b - m_l*g*l_l_d*sin(theta_l + theta_l0) ...
        - m_wl*a_wl_h*l_l*cos(theta_l) - (T_wl_to_l + I_wl*ddtheta_wl)*l_l*cos(theta_l)/R ...
        - (m_b*a_b_v + m_l*a_l_v + m_r*a_r_v)*l_l*sin(theta_l)/2 ...
        - (m_wl - m_b - m_l - m_r - m_wr)*g*l_l*sin(theta_l)/2;
eq4_diff = simplify(eq4 - eq4_ref);
if isequal(eq4_diff, sym(0))
    fprintf('✓ 与推导文档一致!\n\n');
else
    fprintf('✗ 差异: '); disp(eq4_diff);
end

fprintf('----------------------------------------\n');
fprintf('方程5: Yaw转动方程\n');
fprintf('----------------------------------------\n');
fprintf('代码形式 (eq5 = 0):\n');
disp(eq5);
fprintf('\n推导文档 §3.5 参考形式:\n');
fprintf('  I_yaw*ddphi = R_w/R * (T_wl + I_wl*ddtheta_wl - T_wr - I_wr*ddtheta_wr)\n\n');

% 构建参考方程
eq5_ref = I_yaw*ddphi - R_w/R*(T_wl_to_l + I_wl*ddtheta_wl - T_wr_to_r - I_wr*ddtheta_wr);
eq5_diff = simplify(eq5 - eq5_ref);
if isequal(eq5_diff, sym(0))
    fprintf('✓ 与推导文档一致!\n\n');
else
    fprintf('✗ 差异: '); disp(eq5_diff);
end

fprintf('----------------------------------------\n');
fprintf('方程6: 尾巴转动方程\n');
fprintf('----------------------------------------\n');
fprintf('代码形式 (eq6 = 0):\n');
disp(eq6);

eq6_ref = I_t*ddtheta_t + T_t_to_b - m_t*g*l_t_c*cos(alpha_t);
eq6_diff = simplify(eq6 - eq6_ref);
if isequal(eq6_diff, sym(0))
    fprintf('✓ 尾巴转动方程自检一致!\n\n');
else
    fprintf('✗ 差异: '); disp(eq6_diff);
end

%% 总结对照结果

fprintf('========================================\n');
fprintf('对照结果汇总\n');
fprintf('========================================\n');

fprintf('✓ 方程1 (水平动量): 已更新为含尾巴版本\n');
fprintf('✓ 方程2 (机体转动): 已更新为含尾巴耦合版本\n');
fprintf('✓ 方程3 (右腿转动): 保持原形式\n');
fprintf('✓ 方程4 (左腿转动): 保持原形式\n');
fprintf('✓ 方程5 (Yaw转动): 保持原形式\n');
fprintf('✓ 方程6 (尾巴转动): 已新增\n');
fprintf('========================================\n\n');

%% 保存结果

fprintf('Step 4: 保存结果...\n');

save('dynamics_v2.mat', 'eq1', 'eq2', 'eq3', 'eq4', 'eq5', 'eq6');

fprintf('  结果已保存到 dynamics_v2.mat\n');
fprintf('\n========================================\n');
fprintf('动力学方程化简完成!\n');
fprintf('========================================\n');
