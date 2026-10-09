#include "balance_controller.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

// 控制器使用的一组多项式系数和内部工具函数，均为模块内部实现细节。
float k_p11[4] = {-144.7042f, 85.7292f, 110.9807f, 13.3789f};
float k_p12[4] = {-45.6237f, 52.0270f, 7.8988f, 2.0285f};
float k_p13[4] = {45.8036f, -67.7061f, 34.5778f, 11.1415f};
float k_p14[4] = {-30.8544f, 7.9186f, 11.9450f, 11.8960f};
float k_p15[4] = {-53.0698f, -9.7542f, 40.7518f, -18.5331f};
float k_p16[4] = {0.4310f, -3.4549f, 3.5258f, -1.7532f};
float k_p21[4] = {-383.1158f, 303.6294f, -65.6044f, -25.7404f};
float k_p22[4] = {-32.4821f, 27.4416f, -9.4766f, -3.6351f};
float k_p23[4] = {-99.9912f, -2.1225f, 60.5734f, -28.3204f};
float k_p24[4] = {38.3503f, -106.3085f, 80.3793f, -26.7960f};
float k_p25[4] = {-135.7502f, 190.2670f, -94.3212f, -26.6641f};
float k_p26[4] = {-11.5221f, 16.0327f, -8.2076f, -0.75487f};

// MPC 指数增益系数
float MPC_p1[4]  = {-0.005178f, -3.313708f, -0.005235f, -3.313719f};
float MPC_p2[4]  = {-0.004400f, -5.996262f, -0.006948f, -5.992568f};
float MPC_p3[4]  = {0.000063f, -1.834633f, 0.000063f, -1.834633f};
float MPC_p4[4]  = {0.006488f, -2.061431f, 0.006489f, -2.061433f};
float MPC_p5[4]  = {-0.003766f, -0.015098f, -0.003755f, -0.015105f};
float MPC_p6[4]  = {-0.019946f, -0.014435f, -0.019947f, -0.014437f};
float MPC_p7[4]  = {-0.005178f, -3.313708f, -0.005235f, -3.313719f};
float MPC_p8[4]  = {-0.004400f, -5.996262f, -0.006948f, -5.992568f};
float MPC_p9[4]  = {0.000063f, -1.834633f, 0.000063f, -1.834633f};
float MPC_p10[4] = {0.006488f, -2.061431f, 0.006489f, -2.061433f};
float MPC_p11[4] = {-0.003766f, -0.015098f, -0.003755f, -0.015105f};
float MPC_p12[4] = {-0.019946f, -0.014435f, -0.019947f, -0.014437f};

// 将数值限制在 +/- limit 范围内。
double Clip(double value, double limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// 计算三次多项式 p0*l^3 + p1*l^2 + p2*l + p3
double Poly3(const float p[4], double l) {
    return p[0] * l * l * l + p[1] * l * l + p[2] * l + p[3];
}

// 根据 l 值（腿长近似）计算 LQR 增益矩阵 k。
void GetKNoTail(double l, double k[2][6]) {
    k[0][0] = Poly3(k_p11, l);
    k[0][1] = Poly3(k_p12, l);
    k[0][2] = Poly3(k_p13, l);
    k[0][3] = Poly3(k_p14, l);
    k[0][4] = Poly3(k_p15, l);
    k[0][5] = Poly3(k_p16, l);

    k[1][0] = Poly3(k_p21, l);
    k[1][1] = Poly3(k_p22, l);
    k[1][2] = Poly3(k_p23, l);
    k[1][3] = Poly3(k_p24, l);
    k[1][4] = Poly3(k_p25, l);
    k[1][5] = Poly3(k_p26, l);
}

// 对给定状态向量 x 与增益 k 计算 wheel_torque 和 tp（内环输出）。
void CalcLQRNoTail(const double k[2][6], const double x[6], double* wheel_torque, double* tp) {
    *wheel_torque = 0.0;
    *tp = 0.0;
    for (int i = 0; i < 6; ++i) {
        *wheel_torque += k[0][i] * x[i];
        *tp += k[1][i] * x[i];
    }
}

// 计算 MPC 指数增益矩阵 MPC_k[2][6]。
// MPC_k[0]: wheel torque 行（用于 delta 状态修正 Tp）
// MPC_k[1]: Tp 行（用于覆盖到 LQR 增益矩阵的 row 1）
void GetMPCK(double l, double MPC_k[2][6]) {
    MPC_k[0][0] = MPC_p1[0] * std::exp(MPC_p1[1] * l) + MPC_p1[2] * std::exp(MPC_p1[3] * l);
    MPC_k[0][1] = MPC_p2[0] * std::exp(MPC_p2[1] * l) + MPC_p2[2] * std::exp(MPC_p2[3] * l);
    MPC_k[0][2] = MPC_p3[0] * std::exp(MPC_p3[1] * l) + MPC_p3[2] * std::exp(MPC_p3[3] * l);
    MPC_k[0][3] = MPC_p4[0] * std::exp(MPC_p4[1] * l) + MPC_p4[2] * std::exp(MPC_p4[3] * l);
    MPC_k[0][4] = MPC_p5[0] * std::exp(MPC_p5[1] * l) + MPC_p5[2] * std::exp(MPC_p5[3] * l);
    MPC_k[0][5] = MPC_p6[0] * std::exp(MPC_p6[1] * l) + MPC_p6[2] * std::exp(MPC_p6[3] * l);
    MPC_k[1][0] = MPC_p7[0] * std::exp(MPC_p7[1] * l) + MPC_p7[2] * std::exp(MPC_p7[3] * l);
    MPC_k[1][1] = MPC_p8[0] * std::exp(MPC_p8[1] * l) + MPC_p8[2] * std::exp(MPC_p8[3] * l);
    MPC_k[1][2] = MPC_p9[0] * std::exp(MPC_p9[1] * l) + MPC_p9[2] * std::exp(MPC_p9[3] * l);
    MPC_k[1][3] = MPC_p10[0] * std::exp(MPC_p10[1] * l) + MPC_p10[2] * std::exp(MPC_p10[3] * l);
    MPC_k[1][4] = MPC_p11[0] * std::exp(MPC_p11[1] * l) + MPC_p11[2] * std::exp(MPC_p11[3] * l);
    MPC_k[1][5] = MPC_p12[0] * std::exp(MPC_p12[1] * l) + MPC_p12[2] * std::exp(MPC_p12[3] * l);
}

// 用 MPC 增益矩阵 row 0 与 delta 状态向量计算 Tp 修正量。
// 对应 MCU 中 CalcMPC(MPC_k, delta_x, &Delta_Tp) → Tp += Delta_Tp
void CalcMPC(const double MPC_k[2][6], const double delta_x[6], double* delta_Tp) {
    *delta_Tp = MPC_k[0][0] * delta_x[0] + MPC_k[0][1] * delta_x[1] + MPC_k[0][2] * delta_x[2]
              + MPC_k[0][3] * delta_x[3] + MPC_k[0][4] * delta_x[4] + MPC_k[0][5] * delta_x[5];
}

// CalcVmc 已移为 balance_controller.h 内联函数

// 根据当前roll角与目标roll角计算两腿长度期望差值
// Ld0: 当前左右腿长度差 (L_left - L_right)
// theta0: 当前roll角 (rad)
// theta1: 目标roll角 (rad)
// 返回: 两腿长度期望差值 (m)
double CalcLegLengthDiff(double Ld0, double theta0, double theta1) {
    const double roll_error = theta0 - theta1;
    if (roll_error > -kChassisRollDeadband && roll_error < kChassisRollDeadband) {
        theta0 = theta1;
    }
    return kWheelBase * std::tan(theta1) -
           std::cos(theta0) / std::cos(theta1) * (kWheelBase * std::tan(theta0) - Ld0);
}

// 双腿腿长协调控制：将期望差值对称分配到左右腿目标，并限幅
void CoordinateLegLength(double* Ll_ref, double* Lr_ref, double diff, double add) {
    *Ll_ref = *Ll_ref - diff * 0.5 - add;
    *Lr_ref = *Lr_ref + diff * 0.5 + add;

    double* short_leg = (*Ll_ref < *Lr_ref) ? Ll_ref : Lr_ref;
    double* long_leg  = (*Ll_ref < *Lr_ref) ? Lr_ref : Ll_ref;
    double move = kMinLegLength - *short_leg;
    if (move > 0.0) {
        *short_leg += move;
        *long_leg  += move;
    }
    if (*long_leg > kMaxLegLength) {
        *long_leg = kMaxLegLength;
    }
}

// Tripod 3×8 LQR 增益系数表（对应 MCU k_t11~k_t38）
float k_t11[4] = {6.2469f, -41.9148f, 106.0088f, 5.6077f};
float k_t12[4] = {-30.7177f, 37.0456f, 9.0448f, 1.1568f};
float k_t13[4] = {173.8033f, -165.2549f, 60.3308f, 6.0085f};
float k_t14[4] = {77.5643f, -79.0785f, 38.6368f, 6.9574f};
float k_t15[4] = {-5.6619f, -37.8008f, 42.4011f, -13.4443f};
float k_t16[4] = {-2.6918f, -0.1922f, 3.5526f, -1.2572f};
float k_t17[4] = {25.4018f, -18.6926f, -1.0141f, 1.1976f};
float k_t18[4] = {3.9845f, -4.4867f, 0.33002f, -0.05243f};
float k_t21[4] = {-224.5940f, 186.2636f, -49.0933f, -8.5985f};
float k_t22[4] = {-18.8520f, 17.4482f, -7.6320f, -1.4785f};
float k_t23[4] = {-50.2231f, -13.8903f, 40.2927f, -17.7323f};
float k_t24[4] = {0.71901f, -41.7974f, 38.8125f, -14.9342f};
float k_t25[4] = {-111.6966f, 124.1295f, -64.1916f, -34.3698f};
float k_t26[4] = {-10.2428f, 11.8119f, -6.2077f, -1.6657f};
float k_t27[4] = {-4.4906f, 0.99929f, 14.4076f, -2.6014f};
float k_t28[4] = {-1.2232f, 0.86123f, 0.97338f, -0.69476f};
float k_t31[4] = {-6.3241f, -1.0043f, 9.0027f, 2.0114f};
float k_t32[4] = {-4.7162f, 3.8400f, 1.4091f, 0.28278f};
float k_t33[4] = {-49.1874f, 45.5032f, -17.3532f, 6.6594f};
float k_t34[4] = {-45.9416f, 40.8871f, -13.8370f, 4.7884f};
float k_t35[4] = {14.3123f, -16.6531f, 7.6109f, -2.5118f};
float k_t36[4] = {2.2876f, -2.3653f, 1.1076f, -0.25205f};
float k_t37[4] = {0.45991f, 0.3907f, -0.61614f, 0.078056f};
float k_t38[4] = {0.3279f, -0.28074f, -0.046702f, -0.02279f};

// 根据腿长 l 计算 Tripod 模式 3×8 LQR 增益矩阵
// k[0]: wheel torque行, k[1]: Tp行, k[2]: tail torque行
void GetK_Tripod(double l, double k[3][8]) {
    double t1 = l;
    double t2 = l * l;
    double t3 = l * l * l;

    k[0][0] = k_t11[0] * t3 + k_t11[1] * t2 + k_t11[2] * t1 + k_t11[3];
    k[0][1] = k_t12[0] * t3 + k_t12[1] * t2 + k_t12[2] * t1 + k_t12[3];
    k[0][2] = k_t13[0] * t3 + k_t13[1] * t2 + k_t13[2] * t1 + k_t13[3];
    k[0][3] = k_t14[0] * t3 + k_t14[1] * t2 + k_t14[2] * t1 + k_t14[3];
    k[0][4] = k_t15[0] * t3 + k_t15[1] * t2 + k_t15[2] * t1 + k_t15[3];
    k[0][5] = k_t16[0] * t3 + k_t16[1] * t2 + k_t16[2] * t1 + k_t16[3];
    k[0][6] = k_t17[0] * t3 + k_t17[1] * t2 + k_t17[2] * t1 + k_t17[3];
    k[0][7] = k_t18[0] * t3 + k_t18[1] * t2 + k_t18[2] * t1 + k_t18[3];

    k[1][0] = k_t21[0] * t3 + k_t21[1] * t2 + k_t21[2] * t1 + k_t21[3];
    k[1][1] = k_t22[0] * t3 + k_t22[1] * t2 + k_t22[2] * t1 + k_t22[3];
    k[1][2] = k_t23[0] * t3 + k_t23[1] * t2 + k_t23[2] * t1 + k_t23[3];
    k[1][3] = k_t24[0] * t3 + k_t24[1] * t2 + k_t24[2] * t1 + k_t24[3];
    k[1][4] = k_t25[0] * t3 + k_t25[1] * t2 + k_t25[2] * t1 + k_t25[3];
    k[1][5] = k_t26[0] * t3 + k_t26[1] * t2 + k_t26[2] * t1 + k_t26[3];
    k[1][6] = k_t27[0] * t3 + k_t27[1] * t2 + k_t27[2] * t1 + k_t27[3];
    k[1][7] = k_t28[0] * t3 + k_t28[1] * t2 + k_t28[2] * t1 + k_t28[3];

    k[2][0] = k_t31[0] * t3 + k_t31[1] * t2 + k_t31[2] * t1 + k_t31[3];
    k[2][1] = k_t32[0] * t3 + k_t32[1] * t2 + k_t32[2] * t1 + k_t32[3];
    k[2][2] = k_t33[0] * t3 + k_t33[1] * t2 + k_t33[2] * t1 + k_t33[3];
    k[2][3] = k_t34[0] * t3 + k_t34[1] * t2 + k_t34[2] * t1 + k_t34[3];
    k[2][4] = k_t35[0] * t3 + k_t35[1] * t2 + k_t35[2] * t1 + k_t35[3];
    k[2][5] = k_t36[0] * t3 + k_t36[1] * t2 + k_t36[2] * t1 + k_t36[3];
    k[2][6] = k_t37[0] * t3 + k_t37[1] * t2 + k_t37[2] * t1 + k_t37[3];
    k[2][7] = k_t38[0] * t3 + k_t38[1] * t2 + k_t38[2] * t1 + k_t38[3];
}

// 3×8 LQR 矩阵 × 8维状态向量 → [wheel_torque, Tp, tail_torque]
void CalcLQR_Tail(const double k[3][8], const double x[8], double T_Tp_Tt[3]) {
    T_Tp_Tt[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3]
               + k[0][4] * x[4] + k[0][5] * x[5] + k[0][6] * x[6] + k[0][7] * x[7];
    T_Tp_Tt[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3]
               + k[1][4] * x[4] + k[1][5] * x[5] + k[1][6] * x[6] + k[1][7] * x[7];
    T_Tp_Tt[2] = k[2][0] * x[0] + k[2][1] * x[1] + k[2][2] * x[2] + k[2][3] * x[3]
               + k[2][4] * x[4] + k[2][5] * x[5] + k[2][6] * x[6] + k[2][7] * x[7];
}

// Tripod 几何平衡求解器（对应 MCU SolveThetaBetaBodyCenter）
// 根据目标腿长 l 和 pitch 初值，搜索满足尾巴触地 + 水平力平衡的 theta 和 pitch。
void SolveThetaBetaBodyCenter(double l, double& theta_ref, double& phi_ref) {
    double best_theta = 0.0;
    double best_phi = phi_ref;
    double best_res = 1e9;

    const double beta = kTailBetaInitDeg * 3.14159265358979323846 / 180.0;
    const double phi_min = -1.0;
    const double phi_max = 0.0;
    constexpr int N = 160;

    for (int i = 0; i <= N; ++i) {
        double phi = phi_min + (phi_max - phi_min) * static_cast<double>(i) / static_cast<double>(N);

        double c = (kTailPosOffsetVertical * std::cos(phi)
                 - kTailPosOffsetHorizon  * std::sin(phi)
                 + kTailLength * std::sin(beta - phi + kTailBetaOmniToHand)
                 + kTailWheelRadius - kWheelRadius) / l;

        if (c < -1.0 || c > 1.0) continue;

        double theta = std::acos(c);

        double lhs = kTailPosOffsetVertical * std::sin(phi)
                   + kTailPosOffsetHorizon  * std::cos(phi)
                   + kTailLength * std::cos(beta - phi + kTailBetaOmniToHand);

        double rhs = 2.0 * l * std::sin(theta);

        double r = std::abs(lhs - rhs);

        if (r < best_res) {
            best_res = r;
            best_theta = theta;
            best_phi = phi;
        }
    }

    theta_ref = best_theta;
    phi_ref = best_phi;
}

// theta 平衡偏移系数（二元二次曲面拟合）
// [0]: theta_l_eq, [1]: theta_r_eq, [2]: theta_b_eq
float Offset_coef[3][6] = {
    {0.535811f, -3.06383f, 1.49966e-15f, 4.94312f, -1.73472e-15f, -2.44381e-15f},
    {0.535811f, 3.97708e-15f, -3.06383f, -7.34786e-15f, 2.24087e-15f, 4.94312f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

// 根据左右腿长计算 theta 平衡偏移（对应 MCU GetTheta_Pro_NoTail）
void GetTheta_Pro_NoTail(double l_l, double l_r, double theta_eq[3]) {
    double theta_l_eq = Offset_coef[0][0] + Offset_coef[0][1] * l_l + Offset_coef[0][2] * l_r
                      + Offset_coef[0][3] * l_l * l_l + Offset_coef[0][4] * l_l * l_r
                      + Offset_coef[0][5] * l_r * l_r;
    double theta_r_eq = Offset_coef[1][0] + Offset_coef[1][1] * l_l + Offset_coef[1][2] * l_r
                      + Offset_coef[1][3] * l_l * l_l + Offset_coef[1][4] * l_l * l_r
                      + Offset_coef[1][5] * l_r * l_r;
    double theta_b_eq = Offset_coef[2][0] + Offset_coef[2][1] * l_l + Offset_coef[2][2] * l_r
                      + Offset_coef[2][3] * l_l * l_l + Offset_coef[2][4] * l_l * l_r
                      + Offset_coef[2][5] * l_r * l_r;
    theta_eq[0] = theta_l_eq;
    theta_eq[1] = theta_r_eq;
    theta_eq[2] = theta_b_eq;
}

// ProX_Tripod: 4×9 LQR 增益拟合系数（对齐 MCU K_coef_Tripod TAIL_VERSION=1）
// K[n] = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
// 布局: [row][col] 共 4行×9列 = 36组系数
float K_coef_Tripod[36][6] = {
    {-7.56369f, 25.2028f, -29.0859f, -13.1791f, -24.2494f, 55.2151f},        // K[0][0]
    {-14.7224f, 53.7722f, -32.777f, -27.109f, -75.1576f, 89.035f},           // K[0][1]
    {-11.4306f, -11.5429f, -68.4109f, 9.74123f, -46.0447f, 110.799f},        // K[0][2]
    {-1.38634f, -0.240085f, -13.6555f, -1.80924f, -3.53848f, 16.7576f},      // K[0][3]
    {-1.493f, -0.573377f, 7.77841f, 23.8389f, 15.2945f, 4.67106f},           // K[0][4]
    {-0.115468f, 0.370541f, 0.743852f, 0.910853f, -1.01291f, 0.133896f},     // K[0][5]
    {-32.9037f, 15.7369f, -27.4663f, -18.0977f, -5.34635f, 31.7268f},        // K[0][6]
    {-3.50488f, 0.543068f, 2.77077f, -0.685711f, 0.614437f, -4.42839f},      // K[0][7]
    {-0.402051f, 2.25568f, 1.02011f, -1.86307f, -3.08887f, 1.54868f},        // K[0][8]
    {-7.56369f, -29.0859f, 25.2028f, 55.2151f, -24.2494f, -13.1791f},        // K[1][0]
    {-14.7224f, -32.777f, 53.7722f, 89.035f, -75.1576f, -27.109f},           // K[1][1]
    {11.4306f, 68.4109f, 11.5429f, -110.799f, 46.0447f, -9.74123f},          // K[1][2]
    {1.38634f, 13.6555f, 0.240085f, -16.7576f, 3.53848f, 1.80924f},          // K[1][3]
    {-32.9037f, -27.4663f, 15.7369f, 31.7268f, -5.34635f, -18.0977f},        // K[1][4]
    {-3.50488f, 2.77077f, 0.543068f, -4.42839f, 0.614437f, -0.685711f},      // K[1][5]
    {-1.493f, 7.77841f, -0.573377f, 4.67106f, 15.2945f, 23.8389f},           // K[1][6]
    {-0.115468f, 0.743852f, 0.370541f, 0.133896f, -1.01291f, 0.910853f},     // K[1][7]
    {-0.402051f, 1.02011f, 2.25568f, 1.54868f, -3.08887f, -1.86307f},        // K[1][8]
    {-1.40906f, 0.165371f, 11.2637f, 1.83378f, -5.88074f, -7.79185f},        // K[2][0]
    {-0.726048f, -3.45046f, 16.3663f, 4.73718f, -2.80751f, -15.8796f},       // K[2][1]
    {-13.9946f, -10.3398f, 21.6309f, 7.73079f, 7.54722f, -17.8688f},         // K[2][2]
    {-1.78479f, -2.46268f, 3.14336f, 2.54568f, 0.41034f, -0.974715f},        // K[2][3]
    {0.206293f, 10.4468f, 2.66006f, 1.7799f, -4.99877f, -7.66388f},          // K[2][4]
    {0.0884817f, 0.357163f, 0.109836f, 1.2631f, 0.102089f, -0.439532f},      // K[2][5]
    {2.86443f, -1.57946f, 31.1904f, 3.01609f, -7.36168f, -13.3038f},         // K[2][6]
    {0.361537f, -0.0175551f, 2.05916f, -0.0201878f, 0.176891f, -0.490603f},  // K[2][7]
    {0.219756f, -0.365153f, 0.470607f, 0.36015f, -0.0374971f, -0.833573f},   // K[2][8]
    {-1.40906f, 11.2637f, 0.165371f, -7.79185f, -5.88074f, 1.83378f},        // K[3][0]
    {-0.726048f, 16.3663f, -3.45046f, -15.8796f, -2.80751f, 4.73718f},       // K[3][1]
    {13.9946f, -21.6309f, 10.3398f, 17.8688f, -7.54722f, -7.73079f},         // K[3][2]
    {1.78479f, -3.14336f, 2.46268f, 0.974715f, -0.41034f, -2.54568f},        // K[3][3]
    {2.86443f, 31.1904f, -1.57946f, -13.3038f, -7.36168f, 3.01609f},         // K[3][4]
    {0.361537f, 2.05916f, -0.0175551f, -0.490603f, 0.176891f, -0.0201878f},  // K[3][5]
    {0.206293f, 2.66006f, 10.4468f, -7.66388f, -4.99877f, 1.7799f},          // K[3][6]
    {0.0884817f, 0.109836f, 0.357163f, -0.439532f, 0.102089f, 1.2631f},      // K[3][7]
    {0.219756f, 0.470607f, -0.365153f, -0.833573f, -0.0374971f, 0.36015f}    // K[3][8]
};

// ProX_Tripod: Tp/Tt 平衡偏移系数（对齐 MCU T0_coef_Tripod TAIL_VERSION=1）
float T0_coef_Tripod[2][6] = {
    {0.431191f, 4.2145e-14f, -5.19577e-16f, -8.29016e-14f, 5.03629e-16f, 3.13309e-16f},
    {-1.44252f, -2.46897e-14f, 5.39437e-16f, 3.88503e-14f, -3.8052e-15f, 2.00518e-15f}
};

// ProX_Tripod: theta 平衡偏移系数 Bipedal 版（对齐 MCU Offset_coef_Bipedal TAIL_VERSION=1）
float Offset_coef_Bipedal[4][6] = {
    {0.867123f, -4.62177f, 2.80193e-15f, 7.26101f, -5.02487e-15f, -3.41413e-15f},
    {0.867123f, 4.80411e-16f, -4.62177f, -1.59823e-15f, 1.18646e-15f, 7.26101f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

// 最新 MCU TAIL_VERSION=1 的 Bipedal 5x12 LQR 双腿长拟合系数。
float K_coef_Bipedal_Pro[60][6] = {
    {-7.69121f, 34.1657f, -13.0083f, -26.6936f, -37.1583f, 44.7063f},
    {-22.2912f, 98.8947f, -26.1953f, -74.363f, -122.286f, 116.432f},
    {-9.82062f, -26.9227f, -56.037f, 42.0536f, -57.446f, 90.8396f},
    {-1.25073f, -1.66179f, -12.2444f, 0.26186f, -4.97422f, 16.0727f},
    {1.86587f, 16.4443f, 41.3567f, -13.8793f, 35.3998f, -45.6473f},
    {-0.0120036f, 1.23822f, 2.45042f, 0.0429883f, -1.23274f, -1.60082f},
    {-48.0955f, 7.56688f, 44.1354f, -15.704f, -37.9995f, 1.33148f},
    {-2.76632f, 0.572425f, 4.34901f, -1.15136f, -1.57923f, -4.47867f},
    {32.1994f, -27.4829f, 121.614f, 12.557f, -18.884f, -117.651f},
    {0.479561f, 1.91591f, 5.63873f, -1.95413f, -6.09211f, -1.99342f},
    {1.65203f, -2.91606f, 8.09073f, 2.49341f, -2.79406f, -5.90478f},
    {-0.0227486f, 0.920985f, 1.53882f, -0.864576f, -2.3731f, -0.109468f},
    {-7.69121f, -13.0083f, 34.1657f, 44.7063f, -37.1583f, -26.6936f},
    {-22.2912f, -26.1953f, 98.8947f, 116.432f, -122.286f, -74.363f},
    {9.82062f, 56.037f, 26.9227f, -90.8396f, 57.446f, -42.0536f},
    {1.25073f, 12.2444f, 1.66179f, -16.0727f, 4.97422f, -0.26186f},
    {-48.0955f, 44.1354f, 7.56688f, 1.33148f, -37.9995f, -15.704f},
    {-2.76632f, 4.34901f, 0.572425f, -4.47867f, -1.57923f, -1.15136f},
    {1.86587f, 41.3567f, 16.4443f, -45.6473f, 35.3998f, -13.8793f},
    {-0.0120036f, 2.45042f, 1.23822f, -1.60082f, -1.23274f, 0.0429883f},
    {32.1994f, 121.614f, -27.4829f, -117.651f, -18.884f, 12.557f},
    {0.479561f, 5.63873f, 1.91591f, -1.99342f, -6.09211f, -1.95413f},
    {1.65203f, 8.09073f, -2.91606f, -5.90478f, -2.79406f, 2.49341f},
    {-0.0227486f, 1.53882f, 0.920985f, -0.109468f, -2.3731f, -0.864576f},
    {0.949593f, -4.01736f, 10.1595f, 3.35388f, 0.184543f, -11.9639f},
    {3.27707f, -13.2102f, 24.5198f, 8.9654f, 8.78357f, -33.6662f},
    {-12.8755f, -18.7749f, 28.3428f, 22.9898f, 3.31849f, -27.2187f},
    {-1.70363f, -4.54028f, 5.04443f, 5.99638f, -1.07701f, -3.0655f},
    {3.59757f, 14.1722f, -0.881715f, -2.14798f, -1.24472f, -13.8333f},
    {0.4173f, 0.00588123f, -0.0304268f, 1.69146f, 1.0199f, -1.01673f},
    {6.87829f, -10.7342f, 49.6134f, 11.2583f, -7.36869f, -39.1467f},
    {0.586485f, -0.830777f, 1.9412f, 0.65299f, 0.492003f, -0.103434f},
    {11.0536f, -18.4572f, -8.7737f, 18.7122f, 13.6328f, -14.4018f},
    {1.08845f, -2.30585f, -0.257286f, 2.03758f, 1.77641f, -1.511f},
    {1.626f, -3.12078f, -1.0815f, 2.738f, 2.32552f, -1.16344f},
    {0.402964f, -0.900548f, -0.137939f, 0.779487f, 0.714547f, -0.432484f},
    {0.949593f, 10.1595f, -4.01736f, -11.9639f, 0.184543f, 3.35388f},
    {3.27707f, 24.5198f, -13.2102f, -33.6662f, 8.78357f, 8.9654f},
    {12.8755f, -28.3428f, 18.7749f, 27.2187f, -3.31849f, -22.9898f},
    {1.70363f, -5.04443f, 4.54028f, 3.0655f, 1.07701f, -5.99638f},
    {6.87829f, 49.6134f, -10.7342f, -39.1467f, -7.36869f, 11.2583f},
    {0.586485f, 1.9412f, -0.830777f, -0.103434f, 0.492003f, 0.65299f},
    {3.59757f, -0.881715f, 14.1722f, -13.8333f, -1.24472f, -2.14798f},
    {0.4173f, -0.0304268f, 0.00588123f, -1.01673f, 1.0199f, 1.69146f},
    {11.0536f, -8.7737f, -18.4572f, -14.4018f, 13.6328f, 18.7122f},
    {1.08845f, -0.257286f, -2.30585f, -1.511f, 1.77641f, 2.03758f},
    {1.626f, -1.0815f, -3.12078f, -1.16344f, 2.32552f, 2.738f},
    {0.402964f, -0.137939f, -0.900548f, -0.432484f, 0.714547f, 0.779487f},
    {0.712792f, -0.302708f, -0.302708f, -0.875097f, 1.21945f, -0.875097f},
    {2.09691f, -1.75297f, -1.75297f, -2.2752f, 6.30632f, -2.2752f},
    {-9.85347e-12f, -3.31079f, 3.31079f, 5.64841f, -1.07581e-11f, -5.64841f},
    {-1.5745e-12f, -0.739633f, 0.739633f, 0.994938f, -1.25026e-12f, -0.994938f},
    {2.56228f, 2.0858f, -4.93812f, -4.79944f, 1.5661f, 3.26878f},
    {0.180309f, -0.0790601f, -0.308406f, 0.0586868f, 0.236778f, 0.167071f},
    {2.56228f, -4.93812f, 2.0858f, 3.26878f, 1.5661f, -4.79944f},
    {0.180309f, -0.308406f, -0.0790601f, 0.167071f, 0.236778f, 0.0586868f},
    {12.1293f, -12.3115f, -12.3115f, 11.2491f, 10.2684f, 11.2491f},
    {0.559952f, -0.785471f, -0.785471f, 0.572642f, 0.907656f, 0.572642f},
    {-5.3191f, -1.16875f, -1.16875f, 0.97677f, 1.16785f, 0.97677f},
    {-0.855165f, -0.233108f, -0.233108f, 0.157056f, 0.300975f, 0.157056f}
};

float T0_coef_Bipedal[2][6] = {
    {1.01133f, -2.47908e-14f, -2.47908e-14f, 3.25579e-14f, 4.02904e-14f, 3.25579e-14f},
    {-1.44252f, -6.19769e-15f, -6.19769e-15f, 8.13946e-15f, 1.00726e-14f, 8.13946e-15f}
};

// 4×9 LQR 增益计算（对齐 MCU GetK_Pro_Tripod）
void GetK_Pro_Tripod(double l_l, double l_r, double k[4][9]) {
    for (int n = 0; n < 36; ++n) {
        int row = n / 9;
        int col = n % 9;
        k[row][col] = K_coef_Tripod[n][0] + K_coef_Tripod[n][1] * l_l
                    + K_coef_Tripod[n][2] * l_r + K_coef_Tripod[n][3] * l_l * l_l
                    + K_coef_Tripod[n][4] * l_l * l_r + K_coef_Tripod[n][5] * l_r * l_r;
    }
}

// 4×9 LQR × 9维状态 → [Tp_R, Tp_L, wheel_R, wheel_L]（对齐 MCU CalcLQR_Pro_Tripod）
void CalcLQR_Pro_Tripod(const double k[4][9], const double x[9], double Tp_T[4]) {
    Tp_T[0] = k[0][0]*x[0] + k[0][1]*x[1] + k[0][2]*x[2] + k[0][3]*x[3]
            + k[0][4]*x[4] + k[0][5]*x[5] + k[0][6]*x[6] + k[0][7]*x[7] - k[0][8]*x[8];
    Tp_T[1] = k[1][0]*x[0] + k[1][1]*x[1] + k[1][2]*x[2] + k[1][3]*x[3]
            + k[1][4]*x[4] + k[1][5]*x[5] + k[1][6]*x[6] + k[1][7]*x[7] - k[1][8]*x[8];
    Tp_T[2] = k[2][0]*x[0] + k[2][1]*x[1] + k[2][2]*x[2] + k[2][3]*x[3]
            + k[2][4]*x[4] + k[2][5]*x[5] + k[2][6]*x[6] + k[2][7]*x[7] - k[2][8]*x[8];
    Tp_T[3] = k[3][0]*x[0] + k[3][1]*x[1] + k[3][2]*x[2] + k[3][3]*x[3]
            + k[3][4]*x[4] + k[3][5]*x[5] + k[3][6]*x[6] + k[3][7]*x[7] - k[3][8]*x[8];
}

// Tp/Tt 平衡偏移（对齐 MCU GetT0_Pro_Tripod）
void GetT0_Pro_Tripod(double l_l, double l_r, double T0_eq[2]) {
    T0_eq[0] = T0_coef_Tripod[0][0] + T0_coef_Tripod[0][1] * l_l + T0_coef_Tripod[0][2] * l_r
             + T0_coef_Tripod[0][3] * l_l * l_l + T0_coef_Tripod[0][4] * l_l * l_r
             + T0_coef_Tripod[0][5] * l_r * l_r;
    T0_eq[1] = T0_coef_Tripod[1][0] + T0_coef_Tripod[1][1] * l_l + T0_coef_Tripod[1][2] * l_r
             + T0_coef_Tripod[1][3] * l_l * l_l + T0_coef_Tripod[1][4] * l_l * l_r
             + T0_coef_Tripod[1][5] * l_r * l_r;
}

// Bipedal 版 theta 平衡偏移（对齐 MCU GetTheta_Pro_Bipedal）
void GetTheta_Pro_Bipedal(double l_l, double l_r, double theta_eq[4]) {
    theta_eq[0] = Offset_coef_Bipedal[0][0] + Offset_coef_Bipedal[0][1] * l_l
                + Offset_coef_Bipedal[0][2] * l_r + Offset_coef_Bipedal[0][3] * l_l * l_l
                + Offset_coef_Bipedal[0][4] * l_l * l_r + Offset_coef_Bipedal[0][5] * l_r * l_r;
    theta_eq[1] = Offset_coef_Bipedal[1][0] + Offset_coef_Bipedal[1][1] * l_l
                + Offset_coef_Bipedal[1][2] * l_r + Offset_coef_Bipedal[1][3] * l_l * l_l
                + Offset_coef_Bipedal[1][4] * l_l * l_r + Offset_coef_Bipedal[1][5] * l_r * l_r;
    theta_eq[2] = Offset_coef_Bipedal[2][0] + Offset_coef_Bipedal[2][1] * l_l
                + Offset_coef_Bipedal[2][2] * l_r + Offset_coef_Bipedal[2][3] * l_l * l_l
                + Offset_coef_Bipedal[2][4] * l_l * l_r + Offset_coef_Bipedal[2][5] * l_r * l_r;
    theta_eq[3] = Offset_coef_Bipedal[3][0] + Offset_coef_Bipedal[3][1] * l_l
                + Offset_coef_Bipedal[3][2] * l_r + Offset_coef_Bipedal[3][3] * l_l * l_l
                + Offset_coef_Bipedal[3][4] * l_l * l_r + Offset_coef_Bipedal[3][5] * l_r * l_r;
}

void GetK_Pro_Bipedal(double l_l, double l_r, double k[5][12]) {
    for (int n = 0; n < 60; ++n) {
        const int row = n / 12;
        const int col = n % 12;
        k[row][col] = K_coef_Bipedal_Pro[n][0] + K_coef_Bipedal_Pro[n][1] * l_l
                    + K_coef_Bipedal_Pro[n][2] * l_r + K_coef_Bipedal_Pro[n][3] * l_l * l_l
                    + K_coef_Bipedal_Pro[n][4] * l_l * l_r + K_coef_Bipedal_Pro[n][5] * l_r * l_r;
    }
}

void GetT0_Pro_Bipedal(double l_l, double l_r, double T0_eq[2]) {
    for (int row = 0; row < 2; ++row) {
        T0_eq[row] = T0_coef_Bipedal[row][0] + T0_coef_Bipedal[row][1] * l_l
                   + T0_coef_Bipedal[row][2] * l_r + T0_coef_Bipedal[row][3] * l_l * l_l
                   + T0_coef_Bipedal[row][4] * l_l * l_r + T0_coef_Bipedal[row][5] * l_r * l_r;
    }
}

// 与 MCU CalcLQR_Pro_Tail 完全相同的符号约定和输出顺序：
// [Tp_R, Tp_L, wheel_R, wheel_L, tail].
void CalcLQR_Pro_Bipedal(const double k[5][12], const double x[12], double output[5]) {
    for (int row = 0; row < 5; ++row) {
        output[row] = 0.0;
        for (int col = 0; col < 12; ++col) {
            const double sign = (col == 8 || col == 9) ? -1.0 : 1.0;
            output[row] += sign * k[row][col] * x[col];
        }
    }
}

}  // namespace

double PidState::Compute(double error, bool clear_integral) {
    // 与 MCU PID_calc/PID_calc_Leg 一致：初值是 0，首拍保留误差跃变。
    error_hist[2] = error_hist[1];
    error_hist[1] = error_hist[0];
    error_hist[0] = error;

    if (error_deadband_low < error_hist[0] && error_hist[0] < error_deadband_up) {
        error_hist[0] = 0.0;
    }

    if (error_hist[0] > error_sum_low && error_hist[0] < error_sum_up) {
        iout += ki * error_hist[0];
    } else if (reset_iout_outside_range) {
        iout = 0.0;
    }
    if (clear_integral) {
        iout = 0.0;
    }

    dbuf[2] = dbuf[1];
    dbuf[1] = dbuf[0];
    dbuf[0] = n * dbuf[1] + (1.0 - n) * (error_hist[0] - error_hist[1]);

    iout = Clip(iout, max_iout);
    return Clip(kp * error_hist[0] + iout + kd * dbuf[0], max_output);
}

void PidState::Reset() {
    iout = 0.0;
    dbuf[0] = dbuf[1] = dbuf[2] = 0.0;
    error_hist[0] = error_hist[1] = error_hist[2] = 0.0;
}

BalanceController::BalanceController(BalanceControllerConfig config)
    : config_(config) {
    for (int i = 0; i < 2; ++i) {
        leg_length_pid_[i].kp = config.leg_length_kp[i];
        leg_length_pid_[i].ki = config.leg_length_ki[i];
        leg_length_pid_[i].kd = config.leg_length_kd[i];
        leg_length_pid_[i].n = config.leg_length_n[i];
        leg_length_pid_[i].error_sum_low = -config.leg_length_error_sum_limit[i];
        leg_length_pid_[i].error_sum_up = config.leg_length_error_sum_limit[i];
        leg_length_pid_[i].reset_iout_outside_range = false;
        leg_length_pid_[i].max_output = config.leg_length_max_output;
        leg_length_pid_[i].max_iout = config.leg_length_max_iout;
    }

    leg_coordination_pid_.kp = config.leg_coord_kp;
    leg_coordination_pid_.ki = config.leg_coord_ki;
    leg_coordination_pid_.kd = config.leg_coord_kd;
    leg_coordination_pid_.n = 0.0;
    leg_coordination_pid_.max_output = config.leg_coord_max_output;
    leg_coordination_pid_.max_iout = config.leg_coord_max_iout;

    yaw_velocity_pid_.kp = config.yaw_velocity_kp;
    yaw_velocity_pid_.ki = config.yaw_velocity_ki;
    yaw_velocity_pid_.kd = config.yaw_velocity_kd;
    yaw_velocity_pid_.n = 0.0;
    yaw_velocity_pid_.max_output = config.yaw_velocity_max_output;
    yaw_velocity_pid_.max_iout = config.yaw_velocity_max_iout;

    tail_comp_pid_.kp = config.tail_comp_kp;
    tail_comp_pid_.ki = config.tail_comp_ki;
    tail_comp_pid_.kd = config.tail_comp_kd;
    tail_comp_pid_.n = 0.0;
    tail_comp_pid_.max_output = config.tail_comp_max_output;
    tail_comp_pid_.max_iout = config.tail_comp_max_iout;

    tail_up_pid_.kp = config.proX_tail_up_kp;
    tail_up_pid_.ki = config.proX_tail_up_ki;
    tail_up_pid_.kd = config.proX_tail_up_kd;
    tail_up_pid_.n = 0.1;
    tail_up_pid_.max_output = config.proX_tail_up_max_output;
    tail_up_pid_.max_iout = config.proX_tail_up_max_iout;

    tail_z_pid_.kp = config.proX_tail_z_kp;
    tail_z_pid_.ki = config.proX_tail_z_ki;
    tail_z_pid_.kd = config.proX_tail_z_kd;
    tail_z_pid_.n = 0.1;
    tail_z_pid_.max_output = config.proX_tail_z_max_output;
    tail_z_pid_.max_iout = config.proX_tail_z_max_iout;
}

void BalanceController::SetMode(ControlMode mode) {
    if (mode_ == mode) {
        return;
    }
    // MCU active-mode switch clears x/PID state but keeps the Reference vx LPF
    // continuous. Preserve both raw command and filtered reference here.
    const double saved_speed_command = speed_command_;
    const double saved_speed_reference = speed_ref_;
    const double saved_leg_diff = filtered_leg_diff_;
    mode_ = mode;
    Reset();
    speed_command_ = saved_speed_command;
    speed_ref_ = saved_speed_reference;
    filtered_leg_diff_ = saved_leg_diff;
}

void BalanceController::Reset() {
    speed_command_ = 0.0;
    speed_ref_ = 0.0;
    x_ref_ = 0.0;
    filtered_leg_diff_ = 0.0;
    yaw_rate_command_ = 0.0;
    yaw_ref_ = 0.0;
    leg_length_command_ = 0.0;
    tail_angle_command_ = 0.0;
    tail_angle_ref_ = 0.0;
    reference_initialized_ = false;
    for (auto& pid : leg_length_pid_) pid.Reset();
    leg_coordination_pid_.Reset();
    yaw_velocity_pid_.Reset();
    tail_comp_pid_.Reset();
    tail_up_pid_.Reset();
    tail_z_pid_.Reset();
}

ControlMode BalanceController::mode() const {
    return mode_;
}

std::string BalanceController::ModeName() const {
    switch (mode_) {
        case ControlMode::NoTail:
            return "NoTail";
        case ControlMode::Bipedal:
            return "Bipedal";
        case ControlMode::Tripod:
            return "Tripod";
        case ControlMode::ProXTripod:
            return "ProXTripod";
        default:
            return "Tripod";
    }
}

void BalanceController::SetSpeedRef(double v) {
    speed_command_ = Clip(v, kMaxSpeedReference);
}

double BalanceController::speed_ref() const {
    return speed_command_;
}

void BalanceController::SetYawRateRef(double rate) {
    yaw_rate_command_ = Clip(rate, kMaxYawRateReference);
}

void BalanceController::SetLegLengthRef(double length) {
    leg_length_command_ = std::clamp(length, kMinLegLength, kMaxLegLength);
}

double BalanceController::leg_length_ref() const {
    if (leg_length_command_ > 0.0) return leg_length_command_;
    switch (mode_) {
        case ControlMode::Bipedal: return config_.ref_leg_length_bipedal;
        case ControlMode::Tripod: return config_.ref_leg_length_tripod;
        case ControlMode::ProXTripod: return config_.ref_leg_length_proX;
        default: return config_.ref_leg_length_default;
    }
}

void BalanceController::SetTailAngleRef(double angle) {
    tail_angle_command_ = std::clamp(angle, kMinTailReference, kMaxTailReference);
}

void BalanceController::StopMotion() {
    speed_command_ = 0.0;
    yaw_rate_command_ = 0.0;
}

ControlCommand BalanceController::Compute(const RobotState& state) {
    ControlCommand command;

    if (!reference_initialized_) {
        x_ref_ = state.body.x;
        yaw_ref_ = state.body.yaw;
        reference_initialized_ = true;
    }

    // MCU ChassisReference: 每个 2 ms 周期先对遥控速度做离散一阶低通，
    // 再用滤波结果积分位置参考。alpha 是逐采样系数，不再乘 dt。
    speed_ref_ += kMcuSpeedReferenceAlpha * (speed_command_ - speed_ref_);
    x_ref_ += speed_ref_ * kControlPeriod;
    yaw_ref_ = std::remainder(yaw_ref_ + yaw_rate_command_ * kControlPeriod,
                              2.0 * 3.14159265358979323846);
    tail_angle_ref_ += kTailReferenceAlpha * (tail_angle_command_ - tail_angle_ref_);
    command.x_ref = x_ref_;
    command.speed_ref = speed_ref_;
    command.yaw_ref = yaw_ref_;
    command.yaw_rate_ref = yaw_rate_command_;

    // 确定基础目标腿长
    const double base_ref_l0 = leg_length_ref();

    // ===== 腿长统一协调（roll 角 → 左右腿长差分配） =====
    double ref_l0_left = base_ref_l0;
    double ref_l0_right = base_ref_l0;
    {
        const double raw_leg_diff = state.left_leg.l0 - state.right_leg.l0;
        filtered_leg_diff_ = (1.0 - kLegDiffLpfAlpha) * raw_leg_diff
                           + kLegDiffLpfAlpha * filtered_leg_diff_;
        const double roll_for_leg_coordination =
            (mode_ == ControlMode::Bipedal) ? state.body.smooth_roll : state.body.roll;
        double L_diff = CalcLegLengthDiff(filtered_leg_diff_, roll_for_leg_coordination, 0.0);
        if (mode_ != ControlMode::Bipedal) {
            L_diff = -L_diff;
        }
        double delta_L0 = 0.0;
        CoordinateLegLength(&ref_l0_left, &ref_l0_right, L_diff, delta_L0);
    }
    command.ref_l0[0] = ref_l0_left;
    command.ref_l0[1] = ref_l0_right;

    const LegKinematics legs[2] = {state.left_leg, state.right_leg};
    double wheel_torque[2] = {0.0, 0.0};

    // theta 平衡偏移（根据左右腿长）
    double theta_eq[3] = {0.0, 0.0, 0.0};       // NoTail/Bipedal 用
    double theta_eq_bp[4] = {0.0, 0.0, 0.0, 0.0}; // ProXTripod 用 (Bipedal 版)
    if (mode_ == ControlMode::ProXTripod || mode_ == ControlMode::Bipedal) {
        GetTheta_Pro_Bipedal(legs[0].l0, legs[1].l0, theta_eq_bp);
    } else if (mode_ != ControlMode::Tripod) {
        GetTheta_Pro_NoTail(legs[0].l0, legs[1].l0, theta_eq);
    }

    // roll 角速度阻尼（用于腿部力差分配）
    double roll_vel_limit_f = state.body.roll_rate * kDefaultRollVelLimitFactor;
    if (roll_vel_limit_f > 0.2) roll_vel_limit_f = 0.2;
    if (roll_vel_limit_f < -0.2) roll_vel_limit_f = -0.2;

    // Tripod / ProXTripod 几何平衡求解：给定腿长 → 目标 theta / pitch（对应 MCU ChassisReference）
    double ref_theta_tripod = 0.0;
    double ref_pitch_tripod = 0.0;
    double ref_beta_tripod = 0.0;
    if (mode_ == ControlMode::Tripod || mode_ == ControlMode::ProXTripod) {
        ref_pitch_tripod = state.body.pitch;  // 初始猜测用当前pitch
        SolveThetaBetaBodyCenter(base_ref_l0, ref_theta_tripod, ref_pitch_tripod);
        ref_beta_tripod = kTailBetaInitDeg * 3.14159265358979323846 / 180.0;
        command.ref_theta = ref_theta_tripod;
        command.ref_pitch = ref_pitch_tripod;
        command.ref_beta = ref_beta_tripod;
    } else if (mode_ == ControlMode::Bipedal) {
        // 腿角/pitch 附加参考为零；尾角经与 MCU 相同的低通处理。
        command.ref_theta = 0.0;
        command.ref_pitch = 0.0;
        command.ref_beta = tail_angle_ref_;
    }

    // === ProXTripod：4×9 LQR（机体级，一次性计算）===
    double T0_eq[2] = {0.0, 0.0};
    if (mode_ == ControlMode::ProXTripod) {
        double k_px[4][9] = {{0}};
        double x_px[9];
        double Tp_T_px[4];
        GetK_Pro_Tripod(legs[0].l0, legs[1].l0, k_px);
        GetT0_Pro_Tripod(legs[0].l0, legs[1].l0, T0_eq);

        // 构建 9 维状态向量（对齐 MCU LocomotionController_ProX_Tripod 2024版）
        // x[0] = GetBodyXControllerInput() = clip(x_err, ±0.01) for Tripod/ProX
        //        (the chassis x feedback is clipped to the controller range)
        x_px[0] = Clip(state.body.x - x_ref_, 0.01);
        x_px[1] = state.body.x_dot - speed_ref_;
        // x[2] = GetBodyYawAngleError() = clip(WrapToPi(yaw-0), ±0.01) ≈ 0
        x_px[2] = 0.0;
        x_px[3] = state.body.yaw_rate;
        // MCU: theta - rod_Angle - theta_eq（rod_Angle 不含 theta_eq）
        x_px[4] = state.leg_state[0].theta - ref_theta_tripod - theta_eq_bp[0];
        x_px[5] = state.leg_state[0].theta_dot;
        x_px[6] = state.leg_state[1].theta - ref_theta_tripod - theta_eq_bp[1];
        x_px[7] = state.leg_state[1].theta_dot;
        x_px[8] = state.body.pitch_rate;

        CalcLQR_Pro_Tripod(k_px, x_px, Tp_T_px);
        // Tp_T_px: [0]=Tp_R, [1]=Tp_L, [2]=wheel_R, [3]=wheel_L
        command.vmc_Tp[0] = Tp_T_px[1] + T0_eq[0];  // left Tp
        command.vmc_Tp[1] = Tp_T_px[0] + T0_eq[0];  // right Tp
        wheel_torque[0] = Tp_T_px[3];  // left wheel
        wheel_torque[1] = Tp_T_px[2];  // right wheel

        command.ref_theta = ref_theta_tripod;
        command.ref_pitch = ref_pitch_tripod;
        command.ref_beta = ref_beta_tripod;
    }

    // === 最新 MCU Bipedal：机体级 5x12 LQR，一次计算左右腿、轮和尾巴输出 ===
    double bipedal_tail_torque = 0.0;
    if (mode_ == ControlMode::Bipedal) {
        double k_bp[5][12] = {{0}};
        double x_bp[12] = {
            state.body.x - x_ref_,
            state.body.x_dot - speed_ref_,
            std::remainder(state.body.yaw - yaw_ref_, 2.0 * 3.14159265358979323846),
            state.body.yaw_rate - yaw_rate_command_,
            state.leg_state[0].theta - theta_eq_bp[0],
            state.leg_state[0].theta_dot,
            state.leg_state[1].theta - theta_eq_bp[1],
            state.leg_state[1].theta_dot,
            state.body.pitch,
            state.body.pitch_rate,
            state.beta - tail_angle_ref_,
            state.dbeta,
        };
        double output_bp[5] = {0};
        GetK_Pro_Bipedal(legs[0].l0, legs[1].l0, k_bp);
        GetT0_Pro_Bipedal(legs[0].l0, legs[1].l0, T0_eq);
        CalcLQR_Pro_Bipedal(k_bp, x_bp, output_bp);

        command.vmc_Tp[0] = output_bp[1] + T0_eq[0];
        command.vmc_Tp[1] = output_bp[0] + T0_eq[0];
        wheel_torque[0] = output_bp[3];
        wheel_torque[1] = output_bp[2];
        bipedal_tail_torque = output_bp[4] + T0_eq[1];
    }

    // 第一阶段：LQR + 前馈 + 腿长PID，暂存 F 和 Tp
    for (int i = 0; i < 2; ++i) {
        const auto& leg_state = state.leg_state[i];

        if (mode_ == ControlMode::ProXTripod || mode_ == ControlMode::Bipedal) {
            // 机体级 LQR 已在循环外完成，此处仅处理 F_ff/F_pid。
        } else if (mode_ == ControlMode::Tripod) {
            // === Tripod 模式：3×8 LQR，状态含尾巴 beta/beta_dot ===
            double x_t[8] = {
                leg_state.theta - ref_theta_tripod,
                leg_state.theta_dot,
                0.0,
                //leg_state.x - x_ref_,
                leg_state.x_dot - speed_ref_,
                leg_state.phi - ref_pitch_tripod,
                leg_state.phi_dot,
                state.beta - ref_beta_tripod,
                state.dbeta,
            };
            double k_t[3][8] = {{0}};
            double T_Tp_Tt[3] = {0};
            GetK_Tripod(legs[i].l0, k_t);
            CalcLQR_Tail(k_t, x_t, T_Tp_Tt);
            wheel_torque[i] = T_Tp_Tt[0];
            command.vmc_Tp[i] = T_Tp_Tt[1];
            // T_Tp_Tt[2] 是 per-leg 尾巴力矩，基础Tripod中忽略，用双腿平均
        } else {
            // === NoTail：2×6 LQR + MPC ===
            double x[6] = {
                leg_state.theta - theta_eq[i],
                leg_state.theta_dot,
                leg_state.x - x_ref_,
                leg_state.x_dot - speed_ref_,
                leg_state.phi,
                leg_state.phi_dot,
            };

            double k[2][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            double MPC_k[2][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            GetKNoTail(legs[i].l0, k);
            GetMPCK(legs[i].l0, MPC_k);

            for (int j = 0; j < 6; ++j) {
                k[1][j] += MPC_k[1][j];
            }

            CalcLQRNoTail(k, x, &wheel_torque[i], &command.vmc_Tp[i]);

            {
                double delta_x[6] = {
                    leg_state.delta_theta,
                    leg_state.delta_theta_dot,
                    leg_state.delta_x,
                    leg_state.delta_x_dot,
                    leg_state.delta_phi,
                    leg_state.delta_phi_dot,
                };
                double delta_Tp = 0.0;
                CalcMPC(MPC_k, delta_x, &delta_Tp);
                command.vmc_Tp[i] += delta_Tp;
            }
        }

        // 前馈：重力补偿沿腿方向的分量
        // Tripod 与 NoTail/Bipedal 使用不同的前馈公式，对应 MCU ChassisSendCmd
        double F_ff;
        if (mode_ == ControlMode::Tripod || mode_ == ControlMode::ProXTripod) {
            F_ff = (1.0 - config_.tail_F_ff_ratio) * config_.body_mass * config_.gravity
                   * std::cos(leg_state.theta) * 0.5 * config_.ff_ratio;
        } else {
            F_ff = config_.body_mass * config_.gravity * std::cos(leg_state.theta)
                   * 0.5 * config_.ff_ratio;
        }

        // PID：跟踪目标腿长（ProXTripod 在 tail_z 调整后单独计算）
        if (mode_ != ControlMode::ProXTripod) {
            const double ref_l0_i = (i == 0) ? ref_l0_left : ref_l0_right;
            const double l0_error = ref_l0_i - legs[i].l0;
            const double F_pid = leg_length_pid_[i].Compute(l0_error);
            command.leg_length_error[i] = l0_error;
            command.leg_length_F_pid[i] = F_pid;
            command.leg_length_F_ff[i] = F_ff;
            command.vmc_F[i] = F_ff + F_pid;
        } else {
            command.leg_length_F_ff[i] = F_ff;
            command.vmc_F[i] = F_ff;  // 暂存 F_ff，tail_z 后加 F_pid
        }
    }

    // roll 角速度阻尼：左右腿力差分配（MCU ChassisSendCmd）
    command.vmc_F[0] -= roll_vel_limit_f;
    command.vmc_F[1] += roll_vel_limit_f;

    // ===== ProXTripod：尾巴 VMC（尾落地虚拟力控制） =====
    double tail_torque = 0.0;
    if (mode_ == ControlMode::ProXTripod) {
        const double F_ff_tail = config_.body_mass * config_.gravity * config_.tail_F_ff_ratio;

        // tail_up PID：pitch 误差 → tail_z 补偿偏移（对齐 MCU TAIL_UP）
        const double tail_z_comp = tail_up_pid_.Compute(ref_pitch_tripod - state.body.pitch);

        // 反馈 tail_z = 尾巴末端轮子竖直高度
        const double fdb_tail_z = kTailLength
            * std::sin(state.beta - state.body.pitch + kTailBetaOmniToHand);

        // 参考 tail_z
        double ref_tail_z = kTailLength
            * std::sin(ref_beta_tripod - ref_pitch_tripod + kTailBetaOmniToHand);
        ref_tail_z += tail_z_comp;

        // 约束 tail_z 不超过几何允许的最大值
        const double avg_l_ref = (ref_l0_left + ref_l0_right) / 2.0;
        const double avg_theta_ref = ref_theta_tripod;
        double ref_leglength_z = avg_l_ref * std::cos(avg_theta_ref);
        const double ref_leglength_x = avg_l_ref * std::sin(avg_theta_ref);

        if (ref_tail_z < 0.0) {
            ref_tail_z = 0.0;
            ref_leglength_z -= 2.0 * tail_z_comp;
        } else {
            ref_leglength_z -= tail_z_comp;
        }
        // tail_z PID：竖直高度 → 补偿力
        const double F_compensate = tail_z_pid_.Compute(ref_tail_z - fdb_tail_z);

        // 尾巴力矩 = 力臂 × 总竖直力
        const double cos_term = std::cos(state.beta - state.body.pitch + kTailBetaOmniToHand);
        tail_torque = kTailLength * cos_term * (F_ff_tail + F_compensate);

        command.tail_z_comp = tail_z_comp;
        command.fdb_tail_z = fdb_tail_z;
        command.ref_tail_z = ref_tail_z;
        command.tail_F_ff = F_ff_tail;
        command.tail_F_comp = F_compensate;
        command.tail_cos_term = cos_term;

        // tail_z 补偿同时调整目标腿长（对应 MCU 中 ref.rod_L0 的修正）
        const double adjusted_ref_l0 = std::sqrt(ref_leglength_z * ref_leglength_z
                                                + ref_leglength_x * ref_leglength_x);
        ref_l0_left = adjusted_ref_l0;
        ref_l0_right = adjusted_ref_l0;

        // 重新做 roll 协调
        {
            double Ld0 = state.left_leg.l0 - state.right_leg.l0;
            double L_diff = -CalcLegLengthDiff(Ld0, state.body.roll, 0.0);
            double delta_L0 = 0.0;
            CoordinateLegLength(&ref_l0_left, &ref_l0_right, L_diff, delta_L0);
        }
        command.ref_l0[0] = ref_l0_left;
        command.ref_l0[1] = ref_l0_right;

        // 用调整后的 ref_l0 重新计算 F_pid
        for (int i = 0; i < 2; ++i) {
            const double ref_l0_i = (i == 0) ? ref_l0_left : ref_l0_right;
            const double l0_error = ref_l0_i - legs[i].l0;
            const double F_pid = leg_length_pid_[i].Compute(l0_error);
            command.leg_length_error[i] = l0_error;
            command.leg_length_F_pid[i] = F_pid;
            command.vmc_F[i] = command.leg_length_F_ff[i] + F_pid;
        }

        // 重新施加 roll 阻尼
        command.vmc_F[0] -= roll_vel_limit_f;
        command.vmc_F[1] += roll_vel_limit_f;
    }

    // ===== Tripod：尾巴 LQR（双腿平均状态） =====
    if (mode_ == ControlMode::Tripod) {
        const double avg_l = (legs[0].l0 + legs[1].l0) / 2.0;
        double x_avg[8] = {
            (state.leg_state[0].theta     + state.leg_state[1].theta)     / 2.0 - ref_theta_tripod,
            (state.leg_state[0].theta_dot + state.leg_state[1].theta_dot) / 2.0,
            0.0,  // x error zeroed
            (state.leg_state[0].x_dot     + state.leg_state[1].x_dot)     / 2.0 - speed_ref_,
            (state.leg_state[0].phi       + state.leg_state[1].phi)       / 2.0 - ref_pitch_tripod,
            (state.leg_state[0].phi_dot   + state.leg_state[1].phi_dot)   / 2.0,
            state.beta - ref_beta_tripod,
            state.dbeta,
        };
        double k_avg[3][8] = {{0}};
        double T_Tp_Tt_avg[3] = {0};
        GetK_Tripod(avg_l, k_avg);
        CalcLQR_Tail(k_avg, x_avg, T_Tp_Tt_avg);
        tail_torque = T_Tp_Tt_avg[2];

        // tail_comp PID：pitch 误差 → 尾巴力矩补偿（MCU: ref.pitch - body.pitch）
        double tail_comp_out = tail_comp_pid_.Compute(ref_pitch_tripod - state.body.pitch);
        tail_torque -= tail_comp_out;
    }

    // ===== Bipedal：尾巴输出来自同一个 5x12 机体级 LQR =====
    if (mode_ == ControlMode::Bipedal) {
        tail_torque = bipedal_tail_torque;
    }

    // ===== 两腿 theta 协调（ProXTripod 4×9 LQR 自带独立跟踪，不需要此 PID） =====
    if (mode_ != ControlMode::ProXTripod && mode_ != ControlMode::Bipedal) {
        double theta_diff = state.leg_state[1].theta - state.leg_state[0].theta;  // MCU: error = 设定(0)-反馈(θ_L-θ_R)
        double coord_out = leg_coordination_pid_.Compute(theta_diff);
        command.coord_out = coord_out;
        command.vmc_Tp[0] += coord_out;
        command.vmc_Tp[1] -= coord_out;
    } else {
        command.coord_out = 0.0;
    }

    // 第二阶段：VMC 转换 + 力矩分配
    for (int i = 0; i < 2; ++i) {
        double joint_torque[2] = {0.0, 0.0};
        CalcVmc(command.vmc_F[i], command.vmc_Tp[i], legs[i].jacobian, joint_torque);

        // 仿真采用 A 方案的力矩约束；模型侧仍保留同样的物理限幅。
        wheel_torque[i] = Clip(wheel_torque[i], config_.max_wheel_torque);
        joint_torque[0] = Clip(joint_torque[0], config_.max_joint_torque);
        joint_torque[1] = Clip(joint_torque[1], config_.max_joint_torque);

        command.vmc_torque[i * 2] = joint_torque[0];
        command.vmc_torque[i * 2 + 1] = joint_torque[1];

        if (i == 0) {
            command.actuator_torque[0] = joint_torque[1];
            command.actuator_torque[1] = joint_torque[0];
            command.actuator_torque[4] = wheel_torque[i];
        } else {
            command.actuator_torque[2] = joint_torque[1];
            command.actuator_torque[3] = joint_torque[0];
            command.actuator_torque[5] = wheel_torque[i];
        }
    }

    // ProXTripod/Bipedal 的机体级 LQR 已包含 yaw 反馈，不再叠加独立 PID。
    if (mode_ != ControlMode::ProXTripod && mode_ != ControlMode::Bipedal) {
        const double yaw_target = yaw_rate_command_;
        const double yaw_error = yaw_target - state.body.yaw_rate;  // MCU: 设定-反馈
        const double yaw_pid_out = yaw_velocity_pid_.Compute(yaw_error);
        //const double yaw_pid_out = 0;
        command.yaw_pid_out = yaw_pid_out;
        command.actuator_torque[4] = Clip(command.actuator_torque[4] - yaw_pid_out, config_.max_wheel_torque);
        command.actuator_torque[5] = Clip(command.actuator_torque[5] + yaw_pid_out, config_.max_wheel_torque);
    } else {
        command.yaw_pid_out = 0.0;
    }

    command.actuator_torque[6] = Clip(tail_torque, config_.max_tail_torque);

    return command;
}
