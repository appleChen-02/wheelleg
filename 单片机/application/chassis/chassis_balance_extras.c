#include "chassis_balance_extras.h"

// #include "index_bipedal.h"
// #include "index_notail.h"
// #include "index_tripod.h"
#include "arm_math.h"
#include "math.h"

// 两轮平衡
// 最新替换的k_p系列数组，保留原始精度，添加f后缀保证float类型正确性
// 最新替换的k_p系列数组，保留原始精度，添加f后缀保证float类型正确性
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

float MPC_p1[4] = {-0.005178, -3.313708, -0.005235, -3.313719};
float MPC_p2[4] = {-0.004400, -5.996262, -0.006948, -5.992568};
float MPC_p3[4] = {0.000063, -1.834633, 0.000063, -1.834633};
float MPC_p4[4] = {0.006488, -2.061431, 0.006489, -2.061433};
float MPC_p5[4] = {-0.003766, -0.015098, -0.003755, -0.015105};
float MPC_p6[4] = {-0.019946, -0.014435, -0.019947, -0.014437};
float MPC_p7[4] = {-0.005178, -3.313708, -0.005235, -3.313719};
float MPC_p8[4] = {-0.004400, -5.996262, -0.006948, -5.992568};
float MPC_p9[4] = {0.000063, -1.834633, 0.000063, -1.834633};
float MPC_p10[4] = {0.006488, -2.061431, 0.006489, -2.061433};
float MPC_p11[4] = {-0.003766, -0.015098, -0.003755, -0.015105};
float MPC_p12[4] = {-0.019946, -0.014435, -0.019947, -0.014437};

float k_b11[4] = {44.418f, -100.2109f, 131.1999f, 10.6032f};
float k_b12[4] = {-24.4299f, 27.2668f, 13.3207f, 2.4029f};
float k_b13[4] = {79.0847f, -96.0672f, 42.5666f, 6.7912f};
float k_b14[4] = {18.6796f, -38.5022f, 26.9318f, 9.9682f};
float k_b15[4] = {-82.832f, 2.5677f, 42.2145f, -17.0213f};
float k_b16[4] = {-2.2675f, -2.2551f, 3.7185f, -1.4582f};
float k_b17[4] = {1.4284f, 0.50566f, -1.9807f, 1.2175f};
float k_b18[4] = {-0.69563f, 1.351f, -1.2069f, 0.60653f};
float k_b21[4] = {-580.154f, 408.6517f, -52.2241f, -29.5362f};
float k_b22[4] = {-63.3471f, 42.2601f, -4.049f, -6.1167f};
float k_b23[4] = {-81.2231f, -20.8664f, 64.6867f, -25.3732f};
float k_b24[4] = {62.3406f, -148.3106f, 104.8485f, -30.7302f};
float k_b25[4] = {-214.7374f, 264.4667f, -116.2596f, -24.3446f};
float k_b26[4] = {-19.0556f, 22.2842f, -9.5827f, -0.90623f};
float k_b27[4] = {1.1611f, -4.5398f, 3.3815f, 0.47656f};
float k_b28[4] = {6.5074f, -7.753f, 3.5355f, -0.18686f};
float k_b31[4] = {15.7408f, -34.8648f, 20.2421f, 0.48341f};
float k_b32[4] = {-3.306f, -0.57791f, 2.6322f, 0.17294f};
float k_b33[4] = {24.4009f, -24.4043f, 5.8267f, 1.9132f};
float k_b34[4] = {21.2872f, -22.5999f, 6.1335f, 1.6859f};
float k_b35[4] = {26.7027f, -36.5817f, 18.3176f, -8.2732f};
float k_b36[4] = {1.4026f, -2.0386f, 1.1106f, -0.47783f};
float k_b37[4] = {-0.79011f, 1.1154f, -0.66854f, -2.9099f};
float k_b38[4] = {-0.089228f, 0.23618f, -0.20956f, -1.0339f};

// 最新替换的k_t系列数组，保留原始精度，添加f后缀保证float类型正确性
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

/**
 * @brief 获取K矩阵 双轮平衡
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 */
void GetK_NoTail(float l, float k[2][6], float MPC_k[2][6], bool is_take_off)
{
    float t1 = l;
    float t2 = l * l;
    float t3 = l * l * l;

    k[0][0] = k_p11[0] * t3 + k_p11[1] * t2 + k_p11[2] * t1 + k_p11[3];
    k[0][1] = k_p12[0] * t3 + k_p12[1] * t2 + k_p12[2] * t1 + k_p12[3];
    k[0][2] = k_p13[0] * t3 + k_p13[1] * t2 + k_p13[2] * t1 + k_p13[3];
    k[0][3] = k_p14[0] * t3 + k_p14[1] * t2 + k_p14[2] * t1 + k_p14[3];
    k[0][4] = k_p15[0] * t3 + k_p15[1] * t2 + k_p15[2] * t1 + k_p15[3];
    k[0][5] = k_p16[0] * t3 + k_p16[1] * t2 + k_p16[2] * t1 + k_p16[3];
    k[1][0] = k_p21[0] * t3 + k_p21[1] * t2 + k_p21[2] * t1 + k_p21[3];
    k[1][1] = k_p22[0] * t3 + k_p22[1] * t2 + k_p22[2] * t1 + k_p22[3];
    k[1][2] = k_p23[0] * t3 + k_p23[1] * t2 + k_p23[2] * t1 + k_p23[3];
    k[1][3] = k_p24[0] * t3 + k_p24[1] * t2 + k_p24[2] * t1 + k_p24[3];
    k[1][4] = k_p25[0] * t3 + k_p25[1] * t2 + k_p25[2] * t1 + k_p25[3];
    k[1][5] = k_p26[0] * t3 + k_p26[1] * t2 + k_p26[2] * t1 + k_p26[3];

    MPC_k[0][0] = MPC_p1[0] * expf(MPC_p1[1] * t1) + MPC_p1[2] * expf(MPC_p1[3] * t1);
    MPC_k[0][1] = MPC_p2[0] * expf(MPC_p2[1] * t1) + MPC_p2[2] * expf(MPC_p2[3] * t1);
    MPC_k[0][2] = MPC_p3[0] * expf(MPC_p3[1] * t1) + MPC_p3[2] * expf(MPC_p3[3] * t1);
    MPC_k[0][3] = MPC_p4[0] * expf(MPC_p4[1] * t1) + MPC_p4[2] * expf(MPC_p4[3] * t1);
    MPC_k[0][4] = MPC_p5[0] * expf(MPC_p5[1] * t1) + MPC_p5[2] * expf(MPC_p5[3] * t1);
    MPC_k[0][5] = MPC_p6[0] * expf(MPC_p6[1] * t1) + MPC_p6[2] * expf(MPC_p6[3] * t1);
    MPC_k[1][0] = MPC_p7[0] * expf(MPC_p7[1] * t1) + MPC_p7[2] * expf(MPC_p7[3] * t1);
    MPC_k[1][1] = MPC_p8[0] * expf(MPC_p8[1] * t1) + MPC_p8[2] * expf(MPC_p8[3] * t1);
    MPC_k[1][2] = MPC_p9[0] * expf(MPC_p9[1] * t1) + MPC_p9[2] * expf(MPC_p9[3] * t1);
    MPC_k[1][3] = MPC_p10[0] * expf(MPC_p10[1] * t1) + MPC_p10[2] * expf(MPC_p10[3] * t1);
    MPC_k[1][4] = MPC_p11[0] * expf(MPC_p11[1] * t1) + MPC_p11[2] * expf(MPC_p11[3] * t1);
    MPC_k[1][5] = MPC_p12[0] * expf(MPC_p12[1] * t1) + MPC_p12[2] * expf(MPC_p12[3] * t1);

    k[1][0] = k[1][0] + MPC_k[1][0];
    k[1][1] = k[1][1] + MPC_k[1][1];
    k[1][2] = k[1][2] + MPC_k[1][2];
    k[1][3] = k[1][3] + MPC_k[1][3];
    k[1][4] = k[1][4] + MPC_k[1][4];
    k[1][5] = k[1][5] + MPC_k[1][5];

    if (is_take_off) {
        k[0][0] = 0;
        k[0][1] = 0;
        k[0][2] = 0;
        k[0][3] = 0;
        k[0][4] = 0;
        k[0][5] = 0;
        // k[1][0] = 0;
        // k[1][1] = 0;
        k[1][2] = 0;
        k[1][3] = 0;
        k[1][4] = 0;
        k[1][5] = 0;
    }
}

/**
 * @brief 获取K矩阵 尾辅摆起抓取
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 */
void GetK_Bipedal(float l, float k[3][8], bool is_take_off)
{
    float t1 = l;
    float t2 = l * l;
    float t3 = l * l * l;

    k[0][0] = k_b11[0] * t3 + k_b11[1] * t2 + k_b11[2] * t1 + k_b11[3];
    k[0][1] = k_b12[0] * t3 + k_b12[1] * t2 + k_b12[2] * t1 + k_b12[3];
    k[0][2] = k_b13[0] * t3 + k_b13[1] * t2 + k_b13[2] * t1 + k_b13[3];
    k[0][3] = k_b14[0] * t3 + k_b14[1] * t2 + k_b14[2] * t1 + k_b14[3];
    k[0][4] = k_b15[0] * t3 + k_b15[1] * t2 + k_b15[2] * t1 + k_b15[3];
    k[0][5] = k_b16[0] * t3 + k_b16[1] * t2 + k_b16[2] * t1 + k_b16[3];
    k[0][6] = k_b17[0] * t3 + k_b17[1] * t2 + k_b17[2] * t1 + k_b17[3];
    k[0][7] = k_b18[0] * t3 + k_b18[1] * t2 + k_b18[2] * t1 + k_b18[3];

    k[1][0] = k_b21[0] * t3 + k_b21[1] * t2 + k_b21[2] * t1 + k_b21[3];
    k[1][1] = k_b22[0] * t3 + k_b22[1] * t2 + k_b22[2] * t1 + k_b22[3];
    k[1][2] = k_b23[0] * t3 + k_b23[1] * t2 + k_b23[2] * t1 + k_b23[3];
    k[1][3] = k_b24[0] * t3 + k_b24[1] * t2 + k_b24[2] * t1 + k_b24[3];
    k[1][4] = k_b25[0] * t3 + k_b25[1] * t2 + k_b25[2] * t1 + k_b25[3];
    k[1][5] = k_b26[0] * t3 + k_b26[1] * t2 + k_b26[2] * t1 + k_b26[3];
    k[1][6] = k_b27[0] * t3 + k_b27[1] * t2 + k_b27[2] * t1 + k_b27[3];
    k[1][7] = k_b28[0] * t3 + k_b28[1] * t2 + k_b28[2] * t1 + k_b28[3];

    k[2][0] = k_b31[0] * t3 + k_b31[1] * t2 + k_b31[2] * t1 + k_b31[3];
    k[2][1] = k_b32[0] * t3 + k_b32[1] * t2 + k_b32[2] * t1 + k_b32[3];
    k[2][2] = k_b33[0] * t3 + k_b33[1] * t2 + k_b33[2] * t1 + k_b33[3];
    k[2][3] = k_b34[0] * t3 + k_b34[1] * t2 + k_b34[2] * t1 + k_b34[3];
    k[2][4] = k_b35[0] * t3 + k_b35[1] * t2 + k_b35[2] * t1 + k_b35[3];
    k[2][5] = k_b36[0] * t3 + k_b36[1] * t2 + k_b36[2] * t1 + k_b36[3];
    k[2][6] = k_b37[0] * t3 + k_b37[1] * t2 + k_b37[2] * t1 + k_b37[3];
    k[2][7] = k_b38[0] * t3 + k_b38[1] * t2 + k_b38[2] * t1 + k_b38[3];

    if (is_take_off) {
        k[0][0] = 0;
        k[0][1] = 0;
        k[0][2] = 0;
        k[0][3] = 0;
        k[0][4] = 0;
        k[0][5] = 0;
        k[0][6] = 0;
        k[0][7] = 0;

        // k[1][0] = 0;
        // k[1][1] = 0;
        k[1][2] = 0;
        k[1][3] = 0;
        k[1][4] = 0;
        k[1][5] = 0;
        // k[1][6] = 0;
        // k[1][7] = 0;

        // k[2][0] = 0;
        // k[2][1] = 0;
        k[2][2] = 0;
        k[2][3] = 0;
        k[2][4] = 0;
        k[2][5] = 0;
        // k[2][6] = 0;
        // k[2][7] = 0;
    }
}

/**
 * @brief 获取K矩阵 尾辅助平衡
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 */
void GetK_Tripod(float l, float k[3][8], bool is_take_off)
{
    float t1 = l;
    float t2 = l * l;
    float t3 = l * l * l;

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

    if (is_take_off) {
        k[0][0] = 0;
        k[0][1] = 0;
        k[0][2] = 0;
        k[0][3] = 0;
        k[0][4] = 0;
        k[0][5] = 0;
        k[0][6] = 0;
        k[0][7] = 0;

        // k[1][0] = 0;
        // k[1][1] = 0;
        k[1][2] = 0;
        k[1][3] = 0;
        k[1][4] = 0;
        k[1][5] = 0;
        // k[1][6] = 0;
        // k[1][7] = 0;

        // k[2][0] = 0;
        // k[2][1] = 0;
        k[2][2] = 0;
        k[2][3] = 0;
        k[2][4] = 0;
        k[2][5] = 0;
        // k[2][6] = 0;
        // k[2][7] = 0;
    }
}

// K矩阵拟合系数 K_coef[40][6]
// 第n个K元素: K_n = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
float K_coef[40][6] = {
    {-6.10539f, 30.7168f, -7.68515f, -26.6856f, -32.2938f, 32.3048f},     // K[0][0]
    {-10.4627f, 52.0519f, -6.51423f, -44.1088f, -62.8755f, 46.7275f},     // K[0][1]
    {-10.5387f, -38.0567f, -48.5598f, 60.1187f, -33.7719f, 80.85f},       // K[0][2]
    {-1.07963f, -6.15718f, -10.6054f, 8.21607f, -0.251268f, 16.0847f},    // K[0][3]
    {2.58263f, 22.1358f, 28.9077f, -17.4638f, 38.379f, -36.6599f},        // K[0][4]
    {0.440848f, 0.673213f, 2.64085f, -0.024773f, 1.27f, -3.46384f},       // K[0][5]
    {-28.7828f, 5.63849f, 34.4785f, -10.3229f, -57.4251f, -13.0875f},     // K[0][6]
    {-2.90396f, -0.402926f, 6.51846f, -0.210301f, -2.33491f, -6.51342f},  // K[0][7]
    {47.6829f, -57.295f, 135.915f, 45.9647f, -9.24105f, -157.042f},       // K[0][8]
    {1.37154f, -1.72486f, 4.54988f, 1.42658f, -1.62341f, -4.03976f},      // K[0][9]
    {-6.10539f, -7.68515f, 30.7168f, 32.3048f, -32.2938f, -26.6856f},     // K[1][0]
    {-10.4627f, -6.51423f, 52.0519f, 46.7275f, -62.8755f, -44.1088f},     // K[1][1]
    {10.5387f, 48.5598f, 38.0567f, -80.85f, 33.7719f, -60.1187f},         // K[1][2]
    {1.07963f, 10.6054f, 6.15718f, -16.0847f, 0.251268f, -8.21607f},      // K[1][3]
    {-28.7828f, 34.4785f, 5.63849f, -13.0875f, -57.4251f, -10.3229f},     // K[1][4]
    {-2.90396f, 6.51846f, -0.402926f, -6.51342f, -2.33491f, -0.210301f},  // K[1][5]
    {2.58263f, 28.9077f, 22.1358f, -36.6599f, 38.379f, -17.4638f},        // K[1][6]
    {0.440848f, 2.64085f, 0.673213f, -3.46384f, 1.27f, -0.024773f},       // K[1][7]
    {47.6829f, 135.915f, -57.295f, -157.042f, -9.24105f, 45.9647f},       // K[1][8]
    {1.37154f, 4.54988f, -1.72486f, -4.03976f, -1.62341f, 1.42658f},      // K[1][9]
    {1.95724f, -6.27919f, 9.80895f, 6.04641f, 2.16255f, -13.5824f},       // K[2][0]
    {3.91891f, -12.9107f, 11.5124f, 9.60474f, 13.6841f, -21.8715f},       // K[2][1]
    {-13.8984f, -17.1702f, 31.3254f, 21.9567f, 0.958653f, -36.4762f},     // K[2][2]
    {-1.72652f, -4.28274f, 4.99099f, 5.90735f, 0.655169f, -4.18784f},     // K[2][3]
    {3.75865f, 15.1544f, -4.74354f, -4.92839f, -0.512254f, -7.46677f},    // K[2][4]
    {0.407722f, 0.25413f, -0.386471f, 1.57806f, 0.626039f, -0.597572f},   // K[2][5]
    {6.2133f, -12.4205f, 41.0535f, 13.3628f, -8.18126f, -33.0505f},       // K[2][6]
    {0.664234f, -0.719264f, 2.27191f, 0.525118f, 0.323025f, -1.12969f},   // K[2][7]
    {14.2394f, -25.8158f, -28.0358f, 26.6529f, 29.272f, 5.71537f},        // K[2][8]
    {0.894102f, -1.85228f, -1.14752f, 1.6607f, 1.91074f, -0.152343f},     // K[2][9]
    {1.95724f, 9.80895f, -6.27919f, -13.5824f, 2.16255f, 6.04641f},       // K[3][0]
    {3.91891f, 11.5124f, -12.9107f, -21.8715f, 13.6841f, 9.60474f},       // K[3][1]
    {13.8984f, -31.3254f, 17.1702f, 36.4762f, -0.958653f, -21.9567f},     // K[3][2]
    {1.72652f, -4.99099f, 4.28274f, 4.18784f, -0.655169f, -5.90735f},     // K[3][3]
    {6.2133f, 41.0535f, -12.4205f, -33.0505f, -8.18126f, 13.3628f},       // K[3][4]
    {0.664234f, 2.27191f, -0.719264f, -1.12969f, 0.323025f, 0.525118f},   // K[3][5]
    {3.75865f, -4.74354f, 15.1544f, -7.46677f, -0.512254f, -4.92839f},    // K[3][6]
    {0.407722f, -0.386471f, 0.25413f, -0.597572f, 0.626039f, 1.57806f},   // K[3][7]
    {14.2394f, -28.0358f, -25.8158f, 5.71537f, 29.272f, 26.6529f},        // K[3][8]
    {0.894102f, -1.14752f, -1.85228f, -0.152343f, 1.91074f, 1.6607f}      // K[3][9]
};

float K_coef_Bipedal[60][6] = {
    {-5.43731f, 25.0626f, -9.23877f, -19.28f, -26.7409f, 31.9004f},                // K[0][0]
    {-9.76434f, 44.9065f, -8.50728f, -33.0882f, -57.8114f, 47.6745f},              // K[0][1]
    {-14.6778f, -44.3937f, -56.929f, 72.4388f, -33.5796f, 100.49f},                // K[0][2]
    {-1.24269f, -7.25688f, -11.9503f, 10.432f, 0.739052f, 18.513f},                // K[0][3]
    {3.60312f, 4.64399f, 25.4008f, 11.6366f, 37.6033f, -33.4056f},                 // K[0][4]
    {0.463296f, -0.702875f, 2.56166f, 2.61071f, 0.677897f, -3.15192f},             // K[0][5]
    {-28.2136f, 2.76869f, 35.2062f, -3.99433f, -44.3431f, -25.1142f},              // K[0][6]
    {-2.94781f, -0.41296f, 6.84494f, -0.00650813f, -1.48866f, -7.72401f},          // K[0][7]
    {46.8921f, -42.3667f, 113.387f, 36.6219f, -7.10872f, -128.234f},               // K[0][8]
    {1.37129f, -1.05819f, 4.47188f, 1.12923f, -2.21736f, -3.6242f},                // K[0][9]
    {1.78607f, -3.25211f, 5.93566f, 3.24263f, -2.09445f, -4.77773f},               // K[0][10]
    {0.229192f, -0.0782065f, 1.32232f, 0.0766696f, -1.21979f, -0.648595f},         // K[0][11]
    {-5.43731f, -9.23877f, 25.0626f, 31.9004f, -26.7409f, -19.28f},                // K[1][0]
    {-9.76434f, -8.50728f, 44.9065f, 47.6745f, -57.8114f, -33.0882f},              // K[1][1]
    {14.6778f, 56.929f, 44.3937f, -100.49f, 33.5796f, -72.4388f},                  // K[1][2]
    {1.24269f, 11.9503f, 7.25688f, -18.513f, -0.739052f, -10.432f},                // K[1][3]
    {-28.2136f, 35.2062f, 2.76869f, -25.1142f, -44.3431f, -3.99433f},              // K[1][4]
    {-2.94781f, 6.84494f, -0.41296f, -7.72401f, -1.48866f, -0.00650813f},          // K[1][5]
    {3.60312f, 25.4008f, 4.64399f, -33.4056f, 37.6033f, 11.6366f},                 // K[1][6]
    {0.463296f, 2.56166f, -0.702875f, -3.15192f, 0.677897f, 2.61071f},             // K[1][7]
    {46.8921f, 113.387f, -42.3667f, -128.234f, -7.10872f, 36.6219f},               // K[1][8]
    {1.37129f, 4.47188f, -1.05819f, -3.6242f, -2.21736f, 1.12923f},                // K[1][9]
    {1.78607f, 5.93566f, -3.25211f, -4.77773f, -2.09445f, 3.24263f},               // K[1][10]
    {0.229192f, 1.32232f, -0.0782065f, -0.648595f, -1.21979f, 0.0766696f},         // K[1][11]
    {2.92744f, -10.9398f, 14.7999f, 10.7343f, 5.26398f, -21.1061f},                // K[2][0]
    {6.06457f, -22.3815f, 17.6155f, 16.9704f, 25.1871f, -35.431f},                 // K[2][1]
    {-20.1089f, -18.2044f, 51.0904f, 21.0862f, 7.76999f, -67.3972f},               // K[2][2]
    {-2.16279f, -4.93814f, 7.01848f, 6.86875f, 1.34304f, -6.7781f},                // K[2][3]
    {4.48054f, 14.1165f, -8.89592f, -0.733728f, -0.596822f, -4.46379f},            // K[2][4]
    {0.569472f, -0.225503f, -0.924134f, 2.47418f, 0.836007f, -0.192176f},          // K[2][5]
    {9.23753f, -13.5982f, 49.4109f, 13.674f, -3.03077f, -46.7005f},                // K[2][6]
    {1.06314f, -0.926622f, 2.63637f, 0.558246f, 1.14877f, -2.20408f},              // K[2][7]
    {16.8583f, -21.516f, -28.2713f, 23.7022f, 21.1367f, -1.26738f},                // K[2][8]
    {1.18285f, -2.26159f, -1.10314f, 2.05719f, 2.24316f, -0.743271f},              // K[2][9]
    {1.8707f, -3.52739f, -2.10726f, 3.02985f, 3.42211f, -0.0302044f},              // K[2][10]
    {0.502823f, -1.15294f, -0.451021f, 1.00582f, 1.17711f, -0.232633f},            // K[2][11]
    {2.92744f, 14.7999f, -10.9398f, -21.1061f, 5.26398f, 10.7343f},                // K[3][0]
    {6.06457f, 17.6155f, -22.3815f, -35.431f, 25.1871f, 16.9704f},                 // K[3][1]
    {20.1089f, -51.0904f, 18.2044f, 67.3972f, -7.76999f, -21.0862f},               // K[3][2]
    {2.16279f, -7.01848f, 4.93814f, 6.7781f, -1.34304f, -6.86875f},                // K[3][3]
    {9.23753f, 49.4109f, -13.5982f, -46.7005f, -3.03077f, 13.674f},                // K[3][4]
    {1.06314f, 2.63637f, -0.926622f, -2.20408f, 1.14877f, 0.558246f},              // K[3][5]
    {4.48054f, -8.89592f, 14.1165f, -4.46379f, -0.596822f, -0.733728f},            // K[3][6]
    {0.569472f, -0.924134f, -0.225503f, -0.192176f, 0.836007f, 2.47418f},          // K[3][7]
    {16.8583f, -28.2713f, -21.516f, -1.26738f, 21.1367f, 23.7022f},                // K[3][8]
    {1.18285f, -1.10314f, -2.26159f, -0.743271f, 2.24316f, 2.05719f},              // K[3][9]
    {1.8707f, -2.10726f, -3.52739f, -0.0302044f, 3.42211f, 3.02985f},              // K[3][10]
    {0.502823f, -0.451021f, -1.15294f, -0.232633f, 1.17711f, 1.00582f},            // K[3][11]
    {0.62095f, -0.128775f, -0.128775f, -0.657018f, 0.553925f, -0.657018f},         // K[4][0]
    {1.01244f, -0.757463f, -0.757463f, -1.19626f, 3.09886f, -1.19626f},            // K[4][1]
    {-5.48952e-14f, -1.69616f, 1.69616f, 6.56808f, -1.26338e-12f, -6.56808f},      // K[4][2]
    {-3.72462e-15f, -0.435335f, 0.435335f, 0.842787f, -1.23969e-13f, -0.842787f},  // K[4][3]
    {2.0175f, 0.0516433f, -3.60548f, -0.977617f, 1.1977f, 2.96564f},               // K[4][4]
    {0.213229f, -0.213231f, -0.330295f, 0.232843f, 0.211457f, 0.246838f},          // K[4][5]
    {2.0175f, -3.60548f, 0.0516433f, 2.96564f, 1.1977f, -0.977617f},               // K[4][6]
    {0.213229f, -0.330295f, -0.213231f, 0.246838f, 0.211457f, 0.232843f},          // K[4][7]
    {16.7795f, -14.7141f, -14.7141f, 16.201f, 8.55627f, 16.201f},                  // K[4][8]
    {0.602704f, -0.72222f, -0.72222f, 0.708152f, 0.570719f, 0.708152f},            // K[4][9]
    {-5.09933f, -0.938137f, -0.938137f, 0.934407f, 0.769829f, 0.934407f},          // K[4][10]
    {-1.00182f, -0.185284f, -0.185284f, 0.159422f, 0.195246f, 0.159422f}           // K[4][11]

};

// 平衡点偏移拟合系数 Offset_coef[3][6]
// [0]: theta_l_eq, [1]: theta_r_eq, [2]: theta_b_eq
float Offset_coef[3][6] = {
    {0.535811f, -3.06383f, 1.49966e-15f, 4.94312f, -1.73472e-15f, -2.44381e-15f},  // theta_l_eq
    {0.535811f, 3.97708e-15f, -3.06383f, -7.34786e-15f, 2.24087e-15f, 4.94312f},   // theta_r_eq
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}                                           // theta_b_eq
};

float Offset_coef_Bipedal[4][6] = {
    {0.86928f, -4.63178f, 2.18086e-15f, 7.27593f, -2.714e-15f, -3.4464e-15f},    // theta_l_eq
    {0.86928f, 6.13355e-15f, -4.63178f, -1.08374e-14f, 7.06242e-15f, 7.27593f},  // theta_r_eq
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                                        // theta_b_eq
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}                                         // theta_t_eq
};

float T0_coef_Bipedal[2][6] = {
    {1.01806f, 4.8489e-14f, -2.39746e-15f, -7.53195e-14f, 6.93889e-15f, 5.01295e-16f},  // T_leg_eq
    {-2.03611f, -9.69781e-14f, 4.79491e-15f, 1.50639e-13f, -1.38778e-14f, -1.00259e-15f}  // T_t_eq
};

void GetK_MPC(float l, float MPC_k[2][6])
{
    float t1 = l;
    float t2 = l * l;
    float t3 = l * l * l;

    MPC_k[0][0] = MPC_p1[0] * expf(MPC_p1[1] * t1) + MPC_p1[2] * expf(MPC_p1[3] * t1);
    MPC_k[0][1] = MPC_p2[0] * expf(MPC_p2[1] * t1) + MPC_p2[2] * expf(MPC_p2[3] * t1);
    MPC_k[0][2] = MPC_p3[0] * expf(MPC_p3[1] * t1) + MPC_p3[2] * expf(MPC_p3[3] * t1);
    MPC_k[0][3] = MPC_p4[0] * expf(MPC_p4[1] * t1) + MPC_p4[2] * expf(MPC_p4[3] * t1);
    MPC_k[0][4] = MPC_p5[0] * expf(MPC_p5[1] * t1) + MPC_p5[2] * expf(MPC_p5[3] * t1);
    MPC_k[0][5] = MPC_p6[0] * expf(MPC_p6[1] * t1) + MPC_p6[2] * expf(MPC_p6[3] * t1);
    MPC_k[1][0] = MPC_p7[0] * expf(MPC_p7[1] * t1) + MPC_p7[2] * expf(MPC_p7[3] * t1);
    MPC_k[1][1] = MPC_p8[0] * expf(MPC_p8[1] * t1) + MPC_p8[2] * expf(MPC_p8[3] * t1);
    MPC_k[1][2] = MPC_p9[0] * expf(MPC_p9[1] * t1) + MPC_p9[2] * expf(MPC_p9[3] * t1);
    MPC_k[1][3] = MPC_p10[0] * expf(MPC_p10[1] * t1) + MPC_p10[2] * expf(MPC_p10[3] * t1);
    MPC_k[1][4] = MPC_p11[0] * expf(MPC_p11[1] * t1) + MPC_p11[2] * expf(MPC_p11[3] * t1);
    MPC_k[1][5] = MPC_p12[0] * expf(MPC_p12[1] * t1) + MPC_p12[2] * expf(MPC_p12[3] * t1);
}

/**
 * @brief 获取K矩阵 两轮平衡
 * @param[in]  l_l 左腿长
 * @param[in]  l_r 右腿长
 * @param[out] k K矩阵
 */
void GetK_Pro_NoTail(float l_l, float l_r, float k[4][10], float MPC_k[2][6], bool is_take_off)
{
    for (int n = 0; n < 40; n++) {
        int row = n / 10;
        int col = n % 10;
        k[row][col] = K_coef[n][0] + K_coef[n][1] * l_l + K_coef[n][2] * l_r +
                      K_coef[n][3] * l_l * l_l + K_coef[n][4] * l_l * l_r +
                      K_coef[n][5] * l_r * l_r;
    }

    GetK_MPC(l_l, MPC_k);
    k[1][4] = k[1][4] + MPC_k[1][0];
    k[1][5] = k[1][5] + MPC_k[1][1];
    k[1][0] = k[1][0] + MPC_k[1][2];
    k[1][1] = k[1][1] + MPC_k[1][3];
    k[1][8] = k[1][8] + MPC_k[1][4];
    k[1][9] = k[1][9] + MPC_k[1][5];

    GetK_MPC(l_r, MPC_k);
    k[0][6] = k[0][6] + MPC_k[1][0];
    k[0][7] = k[0][7] + MPC_k[1][1];
    k[0][0] = k[0][0] + MPC_k[1][2];
    k[0][1] = k[0][1] + MPC_k[1][3];
    k[0][8] = k[0][8] + MPC_k[1][4];
    k[0][9] = k[0][9] + MPC_k[1][5];

    if (is_take_off) {
        // 第0、1行：只保留 4~7 列
        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 10; col++) {
                if (col < 4 || col > 7) {
                    k[row][col] = 0.0f;
                }
            }
        }

        // 第2、3行：全部清零
        for (int row = 2; row < 4; row++) {
            for (int col = 0; col < 10; col++) {
                k[row][col] = 0.0f;
            }
        }
    }
}

void GetTheta_Pro_NoTail(float l_l, float l_r, float theta_eq[3])
{
    float theta_l_eq = Offset_coef[0][0] + Offset_coef[0][1] * l_l + Offset_coef[0][2] * l_r +
                       Offset_coef[0][3] * l_l * l_l + Offset_coef[0][4] * l_l * l_r +
                       Offset_coef[0][5] * l_r * l_r;
    float theta_r_eq = Offset_coef[1][0] + Offset_coef[1][1] * l_l + Offset_coef[1][2] * l_r +
                       Offset_coef[1][3] * l_l * l_l + Offset_coef[1][4] * l_l * l_r +
                       Offset_coef[1][5] * l_r * l_r;
    float theta_b_eq = Offset_coef[2][0] + Offset_coef[2][1] * l_l + Offset_coef[2][2] * l_r +
                       Offset_coef[2][3] * l_l * l_l + Offset_coef[2][4] * l_l * l_r +
                       Offset_coef[2][5] * l_r * l_r;
    theta_eq[0] = theta_l_eq;
    theta_eq[1] = theta_r_eq;
    theta_eq[2] = theta_b_eq;
}

/**
 * @brief 获取K矩阵 两轮平衡
 * @param[in]  l_l 左腿长
 * @param[in]  l_r 右腿长
 * @param[out] k K矩阵
 */
void GetK_Pro_Bipedal(float l_l, float l_r, float k[5][12], bool is_take_off)
{
    for (int n = 0; n < 60; n++) {
        int row = n / 12;
        int col = n % 12;
        k[row][col] = K_coef_Bipedal[n][0] + K_coef_Bipedal[n][1] * l_l +
                      K_coef_Bipedal[n][2] * l_r + K_coef_Bipedal[n][3] * l_l * l_l +
                      K_coef_Bipedal[n][4] * l_l * l_r + K_coef_Bipedal[n][5] * l_r * l_r;
    }

    if (is_take_off) {
        // 第0、1行：只保留 4~7 列
        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 12; col++) {
                if (col < 4 || col > 7) {
                    k[row][col] = 0.0f;
                }
            }
        }

        // 第2、3行：全部清零
        for (int row = 2; row < 4; row++) {
            for (int col = 0; col < 12; col++) {
                k[row][col] = 0.0f;
            }
        }

        // 第4行：全部清零
        for (int col = 0; col < 12; col++) {
            if (col < 4 || (col > 7 && col < 10)) k[4][col] = 0.0f;
        }
    }
}

void GetTheta_Pro_Bipedal(float l_l, float l_r, float theta_eq[4])
{
    float theta_l_eq = Offset_coef_Bipedal[0][0] + Offset_coef_Bipedal[0][1] * l_l +
                       Offset_coef_Bipedal[0][2] * l_r + Offset_coef_Bipedal[0][3] * l_l * l_l +
                       Offset_coef_Bipedal[0][4] * l_l * l_r +
                       Offset_coef_Bipedal[0][5] * l_r * l_r;
    float theta_r_eq = Offset_coef_Bipedal[1][0] + Offset_coef_Bipedal[1][1] * l_l +
                       Offset_coef_Bipedal[1][2] * l_r + Offset_coef_Bipedal[1][3] * l_l * l_l +
                       Offset_coef_Bipedal[1][4] * l_l * l_r +
                       Offset_coef_Bipedal[1][5] * l_r * l_r;
    float theta_b_eq = Offset_coef_Bipedal[2][0] + Offset_coef_Bipedal[2][1] * l_l +
                       Offset_coef_Bipedal[2][2] * l_r + Offset_coef_Bipedal[2][3] * l_l * l_l +
                       Offset_coef_Bipedal[2][4] * l_l * l_r +
                       Offset_coef_Bipedal[2][5] * l_r * l_r;
    float theta_t_eq = Offset_coef_Bipedal[3][0] + Offset_coef_Bipedal[3][1] * l_l +
                       Offset_coef_Bipedal[3][2] * l_r + Offset_coef_Bipedal[3][3] * l_l * l_l +
                       Offset_coef_Bipedal[3][4] * l_l * l_r +
                       Offset_coef_Bipedal[3][5] * l_r * l_r;
    theta_eq[0] = theta_l_eq;
    theta_eq[1] = theta_r_eq;
    theta_eq[2] = theta_b_eq;
    theta_eq[3] = theta_t_eq;
}

void GetT0_Pro_Bipedal(float l_l, float l_r, float T0_eq[2])
{
    float Tp_eq = T0_coef_Bipedal[0][0] + T0_coef_Bipedal[0][1] * l_l +
                  T0_coef_Bipedal[0][2] * l_r + T0_coef_Bipedal[0][3] * l_l * l_l +
                  T0_coef_Bipedal[0][4] * l_l * l_r + T0_coef_Bipedal[0][5] * l_r * l_r;
    float Tt_eq = T0_coef_Bipedal[1][0] + T0_coef_Bipedal[1][1] * l_l +
                  T0_coef_Bipedal[1][2] * l_r + T0_coef_Bipedal[1][3] * l_l * l_l +
                  T0_coef_Bipedal[1][4] * l_l * l_r + T0_coef_Bipedal[1][5] * l_r * l_r;

    T0_eq[0] = Tp_eq;
    T0_eq[1] = Tt_eq;
}

/**
 * @brief 通过关节phi1和phi4的值获取L0和Phi0
 * @param[in]  phi1
 * @param[in]  phi4
 * @param[out] L0_Phi0 L0和Phi0
 */
void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2])
{
    float XB = LEG_L1 * arm_cos_f32(phi1);
    float YB = LEG_L1 * arm_sin_f32(phi1);
    float XD = LEG_L5 + LEG_L4 * arm_cos_f32(phi4);
    float YD = LEG_L4 * arm_sin_f32(phi4);

    float lBD_2 = (XD - XB) * (XD - XB) + (YD - YB) * (YD - YB);

    float A0 = 2 * LEG_L2 * (XD - XB);
    float B0 = 2 * LEG_L2 * (YD - YB);
    float C0 = LEG_L2 * LEG_L2 + lBD_2 - LEG_L3 * LEG_L3;
    float phi2 = 2 * atan2f((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    float phi3 = atan2f(YB - YD + LEG_L2 * arm_sin_f32(phi2), XB - XD + LEG_L2 * arm_cos_f32(phi2));

    float XC = LEG_L1 * arm_cos_f32(phi1) + LEG_L2 * arm_cos_f32(phi2);
    float YC = LEG_L1 * arm_sin_f32(phi1) + LEG_L2 * arm_sin_f32(phi2);

    float L0 = sqrtf((XC - LEG_L5 / 2.0f) * (XC - LEG_L5 / 2.0f) + YC * YC);
    float Phi0 = atan2f(YC, (XC - LEG_L5 / 2.0f));

    L0_Phi0[0] = L0;
    L0_Phi0[1] = Phi0;
}

/**
 * @brief 获取dL0和dPhi0
 * @param[in]  J 雅可比矩阵
 * @param[in]  d_phi1 
 * @param[in]  d_phi4 
 */
void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2])
{
    // clang-format off
    float d_l0   = J[0][0] * d_phi1 + J[0][1] * d_phi4;
    float d_phi0 = J[1][0] * d_phi1 + J[1][1] * d_phi4;
    // clang-format on
    dL0_dPhi0[0] = d_l0;
    dL0_dPhi0[1] = d_phi0;
}

/**
 * @brief 获取腿部摆杆的等效力
 * @param[in]  J 雅可比矩阵
 * @param[in]  T1 
 * @param[in]  T2 
 * @param[out] F 0-F0 1-Tp
 */
void GetLegForce(float J[2][2], float T1, float T2, float F[2])
{
    float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    // clang-format off
    float inv_J[4] = {J[1][1] / det, -J[0][1] / det, 
                     -J[1][0] / det,  J[0][0] / det};
    // clang-format on
    //F = (inv_J.') * T
    float F0 = inv_J[0] * T1 + inv_J[2] * T2;
    float Tp = inv_J[1] * T1 + inv_J[3] * T2;

    F[0] = F0;
    F[1] = Tp;
}

/**
 * @brief 计算雅可比矩阵
 * @param phi1 
 * @param phi4 
 * @param J 
 */
void CalcJacobian(float phi1, float phi4, float J[2][2])
{
    // float YD, YB, XD, XB, lBD_2, A0, B0, C0, XC, YC;
    // float phi2, phi3;
    // float L0, phi0;
    // float j11, j12, j21, j22;

    // YD = LEG_L4 * sin(phi4);
    // YB = LEG_L1 * sin(phi1);
    // XD = LEG_L5 + LEG_L4 * cos(phi4);
    // XB = LEG_L1 * cos(phi1);
    // lBD_2 = (XD - XB) * (XD - XB) + (YD - YB) * (YD - YB);
    // A0 = 2 * LEG_L2 * (XD - XB);
    // B0 = 2 * LEG_L2 * (YD - YB);
    // C0 = LEG_L2 * LEG_L2 + lBD_2 - LEG_L3 * LEG_L3;
    // phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    // phi3 = atan2(YB - YD + LEG_L2 * sin(phi2), XB - XD + LEG_L2 * cos(phi2));
    // XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    // YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    // L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    // phi0 = atan2(YC, XC - LEG_L5 / 2);

    // j11 = (LEG_L1 * sin(phi0 - phi3) * sin(phi1 - phi2)) / sin(phi3 - phi2);
    // j12 = (LEG_L4 * sin(phi0 - phi2) * sin(phi3 - phi4)) / sin(phi3 - phi2);
    // j21 = (LEG_L1 * cos(phi0 - phi3) * sin(phi1 - phi2)) / (L0 * sin(phi3 - phi2));
    // j22 = (LEG_L4 * cos(phi0 - phi2) * sin(phi3 - phi4)) / (L0 * sin(phi3 - phi2));
    float XB = LEG_L1 * arm_cos_f32(phi1);
    float YB = LEG_L1 * arm_sin_f32(phi1);
    float XD = LEG_L5 + LEG_L4 * arm_cos_f32(phi4);
    float YD = LEG_L4 * arm_sin_f32(phi4);

    float lBD_2 = (XD - XB) * (XD - XB) + (YD - YB) * (YD - YB);

    float A0 = 2.0f * LEG_L2 * (XD - XB);
    float B0 = 2.0f * LEG_L2 * (YD - YB);
    float C0 = LEG_L2 * LEG_L2 + lBD_2 - LEG_L3 * LEG_L3;
    float phi2 = 2.0f * atan2f((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    float phi3 = atan2f(YB - YD + LEG_L2 * arm_sin_f32(phi2), XB - XD + LEG_L2 * arm_cos_f32(phi2));

    float XC = LEG_L1 * arm_cos_f32(phi1) + LEG_L2 * arm_cos_f32(phi2);
    float YC = LEG_L1 * arm_sin_f32(phi1) + LEG_L2 * arm_sin_f32(phi2);

    float L0 = sqrtf((XC - LEG_L5 / 2.0f) * (XC - LEG_L5 / 2.0f) + YC * YC);
    float phi0 = atan2f(YC, (XC - LEG_L5 / 2.0f));

    //获取VMC雅可比矩阵元素
    float sigma1 = arm_sin_f32(phi3 - phi2);
    float sigma2 = arm_sin_f32(phi3 - phi4);
    float sigma3 = arm_sin_f32(phi1 - phi2);
    float sigma4 = arm_sin_f32(phi0 - phi3);
    float sigma5 = arm_cos_f32(phi0 - phi3);
    float sigma6 = arm_sin_f32(phi0 - phi2);
    float sigma7 = arm_cos_f32(phi0 - phi2);

    float j11 = (LEG_L1 * sigma4 * sigma3) / sigma1;
    float j12 = (LEG_L4 * sigma6 * sigma2) / sigma1;
    float j21 = (LEG_L1 * sigma5 * sigma3) / (L0 * sigma1);
    float j22 = (LEG_L4 * sigma7 * sigma2) / (L0 * sigma1);

    J[0][0] = j11;
    J[0][1] = j12;
    J[1][0] = j21;
    J[1][1] = j22;
}

/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  J 雅可比矩阵
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc(float F0, float Tp, float J[2][2], float T[2])
{
    // clang-format off
    float JT[2][2] = {{J[0][0],J[1][0]}, // 转置矩阵
                      {J[0][1],J[1][1]}};
    float F[2] = {F0, Tp};
    // clang-format on
    float T1 = JT[0][0] * F[0] - JT[0][1] * F[1];
    float T2 = JT[1][0] * F[0] - JT[1][1] * F[1];

    T[0] = T1;
    T[1] = T2;
}

/**
 * @brief 通过关节phi1和phi4的值获取L0和Phi0
 * @param[in]  phi1
 * @param[in]  phi4ff
 * @param[in]  dphi1
 * @param[in]  dphi4
 * @param[out] L0_Phi0 L0和Phi0
 * @param[out] dL0_dPhi0 dL0和dPhi0
 */
void Leg_Forward_Kinematics_Solution(
    float phi_1, float phi_4, float L0_Phi0[2], float dphi_1, float dphi_4, float dL0_dPhi0[2])
{
    float XB = LEG_L1 * cosf(phi_1);
    float YB = LEG_L1 * sinf(phi_1);
    float XD = LEG_L5 + LEG_L4 * cosf(phi_4);
    float YD = LEG_L4 * sinf(phi_4);

    float lBD_2 = (XD - XB) * (XD - XB) + (YD - YB) * (YD - YB);

    float A0 = 2 * LEG_L2 * (XD - XB);
    float B0 = 2 * LEG_L2 * (YD - YB);
    float C0 = LEG_L2 * LEG_L2 + lBD_2 - LEG_L3 * LEG_L3;
    float phi2 = 2 * atan2f((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    float phi3 = atan2f(YB - YD + LEG_L2 * sinf(phi2), XB - XD + LEG_L2 * cosf(phi2));

    float XC = LEG_L1 * cosf(phi_1) + LEG_L2 * cosf(phi2);
    float YC = LEG_L1 * sinf(phi_1) + LEG_L2 * sinf(phi2);

    L0_Phi0[0] = sqrt((XC - LEG_L5 / 2.0f) * (XC - LEG_L5 / 2.0f) + YC * YC);  //L0
    L0_Phi0[1] = atan2f(YC, (XC - LEG_L5 / 2.0f));                             //phi0

    //获取VMC雅可比矩阵元素
    float sigma1 = sinf(phi3 - phi2);
    float sigma2 = sinf(phi3 - phi_4);
    float sigma3 = sinf(phi_1 - phi2);
    float sigma4 = sinf(L0_Phi0[1] - phi3);
    float sigma5 = cosf(L0_Phi0[1] - phi3);
    float sigma6 = sinf(L0_Phi0[1] - phi2);
    float sigma7 = cosf(L0_Phi0[1] - phi2);

    //获取VMC逆解矩阵元素
    float sigma8 = LEG_L4 * sigma2;
    float sigma9 = LEG_L1 * sigma3;

    //获取dL0 ddL0 dphi0
    float sigma10 = LEG_L1 * dphi_1;
    float sigma11 = LEG_L5 / 2.0f - XC;
    float sigma12 = sigma10 * sinf(phi_1 - phi3) + sigma8 * dphi_4;
    float sigma13 = sigma12 / sigma1;
    float sigma14 = sigma10 * cosf(phi_1) + sigma13 * cosf(phi2);
    float sigma15 = sigma10 * sinf(phi_1) + sigma13 * sinf(phi2);
    //float last_dL_0 = dL0_dPhi0[0];
    dL0_dPhi0[0] = (YC * sigma14 + sigma11 * sigma15) / L0_Phi0[0];
    //(Leg->ddL_0)=0.19f*(Leg->dL_0-Leg->last_dL_0)/Observer_BalanceStatus.dt+0.81f*(Leg->ddL_0);//一阶低通滤波
    dL0_dPhi0[1] = -(sigma14 * sigma11 - YC * sigma15) / (YC * YC + sigma11 * sigma11);
}

/**
 * @brief 获取腿部摆杆的等效力
 * @param[in]  J 雅可比矩阵
 * @param[in]  T1 
 * @param[in]  T2 
 * @param[out] F 0-F0 1-Tp
 */
void GetLegForce_test(float phi1, float phi4, float T1, float T2, float F[2])
{
    double b_t30_tmp;
    double t10;
    double t11;
    double t2;
    double t3;
    double t30;
    double t30_tmp;
    double t35;
    double t38;
    double t4;
    double t44;
    double t49;
    double t5;
    double t50;
    double t51;
    double t52;
    double t6;
    double t7;
    double t8;
    double t9;
    /*     This function was generated by the Symbolic Math Toolbox version 9.2.
   */
    /*     25-Dec-2025 13:38:28 */

    t2 = cos(phi1);
    t3 = cos(phi4);
    t4 = sin(phi1);
    t5 = sin(phi4);
    t6 = t2 * t2;
    t7 = t3 * t3;
    t8 = t4 * t4;
    t9 = t5 * t5;
    t10 = t2 * t5;
    t11 = t3 * t4;
    t30_tmp = t2 * t3;
    b_t30_tmp = t4 * t5;
    t30 = 1.0 / (((((t6 + t7) + t8) + t9) - t30_tmp * 2.0) - b_t30_tmp * 2.0);
    t35 = sqrt(
        -(t30 *
          ((((((t6 * 2.1626285410633122E+19 + t7 * 2.1626285410633122E+19) +
               t8 * 2.1626285410633122E+19) +
              t9 * 2.1626285410633122E+19) -
             t30_tmp * 4.3252570821266244E+19) -
            b_t30_tmp * 4.3252570821266244E+19) -
           1.245674039652468E+20) /
          2.251799813685248E+21));
    t38 = t35 * t35;
    t44 = (t10 - t11) * (t30 * t30) * (1.0 / t35);
    t49 = t2 * t44 * 27.65952;
    t50 = t3 * t44 * 27.65952;
    t51 = t4 * t44 * 27.65952;
    t52 = t5 * t44 * 27.65952;
    t30_tmp *= t35;
    t30 = b_t30_tmp * t35;
    t30 = 1.0 / (((((((((((((t10 * 4.3252570821266244E+19 - t11 * 4.3252570821266244E+19) +
                            t30_tmp * 8.8270552696461722E+20) +
                           t30 * 8.8270552696461722E+20) +
                          t11 * t38 * 4.503599627370496E+21) -
                         t10 * t38 * 4.503599627370496E+21) +
                        t11 * t44 * 4.8830422354376737E+19) -
                       t10 * t44 * 4.8830422354376737E+19) +
                      t6 * t35 * t44 * 2.491348079304936E+20) +
                     t7 * t35 * t44 * 2.491348079304936E+20) +
                    t8 * t35 * t44 * 2.491348079304936E+20) +
                   t9 * t35 * t44 * 2.491348079304936E+20) -
                  t30_tmp * t44 * 4.982696158609872E+20) -
                 t30 * t44 * 4.982696158609872E+20);
    double T_11 = t30 * (((t5 * 49.0 + t51) - t52) + t3 * t35 * 500.0) * 8.2112142043220214E+18;
    double T_12 = t30 * (((t4 * 49.0 - t51) + t52) - t2 * t35 * 500.0) * -8.2112142043220214E+18;
    double T_21 = t30 * (((t3 * 49.0 + t49) - t50) - t5 * t35 * 500.0) * 8.2112142043220214E+18;
    double T_22 = t30 * (((t2 * 49.0 - t49) + t50) + t4 * t35 * 500.0) * -8.2112142043220214E+18;

    F[0] = T_11 * T1 + T_12 * T2;
    F[1] = T_21 * T1 + T_22 * T2;
}

/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  phi1 钝角 前
 * @param[in]  phi4 锐角 后
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc_test(float F0, float Tp, float phi1, float phi4, float T[2])
{
    double t2;
    double t22;
    double t25;
    double t26;
    double t28;
    double t3;
    double t4;
    double t45;
    double t48;
    double t5;
    double t57;
    double t6;
    double t61;
    double t7;
    double t8;
    double t9;
    /*     This function was generated by the Symbolic Math Toolbox version 8.3.
   */
    /*     25-Dec-2025 11:58:06 */

    t2 = cos(phi1);
    t3 = cos(phi4);
    t4 = sin(phi1);
    t5 = sin(phi4);
    t6 = t2 * t2;
    t7 = t3 * t3;
    t8 = t4 * t4;
    t9 = t5 * t5;
    t22 = t4 - t5;
    t25 = 1.0 / (t2 - t3);
    t28 = ((t6 + t8) - t7) - t9;
    t61 = t2 * t3;
    t57 = t4 * t5;
    t45 = (((((t6 * 2.1626285410633122E+19 + t7 * 2.1626285410633122E+19) +
              t8 * 2.1626285410633122E+19) +
             t9 * 2.1626285410633122E+19) -
            t61 * 4.3252570821266244E+19) -
           t57 * 4.3252570821266244E+19) -
          1.245674039652468E+20;
    t26 = t25 * t25;
    t6 = 1.0 / (((((t6 + t7) + t8) + t9) - t61 * 2.0) - t57 * 2.0);
    t48 = sqrt(-(t6 * t45 / 2.251799813685248E+21));
    t7 = t2 * t5;
    t8 = t3 * t4;
    t6 = 1.0 / t48 *
         (t6 * (t7 * 4.3252570821266244E+19 - t8 * 4.3252570821266244E+19) / 2.251799813685248E+21 -
          (t7 * 2.0 - t8 * 2.0) * (t6 * t6) * t45 / 2.251799813685248E+21);
    t57 = ((t4 * 0.049 + t5 * 0.049) + t3 * t48 / 2.0) - t2 * t48 / 2.0;
    t7 = t2 * t6;
    t45 = t7 / 4.0;
    t6 *= t3;
    t61 = t6 / 4.0;
    t9 = t7 * 0.548469387755102;
    t6 *= 0.548469387755102;
    t7 = t4 * t48;
    t8 = t22 * t25;

    T[0] = F0 * (((t2 * 0.1075 - t9) + t6) + t7 * 1.096938775510204) -
           Tp * (((t8 * (((t2 * 0.049 - t45) + t61) + t7 / 2.0) * 2.193877551020408 -
                   t4 * t26 * t28 * 0.1075) +
                  t2 * t25 * t57 * 2.193877551020408) +
                 t4 * t22 * t26 * t57 * 2.193877551020408);
    t7 = t5 * t48;
    T[1] = F0 * (((t3 * 0.1075 + t9) - t6) - t7 * 1.096938775510204) -
           Tp * (((t8 * (((t3 * 0.049 + t45) - t61) - t7 / 2.0) * 2.193877551020408 +
                   t5 * t26 * t28 * 0.1075) -
                  t3 * t25 * t57 * 2.193877551020408) -
                 t5 * t22 * t26 * t57 * 2.193877551020408);
}

/**
 * @brief 通过L0和Phi0的值计算关节phi1和phi4
 * @param[in]  phi0
 * @param[in]  l0
 * @param[out] phi1_phi4 phi1和phi4
 * @note 用于位置控制时求逆解
 */
void Leg_Inverse_Kinematics_Solution(float phi_0, float L_0, float phi1_phi4[2])
{
    float XC = LEG_L5 / 2.0f + L_0 * cosf(phi_0);
    float YC = L_0 * sinf(phi_0);

    float A = LEG_L1 + XC;
    float B = LEG_L1 * LEG_L1 - XC * XC;
    float C = LEG_L2 * LEG_L2 - YC * YC;
    float D = LEG_L2 * LEG_L2 + YC * YC;

    float phi1 =
        2.0f * atan2f(
                   2.0f * LEG_L1 * YC +
                       sqrtf(2.0f * LEG_L1 * LEG_L1 * D + 2.0f * XC * XC * C - B * B - C * C),
                   A * A - C);
    if (phi1 > DOUBLE_PI) phi1 -= DOUBLE_PI;
    if (phi1 < 0) phi1 += DOUBLE_PI;

    A = LEG_L3 + LEG_L4;
    B = LEG_L5 - XC;
    C = LEG_L3 - LEG_L4;
    D = XC + LEG_L4 - LEG_L5;

    float E = (A * A - B * B - YC * YC);
    float F = (B * B - C * C + YC * YC);
    float phi4 =
        2.0f * atan2f(2.0f * LEG_L4 * YC - sqrtf(E * F), D * D + YC * YC - LEG_L3 * LEG_L3);
    if (phi4 > M_PI) phi4 -= DOUBLE_PI;
    if (phi4 < -M_PI) phi4 += DOUBLE_PI;

    phi1_phi4[0] = phi1;
    phi1_phi4[1] = phi4;
}

/**
 * @brief 通过当前底盘姿态和目标roll角计算两腿长度期望差值
 * @param[in]  Ld0 (m)当前左右腿长度差值(L0l - L0r)
 * @param[in]  theta0 (rad)当前底盘roll角
 * @param[in]  theta1 (rad)目标roll角
 * @return 两腿长度期望差值(m)(L1l - L1r)
 */
inline float CalcLegLengthDiff(float Ld0, float theta0, float theta1)
{
    return WHEEL_BASE * tanf(theta1) -
           cosf(theta0) / cosf(theta1) * (WHEEL_BASE * tanf(theta0) - Ld0);
}

/**
 * @brief 双腿腿长协调控制，维持腿长目标在范围内，同时尽可能达到两腿目标差值
 * @param[in]  Ll_ref   (m)左腿长度指针
 * @param[in]  Lr_ref   (m)右腿长度指针
 * @param[in]  diff (m)腿长差值
 * @param[in]  add  (m)腿长差值补偿量
 */
void CoordinateLegLength(float * Ll_ref, float * Lr_ref, float diff, float add)
{
    *Ll_ref = *Ll_ref + diff * 0.5f + add;
    *Lr_ref = *Lr_ref - diff * 0.5f - add;

    // float delta = *Ll_ref - *Lr_ref;
    // if (delta > MAX_LEG_LENGTH - MIN_LEG_LENGTH) {
    //     *Ll_ref = MAX_LEG_LENGTH;
    //     *Lr_ref = MIN_LEG_LENGTH;
    //     return;
    // } else if (delta < MIN_LEG_LENGTH - MAX_LEG_LENGTH) {
    //     *Ll_ref = MIN_LEG_LENGTH;
    //     *Lr_ref = MAX_LEG_LENGTH;
    //     return;
    // }

    // 先判断短腿范围，再判断长腿范围
    float * short_leg = *Ll_ref < *Lr_ref ? Ll_ref : Lr_ref;
    float * long_leg = *Ll_ref < *Lr_ref ? Lr_ref : Ll_ref;
    float move = 0;
    move = MIN_LEG_LENGTH - *short_leg;
    if (move > 0) {
        *short_leg += move;
        *long_leg += move;
    }
    if (*long_leg > MAX_LEG_LENGTH) {
        *long_leg = MAX_LEG_LENGTH;
    }
}
