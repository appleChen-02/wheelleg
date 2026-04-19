#ifndef CHASSIS_BIPEDAL_EXTRAS_H
#define CHASSIS_BIPEDAL_EXTRAS_H

#include "robot_param.h"
#include "stdbool.h"

extern void GetK_NoTail(float l, float k[2][6], float MPC_k[2][6], bool is_take_off);

extern void GetK_MPC(float l, float MPC_k[2][6]);

extern void GetK_Pro_NoTail(float l_l, float l_r, float k[4][10], float MPC_k[2][6], bool is_take_off);

extern void GetTheta_Pro_NoTail(float l_l, float l_r, float theta_eq[3]);

extern void GetK_Pro_Bipedal(float l_l, float l_r, float k[5][12], bool is_take_off);

extern void GetTheta_Pro_Bipedal(float l_l, float l_r, float theta_eq[4]);

extern void GetT0_Pro_Bipedal(float l_l, float l_r, float T0_eq[2]);

extern void GetK_Bipedal(float l, float k[3][8], bool is_take_off);

extern void GetK_Tripod(float l, float k[3][8], bool is_take_off);

extern void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2]);

extern void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2]);

extern void GetLegForce(float J[2][2], float T1, float T2, float F[2]);

extern void CalcJacobian(float phi1, float phi4, float J[2][2]);

extern void CalcVmc(float F0, float Tp, float J[2][2], float T[2]);

extern void Leg_Forward_Kinematics_Solution(float phi_1, float phi_4, float L0_Phi0[2], float dphi_1, float dphi_4, float dL0_dPhi0[2]);

extern void GetLegForce_test(float phi1, float phi4, float T1, float T2, float F[2]);

extern void CalcVmc_test(float F0, float Tp, float phi1, float phi4, float T[2]);

extern void Leg_Inverse_Kinematics_Solution(float phi0, float l0, float phi1_phi4[2]);

extern inline float CalcLegLengthDiff(float Ld0, float theta0, float theta1);

extern void CoordinateLegLength(float * Ll_ref, float * Lr_ref, float diff, float add);

#endif  // CHASSIS_BIPEDAL_EXTRAS_H
