# CMSIS-DSP V1.4.5 (CMSIS v4.3.0), matching the existing arm_math.h.
# Compile the used functions with GCC; the original ARMCC archive has
# attributes GNU ld cannot merge. Keep each upstream implementation unchanged.
list(APPEND ROBOT_SOURCES
    components/algorithm/cmsis_dsp/Source/FastMathFunctions/arm_sin_f32.c
    components/algorithm/cmsis_dsp/Source/FastMathFunctions/arm_cos_f32.c
    components/algorithm/cmsis_dsp/Source/MatrixFunctions/arm_mat_add_f32.c
    components/algorithm/cmsis_dsp/Source/MatrixFunctions/arm_mat_sub_f32.c
    components/algorithm/cmsis_dsp/Source/MatrixFunctions/arm_mat_mult_f32.c
    components/algorithm/cmsis_dsp/Source/MatrixFunctions/arm_mat_trans_f32.c
    components/algorithm/cmsis_dsp/Source/MatrixFunctions/arm_mat_init_f32.c
    components/algorithm/cmsis_dsp/Source/MatrixFunctions/arm_mat_inverse_f32.c
    components/algorithm/cmsis_dsp/Source/CommonTables/arm_common_tables.c
)
