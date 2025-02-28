///* mpc_controller.c
// *
// * 一个基于有限时域 LQR 算法实现的 MPC 控制示例，
// * 用于控制四足机器人（或其他机器人）的状态反馈控制。
// *
// * 假设状态向量 x = [pos_x, pos_y, vel_x, vel_y]?，
// * 控制输入 u = [force_x, force_y]?，
// * 离散模型为：
// *    x[k+1] = A*x[k] + B*u[k]
// *
// * 其中：
// *    A = [1, 0, dt, 0;
// *         0, 1, 0, dt;
// *         0, 0, 1,  0;
// *         0, 0, 0,  1]
// *
// *    B = [0.5*dt^2,    0;
// *             0, 0.5*dt^2;
// *           dt,      0;
// *             0,     dt]
// *
// * 代价函数：
// *    J = Σ?=0^(N-1) { (x[k]-x_ref)' Q (x[k]-x_ref) + u[k]' R u[k] } + (x[N]-x_ref)' Qf (x[N]-x_ref)
// *
// * 后退递推求解得到最优反馈增益 K[k]，当前控制输入取：
// *    u = -K[0]*(x - x_ref)
// *
// * 本示例给出完整的 MPC 初始化、参数设置以及求解接口。
// */

//#include <stdio.h>
//#include <string.h>
//#include <math.h>
//#include "mpc.h"

///* ----- 参数定义 ----- */
//#define STATE_DIM 4   // 状态维数
//#define INPUT_DIM 2   // 输入维数
//#define HORIZON   10  // 预测时域

///* ----- MPC 控制器数据结构 ----- */
//typedef struct {
//    float A[STATE_DIM * STATE_DIM];         // 状态转移矩阵
//    float B[STATE_DIM * INPUT_DIM];         // 控制输入矩阵
//    float Q[STATE_DIM * STATE_DIM];         // 状态误差权重
//    float R[INPUT_DIM * INPUT_DIM];         // 控制输入权重
//    float Qf[STATE_DIM * STATE_DIM];        // 终端状态权重
//    int horizon;                            // 预测时域长度
//    float dt;                               // 采样时间
//} MPC_Controller;

///* ----- 矩阵辅助函数 ----- */
///* 矩阵乘法： C = A * B
// * A: (A_rows x A_cols), B: (B_rows x B_cols)，要求 A_cols == B_rows
// * C: (A_rows x B_cols)
// */
//static void mat_mult(const float *A,* int A_rows, int A_cols, 
//                     const float *B, int B_rows, int B_cols, 
//                     float *C)
//{
//    int i, j, k;
//    for (i = 0; i < A_rows; i++) {
//        for (j = 0; j < B_cols; j++) {
//            C[i * B_cols + j] = 0.0f;
//            for (k = 0; k < A_cols; k++) {
//                C[i * B_cols + j] += A[i * A_cols + k] * B[k * B_cols + j];
//            }
//        }
//    }
//}

///* 矩阵加法： C = A + B */
//static void mat_add(const float *A, const float *B, int rows, int cols, float *C)
//{
//    int i;
//    for (i = 0; i < rows * cols; i++) {
//        C[i] = A[i] + B[i];
//    }
//}

///* 矩阵减法： C = A - B */
//static void mat_sub(const float *A, const float *B, int rows, int cols, float *C)
//{
//    int i;
//    for (i = 0; i < rows * cols; i++) {
//        C[i] = A[i] - B[i];
//    }
//}

///* 矩阵转置： At = A? */
//static void mat_transpose(const float *A, int rows, int cols, float *At)
//{
//    int i, j;
//    for (i = 0; i < rows; i++)
//        for (j = 0; j < cols; j++)
//            At[j * rows + i] = A[i * cols + j];
//}

///* 矩阵复制 */
//static void mat_copy(const float *src, int rows, int cols, float *dst)
//{
//    memcpy(dst, src, sizeof(float) * rows * cols);
//}

///* 2x2 矩阵求逆
// * M: 2x2 矩阵，Minv 存储结果
// * 若行列式接近 0，则返回 -1，否则返回 0.
// */
//static int inv_2x2(const float *M, float *Minv)
//{
//    float det = M[0] * M[3] - M[1] * M[2];
//    if (fabs(det) < 1e-6f)
//        return -1;
//    Minv[0] =  M[3] / det;
//    Minv[1] = -M[1] / det;
//    Minv[2] = -M[2] / det;
//    Minv[3] =  M[0] / det;
//    return 0;
//}

///* ----- MPC API ----- */

///* 初始化 MPC 控制器（设置采样时间、默认预测时域及默认模型和权重）
// * 默认模型为双积分模型。
// */
//void mpc_init(MPC_Controller *mpc, float dt)
//{
//    mpc->dt = dt;
//    mpc->horizon = HORIZON;

//    // A = [1  0  dt  0;
//    //      0  1   0 dt;
//    //      0  0   1  0;
//    //      0  0   0  1]
//    mpc->A[0]  = 1;  mpc->A[1]  = 0;  mpc->A[2]  = dt; mpc->A[3]  = 0;
//    mpc->A[4]  = 0;  mpc->A[5]  = 1;  mpc->A[6]  = 0;  mpc->A[7]  = dt;
//    mpc->A[8]  = 0;  mpc->A[9]  = 0;  mpc->A[10] = 1;  mpc->A[11] = 0;
//    mpc->A[12] = 0;  mpc->A[13] = 0;  mpc->A[14] = 0;  mpc->A[15] = 1;

//    // B = [0.5*dt^2    0;
//    //          0   0.5*dt^2;
//    //         dt         0;
//    //          0        dt]
//    float dt2 = dt * dt;
//    mpc->B[0] = 0.5f * dt2; mpc->B[1] = 0;
//    mpc->B[2] = 0;          mpc->B[3] = 0.5f * dt2;
//    mpc->B[4] = dt;         mpc->B[5] = 0;
//    mpc->B[6] = 0;          mpc->B[7] = dt;

//    // 默认 Q：状态权重（4x4 对角阵）
//    memset(mpc->Q, 0, sizeof(mpc->Q));
//    mpc->Q[0]  = 1;  mpc->Q[5]  = 1;  mpc->Q[10] = 1;  mpc->Q[15] = 1;

//    // 默认 R：输入权重（2x2 对角阵）
//    memset(mpc->R, 0, sizeof(mpc->R));
//    mpc->R[0] = 0.1f; mpc->R[3] = 0.1f;

//    // 默认 Qf：终端状态权重（4x4 对角阵）
//    memset(mpc->Qf, 0, sizeof(mpc->Qf));
//    mpc->Qf[0]  = 10; mpc->Qf[5]  = 10; mpc->Qf[10] = 10; mpc->Qf[15] = 10;
//}

///* 设置模型参数 */
//void mpc_set_model(MPC_Controller *mpc, const float *A, const float *B)
//{
//    memcpy(mpc->A, A, sizeof(float) * STATE_DIM * STATE_DIM);
//    memcpy(mpc->B, B, sizeof(float) * STATE_DIM * INPUT_DIM);
//}

///* 设置权重参数 */
//void mpc_set_weights(MPC_Controller *mpc, const float *Q, const float *R, const float *Qf)
//{
//    memcpy(mpc->Q, Q, sizeof(float) * STATE_DIM * STATE_DIM);
//    memcpy(mpc->R, R, sizeof(float) * INPUT_DIM * INPUT_DIM);
//    memcpy(mpc->Qf, Qf, sizeof(float) * STATE_DIM * STATE_DIM);
//}

///* 利用 MPC（基于有限时域 LQR 后退递推）求解控制输入
// * 输入：
// *   mpc    - MPC 控制器参数
// *   x      - 当前状态向量 (STATE_DIM x 1)
// *   x_ref  - 参考状态向量 (STATE_DIM x 1)
// * 输出：
// *   u      - 计算得到的控制输入 (INPUT_DIM x 1)
// */
//void mpc_solve(const MPC_Controller *mpc, const float *x, const float *x_ref, float *u)
//{
//    /* 采用后退递推求解最优反馈增益 K[k] 和矩阵 P[k]
//     * 维度说明：
//     *   P[k] : (STATE_DIM x STATE_DIM)
//     *   K[k] : (INPUT_DIM x STATE_DIM)
//     */
//    float P[HORIZON + 1][STATE_DIM * STATE_DIM];
//    float K[HORIZON][INPUT_DIM * STATE_DIM];

//    int horizon = mpc->horizon;
//    int k, i, j;

//    /* P[horizon] = Qf */
//    mat_copy(mpc->Qf, STATE_DIM, STATE_DIM, P[horizon]);

//    /* 为方便计算，预先计算 A? 和 B? */
//    float A_T[STATE_DIM * STATE_DIM], B_T[INPUT_DIM * STATE_DIM];
//    mat_transpose(mpc->A, STATE_DIM, STATE_DIM, A_T);
//    mat_transpose(mpc->B, STATE_DIM, INPUT_DIM, B_T); // B_T: (INPUT_DIM x STATE_DIM)

//    /* 后退递推计算 */
//    for (k = horizon - 1; k >= 0; k--) {
//        /* 1. 计算 temp2 = B? * P[k+1] * B, 结果为 (2x2) */
//        float temp[STATE_DIM * INPUT_DIM];
//        mat_mult(P[k + 1], STATE_DIM, STATE_DIM, mpc->B, STATE_DIM, INPUT_DIM, temp);
//        float temp2[INPUT_DIM * INPUT_DIM];
//        mat_mult(B_T, INPUT_DIM, STATE_DIM, temp, STATE_DIM, INPUT_DIM, temp2);

//        /* 2. S = R + temp2 */
//        float S[INPUT_DIM * INPUT_DIM];
//        for (i = 0; i < INPUT_DIM * INPUT_DIM; i++) {
//            S[i] = mpc->R[i] + temp2[i];
//        }

//        /* 3. 计算 S 的逆（2x2） */
//        float S_inv[INPUT_DIM * INPUT_DIM];
//        if (inv_2x2(S, S_inv) != 0) {
//            /* 若 S 近似奇异，则用单位矩阵代替 */
//            S_inv[0] = 1; S_inv[1] = 0;
//            S_inv[2] = 0; S_inv[3] = 1;
//        }

//        /* 4. 计算 temp1 = B? * P[k+1] * A, 结果为 (2x4) */
//        float tempA[STATE_DIM * STATE_DIM];
//        mat_mult(P[k + 1], STATE_DIM, STATE_DIM, mpc->A, STATE_DIM, STATE_DIM, tempA);
//        float temp1[INPUT_DIM * STATE_DIM];
//        mat_mult(B_T, INPUT_DIM, STATE_DIM, tempA, STATE_DIM, STATE_DIM, temp1);

//        /* 5. 计算反馈增益 K[k] = S_inv * temp1, (2x4) */
//        mat_mult(S_inv, INPUT_DIM, INPUT_DIM, temp1, INPUT_DIM, STATE_DIM, K[k]);

//        /* 6. 计算 P[k] = Q + A? * P[k+1] * A - (A? * P[k+1] * B) * K[k] */
//        float AP[STATE_DIM * STATE_DIM];
//        mat_mult(P[k + 1], STATE_DIM, STATE_DIM, mpc->A, STATE_DIM, STATE_DIM, AP);
//        float tempA2[STATE_DIM * STATE_DIM];
//        mat_mult(A_T, STATE_DIM, STATE_DIM, AP, STATE_DIM, STATE_DIM, tempA2);

//        float tempP_B[STATE_DIM * INPUT_DIM];
//        mat_mult(P[k + 1], STATE_DIM, STATE_DIM, mpc->B, STATE_DIM, INPUT_DIM, tempP_B);
//        float APB[STATE_DIM * INPUT_DIM];
//        mat_mult(A_T, STATE_DIM, STATE_DIM, tempP_B, STATE_DIM, INPUT_DIM, APB);
//        float APB_K[STATE_DIM * STATE_DIM];
//        mat_mult(APB, STATE_DIM, INPUT_DIM, K[k], INPUT_DIM, STATE_DIM, APB_K);

//        float tempP[STATE_DIM * STATE_DIM];
//        for (i = 0; i < STATE_DIM * STATE_DIM; i++) {
//            tempP[i] = tempA2[i] - APB_K[i];
//        }
//        for (i = 0; i < STATE_DIM * STATE_DIM; i++) {
//            P[k][i] = mpc->Q[i] + tempP[i];
//        }
//    }

//    /* 根据当前状态误差，计算控制输入：
//       u = -K[0] * (x - x_ref)
//       其中 K[0] 是 (2x4) 矩阵，x, x_ref 均为 (4x1) 向量 */
//    float x_error[STATE_DIM];
//    for (i = 0; i < STATE_DIM; i++) {
//        x_error[i] = x[i] - x_ref[i];
//    }
//    float temp_u[INPUT_DIM] = {0};
//    for (i = 0; i < INPUT_DIM; i++) {
//        for (j = 0; j < STATE_DIM; j++) {
//            temp_u[i] += K[0][i * STATE_DIM + j] * x_error[j];
//        }
//    }
//    for (i = 0; i < INPUT_DIM; i++) {
//        u[i] = -temp_u[i];
//    }
//}

/////* ----- 示例 main() ----- */
////#ifdef MPC_TEST
////int main(void)
////{
////    MPC_Controller mpc;
////    // 初始化控制器，采样时间 dt = 0.01s（10ms）
////    mpc_init(&mpc, 0.01f);

////    // 当前状态 x，假设机器人初始位置原点、速度为 0
////    float x[STATE_DIM] = {0, 0, 0, 0};
////    // 参考状态 x_ref，例如目标位置 (1,1)，速度为 0
////    float x_ref[STATE_DIM] = {1, 1, 0, 0};

////    float u[INPUT_DIM] = {0};
////    // 计算 MPC 控制输入
////    mpc_solve(&mpc, x, x_ref, u);

////    printf("计算得到控制输入: u0 = %.4f, u1 = %.4f\n", u[0], u[1]);

////    return 0;
////}
////#endif
