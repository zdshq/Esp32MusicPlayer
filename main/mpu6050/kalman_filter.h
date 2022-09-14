
#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H

typedef struct {
    float x;  /* 状态 */
    float A;  /* 先验状态估计 x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* 过程噪声协方差(预测) */
    float r;  /* 测量噪声协方差 */
    float p;  /* 估计误差协方差 */
    float gain;
} kalman1_state;
 

typedef struct {
    float x[2];     /* 状态:[0]-角度[1]-角度差, 2x1 */
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2];     /* 过程噪声协方差(预测),2x1 [q0,0; 0,q1] */
    float r;        /* 测量噪声协方差 */
    float p[2][2];  /* 估计误差协方差,2x2 [p0 p1; p2 p3] */
    float gain[2];  /* 2x1 */
} kalman2_state; 

kalman2_state state;
 
extern void kalman1_init(kalman1_state *state, float init_x, float init_p);
extern float kalman1_filter(kalman1_state *state, float z_measure);
extern void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
extern float kalman2_filter(kalman2_state *state, float z_measure);
 
#endif  