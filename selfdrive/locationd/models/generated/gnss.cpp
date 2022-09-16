#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9078833598802911262) {
   out_9078833598802911262[0] = delta_x[0] + nom_x[0];
   out_9078833598802911262[1] = delta_x[1] + nom_x[1];
   out_9078833598802911262[2] = delta_x[2] + nom_x[2];
   out_9078833598802911262[3] = delta_x[3] + nom_x[3];
   out_9078833598802911262[4] = delta_x[4] + nom_x[4];
   out_9078833598802911262[5] = delta_x[5] + nom_x[5];
   out_9078833598802911262[6] = delta_x[6] + nom_x[6];
   out_9078833598802911262[7] = delta_x[7] + nom_x[7];
   out_9078833598802911262[8] = delta_x[8] + nom_x[8];
   out_9078833598802911262[9] = delta_x[9] + nom_x[9];
   out_9078833598802911262[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1575403851075201513) {
   out_1575403851075201513[0] = -nom_x[0] + true_x[0];
   out_1575403851075201513[1] = -nom_x[1] + true_x[1];
   out_1575403851075201513[2] = -nom_x[2] + true_x[2];
   out_1575403851075201513[3] = -nom_x[3] + true_x[3];
   out_1575403851075201513[4] = -nom_x[4] + true_x[4];
   out_1575403851075201513[5] = -nom_x[5] + true_x[5];
   out_1575403851075201513[6] = -nom_x[6] + true_x[6];
   out_1575403851075201513[7] = -nom_x[7] + true_x[7];
   out_1575403851075201513[8] = -nom_x[8] + true_x[8];
   out_1575403851075201513[9] = -nom_x[9] + true_x[9];
   out_1575403851075201513[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_5039313815134449417) {
   out_5039313815134449417[0] = 1.0;
   out_5039313815134449417[1] = 0;
   out_5039313815134449417[2] = 0;
   out_5039313815134449417[3] = 0;
   out_5039313815134449417[4] = 0;
   out_5039313815134449417[5] = 0;
   out_5039313815134449417[6] = 0;
   out_5039313815134449417[7] = 0;
   out_5039313815134449417[8] = 0;
   out_5039313815134449417[9] = 0;
   out_5039313815134449417[10] = 0;
   out_5039313815134449417[11] = 0;
   out_5039313815134449417[12] = 1.0;
   out_5039313815134449417[13] = 0;
   out_5039313815134449417[14] = 0;
   out_5039313815134449417[15] = 0;
   out_5039313815134449417[16] = 0;
   out_5039313815134449417[17] = 0;
   out_5039313815134449417[18] = 0;
   out_5039313815134449417[19] = 0;
   out_5039313815134449417[20] = 0;
   out_5039313815134449417[21] = 0;
   out_5039313815134449417[22] = 0;
   out_5039313815134449417[23] = 0;
   out_5039313815134449417[24] = 1.0;
   out_5039313815134449417[25] = 0;
   out_5039313815134449417[26] = 0;
   out_5039313815134449417[27] = 0;
   out_5039313815134449417[28] = 0;
   out_5039313815134449417[29] = 0;
   out_5039313815134449417[30] = 0;
   out_5039313815134449417[31] = 0;
   out_5039313815134449417[32] = 0;
   out_5039313815134449417[33] = 0;
   out_5039313815134449417[34] = 0;
   out_5039313815134449417[35] = 0;
   out_5039313815134449417[36] = 1.0;
   out_5039313815134449417[37] = 0;
   out_5039313815134449417[38] = 0;
   out_5039313815134449417[39] = 0;
   out_5039313815134449417[40] = 0;
   out_5039313815134449417[41] = 0;
   out_5039313815134449417[42] = 0;
   out_5039313815134449417[43] = 0;
   out_5039313815134449417[44] = 0;
   out_5039313815134449417[45] = 0;
   out_5039313815134449417[46] = 0;
   out_5039313815134449417[47] = 0;
   out_5039313815134449417[48] = 1.0;
   out_5039313815134449417[49] = 0;
   out_5039313815134449417[50] = 0;
   out_5039313815134449417[51] = 0;
   out_5039313815134449417[52] = 0;
   out_5039313815134449417[53] = 0;
   out_5039313815134449417[54] = 0;
   out_5039313815134449417[55] = 0;
   out_5039313815134449417[56] = 0;
   out_5039313815134449417[57] = 0;
   out_5039313815134449417[58] = 0;
   out_5039313815134449417[59] = 0;
   out_5039313815134449417[60] = 1.0;
   out_5039313815134449417[61] = 0;
   out_5039313815134449417[62] = 0;
   out_5039313815134449417[63] = 0;
   out_5039313815134449417[64] = 0;
   out_5039313815134449417[65] = 0;
   out_5039313815134449417[66] = 0;
   out_5039313815134449417[67] = 0;
   out_5039313815134449417[68] = 0;
   out_5039313815134449417[69] = 0;
   out_5039313815134449417[70] = 0;
   out_5039313815134449417[71] = 0;
   out_5039313815134449417[72] = 1.0;
   out_5039313815134449417[73] = 0;
   out_5039313815134449417[74] = 0;
   out_5039313815134449417[75] = 0;
   out_5039313815134449417[76] = 0;
   out_5039313815134449417[77] = 0;
   out_5039313815134449417[78] = 0;
   out_5039313815134449417[79] = 0;
   out_5039313815134449417[80] = 0;
   out_5039313815134449417[81] = 0;
   out_5039313815134449417[82] = 0;
   out_5039313815134449417[83] = 0;
   out_5039313815134449417[84] = 1.0;
   out_5039313815134449417[85] = 0;
   out_5039313815134449417[86] = 0;
   out_5039313815134449417[87] = 0;
   out_5039313815134449417[88] = 0;
   out_5039313815134449417[89] = 0;
   out_5039313815134449417[90] = 0;
   out_5039313815134449417[91] = 0;
   out_5039313815134449417[92] = 0;
   out_5039313815134449417[93] = 0;
   out_5039313815134449417[94] = 0;
   out_5039313815134449417[95] = 0;
   out_5039313815134449417[96] = 1.0;
   out_5039313815134449417[97] = 0;
   out_5039313815134449417[98] = 0;
   out_5039313815134449417[99] = 0;
   out_5039313815134449417[100] = 0;
   out_5039313815134449417[101] = 0;
   out_5039313815134449417[102] = 0;
   out_5039313815134449417[103] = 0;
   out_5039313815134449417[104] = 0;
   out_5039313815134449417[105] = 0;
   out_5039313815134449417[106] = 0;
   out_5039313815134449417[107] = 0;
   out_5039313815134449417[108] = 1.0;
   out_5039313815134449417[109] = 0;
   out_5039313815134449417[110] = 0;
   out_5039313815134449417[111] = 0;
   out_5039313815134449417[112] = 0;
   out_5039313815134449417[113] = 0;
   out_5039313815134449417[114] = 0;
   out_5039313815134449417[115] = 0;
   out_5039313815134449417[116] = 0;
   out_5039313815134449417[117] = 0;
   out_5039313815134449417[118] = 0;
   out_5039313815134449417[119] = 0;
   out_5039313815134449417[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3385546922502173389) {
   out_3385546922502173389[0] = dt*state[3] + state[0];
   out_3385546922502173389[1] = dt*state[4] + state[1];
   out_3385546922502173389[2] = dt*state[5] + state[2];
   out_3385546922502173389[3] = state[3];
   out_3385546922502173389[4] = state[4];
   out_3385546922502173389[5] = state[5];
   out_3385546922502173389[6] = dt*state[7] + state[6];
   out_3385546922502173389[7] = dt*state[8] + state[7];
   out_3385546922502173389[8] = state[8];
   out_3385546922502173389[9] = state[9];
   out_3385546922502173389[10] = state[10];
}
void F_fun(double *state, double dt, double *out_5368569691260348819) {
   out_5368569691260348819[0] = 1;
   out_5368569691260348819[1] = 0;
   out_5368569691260348819[2] = 0;
   out_5368569691260348819[3] = dt;
   out_5368569691260348819[4] = 0;
   out_5368569691260348819[5] = 0;
   out_5368569691260348819[6] = 0;
   out_5368569691260348819[7] = 0;
   out_5368569691260348819[8] = 0;
   out_5368569691260348819[9] = 0;
   out_5368569691260348819[10] = 0;
   out_5368569691260348819[11] = 0;
   out_5368569691260348819[12] = 1;
   out_5368569691260348819[13] = 0;
   out_5368569691260348819[14] = 0;
   out_5368569691260348819[15] = dt;
   out_5368569691260348819[16] = 0;
   out_5368569691260348819[17] = 0;
   out_5368569691260348819[18] = 0;
   out_5368569691260348819[19] = 0;
   out_5368569691260348819[20] = 0;
   out_5368569691260348819[21] = 0;
   out_5368569691260348819[22] = 0;
   out_5368569691260348819[23] = 0;
   out_5368569691260348819[24] = 1;
   out_5368569691260348819[25] = 0;
   out_5368569691260348819[26] = 0;
   out_5368569691260348819[27] = dt;
   out_5368569691260348819[28] = 0;
   out_5368569691260348819[29] = 0;
   out_5368569691260348819[30] = 0;
   out_5368569691260348819[31] = 0;
   out_5368569691260348819[32] = 0;
   out_5368569691260348819[33] = 0;
   out_5368569691260348819[34] = 0;
   out_5368569691260348819[35] = 0;
   out_5368569691260348819[36] = 1;
   out_5368569691260348819[37] = 0;
   out_5368569691260348819[38] = 0;
   out_5368569691260348819[39] = 0;
   out_5368569691260348819[40] = 0;
   out_5368569691260348819[41] = 0;
   out_5368569691260348819[42] = 0;
   out_5368569691260348819[43] = 0;
   out_5368569691260348819[44] = 0;
   out_5368569691260348819[45] = 0;
   out_5368569691260348819[46] = 0;
   out_5368569691260348819[47] = 0;
   out_5368569691260348819[48] = 1;
   out_5368569691260348819[49] = 0;
   out_5368569691260348819[50] = 0;
   out_5368569691260348819[51] = 0;
   out_5368569691260348819[52] = 0;
   out_5368569691260348819[53] = 0;
   out_5368569691260348819[54] = 0;
   out_5368569691260348819[55] = 0;
   out_5368569691260348819[56] = 0;
   out_5368569691260348819[57] = 0;
   out_5368569691260348819[58] = 0;
   out_5368569691260348819[59] = 0;
   out_5368569691260348819[60] = 1;
   out_5368569691260348819[61] = 0;
   out_5368569691260348819[62] = 0;
   out_5368569691260348819[63] = 0;
   out_5368569691260348819[64] = 0;
   out_5368569691260348819[65] = 0;
   out_5368569691260348819[66] = 0;
   out_5368569691260348819[67] = 0;
   out_5368569691260348819[68] = 0;
   out_5368569691260348819[69] = 0;
   out_5368569691260348819[70] = 0;
   out_5368569691260348819[71] = 0;
   out_5368569691260348819[72] = 1;
   out_5368569691260348819[73] = dt;
   out_5368569691260348819[74] = 0;
   out_5368569691260348819[75] = 0;
   out_5368569691260348819[76] = 0;
   out_5368569691260348819[77] = 0;
   out_5368569691260348819[78] = 0;
   out_5368569691260348819[79] = 0;
   out_5368569691260348819[80] = 0;
   out_5368569691260348819[81] = 0;
   out_5368569691260348819[82] = 0;
   out_5368569691260348819[83] = 0;
   out_5368569691260348819[84] = 1;
   out_5368569691260348819[85] = dt;
   out_5368569691260348819[86] = 0;
   out_5368569691260348819[87] = 0;
   out_5368569691260348819[88] = 0;
   out_5368569691260348819[89] = 0;
   out_5368569691260348819[90] = 0;
   out_5368569691260348819[91] = 0;
   out_5368569691260348819[92] = 0;
   out_5368569691260348819[93] = 0;
   out_5368569691260348819[94] = 0;
   out_5368569691260348819[95] = 0;
   out_5368569691260348819[96] = 1;
   out_5368569691260348819[97] = 0;
   out_5368569691260348819[98] = 0;
   out_5368569691260348819[99] = 0;
   out_5368569691260348819[100] = 0;
   out_5368569691260348819[101] = 0;
   out_5368569691260348819[102] = 0;
   out_5368569691260348819[103] = 0;
   out_5368569691260348819[104] = 0;
   out_5368569691260348819[105] = 0;
   out_5368569691260348819[106] = 0;
   out_5368569691260348819[107] = 0;
   out_5368569691260348819[108] = 1;
   out_5368569691260348819[109] = 0;
   out_5368569691260348819[110] = 0;
   out_5368569691260348819[111] = 0;
   out_5368569691260348819[112] = 0;
   out_5368569691260348819[113] = 0;
   out_5368569691260348819[114] = 0;
   out_5368569691260348819[115] = 0;
   out_5368569691260348819[116] = 0;
   out_5368569691260348819[117] = 0;
   out_5368569691260348819[118] = 0;
   out_5368569691260348819[119] = 0;
   out_5368569691260348819[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_7693232332234970880) {
   out_7693232332234970880[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_7362589840417728767) {
   out_7362589840417728767[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7362589840417728767[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7362589840417728767[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7362589840417728767[3] = 0;
   out_7362589840417728767[4] = 0;
   out_7362589840417728767[5] = 0;
   out_7362589840417728767[6] = 1;
   out_7362589840417728767[7] = 0;
   out_7362589840417728767[8] = 0;
   out_7362589840417728767[9] = 0;
   out_7362589840417728767[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7970106252977737554) {
   out_7970106252977737554[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1763274419696124649) {
   out_1763274419696124649[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1763274419696124649[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1763274419696124649[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1763274419696124649[3] = 0;
   out_1763274419696124649[4] = 0;
   out_1763274419696124649[5] = 0;
   out_1763274419696124649[6] = 1;
   out_1763274419696124649[7] = 0;
   out_1763274419696124649[8] = 0;
   out_1763274419696124649[9] = 1;
   out_1763274419696124649[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_4285468680794536566) {
   out_4285468680794536566[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_5659209831331948492) {
   out_5659209831331948492[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[6] = 0;
   out_5659209831331948492[7] = 1;
   out_5659209831331948492[8] = 0;
   out_5659209831331948492[9] = 0;
   out_5659209831331948492[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_4285468680794536566) {
   out_4285468680794536566[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_5659209831331948492) {
   out_5659209831331948492[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5659209831331948492[6] = 0;
   out_5659209831331948492[7] = 1;
   out_5659209831331948492[8] = 0;
   out_5659209831331948492[9] = 0;
   out_5659209831331948492[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9078833598802911262) {
  err_fun(nom_x, delta_x, out_9078833598802911262);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1575403851075201513) {
  inv_err_fun(nom_x, true_x, out_1575403851075201513);
}
void gnss_H_mod_fun(double *state, double *out_5039313815134449417) {
  H_mod_fun(state, out_5039313815134449417);
}
void gnss_f_fun(double *state, double dt, double *out_3385546922502173389) {
  f_fun(state,  dt, out_3385546922502173389);
}
void gnss_F_fun(double *state, double dt, double *out_5368569691260348819) {
  F_fun(state,  dt, out_5368569691260348819);
}
void gnss_h_6(double *state, double *sat_pos, double *out_7693232332234970880) {
  h_6(state, sat_pos, out_7693232332234970880);
}
void gnss_H_6(double *state, double *sat_pos, double *out_7362589840417728767) {
  H_6(state, sat_pos, out_7362589840417728767);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7970106252977737554) {
  h_20(state, sat_pos, out_7970106252977737554);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1763274419696124649) {
  H_20(state, sat_pos, out_1763274419696124649);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4285468680794536566) {
  h_7(state, sat_pos_vel, out_4285468680794536566);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5659209831331948492) {
  H_7(state, sat_pos_vel, out_5659209831331948492);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4285468680794536566) {
  h_21(state, sat_pos_vel, out_4285468680794536566);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5659209831331948492) {
  H_21(state, sat_pos_vel, out_5659209831331948492);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
