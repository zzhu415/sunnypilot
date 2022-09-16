#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3178670326011605);
void live_err_fun(double *nom_x, double *delta_x, double *out_9058225657074885865);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8800164474790119061);
void live_H_mod_fun(double *state, double *out_4018707832040774139);
void live_f_fun(double *state, double dt, double *out_8776452423874623388);
void live_F_fun(double *state, double dt, double *out_5627873919389290239);
void live_h_4(double *state, double *unused, double *out_5820180861691194180);
void live_H_4(double *state, double *unused, double *out_3143049801553172617);
void live_h_9(double *state, double *unused, double *out_1284905407483105629);
void live_H_9(double *state, double *unused, double *out_8016475336891931529);
void live_h_10(double *state, double *unused, double *out_4947854500495185037);
void live_H_10(double *state, double *unused, double *out_1190450478624676512);
void live_h_12(double *state, double *unused, double *out_2445051630270620842);
void live_H_12(double *state, double *unused, double *out_3238208575489560379);
void live_h_35(double *state, double *unused, double *out_3749151668822029269);
void live_H_35(double *state, double *unused, double *out_6509711858925779993);
void live_h_32(double *state, double *unused, double *out_2166183278143821555);
void live_H_32(double *state, double *unused, double *out_2291012621253870923);
void live_h_13(double *state, double *unused, double *out_8546540552732376870);
void live_H_13(double *state, double *unused, double *out_8463297909824400559);
void live_h_14(double *state, double *unused, double *out_1284905407483105629);
void live_H_14(double *state, double *unused, double *out_8016475336891931529);
void live_h_33(double *state, double *unused, double *out_2559916978055425805);
void live_H_33(double *state, double *unused, double *out_8786475210144914019);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}