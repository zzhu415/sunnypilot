#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9078833598802911262);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1575403851075201513);
void gnss_H_mod_fun(double *state, double *out_5039313815134449417);
void gnss_f_fun(double *state, double dt, double *out_3385546922502173389);
void gnss_F_fun(double *state, double dt, double *out_5368569691260348819);
void gnss_h_6(double *state, double *sat_pos, double *out_7693232332234970880);
void gnss_H_6(double *state, double *sat_pos, double *out_7362589840417728767);
void gnss_h_20(double *state, double *sat_pos, double *out_7970106252977737554);
void gnss_H_20(double *state, double *sat_pos, double *out_1763274419696124649);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4285468680794536566);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5659209831331948492);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4285468680794536566);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5659209831331948492);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}