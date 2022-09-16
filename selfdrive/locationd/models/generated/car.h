#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5911337756844115743);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5485660850319610450);
void car_H_mod_fun(double *state, double *out_8085107026912875082);
void car_f_fun(double *state, double dt, double *out_5894352474097094406);
void car_F_fun(double *state, double dt, double *out_817662128394459820);
void car_h_25(double *state, double *unused, double *out_3753649504837398471);
void car_H_25(double *state, double *unused, double *out_7477841514035474124);
void car_h_24(double *state, double *unused, double *out_5015391030761451135);
void car_H_24(double *state, double *unused, double *out_5305191915029974558);
void car_h_30(double *state, double *unused, double *out_7922259342521802209);
void car_H_30(double *state, double *unused, double *out_2950145183907865926);
void car_h_26(double *state, double *unused, double *out_2920586074860956564);
void car_H_26(double *state, double *unused, double *out_3736338195161417900);
void car_h_27(double *state, double *unused, double *out_4199319549773751608);
void car_H_27(double *state, double *unused, double *out_775381872107441015);
void car_h_29(double *state, double *unused, double *out_2164274428528064253);
void car_H_29(double *state, double *unused, double *out_3460376528222258110);
void car_h_28(double *state, double *unused, double *out_8040266736455178953);
void car_H_28(double *state, double *unused, double *out_5424006799787584361);
void car_h_31(double *state, double *unused, double *out_1661961508788404114);
void car_H_31(double *state, double *unused, double *out_3110130092928066424);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}