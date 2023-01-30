#include "lateral_lqr.h"

static float Lqr_A[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static float Lqr_B[2] = {0.0f, 0.0f};
static float Lqr_Q[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static float Lqr_R = 1.0f;

void lateral_ComputeInit(const struct LateralConfig* lateral_config,
                         struct lqr_settings* lqr_instance) {
  float dt = lateral_config->lqr_settings.time_step;
  float Lf = lateral_config->wheel_base_front;
  float Lr = lateral_config->wheel_base_rear;

  float Lqr_K[2];
  int i = 0;
  float vx0 = lqr_instance->speed_limits_min;
  float vx = vx0;

  Lqr_Q[0][0] = lateral_config->lqr_settings.input_weight *
                lateral_config->lqr_settings.state_input_weight_ratio;
  Lqr_Q[1][1] =
      lateral_config->lqr_settings.input_weight *
      lateral_config->lqr_settings.state_input_weight_ratio *
      lateral_config->lqr_settings.heading_error_lateral_error_weight_ratio;
  Lqr_R = lateral_config->lqr_settings.input_weight;

  for (i = 0; vx <= lqr_instance->speed_limits_max;
       i++, vx = vx0 + (float)i * lqr_instance->dvx) {
    Lqr_A[0][1] = vx * dt;
    Lqr_B[1] = vx / (Lf + Lr) * dt;

    if (compGainMatrix(Lqr_A, Lqr_B, Lqr_Q, Lqr_R, Lqr_K, 1e-4f)) {
      lqr_instance->speed_range[i] = vx;
      lqr_instance->lateral_distance_error_gains[i] = Lqr_K[0];
      lqr_instance->heading_angle_error_gains[i] = Lqr_K[1];
      PRT_INFO(" A: %f  B: %f k0: %f k1: %f\n", Lqr_A[0][1], Lqr_B[1], Lqr_K[0], Lqr_K[1]);
    }
  }

  float lateral_gain = LinearSpline_Build(
      lqr_instance->speed_range, lqr_instance->lateral_distance_error_gains, 0,
      lqr_instance->step_count);
  PRT_INFO("val1 = %f  \n", lateral_gain);
}

static float ComputeFeedback(const PursuitInfo* pursuit_info,
                             const struct LateralConfig* lateral_config,
                             struct lqr_settings* lqr_instance) {
  float aiming_point_y = 0;
  float lateral_distance_error_ = 0.0f;
  float heading_angle_error_ = 0.0f;
  float lateral_threshold = pursuit_info->lateral_threshold;
  heading_angle_error_ = atanf(pursuit_info->active_line_b);
  if (pursuit_info->turn_mode == 1) {

    if(pursuit_info->radius > 1200.0f) {
      ;
    } else if (pursuit_info->radius > 600.0f) {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold -= 0.05f;
      } else {
        lateral_threshold += 0.05f;
      }
    } else if (pursuit_info->radius > 300.0f) {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold -= 0.2f;
      } else {
        lateral_threshold += 0.2f;
      }
    } else {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold -= 0.2f;
      } else {
        lateral_threshold += 0.3f;
      }
    }

    lateral_distance_error_ = pursuit_info->active_line_a +
         pursuit_info->active_line_b * aiming_point_y +
         pursuit_info->active_line_c * aiming_point_y * aiming_point_y +
         pursuit_info->active_line_d * aiming_point_y * aiming_point_y * aiming_point_y +
         lateral_threshold + pursuit_info->vehicle_width / 2;
  } else if (pursuit_info->turn_mode == 2) {
    if(pursuit_info->radius > 1200.0f) {
      ;
    } else if (pursuit_info->radius > 600.0f) {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold += 0.05f;
      } else {
        lateral_threshold -= 0.05f;
      }
    } else if (pursuit_info->radius > 300.0f) {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold += 0.2f;
      } else {
        lateral_threshold -= 0.2f;
      }
    } else {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold += 0.3f;
      } else {
        lateral_threshold -= 0.3f;
      }
    }

    lateral_distance_error_ = pursuit_info->active_line_a +
         pursuit_info->active_line_b * aiming_point_y +
         pursuit_info->active_line_c * aiming_point_y * aiming_point_y +
         pursuit_info->active_line_d * aiming_point_y * aiming_point_y * aiming_point_y -
         lateral_threshold - pursuit_info->vehicle_width / 2;
  } else {
    ;
  }
  PRT_INFO("dx is :%f %f %f %f\n", lateral_distance_error_, aiming_point_y, pursuit_info->radius, pursuit_info->vehicle_speed*3.6f);

  float lateral_gain = LinearSpline_Build(
      lqr_instance->speed_range, lqr_instance->lateral_distance_error_gains,
      pursuit_info->vehicle_speed, lqr_instance->step_count);
  float heading_gain = LinearSpline_Build(
      lqr_instance->speed_range, lqr_instance->heading_angle_error_gains,
      pursuit_info->vehicle_speed, lqr_instance->step_count);
  float lateral_distance_error = saturate(lateral_distance_error_,
                                          lateral_config->lateral_error[0],
                                          lateral_config->lateral_error[1]);
  float heading_angle_error = saturate(heading_angle_error_,
                                       lateral_config->heading_error[0],
                                       lateral_config->heading_error[1]);
  PRT_INFO("lat_err = %f ,  head_err = %f\n", lateral_distance_error_,
    heading_angle_error_);
  PRT_INFO(" lataral: %f, heading: %f  %f %f\n", -lateral_gain * lateral_distance_error, -heading_gain * heading_angle_error,
            -lateral_gain * lateral_distance_error*pursuit_info->steer_ratio*180.0f/
                    PI, -heading_gain * heading_angle_error*pursuit_info->steer_ratio*180.0f/
                    PI);
  return -(lateral_gain * lateral_distance_error +
           heading_gain * heading_angle_error);
}

float lateral_ComputeControl(const PursuitInfo* pursuit_info,
                             const struct LateralConfig* lateral_config,
                             struct lqr_settings* lqr_instance) {
  float u_ff = ComputeFeedforward(pursuit_info, lateral_config);
  float u_fb = ComputeFeedback(pursuit_info, lateral_config, lqr_instance);
  float u_hybrid = u_ff + u_fb;
  float u_all = Restrain(u_hybrid, lateral_config, pursuit_info);
  float steer_cmd = pursuit_info->steer_ratio * u_all * 180.0f /
                    PI;  // 计算方向盘转角 并转为度数
  PRT_INFO(" u_ff: %f, u_fb: %f, u_hybrid: %f, u_all: %f\n", u_ff, u_fb, u_hybrid, u_all);
  PRT_INFO(" u_ff1: %f, u_fb1: %f, u_hybrid: %f, u_all: %f\n", u_ff*pursuit_info->steer_ratio*180.0f/
                    PI, u_fb*pursuit_info->steer_ratio*180.0f/
                    PI, u_hybrid*pursuit_info->steer_ratio*180.0f/
                    PI, u_all*pursuit_info->steer_ratio*180.0f/
                    PI);
  float steering_angle = Restrain_SteeringAngle(steer_cmd, lateral_config, pursuit_info);
  PRT_INFO("steer_cmd is :%f %f\n", steering_angle, steer_cmd);
  return steering_angle;
}
