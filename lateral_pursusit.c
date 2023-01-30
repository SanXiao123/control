#include "lateral_pursuit.h"

static float ComputeFeedback_pursuit(const PursuitInfo* pursuit_info,
                                    const struct LateralConfig* lateral_config) {
  float wheel_base =
      lateral_config->wheel_base_front + lateral_config->wheel_base_rear;
  float aiming_point_y = fmaxf(pursuit_info->vehicle_speed * 3.0f,
                                45.0f);  // 根据3s的移动距离或50米来选择预瞄点
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
      aiming_point_y = aiming_point_y * pursuit_info->radius / 2500.0f;
    } else if (pursuit_info->radius > 300.0f) {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold -= 0.2f;
      } else {
        lateral_threshold += 0.2f;
      }
      aiming_point_y = aiming_point_y * pursuit_info->radius / 2500.0f;
    } else {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold -= 0.2f;
      } else {
        lateral_threshold += 0.3f;
      }
      aiming_point_y = aiming_point_y * pursuit_info->radius / 2500.0f;
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
      aiming_point_y = aiming_point_y * pursuit_info->radius / 2500.0f;
    } else if (pursuit_info->radius > 300.0f) {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold += 0.2f;
      } else {
        lateral_threshold -= 0.2f;
      }
      aiming_point_y = aiming_point_y * pursuit_info->radius / 2500.0f;
    } else {
      if(pursuit_info->curvature > 0.0f) {
        lateral_threshold += 0.3f;
      } else {
        lateral_threshold -= 0.3f;
      }
      aiming_point_y = aiming_point_y * pursuit_info->radius / 2500.0f;
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
  
  float delta =
      atanf(2 * wheel_base * lateral_distance_error_ /
           (lateral_distance_error_ * lateral_distance_error_ +
            aiming_point_y * aiming_point_y));  // 根据纯追踪算法计算车轮转角
  PRT_INFO("lat_err = %f ,  head_err = %f\n", lateral_distance_error_,
    heading_angle_error_);
  float lateral_val = -1.3f * delta;

  float theta = heading_angle_error_ / pursuit_info->steer_ratio;  // 求出车辆与车道方向的角度差rad
  float heading_val = -1.5f * theta;
  PRT_INFO(" theta: %f\n", theta);
  PRT_INFO(" lataral: %f, heading: %f  %f %f\n", lateral_val, heading_val,
            lateral_val*pursuit_info->steer_ratio*180.0f/
                    PI, heading_val*pursuit_info->steer_ratio*180.0f/
                    PI);
  
  return lateral_val + heading_val;
}

float lateral_pursuit_ComputeControl(const struct LateralConfig* lateral_config,
                                     PursuitInfo* pursuit_info) {
  float u_ff = ComputeFeedforward(pursuit_info, lateral_config);
  float u_fb = ComputeFeedback_pursuit(pursuit_info, lateral_config);
  float u_hybrid = u_fb + u_ff;
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
