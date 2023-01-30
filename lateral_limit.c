#include "lateral_limit.h"

#define MAX_STEERINGANGLE (460.0f)
#define MAX_LEN (11)

static float speed_kph[MAX_LEN] = {0.0f, 10.0f, 15.0f, 20.0f, 30.0f, 40.0f, 60.0f, 80.0f, 100.0f, 120.0f, 160.0f};
static float angle_deg[MAX_LEN] = {0.9f, 0.9f, 0.8f, 0.5f, 0.25f, 0.15f, 0.1f, 0.1f, 0.075f, 0.075f, 0.075f};
static float angle_rate[MAX_LEN] = {360.0f, 360.0f, 360.0f, 300.0f, 300.0f, 300.0f, 240.0f, 240.0f, 180.0f, 150.0f, 100.0f};

float Restrain_SteeringAngle(const float current_steering_angle,
                      const struct LateralConfig* lateral_config,
                      const PursuitInfo* pursuit_info) {
  float steering_angle_limit = LinearSpline_Build(
      speed_kph, angle_deg, MPS_TO_KPH(pursuit_info->vehicle_speed),
      MAX_LEN);
  PRT_INFO("steering_angle_limit is :%f \n", steering_angle_limit);
  steering_angle_limit = MAX_STEERINGANGLE * steering_angle_limit;
  float u_min = -steering_angle_limit;
  float u_max = steering_angle_limit;
  float tire_steering_in_rad =
      saturate(current_steering_angle, -steering_angle_limit, steering_angle_limit);
  PRT_INFO("Restrain_SteeringAngle1 is :%f %f %f\n", current_steering_angle, u_min, u_max);

  float steering_rate_limit = LinearSpline_Build(
      speed_kph, angle_rate, MPS_TO_KPH(pursuit_info->vehicle_speed),
      MAX_LEN);
  PRT_INFO("steering_rate_limit is :%f \n", steering_rate_limit);
  steering_rate_limit = steering_rate_limit * lateral_config->step_time;
  u_min = pursuit_info->previous_tire_steering_in_rad - steering_rate_limit;
  u_max = pursuit_info->previous_tire_steering_in_rad + steering_rate_limit;
  PRT_INFO("Restrain_SteeringAngle2 is :%f %f %f %f\n", pursuit_info->previous_tire_steering_in_rad, tire_steering_in_rad, u_min, u_max);
  tire_steering_in_rad = saturate(tire_steering_in_rad, u_min, u_max);

  // update previous
  //pursuit_info->previous_tire_steering_in_rad = tire_steering_in_rad;
  return tire_steering_in_rad;
}

float Restrain(const float current_tire_steering_in_rad,
                      const struct LateralConfig* lateral_config,
                      const PursuitInfo* pursuit_info) {
  float speed = fmaxf(pursuit_info->vehicle_speed, 1.0f);

  // saturate by accleration
  float u_min =
      lateral_config->acceleration_limits[0] *
      (lateral_config->wheel_base_front + lateral_config->wheel_base_rear) / (speed * speed);
  float u_max =
      lateral_config->acceleration_limits[1] *
      (lateral_config->wheel_base_front + lateral_config->wheel_base_rear) /
      (speed * speed);
  PRT_INFO("Restrain1 is :%f %f %f\n", current_tire_steering_in_rad, u_min, u_max);
  float tire_steering_in_rad =
      saturate(current_tire_steering_in_rad, u_min, u_max);
  
  // saturate by jerk
  u_min =
      pursuit_info->previous_tire_steering_in_rad * PI / 180.0f / pursuit_info->steer_ratio +
      lateral_config->jerk_limits[0] * lateral_config->step_time *
          (lateral_config->wheel_base_front + lateral_config->wheel_base_rear) /
          (speed * speed);
  u_max =
      pursuit_info->previous_tire_steering_in_rad * PI / 180.0f / pursuit_info->steer_ratio +
      lateral_config->jerk_limits[1] * lateral_config->step_time *
          (lateral_config->wheel_base_front + lateral_config->wheel_base_rear) /
          (speed * speed);
  PRT_INFO("Restrain2 is :%f %f %f %f\n", pursuit_info->previous_tire_steering_in_rad * PI / 180.0f / pursuit_info->steer_ratio, tire_steering_in_rad, u_min, u_max);
  tire_steering_in_rad = saturate(tire_steering_in_rad, u_min, u_max);

  // update previous
  //pursuit_info->previous_tire_steering_in_rad = tire_steering_in_rad;
  return tire_steering_in_rad;
}

float ComputeFeedforward(const PursuitInfo* pursuit_info,
                                const struct LateralConfig* lateral_config) {
  float ff1 = pursuit_info->curvature * (lateral_config->wheel_base_front +
                                          lateral_config->wheel_base_rear);
  float ff2 = pursuit_info->curvature * lateral_config->understeer_gradient *
              (pursuit_info->vehicle_speed * pursuit_info->vehicle_speed);
  PRT_INFO(" ff1: %f, ff2: %f, cuv: %f %f, %f\n", ff1, ff2, pursuit_info->curvature, ff1*pursuit_info->steer_ratio*180.0f/
                    PI, ff2*pursuit_info->steer_ratio*180.0f/
                    PI);

  return ff1 + ff2;
}
