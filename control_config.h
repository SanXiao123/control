#ifndef CONTROL_CONFIG_
#define CONTROL_CONFIG_

struct LqrSettings {
  float input_weight;
  float state_input_weight_ratio;
  float heading_error_lateral_error_weight_ratio;
  float time_step;
  float speed_range[2];
};

struct PidSettings {
  float p_gain;
  float i_gain;
  float d_gain;
  float time_step;
  float i_limit[2];
};

struct LateralConfig {
  float understeer_gradient;
  float wheel_base_front;
  float wheel_base_rear;

  float lateral_error[2];
  float heading_error[2];

  float steering_angle_limits[2];

  float acceleration_limits[2];

  float jerk_limits[2];

  float tire_to_wheel_polynomial_coefficients[4];

  struct LqrSettings lqr_settings;

  struct PidSettings torque_;
  struct PidSettings torque_pos_;
  struct PidSettings torque_vel_;

  float lateral_error_integration_limits[2];
  float lateral_error_integration_gain;

  float step_time;
};
struct LongitudinalConfig {
  float maximum_brake;

  float acceleration_target_limits[2];

  struct PidSettings pos_;

  struct PidSettings vel_;
};

struct ControlConfig {
  struct LateralConfig lateral_controller_conf;
  struct LongitudinalConfig longitudinal_controller_conf;
};

typedef struct _PursuitInfo {
  unsigned char turn_mode;  // 0:默认值，1：左偏，2：右偏
  float vehicle_speed;
  float radius;
  float curvature;
  float vehicle_width;
  float steer_ratio;
  float lateral_threshold;
  float active_line_a;
  float active_line_b;
  float active_line_c;
  float active_line_d;
  float lateral_distance_error;
  float heading_angle_error;
  float previous_tire_steering_in_rad;
} PursuitInfo;

#endif
