#ifndef STRUCTS_H_
#define STRUCTS_H_

struct LateralState {
  float angle_target;
  float angle_ego;
  float angle_vel_target;
  float angle_vel_ego;
  float torque_state;
  float curvature;
  float lateral_distance_error;
  float heading_angle_error;
  float speed;
  int autodrive_enable;
};

#endif
