#ifndef LATERAL_LQR_H_
#define LATERAL_LQR_H_

#include "lqr.h"
#include "modules/algorithm/abmath/abmath.h"
#include "modules/algorithm/control/control_config.h"
#include "modules/algorithm/control/lateral/lateral_limit.h"

struct lqr_settings {
  float dvx;                               // = 0.5;
  float speed_limits_max;                  // = 40; // m/s
  float speed_limits_min;                  // = 3; // m/s
  int step_count;                          // = 75;
  float speed_range[75];                   // = {0};
  float lateral_distance_error_gains[75];  // = {0};
  float heading_angle_error_gains[75];     // = {0};
  float previous_tire_steering_in_rad_;    // = 0.;
};

#ifdef __cplusplus
extern "C" {
#endif

extern void lateral_ComputeInit(const struct LateralConfig* lateral_config,
                                struct lqr_settings* lqr_instance);
extern float lateral_ComputeControl(const PursuitInfo* pursuit_info,
                                    const struct LateralConfig* lateral_config,
                                    struct lqr_settings* lqr_instance);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
