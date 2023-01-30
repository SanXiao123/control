#ifndef LATERAL_LIMIT_H_
#define LATERAL_LIMIT_H_

#include "modules/algorithm/abmath/abmath.h"
#include "modules/algorithm/control/control_config.h"

#ifdef __cplusplus
extern "C" {
#endif

float Restrain_SteeringAngle(const float current_steering_angle,
                      const struct LateralConfig* lateral_config,
                      const PursuitInfo* pursuit_info);

float Restrain(const float current_tire_steering_in_rad,
                      const struct LateralConfig* lateral_config,
                      const PursuitInfo* pursuit_info);

float ComputeFeedforward(const PursuitInfo* pursuit_info,
                      const struct LateralConfig* lateral_config);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
