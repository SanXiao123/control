#ifndef LATERAL_PURSUIT_H_
#define LATERAL_PURSUIT_H_

#include "modules/algorithm/abmath/abmath.h"
#include "modules/algorithm/control/control_config.h"
#include "modules/algorithm/control/lateral/lateral_limit.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float lateral_pursuit_ComputeControl(
    const struct LateralConfig* lateral_config, PursuitInfo* pursuit_info);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
