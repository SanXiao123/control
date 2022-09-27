#ifndef IIDM_DRIVER_MODEL_H
#define IIDM_DRIVER_MODEL_H

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

struct IDMParam {
  float v0;     // Free flow speed
  float kethe;  // free acceleration exponent  4
  float a;      // maximum acceleration 1.4
  float b;      // desired deceleration 2
  float c;      // coolness factor 0.99
};

#ifdef __cplusplus
extern "C" {
#endif

float Calculate_Setting_Space(const struct IDMParam* config,
                              const float target_spacing, const float ego_speed,
                              const float obs_speed);
float Calculate_Expected_Space(const struct IDMParam* config, const float s0,
                               const float time_gap, const float ego_speed,
                               const float obs_speed);
float CalculateAcceleration_IIDM_Free(const struct IDMParam* config,
                                      const float ego_speed);
float CalculateAcceleration_IIDM(const struct IDMParam* config,
                                 const float s_star, const float ego_speed,
                                 const float front_distance);
float CalculateAcceleration_CAH(const struct IDMParam* config,
                                const float ego_speed, const float obs_speed,
                                const float obs_acc,
                                const float front_distance);
float CalculateAcceleration_ACC(const struct IDMParam* config,
                                const float acc_idm, const float acc_cah);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // IIDM_DRIVER_MODEL_H
