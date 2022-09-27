#include "iidm_driver_model.h"

float Calculate_Setting_Space(const struct IDMParam* config,
                              const float target_spacing, const float ego_speed,
                              const float obs_speed) {
  float speed_factor = 0.;
  if (ego_speed <= config->v0) {
    speed_factor = pow(1 - pow(ego_speed / config->v0, config->kethe), 0.5);
  } else {
    speed_factor = pow(
        1 - pow(config->v0 / ego_speed, config->kethe * config->a / config->b),0.5);
  }
  float s_star = (target_spacing) * speed_factor  +
      ego_speed * (ego_speed - obs_speed) / (2 * sqrt(config->a * config->b));
  return s_star;
}

float Calculate_Expected_Space(const struct IDMParam* config, const float s0,
                               const float time_gap, const float ego_speed,
                               const float obs_speed) {
  float headway_space = s0 + ego_speed * time_gap;  //车头时距对应距离
  float s_star = headway_space + ego_speed * (ego_speed - obs_speed) /
                                     (2 * sqrt(config->a * config->b));
  return s_star;
}

float CalculateAcceleration_IIDM_Free(const struct IDMParam* config,
                                      const float ego_speed) {
  float acc_free = 0.;
  if (ego_speed <= config->v0) {
    acc_free = config->a * (1 - pow(ego_speed / config->v0, config->kethe));
  } else {
    acc_free = -config->b * (1 - pow(config->v0 / ego_speed, config->kethe * config->a / config->b));
  }
  return acc_free;
}

float CalculateAcceleration_IIDM(const struct IDMParam* config,
                                 const float s_star, const float ego_speed,
                                 const float front_distance) {
  float acc_free = 0.;
  float acc_iidm = 0.;
  if (ego_speed <= config->v0) {
    acc_free = config->a * (1 - pow(ego_speed / config->v0, config->kethe));
    float z = s_star / front_distance;
    if (z >= 1.0) {
      acc_iidm = config->a * (1 - z * z);
    } else {
      if (fabs(acc_free) < 0.0001) {
        acc_iidm = 0.;
      } else {
        acc_iidm = acc_free * (1 - pow(z, 2 * config->a / acc_free));
      }
    }
  } else {
    acc_free = -config->b * (1 - pow(config->v0 / ego_speed,config->kethe * config->a / config->b));
    float z = s_star / front_distance;
    if (z >= 1.0) {
      acc_iidm = acc_free + config->a * (1 - z * z);
    } else {
      acc_iidm = acc_free + config->a * (1 - z * z);
    }
  }
  return acc_iidm;
}

float CalculateAcceleration_CAH(const struct IDMParam* config,
                                const float ego_speed, const float obs_speed,
                                const float obs_acc,
                                const float front_distance) {
  float acc_cah;
  float al = min(obs_acc, config->a);
  if (obs_speed * (ego_speed - obs_speed) <= -2 * front_distance * al) {
    if (fabs(obs_speed) < 0.0001 && fabs(al) < 0.0001) {
      acc_cah = 0.;
    } else {
      acc_cah = (ego_speed * ego_speed * al) /
                (obs_speed * obs_speed - 2 * front_distance * al);
    }
  } else {
    if (ego_speed - obs_speed < 0.) {
      acc_cah = al;
    } else {
      float relative_speed = ego_speed - obs_speed;
      acc_cah = al - relative_speed * relative_speed / (2 * front_distance);
    }
  }
  return acc_cah;
}

float CalculateAcceleration_ACC(const struct IDMParam* config,
                                const float acc_idm, const float acc_cah) {
  float acc_val = 0.;
  if (acc_idm >= acc_cah) {
    acc_val = acc_idm;
  } else {
    float acc_idm_t = max(acc_idm, -100.);
    float acc_cah_t = max(acc_idm, -100.);
    float x = (acc_idm_t - acc_cah_t) / config->b;
    float tanh_x = (exp(x) - exp(-x)) / (exp(x) + exp(-x));
    float a_acc = (1 - config->c) * acc_idm_t + config->c * (acc_cah_t + config->b * tanh_x);
    acc_val = a_acc;
    printf("cah: %f, %f\n", x, tanh_x, a_acc);
  }
  if (acc_val < -5) {
    acc_val = -5;
  }
  return acc_val;
}
