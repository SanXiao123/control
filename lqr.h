#ifndef LQR_
#define LQR_

#ifdef __cplusplus
extern "C" {
#endif

int compGainMatrix(const float A[2][2], const float B[2], const float Q[2][2],
                   const float R, float K[2], const float eps);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
