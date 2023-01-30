
#include "lqr.h"

#define fabs(x) (((x) < 0.0) ? (-x) : (x))

int compGainMatrix(const float A[2][2], const float B[2], const float Q[2][2],
                   const float R, float K[2], const float eps) {
  float X[2][2];
  X[0][0] = Q[0][0];
  X[0][1] = Q[0][1];
  X[1][0] = Q[1][0];
  X[1][1] = Q[1][1];

  float maxiter = 150;

  float XN[2][2];
  unsigned int i = 0;
  for (i = 0; i < maxiter; i++) {
    float TF[2][2];  // A.transpose() * X * A
    TF[0][0] = A[0][0] * A[0][0] * X[0][0] + A[1][0] * A[1][0] * X[1][1] +
               A[0][0] * A[1][0] * (X[1][0] + X[0][1]);
    TF[0][1] = A[0][0] * A[0][1] * X[0][0] + A[0][1] * A[1][0] * X[1][0] +
               A[0][0] * A[1][1] * X[0][1] + A[1][0] * A[1][1] * X[1][1];
    TF[1][0] = A[0][0] * A[0][1] * X[0][0] + A[0][0] * A[1][1] * X[1][0] +
               A[0][1] * A[1][0] * X[0][1] + A[1][0] * A[1][1] * X[1][1];
    TF[1][1] = A[0][1] * A[0][1] * X[0][0] + A[1][1] * A[1][1] * X[1][1] +
               A[0][1] * A[1][1] * (X[1][0] + X[0][1]);
    float fac =
        1.0f / (B[0] * B[0] * X[0][0] + B[0] * B[1] * (X[1][0] + X[0][1]) +
               B[1] * B[1] * X[1][1] + R);
    float
        TS[2];  // A.transpose() * X * B * (R + B.transpose() * X * B).inverse()
    TS[0] = fac * (A[0][0] * X[0][0] * B[0] + A[1][0] * X[1][0] * B[0] +
                   A[0][0] * X[0][1] * B[1] + A[1][0] * X[1][1] * B[1]);
    TS[1] = fac * (A[0][1] * X[0][0] * B[0] + A[1][1] * X[1][0] * B[0] +
                   A[0][1] * X[0][1] * B[1] + A[1][1] * X[1][1] * B[1]);
    float TT[2];  // B.transpose() * X * A
    TT[0] = A[0][0] * X[0][0] * B[0] + A[0][0] * X[1][0] * B[1] +
            A[1][0] * X[0][1] * B[0] + A[1][0] * X[1][1] * B[1];
    TT[1] = A[0][1] * X[0][0] * B[0] + A[0][1] * X[1][0] * B[1] +
            A[1][1] * X[0][1] * B[0] + A[1][1] * X[1][1] * B[1];
    float TE[2][2];
    TE[0][0] = TS[0] * TT[0];
    TE[0][1] = TS[0] * TT[1];
    TE[1][0] = TS[1] * TT[0];
    TE[1][1] = TS[1] * TT[1];

    XN[0][0] = TF[0][0] - TE[0][0] + Q[0][0];
    XN[0][1] = TF[0][1] - TE[0][1] + Q[0][1];
    XN[1][0] = TF[1][0] - TE[1][0] + Q[1][0];
    XN[1][1] = TF[1][1] - TE[1][1] + Q[1][1];

    float DXN[2][2];
    DXN[0][0] = fabs(XN[0][0] - X[0][0]);
    DXN[0][1] = fabs(XN[0][1] - X[0][1]);
    DXN[1][0] = fabs(XN[1][0] - X[1][0]);
    DXN[1][1] = fabs(XN[1][1] - X[1][1]);

    float maxCoeff = DXN[0][0];
    if (maxCoeff < DXN[0][1]) {
      maxCoeff = DXN[0][1];
    }
    if (maxCoeff < DXN[1][0]) {
      maxCoeff = DXN[1][0];
    }
    if (maxCoeff < DXN[1][1]) {
      maxCoeff = DXN[1][1];
    }

    if (maxCoeff < eps) {
      break;
    }
    X[0][0] = XN[0][0];
    X[0][1] = XN[0][1];
    X[1][0] = XN[1][0];
    X[1][1] = XN[1][1];
  }

  float fac =
      1.0f / (B[0] * B[0] * XN[0][0] + B[0] * B[1] * (XN[1][0] + XN[0][1]) +
             B[1] * B[1] * XN[1][1] + R);
  float COE[2];
  COE[0] = A[0][0] * XN[0][0] * B[0] + A[0][0] * XN[1][0] * B[1] +
           A[1][0] * XN[0][1] * B[0] + A[1][0] * XN[1][1] * B[1];
  COE[1] = A[0][1] * XN[0][0] * B[0] + A[0][1] * XN[1][0] * B[1] +
           A[1][1] * XN[0][1] * B[0] + A[1][1] * XN[1][1] * B[1];

  K[0] = COE[0] * fac;
  K[1] = COE[1] * fac;

  return 1;
}
