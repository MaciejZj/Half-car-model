#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#define NUM_POINTS 1000

typedef struct SuspensionParams {
  double m_1; // [kg]
  double k_1; // [N/m]
  double C_1; // [N*m/s]

  double m_2; // [kg]
  double k_2; // [N/m]
  double C_2; // [N*m/s]

  double m_s; // [kg]
  double k_3; // [N/m]
  double C_3; // [N*m/s]
  double k_4; // [N/m]
  double C_4; // [N*m/s]

  double I_s; // [kg*m^2]
  double l_F; // [m]
  double l_R; // [m]
} SuspensionParams;

typedef void (*ActuationFun)(double[], const double[],
  const double, const double);

typedef struct SimulationParams {
  double actuationTime; // [s]
  double* actuationVals;
  SuspensionParams suspensionParams;
  ActuationFun actuationFun;
} SimulationParams;

void
ramp(double z[], const double actuationVals[],
     const double t, const double actuationTime)
{
  if(t < actuationTime) {
    z[0] = 0.0; z[1] = 0.0; z[2] = 0.0; z[3] = 0.0;
  } else if(t == actuationTime) {
    z[0] = 0.0; z[1] = 0.0; z[2] = 0.0; z[3] = 0.0;
  } else {
    z[0] = actuationVals[0] * (t - actuationTime);
    z[1] = actuationVals[1]; z[2] = actuationVals[2] * (t - actuationTime); z[3] = actuationVals[3];
  }
}

void
step(double z[], const double actuationVals[],
      const double t, const double actuationTime)
{
  if(t < actuationTime) {
    z[0] = 0.0; z[1] = 0.0; z[2] = 0.0; z[3] = 0.0;
  } else if(t == actuationTime) {
    z[0] = 0.0; z[1] = 1000000.0; z[2] = 0.0; z[3] = 1000000.0;
  } else {
    z[0] = actuationVals[0]; z[1] = 0.0; z[2] = actuationVals[2]; z[3] = 0.0;
  }
}

int
func(double t, const double y[], double f[],
      void *params)
{
  SimulationParams * simulationParams = (SimulationParams *)params;
  SuspensionParams p = simulationParams->suspensionParams;
  double actuationTime = simulationParams->actuationTime;
  double * actuationVals = simulationParams->actuationVals;
  ActuationFun actuationFun = simulationParams->actuationFun;
  double z[4];
  
  (*actuationFun)(z, actuationVals, t, actuationTime);
  
  f[0] = y[1];
  f[1] = (-(p.k_1 + p.k_3)/p.m_1)*y[0]
       + (-(p.C_1 + p.C_3)/p.m_1)*y[1]
       + (p.k_3/p.m_1)*y[4]
       + (p.C_3/p.m_1)*y[5]
       + (-(p.k_3*p.l_F/p.m_1))*y[6]
       + (-(p.C_3*p.l_F/p.m_1))*y[7];
  f[2] = y[3];
  f[3] = (-(p.k_2 + p.k_4)/p.m_2)*y[2]
       + (-(p.C_2 + p.C_4)/p.m_2)*y[3]
       + (p.k_4/p.m_2)*y[4]
       + (p.C_4/p.m_2)*y[5]
       + (p.k_4*p.l_R/p.m_2)*y[6]
       + (p.C_4*p.l_R/p.m_2)*y[7];
  f[4] = y[5];
  f[5] = (p.k_3/p.m_s)*y[0]
       + (p.C_3/p.m_s)*y[1]
       + (p.k_4/p.m_s)*y[2]
       + (p.C_4/p.m_s)*y[3]
       + (-(p.k_3 + p.k_4)/p.m_s)*y[4]
       + (-(p.C_3 + p.C_4)/p.m_s)*y[5]
       + (-(p.l_R*p.k_4 - p.l_F*p.k_3)/p.m_s)*y[6]
       + (-(p.l_R*p.C_4 - p.l_F*p.C_3)/p.m_s)*y[7];
  f[6] = y[7];
  f[7] = (-(p.l_F*p.k_3)/p.I_s)*y[0]
       + (-(p.l_F*p.C_3)/p.I_s)*y[1]
       + ((p.l_R*p.k_4)/p.I_s)*y[2]
       + ((p.l_R*p.C_4)/p.I_s)*y[3]
       + (-(p.l_R*p.k_4 - p.l_F*p.k_3)/p.I_s)*y[4]
       + (-(p.l_R*p.C_4 - p.l_F*p.C_3)/p.I_s)*y[5]
       + (-(p.l_F*p.l_F*p.k_3 + p.l_R*p.l_R*p.k_4)/p.I_s)*y[6]
       + (-(p.l_F*p.l_F*p.C_3 + p.l_R*p.l_R*p.C_4)/p.I_s)*y[7];
      
  f[1] += (p.k_1/p.m_1)*z[0] + (p.C_1/p.m_1)*z[1];
  f[3] += (p.k_2/p.m_2)*z[2] + (p.C_2/p.m_2)*z[3];
  return GSL_SUCCESS;
}

int
main(void)
{
  FILE * temp = fopen("dataGcc.csv", "w");
  FILE * gnuplotPipe = popen ("gnuplot -persistent", "w");
  
  double actuationTime = 3.0;
  double actuationVals[] = {1.0, 1.0, 1.0, 1.0};
  SuspensionParams eqParams = {
    .m_1 = 20.0, .k_1 = 200000.0, .C_1 = 200.0,
    .m_2 = 20.0, .k_2 = 200000.0, .C_2 = 200.0,
    .m_s = 400.0, .k_3 = 30000.0, .C_3 = 1000.0, .k_4 = 30000.0, .C_4 = 1000.0,
    .I_s = 50.0, .l_F = 1.5, .l_R = 1.0
  };
  SimulationParams simParams = {
    .actuationTime = actuationTime,
    .actuationVals = actuationVals,
    .suspensionParams = eqParams,
    .actuationFun = &ramp
  };

  double t = 0.0, t1 = 8.0;
  double x[8] = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  int i, j;
  gsl_odeiv2_system sys = {func, NULL, 8, &simParams};
  gsl_odeiv2_driver * d =
    gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rk8pd,
                                  1e-5, 1e-5, 0.0);

  for (i = 1; i <= NUM_POINTS; i++) {
    double ti = i * t1 / (double)(NUM_POINTS);
    int status = gsl_odeiv2_driver_apply(d, &t, ti, x);
    
    if(status != GSL_SUCCESS) {
      printf("error, return value=%d\n", status);
      break;
    }

    printf("%.2e|", t);
    fprintf(temp, "%lf,", t);
    for(j = 0; j < 8; j++) {
      printf("%.2e|", x[j]);
      fprintf(temp, (j == 7 ? "%lf": "%lf,"), x[j]);
    }
    printf("\n");
    fprintf(temp, "\n");
    
  }

  gsl_odeiv2_driver_free(d);

  fprintf(gnuplotPipe, "set terminal postscript enhanced color rounded; set datafile separator ','; unset key; set xlabel 'time [s]';");
  for (i = 1; i <= 8; i++) {
    char * title = i % 2 == 0 ? "Velocity" : "Position";
    char * ylabel = i % 2 == 0 ? "velocity [m/s]" : "position [m]";
    fprintf(gnuplotPipe, "set title '%s %d'; set output 'sim%d.eps'; set ylabel '%s'; plot 'dataGcc.csv' using 1:%d with lines;\n", title, (int)ceil((float)i/2.0), i, ylabel, i + 1);
  }
  fclose(temp);
  fclose(gnuplotPipe);
  return 0;
}
