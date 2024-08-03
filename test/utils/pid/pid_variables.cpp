// variables.cpp

#include "pid_variables.h"

// Definição das variáveis
double OutputC;
double OutputE;
double OutputD;
double Outputdelta;

double distanciaC;
double distanciaE;  // Distância lida pelo sensor esquerdo
double distanciaD;  // Distância lida pelo sensor direito

double distanciaC_abs;
double distanciaE_abs;  // Distância absoluta lida pelo sensor esquerdo
double distanciaD_abs;  // Distância absoluta lida pelo sensor direito
double delta;
double delta_abs;

float MAX_VOLTAGE = 70.0;
float MIN_PERCENT = 45.0;

double SetpointCentral = 20;
double SetpointLaterais = 6.75;

double kpCentral = 5.0, kiCentral = 3.5, kdCentral = 2.0;
double kpLateral = 14.0, kiLateral = 0.0, kdLateral = 0.0;

PID PIDc(&distanciaC, &OutputC, &SetpointCentral, kpCentral, kiCentral, kdCentral, REVERSE);
PID PIDd(&distanciaD_abs, &OutputD, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);
PID PIDe(&distanciaE_abs, &OutputE, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);
PID PIDdelta(&delta_abs, &Outputdelta, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);