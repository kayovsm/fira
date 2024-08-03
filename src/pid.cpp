#include <PID_v1.h>
#include "sensors.cpp"

double OutputC;
double OutputE;
double OutputD;
double Outputdelta;

double SetpointCentral = 20;
double SetpointLaterais = 6.75;

double kpCentral = 5.0, kiCentral = 3.5, kdCentral = 2.0;
double kpLateral = 14.0, kiLateral = 0.0, kdLateral = 0.0;

PID PIDc(&distanciaC, &OutputC, &SetpointCentral, kpCentral, kiCentral, kdCentral, REVERSE);
PID PIDd(&distanciaD_abs, &OutputD, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);
PID PIDe(&distanciaE_abs, &OutputE, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);
PID PIDdelta(&delta_abs, &Outputdelta, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);

void setupPID() {
  PIDe.SetMode(AUTOMATIC);
  PIDc.SetMode(AUTOMATIC);
  PIDd.SetMode(AUTOMATIC);
  PIDdelta.SetMode(AUTOMATIC);

  PIDe.SetTunings(kpLateral, kiLateral, kdLateral);
  PIDd.SetTunings(kpLateral, kiLateral, kdLateral);
  PIDc.SetTunings(kpCentral, kiCentral, kdCentral);
  PIDdelta.SetTunings(kpLateral, kiLateral, kdLateral);

  PIDc.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);
  PIDe.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);
  PIDd.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);
  PIDdelta.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);
}

void computePID() {
  distanciaD_abs = abs(distanciaD - 6.75);
  distanciaE_abs = abs(distanciaE - 6.75);
  PIDc.Compute();
  Serial.print("Output Central: ");
  Serial.println(OutputC);
  Serial.print("Distancia Esquerda Absoluta: ");
  Serial.println(distanciaE_abs);
  PIDe.Compute();
  Serial.print("Output Esquerda: ");
  Serial.println(OutputE);
  Serial.print("Distancia Direita Absoluta: ");
  Serial.println(distanciaD_abs);
  PIDd.Compute();
  Serial.print("Output Direita: ");
  Serial.println(OutputD);
  PIDdelta.Compute();
}