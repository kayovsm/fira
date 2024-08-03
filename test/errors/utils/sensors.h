#ifndef SENSORS_H
#define SENSORS_H

void imprimeDistancias();
void ler_sensores(int first = 0);

extern double OutputC;
extern double OutputE;
extern double OutputD;
extern double Outputdelta;

extern double distanciaC;
extern double distanciaE;
extern double distanciaD;

extern double distanciaC_abs;
extern double distanciaE_abs;
extern double distanciaD_abs;
extern double delta;
extern double delta_abs;

extern float MAX_VOLTAGE;
extern float MIN_PERCENT;

extern double SetpointCentral;
extern double SetpointLaterais;

extern double kpCentral, kiCentral, kdCentral;
extern double kpLateral, kiLateral, kdLateral;

#endif // SENSORS_H