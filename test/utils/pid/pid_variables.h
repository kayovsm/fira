// variables.h

#ifndef VARIABLES_H
#define VARIABLES_H

#include <PID_v1.h>

// Variáveis de saída dos controladores PID
extern double OutputC;
extern double OutputE;
extern double OutputD;
extern double Outputdelta;

// Variáveis de distância lidas pelos sensores
extern double distanciaC;
extern double distanciaE;  // Distância lida pelo sensor esquerdo
extern double distanciaD;  // Distância lida pelo sensor direito

// Variáveis de distância absoluta
extern double distanciaC_abs;
extern double distanciaE_abs;  // Distância absoluta lida pelo sensor esquerdo
extern double distanciaD_abs;  // Distância absoluta lida pelo sensor direito
extern double delta;
extern double delta_abs;

// Limites de tensão e porcentagem
extern float MAX_VOLTAGE;
extern float MIN_PERCENT;

// Setpoints para os controladores PID
extern double SetpointCentral;
extern double SetpointLaterais;

// Parâmetros dos controladores PID
extern double kpCentral, kiCentral, kdCentral;
extern double kpLateral, kiLateral, kdLateral;

// Instâncias dos controladores PID
extern PID PIDc;
extern PID PIDd;
extern PID PIDe;
extern PID PIDdelta;

// VARIAVEL PARA ESCOLHER ENTRE AJUSTE OU PID
extern bool isPID;

#endif // VARIABLES_H