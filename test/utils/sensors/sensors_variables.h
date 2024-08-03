// sensor_variables.h

#ifndef SENSOR_VARIABLES_H
#define SENSOR_VARIABLES_H

#include <Ultrasonic.h>

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

// Pinagem dos sensores ultrassônicos
#define TRIGD A4  // Pino Trig Sensor Direita
#define ECHOD A5  // Pino Echo Sensor Direita
#define TRIGC A0  // Pino Trig Sensor Centro
#define ECHOC A1  // Pino Echo Sensor Centro
#define TRIGE A2  // Pino Trig Sensor Esquerda
#define ECHOE A3  // Pino Echo Sensor Esquerda

#define MAX_DELTA 30  // Definindo a constante MAX_DELTA

extern Ultrasonic sensorD;
extern Ultrasonic sensorE;
extern Ultrasonic sensorC;

#endif // SENSOR_VARIABLES_H