#include <Arduino.h>
#include <Ultrasonic.h>
#include <math.h>
#include <PID_v1.h>
#include "utils/sensors.h"

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

void imprimeDistancias() {
  Serial.print("Dis Esq: ");
  Serial.print(distanciaE);
  Serial.print(" cm  /  ");

  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" cm   /  ");

  Serial.print("Dis Dir: ");
  Serial.println(distanciaD);
}

// Pinagem dos sensores ultrassônicos
#define TRIGD A4  // Pino Trig Sensor Direita
#define ECHOD A5  // Pino Echo Sensor Direita
#define TRIGC A0  // Pino Trig Sensor Centro
#define ECHOC A1  // Pino Echo Sensor Centro
#define TRIGE A2  // Pino Trig Sensor Esquerda
#define ECHOE A3  // Pino Echo Sensor Esquerda

#define MAX_DELTA 30  // Definindo a constante MAX_DELTA

Ultrasonic sensorD(TRIGD, ECHOD);
Ultrasonic sensorE(TRIGE, ECHOE);
Ultrasonic sensorC(TRIGC, ECHOC);

#define N 5  // número de pontos da média móvel

double values_center[N], values_left[N],
    values_right[N];  // vetor para média móvel

void moving_average()  // retorna a média móvel de acordo com a resolução
                       // designada
{
  int i;               // variável para iterações
  double adder_c = 0;  // variável para somatório
  double adder_d = 0;  // variável para somatório
  double adder_e = 0;  // variável para somatório

  for (i = N - 1; i > 0;
       i--)  // desloca todo vetor descartando o elemento mais antigo
  {
    values_center[i] = values_center[i - 1];
    values_left[i] = values_left[i - 1];
    values_right[i] = values_right[i - 1];
  }

  values_center[0] =
      distanciaC;  // o primeiro elemento do vetor recebe o valor do pulso
  values_left[0] =
      distanciaE;  // o primeiro elemento do vetor recebe o valor do pulso
  values_right[0] =
      distanciaD;  // o primeiro elemento do vetor recebe o valor do pulso

  for (i = 0; i < N; i++)  // faz a somatória
  {
    adder_c = adder_c + values_center[i];
    adder_e = adder_e + values_left[i];
    adder_d = adder_d + values_right[i];
  }

  distanciaC = adder_c / N;  // retorna a média
  distanciaE = adder_e / N;  // retorna a média
  distanciaD = adder_d / N;  // retorna a média
}
void ler_sensores(int first) {
  long microsec = sensorE.timing();
  distanciaE =
      min(MAX_DELTA + 1.6,
          sensorE.convert(
              microsec, Ultrasonic::CM));  // filtro(sensorE.convert(microsec,
                                           // Ultrasonic::CM), distanciaE, 1.0);
  distanciaE = max(0, distanciaE - 1.6);

  microsec = sensorD.timing();
  distanciaD =
      min(MAX_DELTA + 1.5,
          sensorD.convert(
              microsec, Ultrasonic::CM));  // filtro(sensorD.convert(microsec,
                                           // Ultrasonic::CM), distanciaD, 1.0);
  distanciaD = max(0, distanciaD - 1.5);

  microsec = sensorC.timing();
  distanciaC = min(
      40, sensorC.convert(
              microsec, Ultrasonic::CM));  // filtro(sensorC.convert(microsec,
                                           // Ultrasonic::CM), distanciaC, 1.0);

  imprimeDistancias();

  if (first) {
    for (int i = 0; i < N; i++) {
      values_center[i] = distanciaC;
      values_left[i] = distanciaE;
      values_right[i] = distanciaD;
    }
  }

  moving_average();

  delta = distanciaE - distanciaD;
  delta_abs = abs(delta);
}