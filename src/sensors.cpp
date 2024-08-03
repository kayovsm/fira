#include <Ultrasonic.h>
#include <math.h>
#include <Arduino.h>

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

double distanciaC;
double distanciaE;
double distanciaD;
double delta;
double delta_abs;

const int windowSize = 10;
long historyFront[windowSize] = {0};
long historyRight[windowSize] = {0};
long historyLeft[windowSize] = {0};
int index = 0;

void ler_sensores() {
  long microsec = sensorE.timing();
  distanciaE = min(MAX_DELTA + 1.6, sensorE.convert(microsec, Ultrasonic::CM));
  distanciaE = max(0, distanciaE - 1.6);

  microsec = sensorD.timing();
  distanciaD = min(MAX_DELTA + 1.5, sensorD.convert(microsec, Ultrasonic::CM));
  distanciaD = max(0, distanciaD - 1.5);

  microsec = sensorC.timing();
  distanciaC = min(40, sensorC.convert(microsec, Ultrasonic::CM));

  delta = distanciaE - distanciaD;
  delta_abs = abs(delta);
}

void updateHistory(long distanceFront, long distanceRight, long distanceLeft) {
  historyFront[index] = distanceFront;
  historyRight[index] = distanceRight;
  historyLeft[index] = distanceLeft;
  index = (index + 1) % windowSize;
}

long getAverage(long history[]) {
  long sum = 0;
  for (int i = 0; i < index; i++) {
    sum += history[i];
  }
  return sum / index;
}

long getSmoothedDistanceFront() {
  return getAverage(historyFront);
}

long getSmoothedDistanceRight() {
  return getAverage(historyRight);
}

long getSmoothedDistanceLeft() {
  return getAverage(historyLeft);
}