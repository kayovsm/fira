#include <Arduino.h>
#include "sensors.cpp"

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