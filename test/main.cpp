#include <Arduino.h>
#include <SoftwareSerial.h>
#include "motors.cpp"
#include "pid.cpp"
#include "prints.cpp"
#include "sensors.cpp"

SoftwareSerial bluetooth(8, 9);  // rx, tx amarelo e verde respectivamente

void setup() {
  Serial.begin(9600);     // Comunicação Serial com o Computador
  bluetooth.begin(9600);  // Inicializa a comunicação Bluetooth
  setupPins();
  setupPID();
  ler_sensores();
  delay(2000);
}

void loop() {
  ler_sensores();
  imprimeDistancias();
  computePID();
  move(1);
}