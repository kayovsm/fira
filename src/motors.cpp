#include <Arduino.h>
#include "pid.cpp"

#define ENA 5  // ENA PWM Motor Esquerdo
#define ENB 6  // ENB PWM Motor Direito
#define IN1 2  // DIR Motor Esquerdo
#define IN2 4  // DIR Motor Esquerdo
#define IN3 7  // DIR Motor Direito
#define IN4 8  // DIR Motor Direito

void setupPins() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIGD, OUTPUT);
  pinMode(ECHOD, INPUT);
  pinMode(TRIGE, OUTPUT);
  pinMode(ECHOE, INPUT);
  pinMode(TRIGC, OUTPUT);
  pinMode(ECHOC, INPUT);
}

void acelera(float vel_esquerda, float vel_direita) {
  int vel_direita_int = ceil(vel_direita);
  int vel_esquerda_int = ceil(vel_esquerda);

  Serial.println("Função Acelera");
  Serial.print("velocidade direita: ");
  Serial.println(vel_direita_int);

  Serial.print("velocidade esquerda: ");
  Serial.println(vel_esquerda_int);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, vel_esquerda_int);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, vel_direita_int);
}

void ajuste(float referencia, float valor_acelera) {
  Serial.print("Valor acelera: ");
  Serial.println(valor_acelera);

  valor_acelera *= OutputC / MAX_VOLTAGE;

  Serial.print("Valor acelera POS CALCULO: ");
  Serial.println(valor_acelera);

  Serial.print("Referencia: ");
  Serial.println(referencia);

  if (referencia > 0) {
    acelera(valor_acelera, MAX_VOLTAGE);
  } else if (referencia < 0) {
    acelera(MAX_VOLTAGE, valor_acelera);
  } else {
    acelera(valor_acelera, valor_acelera);
  }
}

void move() {
  if (delta < 15 && delta > -15) {
    ajuste(delta, Outputdelta);
  } else {
    if (distanciaD < distanciaE) {
      distanciaD = 6.75 - distanciaD;
      ajuste(distanciaD, OutputD);
    } else {
      distanciaE = distanciaE - 6.75;
      ajuste(distanciaE, OutputE);
    }
  }
}