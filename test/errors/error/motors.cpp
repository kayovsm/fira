#include <Arduino.h>
#include "utils/motors.h"
#include "utils/pid.h"
#include "utils/sensors.h"

// Definição dos pinos para controle dos motores
#define ENA 5  // Pino PWM para o motor esquerdo
#define ENB 6  // Pino PWM para o motor direito
#define IN1 2  // Pino de direção para o motor esquerdo
#define IN2 4  // Pino de direção para o motor esquerdo
#define IN3 7  // Pino de direção para o motor direito
#define IN4 8  // Pino de direção para o motor direito

// Função para configurar os pinos como saída ou entrada
void setupPins() {
  pinMode(ENA, OUTPUT);  // Configura o pino ENA como saída
  pinMode(ENB, OUTPUT);  // Configura o pino ENB como saída
  pinMode(IN1, OUTPUT);  // Configura o pino IN1 como saída
  pinMode(IN2, OUTPUT);  // Configura o pino IN2 como saída
  pinMode(IN3, OUTPUT);  // Configura o pino IN3 como saída
  pinMode(IN4, OUTPUT);  // Configura o pino IN4 como saída

  // pinMode(TRIGD, OUTPUT);  // Configura o pino TRIGD como saída
  // pinMode(ECHOD, INPUT);   // Configura o pino ECHOD como entrada
  // pinMode(TRIGE, OUTPUT);  // Configura o pino TRIGE como saída
  // pinMode(ECHOE, INPUT);   // Configura o pino ECHOE como entrada
  // pinMode(TRIGC, OUTPUT);  // Configura o pino TRIGC como saída
  // pinMode(ECHOC, INPUT);   // Configura o pino ECHOC como entrada
}

// Função para acelerar os motores com velocidades especificadas
void acelera(float vel_esquerda, float vel_direita) {
  // Converte as velocidades para inteiros
  int vel_direita_int = ceil(vel_direita);
  int vel_esquerda_int = ceil(vel_esquerda);

  // Imprime as velocidades no monitor serial
  Serial.println("Função Acelera");
  Serial.print("velocidade direita: ");
  Serial.println(vel_direita_int);
  Serial.print("velocidade esquerda: ");
  Serial.println(vel_esquerda_int);

  // Configura a direção e a velocidade do motor direito
  digitalWrite(IN3, HIGH);  // Define a direção do motor direito
  digitalWrite(IN4, LOW);   // Define a direção do motor direito
  analogWrite(ENB, vel_esquerda_int);  // Define a velocidade do motor direito

  // Configura a direção e a velocidade do motor esquerdo
  digitalWrite(IN1, HIGH);  // Define a direção do motor esquerdo
  digitalWrite(IN2, LOW);   // Define a direção do motor esquerdo
  analogWrite(ENA, vel_direita_int);  // Define a velocidade do motor esquerdo
}

// Função para ajustar a velocidade dos motores com base em uma referência
void ajuste(float referencia, float valor_acelera) {
  // Imprime o valor de aceleração no monitor serial
  Serial.print("Valor acelera: ");
  Serial.println(valor_acelera);

  // Ajusta o valor de aceleração com base na saída do controlador PID
  valor_acelera *= OutputC / MAX_VOLTAGE;

  // Imprime o valor de aceleração ajustado no monitor serial
  Serial.print("Valor acelera POS CALCULO: ");
  Serial.println(valor_acelera);

  // Imprime a referência no monitor serial
  Serial.print("Referencia: ");
  Serial.println(referencia);

  // Ajusta a velocidade dos motores com base na referência
  if (referencia > 0) {
    acelera(valor_acelera, MAX_VOLTAGE);  // Acelera o motor esquerdo
  } else if (referencia < 0) {
    acelera(MAX_VOLTAGE, valor_acelera);  // Acelera o motor direito
  } else {
    acelera(valor_acelera, valor_acelera);  // Acelera ambos os motores igualmente
  }
}

// Função para mover o robô com base nas leituras dos sensores
// 0 -> utiliza PID
// 1 -> utiliza funções calculadas manualmente
void move(int use) {
  // Verifica se o delta está dentro de um intervalo aceitável
  if (delta < 15 && delta > -15) {
    //caso use seja igual a 1, o robô irá calcular a velocidade dos motores com base na função senoidal
    if(use)
    {
      funcoes_math(delta_abs, &Outputdelta);
      funcoes_math(distanciaE_abs, &OutputE);
      funcoes_math(distanciaD_abs, &OutputD);
    }
    ajuste(delta, Outputdelta);  // Ajusta a velocidade dos motores com base no delta
  } else {
    // Ajusta a velocidade dos motores com base nas distâncias dos sensores
    if (distanciaD < distanciaE) {
      distanciaD = 6.75 - distanciaD;  // Calcula a nova distância para o motor direito
      ajuste(distanciaD, OutputD);  // Ajusta a velocidade do motor direito
    } else {
      distanciaE = distanciaE - 6.75;  // Calcula a nova distância para o motor esquerdo
      ajuste(distanciaE, OutputE);  // Ajusta a velocidade do motor esquerdo
    }
  }
}