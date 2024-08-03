#include <PID_v1.h>
#include "sensors.cpp"
#include "utils/pid/pid_variables.h"

// Função para configurar os controladores PID
void setupPID() {
  // Configura os modos dos controladores PID para automático
  PIDe.SetMode(AUTOMATIC); // Configura o controlador PIDe para modo automático
  PIDc.SetMode(AUTOMATIC); // Configura o controlador PIDc para modo automático
  PIDd.SetMode(AUTOMATIC); // Configura o controlador PIDd para modo automático
  PIDdelta.SetMode(AUTOMATIC); // Configura o controlador PIDdelta para modo automático

  // Define os parâmetros de ajuste dos controladores PID
  PIDe.SetTunings(kpLateral, kiLateral, kdLateral); // Define os parâmetros kp, ki, kd para o controlador PIDe
  PIDd.SetTunings(kpLateral, kiLateral, kdLateral); // Define os parâmetros kp, ki, kd para o controlador PIDd
  PIDc.SetTunings(kpCentral, kiCentral, kdCentral); // Define os parâmetros kp, ki, kd para o controlador PIDc
  PIDdelta.SetTunings(kpLateral, kiLateral, kdLateral); // Define os parâmetros kp, ki, kd para o controlador PIDdelta

  // Define os limites de saída dos controladores PID
  PIDc.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE); // Define os limites de saída para o controlador PIDc
  PIDe.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE); // Define os limites de saída para o controlador PIDe
  PIDd.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE); // Define os limites de saída para o controlador PIDd
  PIDdelta.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE); // Define os limites de saída para o controlador PIDdelta
}

// Função para calcular os valores dos controladores PID
void computePID() {
  // Calcula as distâncias absolutas
  distanciaD_abs = abs(distanciaD - 6.75); // Calcula a distância absoluta para o lado direito
  distanciaE_abs = abs(distanciaE - 6.75); // Calcula a distância absoluta para o lado esquerdo

  // Calcula o valor do controlador PID central
  PIDc.Compute(); // Calcula o valor de saída do controlador PIDc
  Serial.print("Output Central: ");
  Serial.println(OutputC);

  // Calcula o valor do controlador PID esquerdo
  Serial.print("Distancia Esquerda Absoluta: ");
  Serial.println(distanciaE_abs);
  PIDe.Compute();
  Serial.print("Output Esquerda: ");
  Serial.println(OutputE);

  // Calcula o valor do controlador PID direito
  Serial.print("Distancia Direita Absoluta: ");
  Serial.println(distanciaD_abs);
  PIDd.Compute();
  Serial.print("Output Direita: ");
  Serial.println(OutputD);

  // Calcula o valor do controlador PID delta
  PIDdelta.Compute();
}

// Função para calcular os valores dos controladores PID
void computeAJUSTE() {
  // Calcula as distâncias absolutas
  distanciaD_abs = abs(distanciaD - 6.75); // Calcula a distância absoluta para o lado direito
  distanciaE_abs = abs(distanciaE - 6.75); // Calcula a distância absoluta para o lado esquerdo

  float valor_acelera = ceil((-50.0/15.5)*abs(delta_abs) + 100.0);


  

  // Calcula o valor do controlador PID central
  PIDc.Compute(); // Calcula o valor de saída do controlador PIDc
  Serial.print("Output Central: ");
  Serial.println(OutputC);

  // Calcula o valor do controlador PID esquerdo
  Serial.print("Distancia Esquerda Absoluta: ");
  Serial.println(distanciaE_abs);
  PIDe.Compute();
  Serial.print("Output Esquerda: ");
  Serial.println(OutputE);

  // Calcula o valor do controlador PID direito
  Serial.print("Distancia Direita Absoluta: ");
  Serial.println(distanciaD_abs);
  PIDd.Compute();
  Serial.print("Output Direita: ");
  Serial.println(OutputD);

  // Calcula o valor do controlador PID delta
  PIDdelta.Compute();
}