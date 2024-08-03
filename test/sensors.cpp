#include <Arduino.h>
#include <Ultrasonic.h>
#include <math.h>

#include "utils/sensors/sensors_variables.h"

// Define o tamanho da janela para suavização dos dados
const int windowSize = 10;

// Arrays para armazenar o histórico das leituras dos sensores
long historyFront[windowSize] = {
    0};  // Histórico das leituras do sensor frontal
long historyRight[windowSize] = {
    0};  // Histórico das leituras do sensor direito
long historyLeft[windowSize] = {
    0};  // Histórico das leituras do sensor esquerdo

// Índice para controlar a posição atual no histórico
int index = 0;

// Função para ler os sensores ultrassônicos
void ler_sensores() {
  // Lê o sensor esquerdo
  long microsec =
      sensorE.timing();  // Obtém o tempo de resposta do sensor esquerdo
  distanciaE =
      min(MAX_DELTA + 1.6,
          sensorE.convert(microsec,
                          Ultrasonic::CM));  // Converte o tempo em distância e
                                             // limita o valor máximo
  distanciaE = max(0, distanciaE - 1.6);     // Ajusta a distância mínima

  // Lê o sensor direito
  microsec = sensorD.timing();  // Obtém o tempo de resposta do sensor direito
  distanciaD =
      min(MAX_DELTA + 1.5,
          sensorD.convert(microsec,
                          Ultrasonic::CM));  // Converte o tempo em distância e
                                             // limita o valor máximo
  distanciaD = max(0, distanciaD - 1.5);     // Ajusta a distância mínima

  // Lê o sensor central
  microsec = sensorC.timing();  // Obtém o tempo de resposta do sensor central
  distanciaC =
      min(40, sensorC.convert(
                  microsec, Ultrasonic::CM));  // Converte o tempo em distância
                                               // e limita o valor máximo

  // Calcula a diferença entre as distâncias dos sensores esquerdo e direito
  delta = distanciaE - distanciaD;  // Calcula a diferença entre as distâncias
  delta_abs = abs(delta);           // Calcula o valor absoluto da diferença
}

// Função para atualizar o histórico das leituras dos sensores
void updateHistory(long distanceFront, long distanceRight, long distanceLeft) {
  historyFront[index] =
      distanceFront;  // Atualiza o histórico do sensor frontal
  historyRight[index] =
      distanceRight;                  // Atualiza o histórico do sensor direito
  historyLeft[index] = distanceLeft;  // Atualiza o histórico do sensor esquerdo
  index = (index + 1) % windowSize;   // Incrementa o índice e o reinicia se
                                      // atingir o tamanho da janela
}

// Função para calcular a média das leituras de um histórico
long getAverage(long history[]) {
  long sum = 0;                      // Inicializa a soma das leituras
  for (int i = 0; i < index; i++) {  // Itera sobre as leituras no histórico
    sum += history[i];               // Soma as leituras
  }
  return sum / index;  // Retorna a média das leituras
}

// Funções para obter as distâncias suavizadas dos sensores
long getSmoothedDistanceFront() {
  return getAverage(historyFront);
}  // Retorna a média das leituras do sensor frontal

long getSmoothedDistanceRight() {
  return getAverage(historyRight);
}  // Retorna a média das leituras do sensor direito

long getSmoothedDistanceLeft() {
  return getAverage(historyLeft);
}  // Retorna a média das leituras do sensor esquerdo