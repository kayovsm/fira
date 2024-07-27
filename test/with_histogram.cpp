#include <math.h>
#include <iostream>



// Variáveis Globais
float distanciaE; // distancia para leitura do sensor esquerdo
float distanciaC; // distancia para leitura do sensor central
float distanciaD; // distancia para leitura do sensor direita
int vel = 100;

unsigned long time;

float MAX_DELTA = 40;

float MAX_VOLTAGE = 140;
float MIN_PERCENT = 35;

float delta;

float tratamento(float vel) // recebe como porcentagem e transforma para analogico
{
  if (vel > 100)
  {
    vel = 100;
  }
  if (vel < MIN_PERCENT)
  {
    vel = MIN_PERCENT;
  }
  vel = (vel)*MAX_VOLTAGE / 100;
  return vel;
}

void acelera(float vel_esquerda, float vel_direita)
{

  int vel_direita_int = ceil(tratamento(vel_direita));
  int vel_esquerda_int = ceil(tratamento(vel_esquerda));

  printf("vel_direita_int: %d", vel_direita_int);

  // Serial.print("VD: ");
  // Serial.println(vel_direita);
  // Serial.print("VE: ");
  // Serial.println(vel_esquerda);
}

const int windowSize = 5;

long historyFront[windowSize] = {0};
long historyRight[windowSize] = {0};
long historyLeft[windowSize] = {0};

int index = 0;

const int threshold = 30; // Distância limite para considerar um obstáculo (em cm)

int occupancyHistogram[3]; // 0: esquerda, 1: frente, 2: direita
int polarHistogram[3];     // 0: esquerda, 1: frente, 2: direita

void updateHistory(long distanceFront, long distanceRight, long distanceLeft)
{
  historyFront[index] = distanceFront;
  historyRight[index] = distanceRight;
  historyLeft[index] = distanceLeft;
  index = (index + 1) % windowSize;
}

long getAverage(long history[])
{
  long sum = 0;
  for (int i = 0; i < windowSize; i++)
  {
    sum += history[i];
  }
  return sum / windowSize;
}

long getSmoothedDistanceFront()
{
  return getAverage(historyFront);
}

long getSmoothedDistanceRight()
{
  return getAverage(historyRight);
}

long getSmoothedDistanceLeft()
{
  return getAverage(historyLeft);
}

void ler_sensores()
{
  long microsec = sensorE.timing();
  distanciaE = min(MAX_DELTA + 2, sensorE.convert(microsec, Ultrasonic::CM)); // filtro(sensorE.convert(microsec, Ultrasonic::CM), distanciaE, 1.0);
  distanciaE = max(0, distanciaE - 2);

  microsec = sensorD.timing();
  distanciaD = min(MAX_DELTA + 1.5, sensorD.convert(microsec, Ultrasonic::CM)); // filtro(sensorD.convert(microsec, Ultrasonic::CM), distanciaD, 1.0);
  distanciaD = max(0, distanciaD - 1.5);

  microsec = sensorC.timing();
  distanciaC = min(MAX_DELTA, sensorC.convert(microsec, Ultrasonic::CM)); // filtro(sensorC.convert(microsec, Ultrasonic::CM), distanciaC, 1.0);

  updateHistory(distanciaC, distanciaD, distanciaE);

  long distanciaC = getSmoothedDistanceFront();
  long distanciaD = getSmoothedDistanceRight();
  long distanciaE = getSmoothedDistanceLeft();

  delta = distanciaE - distanciaD;
}

void setup()
{
  Serial.begin(9600);    // Comunicação Serial com o Computador
  bluetooth.begin(9600); // Inicializa a comunicação Bluetooth
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); // definição dos pinos entradas e saidas
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); // OUTPUT = Saída
  pinMode(IN4, OUTPUT); // INPUT = Entrada
  pinMode(TRIGD, OUTPUT);
  pinMode(ECHOD, INPUT);
  pinMode(TRIGE, OUTPUT);
  pinMode(ECHOE, INPUT);
  pinMode(TRIGC, OUTPUT);
  pinMode(ECHOC, INPUT);

  ler_sensores();
  delay(2000);
  time = millis();
}

void ajuste_ajustado()
{
  // variável que calcula o quanto uma roda deverá diminuir para ajustar o carrinho
  float valor_acelera = ceil(cos(delta * -0.085) * 100);

  // mais a direita
  if (delta > 0)
  {
    acelera(valor_acelera, 100);
  }
  // mais a esquerda
  else if (delta < 0)
  {
    acelera(100, valor_acelera);
  }
  else
  {
    acelera(100, 100);
  }
}

void troca_sensor(long smoothedDistanceRight, long smoothedDistanceLeft)
{

}

void debug(){
  
}


void loop()
{
  while (true)
  {
    ler_sensores();
    ajuste_ajustado();
    printf("deu certo\n");
  }
}