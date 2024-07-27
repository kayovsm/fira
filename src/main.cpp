#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>
#include <math.h>

#define TRIGD A0 // Pino Trig Sensor Direito
#define ECHOD A1 // Pino Echo Sensor Direito

#define TRIGE A2 // Pino Trig Sensor Esquerdo
#define ECHOE A3 // Pino Echo Sensor Esquerdo

#define TRIGC A4 // Pino Trig Sensor Centro
#define ECHOC A5 // Pino Echo Sensor Centro

#define ENA 5  // ENA PWM Motor Esquerdo
#define ENB 6  // ENB PWM Motor Direito
#define IN1 3  // DIR Motor Esquerdo
#define IN2 9  // DIR Motor Esquerdo
#define IN3 10 // DIR Motor Direito
#define IN4 11 // DIR Motor Direito

Ultrasonic sensorD(A0, A1);
Ultrasonic sensorE(A2, A3);
Ultrasonic sensorC(A4, A5);
// porta botao 4

SoftwareSerial bluetooth(8, 9); // rx, tx amarelo e verde respectivamente

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

void imprimeDistancias()
{
  Serial.print("Dis Esq: ");
  Serial.print(distanciaE);
  Serial.print(" cm  /  ");
  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" cm   /  ");
  Serial.print("Dis Dir: ");
  Serial.print(distanciaD);
  Serial.println(" cm");

  bluetooth.print("E: ");
  bluetooth.println(distanciaE);
  bluetooth.print("D: ");
  bluetooth.println(distanciaD);
  bluetooth.print("C: ");
  bluetooth.println(distanciaC);
}

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

  analogWrite(IN3, vel);
  analogWrite(IN4, 0);
  analogWrite(ENB, vel_direita_int);

  analogWrite(IN1, vel);
  analogWrite(IN2, 0);
  analogWrite(ENA, vel_esquerda_int);

  // Serial.print("VD: ");
  // Serial.println(vel_direita);
  // Serial.print("VE: ");
  // Serial.println(vel_esquerda);
}

const int windowSize = 100;

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

float funcao_laterais(float leitura)
{
  float a = acos(0.35) / 100;
  float b = 100;
  return cos(leitura * a) * b;
}

float funcao_frontal(float leitura)
{
  float b = 73 / 3315;
  float a = (1 - 30 * b) / 900;
  return (leitura * leitura * a) + (leitura * b);
}

void ajuste(float referencia, float valor_acelera)
{
  valor_acelera *= funcao_frontal(distanciaC);

  // mais a direita
  if (referencia > 0)
  {
    acelera(valor_acelera, 100);
  }
  // mais a esquerda
  else if (referencia < 0)
  {
    acelera(100, valor_acelera);
  }
  else
  {
    acelera(valor_acelera, valor_acelera);
  }
}

void move()
{
  // define o sensor de referencia
  if (delta < 15 && delta > -15)
  {
    ajuste(delta, funcao_laterais(delta));
  }
  else
  {
    if (distanciaD < distanciaE)
    {
      ajuste(6.75 - distanciaD, funcao_laterais(distanciaD));
    }
    else
    {
      ajuste(distanciaE - 6.75, funcao_laterais(distanciaE));
    }
  }
  // fazer outra função

  // desacelera
}

void loop()
{
  while (true)
  {
    ler_sensores();
    imprimeDistancias();
    move();
  }
}