#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>
#include <math.h>
#include <PID_v1.h>

#define BTN0 A0 // Botão 0
#define BTN1 A1 // Botão 1

#define TRIGE A2 // Pino Trig Sensor Esquerdo
#define ECHOE A3 // Pino Echo Sensor Esquerdo

#define TRIGC A4 // Pino Trig Sensor Centro
#define ECHOC A5 // Pino Echo Sensor Centro

#define TRIGD A6 // Pino Trig Sensor Direito
#define ECHOD A7 // Pino Echo Sensor Direito

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

double OutputC;
double OutputE;
double OutputD;
double Outputdelta;
double SetpointCentral = 17;
double SetpointLaterais = 6.75;

double kpCentral = 5, kiCentral = 1.5, kdCentral = 0;
double kp = 30, ki = 10, kd = 1;

int vel = 100;

unsigned long time;

float MAX_DELTA = 28;

float MAX_VOLTAGE_M1 = 100;
float MAX_VOLTAGE_M2 = 100;
float MIN_PERCENT = 35;
byte lastButton0State = HIGH;
byte lastButton1State = HIGH;

// Variáveis Globais
double distanciaE; // distancia para leitura do sensor esquerdo
double distanciaC; // distancia para leitura do sensor central
double distanciaD; // distancia para leitura do sensor direita

double delta;

PID PIDc(&distanciaC, &OutputC, &SetpointCentral, kpCentral, kiCentral, kdCentral, REVERSE);
PID PIDd(&distanciaD, &OutputD, &SetpointLaterais, kp, ki, kd, DIRECT);
PID PIDe(&distanciaE, &OutputE, &SetpointLaterais, kp, ki, kd, DIRECT);
PID PIDdelta(&delta, &Outputdelta, &SetpointLaterais, kp, ki, kd, DIRECT);

void imprimeDistancias()
{
  // Serial.print("Dis Esq: ");
  // Serial.print(distanciaE);
  // Serial.print(" cm  /  ");

  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" cm   /  ");

  // Serial.print("Dis Dir: ");
  // Serial.print(distanciaD);
  // Serial.println(" cm");

  // bluetooth.print("E: ");
  // bluetooth.println(distanciaE);
  // bluetooth.print("D: ");
  // bluetooth.println(distanciaD);
  bluetooth.print("C: ");
  bluetooth.println(distanciaC);
}

void acelera(float vel_esquerda, float vel_direita)
{

  int vel_direita_int = ceil(vel_direita * 0.9);
  int vel_esquerda_int = ceil(vel_esquerda);

  Serial.print("vel_direita_int: ");
  Serial.println(vel_direita_int);

  Serial.print("vel_direita_int: ");
  Serial.println(vel_esquerda_int);

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
  if (millis() - time >= 10)
  {
    historyFront[index] = distanceFront;
    historyRight[index] = distanceRight;
    historyLeft[index] = distanceLeft;
    index = (index + 1) % windowSize;
    time = millis();
  }
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
  valor_acelera *= OutputC / 100;
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
    ajuste(delta, Outputdelta);
  }
  else
  {
    if (distanciaD < distanciaE)
    {
      ajuste(6.75 - distanciaD, OutputD);
    }
    else
    {
      ajuste(distanciaE - 6.75, OutputE);
    }
  }
  // fazer outra função

  // desacelera
}

double aumentar_no_k = 0.1;

void button(double *constante)
{
  byte buttonState = digitalRead(BTN0);
  if (buttonState != lastButton0State)
  {
    lastButton0State = buttonState;
    if (buttonState == LOW)
    {
      aumentar_no_k *= 10;
      Serial.print("Valor aumentar_no_k: ");
      Serial.println(aumentar_no_k);
    }
  }

  buttonState = digitalRead(BTN1);
  if (buttonState != lastButton1State)
  {
    lastButton1State = buttonState;
    if (buttonState == LOW)
    {
      (*constante) += aumentar_no_k;
    }
  }
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
  pinMode(BTN0, INPUT_PULLUP);
  pinMode(BTN1, INPUT_PULLUP);

  // Turn the PID on
  PIDe.SetMode(AUTOMATIC);
  PIDc.SetMode(AUTOMATIC);
  PIDd.SetMode(AUTOMATIC);
  PIDdelta.SetMode(AUTOMATIC);
  // Adjust PID values
  PIDe.SetTunings(kp, ki, kd);
  PIDd.SetTunings(kp, ki, kd);
  PIDc.SetTunings(kpCentral, kiCentral, kdCentral);
  PIDdelta.SetTunings(kp, ki, kd);

  PIDc.SetOutputLimits(0, 90);

  ler_sensores();
  delay(2000);
  time = millis();
}

void loop()
{
  while (true)
  {
    ler_sensores();

    PIDc.Compute();
    // PIDe.Compute();
    // PIDd.Compute();
    // PIDdelta.Compute();
    Serial.print("Valor kiCentral: ");
    Serial.println(kiCentral);
    button(&kiCentral);
    // Serial.print("Output Esquerda: ");
    // Serial.println(OutputE);
    // Serial.print("Output Direita: ");
    // Serial.println(OutputD);
    imprimeDistancias();
    // move();
    ajuste(0, 100);
    // delay(5000);
  }
}