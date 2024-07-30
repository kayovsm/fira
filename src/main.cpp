#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Ultrasonic.h>
#include <PID_v1.h>

// #define BTN0 A0 // Botão 0
// #define BTN1 A1 // Botão 1

#define TRIGE A2 // Pino Trig Sensor Esquerdo
#define ECHOE A3 // Pino Echo Sensor Esquerdo

#define TRIGC A4 // Pino Trig Sensor Centro
#define ECHOC A5 // Pino Echo Sensor Centro

#define TRIGD A0 // Pino Trig Sensor Direito
#define ECHOD A1 // Pino Echo Sensor Direito

#define ENA 5  // ENA PWM Motor Esquerdo
#define ENB 6  // ENB PWM Motor Direito

#define IN1 2  // DIR Motor Esquerdo
#define IN2 4  // DIR Motor Esquerdo

#define IN3 7 // DIR Motor Direito
#define IN4 8 // DIR Motor Direito

// porta botao 4

SoftwareSerial bluetooth(8, 9); // rx, tx amarelo e verde respectivamente

Ultrasonic sensorD(A0, A1);
Ultrasonic sensorE(A2, A3);
Ultrasonic sensorC(A4, A5);

double OutputC;
double OutputE;
double OutputD;
double Outputdelta;
double SetpointCentral = 20;
double SetpointLaterais = 6.75;

double kpCentral = 5.0, kiCentral = 3.0, kdCentral = 0;
double kp = 30, ki = 10, kd = 1;

int vel = 100;

// unsigned long time;

float MAX_DELTA = 28.0;

float MAX_VOLTAGE_M1 = 95.0;
float MAX_VOLTAGE_M2 = 100.0;
float MIN_PERCENT = 35.0;

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
  Serial.print("Dis Esq: ");
  Serial.print(distanciaE);
  Serial.print(" cm  /  ");

  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" cm   /  ");

  Serial.print("Dis Dir: ");
  Serial.print(distanciaD);
  Serial.println(" cm");

  // bluetooth.print("E: ");
  // bluetooth.println(distanciaE);
  // bluetooth.print("D: ");
  // bluetooth.println(distanciaD);
  // bluetooth.print("C: ");
  // bluetooth.println(distanciaC);
}


void acelera(float vel_esquerda, float vel_direita)
{

  int vel_direita_int = ceil(vel_direita*0.95);
  int vel_esquerda_int = ceil(vel_esquerda);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, vel_direita_int);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, vel_esquerda_int);
}

void ler_sensores()
{
  long microsec;
  // long microsec = sensorE.timing();
  // distanciaE = min(MAX_DELTA + 2, sensorE.convert(microsec, Ultrasonic::CM)); // filtro(sensorE.convert(microsec, Ultrasonic::CM), distanciaE, 1.0);
  // distanciaE = max(0, distanciaE - 2);

  // microsec = sensorD.timing();
  // distanciaD = min(MAX_DELTA + 1.5, sensorD.convert(microsec, Ultrasonic::CM)); // filtro(sensorD.convert(microsec, Ultrasonic::CM), distanciaD, 1.0);
  // distanciaD = max(0, distanciaD - 1.5);

  microsec = sensorC.timing();
  distanciaC = min(40, sensorC.convert(microsec, Ultrasonic::CM)); // filtro(sensorC.convert(microsec, Ultrasonic::CM), distanciaC, 1.0);

  // updateHistory(distanciaC, distanciaD, distanciaE);

  // long distanciaC = getSmoothedDistanceFront();
  // long distanciaD = getSmoothedDistanceRight();
  // long distanciaE = getSmoothedDistanceLeft();

  // delta = distanciaE - distanciaD;
}

void ajuste(float referencia, float valor_acelera)
{
  valor_acelera *= OutputC/100;
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

  PIDc.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE_M1);
  PIDe.SetOutputLimits(0, MAX_VOLTAGE_M1);
  PIDd.SetOutputLimits(0, MAX_VOLTAGE_M1);
  PIDdelta.SetOutputLimits(0, MAX_VOLTAGE_M1);

  ler_sensores();
  delay(2000);
}

void loop()
{
  while (true)
  {
    ler_sensores();
    imprimeDistancias();
    
    PIDc.Compute();
    // PIDe.Compute();
    // PIDd.Compute();
    // PIDdelta.Compute();
    ajuste(0,100);
    // acelera(100,100);
    // move();
  }
}