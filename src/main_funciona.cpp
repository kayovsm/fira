#include <Arduino.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>

#define TRIGD 9  // Pino Trig Sensor Direita
#define ECHOD A4 // Pino Echo Sensor Direita

#define TRIGC A0 // Pino Trig Sensor Centro
#define ECHOC A1 // Pino Echo Sensor Centro

#define TRIGE A2 // Pino Trig Sensor Esquerda
#define ECHOE A3 // Pino Echo Sensor Esquerda

#define ENA 5 // ENA PWM Motor Esquerdo
#define ENB 6 // ENB PWM Motor Direito

#define IN1 2 // DIR Motor Esquerdo
#define IN2 4 // DIR Motor Esquerdo

#define IN3 7 // DIR Motor Direito
#define IN4 8 // DIR Motor Direito

#define POTK A5 // Pino potenciômetro

SoftwareSerial bluetooth(8, 9); // rx, tx amarelo e verde respectivamente

Ultrasonic sensorD(TRIGD, ECHOD);
Ultrasonic sensorE(TRIGE, ECHOE);
Ultrasonic sensorC(TRIGC, ECHOC);

// Variáveis Globais
float distanciaE;
double distanciaC;
float distanciaD;

float speed = 1.0; // throttle in % percent
unsigned long time;
/*Parâmetros para ajustar*/

// Valor máximo 255 para potência total
// float VEL_MAX = 90;

/* Ajuste de alinhamento em reta */

float MAX_DELTA = 40;

float MAX_VOLTAGE = 87;
float MIN_VOLTAGE_ESQ = 50;
float MIN_VOLTAGE_DIR = 30;

double DIS_MAX = 7.1;
double DIS_MIN = 0.1;

// PID PARA O SENSOR CENTRAL ((DES)ACELERAÇÃO)
double OutputC;
double SetpointCentral = 20;
double kpCentral = 5.0, kiCentral = 3.5, kdCentral = 2.0;
PID PIDc(&distanciaC, &OutputC, &SetpointCentral, kpCentral, kiCentral,
         kdCentral, REVERSE);

void ler_sensores()
{
  long microsec = sensorE.timing();
  distanciaE =
      min(MAX_DELTA + 1.6,
          sensorE.convert(
              microsec, Ultrasonic::CM)); // filtro(sensorE.convert(microsec,
                                          // Ultrasonic::CM), distanciaE, 1.0);
  distanciaE = max(0, distanciaE - 1.6);

  microsec = sensorD.timing();
  distanciaD = min(
      MAX_DELTA + 1.5,
      sensorD.convert(microsec,
                      Ultrasonic::CM)); // filtro(sensorD.convert(microsec,
                                        // Ultrasonic::CM), distanciaD, 1.0);
  distanciaD = max(0, distanciaD - 1.5);

  microsec = sensorC.timing();
  distanciaC =
      min(MAX_DELTA,
          sensorC.convert(
              microsec, Ultrasonic::CM)); // filtro(sensorC.convert(microsec,
                                          // Ultrasonic::CM), distanciaC, 1.0);
}

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
}

float tratamento(float vel, float MIN_VOLTAGE)
{
  if (vel > MAX_VOLTAGE)
  {
    vel = MAX_VOLTAGE;
  }
  if (vel < MIN_VOLTAGE)
  {
    vel = MIN_VOLTAGE;
  }
  vel = (vel)*MAX_VOLTAGE / 100;
  return vel;
}

void acelera(float vel_esquerda, float vel_direita)
{
  int vel_direita_int = ceil(tratamento(vel_direita, MIN_VOLTAGE_DIR));
  int vel_esquerda_int = ceil(tratamento(vel_esquerda, MIN_VOLTAGE_ESQ));

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, vel_esquerda_int);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, vel_direita_int);
}

void setup()
{
  Serial.begin(9600); // Comunicação Serial com o Computador
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
  pinMode(POTK, INPUT);

  // PIDc.SetMode(AUTOMATIC);
  // PIDc.SetTunings(kpCentral, kiCentral, kdCentral);
  // PIDc.SetOutputLimits(MIN_VOLTAGE, MAX_VOLTAGE);

  ler_sensores();
  delay(2000);
  time = millis();
  // last_time = time;
}

// unsigned long time_here_left = 0;
// unsigned long time_here_right = 0;
// unsigned long last_time = 0;

double output_func_math(int choice, float MIN_VOLTAGE, double referencia,
                        double referencia_2 = 0)
{
  double c;
  double a;
  switch (choice)
  {
  // cosseno
  case 0:
    return cos(referencia * acos(MIN_VOLTAGE / MAX_VOLTAGE) / DIS_MAX) *
           MAX_VOLTAGE;

  // quadratica
  case 1:
    c = MAX_VOLTAGE;
    a = (MIN_VOLTAGE - MAX_VOLTAGE) / DIS_MAX * DIS_MAX;
    return referencia * referencia * a + c;

  // linear
  case 2:
    // valores iniciais a = -50.0 e c = 15.5
    a = -70;
    c = 16;
    return ceil((a / c) * referencia + 100.0);

  // 3 variaveis, varia em relacao ao tempo
  case 3:
    c = MAX_VOLTAGE;
    a = (MIN_VOLTAGE - MAX_VOLTAGE) / DIS_MAX * DIS_MAX;
    if (a < 1)
    {
      return referencia * referencia * a * referencia_2 + c;
    }
    else
    {
      return referencia * referencia * a / referencia_2 + c;
    }

  // erro
  default:
    return -1;
  }
}

void ajuste(float delta)
{
  // variável que calcula o quanto uma roda deverá diminuir para ajustar o
  // carrinho
  double valor_acelera;

  // CONTROLE PID
  // valor_acelera *= OutputC / MAX_VOLTAGE;

  // float valor_acelera =
  //  mais a direita
  if (delta > 0)
  {
    valor_acelera = output_func_math(2, MIN_VOLTAGE_ESQ, abs(delta));
    valor_acelera = min(valor_acelera, 100);
    valor_acelera = max(valor_acelera, 0);
    // time_here_right += (millis() - last_time);
    // time_here_left = 0;
    //  digitalWrite(led_amarelo_esquerda, HIGH);
    //  digitalWrite(led_azul_direita, LOW);
    acelera(valor_acelera, 100);
  }

  // mais a esquerda
  else if (delta < 0)
  {
    valor_acelera = output_func_math(2, MIN_VOLTAGE_DIR, (delta));
    valor_acelera = min(valor_acelera, 100);
    valor_acelera = max(valor_acelera, 0);
    // time_here_left += (millis() - last_time);
    // time_here_right = 0;
    //  digitalWrite(led_amarelo_esquerda, LOW);
    //  digitalWrite(led_azul_direita, HIGH);
    acelera(100, valor_acelera*0.20);
  }
  else
  {
    // time_here_left = 0;
    // time_here_right = 0;
    //  digitalWrite(led_amarelo_esquerda, LOW);
    //  digitalWrite(led_azul_direita, LOW);
    acelera(100, 100);
  }
  // last_time = millis();
}

// THROTTLE COM FUNÇÃO QUADRATICA QUE NAO SABEMOS COMO, MAS FUNCIONA
void throttle()
{
  float a = 2.0 / 18.0;
  // float b = 2.0/3.0;
  // float valorC = ((a * distanciaC * distanciaC) - (b * distanciaC))/100;
  float valorC = (distanciaC * distanciaC * a);
  unsigned long delta_time = millis() - time;
  // int delta = distanciaE - distanciaD;

  float step = 0.0;
  if (delta_time > 50)
  {
    time = millis();
    step = sqrt(speed / 100.0);
    step += 0.002;
    step = min(step, 1.0);

    speed = max(0.35, (step * step));

    speed = ceil(min(100, max(0, (speed * 100))));

    MAX_VOLTAGE = max(0.35, valorC * speed);
    MAX_VOLTAGE = min(MAX_VOLTAGE * 100, 100);
    // analogWrite(IN3, 0);
    // analogWrite(IN4, 0);
    // analogWrite(ENB, valorC * speed); // direita
    // analogWrite(INB, speed);

    // analogWrite(IN1, 0);
    // analogWrite(IN2, 0);
    // analogWrite(ENA, max(0.35, valorC * speed));
    // analogWrite(INA, speed);

    Serial.print("SPEED: ");
    Serial.println(speed);
    Serial.print("MAX: ");
    Serial.println(MAX_VOLTAGE);
  }
}

void loop()
{
  while (true)
  {
    ler_sensores();
    PIDc.Compute();
    // throttle();
    float delta = distanciaE - distanciaD;
    ajuste(delta);
  }
}