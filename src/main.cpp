#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Ultrasonic.h>
#include <PID_v1.h>

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

double DIS_MAX = 7.1;
double DIS_MIN = 0.1;
float MAX_VOLTAGE = 70.0;
float MIN_VOLTAGE = 55.0;

int SampleTime = 100;

double OutputC;
double OutputE;
double OutputD;

double Outputdelta;

double SetpointCentral = 20;
double SetpointLaterais = DIS_MAX;

// constantes para o PID
double kpCentral = 5.0, kiCentral = 3.5, kdCentral = 2.0;
double kpLateral = 0.7222941748096342, kiLateral = 1832.1658871947106, kdLateral = 916.0829435973553;
//double kpLateral = 8.0, kiLateral = 0.0, kdLateral = 0.0;

float MAX_DELTA = 28.0;



// Variáveis Globais
double distanciaC;
double distanciaE; // distancia para leitura do sensor esquerdodouble distanciaC; // distancia para leitura do sensor central
double distanciaD; // distancia para leitura do sensor direita

double distanciaC_abs;
double distanciaE_abs; // distancia para leitura do sensor esquerdodouble distanciaC; // distancia para leitura do sensor central
double distanciaD_abs; // distancia para leitura do sensor direita

double delta;
double delta_abs;

// int qnt_leituras = 0;

PID PIDc(&distanciaC, &OutputC, &SetpointCentral, kpCentral, kiCentral, kdCentral, REVERSE);
PID PIDd(&distanciaD_abs, &OutputD, &SetpointLaterais, kpLateral, kiLateral, kdLateral, REVERSE);
PID PIDe(&distanciaE_abs, &OutputE, &SetpointLaterais, kpLateral, kiLateral, kdLateral, REVERSE);
PID PIDdelta(&delta_abs, &Outputdelta, &SetpointLaterais, kpLateral, kiLateral, kdLateral, REVERSE);

#define N 5 // número de pontos da média móvel

double values_center[N], values_left[N], values_right[N]; // vetor para média móvel

void moving_average() // retorna a média móvel de acordo com a resolução designada
{
  int i;              // variável para iterações
  double adder_c = 0; // variável para somatório
  double adder_d = 0; // variável para somatório
  double adder_e = 0; // variável para somatório

  for (i = N - 1; i > 0; i--) // desloca todo vetor descartando o elemento mais antigo
  {
    values_center[i] = values_center[i - 1];
    values_left[i] = values_left[i - 1];
    values_right[i] = values_right[i - 1];
  }

  values_center[0] = distanciaC; // o primeiro elemento do vetor recebe o valor do pulso
  values_left[0] = distanciaE;   // o primeiro elemento do vetor recebe o valor do pulso
  values_right[0] = distanciaD;  // o primeiro elemento do vetor recebe o valor do pulso

  for (i = 0; i < N; i++) // faz a somatória
  {
    adder_c = adder_c + values_center[i];
    adder_e = adder_e + values_left[i];
    adder_d = adder_d + values_right[i];
  }

  distanciaC = adder_c / N; // retorna a média
  distanciaE = adder_e / N; // retorna a média
  distanciaD = adder_d / N; // retorna a média
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

  // bluetooth.print("E: ");
  // bluetooth.println(distanciaE);
  // bluetooth.print("D: ");
  // bluetooth.println(distanciaD);
  // bluetooth.print("C: ");
  // bluetooth.println(distanciaC);
}

void acelera(float vel_esquerda, float vel_direita)
{

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

void ler_sensores(int first = 0)
{
  long microsec = sensorE.timing();
  distanciaE = min(MAX_DELTA + 1.6, sensorE.convert(microsec, Ultrasonic::CM)); // filtro(sensorE.convert(microsec, Ultrasonic::CM), distanciaE, 1.0);
  distanciaE = max(0, distanciaE - 1.6);

  microsec = sensorD.timing();
  distanciaD = min(MAX_DELTA + 1.5, sensorD.convert(microsec, Ultrasonic::CM)); // filtro(sensorD.convert(microsec, Ultrasonic::CM), distanciaD, 1.0);
  distanciaD = max(0, distanciaD - 1.5);

  microsec = sensorC.timing();
  distanciaC = min(40, sensorC.convert(microsec, Ultrasonic::CM)); // filtro(sensorC.convert(microsec, Ultrasonic::CM), distanciaC, 1.0);

  // if (first)
  // {
  //   for (int i = 0; i < N; i++)
  //   {
  //     values_center[i] = distanciaC;
  //     values_left[i] = distanciaE;
  //     values_right[i] = distanciaD;
  //   }
  // }

  // moving_average();
 
  delta = distanciaE - distanciaD;
  delta_abs = abs(delta);

  // qnt_leituras++;
}

void ajuste(float referencia, float valor_acelera)
{

  // Serial.print("Valor acelera: ");
  // Serial.println(valor_acelera);

  // valor_acelera *= OutputC / MAX_VOLTAGE;

  // Serial.print("Valor acelera POS CALCULO: ");
  // Serial.println(valor_acelera);

  // mais a direita
  // Serial.print("Referencia: ");
  // Serial.println(referencia);

  if (referencia > 0)
  {
    acelera(valor_acelera, MAX_VOLTAGE);
  }
  // mais a esquerda
  else if (referencia < 0)
  {
    acelera(MAX_VOLTAGE, valor_acelera);
  }
  else
  {
    acelera(valor_acelera, valor_acelera);
  }
}

void move()
{
  // delta -15 MIN   OU 15 MAX
  if (delta < 15 && delta > -15)
  {
    ajuste(delta, Outputdelta);
  }
  else
  {
  // define o sensor de referencia
    if (distanciaD < distanciaE)
    {
      distanciaD = 6.75 - distanciaD;
      ajuste(distanciaD, OutputD);
    }
    else
    {
      distanciaE = distanciaE - 6.75;
      ajuste(distanciaE, OutputE);
    }
  }
}

double output_func_math(int choice, double referencia)
{
  double c;
  double a;
  switch (choice)
  {
  //cosseno
  case 0:
    return cos(referencia * acos(MIN_VOLTAGE / MAX_VOLTAGE) / DIS_MAX) * MAX_VOLTAGE;
    //quadratica
  case 1:
    c = MAX_VOLTAGE;
    a = (MIN_VOLTAGE - MAX_VOLTAGE) / DIS_MAX * DIS_MAX;
  
    return referencia * referencia * a + c;
  default:
    return -1;
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
  pinMode(POTK, INPUT);

  PIDe.SetSampleTime(SampleTime);
  PIDc.SetSampleTime(SampleTime);
  PIDd.SetSampleTime(SampleTime);
  PIDdelta.SetSampleTime(SampleTime);

  // Turn the PID on
  PIDe.SetMode(AUTOMATIC);
  PIDc.SetMode(AUTOMATIC);
  PIDd.SetMode(AUTOMATIC);
  PIDdelta.SetMode(AUTOMATIC);

  // Adjust PID values
  PIDe.SetTunings(kpLateral, kiLateral, kdLateral);
  PIDd.SetTunings(kpLateral, kiLateral, kdLateral);
  PIDc.SetTunings(kpCentral, kiCentral, kdCentral);
  PIDdelta.SetTunings(kpLateral, kiLateral, kdLateral);

  PIDc.SetOutputLimits(MIN_VOLTAGE, MAX_VOLTAGE);
  PIDe.SetOutputLimits(MIN_VOLTAGE, MAX_VOLTAGE);
  PIDd.SetOutputLimits(MIN_VOLTAGE, MAX_VOLTAGE);
  PIDdelta.SetOutputLimits(MIN_VOLTAGE, MAX_VOLTAGE);

  ler_sensores(1);

  // delay(2000);
}

void loop()
{
  // unsigned long now;

  while (true)
  {
    // now = millis();
    ler_sensores();
    // imprimeDistancias();
    // Serial.print("Quantidade de Leituras: ");
    // Serial.println(qnt_leituras);

    distanciaD_abs = abs(distanciaD - 6.75);
    distanciaE_abs = abs(distanciaE - 6.75);

    // // Kp vai de 0 a 30
    // kpLateral = (analogRead(POTK) / 1023.0) * 30.0;
    // // // Ki vai de 2 a 0
    // // kiLateral = (1.0 - analogRead(POTK) / 1023.0) * 2.0;
    // // // Kd vai de 0 a 0.5
    // // kdLateral = analogRead(POTK) / 1023.0 * 0.5;

    // Serial.print("kpLateral: ");
    // Serial.println(kpLateral);

    PIDc.Compute();
    // Serial.print("Output Central: ");
    // Serial.println(OutputC);

    // Serial.print("Distancia Esquerda Absoluta: ");
    // Serial.println(distanciaE_abs);
    PIDe.Compute();
    // Serial.print("Output Esquerda: ");
    // Serial.println(OutputE);

    // Serial.print("Distancia Direita Absoluta: ");
    // Serial.println(distanciaD_abs);
    PIDd.Compute();
    // Serial.print("Output Direita: ");
    // Serial.println(OutputD);

    PIDdelta.Compute();

    // Serial.print("Time: ");
    // Serial.println(now);

    // Serial.print("Input: ");
    // Serial.println(delta);

    // Serial.print("Output: ");
    // Serial.println(Outputdelta);

    Outputdelta = output_func_math(1, delta_abs);
    ajuste(delta,Outputdelta);
    // acelera(100,100);
    // move();
    // delay(2000);
  }
}
