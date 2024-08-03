#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Ultrasonic.h>
#include <PID_v1.h>

#define TRIGD A4 // Pino Trig Sensor Direita
#define ECHOD A5 // Pino Echo Sensor Direita

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

SoftwareSerial bluetooth(8, 9); // rx, tx amarelo e verde respectivamente

Ultrasonic sensorD(TRIGD, ECHOD);
Ultrasonic sensorE(TRIGE, ECHOE);
Ultrasonic sensorC(TRIGC, ECHOC);

double OutputC;
double OutputE;
double OutputD;

double Outputdelta;

double SetpointCentral = 20;
double SetpointLaterais = 7.1;

//
double kpCentral = 5.0, kiCentral = 3.5, kdCentral = 2.0;
double kpLateral = 4.0, kiLateral = 100000000.0, kdLateral = 0.0;

int vel = 100;

float MAX_DELTA = 28.0;

float MAX_VOLTAGE = 70.0;
float MIN_PERCENT = 45.0;

// Variáveis Globais
double distanciaC;
double distanciaE; // distancia para leitura do sensor esquerdodouble distanciaC; // distancia para leitura do sensor central
double distanciaD; // distancia para leitura do sensor direita

double distanciaC_abs;
double distanciaE_abs; // distancia para leitura do sensor esquerdodouble distanciaC; // distancia para leitura do sensor central
double distanciaD_abs; // distancia para leitura do sensor direita
double delta;
double delta_abs;

PID PIDc(&distanciaC, &OutputC, &SetpointCentral, kpCentral, kiCentral, kdCentral, REVERSE);
PID PIDd(&distanciaD_abs, &OutputD, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);
PID PIDe(&distanciaE_abs, &OutputE, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);
PID PIDdelta(&delta_abs, &Outputdelta, &SetpointLaterais, kpLateral, kiLateral, kdLateral, DIRECT);

#define N 20 // número de pontos da média móvel

// unsigned pulseEcho_Center = 0, pulseEcho_Left = 0, pulseEcho_Right = 0;    // armazena pulso de saída do sensor
// unsigned pulseInput_Center = 0, pulseInput_Left = 0, pulseInput_Right = 0; // armazena pulso filtrado
unsigned values_center[N], values_left[N], values_right[N]; // vetor para média móvel

long moving_average(unsigned p_In, unsigned *values) // retorna a média móvel de acordo com a resolução designada
{
  int i;          // variável para iterações
  long adder = 0; // variável para somatório

  for (i = N; i > 0; i--) // desloca todo vetor descartando o elemento mais antigo
    values[i] = values[i - 1];

  values[0] = p_In; // o primeiro elemento do vetor recebe o valor do pulso

  for (i = 0; i < N; i++) // faz a somatória
    adder = adder + values[i];

  return adder / N; // retorna a média
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

void ler_sensores()
{
  long microsec = sensorE.timing();
  distanciaE = min(MAX_DELTA + 1.6, sensorE.convert(microsec, Ultrasonic::CM)); // filtro(sensorE.convert(microsec, Ultrasonic::CM), distanciaE, 1.0);
  distanciaE = max(0, distanciaE - 1.6);
  distanciaE = moving_average(distanciaE, values_left);

  microsec = sensorD.timing();
  distanciaD = min(MAX_DELTA + 1.5, sensorD.convert(microsec, Ultrasonic::CM)); // filtro(sensorD.convert(microsec, Ultrasonic::CM), distanciaD, 1.0);
  distanciaD = max(0, distanciaD - 1.5);
  distanciaD = moving_average(distanciaD, values_right);

  microsec = sensorC.timing();
  distanciaC = min(40, sensorC.convert(microsec, Ultrasonic::CM)); // filtro(sensorC.convert(microsec, Ultrasonic::CM), distanciaC, 1.0);
  distanciaC = moving_average(distanciaC, values_center);

  delta = distanciaE - distanciaD;
  delta_abs = abs(delta);
}

void ajuste(float referencia, float valor_acelera)
{

  Serial.print("Valor acelera: ");
  Serial.println(valor_acelera);

  valor_acelera *= OutputC / MAX_VOLTAGE;

  Serial.print("Valor acelera POS CALCULO: ");
  Serial.println(valor_acelera);

  // mais a direita
  Serial.print("Referencia: ");
  Serial.println(referencia);

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

void funcoes_math(double x, double *output)
{
  *output = cos((x) * acos(45/70)/7.1)*70;
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
  PIDe.SetTunings(kpLateral, kiLateral, kdLateral);
  PIDd.SetTunings(kpLateral, kiLateral, kdLateral);
  PIDc.SetTunings(kpCentral, kiCentral, kdCentral);
  PIDdelta.SetTunings(kpLateral, kiLateral, kdLateral);

  PIDc.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);
  PIDe.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);
  PIDd.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);
  PIDdelta.SetOutputLimits(MIN_PERCENT, MAX_VOLTAGE);

  ler_sensores();
  delay(2000);
}

void loop()
{
  while (true)
  {
    ler_sensores();
    imprimeDistancias();

    distanciaD_abs = abs(distanciaD - 6.75);
    distanciaE_abs = abs(distanciaE - 6.75);
    PIDc.Compute();
    Serial.print("Output Central: ");
    Serial.println(OutputC);
    Serial.print("Distancia Esquerda Absoluta: ");
    Serial.println(distanciaE_abs);
    PIDe.Compute();
    Serial.print("Output Esquerda: ");
    Serial.println(OutputE);
    Serial.print("Distancia Direita Absoluta: ");
    Serial.println(distanciaD_abs);
    PIDd.Compute();
    Serial.print("Output Direita: ");
    Serial.println(OutputD);
    PIDdelta.Compute();
    // ajuste(0,100);
    // acelera(100,100);
    move(1);
    // delay(2000);
  }
}