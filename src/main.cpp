#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
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
#define IN1 3  // DIR Motor Esquerdo
#define IN2 9  // DIR Motor Esquerdo
#define IN3 10 // DIR Motor Direito
#define IN4 11 // DIR Motor Direito

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

// unsigned long time;

float MAX_DELTA = 28;

float MAX_VOLTAGE_M1 = 80;
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
  // bluetooth.print("C: ");
  // bluetooth.println(distanciaC);
}

void acelera(float vel_esquerda, float vel_direita)
{

  int vel_direita_int = ceil(vel_direita);
  int vel_esquerda_int = ceil(vel_esquerda);

  analogWrite(IN3, vel);
  analogWrite(IN4, 0);
  analogWrite(ENB, vel_direita_int);

  analogWrite(IN1, vel);
  analogWrite(IN2, 0);
  analogWrite(ENA, vel_esquerda_int);
}

// =======================================================================================================
// --- Gera o pulso de trigger para o acionamento do sinal de ultrassom ---
// Pulso de 10µs , conforme especificação do fabricante (vide datashet HC-SR04)
//
void trigPulse(const uint8_t trig)                                  //Função para gerar o pulso de trigger para o sensor HC-SR04
{
  
   digitalWrite(trig,HIGH);                       //Saída de trigger em nível alto
   delayMicroseconds(10);                         //Por 10µs ...
   digitalWrite(trig,LOW);                        //Saída de trigger volta a nível baixo

} //end trigPulse

float measureDistance(const uint8_t trig, const uint8_t echo)                           //Função que retorna a distância em centímetros
{

  float pulse;                                    //Armazena o valor de tempo em µs que o pino echo fica em nível alto
        
  trigPulse(trig);                                    //Envia pulso de 10µs para o pino de trigger do sensor
  
  pulse = pulseIn(echo, HIGH);                    //Mede o tempo em que echo fica em nível alto e armazena em pulse
    
  /*
    >>> Cálculo da Conversão de µs para cm:
    
   Velocidade do som = 340 m/s = 34000 cm/s
   
   1 segundo = 1000000 micro segundos
   
      1000000 µs - 34000 cm/s
            X µs - 1 cm
            
                  1E6
            X = ------- = 29.41
                 34000
                 
    Para compensar o ECHO (ida e volta do ultrassom) multiplica-se por 2
    
    X' = 29.41 x 2 = 58.82
 
  */
  
  return (pulse/58.82);                           //Calcula distância em centímetros e retorna o valor
  
  
} //end measureDistante

void ler_sensores()
{
  distanciaE = min(MAX_DELTA + 2, measureDistance(TRIGE, ECHOE)); 
  distanciaE = max(0, distanciaE - 2);

  distanciaD = min(MAX_DELTA + 1.5,  measureDistance(TRIGD, ECHOD)); 
  distanciaD = max(0, distanciaD - 1.5);

  distanciaC = min(MAX_DELTA,  measureDistance(TRIGC, ECHOC)); 
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

  PIDc.SetOutputLimits(0, MAX_VOLTAGE_M1);

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
    PIDe.Compute();
    PIDd.Compute();
    PIDdelta.Compute();

    move();
  }
}