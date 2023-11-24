#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>

#define TRIGD A0 // Pino Trig Sensor Direito
#define ECHOD A1 // Pino Echo Sensor Direito
#define TRIGE A2 // Pino Trig Sensor Esquerdo
#define ECHOE A3 // Pino Echo Sensor Esquerdo
#define TRIGC A4 // Pino Trig Sensor Centro
#define ECHOC A5 // Pino Echo Sensor Centro
#define ENA 5    // ENA PWM Motor Esquerdo
#define ENB 6    // ENB PWM Motor Direito
#define IN1 3    // DIR Motor Esquerdo
#define IN2 9    // DIR Motor Esquerdo
#define IN3 10   // DIR Motor Direito
#define IN4 11   // DIR Motor Direito

#define led_amarelo_esquerda 2
#define led_azul_direita 7

Ultrasonic sensorD(A0, A1);
Ultrasonic sensorE(A2, A3); 
Ultrasonic sensorC(A4, A5);
// porta botao 4

SoftwareSerial bluetooth(13, 12); // rx, tx amarelo e verde respectivamente

// Variáveis Globais
float distanciaE;
float distanciaC;
float distanciaD;
int vel = 100;

float speed = 1.0; // throttle in % percent
unsigned long time;
/*Parâmetros para ajustar*/

// Valor máximo 255 para potência total
// float VEL_MAX = 90;

/* Ajuste de alinhamento em reta */

float MAX_DELTA = 40;

float MAX = 100;
float MIN = 85;

// // Distancia em cm de tolerância no alinhamento em reta
// float TOLERANCIA_AJUSTE = 1;

// // Fator para corrigir o alinhamento (quanto maior mais intensamente ele alinha)
// float INTENSIDADE_AJUSTE = 2;

// /* Ajuste de curvas **/

// // Distancia máxima em cm da parede lateral para iniciar a curva
// float TOLERANCIA_CURVA = 10;

// // Distância tolerável para o sensor da frente
// float DIST_FRENTE_MIN = 5;

// // Distância em cm para o sensor da frente iniciar a curva
// float DIST_FRENTE_MAX = 25;

float filtro(float atual, float anterior, float alpha)
{
  float res = alpha * atual + (1 - alpha) * anterior;
  return res;
}

void ler_sensores()
{
  long microsec = sensorE.timing();
  distanciaE = min(MAX_DELTA+3, sensorE.convert(microsec, Ultrasonic::CM)); // filtro(sensorE.convert(microsec, Ultrasonic::CM), distanciaE, 1.0);
  distanciaE = max(0, distanciaE-3);

  microsec = sensorD.timing();
  distanciaD = min(MAX_DELTA+3, sensorD.convert(microsec, Ultrasonic::CM)); // filtro(sensorD.convert(microsec, Ultrasonic::CM), distanciaD, 1.0);
  distanciaD = max(0, distanciaD-3);

  microsec = sensorC.timing();
  distanciaC = min(MAX_DELTA, sensorC.convert(microsec, Ultrasonic::CM)); // filtro(sensorC.convert(microsec, Ultrasonic::CM), distanciaC, 1.0);
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

  bluetooth.print("E: ");
  bluetooth.println(distanciaE);
  bluetooth.print("D: ");
  bluetooth.println(distanciaD);
  bluetooth.print("C: ");
  bluetooth.println(distanciaC);
}

float tratamento(float vel)
{
  if(vel > 100)
  {
    vel = 100;
  }
  if(vel < 0)
  {
    vel = MIN;
  }
  vel = (vel)*MAX/100;
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
  // pinMode(BOTAO, INPUT_PULLUP);
  // acelera(VEL_MAX, VEL_MAX);
  // Serial.end();
  // delay(5000);
  ler_sensores();
  delay(2000);
  time = millis();
}

void ajuste_ajustado(float delta)
{
  //variável que calcula o quanto uma roda deverá diminuir para ajustar o carrinho
  float valor_acelera = ceil((-50.0/15.5)*abs(delta) + 100.0);
  // float valor_acelera = ceil(10.451613*(abs(delta)+MAX)*(abs(delta)+MAX)-2255.54839*(abs(delta)+MAX)+121138.7097);

  valor_acelera = min(valor_acelera, 100);
  valor_acelera = max(valor_acelera, 0);

  //mais a direita
  if(delta > 0)
  {
    digitalWrite(led_amarelo_esquerda, HIGH);
    digitalWrite(led_azul_direita, LOW);
    acelera(valor_acelera, 100);
  }

  //mais a esquerda
  else if(delta < 0)
  {
    digitalWrite(led_amarelo_esquerda, LOW);
    digitalWrite(led_azul_direita, HIGH);
    acelera(100, valor_acelera);
  }
  else
  {
    digitalWrite(led_amarelo_esquerda, LOW);
    digitalWrite(led_azul_direita, LOW);
    acelera(100, 100);
  }
}

// VERSÃO FEITA ONTEM DE TARDE 21/11
void throttle()
{
    float a = 2.0/15.0;
    float b = 2.0/3.0;
    float valorC = ((a * distanciaC * distanciaC) - (b * distanciaC))/100;
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

        MAX = max(0.35, valorC * speed);
        MAX = min(MAX*100, 100);
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
        Serial.println(MAX);
        
    }
}


// VERSÃO FEITA 13:00 DE 20/11
// void throttle()
// {
//   float MAX=100;
//   unsigned long delta_time = millis() - time;
//   float valorC = ((2/15)*distanciaC*distanciaC - (2/3)*distanciaC)/100;
//   float step = 0.0;
//   if (delta_time > 50)
//   {
//     time = millis();
//     step = sqrt(speed / MAX);
//     step += 0.01;
//     step = min(step, 1.0);

//     speed = max(0.2, (step * step));

//     speed = ceil(min(MAX, max(0, (speed * MAX))));

//     // analogWrite(IN3, 0);
//     // analogWrite(IN4, 0);
//     analogWrite(ENB, speed);
//     // analogWrite(INB, speed);

//     // analogWrite(IN1, 0);
//     // analogWrite(IN2, 0);
//     analogWrite(ENA, speed);
//     // analogWrite(INA, speed);

//     Serial.print("SPEED: ");
//     Serial.println(speed);
//   }
// }

void loop()
{
  while(true)
  {  
    ler_sensores();
    throttle();
    float delta = distanciaE - distanciaD;
    bluetooth.print("Delta: ");
    bluetooth.println(delta);
    bluetooth.print("Centro: ");
    bluetooth.println(distanciaC);
    ajuste_ajustado(delta);
    // distanciaC = 30;
    // ler_sensores();
    // imprimeDistancias();
    // throttle();
    // acelera(100, 100);
    // delay(2000);
    // acelera(37.3,100);
    // acelera(70,70);
    // acelera(100,30);
    // delay(500);
    // while(true)
    // {
    //   acelera(100, 100);
    // }
  }


  // throttle();
  // ajuste_ajustado();
}
