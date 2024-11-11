#include "Arduino.h"
#include "VL53L1X.h"
#include "HardwareSerial.h"
#include <math.h>
#include <stdlib.h>
#include <avr/wdt.h>

#define led_branco 2
#define led_vermelho 3
#define led_verde 7
#define led_azul 8

#define IN1 11 // DIR Motor Direito
#define IN2 10 // DIR Motor Direito
#define IN3 5  // DIR Motor Esquerdo
#define IN4 6  // DIR Motor Esquerdo

#define xshutPinsE 12
#define xshutPinsC 9
#define xshutPinsD 4

VL53L1X sensorC;
VL53L1X sensorDF;
VL53L1X sensorDR;

// Variáveis Globais
float tamanho_carrinho = 14;
float tamanho_pista = 30;

double distanciaDF;
double distanciaC;
double distanciaDR;
double delta;
double media;
double DIS_MAX = 7.7;
double dist_sensores = 4.1;
double angle;

unsigned long time;

float MAX_VOLTAGE = 80;
float MIN_VOLTAGE_ESQ = 50;
float MIN_VOLTAGE_DIR = 30;

int direcao[2];

void apaga_led()
{
  digitalWrite(led_azul, LOW);
  digitalWrite(led_branco, LOW);
  digitalWrite(led_verde, LOW);
  digitalWrite(led_vermelho, LOW);
}

void ler_sensores()
{
  distanciaDF = (sensorDF.read() - 20) / 10;
  if (distanciaDF > 400)
  {
    distanciaDF = (sensorDF.read() - 20) / 10;
  }
  sensorDF.timeoutOccurred() ? distanciaDF = 400 : distanciaDF = distanciaDF;

  distanciaDR = (sensorDR.read() - 20) / 10;
  if (distanciaDR > 400)
  {
    distanciaDR = (sensorDR.read() - 20) / 10;
  }
  sensorDR.timeoutOccurred() ? distanciaDR = 400 : distanciaDR = distanciaDR;

  distanciaC = (sensorC.read() - 10) / 10;
  sensorC.timeoutOccurred() ? distanciaC = 400 : distanciaC = distanciaC;

  if (distanciaC > 600)
  {
    distanciaC = 0;
  }
  if (distanciaDR > 600)
  {
    distanciaDR = 0;
  }
  if (distanciaDF > 600)
  {
    distanciaDF = 0;
  }

  media = (distanciaDF + distanciaDR) / 2;
  angle = atan((distanciaDF - distanciaDR) / dist_sensores);
}

void imprimeDistancias()
{
  Serial.print("Dis DF: ");
  Serial.print(distanciaDF);
  Serial.print(" cm  /  ");
  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" cm   /  ");
  Serial.print("Dis DR: ");
  Serial.print(distanciaDR);
  Serial.println(" cm     /   ");
  Serial.print("Média: ");
  Serial.println(media);
  Serial.print("angle: ");
  Serial.println(angle);
}

float tratamento(float vel)
{
  vel = min(vel, 100);
  vel = max(vel, 0);
  vel = (vel)*MAX_VOLTAGE / 100;
  return vel;
}

void re()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  direcao[0] = IN1;
  direcao[1] = IN4;
}

void parar()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  apaga_led();
  direcao[0] = 0;
  direcao[1] = 0;
}

void frente()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  direcao[0] = IN2;
  direcao[1] = IN3;
}

void direita()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  direcao[0] = IN1;
  direcao[1] = IN3;
}

void esquerda()
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  direcao[0] = IN2;
  direcao[1] = IN4;
}

void acelera(float vel_esquerda, float vel_direita)
{
  int vel_direita_int = round(tratamento((vel_direita)));
  int vel_esquerda_int = round(tratamento((vel_esquerda)));
  analogWrite(direcao[1], vel_esquerda_int);
  analogWrite(direcao[0], vel_direita_int);

  // Serial.println(direcao[0]);
  // Serial.println(direcao[1]);
  // Serial.println(vel_esquerda_int);
  // Serial.println(vel_direita_int);
}

void acende_leds()
{
  digitalWrite(led_azul, HIGH);
  digitalWrite(led_branco, HIGH);
  digitalWrite(led_verde, HIGH);
  digitalWrite(led_vermelho, HIGH);
}

void setup()
{
  while (!Serial)
  {
  }

  Serial.begin(115200); // Comunicação Serial com o Computador
  pinMode(IN1, OUTPUT); // definição dos pinos entradas e saidas
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);      // OUTPUT = Saída
  pinMode(IN4, OUTPUT);      // INPUT = Entrada
  pinMode(led_azul, OUTPUT); // definição dos pinos entradas e saidas
  pinMode(led_branco, OUTPUT);
  pinMode(led_verde, OUTPUT); // OUTPUT = Saída
  pinMode(led_vermelho, OUTPUT);
  pinMode(xshutPinsD, OUTPUT);
  digitalWrite(xshutPinsD, LOW);
  pinMode(xshutPinsC, OUTPUT);
  digitalWrite(xshutPinsC, LOW);
  pinMode(xshutPinsE, OUTPUT);
  digitalWrite(xshutPinsE, LOW);
  pinMode(A3, OUTPUT);
  pinMode(A2, INPUT);
  digitalWrite(A3, HIGH);
  pinMode(xshutPinsE, INPUT);
  delay(10);

  digitalWrite(led_branco, HIGH);
  delay(100);
  digitalWrite(led_branco, LOW);

  sensorDF.setTimeout(500);
  if (!sensorDF.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("E");
    acende_leds();
    while (1)
    {
    }
  }
  sensorDF.setAddress(0x2A);
  sensorDF.startContinuous(50);
  pinMode(xshutPinsD, INPUT);
  delay(10);

  sensorDR.setTimeout(500);
  if (!sensorDR.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("D");
    acende_leds();
    while (1)
    {
    }
  }
  sensorDR.setAddress(0x2A + 1);
  sensorDR.startContinuous(50);
  pinMode(xshutPinsC, INPUT);
  delay(10);

  sensorC.setTimeout(500);
  if (!sensorC.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("C");
    acende_leds();
    while (1)
    {
    }
  }
  sensorC.setAddress(0x2A + 2);
  sensorC.startContinuous(50);

  Serial.print("Tamanho da pista: ");
  Serial.println(tamanho_pista);

  time = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  frente();
  acelera(0, 0);
  delay(4000);

  wdt_enable(WDTO_4S);
}

// DF é o mais próximo dos motores, enquanto o DR é o sensor na parte mais ao fundo do carrinho
void loop()
{
  wdt_reset();
  ler_sensores();

  double distanciaMIN = 5;
  double distanciaMAX = 11;

  if (distanciaDF > tamanho_pista) // 5 -> curva pra direita
  {
    // girar pra direita ate encontrar de novo a parede (alkguma leitura)
    // vai pra frente
    direita();
    while (distanciaDF > tamanho_pista)
    {
      acelera(70, 70);
      ler_sensores();
    }
    frente();
    time = millis();
    while (distanciaDF < tamanho_pista && millis() - time <= 150)
    {
      acelera(100, 85);
      ler_sensores();
    }
  }

  if (distanciaC > 8) // se a distanciaC for maior que 6, sei que posso ir pra frente, mas preciso verificar minha distancia pra parede de referencia
  {
    if (media > distanciaMIN && media < distanciaMAX) // corrigir a inclinação com base na média (na realidade, seria com base no angulo ne?)
    {
      frente();
      acelera(100, 85); // isso era pra andar reto, ajustar
      delay(50);
    }

    else if (min(distanciaDF, distanciaDR) <= distanciaMIN || max(distanciaDF, distanciaDR) >= distanciaMAX)
    {
      ler_sensores();
      double original_angle = angle;
      while (original_angle * angle > 0)
      {
        if (angle > 0)
        {
          direita();
          acelera(100, 100);
          delay(75 * abs(angle));
        }
        else
        {
          esquerda();
          acelera(85, 85);
          delay(75 * abs(angle));
        }
        Serial.print("Delay: ");
        Serial.println(100 * abs(angle));
        frente();
        acelera(0, 0);
        ler_sensores();
        imprimeDistancias();
      }
      acelera(100, 85);
      delay(50);
    }
  }
  else
  {
    while (distanciaC < 8) // GIRAR ATE ENCONTRAR A ABERTURA NA DIREITA, OU VOLTAR POR ONDE VEIO CASO SEJA UM SEM SAIDA
    {
      re();
      acelera(100,70);
      delay(75);
      esquerda();
      acelera(100, 100);
      delay(75);
      parar();
      delay(75);
      frente();
      acelera(0, 0);
      ler_sensores();
    }
  }
}