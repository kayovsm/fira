#include "Arduino.h"
#include "VL53L1X.h"
#include <math.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <Wire.h>

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

void (*reset)(void) = 0;

void apaga_led()
{
  digitalWrite(led_azul, LOW);
  digitalWrite(led_branco, LOW);
  digitalWrite(led_verde, LOW);
  digitalWrite(led_vermelho, LOW);
}

void ler_sensores()
{
  apaga_led();
  distanciaDF = (sensorDF.read()) / 10.0;
  if (sensorDF.timeoutOccurred())
  {
    reset();
  }

  distanciaDR = (sensorDR.read()) / 10.0;
  if (sensorDR.timeoutOccurred())
  {
    reset();
  }

  distanciaC = (sensorC.read()) / 10.0;
  if (sensorC.timeoutOccurred())
  {
    reset();
  }

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

void acende_led(int num) {
  apaga_led();

  switch (num) {
    case 0:
      digitalWrite(led_branco, LOW);
      digitalWrite(led_vermelho, LOW);
      digitalWrite(led_verde, LOW);
      digitalWrite(led_azul, LOW);
      break;
    case 1:
      digitalWrite(led_azul, HIGH);
      break;
    case 2:
      digitalWrite(led_verde, HIGH);
      break;
    case 3:
      digitalWrite(led_verde, HIGH);
      digitalWrite(led_azul, HIGH);
      break;
    case 4:
      digitalWrite(led_vermelho, HIGH);
      break;
    case 5:
      digitalWrite(led_vermelho, HIGH);
      digitalWrite(led_azul, HIGH);
      break;
    case 6:
      digitalWrite(led_vermelho, HIGH);
      digitalWrite(led_verde, HIGH);
      break;
    case 7:
      digitalWrite(led_vermelho, HIGH);
      digitalWrite(led_verde, HIGH);
      digitalWrite(led_azul, HIGH);
      break;
    case 8:
      digitalWrite(led_branco, HIGH);
      break;
    case 9:
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_azul, HIGH);
      break;
    case 10:
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_verde, HIGH);
      break;
    case 11:
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_verde, HIGH);
      digitalWrite(led_azul, HIGH);
      break;
    case 12:
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_vermelho, HIGH);
      break;
    case 13:
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_vermelho, HIGH);
      break;
    case 14:
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_vermelho, HIGH);
      digitalWrite(led_verde, HIGH);
      break;
    case 15:
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_vermelho, HIGH);
      digitalWrite(led_verde, HIGH);
      digitalWrite(led_azul, HIGH);
      break;
    default:
      apaga_led();
      break;
  }
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

void setup()
{
  while (!Serial)
  {
  }

  Serial.begin(115200); // Comunicação Serial com o Computador
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  pinMode(IN1, OUTPUT);  // definição dos pinos entradas e saidas
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
    acende_led(15);
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
    acende_led(15);
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
    acende_led(15);
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
}

// DF é o mais próximo dos motores, enquanto o DR é o sensor na parte mais ao fundo do carrinho
void loop()
{
  ler_sensores();
  // imprimeDistancias();

  double distanciaMIN = 1;
  double distanciaMAX = 5;

  if (distanciaDF > tamanho_pista && distanciaDR < tamanho_pista) // -> curva pra direita
  {
    // girar pra direita ate encontrar de novo a parede (alguma leitura)
    // vai pra frente
    frente();
    while (distanciaDF > tamanho_pista) // 2
    {
      acende_led(2);
      acelera(100, 60);
      ler_sensores();
    }
    while (distanciaDF < tamanho_pista) // 3
    {
      acende_led(3);
      acelera(100, 85);
      ler_sensores();
    }
    acelera(0, 0);
  }
  else // perdi o dois sensores
  {
    frente();
    time = millis();
    while ((distanciaDR > tamanho_pista && distanciaDF > tamanho_pista) && millis() - time <= 150) // 4
    {
      acende_led(4);
      acelera(100, 75);
      ler_sensores();
    }
    if ((distanciaDR > tamanho_pista && distanciaDF > tamanho_pista))
    { // 5 - os dois perderam os sensores -  o que fazer?
      // direita();
      acende_led(5);
    }
    acelera(0, 0);
  }

  if (distanciaC > 8) // se a distanciaC for maior, sei que posso ir pra frente, mas preciso verificar minha distancia pra parede de referencia
  {
    if (media > distanciaMIN && media < distanciaMAX) // 6 - corrigir a inclinação com base na média (na realidade, seria com base no angulo ne?)
    {
      acende_led(6);
      frente();
      acelera(100, 85); // isso era pra andar reto, ajustar
      delay(50);
      ler_sensores();
    }
    else
    {
      ler_sensores();
      double original_angle = angle;
      while (original_angle * angle > 0)
      {
        float max_voltage_original = MAX_VOLTAGE;
        MAX_VOLTAGE = 165;
        if (distanciaDF - distanciaDR > 0) // 7
        {
          acende_led(7);
          direita();
          acelera(100, 100);
          delay(20);
        }
        else // 8
        {
          acende_led(8);
          esquerda();
          acelera(100, 100);
          delay(20);
        }
        MAX_VOLTAGE = max_voltage_original;
        frente();
        acelera(0, 0);
        ler_sensores();
      }
    }
    if (min(distanciaDF, distanciaDR) <= distanciaMIN) // 9
    {
      // forçar pra esquerda
      acende_led(9);
      acelera(100, 90);
    }
    else if (max(distanciaDF, distanciaDR) >= distanciaMAX) // 10
    {
      // forçar pra direita
      acende_led(10);
      acelera(100, 0);
    }
    else // 11
    {
      acende_led(11);
      acelera(100, 85);
    }
    delay(50);
  }
  else
  {
    while (distanciaC < 12) // 12 - GIRAR ATE ENCONTRAR A ABERTURA NA DIREITA, OU VOLTAR POR ONDE VEIO CASO SEJA UM SEM SAIDA
    {
      acende_led(12);
      re();
      acelera(100, 70);
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