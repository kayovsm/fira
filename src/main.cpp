#include "Arduino.h"
#include "VL53L1X.h"
#include "HardwareSerial.h"

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
double DIS_MAX = 7.7;

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

  distanciaC = (sensorC.read()) / 10;
  sensorC.timeoutOccurred() ? distanciaC = 400 : distanciaC = distanciaC;
}

void imprimeDistancias()
{
  Serial.print("Dis Esq: ");
  Serial.print(distanciaDF);
  Serial.print(" cm  /  ");
  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" cm   /  ");
  Serial.print("Dis Dir: ");
  Serial.print(distanciaDR);
  Serial.println(" cm     /   ");
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

  // fazer função para retornar qual o sentido de giro
  // Serial.println(direcao[0]);
  // Serial.println(direcao[1]);
  // Serial.println(vel_esquerda_int);
  // Serial.println(vel_direita_int);
  analogWrite(direcao[1], vel_esquerda_int);
  analogWrite(direcao[0], vel_direita_int);
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
  pinMode(IN3, OUTPUT); // OUTPUT = Saída
  pinMode(IN4, OUTPUT); // INPUT = Entrada

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
}

void acompanha_parede()
{
  apaga_led();
  if (distanciaDR > tamanho_pista) // 5 -> curva pra direita
  {
    digitalWrite(led_azul, HIGH);
    digitalWrite(led_vermelho, HIGH);
    acelera(0, 100);
    delay(150);
    direita();
    {
      acelera(100, 100);
      delay(175);
    }
    parar();
    {
      delay(75);
    }
    frente();
    acelera(0, 0);
  }
  else if (distanciaDR >= 10) // 6
  {
    digitalWrite(led_verde, HIGH);
    digitalWrite(led_vermelho, HIGH);
    direita();
    {
      acelera(80, 80);
      delay(75);
    }
    frente();
    {
      acelera(60, 100);
      delay(175);
    }
    parar();
    {
      delay(50);
    }
    frente();
    acelera(0, 0);
    ler_sensores();
    {
      apaga_led();
      if (distanciaC < 7) // 7
      {
        digitalWrite(led_branco, HIGH);
        re();
        {
          acelera(60, 80);
          delay(350);
        }
        parar();
        {
          delay(75);
        }
      }
    }
    frente();
    acelera(0, 0);
  }
  else if (distanciaDR >= 7) // 9
  {
    digitalWrite(led_branco, HIGH);
    digitalWrite(led_azul, HIGH);
    acelera(60, 100);
    delay(150);
    direita();
    {
      acelera(100, 100);
      delay(75);
    }
    parar();
    {
      delay(75);
    }
    frente();
    acelera(0, 0);
  }
  else if (distanciaDR <= 5) // 10
  {
    digitalWrite(led_branco, HIGH);
    digitalWrite(led_verde, HIGH);
    // digitalWrite(IN1, HIGH);
    // digitalWrite(IN3, LOW);
    esquerda();
    {
      acelera(100, 100);
      delay(50);
    }
    parar();
    digitalWrite(led_branco, HIGH);
    digitalWrite(led_verde, HIGH);
    delay(50);
    frente();
    {
      acelera(100, 40);
      delay(150);
    }
    parar();
    {
      delay(75);
    }
    frente();
    acelera(0, 0);
  }
  else // 11 -> 6 centimetros da parede da direita, pende um pouco pra esquerda
  {
    digitalWrite(led_branco, HIGH);
    digitalWrite(led_verde, HIGH);
    digitalWrite(led_azul, HIGH);
    acelera(60, 100);
    delay(100);
    parar();
    {
      digitalWrite(led_branco, HIGH);
      digitalWrite(led_verde, HIGH);
      digitalWrite(led_azul, HIGH);
      delay(75);
    }
    direita();
    {
      acelera(100, 100);
      delay(50);
    }
    parar();
    {
      delay(75);
    }
    frente();
    acelera(0, 0);
  }
}

void loop()
{
  // frente();
  // while(true){
  //   acelera(100,70);
  // }
  apaga_led();
  frente();
  acelera(0, 0);
  delay(10);
  ler_sensores();
  if (distanciaC > 6) // 1 -> frente livre, acompanhar parede da direita ate achar alguma abertura
  {
    digitalWrite(led_azul, HIGH);
    acompanha_parede();
  }
  else if (distanciaDF > tamanho_pista) // 2 -> frente e direita ocupada, curva à esquerda
  {
    digitalWrite(led_verde, HIGH);

    while (distanciaC < 15)
    {
      ler_sensores();
      esquerda();
      acelera(100, 100);
      delay(100);
      parar();
      {
        delay(75);
      }
    }
    frente();
    acelera(0, 0);
  }
  else if (distanciaDR < ((tamanho_pista - tamanho_carrinho) / 2) && distanciaDF < ((tamanho_pista - tamanho_carrinho) / 2)) // 3 -> caminho sem saida
  {
    digitalWrite(led_azul, HIGH);
    digitalWrite(led_verde, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    while (distanciaDF < 10)
    {
      ler_sensores();
      esquerda();
      acelera(100, 100);
      delay(75);
      parar();
      delay(75);
    }
    frente();
    acelera(0, 0);
  }
  else // 4 -> aproximou demais da parede da frente e tem a esquerda bloqueada
  {
    digitalWrite(led_vermelho, HIGH);
    re();
    {
      acelera(100, 50);
      delay(75);
      acelera(70, 100);
      delay(75);
    }
    parar();
    {
      delay(50);
    }
    frente();
    acelera(0, 0);
  }
}