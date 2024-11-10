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

VL53L1X sensorE;
VL53L1X sensorC;
VL53L1X sensorD;

// Variáveis Globais
float tamanho_carrinho = 14;
float tamanho_pista = 30;

double distanciaE;
double distanciaC;
double distanciaD;
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
  distanciaE = (sensorE.read() - 20) / 10;
  if (distanciaE > 400)
  {
    distanciaE = (sensorE.read() - 20) / 10;
  }
  sensorE.timeoutOccurred() ? distanciaE = 400 : distanciaE = distanciaE;

  distanciaD = (sensorD.read() - 20) / 10;
  if (distanciaD > 400)
  {
    distanciaD = (sensorE.read() - 20) / 10;
  }
  sensorD.timeoutOccurred() ? distanciaD = 400 : distanciaD = distanciaD;

  distanciaC = (sensorC.read()) / 10;
  sensorC.timeoutOccurred() ? distanciaC = 400 : distanciaC = distanciaC;

  delta = distanciaE - distanciaD;
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

  sensorE.setTimeout(500);
  if (!sensorE.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("E");
    acende_leds();
    while (1)
    {
    }
  }
  sensorE.setAddress(0x2A);

  sensorE.startContinuous(50);

  pinMode(xshutPinsD, INPUT);
  delay(10);

  sensorD.setTimeout(500);
  if (!sensorD.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("D");
    acende_leds();
    while (1)
    {
    }
  }
  sensorD.setAddress(0x2A + 1);

  sensorD.startContinuous(50);

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
  delay(5000);
}

void acompanha_parede()
{
  apaga_led();
  if (distanciaD > tamanho_pista) // 5 -> curva pra direita
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
  else if (distanciaD >= 10) // 6
  {
    digitalWrite(led_verde, HIGH);
    digitalWrite(led_vermelho, HIGH);
    direita();
    {
      acelera(100, 100);
      delay(50);
    }
    frente();
    {
      acelera(100, 70);
      delay(150);
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
      if (distanciaC > 6) // 7
      {
        // digitalWrite(led_vermelho, HIGH);
        // digitalWrite(led_verde, HIGH);
        // digitalWrite(led_azul, HIGH);
        // acelera(100, 35);
        // delay(100);
        // parar();
        // delay(75);
      }
      else // 8
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
  else if (distanciaD >= 7) // 9
  {
    digitalWrite(led_branco, HIGH);
    digitalWrite(led_azul, HIGH);
    acelera(80, 100);
    delay(150);
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
  else if (distanciaD <= 5) // 10
  {
    digitalWrite(led_branco, HIGH);
    digitalWrite(led_verde, HIGH);
    // digitalWrite(IN1, HIGH);
    // digitalWrite(IN3, LOW);
    esquerda();
    {
      acelera(100, 100);
      delay(100);
    }
    frente();
    {
      acelera(100, 80);
      delay(100);
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
    acelera(70, 100);
    delay(150);
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
  else if (distanciaE > tamanho_pista) // 2 -> frente e direita ocupada, curva à esquerda
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
  else if (distanciaD < ((tamanho_pista - tamanho_carrinho) / 2) && distanciaE < ((tamanho_pista - tamanho_carrinho) / 2)) // 3 -> caminho sem saida
  {
    digitalWrite(led_azul, HIGH);
    digitalWrite(led_verde, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    while (distanciaE < 10)
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