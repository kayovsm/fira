#include <Arduino.h>
#include <VL53L1X.h>
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
float tamanho_pista = 19;

double distanciaDF;
double distanciaC;
double distanciaDR;
double delta;
double media;
double DIS_MAX = 7.7;
double dist_sensores = 4.1;
double angle;

double distanciaMIN = 4.5;
double distanciaMAX = 10.0;

unsigned long time;

float MAX_VOLTAGE = 80.0;
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

void acende_led(int num)
{
  apaga_led();

  switch (num)
  {
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
}

void girar_direita(int diminuir = 0)
{
  direita();
  acelera(100, 100);
  delay(360 + diminuir);
  frente();
  acelera(0, 0);
}

void ajuste(int delay_time)
{
  ler_sensores();
  if (distanciaDF > tamanho_pista || distanciaDR > tamanho_pista)
  {
    return;
  }
  
  float max_voltage_original = MAX_VOLTAGE;
  MAX_VOLTAGE = 120;
  const float TOLERANCIA = 1.0;

  while (abs(distanciaDF - distanciaDR) > TOLERANCIA)
  {
    if (distanciaDF > distanciaDR)
    {
      // Se a distância dianteira for maior, alinhar para a direita
      direita();
      acende_led(13);
    }
    else
    {
      // Se a distância traseira for maior, alinhar para a esquerda
      esquerda();
      acende_led(14);
    }
    acelera(100, 100);
    delay(30);
    acelera(0, 0);
    delay(30);

    // Atualizar leituras dos sensores
    ler_sensores();
  }

  // if (distanciaDF - distanciaDR > 0) // 7
  // {
  //   direita();
  //   acende_led(7);
  //   acelera(100, 100);
  //   delay(delay_time);
  // }
  // else // 8
  // {
  //   esquerda();
  //   acende_led(8);
  //   acelera(100, 100);
  //   delay(delay_time);
  // }
  // frente();
  // acelera(0, 0);
  // // delay(10);
  MAX_VOLTAGE = max_voltage_original;
}

void alinhar()
{
  ler_sensores();
  float max_voltage_original = MAX_VOLTAGE;
  MAX_VOLTAGE = 120;
  if ((distanciaDF - distanciaDR > 0))
  {
    while (distanciaDF - distanciaDR > 0)
    {
      direita();
      acende_led(13);
      acelera(100, 100);
      delay(15);
      acelera(0, 0);
      delay(50);
      ler_sensores();
    }
    frente();
    acelera(0, 0);
    MAX_VOLTAGE = max_voltage_original;
    return;
  }
  else if (distanciaDF - distanciaDR < 0)
  {
    while (distanciaDF - distanciaDR < 0)
    {
      esquerda();
      acende_led(14);
      acelera(100, 100);
      delay(15);
      acelera(0, 0);
      delay(50);
      ler_sensores();
    }
    frente();
    acelera(0, 0);
    MAX_VOLTAGE = max_voltage_original;
    return;
  }
}
void andar_reto(int vel_dir = 77, int vel_esq = 100)
{
  acelera(vel_esq, vel_dir);
}

void curva_direita()
{
  frente();
  acende_led(1);
  acelera(0,100);
  delay(75);
  acende_led(2);
  girar_direita(-60);
  acende_led(3);
  andar_reto();
  delay(800);
  acende_led(4);
  acelera(100,35);
  delay(1000);
  acende_led(5);
  andar_reto();
  delay(1000);
  acende_led(6);
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

  digitalWrite(led_branco, HIGH);
  delay(100);
  digitalWrite(led_branco, LOW);

  pinMode(xshutPinsE, INPUT);
  delay(10);

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
  imprimeDistancias();

  ler_sensores();
  if (distanciaDF > tamanho_pista && distanciaDR > tamanho_pista && distanciaC > 14) // 5 -> curva pra direita
  {
    // girar pra direita ate encontrar de novo a parede (alguma leitura)
    // vai pra frente
    curva_direita();
  }
  if (distanciaC > 14) // se a distanciaC for maior, sei que posso ir pra frente, mas preciso verificar minha distancia pra parede de referencia
  {
    if ((media > distanciaMIN && media < distanciaMAX) || (distanciaDF > tamanho_pista || distanciaDR > tamanho_pista)) // 6 - andar reto
    {
      acende_led(6);
      frente();
      andar_reto(); // isso era pra andar reto, ajustar
      delay(25);
    }
    if (min(distanciaDF, distanciaDR) <= distanciaMIN) // 9
    {
      // forçar pra esquerda
      frente();
      acende_led(9);
      acelera(100, 80);
    }
    else if (max(distanciaDF, distanciaDR) >= distanciaMAX) // 10
    {
      // forçar pra direita
      acende_led(10);
      frente();
      acelera(100, 0);
      delay(75);
      acelera(100, 73);
      delay(50);
    }
    delay(25);
    ajuste(50);
  }
  else
  {
    while (distanciaC < 17) // 12 - GIRAR ATE ENCONTRAR A ABERTURA NA DIREITA, OU VOLTAR POR ONDE VEIO CASO SEJA UM SEM SAIDA
    {
      imprimeDistancias();
      acende_led(12);
      // re();
      // acelera(100, 70);
      // delay(75);

      esquerda();
      acelera(100, 100);
      delay(150);

      parar();
      delay(25);

      frente();
      acelera(0, 0);
      ler_sensores();
    }
  }
}