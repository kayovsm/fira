#define TRIGD A0 // Pino Trig Sensor Direito
#define ECHOD A1 // Pino Echo Sensor Direito
#define TRIGE A2 // Pino Trig Sensor Esquerdo
#define ECHOE A3 // Pino Echo Sensor Esquerdo
#define TRIGC A4 // Pino Trig Sensor Centro
#define ECHOC A5 // Pino Echo Sensor Centro
#define ENA 5    // ENA PWM Motor Esquerdo
#define ENB 6    // ENB PWM Motor Direito///
#define IN1 3    // DIR Motor Esquerdo
#define IN2 9    // DIR Motor Esquerdo
#define IN3 10    // DIR Motor Direito
#define IN4 11   // DIR Motor Direito
#define alpha 1  // taxa do filtro 0% a 100%

// Variáveis Globais
unsigned int distanciaE; // unsigneg significa valores sem sinasi, + ou -
unsigned int distanciaC; // int considera valores ate 2^16
unsigned int distanciaD;
unsigned char vel = 150; // char considera valores ate 2^8 ou seja, 0 a 256

unsigned int sensorUS(int pinoTrig, int pinoEcho)
{
  unsigned int distancia;
  unsigned int tempoPulsoEcho;
  digitalWrite(pinoTrig, HIGH);                   // Ativa o pino de trigger
  delayMicroseconds(10);                          // Aguarda 10 microssegundos
  digitalWrite(pinoTrig, LOW);                    // Desativa o pino de trigger
  tempoPulsoEcho = pulseIn(pinoEcho, HIGH, 7500); // retorna o tempo em microssegundos
  delay(5);
  distancia = tempoPulsoEcho / 58;

  if ((distancia == 0) || (distancia > 40)) // caso o sensor leia o valor zero ou maior que 40 cm, ele retorna o valor 40
    return (40);

  else
    return (distancia);
}

float filtroE(float y) // filtro para sensor Esquerdo
{                      // mantem uma certa porcentagem do valor lido anteriormente
  static float yf;     // static mantem o ultimo valor lido, nâo cria nova variavel
  yf = alpha * y + (1 - alpha) * yf;
  return (yf);
}

float filtroC(float y)
{
  static float yf; // filtro para sensor central
  yf = alpha * y + (1 - alpha) * yf;
  return (yf);
}

float filtroD(float y)
{
  static float yf; // filtro para sensor central
  yf = alpha * y + (1 - alpha) * yf;
  return (yf);
}

void imprimeDistancias(void) // função para imprimir distancias no monitor serial
{
  Serial.print("Distancia Esquerda: ");
  Serial.print(distanciaE);
  Serial.print(" cm   ");
  Serial.print("Distancia Centro: ");
  Serial.print(distanciaC);
  Serial.print(" cm   ");
  Serial.print("Distancia Direita: ");
  Serial.print(distanciaD);
  Serial.println(" cm");
}
void reDireita(unsigned char vel, unsigned int tempo) // função para marcha ré para direita
{
  analogWrite(ENA, vel + 50); // roda da esquerda gira mias rapido para...
  analogWrite(ENB, vel - 50); // o robo tender a direita
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  delay(tempo);
}
void reEsquerda(unsigned char vel, unsigned int tempo) // função para marcha ré para esquerda
{
  analogWrite(ENA, vel - 50); // roda da direita gira mias rapido para...
  analogWrite(ENB, vel + 50); // o robo tender a direita
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  delay(tempo);
}
void andarFrente(unsigned char vel, unsigned int tempo) // função para andar pra frente
{
  analogWrite(ENA, vel);
  analogWrite(ENB, vel);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  delay(tempo);
}
void virarDireita(unsigned char vel, unsigned int tempo) // função para virar direita
{
  analogWrite(ENA, vel);
  analogWrite(ENB, vel);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  delay(tempo);
}
void virarEsquerda(unsigned char vel, unsigned int tempo) // função para virar esquerda
{
  analogWrite(ENA, vel);
  analogWrite(ENB, vel);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  delay(tempo);
}
void lerSensores(void) // função para ler distancias doas sensores
{
  distanciaE = sensorUS(TRIGE, ECHOE);
  distanciaC = sensorUS(TRIGC, ECHOC);
  distanciaD = sensorUS(TRIGD, ECHOD);
}

void distanciasFiltradas(void) // aplica o filtro nas dintancias lidas
{
  distanciaE = filtroE(distanciaE);
  distanciaC = filtroC(distanciaC);
  distanciaD = filtroD(distanciaD);
}

//___________________________________________________________________________________________________________________________

// Função de Configuração
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
}

//___________________________________________________________________________________________________________________________________________________________

// Função Principal (loop infinito)
void loop()
{
  lerSensores();         // Lê os Sensores
  distanciasFiltradas(); // filtra distancias
  imprimeDistancias();   // imprime as distancias no serial monitor

  if (distanciaE < 15)
  { // Regra curvar direita
    Serial.print("Curva Direita");
    Serial.println();
    virarDireita(150, 170);
  }
  if (distanciaD < 15)
  { // Regra para curvar a esquerda
    Serial.print("Curva Esquerda");
    Serial.println();
    virarEsquerda(150, 170);
  }
  if (distanciaC > 20)
  {
    Serial.print("Em Frente");
    Serial.println();
    andarFrente(254, 50);
  }
  else if (distanciaC > 13)
  {
    Serial.print("Em Frente mais devagar");
    Serial.println();
    andarFrente(150, 50);
  }
  if (distanciaC < 13) // Regra para marcha ré
  {
    if (distanciaD > distanciaE)
    {
      Serial.print("Ré Esquerda");
      Serial.println();
      reEsquerda(200, 150);
    }
    else
    {
      Serial.print("Ré Direita");
      Serial.println();
      reDireita(200, 150);
    }
  }
}