#include <SoftwareSerial.h>

#define RIGHT_MAX_SPEED 250 // velocidade maxima do robo
#define LEFT_MAX_SPEED 250 // velocidade maxima do robo
#define VELOCIDADE_TURBO 0
#define VELOCIDADE_PADRAO 50
#define THRESHOLD_SENSOR 500
#define LIMIAR_LEITURA 0.4

#define rightMotor1 8
#define rightMotor2 9
#define rightMotorPWM 10

#define leftMotor1 6
#define leftMotor2 7
#define leftMotorPWM 11

#define SEN_CURVA A0
#define SEN_CHEGADA A1
#define SEN_0 A2 //sensor mais a esquerda
#define SEN_1 A3
#define SEN_2 A4
#define SEN_3 A5
#define SEN_4 A6


float Kp = 80, Kd = 30, Ki = 0;

typedef struct
{
  int nome;
  float leitura;
  int lowerBound;
  int upperBound;
  int prioridade;
} Sensores;

Sensores sensor[7];

int velocidade = VELOCIDADE_PADRAO, lastError = 0, tempo_chegada=75;
float error;

unsigned long time;

float normalize(int medida, int lb, int ub);

void setup()
{
  Serial.begin(38400);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  //pinMode(motorPower, OUTPUT);

  pinMode(SEN_0, INPUT);
  pinMode(SEN_1, INPUT);
  pinMode(SEN_2, INPUT);
  pinMode(SEN_3, INPUT);
  pinMode(SEN_4, INPUT);
  pinMode(SEN_CURVA, INPUT);
  pinMode(SEN_CHEGADA, INPUT);

  delay(2000);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);

  sensor[0].nome = A0;
  sensor[1].nome = A1;
  sensor[2].nome = A2;
  sensor[3].nome = A3;
  sensor[4].nome = A4;
  sensor[5].nome = A5;
  sensor[6].nome = A6;

  sensor[0].prioridade = 0;
  sensor[1].prioridade = 0;
  sensor[2].prioridade = -3;
  sensor[3].prioridade = -2;
  sensor[4].prioridade = 1;
  sensor[5].prioridade = 2;
  sensor[6].prioridade = 3;

  for (int i = 0; i < 7; i++)
  {
    sensor[i].leitura = 0;
    sensor[i].lowerBound = 1000;
    sensor[i].upperBound = 0;
  }
  time = millis();
  time_atual = millis();
}

void loop() {

  int i;
  /*PARADA*/
  //Com tempo
  
  if (millis() - time > tempo_chegada * 10000)
  {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    delay(60000);
  }

  /*LEITURA SENSORES LINHA*/
  int leitura_sensor;
  for (i = 0; i < 7; i++)
  {
    leitura_sensor = analogRead(sensor[i].nome);
    if (leitura_sensor > sensor[i].upperBound)
      sensor[i].upperBound = leitura_sensor;
    if (leitura_sensor < sensor[i].lowerBound)
      sensor[i].lowerBound = leitura_sensor;
    //normalização
    sensor[i].leitura = normalize(leitura_sensor, sensor[i].lowerBound, sensor[i].upperBound);
  }

  float maior_leitura = 0, soma = 0;
  int ind_maior_leitura = 0;
  for (i = 2; i < 7; i++)
    if (sensor[i].leitura > maior_leitura)
    {
      maior_leitura = sensor[i].leitura;
      ind_maior_leitura = i;
      error += sensor[i].leitura * sensor[i].prioridade;
      soma += sensor[i].leitura;
    }
  error /= 5;
  
  /*CALCULA ERRO*/
  if ((ind_maior_leitura == 2) && (sensor[ind_maior_leitura + 1].leitura < 0.2))
    error = -1;
  else if ((ind_maior_leitura == 6) && (sensor[ind_maior_leitura - 1].leitura < 0.2))
    error = 1;

  if (soma < LIMIAR_LEITURA)
    error = lastError;

  /*CALCULA PID*/
  int I = I + error;
  int pid = Kp * error + Kd * (error - lastError) + Ki * I;
  int rightMotorSpeed;
  int leftMotorSpeed;

  lastError = error;

  /*VELOCIDADE MOTORES*/
  if (error == 0)
  {
    rightMotorSpeed = velocidade;
    leftMotorSpeed = velocidade;
  }
  else {
    rightMotorSpeed = velocidade  + pid;
    leftMotorSpeed = velocidade - pid;
  }

  if (rightMotorSpeed > RIGHT_MAX_SPEED ) rightMotorSpeed = RIGHT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > LEFT_MAX_SPEED ) leftMotorSpeed = LEFT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive


  //digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  //  digitalWrite(rightMotor1, HIGH);
  //digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, leftMotorSpeed);

  //digitalWrite(motorPower, HIGH);
  //  digitalWrite(leftMotor1, HIGH);
  //digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, rightMotorSpeed);
}
float normalize(int medida, int lb, int ub)
{
  float saida;
  if (ub < lb) //pra evitar divisao acidentao por zero, na primeira medida eh possivel
    return 0;
  saida = 1.0f - ((1.0f * medida - lb) / (ub - lb));
  if (saida < 0)
    saida = 0;
  if (saida > 1)
    saida = 1;
  return saida;
}


