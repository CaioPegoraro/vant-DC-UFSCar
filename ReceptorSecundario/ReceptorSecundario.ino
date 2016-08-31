#include <Wire.h>
#include <Servo.h>

#include "I2Cdev.h"
#include "pacote.h"
#include "MPU6050.h"

#define SLAVE_ADDRESS 0x60

pacote dados;
String data = "";

//Variáveis de controle dos motores
Servo motor1; //frente esquerda
Servo motor2; //frente direita
Servo motor3; //tras esquerda
Servo motor4; //tras direita

int flag_calibrar = 0;
int flag_operacao = 0;
int flag_ok = 0; //controle anti-rotação involuntária

int led_Motores = 9; //LED ligado: motores liberados

//PID: Controle de estabilização automático

//Velocidades: A velocidade de cada motor é definida por dois fatores:
//I: O emissor envia um valor base para o motor, no qual podemos controlar a altura do drone, já
//que é possível aumentar ou diminuir todas as intensidades.
//II: O valor calculado pelo PID, que define um valor de acrescimo ou decrescimo para cada motor
//afim de controlar a estabilidade

int vBase[4]; //velocidade base
int vPid[4]; //fator de acrescimo/decrescimo para cada motor

//MPU (acelerômetro/giroscópio)
MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int valX;
int prevValX;

int valY;
int prevValY;

int valX2;
int valY2;

int ledMpu = 8;

int led_pid = 11;

//PID EIXO X
double setX = 0;
double prevX = 0;//leitura passada
double errorX = 0;

//K: "peso" de cada coeficiente
double kpX = 5.0,
       kiX = 0.0,
       kdX = 10.0;

//P: proporcional
//I: integral
//D: derivada
double Px = 0,
       Ix = 0,
       Dx = 0;

double PIDx = 0;

//Controle de tempo para Iy e Dy:
long prevProcX = 0;

//PID EIXO Y
double setY = 0;
double prevY = 0;//leitura passada
double errorY = 0;

//K: "peso" de cada coeficiente
double kpY = 7.0,
       kiY = 0.0,
       kdY = 10.0;

//P: proporcional
//I: integral
//D: derivada
double Py = 0,
       Iy = 0,
       Dy = 0;

double PIDy = 0;

//Controle de tempo para Iy e Dy:
long prevProcY = 0;

//Offset do MPU
int offX = 0;
int offY = 0;

int indiceMy = 1;
int indiceMx = 1;

float varTempo;
float reguladorAtt = 0;
float reguladorPrev = 0;
//FIM VARIÁVEIS DO PID

int tempOff = 1;

void setup() {

  //MPU
  pinMode(ledMpu, OUTPUT);
  pinMode(led_pid, OUTPUT);
  
  Serial.println("Initialize MPU");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("Connected");
    digitalWrite(ledMpu, HIGH);
  }
  else {
    Serial.println("Connection failed");
  }

  //Motores
  for (int i = 0; i < 4; i++) {
    vBase[i] = 65;
    vPid[i] = 0;
  }

  pinMode(led_Motores, OUTPUT);

  motor1.attach(7);
  motor2.attach(6);
  motor3.attach(4);
  motor4.attach(5);

  //Configuração comunicação IC2:
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);
  Serial.setTimeout(50);

  //Variaveis de controle inicialização:
  dados.valor = 0;

  //Calibração dos motores
  motor1.writeMicroseconds(2100);
  motor2.writeMicroseconds(2100);
  motor3.writeMicroseconds(2100);
  motor4.writeMicroseconds(2100);
  delay(3);

  motor1.writeMicroseconds(800);
  motor2.writeMicroseconds(800);
  motor3.writeMicroseconds(800);
  motor4.writeMicroseconds(800);
  delay(2);

  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500);
  motor3.writeMicroseconds(1500);
  motor4.writeMicroseconds(1500);
  delay(2);

  motor1.writeMicroseconds(2100);
  motor2.writeMicroseconds(2100);
  motor3.writeMicroseconds(2100);
  motor4.writeMicroseconds(2100);
  delay(2);

  motor1.write(65);
  motor2.write(65);
  motor3.write(65);
  motor4.write(65);
  delay(2);
}

void loop() {

  if (tempOff == 1) {

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    offX = map(ax, 17000, -17000, -50, 50);
    offY = map(ay, -17000, 17000, 50, -50);

    offX = (-1) * offX;
    offY = (-1) * offY;

    Serial.print("Offx:");
    Serial.println(offX);

    Serial.print("Offy:");
    Serial.println(offY);
    tempOff = 0;
  }

  //Loop: realiza operações de calibração nos motores (constitui uma das ações críticas de segurança para evitar casos em que
  //      os motores hajam de maneira inapropriada (operando em velocidade maxima sem controle por exemplo).

  //Antes de destravar os motores o loop fica executando a calibração dos motores, quando se libera os motores essa operação será
  //executada apenas quando enviado um comando pelo painel de controle.
  //Ao liberar os motores a operação padrão será o controle de estabilidade realizado pelo PID utilizando os valores do acelerômetro.
  //intercalado a essa operação seram executados operações de controle manual (controlado pelo painel de controle).

  //flag de calibrar: indica a prioridade da operação de calibação (mantendo os motores na velocidade mínima/desligados)
  //flag de ok: se for 0 indica que nao houve comandos que utilizam os motores do drone
  if (flag_calibrar == 1 || flag_ok == 0) {
    //calibrar motores

    //Serial.println("calibrando");
    flag_calibrar = 0;

    motor1.write(65);
    motor2.write(65);
    motor3.write(65);
    motor4.write(65);
    delay(2);

  }
  //flag de operação: indica execução de algum comando que
  //utiliza algum dos motores
  //flag de ok: indica que os motores estao liberados por software
  else if (flag_operacao == 1 && flag_ok == 1) {
    //nao precisa calibrar, entao vai processar a operação dos motores
    flag_operacao = 0; //a flag eh valida para uma operação

    if (dados.valor < 180 && dados.valor > 60) {
      //faixa de operação dos motores

      switch (dados.cmd) {

        case 1:
          //Serial.println("acionando motor 1");
          motor1.write(dados.valor);
          break;
        case 2:
          //Serial.println("acionando motor 2");
          motor2.write(dados.valor);
          break;
        case 3:
          //Serial.println("acionando motor 3");
          motor3.write(dados.valor);
          break;
        case 4:
          //Serial.println("acionando motor 4");
          motor4.write(dados.valor);
          break;

        case 6:
          //Serial.println("acionando todos os motores");
          /* motor1.write(dados.valor);
            motor2.write(dados.valor);
            motor3.write(dados.valor);
            motor4.write(dados.valor);
          */
          //Salva o valor de base no vetor de velocidades
          for (int i = 0; i < 4; i++) {
            vBase[i] = dados.valor;
          }

          //Serial.println(dados.valor);
          delay(15);
          break;
      }
    }
  }
  //Operação de estabilização do VANT
  else {

    reguladorAtt = millis() / 1000.0;

    //Serial.println(reguladorAtt);
    //Serial.println(reguladorPrev);

    if ((reguladorAtt - reguladorPrev) > 1) {
      digitalWrite(led_pid, HIGH);
      //O loop de correcaoe executa em intervalos
      //iguais ou maiores a 1segundo
      reguladorPrev = reguladorAtt;

      //Serial.println("Estabilizando PID!");
      //o calculo e a correção são feitos em intervalos
      //de tempo: isso porque dado um desvio é aplicado
      //um critério de correção somando e diminuindo
      //a velocidade de certos motores,
      //após o período pode ser que o ajuste não seja
      //mais necessário (está estável), caso não
      //então um ajuste em cima do ajuste anterior é feito.

      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      valX = map(ax, 17000, -17000, -50, 50);
      valX += offX; //correção de hardware,

      valY = map(ay, -17000, 17000, 50, -50);
      valY += offY;

      /*
                Serial.print("aX: ");
                Serial.print(valX);
                Serial.print(", ");

                Serial.print("Offx: ");
                Serial.print(offX);
                Serial.print(", ");

                Serial.print("aY: ");
                Serial.print(valY);
                Serial.print(", ");

                Serial.print("Offy: ");
                Serial.println(offY);

      */

      //======= calcular PID para eixo Y  =======//
      errorY = setY - valY;

      //quanto tempo levou desde a ultima vez que executou o PID?
      //tempo atual - ultimo tempo
      varTempo = (millis() - prevProcY) / 1000.0;
      prevProcY = millis();

      varTempo = 1;

      //Py
      Py = errorY * kpY;

      //Iy
      Iy += (errorY * kiY) * (varTempo);
      //conversao do tempo para segundos

      //Dy
      Dy = (prevY - valY) * kdY / (varTempo);

      //salva o valor da "leitura passada" para o proximo loop
      prevY = valY;

      PIDy = Py + Iy + Dy; // PIDy = 0 => Equilíbrio atingido!

      /*
                      Serial.print(" Py: ");
                      Serial.print(Py);

                      Serial.print(", Iy: ");
                      Serial.print(Iy);

                      Serial.print(", Dy: ");
                      Serial.print(Dy);

                      Serial.print(" => Valor PIDy: ");
                      Serial.println(PIDy);

      */
      /*
        //preparar o vetor de acrescimo para receber os dados do PIDy e PIDx
        for (int k = 0; k < 4; k++) {
        vPid[k] = 0;
        }
      */
      //calcular o efeito do PID no eixo Y

      //PIDy >0 : Inclinação do eixo dos motores 1 e 2 pra cima
      //Ação: diminuir intensidade nos motores 1 e 2,
      //acrescentar nos motores 3 e 4
      int factor = 10;

      PIDy = PIDy / factor;
      //PIDy = 0;
      if (PIDy > 0) {

        vPid[2] += PIDy;
        vPid[3] += PIDy;

        vPid[0] += (-1) * PIDy;
        vPid[1] += (-1) * PIDy;

      }
      //PIDy <0 : Inclinação do eixo dos motores 3 e 4 pra cima
      //Ação: diminuir intensidade nos motores 3 e 4,
      //acrescentar nos motores 1 e 2
      else if (PIDy < 0) {

        vPid[0] += (-1) * PIDy;
        vPid[1] += (-1) * PIDy;

        vPid[2] += PIDy;
        vPid[3] += PIDy;

      }

      //======= calcular PID para eixo X  =======//
      errorX = setX - valX;

      //quanto tempo levou desde a ultima vez que executou o PID?
      //tempo atual - ultimo tempo
      varTempo = (millis() - prevProcX) / 1000.0;
      prevProcX = millis();

      varTempo = 1;

      //Px
      Px = errorX * kpX;

      //Ix
      Ix += (errorX * kiX) * (varTempo); //conversao do tempo para segundos

      //Dx
      Dx = (prevX - valX) * kdX / (varTempo);

      //salva o valor da "leitura passada" para o proximo loop
      prevX = valX;

      PIDx = Px + Ix + Dx; // PIDx = 0 => Equilíbrio atingido!
      /*
            Serial.print(" Px: ");
            Serial.print(Px);

            Serial.print(", Ix: ");
            Serial.print(Ix);

            Serial.print(", Dx: ");
            Serial.print(Dx);

            Serial.print(" => Valor PIDx: ");
            Serial.println(PIDx);
      */

      //calcular o efeito do PID no eixo X

      //PIDx <0 : Inclinação do eixo dos motores 1 e 3 pra cima
      //Ação: diminuir intensidade nos motores 1 e 3,
      //acrescentar nos motores 2 e 4

      PIDx = PIDx / factor;
      //PIDx =0;
      if (PIDx < 0) {

        vPid[0] += PIDx;
        vPid[2] += PIDx;

        vPid[1] += (-1) * PIDx;
        vPid[3] += (-1) * PIDx;

      }
      //PIDx >0 : Inclinação do eixo dos motores 4 e 2 pra cima
      //Ação: diminuir intensidade nos motores 4 e 2,
      //acrescentar nos motores 1 e 3
      else if (PIDx > 0) {

        vPid[1] += (-1) * PIDx;
        vPid[3] += (-1) * PIDx;

        vPid[0] += PIDx;
        vPid[2] += PIDx;

      }

      for (int j = 0; j < 4; j++) {
        Serial.print("M");
        Serial.print(j);
        Serial.print(": ");
        //trava do intervalo de operação
        if ((vBase[j] + vPid[j]) < 70) {
          vPid[j] = (-1) * (vBase[j] - 70);
        }
        else if ((vBase[j] + vPid[j]) > 180) {
          vPid[j] = 180 - vBase[j];
        }
        Serial.println(vBase[j] + vPid[j]);
      }
      Serial.println("<========>");

      //Atualiza o valor dos motores
      motor1.write(vBase[0] + vPid[0]);
      motor2.write(vBase[1] + vPid[1]);
      motor3.write(vBase[2] + vPid[2]);
      motor4.write(vBase[3] + vPid[3]);

      //FIM DA ESTABILIZAÇÃO
      digitalWrite(led_pid, LOW);
    }

    delay(100);

    /*
      else { //parar motores
        motor1.write(65);
        motor2.write(65);
        motor3.write(65);
        motor4.write(65);
        //Serial.println(65);
      }
    */
  } //fim do if para calculo do intervalo de tempo
}

//A função trata de eventos recebidos do mestre (pela hierarquia IC2)
//quando há um envio por parte dele o escravo (esse receptor)
//desvia para tratamento do dado recebido, que consiste em
//ativar um flag (como se fosse uma senha de banco) para o loop
//executar uma operação unicamente.
//e tb para salvar em uma variavel o valor do dado associado.
//a função foi previamente configurada no setup para esse tipo
//de tratamento.
void receiveEvent(int howMany) {
  data = "";
  while (Wire.available()) { //enqnto estiver disponivel bytes
    data += (char)Wire.read();
  }
  int valor_data = data.toInt();
  //Serial.println(valor_data);
  dados.cmd = valor_data / 1000;
  dados.valor = valor_data - dados.cmd * 1000;

  if (dados.cmd < 7) { //conjunto de comandos que vai
    //acionar um ou mais motores
    flag_operacao = 1;
  }
  else if (dados.cmd == 7) {
    flag_calibrar = 1;
  }
  else if (dados.cmd == 8) {
    //liberar flag para operar os motores
    flag_ok = 1;
    digitalWrite(led_Motores, HIGH);
  }
  else if (dados.cmd == 9) {
    //travar flag para operar os motores
    flag_ok = 0;
    digitalWrite(led_Motores, LOW);
  }
  //Serial.println(dados.cmd);
  //Serial.println(dados.valor);
}
