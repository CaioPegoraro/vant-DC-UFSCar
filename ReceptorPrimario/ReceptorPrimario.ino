//Receptor primário
//Modelo: Arduino UNO
//Objetivo: primeiro receptor, está conectado ao receptor wireless que é capaz de receber e enviar comandos, tem uma rotina de checagem por instruções
//          do emissor, em caso positivo, envia as mesmas via IC2 para o receptor secundário.


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "pacote.h"

RF24 radio(7, 8);
const byte rxAddr[6] = "00002";
const byte wxAddr[6] = "00001";

//Configuração para comunicação IC2 com o arduino mega
#include <Wire.h>
#define SLAVE_ADDRESS 0x60

pacote dados;
byte buff[2];

//Variáveis de hardware
int pin_buzzer = 10;
int status_buzzer = 0;//inicialmente desligado
unsigned long previousMillis = 0;
const long interval = 700;

//leitura da bateria
int sensorValue;

int led_conexao = 4;

void setup()
{
  //IC2
  Wire.begin();

  //Serial
  while (!Serial);
  Serial.begin(9600);

  //Wireless
  radio.begin();
  radio.openWritingPipe(wxAddr);
  radio.openReadingPipe(0, rxAddr);
  radio.startListening();

  //Hardware
  pinMode(led_conexao, OUTPUT);
  pinMode(pin_buzzer, OUTPUT);
  pinMode(A0, INPUT);
}

//Função para enviar uma resposta para o emissor primário
void enviaMsg(int cmd, int valor) {
  //Serial.println("Enviando resposta");
  radio.stopListening();
  dados.cmd = cmd;
  dados.valor = valor;
  //tentativas de enviar ao emissor original
  for (int i = 0; i < 10; i++) {
    radio.write(&dados, sizeof(dados));
  }
  //delay(1);
  radio.startListening();
}

void requestEvent() {

};

void buzzer() {
  //Serial.println("toca o som dj!!");
  tone(pin_buzzer, 4000, 300); //freq = 4000hz
  //delay(500);
}


//Função para enviar uma mensagem para o arduino mega (responsável pelo controle dos motores
//OBS: para garantir a execução é utilizado o pino de interrupcçãoo do arduino mega, desviando
//     para uma rotina de leitura IC2 que receberá o comando enviado do controlador.
void enviaIC2() {
  Serial.println("enviando");
  int valor_pacote = dados.cmd * 1000 + dados.valor;
  //Serial.println(dados.cmd);
  //Serial.println(dados.valor);

  String pacote_ic2 = String(valor_pacote);
  Wire.beginTransmission(SLAVE_ADDRESS);
  for (int x = 0; x < 4; x++) {
    Wire.write(pacote_ic2.charAt(x));
  }
  //Serial.println(pacote_ic2);
  Wire.endTransmission();
}


void loop()
{
  if (status_buzzer == 1) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      buzzer();
    }
  }

  //Descrição do loop: Mantém uma checagem contínua pelo sinal de wireless vindo do emissor primário,
  //                   ao receber um comando checa o tipo, caso seja simples executa a tarefa ou transmite
  //                   para o receptor secundário, caso seja composto, produz os dados necessários e
  //                   faz o envio de retorno ao emissor primário
  if (radio.available())
  {
    radio.read(&dados, sizeof(dados));
    //int tmp = dados.cmd;
    //dados.cmd = dados.valor;
    //dados.valor = tmp;

    //Serial.print("\n cmd: ");
    //Serial.println(dados.cmd);
    //Serial.print("\n valor: ");
    //Serial.println(dados.valor);

    //Serial.println("Rprimário recebido: " + dados.cmd);
    //Serial.println("valor: " + dados.valor);
    //Serial.println(dados.cmd);

    //avalia o tipo do comando
    if (dados.cmd <= 124) { //comandos simples

      switch (dados.cmd) {
        case 14: //Acionamento do buzzer
          status_buzzer = 1;
          break;

        case 15: //Desligamento do buzzer
          status_buzzer = 0;
          break;


        case 1: //Aciona motor1
          enviaIC2();
          break;

          case 2: //Aciona motor2
          enviaIC2();
          break;

          case 3: //Aciona motor3
          enviaIC2();
          break;

          case 4: //Aciona motor4
          enviaIC2();
          break;

        case 6: //Acionar todos os motores em uma velocidade
          enviaIC2();
          break;

        case 7: //Calibrar motores
          enviaIC2();
          break;

        case 8: //Liberar motores
          enviaIC2();
          break;

        case 9: //trava motores
          enviaIC2();
          break;
      }
    }
    else { // >=125 comandos compostos

      switch (dados.cmd) {
        case 125: //Conexao inicial
          digitalWrite(led_conexao, HIGH);
          enviaMsg(125, 0);
          break;

        case 126: //Ler carga da bateria
          sensorValue = analogRead(A0);
          float voltage = sensorValue * (5.0 / 1023.0);
          voltage = voltage * 100; //ajustar casas decimais para emular um inteiro
          int volt_tmp = voltage;
          enviaMsg(126, volt_tmp);
          break;
      }
    }
    /*
        if (dados.valor == 10) {
          enviaMsg(); //envia uma mensagem de volta ao controlador
        }
        else if (dados.valor == 15) {
          enviaIC2(); //envaminha o comando ao receptor secundário
        }
    */

  }
}
