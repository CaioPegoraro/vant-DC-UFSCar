#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;


int valX;
int prevValX;
int valY;
int prevValY;
int incomingByte = 0;

int valX2;
int valY2;

int ledMpu = 8;

//PID EIXO X


//PID EIXO Y
double setY = 0;
double prevY = 0;//leitura passada
double errorY = 0;

//K: "peso" de cada coeficiente
double kpY = 1.0,
       kiY = 0.0,
       kdY = 1.0;

//P: proporcional
//I: integral
//D: derivada
double Py=0, 
       Iy=0, 
       Dy=0;
       
double PIDy = 0;

//Controle de tempo para Iy e Dy:
long prevProcY = 0;


void setup()
{
  Wire.begin();
  Serial.begin(38400);
  pinMode(ledMpu, OUTPUT);

  Serial.println("Initialize MPU");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("Connected");
    digitalWrite(ledMpu, HIGH);
  }
  else {
    Serial.println("Connection failed");
  }
}

void loop()
{
  // envia dados apenas quando recebe dados:
  if (1) {
    // lê o primeiro byte disponível:
    incomingByte = Serial.read();

    // imprime na tela o byte recebido:

    if (1) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      valX = map(ax, 13000, -13000, -50, 50);
      valY = map(ay, -13000, 13000, -50, 50);

      valX += 0; //correção de hardware, o suporte do acelerometro faz com que o setpoint de
                //equilibrio seja (-2,0) para (x,y), a soma corrige para usar o referencial (0,0).
    
      Serial.print("aX: ");
      Serial.print(valX);
      Serial.print(", ");
      Serial.print("aY: ");
      Serial.print(valY);

      //calcular PID para eixo Y
      errorY = setY - valY;

      //quanto tempo levou desde a ultima vez que executou o PID?
      //tempo atual - ultimo tempo
      float varTempo = (millis() - prevProcY) / 1000.0; 
      prevProcY = millis();

      varTempo = 1;
      
      //Py
      Py = errorY * kpY;

      //Iy
      Iy += (errorY * kiY) * (varTempo); //conversao do tempo para segundos
      
      //Dy
      Dy = (prevY - valY) * kdY / (varTempo);

      //salva o valor da "leitura passada" para o proximo loop
      prevY = valY;

      PIDy = Py + Iy + Dy; // PIDy = 0 => Equilíbrio atingido!

      Serial.print(" Py: ");
      Serial.print(Py);

      Serial.print(", Iy: ");
      Serial.print(Iy);

      Serial.print(", Dy: ");
      Serial.print(Dy);
      
      Serial.print(" => Valor PIDy: ");
      Serial.println(PIDy);
      
    }
    //Serial.print("Eu recebi: ");
    //Serial.println(incomingByte, DEC);
  }
delay(100);
}
