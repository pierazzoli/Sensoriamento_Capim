/*
     Componentes:
    Arduino UNO

    1 Sensor de Ultrassônico HC-SR04
    1 Giroscópio (i2c)
    1 Bluetooth (Serial padrao)
    4 VL53l1X (i2c)
    1 Mini tf
    1 Hall
    1 DHT11
    1 Pushbutton
    1 LED
    2 Motores PWM

     Conexões de Hardware:

          VL53L1X 1 | Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Shutdown |  Pin A0
           SDA      |  Pin A4
           SCL      |  Pin A5
           Consumo Máx 40 mA, médio 15 a 20 mA

          VL53L1X 2 | Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Shutdown |  Pin A1
           SDA      |  Pin A4
           SCL      |  Pin A5

          VL53L1X 3 | Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Shutdown |  Pin A2
           SDA      |  Pin A4
           SCL      |  Pin A5

          VL53L1X 4 | Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Shutdown |  Pin A3
           SDA      |  Pin A4
           SCL      |  Pin A5


          Gyroscópio| Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           SDA      |  Pin A4
           SCL      |  Pin A5
          Consumo 5 a 15 mA
  ----------------------------------------
  ----------------------------------------
  ----------------------------------------


          HALL   | Arduino
        ----------------------------------------
           VCC      |  4.5 a 24V
           GND      |  GND
           Shutdown |  Pin D2
           Consumo 25mA

          HC-SR04 1 | Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Trig     |  Pin D3
           Echo     |  Pin D4
          Consumo 15mA

          DHT11     | Arduino
        ----------------------------------------
           VCC      |  3.5 a 5.5V
           GND      |  GND
           Dados    |  Pin D5
         Consumo 0.3mA (measuring) 60uA (standby)

        TF Mini Plus| Arduino
        ----------------------------------------
           VCC      |  +5V     (Atenção Verificar, sem proteção!!!!)
           GND      |  GND     (Atenção Verificar, sem proteção!!!!)
           TX       |  Pin D6
           RX       |  Pin D7  (3.3v)
           Consumo  550mW(low power mode)
           Consumo pico 140mA média de 110mA
            // TX verde e RX branco (3.3v)

          Pushbutton| Arduino
        ----------------------------------------
           GND      |  GND
           Pino     |  Pin D9


          PWM Motor | Arduino
        ----------------------------------------
           Pino     |  Pin D10, Motor 1 output
           Pino     |  Pin D11, Motor 2 output

         LED Onboard| Arduino
        ----------------------------------------
           Pino     |  Pin D13


         Bluetooth  | Arduino
        ----------------------------------------
           VCC      |  3.3 a 5V
           GND      |  GND
           TX       |  Pin D0
           RX       |  Pin D1  (3.3v)
          Consumo 0.5mA standby e 8.5mA transmitindo


          Consumo de pico
           1 (Mini Tf 140mA) + 4 (VL53L1X 40mA) + 1 ( Hall 25 mA) + 1 (HC-SR04 15mA) +
           1 (MPU6050 10mA) + 1 (DHT11 1mA) + 1 LED  + 1 (Bluetooth Hc06 9mA)
           = 360mA
*/

// ==============================================================================

#include <Arduino.h>

#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L1X.h>
#include <TFMPlus.h>
//#include "DHT.h"
#include "Adafruit_Sensor.h"

// ==============================================================================
// ------------ Variáveis e Constantes -----------------------
// ==============================================================================
int8_t i;

#define buffersz 35
int16_t buffer[9][buffersz +1];


#define alturaLimitecm 110
#define alturaInstSensorescm 80

#define alturaSemGramainiea 5
#define alturaPastagemDesejada 12
#define alturaPastagemDesejadaMedia 30
#define alturaPastagemDesejadaAlta 40

#define velocidadeLimiteMaxMps 1.1
#define velocidadeLimiteMinMps 0.1


// ==============================================================================

//HC-04 01
#define hc1_trig_pin 3
#define hc1_echo_pin 4
uint16_t dist_hc1;

//VL53l1x

#define setVLTimeout 500
#define setVLModeShort true
#define setVLMeasurementTimingBudget 50000
#define startVLContinuous 50

VL53L1X vl1;
#define SHUTDOWN_VL1_PIN A0
uint16_t dist_vl1;

VL53L1X vl2;
#define SHUTDOWN_VL2_PIN A1
uint16_t dist_vl2;

VL53L1X vl3;
#define SHUTDOWN_VL3_PIN A2
uint16_t dist_vl3;

VL53L1X vl4;
#define SHUTDOWN_VL4_PIN A3
uint16_t dist_vl4;

// TFmini
TFMPlus tfmP;
#define TFMINI_TX_PIN 6
#define TFMINI_RX_PIN 7
SoftwareSerial tfSerial( TFMINI_RX_PIN, TFMINI_TX_PIN); //verde e branco
uint16_t dist_TF1;

// MPU6050 - Giroscópio Endereco I2C
const int MPU = 0x68;
//Variaveis para armazenar valores do acelerometro e giroscópio
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Hall
#define Hall_interrupt_PIN 2

// PWM Motores
#define PWM_M1_PIN 10
#define PWM_M2_PIN 11

// Botão Start/Stop
#define button_start_stop_PIN 9

/*
  // DHT11
  #define dht_PIN 5
  #define DHTTYPE DHT22
  DHT dht(DHTPIN, DHTTYPE);
*/

// LED
#define LED_PIN 13
bool estadoled = 0; // variavel de controle

// ==============================================================================
// ------------ Protótipo das Funções  -----------------------
// ==============================================================================
void motorPwm (int m1, int m2);
long sensorHC (int trigpin , int echopin);
void giroscopio ( );
void interrupcao ( );
void imprimirBuffer( );
void imprimirStats( );

// ==============================================================================
// ------------ Setup -----------------------
// ==============================================================================
void setup( ) {

  //Serial
  Serial.begin(9600);
  Serial.println("Sistema incializando...");

  //Inicializa o I2C
  Wire.begin();
  Wire.setClock(400000); // usa I2C a 400 kHz

  //HC1
  pinMode( hc1_trig_pin, OUTPUT );   //HC-04 trig
  pinMode( hc1_echo_pin, INPUT  );   //HC-04 echo

  //VL 53L1x
  pinMode( SHUTDOWN_VL1_PIN, OUTPUT );    //Desliga VL1
  digitalWrite(SHUTDOWN_VL1_PIN, LOW);
  pinMode( SHUTDOWN_VL2_PIN, OUTPUT  );   //Desliga VL2
  digitalWrite(SHUTDOWN_VL2_PIN, LOW);
  pinMode( SHUTDOWN_VL3_PIN, OUTPUT  );   //Desliga VL3
  digitalWrite(SHUTDOWN_VL3_PIN, LOW);
  pinMode( SHUTDOWN_VL4_PIN, OUTPUT  );   //Desliga VL4
  digitalWrite(SHUTDOWN_VL4_PIN, LOW);

  pinMode(SHUTDOWN_VL1_PIN, INPUT);
  delay(150);
  vl1.init(true);
  delay(100);
  vl1.setAddress((uint8_t)21);
  vl1.setTimeout(setVLTimeout);

  pinMode(SHUTDOWN_VL2_PIN, INPUT);
  delay(150);
  vl2.init(true);
  delay(100);
  vl2.setAddress((uint8_t)22);
  vl2.setTimeout(setVLTimeout);

  pinMode(SHUTDOWN_VL3_PIN, INPUT);
  delay(150);
  vl3.init(true);
  delay(100);
  vl3.setAddress((uint8_t)23);
  vl3.setTimeout(setVLTimeout);

  pinMode(SHUTDOWN_VL4_PIN, INPUT);
  delay(150);
  vl4.init(true);
  delay(100);
  vl4.setAddress((uint8_t)24);
  vl4.setTimeout(setVLTimeout);

  if (setVLModeShort) {
    vl1.setDistanceMode(VL53L1X::Short);
    vl2.setDistanceMode(VL53L1X::Short);
    vl3.setDistanceMode(VL53L1X::Short);
    vl4.setDistanceMode(VL53L1X::Short);
  }

  //Intervalo entre medidas de 50000 us (50 ms), até 30 ms para short
  vl1.setMeasurementTimingBudget(setVLMeasurementTimingBudget);
  vl2.setMeasurementTimingBudget(setVLMeasurementTimingBudget);
  vl3.setMeasurementTimingBudget(setVLMeasurementTimingBudget);
  vl4.setMeasurementTimingBudget(setVLMeasurementTimingBudget);

  //Taxa Medição VL (ms)
  vl1.startContinuous(startVLContinuous);
  vl2.startContinuous(startVLContinuous);
  vl3.startContinuous(startVLContinuous);
  vl4.startContinuous(startVLContinuous);

  //I2C Giroscópio e acelerômetro
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //TF Mini Plus
  tfSerial.begin( 115200);
  delay(20);
  tfmP.begin( &tfSerial);
  tfmP.sendCommand( SOFT_RESET, 0);
  tfmP.sendCommand( SET_FRAME_RATE, FRAME_20);

  //LED onboard D13
  pinMode(LED_PIN, OUTPUT);

  //Pushbutton
  pinMode(button_start_stop_PIN, INPUT_PULLUP);

  //HallInt
  pinMode(Hall_interrupt_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Hall_interrupt_PIN), interrupcao, RISING);

  //PWM
  pinMode(PWM_M1_PIN, OUTPUT);
  pinMode(PWM_M2_PIN, OUTPUT);

  /*
    // DHT
    dht.begin();
  */

  i = 0;
  Serial.println("Setup dos sensores realizado.");
}


void loop( ) {

  if (digitalRead(button_start_stop_PIN) == LOW) // Se o botão for pressionado
  {
    estadoled = !estadoled; // troca o estado do LED
    if (estadoled) Serial.println ("start; \nID;HC1;VL1;VL2;VL3;VL4;TF1;GyX;GyY;GyZ;");
    else Serial.println ("stop;");

    digitalWrite(LED_PIN, estadoled);
    if (i > 0 )imprimirBuffer( ); //imprime os ultimos dados após o stop

    while (digitalRead(button_start_stop_PIN) == LOW); //Começa após soltar o botão
    delay(100);
  }

  if (estadoled) { //captura dados se Ligado

    buffer[0][i] = int16_t (sensorHC(hc1_trig_pin , hc1_echo_pin )); //HC-SR04 01

    buffer[1][i] = int16_t (vl1.read() / 10); //VL 01
    buffer[2][i] = int16_t (vl2.read() / 10); //VL 02
    buffer[3][i] = int16_t (vl3.read() / 10); //VL 03
    buffer[4][i] = int16_t (vl4.read() / 10); //VL 04
    if (vl1.timeoutOccurred()) {
      Serial.print("vl1 TIMEOUT");
    }
    if (vl2.timeoutOccurred()) {
      Serial.print("vl2 TIMEOUT");
    }
    if (vl3.timeoutOccurred()) {
      Serial.print("vl3 TIMEOUT");
    }
    if (vl4.timeoutOccurred()) {
      Serial.print("vl4 TIMEOUT");
    }

    tfmP.getData(buffer[5][i]);//TF Mini Plus

    giroscopio( );
    buffer[6][i] = GyX;
    buffer[7][i] = GyY;
    buffer[8][i] = GyZ;

    i++;

    if (i >= buffersz) {
      Serial.println ("Buffer Cheio;");
      imprimirBuffer( );
    }
  }
}//fim do loop()


// Define o PWM dos motores
void motorPwm(int m1, int m2)
{
  //adicionar limitadores min e max
  analogWrite( PWM_M1_PIN, m1);
  analogWrite( PWM_M2_PIN, m2);
}


// Sensor HC lê distancia
long sensorHC (int trigpin , int echopin)
{
  digitalWrite( trigpin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( trigpin, LOW );
  int interval = pulseIn( echopin, HIGH );

  return interval * 0.017; //cm
}


// Sensor MPU
void giroscopio( ) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}


// Sensor HC lê distancia
void interrupcao( ) {
  if (estadoled) {
    Serial.println("hall;");
    imprimirBuffer( );
  }
}


// Imprimir os dados no buffer
void imprimirBuffer( ) {

  for (int cont = 0;  cont < i ; cont++) {

    Serial.print(cont);
    Serial.print(";");

    Serial.print(buffer[0][cont]);
    Serial.print(";");
    Serial.print(buffer[1][cont]);
    Serial.print(";");
    Serial.print(buffer[2][cont]);
    Serial.print(";");
    Serial.print(buffer[3][cont]);
    Serial.print(";");
    Serial.print(buffer[4][cont]);
    Serial.print(";");
    Serial.print(buffer[5][cont]);
    Serial.print(";");
    Serial.print(buffer[6][cont]);
    Serial.print(";");
    Serial.print(buffer[7][cont]);
    Serial.print(";");
    Serial.print(buffer[8][cont]);
    Serial.println(";");
  }
  imprimirStats( );

  i = 0;
}



void imprimirStats( ) {

  int16_t min[6];
  int16_t max[6];
  int16_t med[6];

 for (int j = 0 ; j < 6 ; j++){
  min[j] = buffer[j][0];
  max[j] = buffer[j][0];
  med[j] = buffer[j][0];
}

  for (int cont = 1;  cont < i ; cont++) {
    for (int j = 0 ; j < 6 ; j++) {
      if (min[j] > buffer[j][cont]) min[j] = buffer[j][cont];
      med[j] = med[j] + buffer[j][cont];
      if (max[j] < buffer[j][cont]) max[j] = buffer[j][cont];
    }
  }
  for (int j = 0 ; j < i ; j++) med[j] = med[j] / i;

  Serial.print("\nSensor\t;Min\t;Med(");
  Serial.print(i);
  Serial.print(")\t;Max;\nTF1\t;");
  Serial.print(min[5]);
  Serial.print(";\t");
  Serial.print(med[5]);
  Serial.print(";\t");
  Serial.print(max[5]);

  Serial.print(";\t\nHC1\t;");
  Serial.print(min[0]);
  Serial.print(";\t");
  Serial.print(med[0]);
  Serial.print(";\t");
  Serial.print(max[0]);

  Serial.print(";\t\nVL1\t;");
  Serial.print(min[1]);
  Serial.print(";\t");
  Serial.print(med[1]);
  Serial.print(";\t");
  Serial.print(max[1]);

  Serial.print(";\t\nVL2\t;");
  Serial.print(min[2]);
  Serial.print(";\t");
  Serial.print(med[2]);
  Serial.print(";\t");
  Serial.print(max[2]);

  Serial.print(";\t\nVL3\t;");
  Serial.print(min[3]);
  Serial.print(";\t");
  Serial.print(med[3]);
  Serial.print(";\t");
  Serial.print(max[3]);

  Serial.print(";\t\nVL4\t;");
  Serial.print(min[4]);
  Serial.print(";\t");
  Serial.print(med[4]);
  Serial.print(";\t");
  Serial.print(max[4]);
  Serial.println(";\t");
}