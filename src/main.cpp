
/* -----------------------------------------------------------------------
    __________                          .___     .__
   \______   \ ____   ____ _____     __| _/____ |__|___________
     |       _//  _ \_/ ___\\__  \   / __ |/ __ \|  \_  __ \__  \
     |    |   (  <_> )  \___ / __ \_/ /_/ \  ___/|  ||  | \// __ \_
     |____|___/\____/ \_  __>______/\_____|\____ >__||__|  (______/
                        \/
                  _________                      __
                /   _____/ _____ _____ ________/  |_
                \_____  \ /     \\__  \\_  __ \   __\
                /        \  Y Y  \/ __ \|  | \/|  |
               /________ /__|_|__(______/__|   |__|

  --------------------------------------------------------------------------
// @author E. Pierazzoli



    Sistema de sensoreamento para Roçadeira Smart
    Mestrado PPGCAP: 2020-2022

    Version: 1.3
    Date: 04/01/2022

    Autor: Eugênio Pierazzoli
    pierazzoli@gmail.com
    eugenio.pierazzoli@aluno.unipampa.edu.br

    https://github.com/pierazzoli/Sensoriamento_Capim

    PPGCAP (Programa de Pós-graduação em Computação Aplicada)
    Unipampa Campus Bagé e Embrapa Pecuária Sul

    Orientador:     Dr. Naylor Bastiani Perez     (Embrapa Pecuária Sul)
    Coorientador:   Dr. Leonardo Bidese de Pinho  (Unipampa Campos Bagé)

    Bolsista de Iniciação Científica: Willian Domingues

  ------------------------------------------------------------------------
  Componentes:
    1 Arduino Nano (Old Bootloader 328P)
    (ATMEGA328P 16MHz, 2 KB RAM, 30 KB Flash)

    1 HC-SR04
    1 Giroscópio    (I2C)
    1 Bluetooth     (UART)
    3 VL53L1X       (I2C)
    1 TF Mini Plus  (UART)
    1 Hall
    1 DHT11
    1 Push Button
    1 LED
    2 Motores PWM
    1 LDR

   Pinagem Física:

          VL53L1X 1 | Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Shutdown |  Pin A0
           SDA      |  Pin A4
           SCL      |  Pin A5
           Consumo: Máx 40 mA, médio 15 a 20 mA

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

          VL53L1X 4 | Arduino  (danificado)
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Shutdown |  Pin A3 (livre)
           SDA      |  Pin A4
           SCL      |  Pin A5

          Giroscópio| Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           SDA      |  Pin A4
           SCL      |  Pin A5
          Consumo: 5 a 15 mA
        ----------------------------------------

          HALL   | Arduino
        ----------------------------------------
           VCC      |  4.5 a 24V
           GND      |  GND
           Shutdown |  Pin D2
           Consumo: 25mA

          HC-SR04 1 | Arduino
        ----------------------------------------
           VCC      |  +5V
           GND      |  GND
           Trig     |  Pin D3
           Echo     |  Pin D4
          Consumo: 15mA

          DHT11     | Arduino
        ----------------------------------------
           VCC      |  3.5 a 5.5V
           GND      |  GND
           Dados    |  Pin D5
         Consumo: 0.3mA (medindo) 60 uA (standby)

        TF Mini Plus| Arduino
        ----------------------------------------
           VCC      |  +5V (Atenção !!! SENSOR sem proteção!!!!)
           GND      |  GND (Atenção !!! SENSOR sem proteção!!!!)
           TX       |  Pin D6
           RX       |  Pin D7  (3.3v)
           Consumo:  550mW (low power mode)
           Consumo: pico 200, média e 140mA
           (Datasheet SJ-GU-TFmini-S-01 A00 Datasheet.pdf)
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
          Consumo: 0.5mA standby e 8.5mA transmitindo.

  --------------------------------------------------------------------------------
  --------------------------------------------------------------------------------
   Consumo de pico máximo esperado:

      1 (TF Mini Plus 200mA) + 3 (VL53L1X 40mA) + 1 (Hall 25mA) +
      1 (HC-SR04 15mA) + 1 (MPU6050 10mA) + 1 (DHT11 1mA) +  1 LED +
      1 (Bluetooth Hc06 9mA)
      = 380mA
  Notas:
  - Acima de 200mA, os sensores devem ser ligados em uma fonte externa.
  - Todas as fontes e ligações do GND devem ser comuns.
  - A inversão da polaridade do TF Mini Plus causará a queima do componente.
*/
// ==============================================================================
// ------------------------ Dependências ------------------------
// ==============================================================================

#include <Arduino.h>               //framework-arduino-avr 5.1.0
#include <MemoryFree.h>            //v0.3.0
#include <Wire.h>                  //v1.0
/*
  Para múltiplas instâncias, é necessário selecionar qual está ativo, com os comandos
  como bluetooth.listen(); tfmini.listen(); Entretanto o BT foi conectado no TX e RX,
  utilizando buffer de hardware.
  O SoftwareSerial restritos por modelo e alguns modelos já tem mais de um serial em
  hardware.
*/
#include <SoftwareSerial.h>       //v1.0
#include <VL53L1X.h>              //v1.3.0
#include <TFMPlus.h>              //v1.5.0

/*
  #include "DHT.h"                  //v1.4.3
  #include <DHT_U.h>
  #include "Adafruit_Sensor.h"      //v1.1.4
*/

// ==============================================================================
// ------------------ Variáveis Ambientais e Constantes -------------------------
// ==============================================================================
/*!
    @defined    cPI
    @abstract   PI used to calculate the circumference of the wheel.
*/
#define cPI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089

/*!
    @const      version 
    @abstract   Version of the release
*/
const PROGMEM double version = 1.3;
/*!
    @volatile   debug 
    @abstract   Enable or disable debug mode
*/
volatile boolean debug = false;

/*!
    @volatile   rawData 
    @abstract   print the raw data of the sensors
*/
volatile boolean rawData = true;

/*!
    @volatile   statistics  
    @abstract   print the statistics data of the sensors
*/
volatile boolean statistics  = false;

/*!
    @const   continuousMode 
    @abstract   set the sensors to continuous Mode
    @discussion f the sensors are not installed with minimum spacing, 
    they must be triggered sequentially.
*/
const PROGMEM boolean continuousMode = true;

/*!
    @volatile   speedCounter  
    @abstract   Enable hall pulse counting.
*/
volatile boolean speedCounter = true;

/*!
    @volatile   distanceCounter  
    @abstract   Enable the distance counting.
*/
volatile boolean distanceCounter = true;

/*!
    @volatile   distanceCounter  
    @abstract   Index do buffer
*/
volatile uint8_t i; 

/*!
    @defined    bufferSZ
    @abstract   Sensor buffer size to store all sensor data
    @discussion Store until the buffer is full, the free memory shoud observed to change the values.
*/
#define bufferSZ 20
int16_t bufferSensors[8][bufferSZ + 1];


/*!
    @defined    deathZoneSensors
    @abstract   Maximum sensor dead zone value
    @discussion Store until the buffer is full, the free memory shoud observed to change the values.
*/
#define deathZoneSensors        2 


/*!
    @defined    heightSensorInstallCm
    @abstract   Valor medido 76.5 a 79 cm (folgas acoplamento)
    @discussion 
*/
#define heightSensorInstallCm  77


/*!
    @defined    heightSensorLimit
    @abstract   Altura medida com a roda erguida a 20 cm
    @discussion 
*/
#define heightSensorLimit     106 


/*!
    @defined    heightSensorLimit
    @abstract   Altura medida com a roda erguida a 20 cm
    @discussion 
*/
#define heightNoPasture    5

/*!
    @defined    heightMinPasture
    @abstract   Limite inferior
    @discussion 
*/
#define heightMinPasture  12

/*!
    @defined    heightMinHerbs
    @abstract   Limite inferior da erva
    @discussion 
*/
#define heightMinHerbs    30

/*!
    @defined    heightMeanHerbs
    @abstract   Altura média da erva daninha
    @discussion 
*/
#define heightMeanHerbs   40

/*!
    @defined    minSpeedMPS
    @abstract   Velocidade mínima de deslocamento
    @discussion Deslocamento mínimo de 0,4 km/h para o equipamento campo limpo
                visa definir a velocidade mínima de operação e o corte do fluxo
                defensivo
*/
#define minSpeedMPS     0.11                  //Deslocamento mínimo de 0,4 km/h


/*!
    @defined    maxSpeedMPS
    @abstract   Velocidade máxima de deslocamento
    @discussion Deslocamento máxima de 11 km/h para o equipamento campo limpo
                visa definir a velocidade máxima de operação e o corte do fluxo
                defensivo
*/
#define maxSpeedMPS  2.78                  

/*!
    @defined    barHeight
    @abstract   Bar height from the ground (m)
    @discussion 
*/
#define barHeight   0.161                 //Distância sensor-aplicador em m

         

/*!
    @defined bufferSZEquipLenght
    @abstract    Size of buffer 
    @discussion  Simple buffer to store the applicator flow and height data 
                 until the applicator is over the read point.
*/
#define bufferSZEquipLenght 7                 // Distância por pulso = 7 slot PWM (161cm/23cm)

/*!
    @defined bufferSZSections
    @abstract    Number of motors or sections 
    @discussion  Data for the number of motors or sections of the applicator
*/
#define bufferSZSections    2


/*!
    @defined bufferPWM
    @volatile    bufferPWM 
    @discussion  Simple buffer to store the applicator flow and height data 
                until the applicator section is over the read point. The size 
                shoud cover the number of setpoints based on the distance 
                between the bar and the aplicator.
*/
volatile uint8_t bufferPWM[bufferSZSections][bufferSZEquipLenght]; //Motores ou setores das cordas


/*!
    @defined wheelRadius
    @abstract    radius Of Wheel (meters)
    @discussion  Used to calculate running distance. 
                 Measure from the middle of the wheel axle to the ground (meters).
*/
#define wheelRadius   0.337 

/*!
    @defined wheelDiameter 
    @abstract    Wheel radius (meters)
    @discussion  Used to calculate running distance.                  
*/
#define wheelDiameter (wheelRadius * 2.0)   // diametro da roda em m

/*!
    @volatile countHallInterrupts
    @abstract  
    @discussion
*/
volatile int countHallInterrupts = 0;     // Number of pulses

volatile unsigned long lastTime = 0;      // last pulse (ms)

volatile unsigned long timeSpeed = 0;     // store the time in ms for printSpeed

// ==============================================================================
//          --------------- Habilitar sensores ----------------
// ==============================================================================
/*boolean VL1Enable = true;
  boolean VL2Enable = true;
  boolean VL3Enable = true;
  boolean VL4Enable = false;

  boolean TF1Enable = true;

  boolean HC1Enable = true;
  boolean HC2Enable = false;
  boolean HC3Enable = false;

  boolean DHT11Enable = true;

  boolean BluetoothEnable = true;

  boolean MPU6050 = true;

  boolean SensorHallEnable = true;
*/
// ==============================================================================
// --------------- Pinagem e temporizações / ajustes dos sensores ----------------
// ==============================================================================

//Bluetooth
//Nome: UGV
#define setBtName "AT+NAMEUGV" 
//Senha PIN: 0000
#define setBtPIN "AT+PIN0000"
//Velocidade BAUD4 = 9600 ,  BAUD8 = 115200
#define setBtSpeed "AT+BAUD8"
volatile int bluetoothData = 0;

//HC-04 (HC01)
#define HC1_TRIG_PIN 3
#define HC1_ECHO_PIN 4

/*
  //HC-04 (HC02)
  #define HC1_TRIG_PIN A6
  #define HC1_ECHO_PIN A7

  //HC-04 (HC03)
  #define HC1_TRIG_PIN 8
  #define HC1_ECHO_PIN 12
*/

/*
  VL53L1X - Intervalo de timeout em 500ms
  Intervalo entre medidas de 50000 us (50 ms), até 30 ms para o modo de
  curta distância.
*/
#define setVLTimeout 500
#define setVLModeShort true  //para 1,3 metros
#define setVLMeasurementTimingBudget 200000
#define setVLContinuous 200

VL53L1X vl1;
#define SHUTDOWN_VL1_PIN A0

VL53L1X vl2;
#define SHUTDOWN_VL2_PIN A1

VL53L1X vl3;
#define SHUTDOWN_VL3_PIN A2

// TF Mini Plus
TFMPlus tfmP;
#define TFMINI_TX_PIN 6
#define TFMINI_RX_PIN 7
SoftwareSerial tfSerial(TFMINI_RX_PIN, TFMINI_TX_PIN); // Fio verde e branco

// MPU6050 - Acelerômetro e giroscópio
const int MPU = 0x68; //Endereco I2C
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Sensor de campo Hall
#define HALL_INTERRUPT_PIN 2
boolean hall = false;

// PWM Motores
#define PWM_M1_PIN 10
#define PWM_M2_PIN 11

// Botão Start/Stop
#define BUTTON_START_STOP_PIN 9

/*
  // DHT11 - Sensor de Umidade e Temperatura
  #define DHT_PIN 5
  #define DHTTYPE DHT11
  DHT dht(DHT_PIN, DHTTYPE);
*/

// LED
#define LED_PIN 13
volatile bool actionMode = false; // Variável de estado

//LDR (PhotoDiode) Sensor de luz
#define LDR_PIN A6   // Pino analógico de entrada do PhotoDiode

// ==============================================================================
// ---------------------------       Functions        ---------------------------
// ==============================================================================
void  setupBluetooth (void);       //Ajusta o Bluetooth (Senha, nome, velocidade)

void  preStartPosStop (void);      //Muda o estado do LED e imprime estados.
void  hallInterrupts (void);       //Interrupção Hall. Levanta uma flag.
void  hallAction(void);            //Ações para a interrupção dentro do loop.

void  readBTCmd (void);            //Lê char (Tag, start/stop, ping, memória livre)
void  bufferOverflowAction (void); //Descarrega o buffer quando cheio

void  setDataBuffer (void);        // Tarefa de ler os sensores e armazenar
void  printBuffer (void);
void  printStatistics (void);
void  printMinMeanMax (int min, float med, int max);
void  printCompressedBuffer (void);           //formato simplificado
void  printSpeed (boolean mps);               // velocidade de deslocamento (m/s ou km/h)
void  printDistance (void);                   //imprime a distância percorrida
void  printVLDetails (int id, uint16_t range, String status, float signalMCPS, float AmbienteMCPS);

long  getDataHC (int trigpin , int echopin);
void  getDataGY (void);
int   getLDRValue (void);
int   checkDistanceValue (boolean msg);       //Verifica mínimo e máximo esperado

void  motorPwm (int m1, int m2);              // Saída motor ou setor após deslocamento

// ==============================================================================
// ----------------------- Mensages / Internacionalization  ---------------------
// ==============================================================================
//#include <avr/pgmspace.h>

//Start Mensages  
const char string_0[] PROGMEM =  ";debug;"; 
const char string_1[] PROGMEM =  ";SS;Ver:";
const char string_2[] PROGMEM =  ";mem free:";
const char string_3[] PROGMEM =  ";mem buffer:";
const char string_4[] PROGMEM =  ";Mod_Cont;";

//Loop Mensages 
const char string_5[] PROGMEM =  ";EOS;";
const char string_6[] PROGMEM =  ";h;";           //hall 
const char string_7[] PROGMEM =  ";BO;";

//Error Mensages                  
const char string_8[] PROGMEM =  ";HC1_RE;";
//const char string_9[] PROGMEM =  ";HC2_RE;";
//const char string_10[] PROGMEM = ";HC3_RE;";
//const char string_11[] PROGMEM = ";HC4_RE;";
const char string_12[] PROGMEM = ";VL1_TO;";
const char string_13[] PROGMEM = ";VL2_TO;";
const char string_14[] PROGMEM = ";VL3_TO;";
//const char string_15[] PROGMEM = ";VL4_TO;";
const char string_16[] PROGMEM = ";VL";
const char string_17[] PROGMEM = "_RE:";
const char string_18[] PROGMEM = "\tstatus: ";
const char string_19[] PROGMEM = "\tpeak signal: ";
const char string_20[] PROGMEM = "\tambient: ";
const char string_48[] PROGMEM = ";TF1_RE: ";

//Statistics Mensages   
const char string_21[] PROGMEM = "\n;Sen ;Min ;Med(";
const char string_22[] PROGMEM = ");Max;\n;TF1";
const char string_23[] PROGMEM = ";HC1";
//const char string_24[] PROGMEM = ";HC2";
//const char string_25[] PROGMEM = ";HC3";
//const char string_26[] PROGMEM = ";HC4";
const char string_27[] PROGMEM = ";VL1";
const char string_28[] PROGMEM = ";VL2";
const char string_29[] PROGMEM = ";VL3";
//const char string_30[] PROGMEM = ";VL4";

//Menu Mensages  
const char string_31[] PROGMEM = ";ON;";
const char string_32[] PROGMEM = ";OFF;";
const char string_33[] PROGMEM = ";DI;";
const char string_34[] PROGMEM = ";DF;";
const char string_35[] PROGMEM = ";ping;";
const char string_36[] PROGMEM = ";debug;";
const char string_37[] PROGMEM = ";statistics;";
const char string_38[] PROGMEM = ";rawData;";

//Log Mensages  
const char string_39[] PROGMEM = ";start;";
const char string_40[] PROGMEM = ";ID;HC1;VL1;VL2;VL3;TF1;GyX;GyY;GyZ;";
const char string_41[] PROGMEM = ";stop;";

//Movement log Mensages  
const char string_42[] PROGMEM = ";V=";
const char string_43[] PROGMEM = " m/s;";
const char string_44[] PROGMEM = " Km/h;";
const char string_45[] PROGMEM = " Mph;";
const char string_46[] PROGMEM = ";d=";
const char string_47[] PROGMEM = " m;";




//const char string_49[] PROGMEM = ;
//const char string_50[] PROGMEM = ;
//const char string_51[] PROGMEM = ;


/*
Before PROGMEM
RAM:   [========= ]  86.9% (used 1780 bytes from 2048 bytes)
Flash: [======    ]  61.3% (used 18818 bytes from 30720 bytes)

After PROGMEM
RAM:   [=======   ]  70.7% (used 1448 bytes from 2048 bytes)
Flash: [======    ]  61.3% (used 18822 bytes from 30720 bytes)

*/

/////////////////////////////////////////////////////////////////////////////////
void setup( ) {
/////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////////
  //  Serial
  /////////////////////////////////////////////////////////////////////////////////

  Serial.begin(9600); //Bauds
 if (debug)
  {
    Serial.println(string_0); //";debug;
    //Mensagens e modos de operação
    Serial.print(string_1); //"Starting Setup" ";SS;Ver:"
    Serial.print(double(version));
    Serial.print(string_2); //";mem free:"
    Serial.print(freeMemory()); Serial.println(";");

    Serial.print(string_3); //";mem buffer:"
    Serial.print(int(sizeof(bufferSensors))); Serial.println(";");
    if (continuousMode) Serial.println(string_4); //";Mod_Cont;"
}

  /////////////////////////////////////////////////////////////////////////////////
  //  I2C
  /////////////////////////////////////////////////////////////////////////////////

  Wire.begin();
  Wire.setClock(400000); //I2C à 400 kHz


  /////////////////////////////////////////////////////////////////////////////////
  //  TF1 - TF Mini Plus
  /////////////////////////////////////////////////////////////////////////////////

  tfSerial.begin( 115200 );
  delay(20);
  tfmP.begin( &tfSerial);
  tfmP.sendCommand( SOFT_RESET, 0);
  if (continuousMode)
    tfmP.sendCommand( SET_FRAME_RATE, FRAME_20);// FRAME_20 para contínuo
  else
    tfmP.sendCommand( SET_FRAME_RATE, FRAME_0);// FRAME_0 não contíno


  /////////////////////////////////////////////////////////////////////////////////
  //  HC1 - HC-SR04
  /////////////////////////////////////////////////////////////////////////////////

  pinMode( HC1_TRIG_PIN, OUTPUT );   //HC-04 trig
  pinMode( HC1_ECHO_PIN, INPUT  );   //HC-04 echo


  /////////////////////////////////////////////////////////////////////////////////
  // VL1, VL2 e VL3 - Sensores ToF VL53L1X
  /////////////////////////////////////////////////////////////////////////////////

  pinMode( SHUTDOWN_VL1_PIN, OUTPUT );    //Desliga VL1
  digitalWrite(SHUTDOWN_VL1_PIN, LOW);
  pinMode( SHUTDOWN_VL2_PIN, OUTPUT  );   //Desliga VL2
  digitalWrite(SHUTDOWN_VL2_PIN, LOW);
  pinMode( SHUTDOWN_VL3_PIN, OUTPUT  );   //Desliga VL3
  digitalWrite(SHUTDOWN_VL3_PIN, LOW);

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

  if (setVLModeShort) {
    vl1.setDistanceMode(VL53L1X::Short);
    vl2.setDistanceMode(VL53L1X::Short);
    vl3.setDistanceMode(VL53L1X::Short);
  }

  //Intervalo entre medidas de 50000 us (50 ms), até 30 ms para short
  vl1.setMeasurementTimingBudget(setVLMeasurementTimingBudget);
  vl2.setMeasurementTimingBudget(setVLMeasurementTimingBudget);
  vl3.setMeasurementTimingBudget(setVLMeasurementTimingBudget);

  //Medição VL (ms)
  if (continuousMode) {
    vl1.startContinuous(setVLContinuous);
    vl2.startContinuous(setVLContinuous);
    vl3.startContinuous(setVLContinuous);
  }
  else
  {
    vl1.stopContinuous();
    vl2.stopContinuous();
    vl3.stopContinuous();
  }


  /////////////////////////////////////////////////////////////////////////////////
  //  I2C Giroscópio e acelerômetro
  /////////////////////////////////////////////////////////////////////////////////

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);


  /////////////////////////////////////////////////////////////////////////////////
  //  LED onboard D13 - Indica se está capturando.
  /////////////////////////////////////////////////////////////////////////////////

  pinMode(LED_PIN, OUTPUT);


  /////////////////////////////////////////////////////////////////////////////////
  //  Pushbutton
  /////////////////////////////////////////////////////////////////////////////////

  pinMode(BUTTON_START_STOP_PIN, INPUT_PULLUP);


  /////////////////////////////////////////////////////////////////////////////////
  //  Interrupção
  /////////////////////////////////////////////////////////////////////////////////

  pinMode(HALL_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_INTERRUPT_PIN), hallInterrupts, RISING);


  /////////////////////////////////////////////////////////////////////////////////
  // PWM - Motores ou setores
  /////////////////////////////////////////////////////////////////////////////////
  
  pinMode(PWM_M1_PIN, OUTPUT);
  pinMode(PWM_M2_PIN, OUTPUT);

  /*
    /////////////////////////////////////////////////////////////////////////////////
    // Bluetooth - Velocidade, Nome, PIN
    /////////////////////////////////////////////////////////////////////////////////

    // setupBluetooth ();


    /////////////////////////////////////////////////////////////////////////////////
    // DHT11 - Umidade e temperatura
    /////////////////////////////////////////////////////////////////////////////////
    dht.begin();
    Serial.print(F(";Umid: "));
    Serial.print(int(dht.readHumidity()));
    Serial.print(F("%  Temp: "));
    Serial.print(int(dht.readTemperature()));
    Serial.println(F("°C;"));
  */


  /////////////////////////////////////////////////////////////////////////////////
  // LDR
  /////////////////////////////////////////////////////////////////////////////////
  pinMode(LDR_PIN, OUTPUT);

  i = 0; //index do buffer dos sensores
  //Serial.print(";mem:"); Serial.print(freeMemory());Serial.println(";");
  if (debug) Serial.println(string_5);  //Mensagem "End of setup" ";EOS;"

}

/*
  ==============================================================================
  ------------------------------------ Loop ------------------------------------
  ==============================================================================
*/
void loop( ) {
  


  // Controle de Start/Stop pelo botão
  if (digitalRead(BUTTON_START_STOP_PIN) == LOW) // Se o botão for pressionado
  {
    preStartPosStop( ); //ações de start/stop
    while (digitalRead(BUTTON_START_STOP_PIN) == LOW) {}
    delay(100);
  }

  //Controle de Start/Stop pelo bluetooth ou terminal 0 para 1 inicia
  if (Serial.available()) {
    readBTCmd( );
    Serial.flush();    
  }


  //Estado de execução do sistema
  if (actionMode) { //Captura dados se Ligado

    setDataBuffer (); //Lê os sensores de distância e salva no buffer
    checkDistanceValue(debug);
    i++; //Incrementa o buffer

    bufferOverflowAction(); //Tratamento para estouro
    hallAction();           //Tratamento para interrupção
  }
}//fim do loop()


void hallAction(){
 if (hall) {
      if (debug) Serial.println(string_6); //hall ";h;"
      
      hall = false;
      countHallInterrupts++;
      interrupts();
      
      if (rawData)        printBuffer( );
      if (statistics )    printStatistics( );
      i = 0; //Zera Índice do buffer      
    }
}


void bufferOverflowAction(){

  if (i >= bufferSZ) {
      if (debug)          Serial.println (string_7); //Buffer overflow ";BO;"
      if (rawData)        printBuffer( );
      if (statistics )    printStatistics( );
      i = 0; //Zera Índice do buffer
    }
}

/*
  Realiza  a gravação dos valores lidos para cm da distância.
  Usa o inteiro i como indexador da posição. Não realiza testes para validar.
*/
void setDataBuffer ( ) {

  bufferSensors[0][i] = int16_t (getDataHC(HC1_TRIG_PIN , HC1_ECHO_PIN )); //HC-SR04 01

  if (continuousMode) {
    bufferSensors[1][i] = int16_t (vl1.read() / 10); //VL 01
    bufferSensors[2][i] = int16_t (vl2.read() / 10); //VL 02
    bufferSensors[3][i] = int16_t (vl3.read() / 10); //VL 03
    tfmP.getData(bufferSensors[4][i]);//TF Mini Plus
  }
  else
  {
    tfmP.sendCommand( SET_FRAME_RATE, FRAME_0);

    bufferSensors[1][i] = int16_t (vl1.readSingle() / 10); //VL 01
    bufferSensors[2][i] = int16_t (vl2.readSingle() / 10); //VL 02
    bufferSensors[3][i] = int16_t (vl3.readSingle() / 10); //VL 03

    tfmP.sendCommand( SET_FRAME_RATE, FRAME_20);// FRAME_20
    tfmP.getData(bufferSensors[4][i]);//TF Mini Plus
    tfmP.sendCommand( SET_FRAME_RATE, FRAME_0);
  }

  getDataGY();
  bufferSensors[5][i] = GyX;
  bufferSensors[6][i] = GyY;
  bufferSensors[7][i] = GyZ;
}

/*!  
  @function
  @abstract
  @discussion Validate the values read by the distance sensors and returns the sum of errors.
              The msg parameter define whether the error will be printed.
  @param      msg[in]    
  @result     -1  for HC1 - Sensor HC-SR04
              -2  for VL1 - Sensor VL53L1X
              -4  for VL2 - Sensor VL53L1X
              -8  for VL3 - Sensor VL53L1X
              -16 for TF1 - Sensor TF Mini Plus
*/
int checkDistanceValue (boolean msg) {

  volatile int8_t errorCode = 0;

  // range error HC1 (HC-SR04)
  if (bufferSensors[0][i] >= heightSensorLimit || bufferSensors[0][i] <= deathZoneSensors)
  {
    if (msg) Serial.println(string_8); //";HC1_RE;"
    errorCode = -1;
  }

  if (vl1.timeoutOccurred()) if (msg) Serial.println(string_12); // TIMEOUT ";VL1_TO;"
  if (vl2.timeoutOccurred()) if (msg) Serial.println(string_13); // TIMEOUT ";VL2_TO;"
  if (vl3.timeoutOccurred()) if (msg) Serial.println(string_14); // TIMEOUT ";VL3_TO;"

  // range error VL1 (VL53L1X)
  if (bufferSensors[1][i] >= heightSensorLimit || bufferSensors[1][i] <= deathZoneSensors)
  {
    if (msg)
    {
      printVLDetails(1,
                     vl1.ranging_data.range_mm,
                     VL53L1X::rangeStatusToString(vl1.ranging_data.range_status),
                     vl1.ranging_data.peak_signal_count_rate_MCPS ,
                     vl1.ranging_data.ambient_count_rate_MCPS
                    );                    
    }
    errorCode = errorCode  - 2;
  }

  // range error VL2 (VL53L1X)
  if (bufferSensors[2][i] >= heightSensorLimit || bufferSensors[2][i] <= deathZoneSensors)
  {
    if (msg) {
      printVLDetails(2,
                     vl2.ranging_data.range_mm,
                     VL53L1X::rangeStatusToString(vl2.ranging_data.range_status),
                     vl2.ranging_data.peak_signal_count_rate_MCPS,
                     vl2.ranging_data.ambient_count_rate_MCPS);
    }
    errorCode = errorCode - 4;
  }

  // range error VL3 (VL53L1X)
  if (bufferSensors[3][i] >= heightSensorLimit || bufferSensors[3][i] <= deathZoneSensors)
  {
    if (msg)
    {
      printVLDetails(3,
                     vl3.ranging_data.range_mm,
                     VL53L1X::rangeStatusToString(vl3.ranging_data.range_status),
                     vl3.ranging_data.peak_signal_count_rate_MCPS,
                     vl3.ranging_data.ambient_count_rate_MCPS);
    }
    errorCode = errorCode - 8;
  }

  // range error TF1 (TF Mini Plus)
  if (bufferSensors[4][i] >= heightSensorLimit || bufferSensors[4][i] <= deathZoneSensors)
  { if (msg) {
      Serial.print(string_48);// ";TF1_RE: "
      tfmP.printReply();
      Serial.println(";");
    }
    errorCode = errorCode - 16;
  }

  return errorCode;
}

void printVLDetails(int id , uint16_t range, String status, float signalMCPS, float AmbienteMCPS) {
  
  Serial.print(string_16); // ";VL"
  Serial.print(int(id));
  Serial.print(string_17); // "_RE:"
  Serial.print(range);
  Serial.print(string_18); // "\tstatus: "
  Serial.print(status);
  Serial.print(string_19); // "\tpeak signal: "
  Serial.print(signalMCPS);
  Serial.print(string_20); // "\tambient: "
  Serial.print(AmbienteMCPS);
  Serial.println(";");
}



/*!
    @function
    @abstract   Interruption activated by the hall sensor
    @discussion If in operation mode, enable action in the loop.
*/
void hallInterrupts( ) {

  if (actionMode) { //captura ligada    
    hall = true;
    noInterrupts();
  }
}

/*
  @function
  @abstract   Define o PWM dos motores em construção.
  @discussion Usar MAP,
              Em testes:
              Existe um valor mínimo para iniciar o movimento.
              Existe um valor mínimo menor para manter o movimento.
              Existe um valor adicional para acelerar.
              Existe um valor adicional para desacelerar e manter o movimento.
              Bug quando se reseta, dependendo do estado 
              (precisa setar em 0 como o primeiro passso no SETUP).
    @param      m1[in]     Control the Motor 1
    @param      m2[in]     Control the Motor 2
*/
void motorPwm(int m1, int m2) {

  //adicionar limitadores min e max
  analogWrite( PWM_M1_PIN, m1);
  analogWrite( PWM_M2_PIN, m2);
}

/*!
    @function
    @abstract Distance with the HC-SR04 Sensor
    @discussion Use a TTL pulse to calc the distance without temperature calibration. 
                It consider the speed of air as 340 m/s. 
    @param      trigpin[in] PIN of trigger
    @param      echopin[in] PIN of echo
    @result     The distance in cm
  
*/
long getDataHC (int trigpin , int echopin) {

  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite( trigpin, HIGH );
  delayMicroseconds( 10 ); // 10µS TTL pulse (0,01ms)
  digitalWrite( trigpin, LOW );
  int interval = pulseIn( echopin, HIGH );

  return interval * 0.017; //cm considera a velocidade do som no ar de 340m/s
}

/*
  Função para ler os dados giroscópio e acelerômetro. (MPU)
  Salva os 3 eixos
*/
void getDataGY( ) {

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
/*
  Função para imprimir os dados do buffer (matriz [8][i],
  onde i são as linhas armazenadas até o momento).
*/
void printBuffer( ) {

  for (int cont = 0;  cont < i ; cont++) {
    Serial.print(";");  Serial.print(cont);  Serial.print(";");//ID;
    for (int j = 0; j <= 7; j++ ) {
      Serial.print(bufferSensors[j][cont]);  Serial.print(";");//Sensores;
    }
    Serial.println("");
  }
}


/*
  Função para analisar os dados do buffer e imprimir o valor mínimo, médio e máximo
  da distância.
  Configurado para 5 sensores de distância, indo da posição 0 a 4
*/
void printStatistics( ) {

  volatile int16_t   min[5]; //Valor mínimo
  volatile int16_t   max[5]; //Valor máximo
  volatile double    med[5]; //Valor médio

  //Define o início
  for (int j = 0 ; j < 5 ; j++) {
    min[j] = bufferSensors[j][0];
    max[j] = bufferSensors[j][0];
    med[j] = double(bufferSensors[j][0] / (i));
   
  }

  //Acha o min, med e max
  for (int cont = 1;  cont < i ; cont++) {
    for (int j = 0 ; j < 5 ; j++) {
      min[j] = min(min[j], bufferSensors[j][cont] );
      med[j] = float (med[j] + (bufferSensors[j][cont]));
      max[j] = max(bufferSensors[j][cont] , max[j]);
    }
  }
  //Gera as médias
  for (int j = 0 ; j < 5; j++) med[j] = med[j] / i;

  //Imprime
  Serial.print(string_21);   Serial.print(i); //"\n;Sen ;Min ;Med("
  
  Serial.print(string_22);  //");Max;\n;TF1"
  printMinMeanMax (min[4], med[4], max[4]);
  
  Serial.print(string_23);          printMinMeanMax (min[0], med[0], max[0]);
  Serial.print(string_27);          printMinMeanMax (min[1], med[1], max[1]);
  Serial.print(string_28);          printMinMeanMax (min[2], med[2], max[2]);
  Serial.print(string_29);          printMinMeanMax (min[3], med[3], max[3]);

}


/*
  Função auxiliar para imprimir os valores min med max
*/
void printMinMeanMax (int min, float med, int max) {
  Serial.print(";\t");
  Serial.print(min);          Serial.print(";\t");
  Serial.print((med));        Serial.print(";\t");
  Serial.print(max);          Serial.println(";");
}

/*
  Função para definir ajustes no módulo Bluetooth:
    velocidade,
    nome e a
    senha PIN.
*/
void setupBluetooth ( ) {

  //Serial.begin(9600);
  delay(5000);
  Serial.print(setBtName);
  delay(5000);
  Serial.print(setBtPIN);
  delay(5000);
  Serial.print(setBtSpeed);
  delay(5000);
  //Serial.begin(115200);
}


/*
  Menu de opções pelo terminal ou Bluetooth

  0 - Interrompe o experimento.   Imprime ";ON;"
  1 - Iniciar o experimento.       Imprime ";OFF;"

  i - Indica a direção de início do ponto marcado. Imprime ";Dir_INI;"
  f - Indica a direção do final  do ponto marcado. Imprime ";Dir_FIM;"

  d - debug ON
  p - ping para teste. Imprime ";ping;"
  m - memória livre. Imprime ";mem:; QTD;"
  e - estatísticas. Imprime um resumo do buffer (min, med, max)
  r - imprime a rawData

*/
void readBTCmd( ) {
  if (Serial.available())
  {
    bluetoothData = Serial.read();

    switch (bluetoothData) {
      case '1':
        if (!actionMode) {
          Serial.println(string_31); //";ON;"
          actionMode = false;
          preStartPosStop( );
        }
        break;

      case '0':
        if (actionMode) {
          actionMode = true;
          preStartPosStop( );
          Serial.println(string_32); //";OFF;"
        }
        break;

      case 'i':
        Serial.println(string_33); //Dir_INI; ";DI;"
        break;

      case 'f':
        Serial.println(string_34); //Dir_INI; ";DF;"
        break;

      case 'm':
        Serial.print(string_2); //";mem:"
        Serial.print(freeMemory());
        Serial.println(";");
        break;

      case 'p':
        Serial.println(string_35); //";ping;"
        break;

      case 'd':
        debug = !debug;
        Serial.print(string_36); //";debug;"
        Serial.print(debug);
        Serial.println(";");
        break;

      case 'e':
        statistics  = !statistics ;
        Serial.print(string_37); //";statistics;"
        Serial.print(statistics);
        Serial.println(";");
        break;

      case 'r':
        rawData = !rawData;
        Serial.print(string_38); //";rawData;"
        Serial.print(rawData);
        Serial.println(";");
        break;

      default:
        // comando(s)
        break;
    }
  }
  Serial.flush();
}


void preStartPosStop( ) {

  actionMode = !actionMode; // altera o estado do LED
  if (actionMode) {
    //Pré execução
    Serial.println (string_39); //";start;"
    if (rawData) Serial.println (string_40); //";ID;HC1;VL1;VL2;VL3;TF1;GyX;GyY;GyZ;"
    if (speedCounter) countHallInterrupts = 0;
    if (distanceCounter ) countHallInterrupts = 0;
  }
  else {
 
    //Pós execução
    Serial.println (string_41); //";stop;"  
    if (i > 0 ) {
      if (rawData)        printBuffer( );
      if (statistics )    printStatistics( );
      i = 0;
    }
    hallAction();  
    if (distanceCounter)  printDistance( );
    if (speedCounter)     printSpeed(true); // true = m/s, false = km/h         
  }
  
  digitalWrite(LED_PIN, actionMode);

}


void printCompressedBuffer( ) {

  for (int j = 0; j <= 7; j++ ) {
    Serial.print(bufferSensors[j][0]);  Serial.print(";");//Sensores Posição 0;
  }
  Serial.println("");

  for (int cont = 1;  cont < i ; cont++) {
    //Serial.print(";");  Serial.print(cont);  Serial.print(";");//ID;


    for (int j = 0; j <= 7; j++ ) {
      Serial.print(bufferSensors[j][0] - bufferSensors[j][cont]);  Serial.print(";");//Variação dos Sensores;
    }
    Serial.println("");
  }

}

void printSpeed(boolean mps) {

  double speed = 0.0;
  speed = ((cPI * wheelDiameter * (countHallInterrupts / 9) * 1000.0) / ((millis() - timeSpeed) * 4) );
  timeSpeed = millis();
  countHallInterrupts = 0;

  Serial.print (string_42); // ";V="
  if (mps) { // m/s
    Serial.print (speed);
    Serial.println (string_43); // " m/s;"
  }
  else { // km/h
    Serial.print (speed * 3.6);
    Serial.println (string_44); //" Km/h;"
  }
}


void printDistance( ) {

  double distance = 0.0;
  distance = countHallInterrupts * 0.23;

  Serial.print (string_46); //";d="
  Serial.print (distance);
  Serial.println (string_47); // " m;"
}

int getLDRValue( ) {
  float analogVal = (float)analogRead(LDR_PIN);
  int result =  (int)(analogVal / 40.95);   // Converte para %
  //Adicionar uma relação entre os lumens do luximetro e a curva do LDR

  return result;
}

