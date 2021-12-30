/* ---------------------------------------------------------------------
    Autor: Eugênio Pierazzoli
------------------------------------------------------------------------
  Componentes:
    1 Arduino Nano (Old Bootloader 328P)
    (ATMEGA328P 16MHz, 2 KB RAM, 30 KB Flash)

    1 Sensor de Ultrassônico HC-SR04
    1 Giroscópio (I2C)
    1 Bluetooth
    4 VL53L1X (I2C)
    1 TF Mini Plus
    1 Hall
    1 DHT11
    1 Push Button
    1 LED
    2 Motores PWM

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
         Consumo: 0.3mA (measuring) 60uA (standby)

        TF Mini Plus| Arduino
        ----------------------------------------
           VCC      |  +5V (Atenção !!! SENSOR sem proteção!!!!)
           GND      |  GND (Atenção !!! SENSOR sem proteção!!!!)
           TX       |  Pin D6
           RX       |  Pin D7  (3.3v)
           Consumo:  550mW (low power mode)
           Consumo: pico 200 média e 140mA 
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
   Soma do consumo de pico esperado:

      1 (TF Mini Plus 200mA) + 3 (VL53L1X 40mA) + 1 (Hall 25 mA) + 
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
//#include <avr/wdt.h>
#include <Wire.h>                  //v1.0
/*
Para múltiplas instâncias, é necessário selecionar qual está ativo, 
com os comandos bluetooth.listen(); tfmini.listen(); Entretanto o BT foi conectado 
no TX e RX, tirando proveito do buffer de hardware.
O SoftwareSerial restritos por modelo e alguns modelos já tem mais de um serial em hardware.
*/
#include <SoftwareSerial.h>       //v1.0
#include <VL53L1X.h>              //v1.3.0
#include <TFMPlus.h>              //v1.5.0

#include "DHT.h"                  //v1.4.3
#include "Adafruit_Sensor.h"      //v1.1.4

// ==============================================================================
// ------------------ Variáveis Ambientais e Constantes -------------------------
// ==============================================================================
int8_t i;

#define bufferSZ 20
int16_t bufferSensores[8][bufferSZ + 1];

#define zonaMortaSensorCm 2 //Menor valor de zona morta dos sensores de altura
#define alturaInstSensorescm 77 //valor medido 76.5 a 79 cm (folgas de acoplamento)
#define alturaLimiteCm 106 // Altura medida quando a roda é erguida a 20 cm

#define alturaSemGramainiea 5
#define alturaPastagemDesejada 12 //Limite inferior
#define alturaPastagemIndesejadaMedia 30
#define alturaPastagemDesejadaAlta 40

#define velocidadeLimiteMaxMetrosPorSegundo 1.1
#define velocidadeLimiteMinMetrosPorSegundo 0.1

// Distancia total de 161cm / 23cm por pulso = 7 slot pwm
#define bufferSZDistanciaAplicador 7
int16_t bufferPWM[2][7] = {0, 0, 0, 0, 0, 0, 0}; //motores ou setores das cordas

boolean debug = false;

// ==============================================================================
// -----------------Pinagem, temporizações e ajustes dos sensores----------------
// ==============================================================================

//Bluetooth
//Nome: UGV
#define setNomeBluetooth "AT+NAMEUGV"
//Senha PIN: 0000
#define setPINBluetooth "AT+PIN0000"

//Velocidade BAUD4 = 9600 ,  BAUD8 = 115200
#define setVelocidadeBluetooth "AT+BAUD8"
int dadoBluetooth = 0;

//HC-04 01
#define HC1_TRIG_PIN 3
#define HC1_ECHO_PIN 4

/*
  VL53L1X - Intervalo de timeout em 500ms
  Intervalo entre medidas de 50000 us (50 ms), até 30 ms para short
*/
#define setVLTimeout 500
#define setVLModeShort true
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
SoftwareSerial tfSerial( TFMINI_RX_PIN, TFMINI_TX_PIN); //verde e branco

// MPU6050 - Giroscópio 
const int MPU = 0x68; //Endereco I2C
//Variaveis para armazenar valores do acelerometro e giroscópio
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Hall
#define HALL_INTERRUPT_PIN 2
boolean hallInterrupt = false;

// PWM Motores
#define PWM_M1_PIN 10
#define PWM_M2_PIN 11

// Botão Start/Stop
#define BUTTON_START_STOP_PIN 9


// DHT11
#define DHT_PIN 5
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);


// LED
#define LED_PIN 13
bool estadoLED = 0; // variavel de controle

// ==============================================================================
// ----------------------- Protótipo das Funções  -----------------------
// ==============================================================================
void  setupBluetooth (void);      //Ajusta o Bluetooth (Senha, nome, velocidade)

void  preStartPosStop(void);      //Muda o estado do LED e imprime estados.
void  interrupcao (void);         //Interrupção Hall. Gera flag.
void  lerComandosBluetooth (void);//Lê char (Tag, start/stop, ping, memória livre)

void  lerSensoresGravarNoBuffer (void);   // Tarefa de ler os sensores e armazenar
void  imprimirBuffer(void);
void  imprimirStats(void);
void  printMinMedMax (int min, int med, int max);

long  sensorHC (int trigpin , int echopin);
void  giroscopio (void);
int   validarLeituraDistancia (boolean msg); //Verifica mínimo e máximo esperado

void  motorPwm (int m1, int m2);       // Saída motor ou setor após deslocamento



// ==============================================================================
// ----------------------- Setup -----------------------
// ==============================================================================
void setup( ) {

  //Serial
  Serial.begin(9600); //BT modificado para 115200
  Serial.println(";SS;"); //Starting Setup

  //Inicializa o I2C
  Wire.begin();
  Wire.setClock(400000); // usa I2C a 400 kHz

  //HC1
  pinMode( HC1_TRIG_PIN, OUTPUT );   //HC-04 trig
  pinMode( HC1_ECHO_PIN, INPUT  );   //HC-04 echo

  //VL 53L1X
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

  //Taxa Medição VL (ms)
  vl1.startContinuous(setVLContinuous);
  vl2.startContinuous(setVLContinuous);
  vl3.startContinuous(setVLContinuous);

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
  tfmP.sendCommand( SET_FRAME_RATE, FRAME_10);// FRAME_20

  //LED onboard D13
  pinMode(LED_PIN, OUTPUT);

  //Pushbutton
  pinMode(BUTTON_START_STOP_PIN, INPUT_PULLUP);

  //HallInt
  pinMode(HALL_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_INTERRUPT_PIN), interrupcao, RISING);

  //PWM
  pinMode(PWM_M1_PIN, OUTPUT);
  pinMode(PWM_M2_PIN, OUTPUT);

  /*
    // DHT
    dht.begin();
  

  // Bluetooth - Velocidade, Nome, PIN
  //setupBluetooth ();
*/

  i = 0; //index do buffer dos sensores
  Serial.println(";EOS;");//End of setup
  //wdt_enable (WDTO_2S);  // Se tiver algum processo maior que este tempo, terá que ser alterado
}

// ==============================================================================
// ------------ Loop -----------------------
// ==============================================================================
void loop( ) {
  //wdt_reset ();

  // Controle de Start/Stop pelo botão
  if (digitalRead(BUTTON_START_STOP_PIN) == LOW) // Se o botão for pressionado
  {
    preStartPosStop( ); //ações de start/stop
    while (digitalRead(BUTTON_START_STOP_PIN) == LOW) {};
    delay(100);
  }

  //Controle de Start/Stop pelo bluetooth ou terminal 0 para 1 inicia
  if (Serial.available()) {
    lerComandosBluetooth( );
  }
  Serial.flush();//teste

  //Estado de execução do sistema
  if (estadoLED) { //captura dados se Ligado

    lerSensoresGravarNoBuffer (); //Lê os sensores de distância e salva no buffer
    validarLeituraDistancia(debug);
    i++; //Indice do buffer

    if (i >= bufferSZ) {
      Serial.println (";BO;"); //Buffer overflow

      if (debug) imprimirBuffer( );
      imprimirStats( );

      i = 0; //Indice do buffer
    }

    if (hallInterrupt) {
      Serial.println(";h;"); //hall
      hallInterrupt = false;
      if (debug) imprimirBuffer( );
      imprimirStats( );
      i = 0; //Indice do buffer
      interrupts();
    }
  }
}//fim do loop()


/*
  Realiza  a gravação dos valores lidos para cm da distância.
  Usa o inteiro i como indexador da posição. Não realiza testes para validar.
*/
void lerSensoresGravarNoBuffer ( ) {

  bufferSensores[0][i] = int16_t (sensorHC(HC1_TRIG_PIN , HC1_ECHO_PIN )); //HC-SR04 01

  bufferSensores[1][i] = int16_t (vl1.read() / 10); //VL 01
  bufferSensores[2][i] = int16_t (vl2.read() / 10); //VL 02
  bufferSensores[3][i] = int16_t (vl3.read() / 10); //VL 03

  tfmP.getData(bufferSensores[4][i]);//TF Mini Plus

  giroscopio();
  bufferSensores[5][i] = GyX;
  bufferSensores[6][i] = GyY;
  bufferSensores[7][i] = GyZ;
}

/*
  Valida os valores lidos e retorna o somatório de erros
  para valores fora do esperado ou sem resposta.

  Retorno:
    -1 para HC1 - Sensor HC-SR04
    -2 para VL1 - Sensor VL53L1X
    -4 para VL2 - Sensor VL53L1X
    -8 para VL3 - Sensor VL53L1X
    -16 para TF1 - Sensor TF Mini Plus
*/
int validarLeituraDistancia (boolean msg) {

  int resultado = 0;
  if (bufferSensores[0][i] >= alturaLimiteCm || bufferSensores[0][i] <= zonaMortaSensorCm)
  {
    if (msg) Serial.println(";HC1_RE;");// range error HC1 (HC-SR04)
    resultado = -1;
  }

  if (vl1.timeoutOccurred()) if (msg) Serial.println(";VL1_TO;"); // TIME OUT
  if (vl2.timeoutOccurred()) if (msg) Serial.println(";VL2_TO;"); // TIME OUT
  if (vl3.timeoutOccurred()) if (msg) Serial.println(";VL3_TO;"); // TIME OUT

  if (bufferSensores[1][i] >= alturaLimiteCm || bufferSensores[1][i] <= zonaMortaSensorCm)
  { if (msg)Serial.println(";VL1_RE;");// range error VL1 (VL53L1X)
    resultado = resultado  - 2;
  }

  if (bufferSensores[2][i] >= alturaLimiteCm || bufferSensores[2][i] <= zonaMortaSensorCm)
  { if (msg)Serial.println(";VL2_RE;");// range error VL2 (VL53L1X)
    resultado = resultado - 4;
  }

  if (bufferSensores[3][i] >= alturaLimiteCm || bufferSensores[3][i] <= zonaMortaSensorCm)
  { if (msg)Serial.println(";VL3_RE;");// range error VL3 (VL53L1X)
    resultado = resultado - 8;
  }

  if (bufferSensores[4][i] >= alturaLimiteCm || bufferSensores[4][i] <= zonaMortaSensorCm)
  { if (msg)Serial.println(";TF1_RE;");// range error TF1 (TF Mini Plus)
    resultado = resultado - 16;
  }

  return resultado;
}


/*
  Verifica o estado da execução do código e indica a separação de dados por setor por
  meio da interrupção do sensor hall da roda.
*/
void interrupcao( ) {

  // detachInterrupt (HALL_INTERRUPT_PIN);
  if (estadoLED) { //captura ligada
    hallInterrupt = true;
    noInterrupts();
  }
}

/*
  Define o PWM dos motores em construção.
  Usar MAP,
  Em testes:
  Existe um valor mínimo para iniciar o movimento.
  Existe um valor mínimo menor para manter o movimento.
  Existe um valor adicional para acelerar.
  Existe um valor adicional para desacelerar e manter o movimento.
  Bug quando se reseta, dependendo do estado (precisa setar em 0 como o primeiro passso no SETUP).
*/
void motorPwm(int m1, int m2) {

  //adicionar limitadores min e max
  analogWrite( PWM_M1_PIN, m1);
  analogWrite( PWM_M2_PIN, m2);
}

/*
  Função para ler um ou mais Sensores HC-SR04,
  retorna a distância em cm.
*/
long sensorHC (int trigpin , int echopin) {

  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite( trigpin, HIGH );
  delayMicroseconds( 10 ); // 10µS TTL pulse (0,01ms)
  digitalWrite( trigpin, LOW );
  int interval = pulseIn( echopin, HIGH );

  return interval * 0.017; //cm considera a velocidade do som no ar de 340m/s
}

/*
  Função para ler os dados Gyroscópio e acelerômetro. MPU
  Salva os 3 eixos
*/
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
/*
  Função para imprimir os dados do buffer (matriz [8][i],
  onde i são as linhas armazenadas até o momento).
*/
void imprimirBuffer( ) {

  for (int cont = 0;  cont < i ; cont++) {
    Serial.print(";");
    Serial.print(cont);                   Serial.print(";");//ID;
    for (int j = 0; j <= 7; j++ ) {
      Serial.print(bufferSensores[j][cont]);      Serial.print(";");//Sensores;
    }
    Serial.println("");
  }
}


/*
  Função para analisar os dados do buffer e imprimir o valor mínimo, médio e máximo
  da distância.
  Configurado para 5 sensores de distância, indo da posição 0 a 4
*/
void imprimirStats( ) {

  int16_t min[5]; //Valor mínimo
  int16_t max[5]; //Valor máximo
  int16_t med[5]; //Valor médio

  //Define o início
  for (int j = 0 ; j < 5 ; j++) {
    min[j] = bufferSensores[j][0];
    max[j] = bufferSensores[j][0];
    med[j] = bufferSensores[j][0];
  }

  //Acha o min, med e max
  for (int cont = 1;  cont < i ; cont++) {
    for (int j = 0 ; j < 5 ; j++) {
      min[j] = min(min[j], bufferSensores[j][cont] );
      med[j] = med[j] + bufferSensores[j][cont];
      max[j] = max(bufferSensores[j][cont] , max[j]);
    }
  }
  //Gera as médias
  for (int j = 0 ; j < 5; j++) med[j] = med[j] / i;

  //Imprime
  Serial.print("\n;Sensor ;Min ;Med(");   Serial.print(i);
  Serial.print(");Max;\n;TF1"); printMinMedMax (min[4], med[4], max[4]);
  Serial.print(";HC1");          printMinMedMax (min[0], med[0], max[0]);
  Serial.print(";VL1");          printMinMedMax (min[1], med[1], max[1]);
  Serial.print(";VL2");          printMinMedMax (min[2], med[2], max[2]);
  Serial.print(";VL3");          printMinMedMax (min[3], med[3], max[3]);

}


/*
  Função auxiliar para imprimir os valores min med max
*/
void printMinMedMax (int min, int med, int max) {
  Serial.print(";\t");
  Serial.print(min);   Serial.print(";\t");
  Serial.print(med);   Serial.print(";\t");
  Serial.print(max);   Serial.println(";");
}

/*
  Função para definir a
  velocidade do módulo Bluetooth,
  o nome e a
  senha PIN.
*/
void setupBluetooth ( ) {

  //Serial.begin(9600);
  delay(5000);
  Serial.print(setNomeBluetooth);
  delay(5000);
  Serial.print(setPINBluetooth);
  delay(5000);
  Serial.print(setVelocidadeBluetooth);
  delay(5000);
  //Serial.begin(115200);
}


/*
  Menu de opções pelo terminal ou Bluetooth

  0 - Interrompe o experimento.   Imprime ";ON;"
  1 - Inicia o experimento.       Imprime ";OFF;"

  i - Indica a direção de início do ponto marcado. Imprime ";Dir_INI;"
  f - Indica a direção do final  do ponto marcado. Imprime ";Dir_FIM;"

  d - debug ON
  p - ping para teste. Imprime ";ping;"
  m - memória livre. Imprime ";mem_livre; QTD;"
  r - reset

*/
void lerComandosBluetooth( ) {

  dadoBluetooth = Serial.read();

  switch (dadoBluetooth) {
    case 1:
      if (!estadoLED) {
        Serial.println(";ON;");
        estadoLED = false;
        preStartPosStop( );
      }
      break;

    case 0:
      if (estadoLED) {
        estadoLED = true;
        preStartPosStop( );
        Serial.println(";OFF;");
      }
      break;

    case 'i':
      Serial.println(";Dir_INI;");
      break;

    case 'f':
      Serial.println(";Dir_FIM;");
      break;

    case 'm':
      Serial.print(";mem_livre;");
      Serial.print(freeMemory());
      Serial.println(";bytes;"); //bytes de 2048 bytes (Uno e Nano)
      break;

    case 'p':
      Serial.println(";ping;");
      break;

    case 'd':     
      debug = !debug;
      Serial.print(";debug;");
      Serial.print(debug);
      Serial.println(";");
      break;

    default:
      // comando(s)
      break;
  }
  Serial.flush();
}


void preStartPosStop( ) {

  estadoLED = !estadoLED; // altera o estado do LED
  if (estadoLED) {
    Serial.println (";start; \n;ID;HC1;VL1;VL2;VL3;TF1;GyX;GyY;GyZ;");
  }
  else {
    Serial.println (";stop;");
    if (i > 0 ) {
      if (debug) imprimirBuffer( );
      imprimirStats( );
      i = 0;
    }
  }
  digitalWrite(LED_PIN, estadoLED);
}