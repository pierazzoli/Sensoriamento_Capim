/*
    Autor: Eugênio Pierazzoli

   Este software utiliza diversos sensores em um microcontrolador, com o fim de sensorear
a altura da pastagem e outros aspectos de deslocamento. 

O microcontrolador utilizado é o Arduino Nano, cuja restrição em memória RAM limita o tamanho do buffer
de dados.
Os sensores HC-SR04, VL53L1X (I2C) e TF Mini Plus são responsáveis pelo sensoreamento.
O giroscópio alimenta o log e permite quando processado, identifica o estado da barra de sensores, onde a posição 
relativa é alterada de acordo com o terreno e a posição das rodas.

O 

     Componentes:
    Arduino Nano, (Old bootloader 328P)

    1 Sensor de Ultrassônico HC-SR04
    1 Giroscópio (I2C)
    1 Bluetooth (Serial padrao)
    3 VL53L1X (I2C)
    1 TF Mini Plus
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
          
          Acima de 200mA (Arduino Nano), os sensores devem ser ligados em uma fonte externa, 
          lembrando de igualar o GND.

*/
