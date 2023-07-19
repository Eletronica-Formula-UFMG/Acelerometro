/*
  Codigo da placa de aceleracao do sistema de aquisicao do TR08 da equipe Formula SAE UFMG.
  Realiza a leitura de 2 acelerometros e a compactacao do dado para envio atraves da CAN.
  Autor: Lucas Lourenco Reis Resende
  Data: 12/07/2023  
*/

#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

CAN_device_t CAN_cfg;

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

#define ID 0x301;

// Função para compactar um valor float em 2 bytes
void compactFloat(float value, byte *buffer) {
  if (value < 0) {  //Se o valor for negativo trata de um jeito
    value = 6.00 + fabs(value);
  }
  // Escala o valor para o intervalo
  int scaledValue = (int)(value * 256.0 / 12.0);
  //int scaledValue = (int)(value * 65536 / 12.0);

  // Converte o valor para 2 bytes
  buffer[0] = (byte)(scaledValue >> 8);  // byte mais significativo
  buffer[1] = (byte)scaledValue;         // byte menos significativo
}

/*
  Nome: setup
  Descricao: Incializa o modulo CAN, acelerometros e seta os parametros necessarios para a transmissao
*/

void setup() {
  Serial.begin(115200);

  // Inicializa os modulos aceleromentos
  // Try to initialize!
  if (!mpu2.begin(0x69) && !mpu1.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050s Found!");

  // Sets do sensor 1
  mpu1.begin(0x68);
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Sets do sensor 2
  mpu2.begin(0x69);
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("");
  delay(100);

  /* Configura a taxa de transmiss�o e os pinos Tx e Rx */
  CAN_cfg.speed = CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_4;  //GPIO 5 (Pino 6) = CAN TX
  CAN_cfg.rx_pin_id = GPIO_NUM_5;  //GPI 35 (Pino 30) = CAN RX

  /* Cria uma fila de até 10 elementos para receber os frames CAN */
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));

  /* Inicializa o m�dulo CAN */
  ESP32Can.CANInit();
}

/*
  Nome: loop
  Descricao: Faz a leitura dos canais e o envio dos pacotes pelo barramento CAN
*/

void loop() {
  /* Get new sensor events with the readings */

  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);


  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);



  /* Declaração dos pacotes CAN */
  CAN_frame_t rx_frame;
  CAN_frame_t frame_1, frame_2, frame_3, frame_4;

  byte compactedDatax1[2];
  byte compactedDatay1[2];
  byte compactedDataz1[2];

  byte compactedDatax2[2];
  byte compactedDatay2[2];
  byte compactedDataz2[2];

  /* Recebe o próximo pacote CAN na fila */
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
    /*Como esse modulo nao recebe nada, esse laco eh vazio*/

  } else {
    //Dados de aceleracao do modulo 1 já transformado para g
    compactFloat(a1.acceleration.x / 9.81, compactedDatax1);
    compactFloat(a1.acceleration.y / 9.81, compactedDatay1);
    compactFloat(a1.acceleration.z / 9.81, compactedDataz1);

    Serial.println("Acel 1");
    Serial.print(a1.acceleration.x / 9.81);
    Serial.print("\t");
    Serial.print(a1.acceleration.y / 9.81);
    Serial.print("\t");
    Serial.println(a1.acceleration.z / 9.81);

    //Dados de aceleracao do modulo 2 já transformado para g
    compactFloat(a2.acceleration.x / 9.81, compactedDatax2);
    compactFloat(a2.acceleration.y / 9.81, compactedDatay2);
    compactFloat(a2.acceleration.z / 9.81, compactedDataz2);

    Serial.println("Acel 2");
    Serial.print(a2.acceleration.x / 9.81);
    Serial.print("\t");
    Serial.print(a2.acceleration.y / 9.81);
    Serial.print("\t");
    Serial.println(a2.acceleration.z / 9.81);

    /*Manipula os dados que serão enviados*/
    frame_1.FIR.B.FF = CAN_frame_std;
    // Definicao do ID da mensagem
    frame_1.MsgID = ID;
    frame_1.FIR.B.DLC = 8;
    frame_1.data.u8[0] = 1;
    frame_1.data.u8[1] = compactedDatax1[0];
    frame_1.data.u8[2] = compactedDatax1[1];
    frame_1.data.u8[3] = compactedDatay1[0];
    frame_1.data.u8[4] = compactedDatay1[1];
    frame_1.data.u8[5] = compactedDataz1[0];
    frame_1.data.u8[6] = compactedDataz1[1];
    frame_1.data.u8[7] = 0;

    /*Manipula os dados que serão enviados*/
    frame_2.FIR.B.FF = CAN_frame_std;
    // Definicao do ID da mensagem
    frame_2.MsgID = ID;
    frame_2.FIR.B.DLC = 8;
    frame_2.data.u8[0] = 2;
    frame_2.data.u8[1] = compactedDatax2[0];
    frame_2.data.u8[2] = compactedDatax2[1];
    frame_2.data.u8[3] = compactedDatay2[0];
    frame_2.data.u8[4] = compactedDatay2[1];
    frame_2.data.u8[5] = compactedDataz2[0];
    frame_2.data.u8[6] = compactedDataz2[1];
    frame_2.data.u8[7] = 0;

    //Dados de vel. angular do modulo 1
    compactFloat(a1.gyro.x, compactedDatax1);
    compactFloat(a1.gyro.y, compactedDatay1);
    compactFloat(a1.gyro.z, compactedDataz1);

    //Dados de vel. angular do modulo 2
    compactFloat(a2.gyro.x, compactedDatax2);
    compactFloat(a2.gyro.y, compactedDatay2);
    compactFloat(a2.gyro.z, compactedDataz2);

    /*Manipula os dados que serão enviados*/
    frame_3.FIR.B.FF = CAN_frame_std;
    // Definicao do ID da mensagem
    frame_3.MsgID = ID;
    frame_3.FIR.B.DLC = 8;
    frame_3.data.u8[0] = 3;
    frame_3.data.u8[1] = compactedDatax2[0];
    frame_3.data.u8[2] = compactedDatax2[1];
    frame_3.data.u8[3] = compactedDatay2[0];
    frame_3.data.u8[4] = compactedDatay2[1];
    frame_3.data.u8[5] = compactedDataz2[0];
    frame_3.data.u8[6] = compactedDataz2[1];
    frame_3.data.u8[7] = 0;

    /*Manipula os dados que serão enviados*/
    frame_4.FIR.B.FF = CAN_frame_std;
    // Definicao do ID da mensagem
    frame_4.MsgID = ID;
    frame_4.FIR.B.DLC = 8;
    frame_4.data.u8[0] = 4;
    frame_4.data.u8[1] = compactedDatax2[0];
    frame_4.data.u8[2] = compactedDatax2[1];
    frame_4.data.u8[3] = compactedDatay2[0];
    frame_4.data.u8[4] = compactedDatay2[1];
    frame_4.data.u8[5] = compactedDataz2[0];
    frame_4.data.u8[6] = compactedDataz2[1];
    frame_4.data.u8[7] = 0;

    ESP32Can.CANWriteFrame(&frame_1);  //Envia o primeiro pacote
    delay(100);
    ESP32Can.CANWriteFrame(&frame_2);  //Envia o segundo pacote
    delay(100);
    ESP32Can.CANWriteFrame(&frame_3);  //Envia o terceiro pacote
    delay(100);
    ESP32Can.CANWriteFrame(&frame_4);  //Envia o quarto pacote
  }
}
