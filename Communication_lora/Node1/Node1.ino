 #include <stdio.h>
#include <SPI.h>
#include "LoRa.h"

#define WITH_APPKEY

String sendMessage;
String receivedMessage;

unsigned int freq = 865500000;//HZ
unsigned int idlePeriodInMin = 1; // in minute
unsigned int nCycle = idlePeriodInMin*60;
unsigned short id_frame = 0;

/*sensor*/
const int number_of_sensors =  7;
uint16_t id_acc_X_MPU = 650;
uint16_t id_acc_Y_MPU = 651;
uint16_t id_acc_Z_MPU = 652;
uint16_t id_temp_MPU = 653;
uint16_t id_gyro_X_MPU = 654;
uint16_t id_gyro_Y_MPU = 655;
uint16_t id_gyro_Z_MPU = 656;
uint16_t id_node              = 257;
uint16_t id_sensor[number_of_sensors]={id_acc_X_MPU,id_acc_Y_MPU,id_acc_Z_MPU,id_temp_MPU,id_gyro_X_MPU,id_gyro_Y_MPU,id_gyro_Z_MPU}; //7 sensors
float value_f[number_of_sensors];
float value_int[number_of_sensors*2 + 1];


#ifdef WITH_APPKEY
uint8_t my_appKey[4]={5, 6, 7, 8};
#endif

unsigned int cpt=0;
int dataIndex = 0; // Index pour parcourir les valeurs des capteurs
bool isCollectingData = false; // Indique si nous collectons les données des capteurs

char message[150];

void collectData(float *value) {

    while (Serial1.available() > 0 && dataIndex < number_of_sensors*2 + 2) {
        char receivedChar = Serial1.read();
        //Serial.println(dataIndex);
        if (receivedChar == '\n') {
            if (receivedMessage.startsWith("S")) {
                isCollectingData = true; // Commencer à collecter les données
            } else if (receivedMessage.startsWith("E")) {
                isCollectingData = false; // Arrêter de collecter les données
            } else if (isCollectingData) {
                // Convertir les données reçues en flottant
                float sensorValue = atof(receivedMessage.c_str());

                // Vérifier les limites de dataIndex
                if (dataIndex < number_of_sensors * 2 + 1) {
                    value_int[dataIndex] = sensorValue;
                    dataIndex++;
                }
            }

            // Réinitialiser le message reçu
            receivedMessage = "";
        } else {
            receivedMessage += receivedChar; // Accumuler les caractères reçus
        }
    }
    //Serial.print("Reset dataIndex");
    dataIndex = 0;
    isCollectingData = false;
}



void send_data(float * value){
   
  uint8_t app_key_offset = 0;
  int e;
  #ifdef WITH_APPKEY
      app_key_offset = sizeof(my_appKey);
      memcpy(message,my_appKey,app_key_offset);
  #endif
      uint8_t r_size;
      char final_str[120] = "\\";
      char aux[10] = "";
      char id[1] = "";
      sprintf(final_str, "%s!%i!%hd", final_str,id_node, id_frame++);
      for (int i=0; i<number_of_sensors; i++) {
              sprintf(aux,"%4.4f", value[i]);
              sprintf(final_str, "%s#%d/%s", final_str, id_sensor[i], aux);
      }

      r_size=sprintf(message+app_key_offset,final_str);
    
      Serial.println(message);
      //Serial.println(r_size);
      //Serial.println(strlen(message));
      
      LoRa.beginPacket();
      LoRa.print(message);
      LoRa.endPacket(); 
      
      Serial.println("DATA sent correctly !");
  
      for(uint8_t j=0;j<number_of_sensors+1;j++){
        //Serial.println("Test inside send_data");
        value[j]=0.0;
      }
      
}
void setup(){
	Serial.begin(38400);
  Serial1.begin(9600);
  delay(5000);
  Serial.println("Set LoRa modulation\r\n");

  if (!LoRa.begin(freq)) {
    Serial.println("Starting LoRa failed!");
    while (1); 
  }
  
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
}


void loop()
{
	/* Update sensor values*/
  collectData(value_int);
  /*for (int j = 0; j < number_of_sensors*2+1; j++) {
              Serial.println(value_int[j]);
            } 
  */
  //Récupérer la bonne data 
  for (int j = 0; j < number_of_sensors; j++) {
              value_f[j] = value_int[j*2 +1];
            }

  cpt++;
  Serial.print("Frame Number : ");
  Serial.println(cpt);


	/*Send Data via LoRa Module*/
  send_data(value_f);
  //Serial.println("End of send_data");

	 /* wait for #idlePeriodInMin Minute nCycle */
  for (int j = 0; j < idlePeriodInMin * 60; j++) {
    //Serial.println("test");
    delay(1000); // 1 seconde de délai}
  }
}
