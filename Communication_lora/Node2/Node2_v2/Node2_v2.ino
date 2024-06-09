 #include <stdio.h>
#include <SPI.h>
#include "LoRa.h"

#define WITH_APPKEY

String sendMessage;
String receivedMessage;

unsigned int freq = 865200000;//HZ
unsigned int idlePeriodInMin = 1; // in minute
unsigned int nCycle = idlePeriodInMin*60;
unsigned short id_frame = 0;


const int number_of_sensors = 25;
uint16_t id_temp_WURTH       = 675; // Temp(°C)
uint16_t id_temp_WURTH_humidity_sensor      = 676; // Temp(°C)
uint16_t id_temp_WURTH_pressur_sensor       = 677; // Temp(°C)
uint16_t id_temp_WURTH_inertial_sensor      = 678; // Temp(°C)
uint16_t id_temp_MPU          = 679; // Temp(°C)
uint16_t id_temp_DS18B20      = 680; // Temp(°C)
uint16_t id_geiger            = 681; 
uint16_t id_press_WURTH       = 682;
uint16_t id_press             = 683;
uint16_t id_humid_WURTH       = 684;
uint16_t id_magnetic_x        = 685;
uint16_t id_magnetic_y        = 686;
uint16_t id_magnetic_z        = 687;
uint16_t id_acc_x_WURTH       = 688;
uint16_t id_acc_y_WURTH       = 689;
uint16_t id_acc_z_WURTH       = 690;
uint16_t id_gyro_x_WURTH      = 691;
uint16_t id_gyro_y_WURTH      = 692;
uint16_t id_gyro_z_WURTH      = 693;
uint16_t id_acc_x_MPU         = 694;
uint16_t id_acc_y_MPU         = 695;
uint16_t id_acc_z_MPU         = 696;
uint16_t id_gyro_x_MPU        = 697;
uint16_t id_gyro_y_MPU        = 698;
uint16_t id_gyro_z_MPU        = 699;
uint16_t id_node              = 258;
uint16_t id_sensor[number_of_sensors]={id_temp_WURTH,id_temp_WURTH_humidity_sensor,id_temp_WURTH_pressur_sensor,id_temp_WURTH_inertial_sensor,id_temp_MPU,id_temp_DS18B20,id_geiger,id_press_WURTH,id_press,id_humid_WURTH,id_magnetic_x,id_magnetic_y,id_magnetic_z,id_acc_x_WURTH,id_acc_y_WURTH,id_acc_z_WURTH,id_gyro_x_WURTH,id_gyro_y_WURTH,id_gyro_z_WURTH,id_acc_x_MPU,id_acc_y_MPU,id_acc_z_MPU,id_gyro_x_MPU,id_gyro_y_MPU,id_gyro_z_MPU};
float value_f[number_of_sensors];
float value_int[number_of_sensors + 1];

#ifdef WITH_APPKEY
uint8_t my_appKey[4]={5, 6, 7, 8};
#endif

unsigned int cpt=1;

char message[100];


void send_data(float * value){
   
  uint8_t app_key_offset = 0;
  int e;
  #ifdef WITH_APPKEY
      app_key_offset = sizeof(my_appKey);
      memcpy(message,my_appKey,app_key_offset);
  #endif
      uint8_t r_size;
      char final_str[80] = "\\";
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
      
      LoRa.beginPacket();
      LoRa.print(message);
      LoRa.endPacket(); 
      
     Serial.println("DATA sent correctly !");
  
      for(uint8_t j=0;j<number_of_sensors;j++){
        value[j]=0.0;
      }
}
void setup(){
	Serial.begin(9600);
  Serial1.begin(9600);
  delay(5000);

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
   while (Serial1.available() > 0) {

	  cpt++;
    Serial.print("Frame Number : ");
    Serial.println(cpt);

    bool isCollectingData = false; // Indique si nous collectons les données des capteurs
    int dataIndex = 0; // Index pour parcourir les valeurs des capteurs

    // Lire les données de la communication série
    char receivedChar = Serial1.read();

    // Traiter les caractères de fin de ligne
    if (receivedChar == '\n') {
      Serial.println("\n");
      // Vérifier si le message reçu est un signal de début ou de fin de collecte
      if (receivedMessage[0] == 'S') {
        Serial.println("S");           
        isCollectingData = true; // Commencer à collecter les données
        dataIndex = 0; // Réinitialiser l'index
        } 
      else if (receivedMessage[0] == 'E') {
          isCollectingData = false; // Arrêter de collecter les données
        }

            // Si nous sommes en mode collecte de données
        if (isCollectingData) {
          Serial.println("CollectingDATA");

          // Les indices pairs contiennent les valeurs des capteurs
          if (dataIndex % 2 == 0) {
            value_int[dataIndex / 2 ] = atof(receivedMessage.c_str());
                    

            // Vérifier si toutes les données des capteurs ont été reçues
          if (dataIndex / 2 == number_of_sensors + 1) {
            for (int j = 0; j < number_of_sensors; j++) {
              value_f[j] = value_int[j+1];
            }
            // Envoyer les données collectées via LoRa
            Serial.println("Envoie données");
            send_data(value_f);
            // Attendre pendant idlePeriodInMin minute
            for (int j = 0; j < idlePeriodInMin * 60; j++) {
              delay(1000); // 1 seconde de délai
            }
            // Réinitialiser l'index après l'envoi des données
            dataIndex = 0;
            }
          }
          dataIndex++; // Passer à l'index suivant
        }

      // Afficher le message reçu dans le moniteur série
      Serial.println(receivedMessage);
      // Réinitialiser le message reçu
      receivedMessage = "";
    } 
    else {
      // Ajouter les caractères reçus au message
      receivedMessage += receivedChar;
    }
  }
}