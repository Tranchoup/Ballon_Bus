#include <Arduino.h>
#include "WSEN_TIDS.h"
#include "WSEN_HIDS.h"
#include "WSEN_ISDS.h"
//#include "WSEN_PADS.h"

#include <OneWire.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>//https://www.arduino.cc/en/reference/wire
#include <Adafruit_MPU6050.h>//https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>//https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_LIS2MDL.h>


void setup_TIDS_temp_wurth();
void lecture_TIDS_temp_wurth();

void setup_HIDS_humidity_temp_wurth();
void lecture_HIDS_humidity_wurth();

void setup_ISDS_inertial_unit_wurth();
void lecture_ISDS_inertial_unit_wurth();

// void setup_PADS_pressure_wurth();
// void lecture_PADS_pressure_wurth();

void lecture_pression_analogique();
void readDS18B20();
void readMPU( );
void readMagneto();
void setupMPU();
void setupMagneto();
void setupSD();

float myTemperature_TIDS ;
float relHum;
File myFile;
Adafruit_MPU6050 mpu;
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);

Sensor_HIDS sensor_HIDS;
int status_HIDS;

Sensor_TIDS sensor_TIDS;
int status_TIDS;

Sensor_ISDS sensor_ISDS;
int status_ISDS;

// Sensor_PADS sensor_PADS;
// int status_PADS;
// PADS_state_t stateTemperature;
// PADS_state_t statePressure;

int pin_pressur_sensor = A1;
double pressur_volt = 0;
double V_supply = 5;
double P_max = 103421;
double pressur_analog = 0;

OneWire  ds(7);  

void setup()
{
  Serial.begin(9600);
  setup_TIDS_temp_wurth();
  setup_ISDS_inertial_unit_wurth();
  //setup_PADS_pressure_wurth();
  //setup_HIDS_humidity_temp_wurth(); NE FONCTIONNE PAS
   setupSD();
  setupMPU();
  setupMagneto();
}

void loop()
{
  Serial.print("success flash");
  myFile = SD.open("Data.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Ecriture");

    readMPU();
    myFile.println(" ");
    delay(100);
    readMagneto();
    myFile.println(" ");
    delay(100);
    lecture_TIDS_temp_wurth();
    myFile.println(" ");
    delay(100);
    lecture_pression_analogique();
    myFile.println(" ");
    delay(100);
    lecture_ISDS_inertial_unit_wurth();
    myFile.println(" ");
    delay(100);
    //lecture_PADS_pressure_wurth();
    myFile.println(" ");
    delay(100);
    readDS18B20();
    delay(100);
    readDS18B20();
    delay(100);
    myFile.println(millis());
    myFile.println(" --------------------------------------------------------------- ");
    // close the file:
    myFile.close();
    Serial.println("Ecriture Finie.");
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

}

void setup_HIDS_humidity_temp_wurth()
{
    // Initialize the I2C interface
  sensor_HIDS.init(HIDS_ADDRESS_I2C_0);

  //The Output Data Rate One shot mode
  if (WE_FAIL == sensor_HIDS.set_single_conversion())
  {
    Serial.println("Error: set_single_conversion(). STOP!");
  }

  // Check if sensor is ready to measure the humidity
}
void lecture_HIDS_humidity_wurth()
{
  myFile.println("Lecture de HIDS");
  HIDS_state_t temp_drdy_HIDS;
  HIDS_state_t humidity_drdy_HIDS;

  status_HIDS = sensor_HIDS.get_StatusDrdy(&temp_drdy_HIDS, &humidity_drdy_HIDS);
  if (WE_FAIL == status_HIDS)
  {
    Serial.println("Error: get_StatusDrdy(). STOP!");
  }
  else
  {  
    if (0 == humidity_drdy_HIDS)
    {
      Serial.println("Sensor is not ready.");
    }
    else
    {
      
      //Print the humidity value on the serial monitor
      float rh;
      if (WE_FAIL == sensor_HIDS.get_Humidity(&rh))
      {
        Serial.println("Error: get_Humidity(). STOP!");
      }
      else
      {
        myFile.print("The humidity is: ");
        myFile.print(rh);
        myFile.println(" %");
      }
    }
  
    if (0 == temp_drdy_HIDS)
    {
      Serial.println("Sensor is not ready.");
    }
    else
    {
      
  
      float temperature;
      if (WE_FAIL == sensor_HIDS.get_Temperature(&temperature))
      {
        Serial.println("Error: get_Temperature. STOP!");
      }
      else
      {
        myFile.print("The temperature is: ");
        myFile.print(temperature);
        myFile.println(" degC");
      }
    }   
  }
 
}

void setup_ISDS_inertial_unit_wurth()
{


}
void lecture_ISDS_inertial_unit_wurth()
{
    // Initialize the I2C interface
  sensor_ISDS.init(ISDS_ADDRESS_I2C_1);

  // Reset sensor
  status_ISDS = sensor_ISDS.SW_RESET();
  if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error:  SW_RESET(). Stop!");
  }

  // Set FIFO ODR to 26Hz
  status_ISDS = sensor_ISDS.select_ODR(2);
  if (WE_FAIL == status_ISDS)
    {
    myFile.println("Error:  select_ODR(). Stop!");
  }

  // Set high performance mode
  status_ISDS = sensor_ISDS.set_Mode(2);
  if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error:  set_Mode(). Stop!");
  }
    myFile.println("Lecture de ISDS");

  status_ISDS = sensor_ISDS.is_ACC_Ready_To_Read();
  if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error: is_ACC_Ready_To_Read(). Stop!");
  } 
  else if (1 == status_ISDS)
  {
    int16_t acc_X;
    int16_t acc_Y;
    int16_t acc_Z;

#if 0
    // Get acceleration along X axis in mg
   status_ISDS = sensor_ISDS.get_acceleration_X(&acc_X);
   if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error:  get_acceleration_X(). Stop!");
  }
    myFile.println("Acceleration along X axis in [mg]: ");
    myFile.println(acc_X);

    // Get acceleration along Y axis in mg
    status_ISDS = sensor_ISDS.get_acceleration_Y(&acc_Y);
    if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error:  get_accel(). Stop!");
  }
    myFile.println("Acceleration along Y axis in [mg]: ");
    myFile.println(acc_Y);

    // Get acceleration along Z axis in mg
    status_ISDS = sensor_ISDS.get_acceleration_Z(&acc_Z);
    if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error:  get_acceleration_Z(). Stop!");
  }
    myFile.println("Acceleration along Z axis in [mg]: ");
    myFile.println(acc_Z);
#else
    
    status_ISDS = sensor_ISDS.get_accelerations(&acc_X,&acc_Y,&acc_Z);
    if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error:  get_accelerations(). Stop!");
  }
    myFile.println("Acceleration along X,Y,Z axis in [mg]: ");
    myFile.print(acc_X);
    myFile.print(" ");
    myFile.print(acc_Y);
    myFile.print(" ");
    myFile.print(acc_Z);
    myFile.println(" ");
#endif
  }
  else /* status == '0' -> data not ready */
  {
    myFile.println("No data in output register.");
  }
   status_ISDS = sensor_ISDS.is_Gyro_Ready_To_Read();
 if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error: is_Gyro_Ready_To_Read(). Stop!");
  } 
  else if (1 == status_ISDS)
  {
    int32_t gyro_X;
    int32_t gyro_Y;
    int32_t gyro_Z;
#if 0
    // Get X-axis angular rate in [mdps]
   
    status_ISDS = sensor_ISDS.get_angular_rate_X(&gyro_X);
    if (WE_FAIL == status_ISDS)
    {
    myFile.println("Error:  get_angular_rate_X(). Stop!");
  }
    myFile.println("Angular rate in X axis in [mdps]: ");
    myFile.println(gyro_X);

    // Get Y-axis angular rate in [mdps]
    status_ISDS = sensor_ISDS.get_angular_rate_Y(&gyro_Y);
    if (WE_FAIL == status_ISDS)
    {
    myFile.println("Error:  get_angular_rate_Y(). Stop!");
  }
    myFile.println("Angular rate in  Y axis in [mdps]: ");
    myFile.println(gyro_Y);

    // Get Z-axis angular rate in [mdps]
    status_ISDS = sensor_ISDS.get_angular_rate_Z(&gyro_Z);
    if (WE_FAIL == status_ISDS)
    {
    myFile.println("Error:  get_angular_rate_Z(). Stop!");
  }
    myFile.println("Angular rate in  Z axis in [mdps]: ");
    myFile.println(gyro_Z);
#else
    status_ISDS = sensor_ISDS.get_angular_rates(&gyro_X,&gyro_Y,&gyro_Z);
    if (WE_FAIL == status_ISDS)
    {
    myFile.println("Error:  get_angular_rates(). Stop!");
  }
    myFile.println("Angular rate in X,Y,Z axis in [mdps]: ");
    myFile.print(gyro_X);
    myFile.print(" ");
    myFile.print(gyro_Y);
    myFile.print(" ");
    myFile.print(gyro_Z);
    myFile.println(" ");
#endif
  }
  else
  {
    myFile.println("No data in output register.");
  }
    // Check if sensor is ready to measure the temperature
  status_ISDS = sensor_ISDS.is_Temp_Ready();
    
  if (WE_FAIL == status_ISDS)
  {
    myFile.println("Error: is_Temp_Ready(). Stop!");
  }
  else if (1 == status_ISDS)
  {
    //Print the temperature on the serial monitor
    
    float temperature_ISDS;
    if (WE_FAIL == sensor_ISDS.get_temperature(&temperature_ISDS))
      {
    myFile.println("Error:  get_temperature(). Stop!");
    }
    myFile.println("Temperature in [°C]: ");
    myFile.println(temperature_ISDS);
  }
  else
  {
    myFile.println("Sensor not ready.");
  }
}

// void setup_PADS_pressure_wurth()
// {
//     // Initialize the I2C interface
// }
// void lecture_PADS_pressure_wurth()
// {
//     sensor_PADS.init(PADS_ADDRESS_I2C_1);

//     myFile.println("Lecture de PADS");
//     // Set single conversion mode
//   if (WE_FAIL == sensor_PADS.set_single_conversion())
//   {
//     Serial.print("Error: set_single_conversion(). STOP!");
//   }

//   status_PADS = sensor_PADS.ready_to_read(&stateTemperature, &statePressure);
//   if (WE_FAIL == status_PADS)
//   {
//     myFile.print("Error: Sensor DRDY temperature not readable.");
//   }
//   else if (0 == stateTemperature)
//   {
//    myFile.print("Error: temperature DRDY = 0.");
//   }
//   else
//   {
  
//     // Read and calculate the temperature
//     float temperature;
//     sensor_PADS.read_temperature(&temperature);
  
//     myFile.print("temperature: ");
  
//     // Print the temperature on the serial monitor
//     myFile.print(temperature);
//     myFile.println(" degC");
  
//   }

//   if (0 == statePressure)
//   {
//     Serial.print("Error: pressure DRDY = 0.");
//   }
//   else
//   {
//     // Read and calculate the pressure
//     float pressure;
//     sensor_PADS.read_pressure(&pressure);
  
//     myFile.print("pressure: ");
  
//     // Print the pressure on the serial monitor
//     myFile.print(pressure);
//     myFile.println(" kPa");
//   }
// }

void setup_TIDS_temp_wurth()
{
 
}
void lecture_TIDS_temp_wurth()
{// Initialize the I2C interface
      myFile.println("Lecture de TIDS");

  sensor_TIDS.init(TIDS_ADDRESS_I2C_1);

  //Perform a software reset
  status_TIDS = sensor_TIDS.SW_RESET();
  if (WE_FAIL == status_TIDS)
  {
    myFile.println("Error: SW_RESET(). STOP!");
  }

  // Set single conversion mode
  status_TIDS = sensor_TIDS.set_single_conversion();
  if (WE_FAIL == status_TIDS)
  {
    myFile.println("Error: set_single_conversion(). STOP!");
  }

  // Check if sensor is ready to measure the temperature


  int drdy_TIDS = sensor_TIDS.is_ready_to_read();  
  if (WE_FAIL == drdy_TIDS)
  {
    myFile.println("Error: is_ready_to_read(). STOP!");
  }
  else if (1 == drdy_TIDS)
  {
    // Read and calculate the temperature
    status_TIDS = sensor_TIDS.read_temperature(&myTemperature_TIDS);
    if (WE_FAIL == status_TIDS)
    {
      myFile.println("Error: read_temperature(). STOP!");
    }
    
    // Print the temperature on the serial monitor
    myFile.print("The temperature is: ");
    myFile.print(myTemperature_TIDS);
    myFile.println(" Celsius SETUP");
  }
  else
  {
    myFile.print("Sensor is not ready.");
  }
      myFile.println("Lecture de TIDS");

}

void lecture_pression_analogique()
{
    myFile.println("Lecture du capteur analogique de pression");
  pressur_volt = analogRead(pin_pressur_sensor);
  pressur_analog = ((((pressur_volt/1024)*5)-0.10*V_supply)/(0.8*V_supply))*P_max;
  myFile.println("Pression en Pa");
  myFile.println(pressur_analog);
}

void setupSD()
{
  // Initialisation de la carte SD
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}


void setupMPU()
{
  //initialisation MPU5060
 	Serial.println(F("Initialize System"));
 if (!mpu.begin(0x68)) { // Change address if needed
 			Serial.println("Failed to find MPU6050 chip");
 			while (1) {
 					delay(10);
 			}
 	}
 	mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
 	mpu.setGyroRange(MPU6050_RANGE_250_DEG);
 	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}
void readMPU( ) { /* function readMPU */
  myFile.println("Lecture de MPU6050");
 	////Read acceleromter data
 	sensors_event_t a, g, temp;
 	mpu.getEvent(&a, &g, &temp);
 	/* Print out the values */
  myFile.print("MPU : \n");
 	myFile.print("Acceleration X: ");
 	myFile.print(a.acceleration.x);
 	myFile.print(", Y: ");
 	myFile.print(a.acceleration.y);
 	myFile.print(", Z: ");
 	myFile.print(a.acceleration.z);
 	myFile.println(" m/s^2");
 	myFile.print("Rotation X: ");
 	myFile.print(g.gyro.x);
 	myFile.print(", Y: ");
 	myFile.print(g.gyro.y);
 	myFile.print(", Z: ");
 	myFile.print(g.gyro.z);
 	myFile.println(" rad/s");
 	myFile.print("Temperature: ");
 	myFile.print(temp.temperature);
 	myFile.println("C");
}


void setupMagneto()
{
  
    //Initialisation Magnetometre
    while (!Serial)
      delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("LIS2MDL Magnetometer Test");
    Serial.println("");
    /* Enable auto-gain */
  lis2mdl.enableAutoRange(true);
  /* Initialise the sensor */
  if (!lis2mdl.begin()) {  // I2C mode
    /* There was a problem detecting the LIS2MDL ... check your connections */
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    delay(10);
  }
  /* Display some basic information on this sensor */
  lis2mdl.printSensorDetails();

}
void readMagneto(){
    myFile.print("lecture du Magnetometre lis2mdl");
    sensors_event_t event;
    lis2mdl.getEvent(&event);
    myFile.print("X: ");
    myFile.print(event.magnetic.x);
    myFile.print("  ");
    myFile.print("Y: ");
    myFile.print(event.magnetic.y);
    myFile.print("  ");
    myFile.print("Z: ");
    myFile.print(event.magnetic.z);
    myFile.print("  ");
    myFile.println("uT");
}

void readDS18B20()
{
  
  byte i;
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr)) {
    ds.reset_search();
    return;
  }
  

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;

  myFile.println("Temperature DS18B20");
  myFile.print("  Temperature = ");
  myFile.print(celsius);
  myFile.println(" Celsius, ");
}
