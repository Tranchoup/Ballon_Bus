#include <SPI.h>
#include <SD.h>
#include <Wire.h>//https://www.arduino.cc/en/reference/wire
#include <Adafruit_MPU6050.h>//https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>//https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>

File myFile;
Adafruit_MPU6050 mpu;
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);



void setup() {
  Serial.begin(115200);

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
    while (1) delay(10);
  }
  /* Display some basic information on this sensor */
  lis2mdl.printSensorDetails();
}


void loop() {


  myFile = SD.open("Data.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Ecriture");

    readMPU();
    readMagneto();
    // close the file:
    myFile.close();
    Serial.println("Ecriture Finie.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void readMPU( ) { /* function readMPU */
 	////Read acceleromter data
 	sensors_event_t a, g, temp;
 	mpu.getEvent(&a, &g, &temp);
 	/* Print out the values */
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
   delay(1000);
}

void readMagneto(){
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
