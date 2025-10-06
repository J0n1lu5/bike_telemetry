#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <SD.h>
#include <SPI.h>

MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

#define SD_CS   5      // Chip-Select
#define SD_SCK  18     // VSPI_SCK
#define SD_MISO 19     // VSPI_MISO
#define SD_MOSI 23     // VSPI_MOSI

#define Start_switch 26
#define Status_Led 25

// factor for scaling to m/s^2
const float ACC_SCALE = 9.81 / 16384.0;

//variables for time
unsigned int  t = 0;
unsigned int  last_time = 0;
unsigned int  dt = 0;  

// velocity
float dist = 0;

float calc_dist(float accx, float accy, float accz, unsigned int dt, float baseline[]){
  float vx = 0;
  float vy = 0;
  float vz = 0;

  float sx = 0;
  float sy = 0;
  float sz = 0;

  float distance = 0;

  float scale_dt;

  scale_dt = dt / 1e6;

  vx = (accx - baseline[0]) * scale_dt;
  vy = (accy - baseline[1]) * scale_dt;
  vz = (accz - baseline[2]) * scale_dt;

  sx = vx * scale_dt;
  sy = vy * scale_dt;
  sz = vz * scale_dt;

  distance = sqrt(sx*sx + sy*sy + sz*sz);

  return distance;
}

//variables
  int16_t ax1, ay1, az1;
  int16_t ax2, ay2, az2;
  float baseline[3] = {0};
  bool enabled = false;

void setup() {
  Serial.begin(115200);
  
  
  //MPU setup
  Wire.begin(21, 22); // ESP32: SDA=21, SCL=22
  
  delay(200);

  mpu1.initialize();
  mpu2.initialize();

  delay(500);

  Serial.println("\nI2C Scanner");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      delay(5);
    }
  }

  //SD card setup
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  //pinMode(SD_CS, OUTPUT);
  //digitalWrite(SD_CS, HIGH);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD-Karte nicht gefunden!");
    return;
  }
  Serial.println("SD-Karte initialisiert.");

  File Data = SD.open("/Data.csv", FILE_WRITE); 
  if (Data) {
    Data.println("timestamp_ms,x1,y1,z1,x2,y2,z2"); 
    Data.close();
  }
  

  //anschalten
  pinMode(Start_switch, INPUT);
  pinMode(Status_Led,OUTPUT);

}


void loop() {
  
  bool help = true;

  if(digitalRead(Start_switch)==HIGH){
    digitalWrite(Status_Led, HIGH);
    if (enabled==false){
      
      Serial.println("Messung gestartet");
      enabled = true;
    }

    //get sensor data
    mpu1.getAcceleration(&ax1, &ay1, &az1);
    mpu2.getAcceleration(&ax2, &ay2, &az2);

    //get time
    t = micros();
    
  //acceleration in m/s^2
    float accx1 = ax1 * ACC_SCALE;
    float accy1 = ay1 * ACC_SCALE;
    float accz1 = az1 * ACC_SCALE;

    float accx2 = ax2 * ACC_SCALE;
    float accy2 = ay2 * ACC_SCALE;
    float accz2 = az2 * ACC_SCALE;


    String line;
      line.reserve(128);
      line += String(t);
      line += ",";
      line += String(accx1,5); 
      line += ",";
      line += String(accy1,5); 
      line += ",";
      line += String(accz1,5); 
      line += ",";
      line += String(accx2,5); 
      line += ",";
      line += String(accy2,5); 
      line += ",";
      line += String(accz2,5); 

    File Data = SD.open("/Data.csv", FILE_APPEND);
    Data.println(line);
    Data.close();
  }
  else {
    if (enabled==true){
      Serial.println("Messung beendet");
      enabled = false;
    }
    digitalWrite(Status_Led, LOW);
  }

  /*
  dt = t - last_time;
  last_time = t;

  //acceleration in m/s^2
  float accx1 = ax1 * ACC_SCALE;
  float accy1 = ay1 * ACC_SCALE;
  float accz1 = az1 * ACC_SCALE;

  float accx2 = ax2 * ACC_SCALE;
  float accy2 = ay2 * ACC_SCALE;
  float accz2 = az2 * ACC_SCALE;



  //relative accelaration
  float accx = accx1 - accx2;
  float accy = accy1 - accy2;
  float accz = accz1 - accz2;
  
  
  //print sensor data
  Serial.print("Beschleunigung X: "); Serial.print(accx1);
  Serial.print(" | Y: "); Serial.print(accy1);
  Serial.print(" | Z: "); Serial.println(accz1);
  


// eleminate baseline noise  
 if (digitalRead(13) == LOW){
  baseline[0] = accx;
  baseline[1] = accy;
  baseline[2] = accz;
 }

  //calculating distance
 
  dist = calc_dist(accx, accy, accz, dt, baseline);
  Serial.print("Distance:");
  Serial.print(dist,6);
  Serial.print("\n");

  */

  
}
