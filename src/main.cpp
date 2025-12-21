#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <SD.h>
#include <SPI.h>
#include <string>

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

//variables
  int16_t ax1, ay1, az1;
  int16_t ax2, ay2, az2;
  float baseline[3] = {0};
  bool new_record = true;
  bool measuring = false;
  bool stateChange = false;
  String filename = "";

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

static void handleToggleButton() {
  
  if (digitalRead(Start_switch) == LOW){
   measuring = !measuring;
   stateChange = true;
  delay(500);
  }
  
      if (measuring && stateChange) {
        Serial.println("Messung läuft");
        digitalWrite(Status_Led, HIGH);
      } if(!measuring && stateChange) {
        Serial.println("Messung beendet");
        digitalWrite(Status_Led, LOW);
      }
    stateChange = false;
    }
   

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
  pinMode(Start_switch, INPUT_PULLUP);
  pinMode(Status_Led,OUTPUT);

}


void loop() {
  
  handleToggleButton();

  if (!measuring) {
    delay(10); // CPU entlasten
    return;
  }

  if (new_record) {
    filename = String(micros());
    filename = "/Data" + filename + ".csv";
    File Data = SD.open(filename, FILE_WRITE); 
    
    if (Data) {
      Data.println("timestamp_ms,x1,y1,z1,x2,y2,z2"); 
      Data.close();
  }

    new_record = false;
  }

  // --- Messung läuft ---
  mpu1.getAcceleration(&ax1, &ay1, &az1);
  mpu2.getAcceleration(&ax2, &ay2, &az2);

  t = micros();

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
  line += String(accx1, 5);
  line += ",";
  line += String(accy1, 5);
  line += ",";
  line += String(accz1, 5);
  line += ",";
  line += String(accx2, 5);
  line += ",";
  line += String(accy2, 5);
  line += ",";
  line += String(accz2, 5);

  File data = SD.open(filename, FILE_APPEND);
  if (data) {
    data.println(line);
    data.close();
  } else {
    Serial.println("Fehler: Data.csv konnte nicht geöffnet werden");
  }

  
}
