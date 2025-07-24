#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

// factor for scaling to m/s^2
const float ACC_SCALE = 9.81 / 16384.0;

//variables for time
unsigned int  t = 0;
unsigned int  last_time = 0;
unsigned int  dt = 0;  

// velocity
float dist = 0;

float calc_dist(float accx, float accy, float accz, unsigned int dt){
  float vx = 0;
  float vy = 0;
  float vz = 0;

  float sx = 0;
  float sy = 0;
  float sz = 0;

  float distance = 0;

  float scale_dt;

  scale_dt = dt / 1e6;

  vx = accx * scale_dt;
  vy = accy * scale_dt;
  vz = accz * scale_dt;

  sx = vx * scale_dt;
  sy = vy * scale_dt;
  sz = vz * scale_dt;

  distance = sqrt(sx*sx + sy*sy + sz*sz);

  return distance;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32: SDA=21, SCL=22
  
  mpu1.initialize();
  mpu2.initialize();


  Serial.println("\nI2C Scanner");

  if (mpu1.testConnection()) {
    Serial.println("MPU1 erfolgreich verbunden!");
  } else {
    Serial.println("Fehler beim Verbinden mit MPU6050.");
  }

  if (mpu2.testConnection()) {
    Serial.println("MPU1 erfolgreich verbunden!");
  } else {
    Serial.println("Fehler beim Verbinden mit MPU6050.");
  }
}


void loop() {
  
  int16_t ax1, ay1, az1;
  int16_t ax2, ay2, az2;

  //get sensor data
  mpu1.getAcceleration(&ax1, &ay1, &az1);
  mpu2.getAcceleration(&ax2, &ay2, &az2);

  //get time
  t = micros();
  dt = t - last_time;
  last_time = t;

  //acceleration in m/s^2
  float accx1 = ax1 * ACC_SCALE;
  float accy1 = ay1 * ACC_SCALE;
  float accz1 = az1 * ACC_SCALE;

  float accx2 = ax2 * ACC_SCALE;
  float accy2 = ay2 * ACC_SCALE;
  float accz2 = az2 * ACC_SCALE;

  //erdbeschleunigung muss rausgerechnet werden

  //entweder über konstanten wert und oder durch init

  //relative accelaration
  float accx = accx1 - accx2;
  float accy = accy1 - accy2;
  float accz = accz1 - accz2;
  
  /*
  //print sensor data
  Serial.print("Beschleunigung X: "); Serial.print(accx1);
  Serial.print(" | Y: "); Serial.print(accy1);
  Serial.print(" | Z: "); Serial.println(accz1);
  */

  //calculating distance

  dist = calc_dist(accx, accy, accz, dt);
  Serial.print("Distance:");
  Serial.print(dist,6);
  Serial.print("\n");
}
