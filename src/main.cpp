#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

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
  mpu.initialize();


  Serial.println("\nI2C Scanner");

  if (mpu.testConnection()) {
    Serial.println("MPU6050 erfolgreich verbunden!");
  } else {
    Serial.println("Fehler beim Verbinden mit MPU6050.");
  }
}


void loop() {
  
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  //get sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //get time
  t = micros();
  dt = t - last_time;
  last_time = t;

  //acceleration in m/s^2
  float accx = ax * ACC_SCALE;
  float accy = ay * ACC_SCALE;
  float accz = az * ACC_SCALE;

  
  //print sensor data
  Serial.print("Beschleunigung X: "); Serial.print(accx);
  Serial.print(" | Y: "); Serial.print(accy);
  Serial.print(" | Z: "); Serial.println(accz);
  

  //calculating distance
  Serial.print("dt");
  Serial.print(dt);
  Serial.print("\n");

  dist = calc_dist(accx, accy, accz, dt);
  Serial.print("Distance:");
  Serial.print(dist,6);
  Serial.print("\n");
}
