#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SD.h>
#include <SPI.h>

MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

#define SD_CS 5
#define Start_switch 26
#define Status_Led 25

// Struktur für binäre Daten (exakt 16 Bytes groß)
struct __attribute__((packed)) DataRecord {
  uint32_t time_ms; // 4 Bytes
  int16_t ax1, ay1, az1; // 6 Bytes
  int16_t ax2, ay2, az2; // 6 Bytes
};

File dataFile;
bool measuring = false;
bool last_measuring = false;
String filename = "";

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); // I2C auf 400kHz beschleunigen

  mpu1.initialize();
  mpu2.initialize();

  if (!SD.begin(SD_CS)) {
    Serial.println("SD-Fehler!");
    return;
  }

  pinMode(Start_switch, INPUT_PULLUP);
  pinMode(Status_Led, OUTPUT);
}

void loop() {
  // Taster abfragen
  measuring = (digitalRead(Start_switch) == LOW);

  // Flankenerkennung: Start der Messung
  if (measuring && !last_measuring) {
    // Neuen Dateinamen finden
    for (int i = 1; i <= 500; i++) {
      filename = "/run" + String(i) + ".bin";
      if (!SD.exists(filename)) break;
    }
    
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      digitalWrite(Status_Led, HIGH);
      Serial.println("Start: " + filename);
    }
  }

  // Flankenerkennung: Ende der Messung
  if (!measuring && last_measuring) {
    if (dataFile) {
      dataFile.close();
      digitalWrite(Status_Led, LOW);
      Serial.println("Messung beendet und Datei geschlossen.");
    }
  }

  // Während der Messung
  if (measuring && dataFile) {
    DataRecord record;
    record.time_ms = millis();
    mpu1.getAcceleration(&record.ax1, &record.ay1, &record.az1);
    mpu2.getAcceleration(&record.ax2, &record.ay2, &record.az2);

    // Ganze Struktur binär schreiben
    dataFile.write((const uint8_t*)&record, sizeof(record));
    
    // Optional: Alle 512 Bytes (Puffergröße) wird automatisch geschrieben.
    // dataFile.flush(); // NUR nutzen, wenn Datenverlust bei Absturz kritisch ist (bremst aber!)
  }

  last_measuring = measuring;
}