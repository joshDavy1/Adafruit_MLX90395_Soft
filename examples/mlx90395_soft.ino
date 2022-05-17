#include "Adafruit_MLX90395_Soft.h"


Adafruit_MLX90395_Soft sensor = Adafruit_MLX90395_Soft();
Adafruit_MLX90395_Soft sensor2= Adafruit_MLX90395_Soft();

void setup(void)
{
  Serial.begin(115200);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
      delay(10);
  }

  Serial.println("Starting Adafruit MLX90395 Demo3");
  
  if (! sensor.begin_I2C(4, 3, 0x0C)) {          // software i2c
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
  if (! sensor2.begin_I2C(4, 3, 0x0D)) {          // software i2c
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }

  Serial.print("Found a MLX90395 sensor with unique id 0x");
  Serial.print(sensor.uniqueID[0], HEX);
  Serial.print(sensor.uniqueID[1], HEX);
  Serial.println(sensor.uniqueID[2], HEX);


   Serial.print("Found a MLX90395 sensor with unique id 0x");
  Serial.print(sensor2.uniqueID[0], HEX);
  Serial.print(sensor2.uniqueID[1], HEX);
  Serial.println(sensor2.uniqueID[2], HEX);

  sensor.setOSR(MLX90395_OSR_8);
  sensor2.setOSR(MLX90395_OSR_8);
  Serial.print("OSR set to: ");
  switch (sensor.getOSR()) {
    case MLX90395_OSR_1: Serial.println("1 x"); break;
    case MLX90395_OSR_2: Serial.println("2 x"); break;
    case MLX90395_OSR_4: Serial.println("4 x"); break;
    case MLX90395_OSR_8: Serial.println("8 x"); break;
  }
   switch (sensor2.getOSR()) {
    case MLX90395_OSR_1: Serial.println("1 x"); break;
    case MLX90395_OSR_2: Serial.println("2 x"); break;
    case MLX90395_OSR_4: Serial.println("4 x"); break;
    case MLX90395_OSR_8: Serial.println("8 x"); break;
  }
  
  sensor.setResolution(MLX90395_RES_17);
  sensor2.setResolution(MLX90395_RES_17);
  Serial.print("Resolution: ");
  switch (sensor.getResolution()) {
    case MLX90395_RES_16: Serial.println("16b"); break;
    case MLX90395_RES_17: Serial.println("17b"); break;
    case MLX90395_RES_18: Serial.println("18b"); break;
    case MLX90395_RES_19: Serial.println("19b"); break;
  }
  switch (sensor2.getResolution()) {
    case MLX90395_RES_16: Serial.println("16b"); break;
    case MLX90395_RES_17: Serial.println("17b"); break;
    case MLX90395_RES_18: Serial.println("18b"); break;
    case MLX90395_RES_19: Serial.println("19b"); break;
  }
  
  Serial.print("Gain: "); Serial.println(sensor.getGain());
  Serial.print("Gain: "); Serial.println(sensor2.getGain());
}

void loop(void) {
  float x;
  float y;
  float z;
  Serial.print("Sensor 1: ");
  sensor.readData(&x, &y, &z);
  Serial.print("X: "); Serial.print(x);
  Serial.print(" \tY: "); Serial.print(y); 
  Serial.print(" \tZ: "); Serial.print(z); 
  Serial.println(" uTesla ");
  Serial.print("Sensor 2: ");
  sensor2.readData(&x, &y, &z);
  Serial.print("X: "); Serial.print(x);
  Serial.print(" \tY: "); Serial.print(y); 
  Serial.print(" \tZ: "); Serial.print(z); 
  Serial.println(" uTesla ");

  delay(500);
}
