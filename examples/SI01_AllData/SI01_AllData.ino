/*************************************************************
  This is an examples for getting all critical data from SI01

  You can buy one on our store!
  -----> https://xinabox.cc/products/SI01

  Supported on the all ☒CHIPs
  
  The sensor communicates over the I2C Bus.
  
*************************************************************/

#include <xCore.h>
#include <xSI01.h>

xSI01 SI01;

#define PRINT_SPEED 250
static unsigned long lastPrint = 0;

void setup() {
  // Start the Serial Monitor at 115200 BAUD
  Serial.begin(115200);
  
	// Set the I2C Pins for CW01
	#ifdef ESP8266
	  Wire.pins(2, 14);
	  Wire.setClockStretchLimit(15000);
	#endif
  
  if (!SI01.begin()) {
    Serial.println("Failed to communicate with SI01.");
    Serial.println("Check the Connector");
  } else {
    Serial.println("start successful");
  }
  millis();
}

void loop() {
  // Read and calculate data from SL01 sensor
  SI01.poll();

  if ( (lastPrint + PRINT_SPEED) < millis()) {
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    printAttitude(); // Print Roll, Pitch and G-Force
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
}

void printGyro(void) {
  Serial.print("G: ");
  Serial.print(SI01.getGX(), 2);
  Serial.print(", ");
  Serial.print(SI01.getGY(), 2);
  Serial.print(", ");
  Serial.println(SI01.getGZ(), 2);

}

void printAccel(void) {
  Serial.print("A: ");
  Serial.print(SI01.getAX(), 2);
  Serial.print(", ");
  Serial.print(SI01.getAY(), 2);
  Serial.print(", ");
  Serial.println(SI01.getAZ(), 2);
}

void printMag(void) {
  Serial.print("M: ");
  Serial.print(SI01.getMX(), 2);
  Serial.print(", ");
  Serial.print(SI01.getMY(), 2);
  Serial.print(", ");
  Serial.println(SI01.getMZ(), 2);

}

void printAttitude(void) {
  Serial.print("Roll: ");
  Serial.println(SI01.getRoll(), 2);
  Serial.print("Pitch :");
  Serial.println(SI01.getPitch(), 2);
  Serial.print("GForce :");
  Serial.println(SI01.getGForce(), 2);
}
