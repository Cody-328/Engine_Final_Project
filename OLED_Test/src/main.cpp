#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  delay(500);
  Serial.begin(115200);   // 9600 is OK too, 115200 is just nicer for debugging
  delay(500);

  // Use default I2C pins for this board (same ones Qwiic uses)
  Wire.begin();           // <-- this matches what worked before
  Serial.println("I2C initialized");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed!");
    while (1) {
      Serial.println("Running");
      delay(10);
    }
  }
  Serial.println("SSD1306 initialized");
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Works!");
  display.display();
}

void loop() {
}
