// CAN and dicrete IO test version
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX		1 //5
#define CAN_RX		0 //4
#define SOLENOID_A  18
#define SOLENOID_B  19
#define TCC         20
#define BOOT_PIN    9
#define SD_CS     4
#define SD_MISO   5
#define SD_MOSI   6
#define SD_SCLK   7
#define RGB_IO    8

#define TFT_CS        14
#define TFT_DC        15
#define TFT_RST       21 // Or set to -1 and connect to Arduino RESET pin
#define TFT_MISO      5
#define TFT_MOSI      6
#define TFT_SCLK      7
#define TFT_RST       21
#define TFT_BL        22 // Backlight

#define TFT_WIDTH   172
#define TFT_HEIGTH  320


// OPTION 1 (recommended) is to use the HARDWARE SPI pins, which are unique
// to each board and not reassignable. For Arduino Uno: MOSI = pin 11 and
// SCLK = pin 13. This is the fastest mode of operation and is required if
// using the breakout board's microSD card.

// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// OPTION 2 lets you interface the display using ANY TWO or THREE PINS,
// tradeoff being that performance is not as fast as hardware SPI above.
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 13  // Clock out

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

CanFrame rxFrame;
void sendCANFrame(uint8_t obdId) {
	CanFrame obdFrame = { 0 };
	obdFrame.identifier = 0x650;
	obdFrame.extd = 0;
	obdFrame.data_length_code = 8;
	obdFrame.data[0] = 2;
	obdFrame.data[1] = 1;
	obdFrame.data[2] = obdId;
	obdFrame.data[3] = 0x00;    
	obdFrame.data[4] = 0x00;   
	obdFrame.data[5] = 0x00;   
	obdFrame.data[6] = 0x00;
	obdFrame.data[7] = 0x00;
    // Accepts both pointers and references 
  ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}

float p = 3.1415926;

void setup(void) {
  pinMode(SOLENOID_A, INPUT);   
  pinMode(SOLENOID_B, INPUT);   
  pinMode(TCC, INPUT);   
  pinMode(BOOT_PIN, INPUT);
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_BL, OUTPUT);

//  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
  Serial.begin(115200);
  delay(1000);

  tft.init(TFT_WIDTH, TFT_HEIGTH, SPI_MODE3);           // Init ST7789 172x320
  tft.setRotation(2); // 2 means USB connector points down
  digitalWrite(TFT_BL,100); // Backlight intensity
  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  tft.setSPISpeed(40000000); //Default is 40000000
  
  ESP32Can.setPins(CAN_TX, CAN_RX);
	ESP32Can.setRxQueueSize(5);
	ESP32Can.setTxQueueSize(5);

  if(ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }  

  Serial.println(F("Initialized"));

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(4);
  tft.fillScreen(ST77XX_BLUE);
  tft.setCursor(50, 100);
  tft.print(1);
  delay(200);
  tft.fillRect(50, 100, 30, 40, ST77XX_BLUE);
  tft.setCursor(50, 100);
  tft.print(2);
  delay(200);
  tft.fillRect(50, 100, 30, 40, ST77XX_BLUE);
  tft.setCursor(50, 100);
  tft.print(3);
  delay(200);
  tft.fillRect(50, 100, 30, 40, ST77XX_BLUE);
  tft.setCursor(50, 100);
  tft.print(4);
  Serial.println("Setup done");

}

void loop() {
  int gear, lockup = 0;
  static uint32_t lastStamp = 0;
  uint32_t currentStamp = millis();
  
  if(currentStamp - lastStamp > 100) {   // sends OBD2 request every 100 ms
      lastStamp = currentStamp;
      sendCANFrame(5);
      Serial.printf("CAN TX\r");
  }

  // You can set custom timeout, default is 1000
  if(ESP32Can.readFrame(rxFrame, 100)) {
      // Comment out if too many requests 
      Serial.printf("Received frame: %03X \r\n", rxFrame.identifier);
      if(rxFrame.identifier == 0x7E8) {   // Standard OBD2 frame responce ID
          Serial.printf("Coolant temp: %3d°C \r\n", rxFrame.data[3] - 40); // Convert to °C
      }
  }
  gear = (digitalRead(SOLENOID_A) + (digitalRead(SOLENOID_B) * 2));
  lockup = (digitalRead(TCC));
  if (!digitalRead(BOOT_PIN)) tft.invertDisplay(false);
  else tft.invertDisplay(true);
  tft.fillRect(50, 100, 30, 40, ST77XX_BLUE);
  tft.setCursor(50, 100);
  tft.print(gear+1);
  // Serial.println("Loop");
}


void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t y=0; y < tft.height(); y+=5) {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x=0; x < tft.width(); x+=5) {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}

void testdrawrects(uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x=tft.width()-1; x > 6; x-=6) {
    tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
  }
}

void testroundrects() {
  tft.fillScreen(ST77XX_BLACK);
  uint16_t color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = tft.width()-2;
    int h = tft.height()-2;
    for(i = 0 ; i <= 16; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}



