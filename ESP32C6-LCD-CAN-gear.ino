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

#define CAN_EspeedL_1000 0xA0 // 1000 rpm
#define CAN_EspeedH_1000 0x0F
#define CAN_VspeedL_0xAA_20 0xC8 // 20 km/h
#define CAN_VspeedH_0xAA_20 0x00

#define ChkSumOffset_AA   245 
#define ChkSumOffset_C4   108
#define ChkSumOffset_C8   108
#define ChkSumOffset_130   11 
#define ChkSumOffset_130_B 199
#define ChkSumOffset_1A0  162
#define ChkSumOffset_1D0   94

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

int ChkSum_AA, ChkSum_C4, ChkSum_C8, ChkSum_130, ChkSum_1A0, ChkSum_1D0 = 0; 
int MsgCtr_AA, MsgCtr_C4, MsgCtr_C8, MsgCtr_130, MsgCtr_1A0, MsgCtr_1D0 = 0;

CanFrame rxFrame;


void send_0xAA_Frame(uint8_t AccPedal) { // 20 ms // AccPedal
	CanFrame _0xAA_Frame = { 0 };
	_0xAA_Frame.identifier = 0xAA; // 170
	_0xAA_Frame.extd = 0;
	_0xAA_Frame.data_length_code = 8;
	_0xAA_Frame.data[0] = ChkSum_AA;
	_0xAA_Frame.data[1] = 0x40 + MsgCtr_AA;
	_0xAA_Frame.data[2] = 0x1A;
	_0xAA_Frame.data[3] = 0x5C;    
	_0xAA_Frame.data[4] = 0x00; //CAN_EspeedL_1000;   
	_0xAA_Frame.data[5] = 0x00; //CAN_EspeedH_1000;   
	_0xAA_Frame.data[6] = 0x94;
	_0xAA_Frame.data[7] = 0x00;
  ESP32Can.writeFrame(_0xAA_Frame);  // timeout defaults to 1 ms
}

void send_0xC4_Frame(uint8_t SteerAng) { // 10 ms // SteeringWheelAngle
	CanFrame _0xC4_Frame = { 0 };
	_0xC4_Frame.identifier = 0xC4; // 196
	_0xC4_Frame.extd = 0;
	_0xC4_Frame.data_length_code = 7;
	_0xC4_Frame.data[0] = 0xBD;
	_0xC4_Frame.data[1] = 0x02;
	_0xC4_Frame.data[2] = 0xFC;
	_0xC4_Frame.data[3] = 0x00;    
	_0xC4_Frame.data[4] = 0x00;   
	_0xC4_Frame.data[5] = 0xFF;   
	_0xC4_Frame.data[6] = 0xF1;   
  ESP32Can.writeFrame(_0xC4_Frame);  // timeout defaults to 1 ms
}

void send_0xC8_Frame(uint8_t SteerAng) { // 200 ms // SteeringWheelAngle_slow
	CanFrame _0xC8_Frame = { 0 };
	_0xC8_Frame.identifier = 0xC8; // 200
	_0xC8_Frame.extd = 0;
	_0xC8_Frame.data_length_code = 6;
	_0xC8_Frame.data[0] = 0xBA;
	_0xC8_Frame.data[1] = 0x02;
	_0xC8_Frame.data[2] = MsgCtr_C8;
	_0xC8_Frame.data[3] = 0x00;    
	_0xC8_Frame.data[4] = 0x00;   
	_0xC8_Frame.data[5] = ChkSum_C8;   
  ESP32Can.writeFrame(_0xC8_Frame);  // timeout defaults to 1 ms
}

void send_0x130_Frame(uint8_t TerminalStatus) { // 100 ms // TerminalStatus
	CanFrame _0x130_Frame = { 0 };
	_0x130_Frame.identifier = 0x130; // 304
	_0x130_Frame.extd = 0;
	_0x130_Frame.data_length_code = 5;
	_0x130_Frame.data[0] = 0x45;
	_0x130_Frame.data[1] = 0x43;
	_0x130_Frame.data[2] = 0x29;
	_0x130_Frame.data[3] = 0x0F;    
	_0x130_Frame.data[4] = ChkSum_130; // H nibble is checksum and L nibble is counter
  ESP32Can.writeFrame(_0x130_Frame);  // timeout defaults to 1 ms
}

void send_0x1A0_Frame(uint8_t Speed) { // 160 ms // Speed
	CanFrame _0x1A0_Frame = { 0 };
	_0x1A0_Frame.identifier = 0x1A0; // 416
	_0x1A0_Frame.extd = 0;
	_0x1A0_Frame.data_length_code = 8;
	_0x1A0_Frame.data[0] = 0xF0;
	_0x1A0_Frame.data[1] = 0x10;
	_0x1A0_Frame.data[2] = 0x00;
	_0x1A0_Frame.data[3] = 0x00;    
	_0x1A0_Frame.data[4] = 0x00;   
	_0x1A0_Frame.data[5] = 0x00;   
	_0x1A0_Frame.data[6] = (MsgCtr_1A0 * 0x10) + 0x08; // H nibble is counter
	_0x1A0_Frame.data[7] = ChkSum_1A0;
  ESP32Can.writeFrame(_0x1A0_Frame);  // timeout defaults to 1 ms
}

void send_0x1D0_Frame(uint8_t EngineData) { // 200 ms // EngineData
	CanFrame _0x1D0_Frame = { 0 };
	_0x1D0_Frame.identifier = 0x1D0; // 464
	_0x1D0_Frame.extd = 0;
	_0x1D0_Frame.data_length_code = 8;
	_0x1D0_Frame.data[0] = 0x7C;
	_0x1D0_Frame.data[1] = 0xFF;
	_0x1D0_Frame.data[2] = 0x40 + MsgCtr_1D0;
	_0x1D0_Frame.data[3] = 0xB1;    
	_0x1D0_Frame.data[4] = 0x2D;   
	_0x1D0_Frame.data[5] = 0xB6;   
	_0x1D0_Frame.data[6] = 0xCC;
	_0x1D0_Frame.data[7] = 0xA6;
  ESP32Can.writeFrame(_0x1D0_Frame);  // timeout defaults to 1 ms
}

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
  
  tft.init(TFT_WIDTH, TFT_HEIGTH, SPI_MODE3);           // Init ST7789 172x320
  tft.setRotation(2); // 2 means USB connector points down
  digitalWrite(TFT_BL,1); // Backlight intensity
  tft.setSPISpeed(40000000); //Default is 40000000
  
  ESP32Can.setPins(CAN_TX, CAN_RX);
	ESP32Can.setRxQueueSize(5);
	ESP32Can.setTxQueueSize(5);

  if(ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }  

  tft.fillScreen(ST77XX_BLUE);
  tft.setTextSize(4);
   
  Serial.println(F("Initialized"));

}

void loop() {
  static int lastgear, lastlockup = 0;
  int gear, lockup = 0;
  static uint32_t lastStamp_AA_20ms, lastStamp_C4_10ms, lastStamp_C8_200ms, lastStamp_130_100ms, lastStamp_1A0_160ms, lastStamp_1D0_200ms  = 0;
  uint32_t currentStamp = millis();

  if(currentStamp - lastStamp_AA_20ms > 19) {   // sends frame every 20 ms
      if (MsgCtr_AA < 14) MsgCtr_AA++; else MsgCtr_AA = 0;
      ChkSum_AA = (MsgCtr_AA + ChkSumOffset_AA) % 0x100; // Test with precalculated values
      lastStamp_AA_20ms = currentStamp;
      send_0xAA_Frame(MsgCtr_AA);
      Serial.print(lastStamp_AA_20ms);
      Serial.print(" 0xAA \n\r");
  }    
/*
  if(currentStamp - lastStamp_C4_10ms > 9) {   // sends frame every 10 ms
      if (MsgCtr_C4 < 14) MsgCtr_C4++; else MsgCtr_C4 = 0;
      // ChkSum_C4 = (MsgCtr_C4 + ChkSumOffset_C4) % 0x100;
      lastStamp_C4_10ms = currentStamp;
      send_0xC4_Frame(MsgCtr_C4);
      Serial.print(lastStamp_C4_10ms);
      Serial.print(" 0xC4 \n\r");
  }    
*/
/*
  if(currentStamp - lastStamp_C8_200ms > 199) {   // sends frame every 200 ms
      if (MsgCtr_C8 < 14) MsgCtr_C8++; else MsgCtr_C8 = 0;
      ChkSum_C8 = ((MsgCtr_C8 * 0x10) + ChkSumOffset_C8) % 0x100; // Test with precalculated values
      lastStamp_C8_200ms = currentStamp;
      send_0xC8_Frame(MsgCtr_C8);
      Serial.print(lastStamp_C8_200ms);
      Serial.print(" 0xC8 \n\r");
  }
*/
  if(currentStamp - lastStamp_130_100ms > 99) {   // sends frame every 100 ms
      if (MsgCtr_130 < 14) MsgCtr_130++; else MsgCtr_130 = 0;
      // ChkSum_130 = (((MsgCtr_130 + ChkSumOffset_130) % 0x10) * 0x10) + MsgCtr_C4;
      ChkSum_130 = ((MsgCtr_130 + ChkSumOffset_130_B) % 0x10);
      ChkSum_130 = (ChkSum_130 * 0x10) + MsgCtr_130;
      lastStamp_130_100ms = currentStamp;
      send_0x130_Frame(MsgCtr_130); // 0x1A0
      Serial.print(lastStamp_130_100ms);
      Serial.print(" 0x130 \n\r");
  }

  if(currentStamp - lastStamp_1A0_160ms > 159) {   // sends frame every 160 ms
      if (MsgCtr_1A0 < 14) MsgCtr_1A0++; else MsgCtr_1A0 = 0;
      ChkSum_1A0 = ((MsgCtr_1A0 * 0x10) + ChkSumOffset_1A0) % 0x100;  // Test with precalculated values
      lastStamp_1A0_160ms = currentStamp;
      send_0x1A0_Frame(MsgCtr_1A0);
      Serial.print(lastStamp_1A0_160ms);
      Serial.print(" 0x1A0 \n\r");
  }
/*
  if(currentStamp - lastStamp_1D0_200ms > 199) {   // sends frame every 200 ms
      if (MsgCtr_1D0 < 14) MsgCtr_1D0++; else MsgCtr_1D0 = 0;
      // ChkSum_1D0 = (MsgCtr_1D0 + ChkSumOffset_1D0) % 0x100;  // Test with precalculated values
      lastStamp_1D0_200ms = currentStamp;
      send_0x1D0_Frame(MsgCtr_1D0);
      Serial.print(lastStamp_1D0_200ms);
      Serial.print(" 0x1D0 \n\r");
  }
*/
/*
  if(ESP32Can.readFrame(rxFrame, 100)) { // You can set custom timeout, default is 1000
       //Serial.printf("Received frame: %03X \r\n", rxFrame.identifier);
      if(rxFrame.identifier == 0x1A0) {   // 
          Serial.printf("V-spd: %3dkm/h \r\n", rxFrame.data[0]); 
      }
  }
*/
/*
  gear = (digitalRead(SOLENOID_A) + (digitalRead(SOLENOID_B) * 2));
  lockup = (digitalRead(TCC));
  if (gear != lastgear) {
    lastgear = gear;
    tft.fillRect(50, 100, 40, 50, ST77XX_BLUE);
    tft.setCursor(50, 100);
    tft.print(gear+1);
    }
if (lockup != lastlockup) {
    lastlockup = lockup;
    tft.fillRect(100, 100, 40, 50, ST77XX_BLUE);
    tft.setCursor(100, 100);
    if (lockup) tft.print("L");
    else tft.print("U");
    }
*/
}





