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

#define CAN_EspeedL_1000 0xA0
#define CAN_EspeedH_1000 0x0F
#define CAN_VspeedL_20 0xC8 // 20 km/h
#define CAN_VspeedH_20 0x00
#define ChkSumOffset_0xAA  85 
#define ChkSumOffset_0x130  1 //?
#define ChkSumOffset_0x1A0 94
#define ChkSumOffset_0xC8 108

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

int EChkSum, E2ChkSum, CChkSum, VChkSum, SChkSum, EMsgCtr, E2MsgCtr, CMsgCtr, VMsgCtr, SMsgCtr = 0;

CanFrame rxFrame;

void sendAccPedalFrame(uint8_t AccPedal) { // 20 ms
	CanFrame AccPedalFrame = { 0 };
	AccPedalFrame.identifier = 0xAA;
	AccPedalFrame.extd = 0;
	AccPedalFrame.data_length_code = 8;
	AccPedalFrame.data[0] = EChkSum;
	AccPedalFrame.data[1] = 0x40 + EMsgCtr;
	AccPedalFrame.data[2] = 0x1A;
	AccPedalFrame.data[3] = 0x5C;    
	AccPedalFrame.data[4] = CAN_EspeedL_1000;   
	AccPedalFrame.data[5] = CAN_EspeedH_1000;   
	AccPedalFrame.data[6] = 0x94;
	AccPedalFrame.data[7] = 0x00;
    // Accepts both pointers and references 
  ESP32Can.writeFrame(AccPedalFrame);  // timeout defaults to 1 ms
}

void sendEngineDataFrame(uint8_t EngineData) { // 20 ms
	CanFrame EngineDataFrame = { 0 };
	EngineDataFrame.identifier = 0x1D0;
	EngineDataFrame.extd = 0;
	EngineDataFrame.data_length_code = 8;
	EngineDataFrame.data[0] = 0x80;
	EngineDataFrame.data[1] = 0x80;
	EngineDataFrame.data[2] = 0x60 + E2MsgCtr;
	EngineDataFrame.data[3] = 0xD0;    
	EngineDataFrame.data[4] = 0xCE;   
	EngineDataFrame.data[5] = 0x3A;   
	EngineDataFrame.data[6] = 0x0D;
	EngineDataFrame.data[7] = 0x91;
      // Accepts both pointers and references 
  ESP32Can.writeFrame(EngineDataFrame);  // timeout defaults to 1 ms
}

/*
void sendTerminalStatusFrame(uint8_t CAS) { // 20 ms
	CanFrame TerminalStatus = { 0 };
	TerminalStatus.identifier = 0x130;
	TerminalStatus.extd = 0;
	TerminalStatus.data_length_code = 5;
	TerminalStatus.data[0] = 0x45;
	TerminalStatus.data[1] = 0x03;
	TerminalStatus.data[2] = 0x29;
	TerminalStatus.data[3] = 0x0F;    
	TerminalStatus.data[4] = CMsgCtr + 0x00; // H nibble is counter and L nibble is checksum
	    // Accepts both pointers and references 
  ESP32Can.writeFrame(TerminalStatus);  // timeout defaults to 1 ms
}
*/
void sendSpeedFrame(uint8_t Speed) { // 100 ms
	CanFrame SpeedFrame = { 0 };
	SpeedFrame.identifier = 0x1A0;
	SpeedFrame.extd = 0;
	SpeedFrame.data_length_code = 8;
	SpeedFrame.data[0] = 0x90;
	SpeedFrame.data[1] = 0x11;
	SpeedFrame.data[2] = 0x00;
	SpeedFrame.data[3] = 0x00;    
	SpeedFrame.data[4] = 0x00;   
	SpeedFrame.data[5] = 0x00;   
	SpeedFrame.data[6] = VMsgCtr * 0x10;
	SpeedFrame.data[7] = VChkSum;
    // Accepts both pointers and references 
  ESP32Can.writeFrame(SpeedFrame);  // timeout defaults to 1 ms
}
/*
void sendSteerAngFrame(uint8_t SteerAng) { // 200 ms
	CanFrame SteerAngFrame = { 0 };
	SteerAngFrame.identifier = 0xC8;
	SteerAngFrame.extd = 0;
	SteerAngFrame.data_length_code = 6;
	SteerAngFrame.data[0] = 0xBA;
	SteerAngFrame.data[1] = 0x02;
	SteerAngFrame.data[2] = SMsgCtr;
	SteerAngFrame.data[3] = 0x00;    
	SteerAngFrame.data[4] = 0x00;   
	SteerAngFrame.data[5] = SChkSum;   
	  // Accepts both pointers and references 
  ESP32Can.writeFrame(SteerAngFrame);  // timeout defaults to 1 ms
}
*/

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
  static uint32_t ElastStamp, ClastStamp, VlastStamp, SlastStamp = 0;
  static uint32_t E2lastStamp = 50;
  uint32_t currentStamp = millis();
  
/*  
  if(currentStamp - ElastStamp > 19) {   // sends frame every 20 ms
      if (EMsgCtr < 14) EMsgCtr++; else EMsgCtr = 0;
      // EChkSum = EMsgCtr + ChkSumOffset_0xAA;
      EChkSum = EMsgCtr + 0xF5; // Test with precalculated values
      ElastStamp = currentStamp;
      sendEspeedFrame(EMsgCtr);
      Serial.print(ElastStamp);
      Serial.print(" E \n\r");
  }    
*/

  if(currentStamp - E2lastStamp > 99) {   // sends frame every 100 ms
      if (E2MsgCtr < 14) E2MsgCtr++; else E2MsgCtr = 0;
      E2lastStamp = currentStamp;
      sendEngineDataFrame(E2MsgCtr); //0x1D0
      Serial.print(E2lastStamp);
      Serial.print(" 0x1D0 \n\r");
  }    

/*
  if(currentStamp - ClastStamp > 99) {   // sends frame every 100 ms
      if (CMsgCtr < 14) CMsgCtr++; else CMsgCtr = 0;
      CChkSum = (CMsgCtr * 0x10) + (CMsgCtr + 0xB5) % 0x10; // Test with precalculated values
      ClastStamp = currentStamp;
      sendCASFrame(CMsgCtr);
      Serial.print(ClastStamp);
      Serial.print(" C \n\r");
  }
  */
  if(currentStamp - VlastStamp > 159) {   // sends frame every 160 ms
      if (VMsgCtr < 14) VMsgCtr++; else VMsgCtr = 0;
      VChkSum = (VMsgCtr * 0x10) + 0xA1;
      VChkSum = VChkSum % 0x100;  // Precalculated checksum offset value
      VlastStamp = currentStamp;
      sendSpeedFrame(VMsgCtr); // 0x1A0
      Serial.print(VlastStamp);
      Serial.print(" 0x1A0 \n\r");
  }
/*  
  if(currentStamp - SlastStamp > 199) {   // sends frame every 200 ms
      if (SMsgCtr < 14) SMsgCtr++; else SMsgCtr = 0;
      // SChkSum = SMsgCtr + ChkSumOffset_0xC8;
      SChkSum = SMsgCtr + 0x50;  // Test with precalculated values
      SlastStamp = currentStamp;
      sendSteerAngFrame(SMsgCtr);
      Serial.print(SlastStamp);
      Serial.print(" S \n\r");
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





