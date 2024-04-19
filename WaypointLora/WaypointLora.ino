/* Onboard Lora to read incoming messages and send data to base station
*/
#include <SPI.h>
#include <RH_RF95.h>
#include "Constants.h"
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!PISerial);
  PISerial.begin(115200); //Must be same baud rate as Arduino Main
  PISerial.setTimeout(10); //avoid long delays to mistimed reads

  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    DebugSerial.println("LoRa radio init failed");
  }
  DebugSerial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    DebugSerial.println("setFrequency failed");
  }
  DebugSerial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

void loop()
{
  if(PISerial.available()>0){ //Send message
    String data = PISerial.readStringUntil('\n');
    if(data.substring(0,data.indexOf(":"))=="R"){
    char radiopacket[data.length()-1] = "";
    data.substring(data.indexOf(":")+1).toCharArray(radiopacket, data.length()-1);//add everything after : to message packet
    radiopacket[data.length()-2] = 0; //Must set last bit to 0
    rf95.send((uint8_t *)radiopacket, data.length()-1);
    rf95.waitPacketSent();
    }
    else{
      PISerial.println("Couldn't parse radio message");
    }
  }

  if (rf95.waitAvailableTimeout(10)){ //Listen for message
    if (rf95.recv(buf, &len)){
      DebugSerial.print("Got message: ");
      DebugSerial.println((char*)buf);
      DebugSerial.print("RSSI: ");
      DebugSerial.println(rf95.lastRssi(), DEC);    
      PISerial.println((char*)buf);
    }
  }
}