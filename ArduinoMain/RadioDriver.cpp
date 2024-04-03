// #include "RadioDriver.h"

// RadioDriver::RadioDriver(): rf95(RFM95_CS, RFM95_INT){
//   pinMode(RFM95_RST, OUTPUT);

//   if(!rf95.init()) {
//     DebugSerial.println("LoRa radio init failed");
//   }
//   DebugSerial.println("LoRa radio initialized");

//   // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
//   if (!rf95.setFrequency(RF95_FREQ)) {
//     Serial.println("setFrequency failed");
//   }
//   DebugSerial.print("Set Freq to: "); DebugSerial.println(RF95_FREQ);
//   // The default transmitter power is 13dBm, using PA_BOOST.
//   // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
//   // you can set transmitter powers from 5 to 23 dBm:
//   rf95.setTxPower(23, false);
// }
// //Convert string to char buffer with 0 appended to end
// void RadioDriver::transmitMessage(String msg){
//   char buf[msg.length()+1];
//   msg.toCharArray(buf, msg.length()+1);
//   buf[msg.length()] = 0;
//   rf95.send((uint8_t *)buf, msg.length()+1);
//   DebugSerial.println("Waiting for packet to complete..."); delay(10);
//   rf95.waitPacketSent();
//   // Now wait for a reply
//   /*uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//   uint8_t len = sizeof(buf);  
//   delay(1000);*/
// }