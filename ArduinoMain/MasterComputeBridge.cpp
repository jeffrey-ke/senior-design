/* This class controls the communication between the arduino and PI.
  It contains all the necesary drivers for the different sensors.
*/
#include "MasterComputeBridge.h"

MasterComputeBridge::MasterComputeBridge(){
  thrusterSetup();
}
void MasterComputeBridge::thrusterSetup(){
  thruster1 = ThrusterDriver(TPIN1);
  thruster2 = ThrusterDriver(TPIN2);
  thruster3 = ThrusterDriver(TPIN3);
  thruster4 = ThrusterDriver(TPIN4);
}
/* Parses command read from PI and executes associated command. After executing takes result to 
   build the return message which is stored in member variable functionReturn 
*/
void MasterComputeBridge::giveCommand(String command){
  int seperator = command.indexOf(":");
  //Parse and execute Thruster command
  if(command.substring(0,seperator) == "T"){
    int id = command.substring(seperator+1, command.indexOf(",",seperator+1)).toInt();
    seperator = command.indexOf(",",seperator+1);
    int vel = command.substring(seperator+1).toInt();
    if(id==1){ thruster1.setVelocity(vel); }
    else if(id==2){ thruster2.setVelocity(vel); }
    else if(id==3){ thruster3.setVelocity(vel); }
    else if(id==4){ thruster4.setVelocity(vel); }
    functionReturn = "T:"; //Build return string
    functionReturn.concat(String(id));
    functionReturn.concat(",");
    functionReturn.concat(String(vel));
    DebugSerial.println(functionReturn);
  }
  //Parse and execute Radio command
  else if(command.substring(0,seperator) == "R"){
    String msg = command.substring(seperator+1);
   //Lora.transmitMessage(msg);
  }
  //Parse and execute Ping command
  else if(command.substring(0,seperator) == "P"){
      
  }
  //Parse and execute IMU command
  else if(command.substring(0,seperator) == "I"){
    sensors_event_t orientationData = IMU.getData();
    functionReturn = "I:"; //Build return string
    functionReturn.concat(String(orientationData.orientation.x));
    functionReturn.concat(",");
    functionReturn.concat(String(orientationData.orientation.y));
    functionReturn.concat(",");
    functionReturn.concat(String(orientationData.orientation.z));
    DebugSerial.println(functionReturn);
  }
  //Parse and execute GPS command
  else if(command.substring(0,seperator) == "G"){
    double* coords = GPS.getData();
    functionReturn = "G:"; //Build return string
    functionReturn.concat(String(coords[0]));
    functionReturn.concat(",");
    functionReturn.concat(String(coords[1]));
  }
  //Parse and execute EStop command
  else if(command.substring(0,seperator) == "E"){

  }
  else {
    DebugSerial.print("Unable to parse command- ");
    DebugSerial.println(command);
  } 
}
/* Returns the function return message and clears it. If it is already empty returns warning
*/
String MasterComputeBridge::returnCommand(){
  if(functionReturn==""){
    return "No return message found";
  }
  String temp = functionReturn;
  functionReturn = "";
  return temp;
}
