#include <IOexpander.h>
#include <Wire.h>;
#include <inttypes.h>;

IOexpander IOexp;


void setup()
{
  Serial.begin(9600);
  Serial.println("IOexpander example");

  
  if(IOexp.init(0x20, MCP23016))
    Serial.println("Communication with IOexpander works!");
  else
    Serial.println("No communication with the IOexpander!!");  

  pinMode(13,OUTPUT);
  IOexp.pinModePort(0,OUTPUT); 
  IOexp.pinModePort(1,INPUT);
}

int sw = 0;
int x = 100;
long next = 1000;
long next2 = 1000;


void loop()
{
  //blinks port 0.7 to 0.4
  
  if(millis() >= next){
    IOexp.digitalWritePort(0,LOW);
    switch(sw){
      case 0:
        IOexp.digitalWrite(0,4,HIGH);
        sw++;
        break;
      case 1:
        IOexp.digitalWrite(0,5,HIGH);
        sw++;
        break;
      case 2:
        IOexp.digitalWrite(0,6,HIGH);
        sw++;
      break;
      case 3:
        IOexp.digitalWrite(0,7,HIGH);
        sw= 0;
        Serial.print("next = ");Serial.println(next,DEC);  
      break; 
      default:
        sw= 0;
        break;
    }
    next = millis() + x;
  }
  
  //check the buttons on de IO expander
  if(millis() >= next2){
    if(IOexp.digitalRead(1,0)){
      digitalWrite(13,HIGH);
      x = 30;
    }else {
      x = 100;
      digitalWrite(13,LOW);
    }
    next2 = millis() + 50;
  }
}
