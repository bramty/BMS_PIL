// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

byte data[8];

// Serial Output String Buffer
char msgString[128];                        // Array to store serial string

// CAN TX Variables
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

#define MasterID 0x100
#define SlaveID1 0x101
#define SlaveID2 0x102
#define SlaveID3 0x103

void setup()
{
  Serial.begin(115200);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
}

void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    for(byte i=0; i<8; i++){
      data[i] = rxBuf[i];
    }

    if(rxId == MasterID){
      if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data R:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data R:", rxId, len);
  
      Serial.print(msgString);
  
      if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for(byte i = 0; i<len; i++){
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }
    }        
    Serial.println();
  }

  if(millis() - prevTX >= invlTX){                    // Send this at a one second interval. 
  prevTX = millis();
  
  //Some silly data changes
  data[4] = data[4] - 0x04;
  //------------------------------

  byte sndStat = CAN0.sendMsgBuf(SlaveID1, 8, data);
    
  if(sndStat == CAN_OK)
    Serial.println("Slave 1 Status Sent Successfully!");
  else
    Serial.println("Error Sending Message...");
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
