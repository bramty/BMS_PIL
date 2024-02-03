// CAN Send Example
//

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

// Serial Output String Buffer
char msgString[128];

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);     // Set CS to pin 10

#define MasterID 0x100
#define SlaveID1 0x101
#define SlaveID2 0x102

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  pinMode(CAN0_INT, INPUT);
}

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    for(byte i=0; i<8; i++){
      data[i] = rxBuf[i];
    }

    if(rxId == SlaveID1){
      if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  Data Slave 1:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, "Standard ID: 0x%.3lX  Data Slave 1 :", rxId, len);
  
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
    }else if(rxId == SlaveID2){
      if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  Data Slave 2:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, "Standard ID: 0x%.3lX  Data Slave 2 :", rxId, len);
  
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
    }else{
      Serial.println("ID non recognized");
    }
    
    Serial.println();
  }

  // Some silly data changes
  data[4] = data[4] + 0x01;
  //--------------------------------------

  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(MasterID, 0, 8, data);

  if(sndStat == CAN_OK){
    Serial.println("Master Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  delay(250);   // send data per 250ms
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
