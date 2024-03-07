//****************************************************
// BMS Master-Slave PIL
// Master Module
//
// v4, 04.03.2024
// + Arduino Nano 
// + CAN Shield
//
//**************************************************** 

#include <SPI.h>
#include <mcp_can.h>

/*  Arduino pins    CAN Shield pins
         2          INT
        13          SCK
        12          S0
        11          SI
        10          CS
        Vcc         Vcc
        GND         GND
*/

#define CAN_Int_Pin 2       // pins for the CAN module
#define CAN_CS_Pin 10

// static const byte MCP2515_CS = 10 ;
// static const byte MCP2515_INT = 2 ;

MCP_CAN CAN(CAN_CS_Pin); 

#define N_BATT 3

#define MasterID 0x100      // ID for Master
#define SlaveID1 0x101      // IDs for Slaves
#define SlaveID2 0x102
#define SlaveID3 0x103

byte Slave_ID[N_BATT] = {0x01, 0x02, 0x03};

// The structure of CAN frame 
// {ID_Master, ID_Slave, Balancing, Time_HighByte, Time_LowByte, 0, 0, 0}
// Balancing = OFF => 0x55
// Balancing = ON => 0xAA
byte TxBuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte RxBuf[8];
long unsigned int RxID;
byte len = 0;

#define Yel_LED_Pin 8    // LEDs from the Master PCB
#define Red_LED_Pin 9

unsigned long int i = 0; 
int CAN_Timeout = 1000;      // timeout for transmitting of CAN frames

#define SlaveStopBalance  0x00
#define SlaveStartBalance 0x01

int Cell_Bal[N_BATT];                // array for balancing states for all cells
int Cell_Volt[N_BATT];               // array for cells voltages
int Cell_Temp[N_BATT];               // array for cells temperatures
bool MaxViolation_Cell_Volt[N_BATT]; // array storing max voltage threshold violation state, 1:violated
bool MinViolation_Cell_Volt[N_BATT]; // array storing min voltage threshold violation state, 1:violated
bool MaxViolation_Cell_Temp[N_BATT]; // array storing max temperature threshold violation state, 1:violated
bool StateChange_Cell_Bal[N_BATT];   // array indicating changes in balancing requirement states, 1:required to change balancing state
int buff_Cell_Bal[N_BATT];           // buffer array for balancing states to send via CAN

byte j;

const int balance_diff = (int)(0.05*1000.0);    
const int MAX_V = (int)(3.6*1000.0);
const int NOM_V = (int)(3.2*1000.0);
const int MIN_V = (int)(2.5*1000.0);
const int MAX_TEMP = 45;

static uint32_t Current_Time; 
static uint32_t Last_Time; 

bool initialized;

void setup (){
  // Initialize PINs
  pinMode(Yel_LED_Pin, OUTPUT); digitalWrite(Yel_LED_Pin, LOW);
  pinMode(Red_LED_Pin, OUTPUT); digitalWrite(Red_LED_Pin, LOW);

  Serial.begin(115200);

  // Initialize CAN bus at 500 kbps, 8 MHz
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)){
    Serial.println("CAN bus initialization FAILED!");
    delay(100);
  }
  Serial.println("CAN bus initialization OK!");
  CAN.setMode(MCP_NORMAL);

  // Resetting monitoring and buffer variables
  for(j=0;j<N_BATT;j++){
    MaxViolation_Cell_Volt[j] = false;
    MinViolation_Cell_Volt[j] = false;
    MaxViolation_Cell_Temp[j] = false;
    StateChange_Cell_Bal[j] = false;
    buff_Cell_Bal[j] = 0x00;
    Cell_Bal[j] = 0;
    Cell_Volt[j] = 0;
    Cell_Temp[j] = 0;
  }

  Last_Time = millis();

  // BMS Master not yet receiving all data from BMS Slave
  initialized = false;

} // end setup()

void loop () {
  // Received CAN frames from Slave Nodes
  if (digitalRead(CAN_Int_Pin) == LOW){
    
    CAN.readMsgBuf(&RxID, len, RxBuf);

    // Data received from Cell 1
    if (RxID == 0x101){         
      Cell_Bal[0] = RxBuf[0];
      Cell_Volt[0] = RxBuf[1] * 256 + RxBuf[2];
      Cell_Temp[0] = RxBuf[3];
    }

    // Data received from Cell 2
    if (RxID == 0x102){         
      Cell_Bal[1] = RxBuf[0];
      Cell_Volt[1] = RxBuf[1] * 256 + RxBuf[2];
      Cell_Temp[1] = RxBuf[3];
    }

    // Data received from Cell 3
    if (RxID == 0x103){
      Cell_Bal[2] = RxBuf[0];
      Cell_Volt[2] = RxBuf[1] * 256 + RxBuf[2];
      Cell_Temp[2] = RxBuf[3];
    }
    
  } // end if(digitalRead(CAN_Int_Pin) == LOW) 

  // check if all data from BMS Slave is received
  if(!initialized)
    initialized = checkInitialized(Cell_Volt, Cell_Temp);

  // check balancing condition for all cells only if all data from BMS Slave is received
  if(initialized)
    checkBalancingState(Cell_Bal, Cell_Volt);

  // check cell voltages and temperatures agains their threshold
  checkCellsThreshold(Cell_Volt, Cell_Temp, MAX_V, MIN_V, MAX_TEMP);

  // check if there's a cell voltage exceeding maximum voltage threshold
  if( MaxViolation_Cell_Volt[0] ||
      MaxViolation_Cell_Volt[1] ||
      MaxViolation_Cell_Volt[2] ) {
    digitalWrite(Yel_LED_Pin, HIGH);
  } else {
    digitalWrite(Yel_LED_Pin, LOW);
  }

  // check if there's a cell voltage exceeding minimum voltage threshold
  if( MinViolation_Cell_Volt[0] ||
      MinViolation_Cell_Volt[1] ||
      MinViolation_Cell_Volt[2] ) {
    digitalWrite(Red_LED_Pin, HIGH);
  } else {
    digitalWrite(Red_LED_Pin, LOW);
  }

  // check if there's a cell temperature exceeding maximum temperature threshold
  if( MaxViolation_Cell_Temp[0] ||
      MaxViolation_Cell_Temp[1] ||
      MaxViolation_Cell_Temp[2] ){
    /*
      action when one cell temperature exceeds the threshold
    */
  } else {
    /*
      action when not one cell temperature exceeds the threshold
    */
  }

  Current_Time = millis();

  // sending balancing state update if there's any
  if (Current_Time - Last_Time > CAN_Timeout) {
    sendInfoSerial(Cell_Bal, Cell_Volt, Cell_Temp, StateChange_Cell_Bal);
    for(j=0; j<N_BATT; j++){
      if( StateChange_Cell_Bal[j] ){
        TxBuf[0] = Slave_ID[j];
        TxBuf[1] = buff_Cell_Bal[j];
        CAN.sendMsgBuf(MasterID, 0, 8, TxBuf);
        StateChange_Cell_Bal[j] = false;
        Last_Time = Current_Time;          
        delay(100);
      }
    }
  }
} // end loop()

bool checkInitialized(int voltages[N_BATT], int temp[N_BATT]){
  bool initialized = false;

  for(i=0; i<N_BATT; i++){
    if((voltages[i]!=0x00) && (temp[i]!=0x00))
      initialized = true;
    else{
      initialized = false;
      break;
    }
  }

  return initialized;
}

void sendInfoSerial(int balState[N_BATT], int voltages[N_BATT], int temp[N_BATT], bool stateChange[N_BATT]){
  Serial.println("BMS");

  Serial.print("Bal:"); 
  for (j=0; j<=N_BATT-1; j++) 
  {
    Serial.print(balState[j]);
    if (j != N_BATT-1){
      Serial.print(",");
    }
  }
  Serial.println();
    
  Serial.print("Vol:"); 
  for (j=0; j<=N_BATT-1; j++) 
  {
    Serial.print(voltages[j]);
    if (j != N_BATT-1){
      Serial.print(",");
    }
  }
  Serial.println();    

  Serial.print("Tmp:"); 
  for (j=0; j<=N_BATT-1; j++) 
  {
    Serial.print(temp[j]);
    if (j != N_BATT-1){
      Serial.print(",");
    }
  }
  Serial.println();    

  Serial.print("StateChange:"); 
  for (j=0; j<=N_BATT-1; j++) 
  {
    Serial.print(stateChange[j]);
    if (j != N_BATT-1){
      Serial.print(",");
    }
  }
  Serial.println();   
}

void checkBalancingState(int balState[N_BATT], int voltages[N_BATT]){
  int mean_voltage = (int)(voltages[0]+voltages[1]+voltages[2])/N_BATT;

  for(j=0; j<N_BATT; j++){
    if( voltages[j]-mean_voltage > balance_diff ){
      if(balState[j]==SlaveStopBalance){
        buff_Cell_Bal[j] = SlaveStartBalance; // State to be updated 0x00->0x01
        StateChange_Cell_Bal[j] = true; // Balancing state needs to be updated
      } else if ((balState[j]==SlaveStartBalance) && (StateChange_Cell_Bal[j]==true)){
        buff_Cell_Bal[j] = SlaveStartBalance; // State not to be updated
        StateChange_Cell_Bal[j] = false; // Balancing state no need to be updated 
      }
    } else {
      if(balState[j]==SlaveStartBalance){
        buff_Cell_Bal[j] = SlaveStopBalance; // State to be updated 0x01->0x00
        StateChange_Cell_Bal[j] = true; // Balancing state needs to be updated
      } else if((balState[j]==SlaveStopBalance) && (StateChange_Cell_Bal[j]==true)){
        buff_Cell_Bal[j] = SlaveStopBalance; // State not to be updated
        StateChange_Cell_Bal[j] = false; // Balancing state no need to be updated        
      }
    }
  }
} 

void checkCellsThreshold(int voltages[N_BATT], int temp[N_BATT], int max_v, int min_v, int max_temp){
  for(j=0; j<N_BATT; j++){
    // check cell voltages against the maximum threshold
    if( voltages[j] > max_v )
      MaxViolation_Cell_Volt[j] = true;
    else
      MaxViolation_Cell_Volt[j] = false;
    
    // check cell voltages against the minimum threshold    
    if( voltages[j] < min_v )
      MinViolation_Cell_Volt[j] = true;
    else
      MinViolation_Cell_Volt[j] = false;
    
    // check cell temperatures against the maximum threshold    
    if( temp[j] > max_temp )
      MaxViolation_Cell_Temp[j] = true;
    else
      MaxViolation_Cell_Temp[j] = false;
  }
}

