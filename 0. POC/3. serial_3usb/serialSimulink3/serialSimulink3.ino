#define BUFFER_SIZE 9
uint8_t buff[BUFFER_SIZE];

typedef union {
  float fval;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t yk1, yk2, rk, uk;

// the setup function runs once when you press reset or power the board
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  bool readStatus = readFromMatlab(&yk1, &yk2);
  if(readStatus)
     digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
  
  uk.fval = yk1.fval;
  rk.fval = yk2.fval;

  writeToMatlab(uk, rk);

  delay(50);
}

bool readFromMatlab( FLOATUNION_t* f1, FLOATUNION_t* f2 ){
  bool state = false;
  int count = 0;
  FLOATUNION_t f;
  
  // read the incoming bytes:
  int rlen = Serial.readBytesUntil('\n', buff, BUFFER_SIZE);
  state = true;
  for( int i=0;i<4;i++){
    f1->bytes[i]=buff[i];
  }
  for( int i=4;i<8;i++){
    f2->bytes[i-4]=buff[i];
  }
  state = true;

  return state;
}

void writeToMatlab( FLOATUNION_t fnumber, FLOATUNION_t fnumber2 ) {
  // Print header: Important to avoid sync errors!
  Serial.write('A');

  for (int i=0; i<4; i++){
	  Serial.write(fnumber.bytes[i]); 
  }
  for (int i=0; i<4; i++){
	  Serial.write(fnumber2.bytes[i]); 
  }
  Serial.print('\n');
}

