#define NUM_BLOCKS          (2)
#define NUM_BLOCK_MESSAGES  (250)
#define STRING_SIZE         (80)

static char *buf[ NUM_BLOCKS ][ NUM_BLOCK_MESSAGES ];

void setup() {
  Serial.begin(115200);
  for ( int j=0; j < NUM_BLOCKS; j++ ) {
    for ( int i=0; i < NUM_BLOCK_MESSAGES; i++ ) {
       buf[j][i] = (char *)malloc(sizeof(char)* STRING_SIZE ); 
    }
  }
  
  for (int i=0; i < (NUM_BLOCKS*NUM_BLOCK_MESSAGES); i++) {
     sprintf( buf[i/NUM_BLOCK_MESSAGES][i%NUM_BLOCK_MESSAGES], "counting variable: %08lu", i );
  }
}

void loop() {
  for (int i=0; i < (NUM_BLOCKS*NUM_BLOCK_MESSAGES); i++) {
     Serial.println( buf[i/NUM_BLOCK_MESSAGES][i%NUM_BLOCK_MESSAGES] );
     delay(10);
  }
  delay(5000);

}

