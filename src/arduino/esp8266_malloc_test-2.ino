/////////////////////////////////////////////////////////////////////////////////////////////////
// Date: 2017-03-24
/////////////////////////////////////////////////////////////////////////////////////////////////

#define NUM_BLOCKS             (8)
#define NUM_BLOCK_DATA_UNITS   (250)
#define NUM_DATA_UNITS_TOTAL   (NUM_BLOCKS * NUM_BLOCK_DATA_UNITS)

/////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _data_unit {
  uint16_t cnt;
  int16_t  volt;
  int16_t  current;
  uint16_t ts;
} data_unit_t;

#define TOTAL_MEM_ALLOC        (sizeof(data_unit_t) * NUM_DATA_UNITS_TOTAL)

static data_unit_t *buf[ NUM_BLOCKS ][ NUM_BLOCK_DATA_UNITS ];

void setup() {
  Serial.begin(921600);
  delay(100);
  randomSeed( analogRead(A0) );


  for ( int j=0; j < NUM_BLOCKS; j++ ) {
    for ( int i=0; i < NUM_BLOCK_DATA_UNITS; i++ ) {
       buf[j][i] = (data_unit_t *)malloc( sizeof(data_unit_t) );
    }
  }

  Serial.printf( "\n\n\n\Total memory allocated: %ld bytes\n", TOTAL_MEM_ALLOC );
  Serial.printf( "Total data records: %ld\n", NUM_DATA_UNITS_TOTAL );

  data_unit_t *p;
  for (int i=0; i < NUM_DATA_UNITS_TOTAL; i++) {
     p = buf[i/NUM_BLOCK_DATA_UNITS][i%NUM_BLOCK_DATA_UNITS];
     p->cnt     = (uint16_t)i;
     p->volt    = (int16_t)random(4500,5500);
     p->current = (int16_t)random(0,500);
     p->ts      = (uint16_t)random(2500,3500);
  }
  delay(2000);
}

void loop() {
  data_unit_t *p;
  for (int i=0; i < NUM_DATA_UNITS_TOTAL; i++) {
     p = buf[i/NUM_BLOCK_DATA_UNITS][i%NUM_BLOCK_DATA_UNITS];
     Serial.printf( "%u,%d,%d,%d,%ld\n", p->cnt, p->volt, p->current, (p->volt * p->current)/1000, p->ts );
     delay(5);
  }
  delay(5000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
