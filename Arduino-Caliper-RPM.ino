#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

#define CLOCK_PIN_CALIPER 3   // pin for caliper clock
#define DATA_PIN_CALIPER  11  // pin for caliper data
#define BUTTON_PIN 10         // pin for a button
#define TRIGGER_PIN_RPM 2     // pin for rotation trigger

// LCD
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

/* excellent caliper documentation see:
 * http://www.shumatech.com/support/chinese_scales.htm
 * or
 * https://www.yuriystoys.com/2013/07/chinese-caliper-data-format.html
 * 
 * also interessting solution:
 * https://github.com/urill/read_caliper
 * 
 * also important for measurement improvement:
 * HW hack: https://www.instructables.com/id/How-To-Fix-Digital-Calipers/
 */


// CONST RPM
const byte QTY_MAGNETS = 1; // quantity of used magnets
const int RINGBUFFER_SIZE_RPM = 5;  // quantity of values for median calculation
const int MIN_RPM = 2;  // min. roations per minute; below is 0
const int MIN_DIFFTIME_RPM = 4000;  // min. time of trigger diffenrence (debounce)
const unsigned long MAX_DIFFTIME_RPM = 60000000 / MIN_RPM; // 60000000 ... 1 minute


// CONST CALIPER
const unsigned long SEQSTART_CALIPER = 20000L; // Threshold in microseconds which signals a new sequence start on CLOCK_PIN
const byte MESSAGESIZE_CALIPER = 24; // containing the measure data, sign and unit



struct caliperValues {
   float absdist;
   float reldist;
   String unit;
   String debug;
};

struct caliperValues currentCaliperValues;
struct caliperValues lcdValueCaliper;

// UI
long buttonTimer = 0;
long longPressTime = 3000;

boolean buttonActive = false;
boolean longPressActive = false;
boolean hasLcdToBeUpdatedRPM = false;
boolean hasLcdToBeUpdatedCaliper = false;


// CALIPER
float absdistOffset = 0.0;
float reldistOffset = 0.0;

volatile unsigned long timeLastTriggerCaliper = 0;
unsigned long timeDiffCaliper = 0; // difference between current and last timestamp (as micros)
volatile int bitCounterCaliper = 0;

volatile boolean currentUnitCaliper = "";
volatile int currentValueCaliper = 0;
volatile int collectingValueCaliper = 0;


// RPM
volatile unsigned long timeLastTriggerRPM = 0;
unsigned long timeDiffRPM = 0;
unsigned long currentTimeDiffRPM = 0;

boolean useRingBuffer = true;
volatile uint8_t bufferCounter = 0;
long ringBuffer[RINGBUFFER_SIZE_RPM];
long ringBuffer_save[RINGBUFFER_SIZE_RPM];

volatile boolean isRPMTriggered = false;
unsigned long lastTimeDiffRPM = 0UL;

float measuredRPM = 0.0;
float lcdValueRPM = 0.0;

unsigned long timeCurrentTriggerRPM = 0UL;
unsigned long timeCurrentTriggerCaliper = 0UL;


/* quicksort start
 *from http://www.codecodex.com/wiki/Quicksort 
 */
void quicksort(long x[], long first, long last) {  
    long pivIndex = 0;  
    if(first < last) {  
        pivIndex = partition(x,first, last);  
        quicksort(x,first,(pivIndex-1));  
        quicksort(x,(pivIndex+1),last);  
    }  
}  
  
long partition(long y[], long f, long l) {  
    long up,down,temp;  
    long piv = y[f];  
    up = f;  
    down = l;  
    goto partLS;  
    do {   
        temp = y[up];  
        y[up] = y[down];  
        y[down] = temp;  
    partLS:  
        while (y[up] <= piv && up < l) {  
            up++;  
        }  
        while (y[down] > piv  && down > f ) {  
            down--;  
        }  
    } while (down > up);  
    y[f] = y[down];  
    y[down] = piv;  
    return down;  
}  
/* quicksort end */


/**
 * Interrupt function: get delta Time between last and current trigger timestamp 
 */
void rotationTrigger() {
  timeCurrentTriggerRPM = micros();
  if ( (timeCurrentTriggerRPM - timeLastTriggerRPM) > MIN_DIFFTIME_RPM) {
    timeDiffRPM = timeCurrentTriggerRPM - timeLastTriggerRPM;
    timeLastTriggerRPM = timeCurrentTriggerRPM;

    if (useRingBuffer) {
      ringBuffer[bufferCounter++ % RINGBUFFER_SIZE_RPM]=(long)timeDiffRPM;
    }

    isRPMTriggered = true;
  }
}


/**
 * Interrupt function: get caliper bit sequence
 */
void caliperTrigger() {
  int data = digitalRead(DATA_PIN_CALIPER);

  timeDiffCaliper = micros() - timeLastTriggerCaliper;
  timeLastTriggerCaliper = micros();
 
  if (timeDiffCaliper > SEQSTART_CALIPER) {  // new sequence start
    bitCounterCaliper=0;
    collectingValueCaliper = 0;
  }

  if (bitCounterCaliper<21) {
    if (data) collectingValueCaliper|= 1<<(bitCounterCaliper-1);
  }
    
  if (bitCounterCaliper == 21 && data == 1) collectingValueCaliper *= -1;

  if (bitCounterCaliper ==23) {
    currentValueCaliper = collectingValueCaliper;
    currentUnitCaliper = data;
  }
  
  bitCounterCaliper++;
}
 

/*
 * helper routines
 */
boolean equals (caliperValues a, caliperValues b) {
  boolean isEqual = false;
  if (a.absdist == b.absdist && a.reldist == b.reldist) isEqual = true;
  return isEqual;
}

void lcd_write_float(int col, int row, float aValue, String text) {
  char buf[9];
  dtostrf(aValue,9,2,buf);

  lcd.setCursor(col, row);
  lcd.print(buf+text);
}


/*
 * SETUP
 */
void setup() 
{
  Serial.begin( 115200 );

  // init buffer
  for (uint8_t i=0;i<RINGBUFFER_SIZE_RPM;i++) {
    ringBuffer[i]=0;
  }
  
  // init LCD
  // See http://playground.arduino.cc/Main/I2cScanner
  int error;
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  if (error) {
    Serial.print("Error: LCD not found");
    Serial.print(error);
  }

  lcd.begin(20, 4); // initialize the lcd
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();

  // caliper
  pinMode( CLOCK_PIN_CALIPER, INPUT );
  pinMode( DATA_PIN_CALIPER, INPUT );
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN_CALIPER), caliperTrigger, FALLING);

  // button
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // RPM, tachometer
  pinMode(TRIGGER_PIN_RPM, INPUT);       // Pin 2 ist INT0
  digitalWrite(TRIGGER_PIN_RPM, HIGH);   // internal PullUp
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN_RPM), rotationTrigger, FALLING);
  
  //Serial.print( "Ready:" );
  hasLcdToBeUpdatedRPM = true;
  hasLcdToBeUpdatedCaliper = true;
}

/*
 * LOOP
 */
void loop () {
  struct caliperValues myCaliperValues;
  
  currentTimeDiffRPM = timeDiffRPM;
  lastTimeDiffRPM = micros() - timeLastTriggerRPM;

  if (!isRPMTriggered && currentTimeDiffRPM>0) {
    if (lastTimeDiffRPM > currentTimeDiffRPM) {
      if (useRingBuffer) {
        ringBuffer[bufferCounter++ % RINGBUFFER_SIZE_RPM]=(long)lastTimeDiffRPM;
      }
    }
  }


  // calculate MEDIAN of currentTimeDiffRPM
  if (useRingBuffer) {
    memcpy(ringBuffer_save, ringBuffer, sizeof(ringBuffer[0])*RINGBUFFER_SIZE_RPM);
    quicksort(ringBuffer_save, 0, RINGBUFFER_SIZE_RPM);
    currentTimeDiffRPM = ringBuffer_save[(int) (RINGBUFFER_SIZE_RPM / 2) ];
  }  
  
  // calculate RPM
  boolean isMeasureInTime = (MIN_RPM != NULL && 
    ((micros()-timeLastTriggerRPM) < MAX_DIFFTIME_RPM) &&
    (currentTimeDiffRPM < MAX_DIFFTIME_RPM) );
    
  measuredRPM = isMeasureInTime && currentTimeDiffRPM > 0 ? 60000000 / currentTimeDiffRPM / QTY_MAGNETS: 0;
  if (measuredRPM != lcdValueRPM) {
    hasLcdToBeUpdatedRPM = true;
    lcdValueRPM = measuredRPM;
  }

  isRPMTriggered = false;


  // calculate distance
  myCaliperValues.absdist = currentValueCaliper / 100.000;
  myCaliperValues.reldist = currentValueCaliper / 100.000;
  myCaliperValues.unit = (currentUnitCaliper?"in":"mm");

  if (!equals(lcdValueCaliper, myCaliperValues)) {
    hasLcdToBeUpdatedCaliper = true;
    lcdValueCaliper = myCaliperValues;
  }

  // UI
  if (hasLcdToBeUpdatedRPM || hasLcdToBeUpdatedCaliper) {
    // lcd.clear();
    String text1 = " "+ lcdValueCaliper.unit+" abs";
    String text2 = " "+ lcdValueCaliper.unit+" rel";
    if (hasLcdToBeUpdatedRPM) lcd_write_float(3,0,lcdValueRPM ," RPM");
    if (hasLcdToBeUpdatedCaliper) lcd_write_float(3,1,lcdValueCaliper.absdist-absdistOffset,text1);
    if (hasLcdToBeUpdatedCaliper) lcd_write_float(3,2,lcdValueCaliper.reldist-reldistOffset,text2);
    hasLcdToBeUpdatedRPM = false;
    hasLcdToBeUpdatedCaliper = false;
  }

  // button
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonActive) {
      buttonActive = true;
      buttonTimer = millis();
      
      reldistOffset = lcdValueCaliper.reldist;
      hasLcdToBeUpdatedCaliper = true;
    }
    if ((millis() - buttonTimer > longPressTime) && (!longPressActive)) {
      longPressActive = true;
      absdistOffset = lcdValueCaliper.absdist;
      reldistOffset = lcdValueCaliper.reldist;
      hasLcdToBeUpdatedCaliper = true;
    }
    
    digitalWrite(LED_BUILTIN, longPressActive);
    lcd.setCursor(0, 0);
    lcd.print("*");
  } else {
    if (buttonActive) {
      if (longPressActive) {
        longPressActive = false;
        absdistOffset = lcdValueCaliper.absdist;
        reldistOffset = lcdValueCaliper.reldist;
      } else {
        reldistOffset = lcdValueCaliper.reldist;
      }
      digitalWrite(LED_BUILTIN, longPressActive);
      buttonActive = false;
      hasLcdToBeUpdatedCaliper = true;

      lcd.setCursor(0, 0);
      lcd.print(" ");
    }
  }
}
