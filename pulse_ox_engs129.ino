#include <SPI.h>
#include <TimerThree.h>

#define baudRate 115200
byte data[2];
int RED_LED_DATA_POINT;
int IR_LED_DATA_POINT;

// SPI pins
const int CSPin   = 10;
const int MOSIPin = 11;
const int MISOPin0 = 12; // connect PMODAD1.D0 to Teensy.pin12
const int MISOPin1 = 39; // connect PMODAD1.D1 to Teensy.pin39
const int SCKPin  = 14;
const int IR = 17;  
const int RED = 16; 

const int MAX_ADC_VALUE = 4096;
const int MIN_ADC_VALUE = 0;
const float LOWER_THRESH_ADC_VOLTAGE = 1.0;
const float UPPER_THRESH_ADC_VOLTAGE = 2.5;
const float TARGET_ADC_VOLTAGE = 2.0;

const int MAX_STATE = 90;
const int MIN_STATE = 20;
const int PERIOD = 200; // 200*10us = 2 ms
const int DEFAULT_STATE_LENGTH = 40;

int RED_ON_STATE_LENGTH = DEFAULT_STATE_LENGTH;
int RED_OFF_STATE_LENGTH = (PERIOD >> 1)-RED_ON_STATE_LENGTH;
int IR_ON_STATE_LENGTH = DEFAULT_STATE_LENGTH;
int IR_OFF_STATE_LENGTH = (PERIOD >> 1)-IR_ON_STATE_LENGTH;

const int AVG_DATA_POINTS = 50; 
int RED_LED_AVG_BUFFER[AVG_DATA_POINTS];
int IR_LED_AVG_BUFFER[AVG_DATA_POINTS];
const int BPM_DATA_POINTS = 200; 
int BPM_BUFFER[BPM_DATA_POINTS];
int AVERAGE_SUM_COUNT = 0;
int avgarrayIndex = 0;

int RED_LED_MAX = MIN_ADC_VALUE;
int IR_LED_MAX = MIN_ADC_VALUE;
int RED_LED_MIN = MAX_ADC_VALUE;
int IR_LED_MIN = MAX_ADC_VALUE;

int CURRENT_BUFFER = 0;
int BPM_BUFFER_INDEX = 0;
int state = 0;  
volatile int count = 0; // use volatile for shared variables
int countCopy;          // holds a copy of the count
volatile int ADCCount = 0;

int RED_LED_SUM = 0;
int IR_LED_SUM = 0;

// SPI Settings: speed, mode and endianness
SPISettings settings(1000000, MSBFIRST, SPI_MODE2);  // 1MHz, MSB, 

void setup() {
  // put your setup code here, to run once:
  pinMode(IR, OUTPUT);  // designate pin 13 an output pin
  pinMode(RED, OUTPUT); // designate pin 14 an output pin
  Timer3.initialize(10);
  Timer3.attachInterrupt(ISR); // call ISR every 25 us

  pinMode(CSPin, OUTPUT);
  Serial.begin(baudRate);

  // Configure SPI Pins
  SPI.begin();
  SPI.setMISO(MISOPin0);
  SPI.setMOSI(MOSIPin);
  SPI.setSCK(SCKPin);
}

// keep count, in 25 us divisions
void ISR(void)
{  
  count = count + 1;
  ADCCount = ADCCount + 1;
}

void loop()
{   
  

  noInterrupts();
  countCopy = count;
  interrupts();
  
  // finite state machine    
  switch (state) {
    case 0:
      if (countCopy >= RED_ON_STATE_LENGTH){
          noInterrupts();
          count = 0;
          state = 1;
          interrupts();
        }
        else{
        // red ON
        digitalWrite(RED, HIGH);                  
        }
      break;
    case 1:
      if (countCopy >= RED_OFF_STATE_LENGTH){
          noInterrupts();
          count = 0;
          state = 2;
          interrupts();       
        }
        else{
        // red OFF                  
        digitalWrite(RED, LOW);
        }
      break;
    case 2:
      if (countCopy >= IR_ON_STATE_LENGTH){
          noInterrupts();
          count = 0;
          state = 3;
          interrupts();       
        }
        else{
        // IR ON                  
        digitalWrite(IR, HIGH);
        }
      break;
    case 3:
      if (countCopy >= IR_OFF_STATE_LENGTH){
          noInterrupts();
          count = 0;
          state = 0;
          interrupts();       
        }
        else{
        // IR OFF     
        digitalWrite(IR, LOW);
        }
      break;
  }

  if (ADCCount >= 100) {
     //read ADC0
    getADC(data, MISOPin0);
    RED_LED_DATA_POINT = ((data[0] << 8) + data[1]);
    RED_LED_SUM += RED_LED_DATA_POINT;
    //read ADC1
    getADC(data, MISOPin1);  
    IR_LED_DATA_POINT = ((data[0] << 8) + data[1]);
    IR_LED_SUM += IR_LED_DATA_POINT;
    AVERAGE_SUM_COUNT += 1;

    if (AVERAGE_SUM_COUNT == 20) {
      RED_LED_AVG_BUFFER[avgarrayIndex] = RED_LED_SUM / 20;
      IR_LED_AVG_BUFFER[avgarrayIndex] = IR_LED_SUM / 20;
      

      if (RED_LED_AVG_BUFFER[avgarrayIndex] > RED_LED_MAX) {
        RED_LED_MAX = RED_LED_AVG_BUFFER[avgarrayIndex];
      }
      if (IR_LED_AVG_BUFFER[avgarrayIndex] > IR_LED_MAX) {
        IR_LED_MAX = IR_LED_AVG_BUFFER[avgarrayIndex];
      }
      if (RED_LED_AVG_BUFFER[avgarrayIndex] < RED_LED_MIN) {
        RED_LED_MIN = RED_LED_AVG_BUFFER[avgarrayIndex];
      }
      if (IR_LED_AVG_BUFFER[avgarrayIndex] < IR_LED_MIN) {
        IR_LED_MIN = IR_LED_AVG_BUFFER[avgarrayIndex];
      }

      RED_LED_SUM = 0;
      IR_LED_SUM = 0;

      // Serial.print(RED_LED_AVG_BUFFER[avgarrayIndex]);
      // Serial.print("\t");
      // Serial.print(IR_LED_AVG_BUFFER[avgarrayIndex]);
      // Serial.print("\t");

      if (CURRENT_BUFFER >= 6){
        BPM_BUFFER[BPM_BUFFER_INDEX] = IR_LED_AVG_BUFFER[avgarrayIndex];
        BPM_BUFFER_INDEX += 1;
      }
      avgarrayIndex += 1;
    }

    if (avgarrayIndex == AVG_DATA_POINTS){
      CURRENT_BUFFER += 1;
      avgarrayIndex  = 0;
      if (CURRENT_BUFFER < 10) {
        RED_LED_MAX = MIN_ADC_VALUE;
        IR_LED_MAX = MIN_ADC_VALUE;
        RED_LED_MIN = MAX_ADC_VALUE;
        IR_LED_MIN = MAX_ADC_VALUE;
      }
    }
      
    if (CURRENT_BUFFER == 10) {
      float RED_LED_PEAK_TO_PEAK_VOLTAGE = ((float)(RED_LED_MAX - RED_LED_MIN)) / 4096.0 * 3.3;
      Serial.print("RED_LED_PEAK_TO_PEAK_VOLTAGE (V): ");
      Serial.print(RED_LED_PEAK_TO_PEAK_VOLTAGE);

      float RED_LED_DUTY_CYCLE = (float)(RED_ON_STATE_LENGTH)/2.0/100.0;
      Serial.print("\n\rRED_LED_DUTY_CYCLE (%): ");
      Serial.print(RED_LED_DUTY_CYCLE);
      
      float IR_LED_PEAK_TO_PEAK_VOLTAGE = ((float)(IR_LED_MAX - IR_LED_MIN)) / 4096.0 * 3.3;
      Serial.print("\n\rIR_LED_PEAK_TO_PEAK_VOLTAGE (V): ");
      Serial.print(IR_LED_PEAK_TO_PEAK_VOLTAGE);

      float IR_LED_DUTY_CYCLE = (float)(IR_ON_STATE_LENGTH)/2.0/100.0;
      Serial.print("\n\rIR_LED_DUTY_CYCLE (%): ");
      Serial.print(IR_LED_DUTY_CYCLE);
      
      float RATIO = (RED_LED_DUTY_CYCLE*RED_LED_PEAK_TO_PEAK_VOLTAGE) / (IR_LED_DUTY_CYCLE*IR_LED_PEAK_TO_PEAK_VOLTAGE);
      // Serial.print("\n\rRATIO: ");
      // Serial.print(RATIO);

      float bloodOxygen = -28.9 * RATIO + 114;
      Serial.print("\n\rBlood Oxygen (%): ");
      Serial.print(bloodOxygen);

      float beatsPerMinute = getBeatsPerMinute(BPM_BUFFER, IR_LED_MIN, IR_LED_MAX);
      
      Serial.print("\n\rBeats per minute (bpm): ");
      Serial.print(beatsPerMinute);
      Serial.print("\n\r---------------------------------------------\n\r"); 

      RED_ON_STATE_LENGTH = getDutyCycle(RED_LED_PEAK_TO_PEAK_VOLTAGE, RED_ON_STATE_LENGTH);
      IR_ON_STATE_LENGTH = getDutyCycle(IR_LED_PEAK_TO_PEAK_VOLTAGE, IR_ON_STATE_LENGTH);

    // adjust RED_ON_STATE_LENGTH and IR_ON_STATE_LENGTH according to desired duty cycle -- 8-15
      RED_OFF_STATE_LENGTH=(PERIOD >> 1)-RED_ON_STATE_LENGTH;  // state1 lasts (500 us minus the state0 duration)
      IR_OFF_STATE_LENGTH=(PERIOD >> 1)-IR_ON_STATE_LENGTH;  // state3 lasts (500 us minus the state2 duration)

      CURRENT_BUFFER = 0;
      BPM_BUFFER_INDEX = 0;
      RED_LED_MAX = MIN_ADC_VALUE;
      IR_LED_MAX = MIN_ADC_VALUE;
      RED_LED_MIN = MAX_ADC_VALUE;
      IR_LED_MIN = MAX_ADC_VALUE;
    }
    if (AVERAGE_SUM_COUNT == 20) {
      AVERAGE_SUM_COUNT = 0;
      // Serial.print("\n\r"); 
    }
    
    ADCCount = 0;
  }
}

int getDutyCycle(float peakToPeak, int state) {
  // MAX DUTY CYCLE 45%
  // Min Duty Cycle 10%
  int updatedState = state;

  if (peakToPeak < LOWER_THRESH_ADC_VOLTAGE || peakToPeak > UPPER_THRESH_ADC_VOLTAGE) {
    int tempState =(int) ((TARGET_ADC_VOLTAGE/peakToPeak)*(float)(state));
    if (tempState > MAX_STATE) {
      updatedState = MAX_STATE;
    }
    else if (tempState < MIN_STATE) {
      updatedState = MIN_STATE;
    }
    else {
      updatedState = tempState;
    }
  }
  return updatedState;
}

float getBeatsPerMinute(int bpmBuffer[BPM_DATA_POINTS], int min, int max) {
  int startingPoint = BPM_DATA_POINTS - 1; 
  while (bpmBuffer[startingPoint] != min) {
        startingPoint -=1;
  }

  int firstMax = 0;
  int firstTimestamp = 0;
  int secondMax = 0;
  int secondTimestamp = 0;

  for (int i = startingPoint; i > startingPoint - 25; i--) {
    if (firstMax < bpmBuffer[i]) {
      firstMax = bpmBuffer[i];
      firstTimestamp = i * 20;
    }
  }

  for (int i = startingPoint - 25; i > startingPoint - 75; i --) {
    if (secondMax < bpmBuffer[i]) {
      secondMax = bpmBuffer[i];
      secondTimestamp = i * 20;
    }
  }

  float beatsPerMinute = 60000.0 / ((float)(firstTimestamp - secondTimestamp));
  return beatsPerMinute;
}

void getADC(byte* data, int whichMISO) {
    SPI.setMISO(whichMISO);    // set MISO pin
    SPI.beginTransaction(settings);
    digitalWrite(CSPin,LOW);   // pull CSPin low to start SPI communication
    data[0] = SPI.transfer(0); // grab upper byte
    data[1] = SPI.transfer(0); // grab lower byte    
    digitalWrite(CSPin,HIGH);  // set CSPin high to end SPI communication
    SPI.endTransaction();
}
