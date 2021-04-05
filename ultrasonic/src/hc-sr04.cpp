/*
 * Code for interfacing a 32U4 with the SR-HR04 ultrasonic sensor. 
 * 
 * This uses the Input Capture feature of the ATmega32U4 (e.g., Leonardo) to get precision readings.
 * Specifically, you must connect the pulse width pin to pin 13 (ICP3) on the 32U4.
 * You are welcome to use whatever pin you want for triggering a ping, just be sure to change it from the default.
 * 
 * The input capture first looks for a rising edge, then a falling edge
 * The difference between the two is the pulse width, which is a direct measurement 
 * of the (round trip) timer counts to hear the echo.
 * 
 * But note that the timing is in timer counts, which must be converted to time.
 */

#include <Arduino.h>
#include <Romi32U4.h>
float IRAvg;
void find_med(float (*prevArr)[5], float *med_val);
float USAvg; 
float prevValUS[5];
int prevValUSPH = 0;
int prevValIRPH = 0;
float prevValIR[5];
float USMed;
float IRMed;
Romi32U4Motors motors; 
int currEff = 0;
int lastEff = 0;
float distanceIR;

unsigned long timeCheck = 0;
float kp = 13;
float ki = .08;
float kd = 0;
float iTerm = 0;
int counters = 0;
int dIn = 0;
void movePID(int targetDistCM) {
float AvgMed = (USMed + IRMed)/2;
unsigned long int currTime = millis();
  if(currTime-timeCheck >= 10) {
      lastEff = currEff;

  float error = targetDistCM - AvgMed;
    
    if(abs(error) < 2) 
     iTerm += error;
     else iTerm = 0; 
  
    currEff = (kp*error) + ki*iTerm - (kd*dIn);
    dIn = currEff - lastEff; // Isn't this always zero??
    counters++; 
    if(counters > 10) {
    Serial.print(IRAvg);
    Serial.print('\t');
    Serial.print(IRMed);
    Serial.print('\t');
    Serial.print(distanceIR);
    Serial.print('\n');
    counters = 0;
    }
    timeCheck += 10;
  }
  //motors.setEfforts(-currEff, -currEff);
  motors.setEfforts(0, 0);
}


void rollingAvg(float (*prevVal)[5], float *Avg_Val);
void Volt_SharpIR();
float USslope = 0.0172558087; // 1/57.9515
float USintercept = 0.7771446813; // 45.0367/57.9515
volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;
const int IRPin = A0;


//define the states for the echo capture
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};

//and initialize to IDLE
volatile PULSE_STATE pulseState = PLS_IDLE;

//this may be most any pin, connect the pin to Trig on the sensor
const uint8_t trigPin = 14;

//for scheduling pings
uint32_t lastPing = 0;
const uint32_t PING_INTERVAL = 100; //ms

/*
 * Commands the ultrasonic to take a reading
 */
void CommandPing(int trigPin)
{
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;
  
  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //we'll allow a delay here for convenience; it's only 10 us
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

void setup()
{
  Serial.begin(115200);
 // while(!Serial) {} //you must open the Serial Monitor to get past this step!
  Serial.println("setup");
  pinMode(IRPin, INPUT);
  noInterrupts(); //disable interupts while we mess with the control registers
  
  //sets timer 3 to normal mode (16-bit, fast counter)
  TCCR3A = 0; 
  
  interrupts(); //re-enable interrupts

  //note that the Arduino machinery has already set the prescaler elsewhere
  //so we'll print out the value of the register to figure out what it is
  Serial.print("TCCR3B = ");
  Serial.println(TCCR3B, HEX);

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)

  lastPing = millis();

  Serial.println("/setup");
}

void loop() 
{
  
  //schedule pings roughly every PING_INTERVAL milliseconds
  uint32_t currTime = millis();
  if((currTime - lastPing) >= PING_INTERVAL && pulseState == PLS_IDLE)
  {
    lastPing = currTime;
    CommandPing(trigPin); //command a ping
  }
  
  if(pulseState == PLS_CAPTURED) //we got an echo
  {
    //update the state to IDLE
    pulseState = PLS_IDLE;

    /*
     * Calculate the length of the pulse (in timer counts!). Note that we turn off
     * interrupts for a VERY short period so that there is no risk of the ISR changing
     * pulseEnd or pulseStart. The way the state machine works, this wouldn't 
     * really be a problem, but best practice is to ensure that no side effects can occur.
     */
    noInterrupts();
    uint16_t pulseLengthTimerCounts = pulseEnd - pulseStart;
    interrupts();
    
    //EDIT THIS LINE: convert pulseLengthTimerCounts, which is in timer counts, to time, in us
    //You'll need the clock frequency and the pre-scaler to convert timer counts to time
    
    uint32_t pulseLengthUS = pulseLengthTimerCounts*4; //pulse length in us 4uS per tick of timer
    
    //EDIT THIS LINE AFTER YOU CALIBRATE THE SENSOR: put your formula in for converting us -> cm
    float distancePulse = USslope*pulseLengthUS + USintercept;    //distance in cm
    // Array that holds the last 5 sets of distances
    prevValUS[prevValUSPH] = distancePulse;
    find_med(&prevValUS, &USMed);
    rollingAvg(&prevValUS, &USAvg);
    prevValUSPH = (prevValUSPH == 4)? 0 : prevValUSPH + 1;
    // float Average = rollingAvg();
    // float Median = find_med();

    // Serial.print(millis());
    // Serial.print('\t');
    // Serial.print(pulseLengthTimerCounts);
    // Serial.print('\t');
    // Serial.print(pulseLengthUS);
    // Serial.print('\t');
    // Serial.print(distancePulse);
    // Serial.print('\t');
    // Serial.print(Average);
    // Serial.print('\t');
    // Serial.print(Median);
    // Serial.print('\n');
    
  }
  Volt_SharpIR();
  movePID(20);
}

/*
 * ISR for input capture on pin 13. We can precisely capture the value of TIMER3
 * by setting TCCR3B to capture either a rising or falling edge. This ISR
 * then reads the captured value (stored in ICR3) and copies it to the appropriate
 * variable.
 */
ISR(TIMER3_CAPT_vect)
{
  if(pulseState == PLS_WAITING_LOW) //we're waiting for a rising edge
  {
    pulseStart = ICR3; //copy the input capture register (timer count)
    TCCR3B &= 0xBF;    //now set to capture falling edge on pin 13
    pulseState = PLS_WAITING_HIGH;
  }

  else if(pulseState == PLS_WAITING_HIGH) //waiting for the falling edge
  {
    pulseEnd = ICR3;
    pulseState = PLS_CAPTURED; //raise a flag to indicate that we have data
  }
}

void rollingAvg (float (*prevArr)[5], float *Avg_Val) {
  float sumAvg = 0;
  float avg;
  
  for (int i = 0; i < 5; i++) {
    sumAvg = sumAvg + (*prevArr)[i];
  }
  avg = sumAvg / 5;

  *Avg_Val = avg;
}

/*
void swapNum(int *p, int *q) {
  int t;
  
  t=*p;
  *p=q;
  *q=t;
}

void sortArray(int a[], int n) {
  int i,j,temp;

  for(i=0; i < n-1; i++) {
    for (j=0, j < n-i-1; j++) {
      if(a[j] > a[j+1])
        swapNum(&a[j],&a[j+1]);
    }
  }
}

// float Median (float prevVal) {
//   int a = prevVal;
//   int n = 5;
//   int sum, i;

//   sortArray(a,n);

//   n = (n+1) / 2 - 1;

//   printf("Median Distance = %d ", a[n] );

//   return 0;
// }

*/
uint32_t lastTime = 0;
float VoltageIR = 0;
float multiplier_ADC = 0.0048828125; //5/1024
uint32_t currentTime;
unsigned int INTERVAL = 52; 
int ADCVal = 0;
int counter = 0;
float IR_A_Coeff = 32.3744; // Function is y = A / (x+B) - > x = A/y - B
float IR_B_Coeff = 5.1941;
void Volt_SharpIR() {

  currentTime = micros();
  if(currentTime - lastTime >= INTERVAL)
  {
    ADCVal = analogRead(IRPin);
    VoltageIR = ADCVal*multiplier_ADC;
    
    distanceIR = (IR_A_Coeff / VoltageIR) - IR_B_Coeff;
    prevValIR[prevValIRPH] = distanceIR;
    find_med(&prevValIR, &IRMed);
    rollingAvg(&prevValIR, &IRAvg);
    prevValIRPH = (prevValIRPH == 4)? 0 : prevValIRPH + 1;
    lastTime = currentTime;
    // counter++;
    // if(counter > 1000) {
    // Serial.print(millis());
    // Serial.print('\t');
    // Serial.print(ADC);
    // Serial.print('\t');
    // Serial.print(VoltageIR);
    // Serial.print('\t');
    // Serial.print(distanceIR);
    // Serial.print('\n');
    // counter = 0;
    // }

  }

  
}
float min_val;
int min_valInd;
void find_med(float (*prevArr)[5], float *med_val) {
    min_val = (*prevArr)[0];
    for(int j = 0; j < 3; j++) {
     min_val = (*prevArr)[j];   
    for(int i = j+1; i < 5; i++) {
        if(min_val > (*prevArr)[i]) {
            min_valInd = i;
            min_val = (*prevArr)[i];
        }
    }
    if(min_val != (*prevArr)[j]) {
        (*prevArr)[min_valInd] = (*prevArr)[j];
        (*prevArr)[j] = min_val;
    }
    
    }
    *med_val = min_val;
}
// float min_val;
// int min_valInd;
// float prevValUS[5];
// float prevValIR[5];
// float find_med(float *prevArr[]) {
//     min_val = prevVal[0];
//     for(int j = 0; j < 3; j++) {
//      min_val = prevVal[j];   
//     for(int i = j+1; i < 5; i++) {
//         if(min_val > prevVal[i]) {
//             min_valInd = i;
//             min_val = prevVal[i];
//         }
//     }
//     if(min_val != prevVal[j]) {
//         prevVal[min_valInd] = prevVal[j];
//         prevVal[j] = min_val;
//     }
    
//     }
//     return min_val;
// }
