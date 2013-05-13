#include <C:\Users\jakub\electronics\microscope_1_3\wavetable.h>         // table with waveforms
// Do not add anything before the above line
// Do not edit or delete the first line


/***********************************/
/* Define constants in the program */
/***********************************/
// define pins
const int dac7741_RW = 6;      // output pin for dac7741 read/~write
const int dac7741_RST = 7;     // output pin for dac7741 DAC ~reset
const int dac7741_RSTSEL = 8;  // output pin for dac7741 DAC ~reset select (keep high)
const int goingupX = 13;       // output pin showing that the stage is moving in +X direction
const int piezo = 12;          // output pin piezo
const int cameraReadyInt = 3;  // Camera ready Interrupt on pin 20
const int cameraReadyPin = 20; // Camera ready pin number
const int moveOnOffPin = 21;   // on pin 21: TRUE: acquisition, FALSE: wait
const int reverseDirPin = 3;   // Reverse/forward scan direction pin number

/***********************************/
/* Define variables in the program */
/***********************************/
// Timer2 reload value, globally available, defines how often the DAC is updated */  
byte tcnt2 = 10;
int toggle = 0;  
int dX = 1;
int dY = 1;
int X = 0;
int Y = 0;
int Z = 0;
unsigned int Xpos = 0;
unsigned int Ypos = 0;
unsigned int Zpos = 0;
unsigned int i = 0;

byte tY = 0;
byte iZ = 0;
/* byte stopY[] = {2, 2};         // How many X scans to remain on Y turn position
                               // {2, 2} for bidirectional scan
                               // {3, 1} for single directional scan
*/

void ioinit (void)
{
  /************************************/
  /* Define 16-bit data port for DACs */
  /************************************/
  PORTA = B00000000;   // lower byte LSB(PA0, digital pin 22), MSB(PA7, digital pin 29)
  DDRA = B11111111;    // data direction register A
  PORTC = B00000000;   // upper byte LSB(PC0, digital pin 37), MSB(PC7, digital pin 30)
  DDRC = B11111111;    // data direction register C
  DDRL = B11111111;    // data direction register L
  PORTL = B11111111;   // CS and LDAC (PL0, digital pin 49), MSB(PL7, digital pin 42)

  /* Configure control pins for DACs */
  pinMode(dac7741_RW, OUTPUT);
  pinMode(dac7741_RST, OUTPUT);
  pinMode(dac7741_RSTSEL, OUTPUT);

  /* Configure the pin with the diode to output to show interrupt behavior */
  pinMode(goingupX, OUTPUT);

  /* Configure the pin with the piezo to output */
  pinMode(piezo, OUTPUT);

  /* Configure pin 21 as output telling other Arduino if scanning is active */
  pinMode(moveOnOffPin, OUTPUT);
  
  /* Configure pin 20 as input listening if camera is acquiring */
  pinMode(cameraReadyPin, INPUT);

  /* Configure pin 3 as output: TRUE reverse, FALSE forward */
  pinMode(reverseDirPin, OUTPUT);
  pinMode(A15, OUTPUT);
}

void resetDAC()
{
  /* Initialize the DAC */
  digitalWrite(dac7741_RW, 0);      // keep 0 to write into the DAC
  digitalWrite(dac7741_RSTSEL, 1);  // keep 1 to reset the DAC to midscale
  digitalWrite(dac7741_RST, 0);     // go to zero to reset the DAC
  digitalWrite(dac7741_RST, 1);     // keep 1 to resume operation
  //for(i=0; i<1000; i++){}
  //delay(stagewait);                 // Give the stage 20 ms to move to the center
}

void moveX(unsigned int pos)
{
  PORTA=lowByte(pos);
  PORTC=highByte(pos);
  PORTL=B11001111;        // CS one  and A0,A1,A2 = 100  W26.1
  PORTL=B01001111;        // CS zero and A0,A1,A2 = 100  W26.1
  PORTL=B11111110;
  PORTL=B11111111;
}

void moveY(unsigned int pos)
{
  PORTA=lowByte(pos);
  PORTC=highByte(pos);
  PORTL=B10101111;        // CS one  and A0,A1,A2 = 010  W27.1
  PORTL=B00101111;        // CS zero and A0,A1,A2 = 010  W27.1
  PORTL=B11111110;
  PORTL=B11111111;
}

void moveZ(unsigned int pos)
{
  PORTA=lowByte(pos);
  PORTC=highByte(pos);
  PORTL=B11101111;        // CS one  and A0,A1,A2 = 110  W12.3
  PORTL=B01101111;        // CS zero and A0,A1,A2 = 110  W12.3
  PORTL=B11111110;
  PORTL=B11111111;
  //digitalWrite(13, upY);
}

void timerstop()
{
  /* First disable the timer overflow interrup */
  TIMSK2 &= ~(1<<TOIE2);
  /* Reload the timer */
  TCNT2 = tcnt2;

  /* Tell other Arduino that scanning is over */
  digitalWrite(moveOnOffPin, 0);
  X = 0;
  Y = 0;
  Z = 0;
  dX = 1;
  dY = 1;
}

void stopRun()
{
  timerstop();

  /* Reset the DAC to initial position */
  resetDAC();
}

void endBeep()
{
  /* Sound the beeper to tell that scanning finishes */
  for(i=0; i<800; i++)
  {
    digitalWrite(piezo, 1);
    delayMicroseconds(125);
    digitalWrite(piezo, 0);
    delayMicroseconds(125);
  }
  delay(100);
  for(i=0; i<800; i++)
  {
    digitalWrite(piezo, 1);
    delayMicroseconds(160);
    digitalWrite(piezo, 0);
    delayMicroseconds(160);
  }
}

void incrementZ()
{
  Z = Z + 1;
  if (Z % 2)
  {
    digitalWrite(A15, LOW);
  }
  else
  {
    digitalWrite(A15, HIGH);
  }
  
  if (Z==__wavepointsZ__)
  {
    stopRun();
    endBeep();
  }
}

void incrementY()
{
  Y = Y + dY;              // increment to the next Y
  if (dY==0)
  {
    incrementZ();
    Zpos = wavetableZ[Z];
    moveZ(Zpos);
  }
  if (Y==__wavepointsY__)
  {
//    digitalWrite(13, HIGH);
    dY = dY - 1;
  }
  if (Y==0)
  {
//    digitalWrite(13, LOW);
    dY = dY + 1;
  }
  
  
  if (Y % 2)
  {
    digitalWrite(A14, LOW);
  }
  else
  {
    digitalWrite(A14, HIGH);
  }
}

void incrementX()
{
  //digitalWrite(13, HIGH);
  X = X + dX;              // increment to the next X

  if (dX==0)
  {
    incrementY();
    Ypos = wavetableY[Y];
    moveY(Ypos);
  }

  if (X==__wavepointsX__)
  {
    digitalWrite(reverseDirPin, HIGH);
    dX = dX - 1;
  }
  if (X==0)
  {
    digitalWrite(reverseDirPin, LOW);
    dX = dX + 1;
  }
  //digitalWrite(13, LOW);
}

void timersetup()
{
  /* First disable the timer overflow interrupt while we're configuring */
  TIMSK2 &= ~(1<<TOIE2);

  /* Tell other Arduino that we are moving up */
  digitalWrite(reverseDirPin, LOW);

  /* Sound the beeper to tell that scanning starts */
  for(i=0; i<800; i++)
  {
    digitalWrite(piezo, 1);
    delayMicroseconds(125);
    digitalWrite(piezo, 0);
    delayMicroseconds(125);
  }

  
  /* The timer/counter A has two control registers TCCR2A and TCCR2B for configuration */
  /* Configure timer2 in normal mode (pure counting, no PWM etc.) */
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));  // puts zeros in WGM20 and WGM21
  TCCR2B &= ~(1<<WGM22);                 // puts zeros in WGM22

  /* Select clock source: internal I/O clock */
  ASSR &= ~(1<<AS2);

  /* Disable Compare Match A interrupt enable (only want overflow) */
  TIMSK2 &= ~(1<<OCIE2A);

  /* Now configure the prescaler to CPU clock divided by 8
   * The register values CS22, CS21 and CS20 are 0 1 0 */
  TCCR2B |= (1<<CS21); // Set bits
  TCCR2B &= ~((1<<CS22) | (1<<CS20));             // Clear bit

  /* We need to calculate a proper value to load the timer counter. 
   * The following loads the value 131 into the Timer 2 counter register 
   * The math behind this is: 
   * (CPU frequency) / (prescaler value) = 16MHz/8 = 2 MHz := 125ns. 
   * (desired period) / 125us = 120. 
   * MAX(uint8) + 1 - 120 = 136; 
   */  

  /* Save value globally for later reload in ISR */
  //tcnt2 = 182;
  //tcnt2 = 100;

  /* Finally load end enable the timer */  
  TCNT2 = tcnt2;
  TIMSK2 |= (1<<TOIE2);
  /* get the stage into the starting position */
  //initstage();
}

ISR(TIMER2_OVF_vect)
{  
  /* Reload the timer */
  TCNT2 = tcnt2;
  /* Write to a digital pin so that we can confirm our timer */

  // move the stage to the next X
  Xpos = wavetableX[X];
  moveX(Xpos);
  incrementX();
}

void cameraReady(void)
{
  /* Interrupt issued by the camera
     True when it is ready to acquire data
     False when it is finished acquiring */
  if (digitalRead(cameraReadyPin))
  {
    // Reset the waveform counters
    X = 0;
    Y = 0;
    Z = 0;
    // Make sure the scans starts in the right direction
    dY = 1;
    dX = 1;

    // Get to the initial position
    Xpos = wavetableX[X];
    Ypos = wavetableY[Y];
    Zpos = wavetableZ[Z];
    // Reset the DAC
    moveX(Xpos);    // Go to the starting X position
    moveY(Ypos);    // Go to the starting Y position
    moveZ(Zpos);    // Go to the starting Z position
    digitalWrite(moveOnOffPin, 0);  // Tell other Arduino that scanning stopped
    digitalWrite(reverseDirPin, 1); // Tell other Arduino that a new line will start
    for(i=0; i<2000; i++)
    {
      delayMicroseconds(500);
    }
    digitalWrite(moveOnOffPin, 1);  // Tell other Arduino that scanning is started
    timersetup();
  }
  else
  {
    //timerstop();
    //resetDAC();
    stopRun();
  }
}

void setup()
{
  ioinit();
  resetDAC();
  stopRun();
  //timersetup();
  //cameraReady();
  attachInterrupt(cameraReadyInt, cameraReady, CHANGE);
}

void loop() 
{ 

} 
