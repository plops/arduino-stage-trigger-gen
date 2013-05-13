/************************/
/* Define substitutions */
/************************/
#define RESET_TIMER2 TCNT2 = countStart


/* Include headers */
#include "SPI.h"

/***********************************/
/* Define constants in the program */
/***********************************/
// define pins
const byte ss = 53;            // ~SYNC signal pin number for SPI
const byte overflowInt = 5;    // Overflow interrupt 5 on pin 18
const byte overflowPin = 18;   // Overflow interrupt pin number
const byte reverseDirInt = 4;  // Change direction interrupt 4 on pin 19
const byte reverseDirPin = 19; // Change direction pin number
const byte moveOnOffInt = 2;   // Stage moving interrupt 2 on pin 21
const byte moveOnOffPin = 21;  // Stage moving interrupt pin number
const byte cameraReadyPin = 20;// Camera shutter pin
//const byte tcnt2 = 240;      // Camera trigger pulse length timer value
const byte extTrigPin = 4;     // External trigger pin

/***********************************/
/* Define variables in the program */
/***********************************/
// 34 byte long buffer storing serial data
byte serialbuffer[34];
// Serial FIFO index
byte in = 0;
// Serial transfer length (up to 32 bytes) plus 2.
// For transfer of 4 bytes, serlen should be 4+2=6.
byte serlen = 0;
// counting index for Waveform buffer transfer
unsigned int inW = 0;
// length of the waveform
unsigned int waveL = 0;
// length of the waveform minus 1
unsigned int waveL0 = 0;
// waveform storage buffer
unsigned int waveform[1024];
// general counting index
unsigned int i = 0;



// enable automatic update of DA enable
byte enDAmanual = 0;

// value of automatic update of DA enable
byte manDA;
// value of automatic update of DA enable
byte autoDA[8];
// counting index for automatic update
byte autoDAin = 0;
// number of automatic updates
byte autoDAnum = 0;
// DA enable register
byte enDAauto = 0;

// enable automatic update of shutter enable
byte autoSHen = 0;
// value of automatic update of shutter enable
byte autoSH = 0;
// shutter enable register
byte enSH = 0;

// Checksum for serial data
byte cksum = 0;
// Pixel clock counting index
unsigned int pixel = 0;
// Counting index for X position
int X = 0;
// Increasing (true) / decreasing (false) X position
int dX = 1;
// Immediate X position
unsigned int Xpos = 0;
// Flag to see if the overflow interrupt is due to direction switch
// or due to distance travelled
boolean CK = true;

/* Timer 2 parameters */
// we need to disable timer0. to store the TIMSK0 register value, define a byte
byte oldtimsk0 = 0;
// Starting count value for timer2 overflow
byte countStart = 0;
// Prescaler value for timer2
byte prescaler = 1;


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
  DDRK = B11111111;    // data direction register K
  PORTK = B00000000;   // disable the DA converter enable bus
  DDRF = B11111111;    // data direction register F
  PORTF = B00000000;   // disable the shutter converter enable bus
  
  /* Configure control pins for SPI */
  pinMode(ss, OUTPUT);
  /* Configure pin 18 as input for overflow interrupt */
  pinMode(overflowPin, INPUT);
  /* Configure pin 19 as input for reverse direction interrupt */
  // pinMode(reverseDirPin, INPUT);
  /* Configure pin 20 as input camera ready input */
  pinMode(reverseDirPin, INPUT);
  /* Configure pin 21 as input for movement control pin */
  pinMode(moveOnOffPin, INPUT);
  //pinMode(A10, OUTPUT);
  digitalWrite(ss, HIGH);
  //digitalWrite(A10, HIGH);


  /* Configure pin 4 as output: External trigger pulse */
  pinMode(extTrigPin, OUTPUT);

  pinMode(13, OUTPUT);
}

void initSPI(void)
{
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
}

void initSERIAL(void)
{
  Serial.begin(9600);
}

void initTIMER(void)
{
//  /* First disable the timer overflow interrupt while we're configuring */
//  TIMSK2 &= ~(1<<TOIE2);
//  
//  /* The timer/counter A has two control registers TCCR2A and TCCR2B for configuration */
//  /* Configure timer2 in normal mode (pure counting, no PWM etc.) */
//  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));  // puts zeros in WGM20 and WGM21
//  TCCR2B &= ~(1<<WGM22);                 // puts zeros in WGM22
//
//  /* Select clock source: internal I/O clock */
//  ASSR &= ~(1<<AS2);
//
//  /* Disable Compare Match A interrupt enable (only want overflow) */
//  TIMSK2 &= ~(1<<OCIE2A);
//
//  /* Now configure the prescaler to CPU clock divided by 8
//   * The register values CS22, CS21 and CS20 are 0 1 0 */
////  TCCR2B |= (1<<CS21); // Set bits
////  TCCR2B &= ~((1<<CS22) | (1<<CS20));             // Clear bit
//
//  /* Now configure the prescaler to CPU clock divided by 64
//   * The register values CS22, CS21 and CS20 are 1 0 0 */
//  TCCR2B |= (1<<CS22); // Set bits
//  TCCR2B &= ~((1<<CS21) | (1<<CS20));             // Clear bit
//
//  /* We need to calculate a proper value to load the timer counter. 
//   * The following loads the value 131 into the Timer 2 counter register 
//   * The math behind this is: 
//   * (CPU frequency) / (prescaler value) = 16MHz/8 = 2 MHz := 125ns. 
//   * (desired period) / 125us = 120. 
//   * MAX(uint8) + 1 - 120 = 136; 
//   */  
//
//
  /* Timer 2 is normally not incrementing, the counting starts when the clock is enabled */
  /* This is done by setting the prescaler: TCCR2B = B00000001;                          */
  //Timer2 Overflow Interrupt Enable  
  TIMSK2 = B00000001;
  // Use internal clock
  ASSR = B00000000;
  // Timer/Counter Control Register A
  TCCR2A = B00000000;
  // Timer/Counter Control Register B
  TCCR2B = B00000000;
  // Reset timer2
  RESET_TIMER2;


}

/* Overflow timer function */
ISR(TIMER2_OVF_vect)
{  
  /* Stop the timer by stopping its clock */
  //TIMSK2 &= ~(1<<TOIE2);
  TCCR2B = B00000000;
  /* reset the timer2 to the initial value */
  RESET_TIMER2;
 
  
  /* Switch off AOMs */
  //enDAauto &= B11111100;
  // enDAauto = B00000000;
  // PORTK = (~autoDA & enDAmanual) | (autoDA & enDAauto);
  PORTK = (manDA & enDAmanual);
  //PORTK=B11111100;
    digitalWrite(extTrigPin, LOW);

}

void checkSum(void)
{
  /* Checksum from https://github.com/alvieboy/arduino-oscope/blob/master/oscope.pde
     It makes bitwise XOR on the entire stack of data */
  cksum = 0;
  for (i=0;i<in;i++)
  {
    cksum^=serialbuffer[i];
  }
  Serial.write(cksum);
  /* Octave code to calculate the checksum
  A=[repmat('0', 5, 1) dec2bin(double('Dqwer'))]
  W='        ';V=repmat(false,1,8);u=0;for i = 1:8;for j=1:5; u=u+1;V(i)=xor(V(i), logical(bin2dec(A(u))));end;end;for i=1:8; W(i)=dec2bin(V(i));end;W=bin2dec(W);char(W)
  */
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

void incrementX(void)   // Interrupt 0 response
{
  /* Function to change the X position voltage to the next value */
  //
  if (CK)
  {
    //digitalWrite(13, HIGH);
    /* Raise the external trigger pin to HIGH */
    digitalWrite(extTrigPin, HIGH);

    /* Switch on AOMs */
    //enDAauto |= B00000011;
    //enDAauto = B00010000;
    // Green LASER enabled: enDAauto = B01000000;
    // Red LASER enabled:   enDAauto = B00100000;
    PORTK = (manDA & enDAmanual) | (autoDA[autoDAin]);
    autoDAin++;
    autoDAin = autoDAin % autoDAnum;
    TCCR2B = prescaler;
    //PORTK = B11111111;
    /* Load the timer */
    //TCNT2 = tcnt2;

    /* Start the timer */
    // TIMSK2 |= (1<<TOIE2);
  //  digitalWrite(13, HIGH);
  //  delayMicroseconds(100);
  //  digitalWrite(13, LOW);
    
    /* Return the external trigger pin to LOW */
//    digitalWrite(extTrigPin, LOW);
    //delayMicroseconds(100);
    /* Switch off AOMs */
    //enDAauto &= B11111100;
    //enDAauto = B00000000;
    //PORTK = (~autoDA & enDAmanual) | (autoDA & enDAauto);

  }
  
  CK = true;

  /* Increment to the next X position */
  X = X + dX;
  if (X == waveL0 | X == 0)
  {
    detachInterrupt(overflowInt);
    digitalWrite(13, LOW);
  }

  /* Move X position */
  Xpos = waveform[X];
  moveX(Xpos);


}

void revXdir(void)      // Interrupt 1 response
{
  /* Function to register change of X movement direction 
     When 0, positive direction movement, when 1 negative direction
     The other Arduino should issue 1 to pin 3 followed by zero to start from
     from the correct initial position */
  detachInterrupt(overflowInt);

  /* Make sure that clock is not due to interrupt attachment */
  CK = false;

  /* Check if this is a RISING or FALLING edge
     and change the upX variable accordingly */
  if (digitalRead(reverseDirPin))
  {
    //digitalWrite(13, HIGH);
    /* We are going down */
    dX = -1;

    // Counting down means we need to start from the maximum value
    X = waveL0;

    /* Move X position */
    Xpos = waveform[X];
    moveX(Xpos);

    /* Wait 1 ms to stabilize */
    delayMicroseconds(100);
    //X = waveL0;

    /* Activate overflow interrupt */
    attachInterrupt(overflowInt, incrementX, RISING);
    //CK=true;
    // Counting down means we need to start from maximum value
    //X = waveL0;

    /* Move X position */
    //Xpos = waveform[X];
    //moveX(Xpos);
  }
  else
  {
    //digitalWrite(13, LOW);
    /* We are going up */
    dX = 1;

    // Counting up means we need to start from zero
    X = 0;

    /* Move X position */
    Xpos = waveform[X];
    moveX(Xpos);

    /* Wait 1 ms to stabilize */
    delayMicroseconds(100);

    /* Activate overflow interrupt */
    attachInterrupt(overflowInt, incrementX, FALLING);

    // Counting up means we need to start from zero
    //X = 0;

    /* Move X position */
    //Xpos = waveform[X];
    //moveX(Xpos);
  }
}

void moveOnOff(void)     // Interrupt 2 response on pin 21
{
  /* The other Arduino needs to issue one to pin 21 to start the pixel clock generation
     Switch it to 0 to stop pixel clock */

  if (digitalRead(moveOnOffPin))
  {
    //enDAauto |= B00001100;
    //PORTK = (~autoDA & enDAmanual) | (autoDA & enDAauto);
    //attachInterrupt(overflowInt, incrementX, RISING);
    oldtimsk0 = TIMSK0;
    TIMSK0 &= B11111110;
    attachInterrupt(reverseDirInt, revXdir, CHANGE);
  }
  else
  {
    //enDAauto = B00000000;
    //PORTK = (~autoDA & enDAmanual) | (autoDA & enDAauto);
    detachInterrupt(overflowInt);
    detachInterrupt(reverseDirInt);
    TIMSK0 = oldtimsk0;
  }

  //if (digitalRead(moveOnOffPin))
  //{
    //attachInterrupt(overflowInt, incrementX, RISING);
    //enDAauto |= B11111111;
    //PORTK = B11111111; //(~autoDA & enDAmanual) | (autoDA & enDAauto);
  //}
  //else
  //{
    //detachInterrupt(overflowInt);
    //enDAauto = 0;
    //PORTK = (~autoDA & enDAmanual) | (autoDA & enDAauto);
  //}
}

void setup()
{
  ioinit();
  initSPI();
  initSERIAL();
  initTIMER();
  attachInterrupt(moveOnOffInt, moveOnOff, CHANGE);
}

void loop()
{
/*  delayMicroseconds(90);
  PORTK = (~autoDA & enDAmanual) | (autoDA & enDAauto);
  delayMicroseconds(10);
  PORTK = (~autoDA & B00000000) | (autoDA & enDAauto);
*/
  // if we get a valid byte
  if (Serial.available() > 0)
  {
    // get incoming byte:
    serialbuffer[in] = Serial.read();
    in++;
    /* Read the very first byte of the serial buffer
       It contains the length of the data transfer "serlen" */
    if (in == 1)
    {
      serlen = serialbuffer[0];
    }
    if (in == serlen)
    {
      switch (serialbuffer[1])      // the second byte contains the command
      {
        /* if the command is D (68 in ASCII) for DAC */
        case 68:
          //Serial.print('A', BYTE);
          digitalWrite(ss, LOW);      // assert ~SYNC pin low to make DA converter listen
          for(i = 2; i < serlen; i++)  // loop through last 4 bytes of the array
          {
            SPI.transfer(serialbuffer[i]);// send over SPI byte by byte to the DA converter
          }
          digitalWrite(ss, HIGH);     // assert ~SYNC pin high to finish transfer
          checkSum();
          /* Checksum from https://github.com/alvieboy/arduino-oscope/blob/master/oscope.pde
             It makes bitwise XOR on the entire stack of data
          for (i=0;i<rsize;i++) {
            cksum^=buf[i];
            Serial.write(buf[i]);
          }
          */
          //Serial.print('A', BYTE);    // send a capital A to acknowledge the process
          in = 0;
          serlen = 0;
        break;

        /* if the command is E (69 in ASCII) for Enable DAC */
        case 69:
          manDA = serialbuffer[2];      // ones tell, which bits are manually controlled
          enDAmanual = serialbuffer[3]; // ones tell, which bits should be enabled
          autoDAnum = serialbuffer[4];  // how many automatic states are there
          for(i = 5; i < serlen; i++)   // loop through the remaining bytes of the array
          {
            autoDA[i-5] = serialbuffer[i]; // order, in which hardware should be switched on
          }
          autoDAin = 0;
          PORTK = (manDA & enDAmanual);
          //PORTK = enDA;
          //Serial.print('A', BYTE);    // send a capital A to acknowledge the process
          //Serial.write(enDA);         // send a capital A to acknowledge the process
          checkSum();
          in = 0;
          serlen = 0;
        break;

        /* if the command is L (76 in ASCII) for Length of waveform */
        case 76:
          waveL = word(serialbuffer[2], serialbuffer[3]);
          waveL0 = waveL-1;
                                      // save the length of waveform that will be sent
          inW = 0;                    // reset the waveform index to 0
          checkSum();
          in = 0;
          serlen = 0;
        break;

        /* if the command is S (83 in ASCII) for Enable Shutter */
        case 83:
          autoSH = serialbuffer[2];
          enSH = serialbuffer[3];
          PORTF = (~autoSHen & enSH) | (autoSHen & autoSH);
          checkSum();
          in = 0;
          serlen = 0;
        break;

        /* if the command is W (87 in ASCII) for waveform */
        case 87:
          for(i = 2; i < serlen; i=i+2)  // loop through the packet from byte 2 to the end in steps of two
          {
            // Combine the MSB and LSB byte into a 16-bit word
            waveform[inW] = word(serialbuffer[i], serialbuffer[i+1]);

            inW++;                    // increment the counting index for waveform
            //if (inW == waveL)         // if full waveform is sent
            //{
            //  Serial.print('K', BYTE);
            //}
          }
          checkSum();
          if (inW == waveL)
          {
            /* Calculate 16-bit checksum of the waveform */
            unsigned int cksum16 = 0;
            for (i=0;i<waveL;i++)
            {
              cksum16^=waveform[i];
            }
            Serial.write(highByte(cksum16));
            Serial.write(lowByte(cksum16));
//            digitalWrite(13, HIGH);
//            for (i=0;i<waveL;i++)
//            {
//              Xpos = waveform[i];
//              moveX(Xpos);
//              delayMicroseconds(100);
//            }
//            digitalWrite(13, LOW);
          }
          in = 0;
          serlen = 0;
        break;
        
        /* if the command is T (84 in ASCII) for Timer2 setup */
        case 84:
          prescaler = serialbuffer[2];
          countStart = serialbuffer[3];

          checkSum();
          in = 0;
          serlen = 0;
        break;
      }
    }
  }
}

