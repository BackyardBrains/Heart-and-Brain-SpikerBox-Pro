/*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains Sep. 2019
  * Written by Stanislav Mircic
  * 
  * Made to work with ATmega328 (Arduino UNO)
  * Made for Heart and Brain SpikerBox Pro version V0.1
  * 
  * A0 - EEG signal from bioamplifier CH1
  * A1 - EEG signal from bioamplifier CH2
  * A2 - not used (Probably ENCODER)
  * A3 - additional analog inputs CH4 or event 4
  * A4 - additional analog input CH3 or event 5
  * A5 - monitoring of voltage on battery
  * 
  * D0  - RX
  * D1  - TX
  * D2  - Debug
  * 
  * 
  * D5  - Red LED (inverted)
  * 
  * 
  * 
  * 
  * D8  - Latch Shift Register
  * D9  - Clock Shift Register
  * D10 - 5KHz oscilator 
  * D11 - Data Shift Register
  * D12 - Blue LED (inverted)
  * D13 - Green LED (inverted)
  * ----------------------------------------------------------------------------------------------------
  */


//Clear/reset bit in register "cbi" macro
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
//Set bit in register "sbi" macro 
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define CURRENT_SHIELD_TYPE "HWT:MUSCLESS;"

#define SIZE_OF_COMMAND_BUFFER 30               //command buffer size
char commandBuffer[SIZE_OF_COMMAND_BUFFER];     //receiving command buffer

//--------------------- VU meter shift registers variable/constants 

//D8 port B 0. bit
#define SHIFT_LATCH_PIN 8                       //latch pin for shift register
#define LATCH_PIN B00000001
#define NOT_LATCH_PIN B11111110
//D9 port B 1. bit
#define SHIFT_CLOCK_PIN 9                       //clock pin for shift register
#define CLOCK_PIN B00000010
#define NOT_CLOCK_PIN B11111101
//D11 port B 3. bit
#define SHIFT_DATA_PIN 11                       //serial data pin for shift register
#define DATA_PIN B00001000
#define NOT_DATA_PIN B11110111
//D10 Port B 2.bit
#define OSCILATOR_PIN 10                        //5kHz oscilator pin

#define BLUE_LED_PIN 12                         //blue led
#define GREEN_LED_PIN 13                        //green led
#define RED_LED_PIN 5                           //red led

#define LOW_RAIL_VOLTAGE_FIRST_HIGH 840
#define LOW_RAIL_VOLTAGE_FIRST_LOW 822 //2.65V @3.3V
#define LOW_RAIL_VOLTAGE_SECOND_HIGH 700
#define LOW_RAIL_VOLTAGE_SECOND_LOW 682 //2.2V @3.3V
#define BATERY_DEATH_VOLTAGE_HIGH 580
#define BATERY_DEATH_VOLTAGE_LOW 490 //1V @2.6

#define POWER_MODE_SOLID_GREEN 0
#define POWER_MODE_SOLID_RED 1
#define POWER_MODE_BLINKING_RED 2
#define POWER_MODE_LEDS_OFF 3
byte powerMode = 0;
unsigned int blinkingLowVoltageTimer = 0;
#define BLINKING_LOW_VOLTAGE_TIMER_MAX_VALUE 500

//D13 port B 5. bit
#define GREEN_LED B00100000
#define NOT_GREEN_LED B11011111
//D12 port B 4. bit
#define BLUE_LED B00010000
#define NOT_BLUE_LED B11101111
//D5 is port D 5. bit
#define RED_LED B00100000
#define NOT_RED_LED B11011111

//registers that contain state of LEDs for all 6 VU meters
//we have 6 VU meters with 6 LEDs so we need 36 bits. That fits in 
//five 8bit registers
byte shiftRegByte;
#define MODE_PREPARE_SHIFT_REGISTERS 0          //when in this mode we calculate what leds will be ON
#define SHIFT_OUT_SHIFT_REGISTERS 1             //when in this mode we shift out bits for LEDS to shift reg. 
byte vuMeterMode = MODE_PREPARE_SHIFT_REGISTERS;//variable that holds current mode 
//index of VU meter for which we are currently preparing data
byte vuMeterIndex;                               
byte ledIndex;                                  //index of LED in VU meter
uint16_t movingThresholdSum;                    //voltage threshold for LED in VU meter
//index of register for which we are currenlt preparing data or which we currently shift out
//functionality depending on vuMeterMode
volatile byte registerIndex = 0;  
byte bitMask = 1;                               //general purpose bit mask byte variable


//---------------------- ADC sampling related variable/constants
#define MAX_NUMBER_OF_CHANNELS 6                //Maximum number of analog channels
//How many analog channels we are currently sending via serial to SpikeRecorder
byte numberOfChannels = 2;

//Note:
//Since we have to refresh all the analog channels for VU meters all the time we have to 
//measure MAX_NUMBER_OF_CHANNELS all the time. According to spec. when only one
//channel is ON in SpikeRecorder we have to sample it at 10kHz. Since Arduino can not
//sample 6 channels at 10kHz we have to sample at maximal frequency only channels that 
//we need to send to SpikeRecorder via serial. Rest of the chanels we sample at lower 
//frequency in round-robin scheme one channel each period of timer 

//index of "normal" (channels that we send over serial) analog channel that we need
//to measure next 
volatile  byte regularChannelsIndex;   
//index of "additional" (channels that we do not send over serial but we use to refresh VU meters) 
//analog channel that we need to measure next          
volatile  byte roundRobinChannelIndex;
//holds how many channels we measured since beginning of timer period
volatile  byte adcInterruptIndex;
//index of last measured channel. We need this to know where to store
//result of ADC when ADC "finish" interrupt occur 
volatile  byte lastADCIndex;

// Output Compare Registers  value = (16*10^6) / (Fs*8) - 1  set to 1999 for 1000 Hz 
// sampling, set to 3999 for 500 Hz sampling, set to 7999 for 250Hz sampling, 
// 198 for 10000 Hz Sampling. Used for main timer that defines period of measurements
int interrupt_Number = 198;

//main buffer that contains real measurements
volatile uint16_t samplingBuffer[MAX_NUMBER_OF_CHANNELS];


//variable that signals to main loop that output frame buffer is ready for sending
//0-not ready;1-ready for sending
byte outputBufferReady = 0;
//Output frame buffer that contains measured EMG data formated according to 
//SpikeRecorder serial protocol
byte outputFrameBuffer[MAX_NUMBER_OF_CHANNELS*2+64];


#define ESCAPE_SEQUENCE_LENGTH 6
byte escapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,128,255};
byte endOfescapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,129,255};
byte sizeOfextendedPackage = 0;//used when we have to send message and samples
byte sendExtendedMessage = 0;

void timer1_timer2_init()
{
    //-----------------------------
    //Setting 5kHz timer (Timer 1)
    
    //Set CTC mode with WGM
    //TCCR1A = WGM11 WGM10 ==00
    cbi(TCCR1A,WGM11);
    cbi(TCCR1A,WGM10);
    //TCCR1B = WGM13 WGM12 == 11   or 01
    cbi(TCCR1B,WGM13);
    sbi(TCCR1B, WGM12);

    //how pin toggles when timer reaches value (ON,OFF, toggle etc.)
    cbi(TCCR1A, COM1B1);            //toggle OC1A
    sbi(TCCR1A, COM1B0);            //toggle OC1A
    
    //CS12 CS11 CS10 == 001    no prescaler
    cbi(TCCR1B, CS11);
    cbi(TCCR1B, CS12);
    sbi(TCCR1B, CS10);

   OCR1A = 1600;    //when to reset
   OCR1B = 900;    //when to toggle (not working)

  //-----------------------------
  //setting sampling timer (Timer2)
  //OCIE2B OCIE2A TOIE2

   //enable interrupt for timer 2 A
   sbi(TIMSK2,OCIE2A);
   //TIMSK2 = (TIMSK2 & B11111001) | 0x06;
   //TCCR2B = (TCCR2B & B11111000) | 0x07;

   //CTC
   sbi(TCCR2A,WGM21);
   cbi(TCCR2A,WGM20);
   cbi(TCCR2B,WGM22); 

   //prescaller
   //TCCR2B   CS22 CS21 CS20  = 001 no prescaling
   sbi(TCCR2B,CS22);
   cbi(TCCR2B,CS21);
   sbi(TCCR2B,CS20);

    //1000Hz ~ 
    //101    
    //OCR2A = 124;
    //change OCRA to change sampling freq 249 for 500Hz and 124 to 1kHz
   OCR2A = 124;
   OCR2B = 2;
}


 
void setup()
{
  Serial.begin(230400);//230400);       //begin Serial comm

  Serial.setTimeout(2);

  //set pins to output for shift register
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_DATA_PIN, OUTPUT);

  pinMode(2, OUTPUT);//debug
  pinMode(3, OUTPUT);//debug
  pinMode(4, OUTPUT);//debug

  //status LEDs
  pinMode(OSCILATOR_PIN, OUTPUT);//5kHz oscilator
  pinMode(BLUE_LED_PIN, OUTPUT);//blue LED
  pinMode(GREEN_LED_PIN, OUTPUT);//green LED
  pinMode(RED_LED_PIN, OUTPUT);//red LED 
  digitalWrite(BLUE_LED_PIN, HIGH);//turn OFF (inverted)
  digitalWrite(GREEN_LED_PIN, HIGH);//turn OFF (inverted)
  digitalWrite(RED_LED_PIN, HIGH);//turn OFF (inverted)
  
  clearAllLeds();

  //stop interrupts
  cli();
 
  cbi(ADMUX,REFS0);  // Set ADC reference to AVCC
  cbi(ADMUX,ADLAR);// Left Adjust the result
  sbi(ADCSRA,ADEN);// Enable ADC
  sbi(ADCSRA,ADIE);// Enable ADC Interrupt

  //set ADC clock division to 16
  sbi(ADCSRA,ADPS2);//1
  cbi(ADCSRA,ADPS1);//0
  cbi(ADCSRA,ADPS0);//0

  
  

  timer1_timer2_init();





  
   //initialize variables for SDC sampling scheme 
   numberOfChannels = 2;
   regularChannelsIndex = 0;
   roundRobinChannelIndex = numberOfChannels;
   adcInterruptIndex = 0;
   lastADCIndex = 0;

   // Enable Global Interrupts
   sei();                   
}



//----------------------------- Main Loop ----------------------------------------------------
void loop()
{
   if(outputBufferReady == 1)//if we have new data
   {
PORTD |= B00001000;//debug
     //write data from outputFrameBuffer
     if(sendExtendedMessage)
     {
       Serial.write(outputFrameBuffer, sizeOfextendedPackage);
     }
     else
     {
       Serial.write(outputFrameBuffer, 4);
     }
     outputBufferReady = 0;




      // ------------------------------- CHECK THE POWER RAIL VOLTAGE -------------------------------
      uint16_t tempADCresult = samplingBuffer[5];//battery ADC result
  
  
      //detect voltage level and set mode (with histeresis)
      if(tempADCresult>=LOW_RAIL_VOLTAGE_FIRST_HIGH)   //Vcc > X > LOW_RAIL_VOLTAGE_FIRST_HIGH
      {
          powerMode = POWER_MODE_SOLID_GREEN;
  
      }
      else
      {
          if(tempADCresult>=LOW_RAIL_VOLTAGE_FIRST_LOW) //LOW_RAIL_VOLTAGE_FIRST_HIGH > X > LOW_RAIL_VOLTAGE_FIRST_LOW
          {
                  if(powerMode == POWER_MODE_SOLID_GREEN)
                  {
                      //do nothing it is in POWER_MODE_SOLID_GREEN
                  }
                  else
                  {
                      powerMode = POWER_MODE_SOLID_RED;
                  }
  
          }
          else
          {
              if(tempADCresult>=LOW_RAIL_VOLTAGE_SECOND_HIGH)  //LOW_RAIL_VOLTAGE_FIRST_LOW > X > LOW_RAIL_VOLTAGE_SECOND_HIGH
              {
                  powerMode = POWER_MODE_SOLID_RED;
              }
              else
              {
                  if(tempADCresult>=LOW_RAIL_VOLTAGE_SECOND_LOW)  //LOW_RAIL_VOLTAGE_SECOND_HIGH > X > LOW_RAIL_VOLTAGE_SECOND_LOW
                  {
                          if(powerMode == POWER_MODE_SOLID_RED)
                          {
                              //do nothing it is in POWER_MODE_SOLID_RED
                          }
                          else
                          {
                              powerMode = POWER_MODE_BLINKING_RED;
                          }
                  }
                  else
                  {
                      if(powerMode != POWER_MODE_LEDS_OFF)//once in POWER_MODE_LEDS_OFF do not exit untill voltage is way beyond
                      {
                          if(tempADCresult>=BATERY_DEATH_VOLTAGE_HIGH) //LOW_RAIL_VOLTAGE_SECOND_LOW > X > BATERY_DEATH_VOLTAGE_HIGH
                          {
                              powerMode = POWER_MODE_BLINKING_RED;
                          }
                          else
                          {

                              if(tempADCresult>=BATERY_DEATH_VOLTAGE_LOW)  //BATERY_DEATH_VOLTAGE_HIGH > X > BATERY_DEATH_VOLTAGE_LOW
                              {
                                  if(powerMode == POWER_MODE_BLINKING_RED)
                                  {
                                      //do nothing it is in POWER_MODE_BLINKING_RED
                                  }
                                  else
                                  {
                                      powerMode = POWER_MODE_LEDS_OFF;
                                  }
                              }
                              else                                     //BATERY_DEATH_VOLTAGE_LOW > X
                              {
                                  powerMode = POWER_MODE_LEDS_OFF;
                              }
                          }
                      }
                  }
              }
          }
      }//end of power mode if selection


      //set LEDs according to power mode
      switch(powerMode)
      {
          case POWER_MODE_SOLID_GREEN:
              PORTB &= NOT_GREEN_LED;
              PORTD |=  RED_LED;
          break;
          case POWER_MODE_SOLID_RED:
              PORTB |= GREEN_LED;
              PORTD &= NOT_RED_LED;
          break;
          case POWER_MODE_BLINKING_RED:
              if(blinkingLowVoltageTimer>0)
              {
                  blinkingLowVoltageTimer = blinkingLowVoltageTimer -1;
              }
              else
              {
                  blinkingLowVoltageTimer = BLINKING_LOW_VOLTAGE_TIMER_MAX_VALUE;
  
                  PORTD ^=  RED_LED;//blinking red
                  PORTB |= GREEN_LED;
              }
          break;
          case POWER_MODE_LEDS_OFF:
              PORTD |=  RED_LED;
              PORTB |=  GREEN_LED;
          break;
      }

     //-------------------------------- MAIN LOOP calculate what diode needs to be ON for VU meter ---------------------------------------
     //we calculate one VU meter at per period of timer since it takes too long to calculate all VU meters in one pass
     if(vuMeterMode == MODE_PREPARE_SHIFT_REGISTERS)
     {
              //!Here we should set shift reg. byto to whatever we want!
              shiftRegByte++;
              
              bitMask = B10000000;//prepare for bit shifting
              vuMeterMode = SHIFT_OUT_SHIFT_REGISTERS;
              PORTB &= NOT_LATCH_PIN;

     }
     else
     {
      //-------------------------------- MAIN LOOP shift out one bit of data --------------------------------------------------- 
        //shift out one bit of data  at the time, next time we will shift next one
        //until we shift all 8  Than we will change mode to MODE_PREPARE_SHIFT_REGISTERS
        //we do this to save time since whole main loop needs to execute in less than 100usec
        
        //shifting one bit of data to shift registers
        if(shiftRegByte & bitMask)
        {
            PORTB |=  DATA_PIN;
        }
        else
        {
            PORTB &=  NOT_DATA_PIN;
        }
         //pulse the clock for shift
         PORTB |=  CLOCK_PIN;
         PORTB &=  NOT_CLOCK_PIN;

         //move to next bit
         bitMask = bitMask>>1;

        //if we are finished with one whole register
        if(bitMask==0)
        {
            vuMeterMode = MODE_PREPARE_SHIFT_REGISTERS;
            PORTB |= LATCH_PIN;
        }
       
     }

    PORTD &= B11110111;//debug
   }//end of (outputBufferReady == 1)

   if(Serial.available()>0)
   {

      // read untill \n from the serial port:
      String inString = Serial.readStringUntil('\n');
    
      //convert string to null terminate array of chars
      inString.toCharArray(commandBuffer, SIZE_OF_COMMAND_BUFFER);
      commandBuffer[inString.length()] = 0;
      
      
      // breaks string str into a series of tokens using delimiter ";"
      // Namely split strings into commands
      char* command = strtok(commandBuffer, ";");
      while (command != 0)
      {
          // Split the command in 2 parts: name and value
          char* separator = strchr(command, ':');
          if (separator != 0)
          {
              // Actually split the string in 2: replace ':' with 0
              *separator = 0;
              --separator;
              
              if(*separator == 'b')//if we received command for impuls
              {
                sendMessage(CURRENT_SHIELD_TYPE);
              }
          }
          // Find the next command in input string
          command = strtok(0, ";");
      }

   }

}//end of main loop





//------------------------- Timer interrupt ---------------------------------

ISR(TIMER2_COMPA_vect){

  // Start ADC Conversions 
  //do this at the begining since ADC can work in 
  //paralel with this timer handler
  ADCSRA |=B01000000; 
  
PORTD |= B00000100;//debug
  //PORTB ^= B00000100;//5kHz oscilator
  //convert data to frame according to protocol
  //first bit of every byte is used to flag start of the frame
  //so first bit is set only on first byte of frame (| 0x80)
  outputFrameBuffer[0]= (samplingBuffer[0]>>7)| 0x80;
  outputFrameBuffer[1]=  samplingBuffer[0] & 0x7F;
  outputFrameBuffer[2]= (samplingBuffer[1]>>7)& 0x7F;
  outputFrameBuffer[3]=  samplingBuffer[1] & 0x7F;
  
  //signal main loop to send frame
  outputBufferReady = 1;
  
}



//--------------------------- ADC interrupt -----------------------------------------
//This is called when ADC conversion is complete.
ISR(ADC_vect)           
 {
PORTD |= B00010000;//debug
     
      samplingBuffer[lastADCIndex] = ADCL | (ADCH << 8);// store lower and higher byte of ADC

      if(adcInterruptIndex <numberOfChannels)
      {
        //we have to sample channels that are currently in use
        //channels that needs to be sampled every period of timer

         ADMUX =  B01000000 | regularChannelsIndex; //set ADC to A0 channel
         lastADCIndex = regularChannelsIndex;
         regularChannelsIndex++;
         if(regularChannelsIndex == numberOfChannels)
         {
            regularChannelsIndex = 0;
         }
      }
      else
      {
          //we have to sample channels that are not currently in use
          //we sample them at slower pace, one each timer period (round robin scheme)

           ADMUX =  B01000000 | roundRobinChannelIndex; //Select ADC Channel/ REFS0 set means AVcc is reference
           lastADCIndex = roundRobinChannelIndex;
           roundRobinChannelIndex++;
           if(roundRobinChannelIndex==MAX_NUMBER_OF_CHANNELS)
           {
             roundRobinChannelIndex = numberOfChannels;  
           }
      
      }    
    
      adcInterruptIndex++;
      if(adcInterruptIndex>numberOfChannels)
      {
          //if we sampled all channels that are in use
          //and one unused channel do not trigger ADC
          //but whait for next timer period and timer will trigger it
          adcInterruptIndex = 0;
PORTD &= B11111011;//debug
      }
      else
      {
        //start sampling of ADC for next channel lastADCIndex
        //used or unused
        if(adcInterruptIndex<6)
        {
          ADCSRA |=B01000000;    // Start ADC Conversions  
        }
        else
        {
          adcInterruptIndex = 0;
        }
      }
 PORTD &= B11101111;//debug
 
 } 




//push message to main sending buffer
//timer for sampling must be dissabled when 
//we call this function
void sendMessage(const char * message)
{

  int i;
  int head = numberOfChannels<<1;
  //send escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      outputFrameBuffer[head++] = escapeSequence[i];
     
  }

  //send message
  i = 0;
  while(message[i] != 0)
  {
      outputFrameBuffer[head++] = message[i++];
      
  }

  //send end of escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      outputFrameBuffer[head++] = endOfescapeSequence[i];
      
  }
  sizeOfextendedPackage = head;//used when we have to send message and samples
  sendExtendedMessage = 1;
   
}



// ------------------------- SHIFT registers and VU meter code ------------------------------------------


//
// Clear all LEDs (set to LOW)
//
void clearAllLeds()
{
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  //shift out 1 byte of zeros
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, 0);
  digitalWrite(SHIFT_LATCH_PIN, HIGH);
}