/*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains Dec. 2019
  * Written by Stanislav Mircic
  * P300 experiment code with all other functionality 
  * of normal Heart and Brain SpikerBox Pro
  * 
  * Made to work with ATmega328 (Arduino UNO)
  * Made for Heart and Brain SpikerBox Pro version V0.4
  * 
  * A0 - EEG signal from bioamplifier CH1
  * A1 - EEG signal from bioamplifier CH2
  * A2 - ENCODER
  * A3 - additional analog inputs CH4 or event 4
  * A4 - additional analog input CH3 or event 5
  * A5 - monitoring of voltage on battery
  * 
  * D0  - RX
  * D1  - TX
  * D2  - event 1 
  * D3  - event 2
  * D4  - event 3
  * D5  - Red LED (inverted)
  * D6  - Speaker
  * D7  - Button
  * D8  - Green LED for experiment
  * D9  - Red LED for experiment
  * D10 - 5KHz oscilator 
  * D11 - 
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

#define CURRENT_SHIELD_TYPE "HWT:HBSBPRO;"


#define TONE_PERIOD_MS 2000                 //max 6000
#define TONE_DURATION_MS 300                //max BEEP_PERIOD_MS
#define CHANCE_OF_ODD_TONE_PERCENT 10       //max 100
#define FREQUENCY_OF_NORMAL_TONE_HZ 300     //Frequency of normal tone
#define FREQUENCY_OF_ODD_TONE_HZ 500        //Frequency of odd tone 


unsigned int chancesOfOddToneInMaxRND = 6553;
unsigned int periodOfNormalWave = 4;
unsigned int periodOfOddWave = 2;
unsigned int randNumber;

unsigned int counterForTonePeriod = 1;
unsigned int counterForToneDuration = 0;
unsigned int currentPeriodOfToneWave = 0;
unsigned int counterForWavePeriod = 0;


//operation modes
#define OPERATION_MODE_DEFAULT 0
#define OPERATION_MODE_BNC 1
#define OPERATION_MODE_FIVE_DIGITAL 2
#define OPERATION_MODE_HAMMER 3
#define OPERATION_MODE_MORE_ANALOG 4
#define OPERATION_MODE_JOYSTICK 5



#define SIZE_OF_COMMAND_BUFFER 30               //command buffer size
char commandBuffer[SIZE_OF_COMMAND_BUFFER];     //receiving command buffer

//--------------------- VU meter shift registers variable/constants 



#define CH_1_BUFFER_INDEX    0
#define CH_2_BUFFER_INDEX    1
#define CH_3_BUFFER_INDEX    4
#define CH_4_BUFFER_INDEX    3
#define ENCODER_BUFFER_INDEX 2
#define BATTERY_BUFFER_INDEX 5

#define EVENT_1_PIN 2
#define EVENT_2_PIN 3
#define EVENT_3_PIN 4
//A3 and A4 are used as a digital inputs also
#define EVENT_4_PIN 17
#define EVENT_5_PIN 18

//these Joystick IO pins are on PORTD
#define JOYSTICK_CL B00000100
#define JOYSTICK_TX B00001000
#define JOYSTICK_RX B00010000

//speaker pin 6 on port D
#define SPEAKER_BIT B01000000
#define NOT_SPEAKER_BIT B10111111
#define SPEAKER_PIN 6


//D8 port B 0. bit
#define RED_EXP_PIN 8                       //green led for experiment pin
#define RED_EXP_BIT B00000001
#define NOT_RED_EXP_BIT B11111110

//D9 port B 1. bit
#define GREEN_EXP_PIN 9                       //red led for experiment pin
#define GREEN_EXP_BIT B00000010
#define NOT_GREEN_EXP_BIT B11111101

//D11 port B 3. bit
#define BUTTON_PIN 7                           //button 
#define BUTTON_BIT B10000000
#define NOT_BUTTON_BIT B01111111
byte buttonPressed = 0;
volatile uint16_t counterButtonPressed = 0;
byte P300Active = 0;
#define LONG_PRESS_IN_100uS 1200
#define SHORT_PRESS_IN_100uS 1100
#define EXPERIMENT_MODE_SOUND 1
#define EXPERIMENT_MODE_LIGHT 2
byte modeOfP300Experiment = EXPERIMENT_MODE_SOUND;


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

uint8_t numberOfEventsUntillOdd = 0;

//---------------------- ADC sampling related variable/constants
#define MAX_NUMBER_OF_CHANNELS 6                //Maximum number of analog channels
//How many analog channels we are currently sending via serial to SpikeRecorder


//Note:
//We have to sample at maximal frequency only channels that 
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



//main buffer that contains real measurements
volatile uint16_t samplingBuffer[MAX_NUMBER_OF_CHANNELS];


//variable that signals to main loop that output frame buffer is ready for sending
//0-not ready;1-ready for sending
byte outputBufferReady = 0;
//Output frame buffer that contains measured EMG data formated according to 
//SpikeRecorder serial protocol
byte headout = 0;
byte tailout = 0;
byte outputFrameBuffer[256];


#define ESCAPE_SEQUENCE_LENGTH 6
byte escapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,128,255};
byte endOfescapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,129,255};

byte numberOfChannels = 2;
byte numberOfChannelsToSend = 2;


#define SAMPLE_RATE_500  249
#define SAMPLE_RATE_1000 124

#define DEBOUNCE_TIME 200
//debouncer variables
//used for events in normal mode and reaction timer
unsigned int debounceTimer1 = 0;
unsigned int debounceTimer2 = 0;
unsigned int debounceTimer3 = 0;
unsigned int debounceTimer4 = 0;
unsigned int debounceTimer5 = 0;
unsigned int eventEnabled1 = 1;
unsigned int eventEnabled2 = 1;
unsigned int eventEnabled3 = 1;
unsigned int eventEnabled4 = 1;
unsigned int eventEnabled5 = 1;


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
   OCR2A = SAMPLE_RATE_1000;
   OCR2B = 2;
}








//3.3V/1023 = 0.00322580645 V for one AD unit
//How many AD units is each threshold
//0.499V //0.75
#define EXP_BOARD_VOLTAGE_LEVEL_1 155
//0.999V //1.51
#define EXP_BOARD_VOLTAGE_LEVEL_2 310
//1.499V //2.27
#define EXP_BOARD_VOLTAGE_LEVEL_3 465
//1.999V //3.03
#define EXP_BOARD_VOLTAGE_LEVEL_4 620
//2.49V //3.78
#define EXP_BOARD_VOLTAGE_LEVEL_5 775
//2.99V //4.54
#define EXP_BOARD_VOLTAGE_LEVEL_6 930

uint8_t operationMode =0;

int currentEncoderVoltage = 0;
int lastEncoderVoltage = 0;

uint8_t waitingForEncoder = 0;
uint16_t debounceEncoderTimer = 0;
uint16_t blinkingBoardDetectionTimer = 0;
#define ENCODER_DEBUNCE_TIME 1500
#define BLINKING_ENCODER_DEBUNCE_TIME 300

int smoothDataRate = 20;
byte emptyBuffer[256];


uint8_t TX_joystick_buffer = 0;
uint8_t RX_joystick_buffer = 0;
uint8_t last_joystick_state = 0;
uint8_t new_joystick_state = 0;
#define BITS_BETWEEN_TWO_SERIAL 13
#define SAMPLES_FOR_ONE_BIT 1
#define HALF_SAMPLES_FOR_ONE_BIT 0
uint8_t bits_counter = BITS_BETWEEN_TWO_SERIAL;
uint8_t one_bit_counter = SAMPLES_FOR_ONE_BIT;
uint8_t lastReceivedButtons = 0;
uint8_t sendJoystickMessage = 0;
//uint16_t ledCounters[8];
uint8_t hostJoystickState = 0;
uint8_t joystickMessage[8];
float tempCalculationNumber;


//
// Create a random integer from 0 - 65535
//
unsigned int rng() {
  static unsigned int y = 0;
  y += micros(); // seeded with changing number
  y ^= y << 2; y ^= y >> 7; y ^= y << 7;
  return (y);
} 

void setup()
{
  Serial.begin(230400);      //begin Serial comm
  Serial.setTimeout(2);
  for (int g= 0;g<256;g++)
  {
    emptyBuffer[g] = 0;  
  }

  tempCalculationNumber = ((float)CHANCE_OF_ODD_TONE_PERCENT)/100.0;
  chancesOfOddToneInMaxRND = tempCalculationNumber*65535.0;

  //set pins for LEDs
  pinMode(GREEN_EXP_PIN, OUTPUT);
  pinMode(RED_EXP_PIN, OUTPUT);
  digitalWrite(GREEN_EXP_PIN, LOW);//turn OFF 
  digitalWrite(RED_EXP_PIN, LOW);//turn OFF 
  
  pinMode(BUTTON_PIN, INPUT);

  pinMode(EVENT_1_PIN, INPUT);
  pinMode(EVENT_2_PIN, INPUT);
  pinMode(EVENT_3_PIN, INPUT);

  
  joystickMessage[0]= 'J';
  joystickMessage[1]= 'O';
  joystickMessage[2]= 'Y';
  joystickMessage[3]= ':';
  joystickMessage[4]= 240;//11110000 - since we want to avoid sending zero because issue with zero
  joystickMessage[5]= 240;//11110000 - terminated strings we will divide one byte to two bytes and use just 4 LSB
  joystickMessage[6]= ';';
  joystickMessage[7]= 0;


  pinMode(SPEAKER_PIN,OUTPUT);
  
  //status LEDs
  pinMode(OSCILATOR_PIN, OUTPUT);//5kHz oscilator
  pinMode(BLUE_LED_PIN, OUTPUT);//blue LED
  pinMode(GREEN_LED_PIN, OUTPUT);//green LED
  pinMode(RED_LED_PIN, OUTPUT);//red LED 
  digitalWrite(BLUE_LED_PIN, HIGH);//turn OFF (inverted)
  digitalWrite(GREEN_LED_PIN, HIGH);//turn OFF (inverted)
  digitalWrite(RED_LED_PIN, HIGH);//turn OFF (inverted)
  

  randNumber = rng();
  numberOfEventsUntillOdd = 3+(randNumber>>12);
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


//
// Setup operation mode inputs outputs and state
//
void setupOperationMode(void)
{
    cli();
    //for now (untill joystick) all are inputs
    pinMode(EVENT_1_PIN, INPUT);
    pinMode(EVENT_2_PIN, INPUT);
    pinMode(EVENT_3_PIN, INPUT);
    switch(operationMode)
    {
        case OPERATION_MODE_BNC:
            OCR2A = SAMPLE_RATE_1000;
            numberOfChannels = 2;
        break;
        case OPERATION_MODE_FIVE_DIGITAL:
            OCR2A = SAMPLE_RATE_1000;
            numberOfChannels = 2;
            break;
        case OPERATION_MODE_JOYSTICK:
            OCR2A = SAMPLE_RATE_1000;
            numberOfChannels = 3;
            pinMode(EVENT_1_PIN, OUTPUT);//clock for joystick
            pinMode(EVENT_2_PIN, OUTPUT);//TX for joystick
            pinMode(EVENT_3_PIN, INPUT);//RX for joystick
            break;
        case OPERATION_MODE_HAMMER:
            OCR2A = SAMPLE_RATE_1000;
            numberOfChannels = 3;
        break;
        case OPERATION_MODE_MORE_ANALOG:
            OCR2A = SAMPLE_RATE_1000;
            numberOfChannels = 4;
        break;
        case OPERATION_MODE_DEFAULT:
            OCR2A = SAMPLE_RATE_1000;
            numberOfChannels = 2;
        break;
        default:
            OCR2A = SAMPLE_RATE_1000;
            numberOfChannels = 2;
        break;
    }
    regularChannelsIndex = 0;
    numberOfChannelsToSend = numberOfChannels;
    roundRobinChannelIndex = numberOfChannels;
    adcInterruptIndex = 0;
    lastADCIndex = 0;
    
    sei();
}





//----------------------------- Main Loop ---------------------------------------------------------
void loop()
{
   if(outputBufferReady == 1)//if we have new data
   {
      //------------------------- SEND DATA -------------------------------------------------------
      //write data from outputFrameBuffer
      int sent=0;
      while(headout!=tailout)
      {
        sent++;
        //Serial.write(outputFrameBuffer[tailout]);
        tailout++;
      }
      if(sent>0)
      {
        int diff = smoothDataRate-sent;
        if(diff>0)
        {
              //Serial.write(emptyBuffer,diff);
        }  
      }
      outputBufferReady = 0;

      //------------------------------  GENERATE TONE FOR P300 --------------------------------------

      //------------------------------ GENERATE SOUND ------------------------------

      if(P300Active)
      {
            
            counterForTonePeriod--;
            if(counterForTonePeriod==0)
            {
                numberOfEventsUntillOdd--;
                counterForTonePeriod = TONE_PERIOD_MS;
                if(numberOfEventsUntillOdd==0)
                {
                      sendMessage("EVNT:2;");
                      randNumber = rng();
                      numberOfEventsUntillOdd = 3+(randNumber>>12);

                      currentPeriodOfToneWave = periodOfOddWave;
                      if(modeOfP300Experiment == EXPERIMENT_MODE_SOUND)
                      {
                            PORTD ^= SPEAKER_BIT;
                            PORTB &= NOT_GREEN_EXP_BIT;
                            PORTB &= NOT_RED_EXP_BIT;
                      }
                      else
                      {
                          PORTB |= RED_EXP_BIT;
                      }
                }
                else
                {

                      sendMessage("EVNT:1;");
                      currentPeriodOfToneWave = periodOfNormalWave;
                      if(modeOfP300Experiment == EXPERIMENT_MODE_SOUND)
                      {
                            PORTD ^= SPEAKER_BIT;
                            PORTB &= NOT_GREEN_EXP_BIT;
                            PORTB &= NOT_RED_EXP_BIT;
                      }
                      else
                      {
                          PORTB |= GREEN_EXP_BIT;
                      }
                }
                
               
                counterForWavePeriod = currentPeriodOfToneWave;
                counterForToneDuration = TONE_DURATION_MS;
                
            }
      
            if(counterForToneDuration>0)
            {
              counterForToneDuration--;
              counterForWavePeriod--;
              if(counterForWavePeriod ==0)
              {
                //flip tone output
                if(modeOfP300Experiment == EXPERIMENT_MODE_SOUND)
                {
                    PORTD ^= SPEAKER_BIT;
                }
                counterForWavePeriod = currentPeriodOfToneWave;
              }
            }
            else
            {
              //Turn off tone generator  
              PORTD &= NOT_SPEAKER_BIT;
              PORTB &= NOT_GREEN_EXP_BIT;
              PORTB &= NOT_RED_EXP_BIT;
            }
      }
      else
      {
          PORTD &= NOT_SPEAKER_BIT;
          PORTB &= NOT_GREEN_EXP_BIT;
          PORTB &= NOT_RED_EXP_BIT;
      }
      



      //-------------------------------- BUTON FUNCTIONALITY --------------------------------------


      if(PIND & BUTTON_BIT)
      {
        
          if(buttonPressed)
          {
              if(counterButtonPressed>0)
              {
                
                    counterButtonPressed = counterButtonPressed-1;

              }
              if(counterButtonPressed==1)
              {
                 
                    if(modeOfP300Experiment==EXPERIMENT_MODE_SOUND)
                    {
                        
                        modeOfP300Experiment = EXPERIMENT_MODE_LIGHT;
                    }
                    else
                    {
                        modeOfP300Experiment = EXPERIMENT_MODE_SOUND;
                    }
              }
            
          }
          else
          {
            
            counterButtonPressed = LONG_PRESS_IN_100uS;
            buttonPressed = 1;
            
          }
      }
      else
      {
       
          if(buttonPressed)
          {
            
            if(counterButtonPressed<SHORT_PRESS_IN_100uS)
            {
                
              
                buttonPressed = 0;
                if(counterButtonPressed>0)
                {
                      if(P300Active)
                      {
                        
                          P300Active = 0;
                      }
                      else
                      {
                        
                          P300Active = 1;
                      }
                }
            }
          }
        
      }

      // ------------------------------- CHECK THE POWER RAIL VOLTAGE -------------------------------
      uint16_t tempADCresult = samplingBuffer[BATTERY_BUFFER_INDEX];//battery ADC result
  
  
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






    // ------------------- BOARD DETECTION -----------------------------
    currentEncoderVoltage = samplingBuffer[ENCODER_BUFFER_INDEX];



    if((currentEncoderVoltage - lastEncoderVoltage)>100 || (lastEncoderVoltage - currentEncoderVoltage)>100)
    {
          debounceEncoderTimer = ENCODER_DEBUNCE_TIME;
          waitingForEncoder = 1;
    }

    if(waitingForEncoder)
    {

        debounceEncoderTimer--;
        if(debounceEncoderTimer == 0)
        {
              //if time has expired
              //turn OFF blue LED
              //continue with Red and Green power leds status
              PORTB |= BLUE_LED;
              waitingForEncoder = 0;
              

              if(currentEncoderVoltage < EXP_BOARD_VOLTAGE_LEVEL_1 )
              {
                  //default
                  if(operationMode != OPERATION_MODE_DEFAULT)
                  {
                      operationMode = OPERATION_MODE_DEFAULT;
 
                      setupOperationMode();
                      sendMessage("BRD:0;");
                  }
      
              }
              else if((currentEncoderVoltage >= EXP_BOARD_VOLTAGE_LEVEL_1) && (currentEncoderVoltage < EXP_BOARD_VOLTAGE_LEVEL_2))
              {
                  //first board BNC
                  if(operationMode != OPERATION_MODE_MORE_ANALOG)
                  {
                      operationMode = OPERATION_MODE_MORE_ANALOG;
                      
                      setupOperationMode();
                      sendMessage("BRD:1;");
                  }
      
              }
              else if((currentEncoderVoltage >= EXP_BOARD_VOLTAGE_LEVEL_2) && (currentEncoderVoltage < EXP_BOARD_VOLTAGE_LEVEL_3))
              {
                  //second board - Reaction timer
      
                  if(operationMode != OPERATION_MODE_BNC)
                  {
                      operationMode = OPERATION_MODE_BNC;
                      
                      setupOperationMode();
                      sendMessage("BRD:2;");
      
                  }
      
              }
              else if((currentEncoderVoltage >= EXP_BOARD_VOLTAGE_LEVEL_3) && (currentEncoderVoltage < EXP_BOARD_VOLTAGE_LEVEL_4))
              {
                  //forth board
                  if(operationMode != OPERATION_MODE_HAMMER)
                  {
                      operationMode = OPERATION_MODE_HAMMER;
                      
                      setupOperationMode();
                      sendMessage("BRD:4;");
                  }
              }
              else if((currentEncoderVoltage >= EXP_BOARD_VOLTAGE_LEVEL_4) && (currentEncoderVoltage < EXP_BOARD_VOLTAGE_LEVEL_5))
              {
                 //fifth board
      
                  if(operationMode != OPERATION_MODE_JOYSTICK)
                  {
                      operationMode = OPERATION_MODE_JOYSTICK;
                      
                      setupOperationMode();
                      sendMessage("BRD:5;");
      
                  }
              }
              else if((currentEncoderVoltage >= EXP_BOARD_VOLTAGE_LEVEL_5) && (currentEncoderVoltage < EXP_BOARD_VOLTAGE_LEVEL_6))
              {
                if(operationMode != OPERATION_MODE_DEFAULT)
                {
                    //sixth board
                    
                    operationMode = OPERATION_MODE_DEFAULT;
                    setupOperationMode();
                    sendMessage("BRD:0;");
                }
              }
              else if((currentEncoderVoltage >= EXP_BOARD_VOLTAGE_LEVEL_6))
              {
                if(operationMode != OPERATION_MODE_DEFAULT)
                {
                    //sixth board
                    
                    operationMode = OPERATION_MODE_DEFAULT;
                    setupOperationMode();
                    sendMessage("BRD:0;");
                }
              }

              
        }
        else
        {
              //blinking LED during 1.5sec detection/debounce time
              if(blinkingBoardDetectionTimer==0)
              {
                    blinkingBoardDetectionTimer = BLINKING_ENCODER_DEBUNCE_TIME;
                    //turn off Red and Green and toggle blue
                    PORTB ^= BLUE_LED;
                    PORTD |= RED_LED;
                    PORTB |= GREEN_LED;
              }
              else
              {
                    PORTD |= RED_LED;
                    PORTB |= GREEN_LED;
              }
              blinkingBoardDetectionTimer--;
        }
    }//if (waitingForEncoder) end
    lastEncoderVoltage = currentEncoderVoltage;







    //===================== Power LEDS ==============================

      if(waitingForEncoder==0)
      {

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
      }


      //--------------------- BOARD EXECUTION -------------------------------

        
        switch(operationMode)
        {
            case OPERATION_MODE_JOYSTICK:

                  if(one_bit_counter==1)
                  {
                    one_bit_counter = 0;

                    bits_counter--;
                    if(bits_counter<8)
                    {
        
                        if((TX_joystick_buffer>>bits_counter) & 1)
                        {
                            PORTD |= JOYSTICK_TX;
                        }
                        else
                        {
                            PORTD &= ~JOYSTICK_TX;
                        }
                       PORTD |= JOYSTICK_CL;
                    }
                  }
                  else
                  {
                      one_bit_counter =1;
                      if(bits_counter<8)
                      {
                          RX_joystick_buffer = RX_joystick_buffer<<1;
                          if(digitalRead(EVENT_3_PIN)==HIGH)
                          {
                              RX_joystick_buffer++; 
                          }
                      }
                      PORTD &= ~(JOYSTICK_CL);


                      //hostJoystickState = 171;
                      if(bits_counter==0)
                      {
                          bits_counter = BITS_BETWEEN_TWO_SERIAL;
                          TX_joystick_buffer = RX_joystick_buffer | hostJoystickState;
                          if(RX_joystick_buffer != lastReceivedButtons)
                          {
                              joystickMessage[4] = 0xF0;
                              joystickMessage[4] |= RX_joystick_buffer & 0x0F;
                              joystickMessage[5] = 0xF0;
                              joystickMessage[5] |= (RX_joystick_buffer>>4) & 0x0F;
                              sendJoystickMessage = 1;
                          }
                          lastReceivedButtons = RX_joystick_buffer;
                      }   
                  }
                break;
            case OPERATION_MODE_DEFAULT:
            case OPERATION_MODE_FIVE_DIGITAL:

                //two additional digital inputs



                //================= EVENT 5 code ======================

                if(debounceTimer5>0)
                {
                    debounceTimer5--;
                }
                else
                {
                    if(eventEnabled5>0)
                    {
                            if(digitalRead(EVENT_5_PIN)==HIGH)
                            {
                                    eventEnabled5 = 0;
                                    debounceTimer5 = DEBOUNCE_TIME;
                                    sendMessage("EVNT:5;");
                                    
                            }
                    }
                    else
                    {
                        if(digitalRead(EVENT_5_PIN)==LOW)
                        {
                            eventEnabled5 = 1;
                        }

                    }
                }


            case OPERATION_MODE_HAMMER:

                //================= EVENT 4 code ======================
                if(debounceTimer4>0)
                {
                    debounceTimer4--;
                }
                else
                {
                    if(eventEnabled4>0)
                    {
                            if(digitalRead(EVENT_4_PIN)==HIGH)
                            {
                                    eventEnabled4 = 0;
                                    debounceTimer4 = DEBOUNCE_TIME;
                                    sendMessage("EVNT:4;");
                                    
                            }
                    }
                    else
                    {
                        if(digitalRead(EVENT_4_PIN)==LOW)
                        {
                            eventEnabled4 = 1;
                        }

                    }
                }


            case OPERATION_MODE_BNC:
            case OPERATION_MODE_MORE_ANALOG:

                //============== event 1 =================

                    if(debounceTimer1>0)
                    {
                        debounceTimer1--;
                    }
                    else
                    {
                        if(eventEnabled1>0)
                        {
                                if(digitalRead(EVENT_1_PIN)==HIGH)
                                {
                                        eventEnabled1 = 0;
                                        debounceTimer1 = DEBOUNCE_TIME;
                                        sendMessage("EVNT:1;");
                                        
                                }
                        }
                        else
                        {
                            if(digitalRead(EVENT_1_PIN)==LOW)
                            {
                                eventEnabled1 = 1;
                            }
                        }
                    }

                    //================= EVENT 2 code ======================

                    if(debounceTimer2>0)
                    {
                        debounceTimer2--;
                    }
                    else
                    {
                        if(eventEnabled2>0)
                        {
                                if(digitalRead(EVENT_2_PIN)==HIGH)
                                {
                                        eventEnabled2 = 0;
                                        debounceTimer2 = DEBOUNCE_TIME;
                                        sendMessage("EVNT:2;");
                                        
                                }
                        }
                        else
                        {
                            if(digitalRead(EVENT_2_PIN)==LOW)
                            {
                                eventEnabled2 = 1;
                            }

                        }
                    }

                    //================= EVENT 3 code ======================

                    if(debounceTimer3>0)
                    {
                        debounceTimer3--;
                    }
                    else
                    {
                        if(eventEnabled3>0)
                        {
                                if(digitalRead(EVENT_3_PIN)==HIGH)
                                {
                                        eventEnabled3 = 0;
                                        debounceTimer3 = DEBOUNCE_TIME;
                                        sendMessage("EVNT:3;");
                                        
                                }
                        }
                        else
                        {
                            if(digitalRead(EVENT_3_PIN)==LOW)
                            {
                                eventEnabled3 = 1;
                            }

                        }
                    }
            default:
                //do nothing, that is for joystick
                break;
        }

        if(sendJoystickMessage==1)
        {
          sendJoystickMessage =0;
          sendMessage(joystickMessage);
        }


   }//end of (outputBufferReady == 1)


   //------------------- SERIAL RECEIVE -----------------------------------------
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
              if(*separator == 'c')//if we received command for number of channels
              {
                //separator = separator+2;
                //numberOfChannelsToSend = (byte)atoi(separator);//read number of channels
              }
              if(*separator == 'b')//if we received command for impuls
              {
                sendMessage(CURRENT_SHIELD_TYPE);
              }
              //last char from ledon
              if (*separator == 'n')
              {
                 //jump one after "ledon:"
                 separator = separator+2;
                 int indexOfLedToLightUp = *separator-48;//get index of led from ASCII to int
                 hostJoystickState |= 0x1<<indexOfLedToLightUp;

              }
              // last char from ledoff
              if (*separator == 'f')
              {
                separator = separator+2;
                int indexOfLedToLightUp = *separator-48;//get index of led from ASCII to int
                hostJoystickState &= ~(0x1<<indexOfLedToLightUp);
              }
              if(*separator == 'd')//last char from "board"
              {
                
                   switch(operationMode)
                    {
                        case OPERATION_MODE_BNC:
                            sendMessage("BRD:1;");
                        break;
                        case OPERATION_MODE_FIVE_DIGITAL:
                            sendMessage("BRD:3;");
                        break;
                        case OPERATION_MODE_HAMMER:
                            sendMessage("BRD:4;");
                        break;
                        case OPERATION_MODE_JOYSTICK:
                            sendMessage("BRD:5;");
                        break;
                        case OPERATION_MODE_MORE_ANALOG:
                            sendMessage("BRD:1;");
                        break;
                        case OPERATION_MODE_DEFAULT:
                            sendMessage("BRD:0;");
                        break;
                    }
               }
          }
          // Find the next command in input string
          command = strtok(0, ";");
      }
   }//end of serial receive


}//end of main loop





//------------------------- Timer interrupt ---------------------------------

ISR(TIMER2_COMPA_vect){
  
  // Start ADC Conversions 
  //do this at the begining since ADC can work in 
  //paralel with this timer handler
  ADCSRA |=B01000000; 

  //PORTB ^= B00000100;//5kHz oscilator
  //convert data to frame according to protocol
  //first bit of every byte is used to flag start of the frame
  //so first bit is set only on first byte of frame (| 0x80)
  outputFrameBuffer[headout++]= (samplingBuffer[CH_1_BUFFER_INDEX]>>7)| 0x80;
  outputFrameBuffer[headout++]=  samplingBuffer[CH_1_BUFFER_INDEX] & 0x7F;
  outputFrameBuffer[headout++]= (samplingBuffer[CH_2_BUFFER_INDEX]>>7)& 0x7F;
  outputFrameBuffer[headout++]=  samplingBuffer[CH_2_BUFFER_INDEX] & 0x7F;
  if(numberOfChannels>2)
  {
    outputFrameBuffer[headout++]= (samplingBuffer[CH_3_BUFFER_INDEX]>>7)& 0x7F;
    outputFrameBuffer[headout++]=  samplingBuffer[CH_3_BUFFER_INDEX] & 0x7F;
  }
  if(numberOfChannels>3)
  {
    outputFrameBuffer[headout++]= (samplingBuffer[CH_4_BUFFER_INDEX]>>7)& 0x7F;
    outputFrameBuffer[headout++]=  samplingBuffer[CH_4_BUFFER_INDEX] & 0x7F;
  }
  
  //signal main loop to send frame
  outputBufferReady = 1;
  
}



//--------------------------- ADC interrupt -----------------------------------------
//This is called when ADC conversion is complete.
ISR(ADC_vect)           
 {
      samplingBuffer[lastADCIndex] = ADCL | (ADCH << 8);// store lower and higher byte of ADC
      lastADCIndex++;
      if(lastADCIndex==6)
      {
          lastADCIndex =0;
           ADMUX =  B01000000;
      }
      else
      {
        ADMUX =  B01000000 | lastADCIndex;
        ADCSRA |=B01000000;    // Start ADC Conversions 
      }
 } 




//push message to main sending buffer
//timer for sampling must be dissabled when 
//we call this function
void sendMessage(const char * message)
{

  int i;

  //send escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      outputFrameBuffer[headout++] = escapeSequence[i];
     
  }

  //send message
  i = 0;
  while(message[i] != 0)
  {
      outputFrameBuffer[headout++] = message[i++];
      
  }

  //send end of escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      outputFrameBuffer[headout++] = endOfescapeSequence[i];
      
  }
   
}
