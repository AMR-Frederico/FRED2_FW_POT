

 #include "encoder.hpp" // header in local directory


//--------------------------------------------------
//Pinout - LEGADO
//--------------------------------------------------
// volatile int DI_ENCODER_CH_A = 36;
// volatile int DI_ENCODER_CH_B = 39;

//--------------------------------------------------
//Configuration
//--------------------------------------------------
int encoderPPR = 2400; // dont ask questions 

Encoder* Encoder::obj_Encoder = 0;

//--------------------------------------------------
//Variable Declaration/Initialization
//--------------------------------------------------
volatile bool firstRead[2] = {true, true};

//only to debug interruption time
volatile unsigned long interr_curTime_us = 0;
volatile unsigned long interr_prevTime1_us = 0;
volatile unsigned long interr_prevTime2_us = 0;
volatile unsigned long interr_deltaTime_us = 0;
volatile unsigned long interr_highestTime_us = 0;

//Pulse Counter
volatile bool curAL, curBL, prevAL, prevBL; 
volatile bool curAR, curBR, prevAR, prevBR; 

volatile unsigned long pulseTime[2] = {0, 0};
volatile double encoderCount[2] = {0, 0};
volatile double encoderPulseCount[2] = {0, 0};
volatile double encoderPulseError[2] = {0, 0};
volatile bool encoderErro[2] = {0, 0};



//Calculate Current Angle
double curAngle[2] = {0, 0};


//Calculate RPM
volatile unsigned long pulseTimeLatch[2] = {0, 0};
volatile unsigned long pulsePrevTime[2] = {0, 0};
volatile unsigned long pulsePrevPrevTime[2] = {0, 0};
volatile unsigned long pulseDeltaTime[2] = {0, 0};

volatile double encoderCountLatch[2] = {0, 0};
volatile double encoderPrevCount[2] = {0, 0};
volatile double encoderPrevPrevCount[2] = {0, 0};
volatile double encoderDeltaCount[2] = {0, 0};

volatile bool lastPulseForward[2] = {0, 0};
volatile bool lastPulseBackward[2] = {0, 0};

volatile double curRPM[2] = {0, 0};
volatile double prevRPM[2] = {0, 0};
volatile double highestRPM[2] = {0, 0};

volatile int filterGain[2] = {20, 20};
volatile double curRPM_Filtered[2] = {0, 0};

volatile int motor_gear = 60 ;
volatile int encoder_gear = 20; 
volatile int relation_motor_encoder = motor_gear/encoder_gear;

//--------------------------------------------------
// Functions
//--------------------------------------------------

void IRAM_ATTR interruptionChAL();
void IRAM_ATTR interruptionChBL();
void IRAM_ATTR interruptionChAR();
void IRAM_ATTR interruptionChBR();

Encoder::Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR){
    this-> DI_ENCODER_CH_AL = pin_AL;
    this-> DI_ENCODER_CH_BL = pin_BL;
    this-> DI_ENCODER_CH_AR = pin_AR;
    this-> DI_ENCODER_CH_BR = pin_BR;
    //Serial.println("EncSetup");
    
    //IO
    pinMode(DI_ENCODER_CH_AL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_BL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_AR, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_BR, INPUT_PULLUP);

    //Read Channels AB from Left Encoder Initial State
    curAL = digitalRead(DI_ENCODER_CH_AL);
    curBL = digitalRead(DI_ENCODER_CH_BL);
    prevAL = curAL;
    prevBL = curBL;

    //Read Channels AB from Right Encoder Initial State
    curAR = digitalRead(DI_ENCODER_CH_AR);
    curBR = digitalRead(DI_ENCODER_CH_BR);
    prevAR = curAR;
    prevBR = curBR;



}


void Encoder::setup()
{
    obj_Encoder = this;
    //Configure Interrupt
    attachInterrupt(DI_ENCODER_CH_AL, Encoder::interruptionChAL, CHANGE);
    attachInterrupt(DI_ENCODER_CH_BL, Encoder::interruptionChBL, CHANGE);
    attachInterrupt(DI_ENCODER_CH_AR, Encoder::interruptionChAR, CHANGE);
    attachInterrupt(DI_ENCODER_CH_BR, Encoder::interruptionChBR, CHANGE);
    
}


void IRAM_ATTR Encoder::interruptionChAL()
{

    //Serial.println("A");
    //  interr_prevTime1_us = micros();

    pulseTime[LEFT] = micros();

    curAL = digitalRead(obj_Encoder-> DI_ENCODER_CH_AL);

    if(curAL == prevAL)
    {
        //depois da interrupção, já leu e voltou ao valor anterior -> debounce = faz nada
    }
    else
    {
    curBL = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);
// /
    if (curBL != prevBL){
        encoderErro[LEFT] = true;
        encoderPulseError[LEFT]++;
        //Serial.println("E_A");
    }

    // Serial.print("A:");
    // Serial.print(prevAL);
    // Serial.print(prevBL);
    // Serial.print("-");
    // Serial.print(curAL);
    // Serial.print(curBL);
    // Serial.print("-");
    // Serial.println(encoderPulseError);

    prevAL = curAL;
    prevBL = curBL;

    if (curAL == false)
    {
        if (curBL == true)
        {
        encoderCount[LEFT]++;
        }
        else {
        encoderCount[LEFT]--;
        }
        
    }
    if (curAL == true)
    {
        if (curBL == false)
        {
        encoderCount[LEFT]++;
        }
        else {
        encoderCount[LEFT]--;
        }
    }

    encoderPulseCount[LEFT]++;


    //      interr_prevTime2_us = interr_prevTime1_us;
    //      interr_curTime_us = micros();
    //      interr_deltaTime_us = interr_curTime_us - interr_prevTime2_us;
    //      
    //      if (interr_deltaTime_us > interr_highestTime_us) interr_highestTime_us = interr_deltaTime_us;
    
    }
}

void IRAM_ATTR Encoder::interruptionChBL()
{
    //Serial.println("B");

    pulseTime[LEFT] = micros();

    curBL = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);

    if(curBL == prevBL)
    {
    //depois da interrupção, já leu e voltou ao valor anterior -> debounce = faz nada
    }
    else
    {

    curAL = digitalRead(obj_Encoder->DI_ENCODER_CH_AL);

    if (curAL != prevAL){
        encoderErro[LEFT] = true;
        encoderPulseError[LEFT]++;
        //Serial.println("E_B");
    }

    //    Serial.print("B - ");
    //    Serial.print(prevAL);
    //    Serial.print(prevBL);
    //    Serial.print("-");
    //    Serial.print(curAL);
    //    Serial.print(curBL);
    //    Serial.print("-");
    //    Serial.println(encoderPulseError[LEFT]);

    prevBL = curBL;
    prevAL = curAL;

    if (curBL == false)
    {
        if (curAL == false)
        {
        encoderCount[LEFT]++;
        }
        else {
        encoderCount[LEFT]--;
        }
        
    }
    if (curBL == true)
    {
        if (curAL == true)
        {
        encoderCount[LEFT]++;
        }
        else {
        encoderCount[LEFT]--;
        }
    }

    encoderPulseCount[LEFT]++;

    }
  
}

void IRAM_ATTR Encoder::interruptionChAR()
{

    //Serial.println("A");
    //  interr_prevTime1_us = micros();

    pulseTime[RIGHT] = micros();

    curAR = digitalRead(obj_Encoder-> DI_ENCODER_CH_AR);

    if(curAR == prevAR)
    {
        //depois da interrupção, já leu e voltou ao valor anterior -> debounce = faz nada
    }
    else
    {
    curBL = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);
// /
    if (curBR != prevBR){
        encoderErro[RIGHT] = true;
        encoderPulseError[RIGHT]++;
        //Serial.println("E_A");
    }

    // Serial.print("A:");
    // Serial.print(prevAR);
    // Serial.print(prevBR);
    // Serial.print("-");
    // Serial.print(curAR);
    // Serial.print(curBR);
    // Serial.print("-");
    // Serial.println(encoderErro[RIGHT]);

    prevAR = curAR;
    prevBR = curBR;

    if (curAR == false)
    {
        if (curBR == true)
        {
        encoderCount[RIGHT]++;
        }
        else {
        encoderCount[RIGHT]--;
        }
        
    }
    if (curAR == true)
    {
        if (curBR == false)
        {
        encoderCount[RIGHT]++;
        }
        else {
        encoderCount[RIGHT]--;
        }
    }

    encoderPulseCount[RIGHT]++;


    //      interr_prevTime2_us = interr_prevTime1_us;
    //      interr_curTime_us = micros();
    //      interr_deltaTime_us = interr_curTime_us - interr_prevTime2_us;
    //      
    //      if (interr_deltaTime_us > interr_highestTime_us) interr_highestTime_us = interr_deltaTime_us;
    
    }
}

void IRAM_ATTR Encoder::interruptionChBR()
{
    //Serial.println("B");

    pulseTime[RIGHT] = micros();

    curBR = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);

    if(curBR == prevBR)
    {
    //depois da interrupção, já leu e voltou ao valor anterior -> debounce = faz nada
    }
    else
    {

    curAR = digitalRead(obj_Encoder->DI_ENCODER_CH_AR);

    if (curAR != prevAR){
        encoderErro[RIGHT] = true;
        encoderPulseError[RIGHT]++;
        //Serial.println("E_B");
    }

    //    Serial.print("B - ");
    //    Serial.print(prevAR);
    //    Serial.print(prevBR);
    //    Serial.print("-");
    //    Serial.print(curAR);
    //    Serial.print(curBR);
    //    Serial.print("-");
    //    Serial.println(encoderErro[RIGHT]);

    prevBR = curBR;
    prevAR = curAR;

    if (curBR == false)
    {
        if (curAR == false)
        {
        encoderCount[RIGHT]++;
        }
        else {
        encoderCount[RIGHT]--;
        }
        
    }
    if (curBR == true)
    {
        if (curAR == true)
        {
        encoderCount[RIGHT]++;
        }
        else {
        encoderCount[RIGHT]--;
        }
    }

    encoderPulseCount[RIGHT]++;

    }
  
}

double Encoder::readAngle(int encoder_side){
    curAngle[encoder_side] = ((double)360.0 * (double)encoderCount[encoder_side]) / ((double)(3.0*encoderPPR));

    return curAngle[encoder_side];

}

double Encoder::readPulses(int encoder_side){
    return encoderCount[encoder_side];

}


double Encoder::readRPM(int encoder_side)
{

    noInterrupts(); //Disable interruptions - Critical code
    pulseTimeLatch[encoder_side] = pulseTime[encoder_side];
    encoderCountLatch[encoder_side] = encoderCount[encoder_side];
    interrupts(); //Re-Enable interruptions

    encoderDeltaCount[encoder_side] = encoderCountLatch[encoder_side] - encoderPrevCount[encoder_side];

    
    if (firstRead[encoder_side]==true)
    {
        curRPM[encoder_side] = 0;
        firstRead[encoder_side] = false;
    }
    else
    {
        if (encoderDeltaCount[encoder_side] != 0) //New pulses
        {
            
            
            pulseDeltaTime[encoder_side] = pulseTimeLatch[encoder_side] - pulsePrevTime[encoder_side];
            
            if (pulseDeltaTime[encoder_side] < 1) //Prevent division by 0
            {
                curRPM[encoder_side] = 0;
            }
            else
            {
                curRPM[encoder_side] = (double)encoderDeltaCount[encoder_side]*(1000000.0/(double)pulseDeltaTime[encoder_side])*60.0/2400.0;
            }

            //Set flag to remember if going forward or backward
            if (encoderDeltaCount[encoder_side]>0)
            {
                lastPulseForward[encoder_side] = true;
                lastPulseBackward[encoder_side] = false;
            }else
            {
                lastPulseForward[encoder_side] = false;
                lastPulseBackward[encoder_side] = true;
            }
            
        }
        else //No new pulses, use current time
        {
            pulseDeltaTime[encoder_side] = micros() - pulsePrevTime[encoder_side];
         
            if (pulseDeltaTime[encoder_side] < 1)
            {
                curRPM[encoder_side] = 0;
            }
            else
            {
                curRPM[encoder_side] = (double)1.0*(1000000.0/(double)pulseDeltaTime[encoder_side])*60.0/2400.0;
            }

            if (lastPulseBackward[encoder_side])
            {
                curRPM[encoder_side] = -curRPM[encoder_side];
                
            }

        }
    }
    
    
    encoderPrevPrevCount[encoder_side] = encoderPrevCount[encoder_side];
    encoderPrevCount[encoder_side] = encoderCountLatch[encoder_side];


    pulsePrevPrevTime[encoder_side] = pulsePrevTime[encoder_side];
    if (encoderDeltaCount[encoder_side] != 0) //New pulses to update time
    {
        pulsePrevTime[encoder_side] = pulseTimeLatch[encoder_side];
    }
    

    if (curRPM[encoder_side] < 0.5 && curRPM[encoder_side] > -0.5)
    {
        curRPM[encoder_side] = 0;
    }

    //filter glitches (position overflow)
    if ((curRPM[encoder_side] > 10000) | (curRPM[encoder_side] < -10000)){
        curRPM[encoder_side] = prevRPM[encoder_side];
    }  else {
        prevRPM[encoder_side] = curRPM[encoder_side];
    }
    
    //---
    curRPM_Filtered[encoder_side] = (curRPM_Filtered[encoder_side]*filterGain[encoder_side]+curRPM[encoder_side])/(filterGain[encoder_side]+1);
    if (curRPM_Filtered[encoder_side] < 0.1 && curRPM_Filtered[encoder_side] > -0.1)
    {
        curRPM_Filtered[encoder_side] = 0;
    }
    

    //Store highest RPM ever
    if (curRPM[encoder_side] > highestRPM[encoder_side]){
        highestRPM[encoder_side] = curRPM[encoder_side];
    }

    return curRPM_Filtered[encoder_side]/relation_motor_encoder;

}

void Encoder::reset()
{
  encoderCount[LEFT] = 0;
  encoderCount[RIGHT] = 0;
  encoderErro[LEFT] = false;
  encoderErro[RIGHT] = false;
  highestRPM[LEFT] = 0;
  highestRPM[RIGHT] = 0;
  encoderPulseCount[LEFT] = 0;
  encoderPulseCount[RIGHT] = 0;
  encoderPulseError[LEFT] = 0;
  encoderPulseError[RIGHT] = 0;
  
}


//--------------------------------------------------
//Debug Print
//--------------------------------------------------
void Encoder:: debugPrint() {

    Serial.print("|Encoder->");

    //Serial.print(digitalRead(DI_ENCODER_CH_A));
    //Serial.println(digitalRead(DI_ENCODER_CH_B));

    // Serial.print(", ErroCount");
    // Serial.print(encoderPulseError);

    //---

    Serial.print(", CurAngle Left:");
    Serial.print(curAngle[LEFT]);
    Serial.print(", CurAngle Right:");
    Serial.print(curAngle[RIGHT]);

    //---

    // Serial.print("P_T:");
    // Serial.print(pulseTimeLatch);

    // Serial.print(", P_PT:");
    // Serial.print(pulsePrevPrevTime);

    Serial.print(", P_DT_LEFT:");
    Serial.print(pulseDeltaTime[LEFT]);
    Serial.print(", P_DT_RIGHT:");
    Serial.print(pulseDeltaTime[RIGHT]);

    // Serial.print(", P_C:");
    // Serial.print(encoderCountLatch);

    // Serial.print(", P_PC:");
    // Serial.print(encoderPrevPrevCount);
    
    Serial.print(", P_DC_LEFT:");
    Serial.print(encoderDeltaCount[LEFT]);
    Serial.print(", P_DC_RIGHT:");
    Serial.print(encoderDeltaCount[RIGHT]);

    Serial.print(", RPM LEFT:");
    Serial.print(curRPM[LEFT]);
    Serial.print(", RPM RIGHT:");
    Serial.print(curRPM[RIGHT]);

    // Serial.print(", RPM_F:");
    // Serial.print(curRPM_Filtered);
    
    // Serial.print(", RPM_H:");
    // Serial.print(highestRPM);
    

    Serial.println();

}

EncoderData Encoder::get_encoder_data(int id){
  
    
    double angle = readAngle(id);
    double rpm =  readRPM(id);
    double ticks = readPulses(id);

    return EncoderData{angle, rpm, ticks};
}
