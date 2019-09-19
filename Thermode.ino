
//***********************************************************************************
//*********** defines
//***********************************************************************************

// thermode v 1.0b
// just see whether GIT works


#include "Thermode.h"
#include "SerialCommand.h"

#define SCALE_FACTOR     1000  //all floats are multiplied by this internally
#define A         1
#define B         2
#define DIGIHI    5      //H Pulse for x ms


#define SCK       16E6    // clock at 16 MHz
#define MAX_TIME  4194240 //this is the longest in us the timer can do precisely


// For prescaler = 8, 64, 256, 1024 use OSP_SET_AND_FIRE_LONG(cycles) instead. The "wait" time-waster makes it work!
//#define wait {delayMicroseconds(2);} // Un-comment this for prescaler = 8
//#define wait {delayMicroseconds(5);} // ...for prescaler = 64, make sure we get at least one clock
//#define wait {delayMicroseconds(17);} // ...for prescaler = 256

#define wait {delayMicroseconds(65);} // ...for prescaler = 1024
#define OSP_SET_AND_FIRE_LONG_A(cycles) {uint16_t m=0xffff-(cycles-1); OCR1A=m; wait; TCNT1 = m-1;} // for prescaler > 1
#define OSP_SET_AND_FIRE_LONG_B(cycles) {uint16_t m=0xffff-(cycles-1); OCR1B=m; wait; TCNT1 = m-1;}
#define OSP_INPROGRESS() (TCNT1>0)


//***********************************************************************************
//*********** initialize global variables
//***********************************************************************************


//#define UpPin PB6     //OC1B
//#define DownPin PB5   //OC1A
#define UpPin 12     //OC1B
#define DownPin 11   //OC1A


//OC1A = GPIO port PB5 = Arduino Digital Pin D11 Mega2560 DOWN blue
//OC1B = GPIO port PB6 = Arduino Digital Pin D12 Mega2560 UP green


const float SWversion          = 2.2;
const byte  StartPin           = 7;
const byte  LedPin             = 13;
const byte  Digitimer          = 6;

String LastCmd; //last cmd goes here

boolean  RoR_set, T_set;
int32_t RoR, T, oldT, cps, prescaler;
uint8_t DebugMode;

//complex variables
//
SerialCommand sCmd;                               // The demo SerialCommand object

//***********************************************************************************
//*********** Initialize
//***********************************************************************************

void setup()
{
    DebugMode = 1;
    pinMode(UpPin, OUTPUT);
    digitalWrite(UpPin, LOW);

    pinMode(DownPin, OUTPUT);
    digitalWrite(DownPin, LOW);

    pinMode(StartPin, OUTPUT);
    digitalWrite(StartPin, LOW);

    pinMode(Digitimer, OUTPUT);
    digitalWrite(Digitimer, LOW);

    pinMode(LedPin, OUTPUT);
    digitalWrite(LedPin, LOW);

	
	TCCR1B = 0; // Halt counter by setting clock select bits to 0 (No clock source).
    // This keeps anything from happening while we get set up

    TCNT1 = 0x0000; // Start counting at bottom.

	
    RoR_set  = false;
    T_set    = false;
	
    Serial.begin(115200);


    sCmd.addCommand("DIAG",  processDIAG);
    sCmd.addCommand("MOVE",  processMOVE);
    sCmd.addCommand("START",  processSTART);
    sCmd.addCommand("ROR",  processROR);
    sCmd.addCommand("T",  processT);
    sCmd.addCommand("SET",  processSET);
    sCmd.addCommand("SETPREC",  processSETPREC);
    sCmd.addCommand("SHOCK", processSHOCK);
    sCmd.addCommand("GETTIME", processGETTIME);
    sCmd.addCommand("DEBUG", processDEBUG);
    sCmd.addCommand("HELP",  processHELP);

    sCmd.setDefaultHandler(unrecognized);         // Handler for command that isn't matched  (says "What?")

}


//***********************************************************************************
//*********** Run
//***********************************************************************************

void loop()
{
    sCmd.readSerial();                             //  parse
}


//***********************************************************************************
//*********** Setup Parse handlers
//***********************************************************************************


void processDIAG()
{
    char *arg;
    displayStatusSerial();
}

void processDEBUG()
{
    char *arg;
    LastCmd = "DEBUG;";
    uint8_t New;
    arg = sCmd.next();                           // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)                              // if there is more, take it
    {
        New = atoi(arg);
        DebugMode = check_range_def(New, (byte)0, (byte)4, (byte)1);
        Serial.print(F("DebugMode = "));
        Serial.println(DebugMode);
    }
}



void processMOVE()
{
    char *arg;
    int32_t New;
    LastCmd = "MOVE;";
    arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)                              // As long as it existed, take it
    {
        LastCmd = LastCmd + arg;
        if ((RoR_set) && (T_set) && (!OSP_INPROGRESS()))
        {
            New = atol(arg);
            if (abs(New) < MAX_TIME)
            {
                ramp_temp_prec(New); //Better to pass over us and then decide in ramp which prescaler to use
            }
            else
            {
                if (DebugMode > 0)
                {
                    Serial.print(F("Pulse time longer than "));
                    Serial.println(MAX_TIME);
                    Serial.println(F("using ramp_temp"));
                }
                ramp_temp(New / 1000);
            }
        }
        else
        {
            Serial.println(F("Not executed: Check RoR, T and make sure no pulse is active"));
        }
    }
}



void processSTART()
{
    LastCmd = "START;";
    Serial.print(F("START "));
    pinMode(StartPin, OUTPUT);
    digitalWrite(StartPin, HIGH);   // port high
    delay(100);                     // wait
    digitalWrite(StartPin, LOW);    // port low
}

void processROR()
{
    char *arg;
    double New;
    LastCmd = "ROR;";
    arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)                              // As long as it exists, take it
    {
        LastCmd = LastCmd + arg;
        New = atof(arg);
        RoR = New*SCALE_FACTOR;
        RoR_set = true;
        Serial.print(F("RoR set to "));
        Serial.println((float)RoR/SCALE_FACTOR,2);
    }
}

void processT()
{
    char *arg;
    double New;
    LastCmd = "T;";
    arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)                              // As long as it existed, take it
    {
        LastCmd = LastCmd + arg;
        New = atof(arg);
        T = New*SCALE_FACTOR;
        T_set = true;
        Serial.print(F("Start Temp set to "));
        Serial.println((float)T/SCALE_FACTOR,2);
    }
}

void processSET()
{
    char *arg;
    double New, diff, diff_ms;
    int    int_diff_ms;

    LastCmd = "SET;";
    arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)                              // As long as it existed, take it
    {
        LastCmd = LastCmd + arg;
        New = atof(arg);

        if ((RoR_set) && (T_set) && (!OSP_INPROGRESS()))
        {
            oldT = T;
            diff = (New*SCALE_FACTOR) - oldT;
            diff_ms = diff / RoR * 1000;
            int_diff_ms = round(diff_ms);
            // now ramp it up
            ramp_temp(int_diff_ms);
        }
        else  // RoR_set or T_set not set
        {
            Serial.println(F("Not executed: Check RoR, T and make sure no pulse is active"));
        }
    }
}

void processSETPREC()
{

    char *arg;
    float New;
    int64_t diff;
    int32_t diff_us;

    LastCmd = "SETPREC;";
    arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)                              // As long as it existed, take it
    {
        LastCmd = LastCmd + arg;
        New = atof(arg);

        if ((RoR_set) && (T_set) && (!OSP_INPROGRESS()))
        {
            oldT = T;
            diff = New * SCALE_FACTOR - oldT;

            diff_us = (diff*1000000) / RoR;

            if (abs(diff_us) < MAX_TIME)
            {
                ramp_temp_prec(diff_us);
            }
            else
            {
                if (DebugMode > 0)
                {
                    Serial.print(F("Pulse time longer than "));
                    Serial.println(MAX_TIME);
                    Serial.println(F("using SET"));
                }
                ramp_temp(diff_us / 1000);
            }
        }
        else  // RoR_set or T_set not set
        {
            Serial.println(F("Not executed: Check RoR, T and make sure no pulse is active"));
        }
    }
}


void processSHOCK()
{
    char *arg;
    uint16_t New, dur_int, isi;
    uint32_t StartTime, CurrentTime;
    LastCmd = "SHOCK;";
    arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)                              // As long as it existed, take it
    {
        LastCmd = LastCmd + arg;
        New = atoi(arg);
        dur_int = New;
    }
    arg = sCmd.next();                           // Get the next argument from the SerialCommand object buffer
    if (arg != NULL)
    {
        LastCmd = LastCmd + arg;
        New = atoi(arg);
        isi = New;
        if (DebugMode > 0)
        {
            Serial.print(F("Stimulating for "));
            Serial.print(dur_int);
            Serial.print(F(" ms, every "));
            Serial.print(isi);
            Serial.println(F(" ms"));

            Serial.print(F("Elapsed time since start: "));
            Serial.print(StartTime);
            Serial.println(F(" ms."));
        }
    }
    digitalWrite(LedPin, HIGH);   // LED ON
    StartTime = millis();
    while (millis() - StartTime < dur_int)
    {
        digitalWrite(Digitimer, HIGH);  //
        delay(DIGIHI);                    // waits for ms
        digitalWrite(Digitimer, LOW);   //
        delay(isi-DIGIHI);
    }
    digitalWrite(LedPin, LOW);   // LED OFF
}


void processGETTIME()
{
    LastCmd = "START;";
    Serial.println(millis());
}

void processHELP()
{
    char *arg;
    LastCmd = "HELP;";
    displayHelp();
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command)
{
    LastCmd  = "What?"; //could print the whole command
}


//***********************************************************************************
//*********** diagnostics, free RAM (i.e. between stack and heap)
//***********************************************************************************
int freeRam ()

{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}



//***********************************************************************************
//*********** status display over serial port evoked bey serial command DIAG;
//***********************************************************************************
void displayStatusSerial ()


{

    Serial.print(F("+++ "));
    Serial.print(F(" V:"));
    Serial.print(SWversion, 1);
    Serial.print(F(" RAM:"));
    Serial.print(freeRam());
    Serial.println(F(" +++"));

    Serial.print(F("Debug level: "));
    Serial.println(DebugMode);

    Serial.print(F("Current Temp: "));
    Serial.print((float)T/SCALE_FACTOR,2);
    Serial.println(F(" C"));

    Serial.print(F("Rate of rise: "));
    Serial.print((float)RoR/SCALE_FACTOR,2);
    Serial.println(F(" C/s"));

    Serial.print(F("Current Temp: "));
    Serial.print(T);
    Serial.println(F(" raw"));

    Serial.print(F("Rate of rise: "));
    Serial.print(RoR);
    Serial.println(F(" raw"));

    Serial.print(F("Last serial command: "));
    Serial.println(LastCmd);

}

//***********************************************************************************
//*********** status help over serial port evoked by serial command HELP;
//***********************************************************************************
void displayHelp ()


{
    Serial.println(F("DIAG          - Get diagnostics"));
    Serial.println(F("DEBUG;XX      - Set debug state (0: OFF)"));
    Serial.println(F("ROR;XX.xx     - Set rate of rise to XX.xx C/s"));
    Serial.println(F("T;XX.xx       - Set baseline temperature to XX.xx C"));
    Serial.println(F("START         - Send 100ms TTL pulse to start program"));
    Serial.println(F("MOVE;XX       - Move temp up/down for XX us"));
    Serial.println(F("SET;XX.xx     - Set new temperature to XX.xx C (less precise all pulse durations)"));
    Serial.println(F("SETPREC;XX.xx - Set new temperature to XX.xx C (more precise max 4s pulse)"));
    Serial.println(F("SHOCK;xx;yy   - Set Digitimer total stimulus duration (xx) and interval between pulses (yy) in ms"));
    Serial.println(F("GETTIME       - Get Arduino time in ms"));
    Serial.println();
    Serial.println(F("ROR and T need to be called BEFORE you can use MOVE, SET and SETPREC"));
}




//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp(int ms)

{
    int64_t inc1;                                 // cannot be avoided
    int32_t inc2;

    // now we adjust T
    inc1 = (int64_t)ms * RoR;
    inc2 = inc1/1000;
    oldT = T;
    T    = T + inc2;
    if (DebugMode > 0)
    {
        Serial.print(F("Going from "));
        Serial.print((float)oldT/SCALE_FACTOR, 2);
        Serial.print(F("C to "));
        Serial.print((float)T/SCALE_FACTOR, 2);
        Serial.print(F("C in "));
        Serial.print(ms);
        Serial.println(F("ms"));
    }


    if   (ms < 0)
    {
        ms = abs(ms);
        if (DebugMode>0)
        {
            Serial.print(F("Ramping down:  "));
            Serial.println(ms);
        }
        TCCR1B = 0;
		digitalWrite(LedPin, HIGH);   // LED ON
        pinMode(DownPin, OUTPUT);
        digitalWrite(DownPin, HIGH);  //
        delay(ms);                    // waits for ms
        digitalWrite(DownPin, LOW);   //
        digitalWrite(LedPin, LOW);   // LED OFF
    }
    else
    {
        if (DebugMode>0)
        {
            Serial.print(F("Ramping up:  "));
            Serial.println(ms);
        }
        TCCR1B = 0;
		digitalWrite(LedPin, HIGH);   // LED ON
        pinMode(UpPin, OUTPUT);
        digitalWrite(UpPin, HIGH);   // port high
        delay(ms);                  // waits for ms
        digitalWrite(UpPin, LOW);    // sets the LED off
        digitalWrite(LedPin, LOW);   // LED ON
    }
}



//***********************************************************************************
//*********** Prepare timer
//***********************************************************************************

void osp_setup(uint8_t which, int32_t prescaler) {

    TCCR1B = 0; // Halt counter by setting clock select bits to 0 (No clock source).
// This keeps anything from happening while we get set up

    TCNT1 = 0x0000; // Start counting at bottom.

    ICR1 = 0;// Set TOP to 0, Mode 14. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
// We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX
// and then overflow back to 0 and get locked up again.



    if (which == A)
    {
        OCR1A = 0xffff;
        TCCR1A = (1<<COM1A0) | (1<<COM1A1) | (1<<WGM11); // OC1A=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
// Set OC1A to output, pick your board- Uno vs 2560
// DDRB = (1<<1);     // Set pin to output (Note that OC1A = GPIO port PB1 = Arduino Digital Pin D9 Uno)
        DDRB = (1<<5);     // Set pin to output (Note that OC1A = GPIO port PB5 = Arduino Digital Pin D11 Mega2560)
    }
    else if (which == B)
    {
        OCR1B = 0xffff;
        TCCR1A = (1<<COM1B0) | (1<<COM1B1) | (1<<WGM11); // OC1B=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
// Set OC1B to output, pick your board- Uno vs 2560
//DDRB = (1<<2);     // Set pin to output (Note that OC1B = GPIO port PB2 = Arduino Digital Pin D10 Uno)
        DDRB = (1<<6);     // Set pin to output (Note that OC1B = GPIO port PB6 = Arduino Digital Pin D12 Mega2560)
    }

    else
    {
        Serial.println(F("ERROR only OC1A or OC1B supported"));
    }

//   (using Chris Hahn's notation here)
// Prescaler  Setup - Choose one of these, then choose a matching "wait" delay statement below.
//TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10); // Prescaler = 1; Start counting now. Max ~4mS
//TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Prescaler = 8; Start counting now. Max ~32mS, starts in ~10uS or better
//TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS11); // Prescaler = 64; Start counting now. Max ~.26 sec, starts in ~20uS or better
    if (prescaler == 256)
    {
        TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS12); // Prescaler = 256; Start counting now. Max ~1.05 sec, starts in ~64uS or better
    }
    else if (prescaler == 1024)
    {
        TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS12); // Prescaler = 1024; Start counting now. Max ~4 sec, starts in ~180uS or better
    }
    else
    {
        Serial.println(F("ERROR only 256 or 1024 supported"));
    }
}


//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp_prec(int32_t o_us)  // specifiy in us
{
    int64_t inc1;                                 // cannot be avoided
    int32_t inc2, o_tic;

    if ((RoR_set) && (T_set) && (!OSP_INPROGRESS()))
    {

        if (abs(o_us) < 1048560) // we can use 256 as a prescaler
        {
            prescaler = 256;    // we get up to ~4 s pulses
        }
        else
        {
            prescaler = 1024;
        }

        cps = SCK/prescaler;      // tics per second
        // now we adjust T
        inc1 = (int64_t)o_us * RoR;
        //Serial.print(F("inc1:  "));
        //Serial.println((int32_t)inc1);
        inc2 = inc1/1000000;
        //Serial.print(F("inc2:  "));
        //Serial.println(inc2);
        oldT = T;
        T    = T + inc2;
        // now convert us into tics
        o_tic = (int64_t)o_us * cps / 1000000; //convert from us to tics

        if (DebugMode > 0)
        {
            Serial.print(F("Going from "));
            Serial.print((float)oldT/SCALE_FACTOR, 2);
            Serial.print(F("C to "));
            Serial.print((float)T/SCALE_FACTOR, 2);
            Serial.print(F("C in "));
            Serial.print(o_us);
            Serial.print(F("us i.e. "));
            Serial.print(o_tic);
            Serial.println(F("tics"));
            Serial.print(F("Prescaler:  "));
            Serial.println(prescaler);
        }

        if   (o_tic < 0)
        {
            o_tic = abs(o_tic);

            if (DebugMode>0)
            {
                Serial.print(F("Ramping down:  "));
                Serial.println(o_tic);
            }
            osp_setup(A, prescaler);
            OSP_SET_AND_FIRE_LONG_A(o_tic)     // Use this for prescaler > 1!
        }
        else
        {
            if (DebugMode>0)
            {
                Serial.print(F("Ramping up:  "));
                Serial.println(o_tic);
            }
            osp_setup(B,prescaler);
            OSP_SET_AND_FIRE_LONG_B(o_tic)   // Use this for prescaler > 1!
        }
    }
    else  // RoR_set or T_set not set
    {
        Serial.println(F("Not executed: Check RoR, T and make sure no pulse is active"));
    }

}
