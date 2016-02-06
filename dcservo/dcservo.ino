
/* This one is not using any PinChangeInterrupt library */

/*
   This program uses an Arduino for a closed-loop control of a DC-motor. 
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).
   
*/
#include <EEPROM.h>
#include <PID_v1.h>
#define encoder0PinA      2   // Needs to be on D2 to allow attachment to INT0. see setup()
#define encoder0PinB      8   // Needs to be on this pin for Pin Change Interrrupt to work. see pciSetup()
#define DIR_INPUT        A0   // Needs to be on A0(D14) to match port masking in ISR.  See countStep()
#define STEP_INPUT        3   // Needs to be on D3 to allow attachment to INT1.  see setup()
#define M1                9   // Motor's Forward PWM output
#define M2               10   // Motor's Backward PWM output

#define MANUAL_FWD        4   // Manual switch input - forward direction
#define MANUAL_REV        5   // Manual switch input - reverse direction
#define MANUAL_STEP_SIZE  2   // Manual switch step size (steps per 10mS)

#define LIMIT_SW_FWD      6   // Limit switch input - goes low when reaching end of forward travel
#define LIMIT_SW_REV      7   // Limit switch input - goes low when reaching end of reverse travel

byte pos[1000];
int p=0;

double kp=5;
double ki=20;
double kd=0.2;

double input=0;
double output=0;
double setpoint=0;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);

boolean monitorPosition=false;

long target1=0;  // destination location at any moment
volatile long encoder0Pos = 0;

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;
byte skip=0;

// Install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin) 
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}

void setup()
{ 
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);  
  
  pinMode(MANUAL_FWD, INPUT_PULLUP);
  pinMode(MANUAL_REV, INPUT_PULLUP);
  
  pinMode(LIMIT_SW_FWD, INPUT_PULLUP);
  pinMode(LIMIT_SW_REV, INPUT_PULLUP);
  
  pciSetup(encoder0PinB);
  attachInterrupt(0, encoderInt, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, countStep,  RISING);  // step  input on interrupt 1 - pin 3
  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  
  digitalWrite(DIR_INPUT, 1);  //activate weak pull-ups on inputs
  digitalWrite(STEP_INPUT, 1); //activate weak pull-ups on inputs
  
  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();
  
  //Setup the PID parameters
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255,255);
} 

void loop()
{
  input = encoder0Pos; 

  if(Serial.available())  process_line(); // it may induce a glitch to move motion, so use it sparingly 
  if(monitorPosition) 
  {
    if(millis() % 1000 == 0) printPos();
  }

  if(millis() % 10 == 0)  // every 100ms, check for manual override input and adjust target accordingly
  {
    if(!digitalRead(MANUAL_FWD))
    {
      target1+=MANUAL_STEP_SIZE;
    }
    if(!digitalRead(MANUAL_REV))
    {
      target1-=MANUAL_STEP_SIZE;
    }
  }

  setpoint=target1*10;  // applying a multiplier here since the motor cannot accurately position less than this resolution
  myPID.Compute();
  pwmOut(output); 
}

void pwmOut(int out)
{
  if(out>0)
  { 
    analogWrite(M1,0); analogWrite(M2,abs(out)); 
  }
  else
  {
    analogWrite(M2,0); analogWrite(M1,abs(out)); 
  }
}

const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};               // Quadrature Encoder Matrix
static unsigned char New, Old;

ISR (PCINT0_vect) // handle pin change interrupt for D8
{
  Old = New;
  New = (PINB & 1 )+ ((PIND & 4) >> 1); //
  encoder0Pos+= QEM [Old * 4 + New];
}

void encoderInt() // handle pin change interrupt for D2
{
  Old = New;
  New = (PINB & 1 )+ ((PIND & 4) >> 1); //
  encoder0Pos+= QEM [Old * 4 + New];
}

void countStep()
{
  if (PINC&B0000001) target1--;else target1++;  // pin A0 represents direction
} 

void process_line()
{
 char cmd = Serial.read();
 if(cmd>'Z') cmd-=32;
 switch(cmd)
 {
  case 'A': monitorPosition = !monitorPosition; break;
  case 'P': kp=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'I': ki=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'D': kd=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'X': target1=Serial.parseInt(); break;
  case '?': printPos(); break;
  case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.println(kd); break;
  case 'W': writetoEEPROM(); break;
  case 'H': help(); break;
  case 'K': eedump(); break;
  case 'R': recoverPIDfromEEPROM() ; break;
 }
 while(Serial.read()!=10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos()
{
  Serial.print(F("Position=")); Serial.print(encoder0Pos); Serial.print(F(" PID_output=")); Serial.print(output); Serial.print(F(" Target=")); Serial.println(setpoint);
}

void help()
{
 Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
 Serial.println(F("by misan"));
 Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
 Serial.println(F("A will toggle on/off showing status every second\n")); 
 Serial.println(F("P123.34 sets proportional term to 123.34"));
 Serial.println(F("I123.34 sets integral term to 123.34"));
 Serial.println(F("D123.34 sets derivative term to 123.34"));
 Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
 Serial.println(F("? prints out current encoder, output and setpoint values"));
 Serial.println(F("Q will print out the current values of P, I and D parameters")); 
 Serial.println(F("W will store current values of P, I and D parameters into EEPROM")); 
 Serial.println(F("H will print this help message again")); 
}

void writetoEEPROM()  // keep PID set values in EEPROM so they are kept when arduino goes off
{ 
  eeput(kp,0);
  eeput(ki,4);
  eeput(kd,8);
  double cks=0;
  for(int i=0; i<12; i++)
  {
    cks+=EEPROM.read(i);
  }
  eeput(cks,12);
  Serial.println("\nPID values stored to EEPROM");
  //Serial.println(cks);
}

void recoverPIDfromEEPROM()
{
  double cks=0;
  double cksEE;
  for(int i=0; i<12; i++) cks+=EEPROM.read(i);
  cksEE=eeget(12);
  //Serial.println(cks);
  if(cks==cksEE)
  {
    Serial.println(F("*** Found PID values on EEPROM"));
    kp=eeget(0);
    ki=eeget(4);
    kd=eeget(8);
    myPID.SetTunings(kp,ki,kd); 
  }
  else Serial.println(F("*** Bad checksum"));
}

void eeput(double value, int dir)  // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
{ 
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++)
  {
    EEPROM.write(i,addr[i-dir]);
  }
}

double eeget(int dir)   // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
{
  double value;
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++) 
  {
    addr[i-dir]=EEPROM.read(i);
  }
  return value;
}

void eedump() 
{
 for(int i=0; i<16; i++) { Serial.print(EEPROM.read(i),HEX); Serial.print(" "); }Serial.println(); 
}
