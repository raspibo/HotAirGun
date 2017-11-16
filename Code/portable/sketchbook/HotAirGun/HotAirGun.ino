// include the library code:
#include <Wire.h>
#include <I2cController.h>
#include <EEPROM.h>

//Parameter definition
#define    D_MinT    25  
#define   D_MaxT    450
#define   D_TempGunApp  100
#define   D_AirFlowApp  100   
#define   D_AirFlowMin  40
#define   D_AirFlowMax  100
#define   D_Min_Pid 0
#define   D_Max_Pid 100
#define   D_AutoOffTime 3600

#define   LCD_Update  100
//#define   PID_Update  100

#define   PidTime     40      //Period 1 sec
#define   TStop       25
//#define   Kp            1
//#define   Ki            5
//#define   Kd            0
#define   SumE_Min      -1000
#define   SumE_Max      1000
#define   PidOutMin     0
#define   PidOutMax     1200

//EEPROM data storage
#define   M_Temp    2
#define   M_Temp1   3
#define   M_AirFlow 4
#define   M_KP    5
#define   M_KI    6
#define   M_KD    7
#define   M_MinT    8
#define   M_MaxT    9
#define   M_MaxT1   10  
#define   M_AutoOffTime   11  
#define   M_AutoOffTime1  12  

//I2cController vars definition
I2cControllerLib contr(0x20); // Connect via i2c, default address #0 (A0-A2 not jumpered)
#define EN_A 1      // Encored scroll A0
#define EN_B 2      // Encoder scroll A1 
#define EN_C 0      // Encoder click  A2
#define POTDIVIDER 4    // Encoder tick divider for sensibilty regulation
#define BTN_NUM 5     // Number of buttons
#define BTN_1 3     // Single button A3
#define BTN_2 4     // Single button A4
#define BTN_3 5     // Single button A5
#define BTN_4 6     // Single button A6
#define BTN_5 7     // Single button A7
#define CONTR_NOEVENT  0  // No event detected
#define CONTR_SCROLL 1    // Encoder movement detected -Z > read Pot value
#define CONTR_CLICK  2    // Encoder click detected 
#define CONTR_BTN    3    // Button pressed
uint8_t controllerEvent=CONTR_NOEVENT;  //Store encoder events
int clickcount=0;   
int ticPot=0;           //Used for cont of single step of rotary encoder 
int PotDivider=POTDIVIDER;    //Divider for ticPot for best regulation of sensitivity
int Pot=0;      
char ActEnc, OldEnc;    //Diff between encoder positions
int ModVal;     //Temporary var to store parameter during set
boolean initPar=false;    //Used by menus

//Pin definition
#define ZEROCINTPIN 2 //Interrupt pin used by zero crossing detector circuit DO NOT CHANGE
#define MCPINTPIN 3 //Interrupt pin used by I2C controller (MCP23107)      DO NOT CHANGE

#define SO    5 //MAX6675 signal serial out aka MISO on SPI
#define CS    6 //MAX6675 signal Chip select
#define SCK   7 //MAX6675 signal clock
#define TILTSENSOR  8 //Tilt sensor for gun
#define GATE    9 //TRIAC gate
#define EMERG_RELAY 8 //Emergency and power relay
#define P_FAN_PWM   11  //Define pin D10 for pwm signal for gun
#define _STARTSTOP  12  //Pin used for activation of hot air production (also used for magnetic sensor on gun)
#define DEBUGLED      13  //Led used for debug purpose

//PWM and timers vars
#define PULSE 0x0F      //trigger pulse width (counts)
#define sbi(port,bit) (port)|=(1<<(bit))  //Fast toggle routine for pins

//Interrupts vars
byte MCPIntPin = MCPINTPIN; // Interrupts from the MCP will be handled by this PIN on Arduino
byte arduinoMCPInterrupt = 1; // ... and this interrupt vector
byte ZeroCIntPin = ZEROCINTPIN; // Interrupts from zero crossing circuit
byte arduinoZeroCInterrupt = 0; // ... and this interrupt vector
volatile boolean awakenByMCPInterrupt   = false;  
unsigned long lastMillisInterrupt = 0;
uint8_t phaseCounter=0;   //increment at every phase zero crossing at 100 (=1 sec)  increment opTime

//Timing vars
unsigned long LcdUpd;           //Last time lcd was updated for real time menu refresh es Home 
long relayStartTime=0;    //Relay time start: wait some time after start button press
int AutoOffTime;    //Time for auto shutdown in seconds
int opTime=0;     //operation Timer normally equal to millis(), used for temperature welding curve time calculation, displayed in home menu
long last_PIDTime=0;    //Last time PID was calculated

//PID vars
char KI, KD, KP;    //Used by PID
bool DoPid=0;     //Set to true if it's time to recalculate PID
signed int  SumE, Int_Res, Dev_Res, Err, Err1; //Used by PID
signed long Pid_Res;    //Used by PID
uint16_t TCNT_timer=0;    //Calculate PID controller pulse width, used by interrupt

//Temp, Air, status vars
int ActTemp=0;      //Actual gun air temp
int TempGunApp=0;   //Target temperature for PID temporary variable when weld cycle is not active
int TempGun=0;      //Target temperature for PID, at work the air flow with this temp from gun
int AirFlowApp=0;   //Temporary variable for air flow when weld cycle is not active
int AirFlow=0;
int MaxT = 0;
int MinT = 30;
boolean StartStop, PrevStartStop;
uint8_t WeldStatus;

int X=0;

//Menu and display vars
#define MENU_HOME 0
#define MENU_TITLES 12 
int menu=MENU_HOME;
int menudec,menuunit;
int menuold=-1;
/*
   First char convention for menu voice handling:
   x=empty voice -> skip next level or (for last) home
   v=scroll -> next voice on right or left, click -> next voice down (default)
   h=goto home on click
   m=modify value, scroll adjust value, click save
   n=nothing used for home and menu titles used as spacer
   a=apply and go to home
 */
const char *menuvoice[MENU_TITLES][12]=
{
  {"nHome"},                          //00
  {"nSetFast"     ,"vAirTemp" ,"vAirFlow" ,"hExit"},              //10,11,12..
  {"x"            ,"mModAirTemp"  ,"mModAirFlow"  },                //20,21,22..
  {"x"            ,"hSaveAirTemp" ,"hSaveAirFlow" },                //30,31,32...
  {"nSetPar"      ,"vKP"    ,"vKI"    ,"vKD"    ,"vMinT"    ,"vMaxT"    ,"hExit"},  //40,41..
  {"x"            ,"mMod KP"  ,"mMod KI"  ,"mMod KD"  ,"mMod MinT"    ,"mMod MaxT"    },    //50,51,52.....
  {"x"            ,"hSave KP" ,"hSave KI" ,"hSave KD" ,"hSave MinT"   ,"hSave MaxT"   },    //60...
  {"nMaterials Presets" ,"aSn"    ,"aheat shrink" ,"aLDPE"  ,"aPP/Hard PVC/HDPE"  ,"aABS/PC/Soft PVC" ,"hExit"},  //70..  
  {"nWeldCurve"   ,"aTempCurve" ,"hExit"},                      //80..  
  {"nFunct"   ,"vAutoOff" ,"vPPPreset"    ,"vDefault" ,"hExit"},              //90..  
  {"x"      ,"mAutoOffTime" ,"mPPPreset"  ,"mSure?"},                 //100..
  {"x"      ,"hSaveAutoOff" ,"hSavePPPreset","hSaveDefault"},               //110..
};
char menuDisplay[16] = "";

/*

   tempCurve var 

   |           _______________<time
   |         / ^target        \
   |  ______/<time             \
   | /   ^target
   |/
   L--------------------------
   Set the themperature to a certain value.
   Calculate next event time and set it.

   const uint8_t tempCurve[]={target,time,target,time};    
 */ 

uint8_t curveInd=0;       //operation index for tempCurve array
boolean weldCycle=0;      //indicate if weld cycle temp is active
const uint8_t tempCurve[]={     //target temp   //time to mantain temp
  150,      6,    //prehot
  155,      4,    //weld temp
  150,      5,          //weld temp 
  180                                 //temperature after welding
};    




void handleMCPInterrupt() {
  detachInterrupt(arduinoMCPInterrupt);
  //  if (millis()>lastMillisInterrupt+5) {
  //A    |¯¯|__|¯¯|__|¯
  //B  - __|¯¯|__|¯¯|__  +
  // Nel canale A pin D3 condensatore ceramico 103K 10nF
  uint8_t ActEnc=0;
  uint8_t intPin=contr.getLastInterruptPin();
  uint8_t valPin=contr.getLastInterruptPinValue();
  //Switch che  decide come gestire interrupt in base al pin che lo ha generato se A0 o A1 uint8_terpreta encoder altrimenti uint8_terpretazione come pulsante
  switch(intPin) {
    case EN_A:
    case EN_B:
      Pot=0;
      bitWrite(ActEnc, 0,contr.digitalRead(EN_A)); //Read current value of pin and set ActEnc var
      bitWrite(ActEnc, 1,contr.digitalRead(EN_B)); //Read current value of pin and set ActEnc var
      switch(ActEnc) {
        case 0: //00
          if(OldEnc==2) ticPot++;  //10
          if(OldEnc==1) ticPot--;  //01
          break;
        case 1: //01
          if(OldEnc==0) ticPot++;  //00
          if(OldEnc==3 ) ticPot--;  //11
          break;
        case 2: //10
          if(OldEnc==3) ticPot++;  //11
          if(OldEnc==0 ) ticPot--;  //00
          break;
        case 3: // 11
          if(OldEnc==1) ticPot++;  //01
          if(OldEnc==2 ) ticPot--;  //10
          break;
      }
      OldEnc=ActEnc;
      if (ticPot >= PotDivider) {
        Pot++;
        ticPot=0;
      } else if (ticPot <= -PotDivider)  {
        Pot--;
        ticPot=0;
      } 
      controllerEvent=CONTR_SCROLL;
      break;
    case EN_C:
      if (valPin) {
        //Serial.print("click ");
        controllerEvent=CONTR_CLICK;
        clickcount++;
        //Serial.println(clickcount);
      }
      break;
    default:
      if (millis() > lastMillisInterrupt+10) {
        //Serial.print("Button: A");
        //Serial.println(intPin);
        controllerEvent=CONTR_BTN;
#ifndef STARTSTOP
        if (intPin==4) {  //Use button as start welding controller
          StartStop=1;
        }                    
        if (intPin==6) {  //Use button as stop welding controller
          StartStop=0;
        }                    
#endif
      }
      break;
  }

  //  }


  lastMillisInterrupt=millis();
  cleanMCPInterrupts();
  //we set callback for the arduino INT handler.
  attachInterrupt(arduinoMCPInterrupt, MCPintCallBack, FALLING);
}


void MCPintCallBack() {     //Low priority interrupt, the callback simply set a variable for interrupt handling fuctions in main loop.
  awakenByMCPInterrupt = true;
}

void cleanMCPInterrupts() {     
  //EIFR = 0x01;
  awakenByMCPInterrupt = false;
}

void ZeroCCallBack() {      //High priority interrupt, only minimal operation and no time consumption routine
  detachInterrupt(arduinoZeroCInterrupt);
  sbi(PINB,5);              //Defined by macro on top for fast toggle pin D13 = DEBUGLED
  phaseCounter++;
  if (phaseCounter>=PidTime ) {
    opTime--;
    phaseCounter=0;
    DoPid=1;
  }
  /*  TCNT_timer=63450; // 2ms
    TCNT_timer=64000; // 4ms 
    TCNT_timer=65000; // 8ms 
    TCNT_timer=63000; //NC
    TCNT_timer=63200; // 645us
    TCNT_timer=63100; // 250us
    TCNT_timer=63080; // 150us
    TCNT_timer=63060; // 79us
   */
  //TCNT_timer=63060+Pid_Res;  

  TCNT1H = TCNT_timer >> 8;  
  TCNT1L = TCNT_timer & 0x00FF;
  TIMSK1 |= (1<<TOIE1);
  //we set callback for the arduino INT handler.
  attachInterrupt(arduinoZeroCInterrupt, ZeroCCallBack, RISING);
}

ISR(TIMER1_OVF_vect){ //timer1 overflow
  if (digitalRead(GATE)==0) {
    TCNT1H = 0xFF;  
    TCNT1L = 0xFF - PULSE;
    if (StartStop==1 && Pid_Res > 10 && AirFlow > D_AirFlowMin && TempGun!=0) { //Check if StartStop var is active > welding active and some controls to vars
      digitalWrite(GATE,HIGH); //turn on TRIAC gate
    }
  } else {
    digitalWrite(GATE,LOW); //turn off TRIAC gate
    TIMSK1 &= ~(1<<TOIE1);
  }
}



void modParMenu() {
  if (!initPar){            //First: set Modval with actual value of parameter
    switch (menu) {
      case 21:  ModVal  = TempGunApp; break;
      case 22:  ModVal  = AirFlowApp; break;
      case 51:  ModVal  = KP;   break;
      case 52:  ModVal  = KI;   break;
      case 53:  ModVal  = KD;   break;
      case 54:  ModVal  = MinT;   break;
      case 55:  ModVal  = MaxT;   break;
      case 101: ModVal  = AutoOffTime;  break;
    }  
    initPar=true; 
    contr.setCursor(0, 1);
    contr.print(ModVal);
  }else {                   //Second: check if value is valid
    ModVal=ModVal+Pot;
    switch (menu) {
      case 21:  if (ModVal  < 30) ModVal=30 ; if (ModVal  > D_MaxT) ModVal=D_MaxT ; break;
      case 22:  if (ModVal  < D_AirFlowMin) { ModVal= D_AirFlowMin; };  if (ModVal > D_AirFlowMax) {ModVal = D_AirFlowMax;}; break;
      case 51:  if (ModVal  < D_Min_Pid) { ModVal= D_Min_Pid; };  if (ModVal > D_Max_Pid) {ModVal = D_Max_Pid;}; break;
      case 52:  if (ModVal  < D_Min_Pid) { ModVal= D_Min_Pid; };  if (ModVal > D_Max_Pid) {ModVal = D_Max_Pid;}; break;
      case 53:  if (ModVal  < D_Min_Pid) { ModVal= D_Min_Pid; };  if (ModVal > D_Max_Pid) {ModVal = D_Max_Pid;}; break;
      case 54:  if (ModVal  < 30) ModVal=30 ; if (ModVal  > D_MaxT) ModVal=D_MaxT ; break;
      case 55:  if (ModVal  < 30) ModVal=30 ; if (ModVal  > D_MaxT) ModVal=D_MaxT ; break;
    }   
  } 
  contr.setCursor(5, 1);
  contr.print("==>");
  contr.setCursor(8, 1);
  contr.print("   ");
  contr.setCursor(8, 1);
  contr.print(ModVal);
}
void saveParMenu() {
  switch (menu) {   //Third: save valid modified parameter on eeprom
    case 31:  TempGunApp = ModVal;  EEPROM.write(M_Temp, TempGunApp);     EEPROM.write(M_Temp1, (TempGunApp >> 8)); break;          //Save long value  to two eeprom memory bytes.
    case 32:  AirFlowApp = ModVal;  EEPROM.write(M_AirFlow, AirFlowApp);  break;
    case 61:  KP  = ModVal; EEPROM.write(M_KP, KP);     break;
    case 62:  KI  = ModVal;   EEPROM.write(M_KI, KI);     break;
    case 63:  KD  = ModVal; EEPROM.write(M_KD, KD);     break;
    case 64:  MinT  = ModVal;   EEPROM.write(M_MinT, MinT);   break;
    case 65:  MaxT  = ModVal;     EEPROM.write(M_MaxT, MaxT);           EEPROM.write(M_MaxT1, (MaxT >> 8));         break;
    case 111:   AutoOffTime= ModVal;    EEPROM.write(M_AutoOffTime, AutoOffTime);       EEPROM.write(M_AutoOffTime1, (AutoOffTime >> 8)); break;  //Save long value  to two eeprom memory bytes.
  }  
  initPar=false;
}

void setMac() {
  switch (menu) {
    case 71:  TempGunApp = 300; AirFlowApp=80; break;           //Example for predefined temp and air flow for Sn
    case 72:  TempGunApp = 120; AirFlowApp=100; break;          //Example for predefined temp and air flow for a specific function heat shrink tubing
    case 73:  TempGunApp = 270; AirFlowApp=100; break;          //Example for predefined temp and air flow for a specific function weld LDPE 
    case 74:  TempGunApp = 300; AirFlowApp=100; break;          //Example for predefined temp and air flow for a specific function weld PP,Hard PVC, Hard PE
    case 75:  TempGunApp = 350;   AirFlowApp=100; break;          //Example for predefined temp and air flow for a specific function weld ABS, PC, Soft PVC

    case 81:  weldCycle=1;  break;           //Set weldCycle var now at every loop weldCurve routine is checked to the end of temperature weld cycle
  }  
  initPar=false;
}

void weldCurve() {
  if (curveInd%2==0) {               //If even element of array set temp
    TempGun = tempCurve[curveInd];
  } 
  if (curveInd%2==0 && ActTemp==TempGun) {        //If temp reached increment index
    curveInd++;
    opTime=tempCurve[curveInd];
  }
  if (ActTemp==TempGun && opTime<=0) {
    //Serial.println("Time reached");
    curveInd++;
  }
  if (curveInd>=sizeof(tempCurve)) {
    curveInd=0;
    weldCycle=0;
    menu=0;
    //Serial.println("Weld curve end");
  }
}


void PID (void)    //Controllo PID
{
  if (opTime & 1) ActTemp=TempC();
  Int_Res = Dev_Res = 0;
  Err1 = Err;
  Err = (TempGun-ActTemp);
  // Integrale
  SumE = SumE + Err;                      // SumE is the summation of the error terms
  if(SumE > SumE_Max)SumE = SumE_Max;
  if(SumE < SumE_Min)SumE = SumE_Min;

  //Int_Res = SumE / 10;                 // Ki*SumE/(Kp*Fs*X) where X is an unknown scaling factor
  Int_Res = SumE * KI;                   // combination of scaling factor and Kp
  //Int_Res = Int_Res ;// / 16;

  // Calculate the derivative term
  // Dev_Res = Err - Err1;
  /*if(Dev_Res > 120)Dev_Res = 120;
    if(Dev_Res < -120)Dev_Res = -120;*/

  Dev_Res =   KD*(Err - Err1);               // Derivative Kd(en0-en3)/(Kp*X*3*Ts)
  //Dev_Res = Dev_Res /2;

  if(Dev_Res > 120)Dev_Res = 120;
  if(Dev_Res < -120)Dev_Res = -120;


  // C(n) = K(E(n) + (Ts/Ti)SumE + (Td/Ts)[E(n) - E(n-1)])
  Pid_Res = Err + Int_Res + Dev_Res;        // Sum the terms
  Pid_Res = Pid_Res * KP>>1;                // multiply by Kp then scale
  if(Pid_Res> PidOutMax)  Pid_Res=PidOutMax;
  if(Pid_Res< PidOutMin)  Pid_Res=PidOutMin;
  TCNT_timer=63060+Pid_Res;
  last_PIDTime=millis();
}


byte TC_Read(void) { 
  int i;
  byte d = 0;

  for (i=7; i>=0; i--)
  {
    digitalWrite(SCK, LOW);
    _delay_ms(1);
    if (digitalRead(SO)) {
      //set the bit to 0 no matter what
      d |= (1 << i);
    }

    digitalWrite(SCK, HIGH);
    _delay_ms(1);
  }

  return d;
}


signed int TempC(){
  uint16_t v;

  digitalWrite(CS, LOW);
  _delay_ms(1);

  v = TC_Read();
  v <<= 8;
  v |= TC_Read();

  digitalWrite(CS, HIGH);

  if (v & 0x2) {
    //no MAX6675
    return -200; 
    //return -100;
  }

  if (v & 0x4) {
    //no thermocouple attached
    return -100; 
    //return -100;
  }

  v >>= 3;
  return v*0.25;
}

bool checkemptyeeprom() {
  int x=0;
  int param=0;
  int retval=0;
  for (x=0; x < EEPROM.length(); x++) {
    param=EEPROM.read(x);
    //Serial.println(param);
    if (param!=0) {
      retval=1;
    }
  }
  return retval;
}

bool checkerror() {
  if (ActTemp==-200) {
    contr.setCursor(0,1);
    contr.print("E: MAX6675 timeout");  
    Serial.println("Error: MAX6675 IC not responding!");
    return 1;
  } 
  if (ActTemp==-100) {
    contr.setCursor(0,1);
    contr.print("E: Temp sens. disc");  
    Serial.println("Error: Temperature sensor disconnected!");
    return 1;
  } 
  if (ActTemp>=MaxT) {
    contr.setCursor(0,1);
    contr.print("E: Temp > Max_T"); 
    Serial.println("Error: Temperature too high > MaxT!");
    return 1;
  }
  if (millis() > 3000 && millis()>=last_PIDTime+2000 && StartStop==1) {
    contr.setCursor(0,1);
    contr.print("E: Zero cross KO");  
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(last_PIDTime);
    Serial.print("\t");
    Serial.println("Error: Zero crossing circuit error!");
    return 1;
  } 

  return 0;
}

void startstop() {            //Handler for start/stop button
#ifdef STARTSTOP  
  StartStop=digitalRead(STARTSTOP);
#endif
  if (StartStop!=PrevStartStop) {     //StartStop button change, status check
    bitWrite(WeldStatus, 0,StartStop); 
    bitWrite(WeldStatus, 1,PrevStartStop); 
    switch(WeldStatus) {
      case 1: //01
        AirFlow=AirFlowApp;
        if (relayStartTime==0) {
          relayStartTime=millis()+2000;   //Calculate delay from now and relay activation
        }
        break;
      case 2: //10
        relayStartTime=0;   //Reset Relay activation time count
        TempGun=0;      //Set TempGun to 0 -> shutdown triac, do not change
        delay(150);
        digitalWrite(EMERG_RELAY,LOW);  //Disable power to gun
        if (ActTemp<D_MinT) {   //When weld cicle finish (button set to stop) wait for cold temperature and stop air flow
          AirFlow=0;
        }
        break;
    } 
    PrevStartStop=StartStop;
  }
  if (WeldStatus==1 && millis() > relayStartTime && digitalRead(TILTSENSOR)==0 ) {
    digitalWrite(EMERG_RELAY,HIGH);         //Enable power to gun
    TempGun=D_MinT;
    relayStartTime=0;
  }
  if (WeldStatus==1 && millis() > relayStartTime && digitalRead(TILTSENSOR)==1) {
    TempGun=TempGunApp;
  }
}

void setup() {
  Serial.begin(2000000);
  //Serial.print("Startup");
  contr.setMCPType(LTI_TYPE_MCP23017); 
  // set up the LCD's number of rows and columns:
  contr.begin(16, 2);
  // Print a message to the LCD.
  contr.clear();
  contr.setBacklight(HIGH);
  contr.setCursor(0, 0);
  contr.print("Hot Air Gun");
  contr.setCursor(0,1);
  contr.print("Wait....");
  if (checkemptyeeprom()==0) {
    ModVal=D_TempGunApp;
    menu=31;saveParMenu();
    ModVal=D_AirFlowApp;
    menu=32;saveParMenu();
    ModVal=KP;  
    menu=61;saveParMenu();
    ModVal=KI;  
    menu=62;saveParMenu();
    ModVal=KD;
    menu=63;saveParMenu();
    ModVal=D_MinT;
    menu=64;saveParMenu();
    ModVal=D_MaxT;
    menu=65;saveParMenu();
    ModVal=D_AutoOffTime;
    menu=111;saveParMenu(); 
    Serial.println("EEPROM empty! Saving default values");
  };
  TempGunApp = EEPROM.read(M_Temp1);
  TempGunApp = (TempGunApp << 8) + EEPROM.read(M_Temp);
  TempGun=0;
  AirFlowApp = EEPROM.read(M_AirFlow);
  //Serial.print("Setup AirFlow:");
  //Serial.println(AirFlow);
  KP = EEPROM.read(M_KP);
  KI = EEPROM.read(M_KI);
  KD = EEPROM.read(M_KD);
  //LcdCont=EEPROM.read(M_Cont);
  MinT = EEPROM.read(M_MinT);
  MaxT = EEPROM.read(M_MaxT1);
  MaxT = (MaxT << 8) + EEPROM.read(M_MaxT);
//  AutoOffTime = EEPROM.read(M_AutoOffTime1);
//  AutoOffTime = (AutoOffTime << 8) + EEPROM.read(M_AutoOffTime);
//  AutoOffTime * = -1;  //Convert to negative
  AutoOffTime=-30000;
  Serial.print("AOffTime");
  Serial.println(AutoOffTime);
  
  
  if (TempGunApp > D_MaxT) TempGunApp = D_MaxT;
  if (AirFlowApp < D_AirFlowApp) AirFlowApp = D_AirFlowApp;
  if (KP < D_Min_Pid) KP = D_Min_Pid;
  if (KP > D_Max_Pid) KP = D_Max_Pid;
  if (KI < D_Min_Pid) KI = D_Min_Pid;
  if (KI > D_Max_Pid) KI = D_Max_Pid;
  if (KD < D_Min_Pid) KD = D_Min_Pid;
  if (KD > D_Max_Pid) KD = D_Max_Pid;
  if (MinT < D_MinT) MinT = D_MinT;
  if (MaxT > D_MaxT) MaxT = D_MaxT;
  contr.setupEncoder(EN_A,EN_B,EN_C); // Encoder setup 
  uint8_t btnCross[BTN_NUM] = { BTN_1, BTN_2, BTN_3, BTN_4, BTN_5}; // Five button on cross disposition, setup function
  contr.setIntCross(btnCross, BTN_NUM);
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(ZeroCIntPin), ZeroCCallBack,  FALLING);
  attachInterrupt(digitalPinToInterrupt(MCPIntPin),   MCPintCallBack, FALLING);
  interrupts();
  handleMCPInterrupt();

  menu=MENU_HOME;

  pinMode(TILTSENSOR,   INPUT);
  pinMode(GATE,     OUTPUT);
  pinMode(EMERG_RELAY,  OUTPUT);
  pinMode(P_FAN_PWM,  OUTPUT);
#ifdef STARTSTOP  
  pinMode(STARTSTOP,  INPUT);
#endif  
  pinMode(DEBUGLED,   OUTPUT);

  //define pin modes MAX6675
  pinMode(CS, OUTPUT);
  pinMode(SCK, OUTPUT); 
  pinMode(SO, INPUT);
  digitalWrite(CS, HIGH);


  //Set TMR1 related registers, used for Triac driving
  //Useful info at http://forum.arduino.cc/index.php?topic=94100.0
  //TIMSK1 Timer Interrupt Mask Register
  //TOIE Overflow Interrupt disable during setup 
  TIMSK1 &= _BV(TOIE1);  
  //TCCR1A – Timer/Counter1 Control Register A
  //WGM11:0:  Waveform Generation Mode for  timer1
  //WGM10 = WGM11 = WGM12 = WGM13 = 0 ===>> "Normal mode, only count, no PWM, no fast PWM ecc"  
  TCCR1A &= ~(_BV(WGM11) | _BV(WGM10)); 
  //TCCR1B – Timer/Counter1 Control Register B
  TCCR1B &= ~(_BV(WGM12) | _BV(WGM13));  
  //OCIE1A Overflow Interrupt register
  TIMSK1 &= _BV(OCIE1A);  
  //CS11 Clock Select -> CS11 Clock quartz with prescaling 8
  TCCR1B |= (1<<CS11); 

  //Set TMR2 for PWM at 16 MHz, used for fan PWM control
  //TCCR1A – Timer/Counter1 Control Register A
  //COM2A1: COM2An: Compare Output Mode for Channel A on non-PWM mode (depend of  WGM2[2:0] bit). Clear OC2A on Compare Match in  
  //COM2B1: Compare Output Mode for Channel B. Clear OC2B on Compare Match.
  //WGM20: Waveform Generation Mode for timer2
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  //TCCR2B – Timer/Counter2 Control Register B
  //CS22:0: Clock Select -> CS22 Clock quartz/64 by prescaler. Required by 8 bit counter.
  TCCR2B = _BV(CS22);

}

void loop() {
//  Error and emergengy handling function
//  if (checkerror()==1) {
//    TempGun=0;
//    AirFlow=100;
//    digitalWrite(EMERG_RELAY,LOW);    //Disable power to gun
//    Serial.println("Shutdown after error!");
//    exit(0);
//  }
  if (DoPid) {          //Check if PID recalc is needed(recalc after 1 second) 

    PID();          
    DoPid=0;
    Serial.print(ActTemp);
    Serial.print("\t");
    Serial.print(TempGun);
    Serial.print("\t");
   Serial.println(Pid_Res);
//   Serial.print("\t");
//    Serial.print(KP,DEC);
//    Serial.print("\t");
//    Serial.println(KI,DEC);
//    Serial.print("\t");
//    Serial.println(KD,DEC);

  } else {
    startstop();
    if (awakenByMCPInterrupt) {
      handleMCPInterrupt();   //Handle low priority interrupt (user interface: encoder, switch) if fired
    } else {

      OCR2A =map(AirFlow,0,100,0,255);  //Adapt percent value to 0-255 scale for timer and apply to OCR2A register for pwm generation.

      if (menu!=menuold) {
        contr.clear();
        menudec=menu/10;
        menuunit=menu%10;
        contr.print(&menuvoice[menudec][menuunit][1]);  //Start printing lcd from second char to hide first menu control char
        menuold=menu;
      } else if (menu==0 && millis() > LcdUpd+1000) {         //Home menu special handler for live update var every second
        contr.clear();
        contr.print("T: ");
        contr.print(ActTemp);
        contr.print("/");
        contr.print(TempGun);
        contr.print(" A:");
        contr.print(AirFlow);
        contr.print("%");
        contr.setCursor(15, 0);
        if (StartStop==1) { contr.print("W");} else { contr.print("S");}
        contr.setCursor(0, 1);
        if (weldCycle>0) { contr.print("t:"); contr.print(opTime); } else {contr.print("o:"); contr.print(opTime);}
        LcdUpd=millis();
      } 

      if (menuunit==0 && menuvoice[menudec][menuunit]=="x" ){ //Menu title on click, next menu title Home -> Set Par -> Functions, if X skip to next level
        if (menudec < MENU_TITLES-2) {                  //Skip next level if not maximum level 
          menu=menu+10;
        } else {
          menu=MENU_HOME;         //if maximum level return to home
        }
      } 
      if (controllerEvent==CONTR_CLICK && menuvoice[menudec][menuunit][0]=='h'){ //On menuvoice=Exit goto menu title
        PotDivider=POTDIVIDER;          //Restore value encoder tick divider for sensibilty regulation (modified in m voice handler)
        saveParMenu();      //For specific voices save parameters on eeprom       
        menu=MENU_HOME;
        controllerEvent=CONTR_NOEVENT;
      } 
      if (controllerEvent==CONTR_CLICK && menuvoice[menudec][menuunit][0]=='a'){ //For first letter menu voice=a (apply predefined value) and go to home
        setMac();
        menu=MENU_HOME;
        controllerEvent=CONTR_NOEVENT;
      } 
      if (controllerEvent==CONTR_CLICK && menuvoice[menudec][menuunit][0]=='m'){ //For first letter menu voice=m (modify value) specific handler
        contr.setCursor(0, 1);
        contr.print("Scroll for mod");
      } 
      if (controllerEvent==CONTR_SCROLL && menuvoice[menudec][menuunit][0]=='m'){ //For first letter menu voice=m (modify value) specific handler
        PotDivider=1;                   //Set encoder divider for maximun sensibility, this speed value change (ie temp regulation)
        modParMenu();
        controllerEvent=CONTR_NOEVENT;
      } else if (controllerEvent==CONTR_CLICK && menudec <= MENU_TITLES-1 ){
        menu=menu+10;
        controllerEvent=CONTR_NOEVENT;
      } 


      if (controllerEvent==CONTR_SCROLL && menu > 0 ){
        if (menuvoice[menudec][menuunit][0]!='h' && Pot==+1) {   //scorre solo fino all'ultima voce del menu
          menu=Pot+menu;
        } else if (menuunit>0 && Pot==-1) {  //torna indietro ruotando la rotella solo però fino alla  prima voce
          menu=Pot+menu;
        }
        Pot=0;
        controllerEvent=CONTR_NOEVENT;
      };
      if (weldCycle>0) {  //Check if weld temperature cycle is active and then invoke it
        weldCurve();
      }

      if (opTime < AutoOffTime ) {  //Shutdown if inactive, other settings are required.....
        TempGun=0;
        TempGunApp=0;
        contr.setCursor(0, 1);
        contr.print("Auto Power OFF");
        AirFlow=100;
        AirFlowApp=100;
        //Serial.println("Auto Power OFF");
        if (ActTemp<=40) {
          AirFlow=0;    //when air on gun is lower than 40 degrees cut off pwn on fan
          OCR2A =0;
          Serial.println("Fan stop");
          delay(500);
          exit(0);
        }
      }
    }
  }
}
