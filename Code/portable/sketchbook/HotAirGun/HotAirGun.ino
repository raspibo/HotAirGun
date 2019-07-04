

// include the library code:
#include <Wire.h>
#include <CtrlPanel.h>
#include <EEPROM.h>

//Debug
#define SerialPlot
//#define F_Debug			//Uncomment to enable the "fast debug" use for event
#define S_Debug			//Uncomment to enable the schedule debug
#define TDebug		1000

//Parameter definition
#define D_MinT		200	//Default minimal temperature 
#define D_MaxT		450     //Default max temperature (remember.... max 450!)
#define D_KP		2
#define D_KI		5
#define D_KD		7
#define D_TempGunApp	250
#define D_AirFlowApp	100
#define	D_AirFlowMin	40	//Default minimal air flow
#define	D_AirFlowMax	100	//Default minimal air flow
#define	D_Min_Pid	0
#define	D_Max_Pid	250
#define D_WeldCurv	1
#define D_Target1   	200	//Default target temp for weld curve
#define D_Time1		20	//Default target temp for weld curve
#define D_Target2   	250	//Default target temp for weld curve
#define D_Time2		20	//Default target temp for weld curve
#define	D_AutoOffTime	32000	//Default off timer 


//#define   LCD_Update	100	//UNUSED? LCD refresh interval
//#define   PID_Update	100

#define   PidTime	100	//Period 1 sec 60Hz		100	/Period 1S 50Hz
#define   TStop		50	//Safe temp to disable GUN AirFlow

//#define   SumE_Min      -1000	//UNUSED? 
//#define   SumE_Max      1000	//UNUSED? 
#define   Pid_Out_Min	0	//Minimum limit for Pid calculation
#define   Pid_Out_Max	2000 ////1200 //Maximum limit for Pid calculation

//EEPROM data storage
#define   M_Temp	2
#define   M_Temp1	3
#define   M_AirFlow	4
#define   M_KP		5
#define   M_KI		6
#define   M_KD		7
#define   M_MinT	8
#define   M_MaxT	9
#define   M_MaxT1	10
#define   M_AutoOffTime	11
#define   M_AutoOffTime1	12
#define   M_WeldCurv	13
#define   M_Target1		14
#define   M_Target11	15
#define   M_Time1		16
#define   M_Target2		17
#define   M_Target21	18
#define   M_Time2		19



//I2cController vars definition
CtrlPanelLib contr(0x20);	// Connect via i2c, default address #0 (A0-A2 not jumpered)
#define POTDIVIDER		4	// Encoder tick divider for sensibilty regulation
#define CONTR_NOEVENT	0	// No event detected
#define CONTR_SCROLL	1	// Encoder movement detected -Z > read Pot value

uint8_t ControllerEvent = CONTR_NOEVENT; //Store encoder events
int TicPot = 0;           		//Used for cont of single step of rotary encoder
int PotDivider = POTDIVIDER;    	//Divider for TicPot for best regulation of sensitivity
int Pot , PotOld;
char ActEnc, OldEnc;   			//Diff between encoder positions
uint8_t PortA, IntA;
int ModVal, OldModVal;     				//Temporary var to store parameter during set
boolean InitPar = false;    		//Used by Menus
boolean EncClick, P1, P2, P3; //P4, P5;

//Pin definition
#define ZEROCINTPIN 2	//Interrupt pin used by zero crossing detector circuit DO NOT CHANGE
#define MCPINTPIN 	3	//Interrupt pin used by I2C controller (MCP23107)      DO NOT CHANGE
#define SO			5	//MAX6675 signal serial out aka MISO on SPI
#define CS			6	//MAX6675 signal Chip select
#define SCK			7	//MAX6675 signal clock
#define GATE		9 	//TRIAC gate
#define EMERG_RELAY 8	//Emergency and power relay
#define P_FAN_PWM	11  //Define pin D10 for pwm signal for gun
#define _STARTSTOP	12  //Pin used for activation of hot air production (also used for magnetic sensor on gun)
#define DEBUGLED	13  //Led used for debug purpose
#define HOLDER_SENS A3	//Tilt sensor for gun
#define FOOT_SWITCH A4	// Switch to anable the welding curve


//PWM and timers vars
#define PULSE	0x0F      //trigger pulse width (counts)
#define sbi(port,bit) (port)|=(1<<(bit))  //Fast toggle routine for pins

//Interrupts vars
byte MCPIntPin = MCPINTPIN; // Interrupts from the MCP will be handled by this PIN on Arduino
byte ArdMCPInterrupt = 1; // ... and this interrupt vector
byte ZeroCIntPin = ZEROCINTPIN; // Interrupts from zero crossing circuit
byte ArdZeroCInterrupt = 0; // ... and this interrupt vector

volatile boolean AwakenByMCPInterrupt   = false;
uint8_t PhaseCounter = 0; //increment at every phase zero crossing at 100 (=1 sec)  increment OpTime

//Timing vars
long	LcdUpd;           //Last time lcd was updated for real time Menu refresh es Home
long	RelayStartTime = 0;  //Relay time start: wait some time after start button press
int		AutoOffTime;    //Time for auto shutdown in seconds
int		OpTime = 0;   //operation Timer normally equal to millis(), used for temperature welding curve time calculation, displayed in home Menu
long	LastPIDTime = 0;  //Last time PID was calculated
long	TimeDeb;

//PID vars
char KI, KD, KP;    //Used by PID
bool DoPid = 0;   //Set to true if it's time to recalculate PID
long  SumE, Int_Res, Dev_Res, Err;//Used by PID
signed long Pid_Res;    //Used by PID
uint16_t TCNT_timer = 0;  //Calculate PID controller pulse width, used by interrupt

//Temp, Air, status vars
int LastActTemp = 0;
int ActTemp = 0;    //Actual gun air temp
int TempGunApp = 0; //Target temperature for PID temporary variable when weld cycle is not active
int TempGun = 0;    //Target temperature for PID, at work the air flow with this temp from gun
int AirFlowApp = 0; //Temporary variable for air flow when weld cycle is not active
int AirFlow = 0;
int MaxT = 0;
int MinT = 30;
boolean StartStop, PrevStartStop;
uint8_t WeldStatus;

//WeldCycle vars
//uint8_t CurveInd = 0;     //operation index for TempCurve array
boolean WeldCycle = 0;    //indicate if weld cycle temp is active
int WeldCurv, Target1, Time1, Target2, Time2;
long TimeStabl, TimeBuzz, TimeRamp, TimeTarg1, TimeTarg2;
bool TempInDelt, StartWlC, LiftUp;
int Delta;
#define Del_WlC	3	//Store ActTemp - TempGun on weld curve cycle
#define TBuzz	1000	//Buzzer time duration ms
#define TRamp	300

int X = 0;

uint8_t GG;

//Menu and display vars
#define Menu_HOME 0
#define Menu_TITLES 5
#define TimeLcd 1000
int Menu = Menu_HOME;
int MenuDec, MenuUnit;
int MenuOld = -1;
bool EditMode, EditPar, SaveConf, SavePar, SetMacEn;

/*
   First char convention for Menu voice handling:
   v=scroll -> next voice on right or left, click -> next voice down (default)
   h=goto home on click
   a=apply and go to home
*/
const char *MenuVoice[Menu_TITLES][10] =
{
  {"nFastSet", "vAirTemp", "vAirFlow", "hExit"},           //0,1,2..
  {"nParSet", "vKP", "vKI", "vKD", "vMinT", "vMaxT", "hExit"}, //10,11,12..
  {"nMaterials Presets", "aSn", "aheat shrink", "aLDPE", "aPP/Hard PVC/HDPE", "aABS/PC/Soft PVC", "hExit"}, //20..
  {"nWeldCurve", "aVeldCycle", "vTempCurve", "vTarget1", "vTime1", "vTarget2", "vTime2", "hExit"},                    //30..
  {"nFunct", "vAutoOff", "aDefault Setting", "hExit"},          //40..
};

/*

   TempCurve var

   |           _______________<Time2
   |         / ^Target2        \
   |  ______/<Time1             \
   | /   ^Target1
   |/
   L-----------------------------
   Set the themperature to a certain value.
   Calculate next event time and set it.

   const uint8_t TempCurve[]={target,time,target,time};
*/

/*
const uint8_t TempCurve[] = {   //target temp   //time to mantain temp
  150,      6,    //prehot
  155,      4,    //weld temp
  150,      5,    //weld temp
  180             //temperature after welding
};
*/

void HandleMCPInterrupt() {
  detachInterrupt(ArdMCPInterrupt);
  //A    |¯¯|__|¯¯|__|¯
  //B    ¯|__|¯¯|__|¯¯|__  +
  AwakenByMCPInterrupt = false;
  //uint8_t PortA = 0;
  PortA = contr.readRegister(MCP23017_GPIOA);
  IntA =  contr.readRegister(MCP23017_INTCAPA);
  ActEnc = ((PortA & 0b110) >> 1 );

  //Pot = 0;
  AwakenByMCPInterrupt = false;
  //PortA = contr.readRegister(MCP23017_GPIOA);

  ActEnc = ((PortA & 0b110) >> 1 );
  switch (ActEnc) {
    case 0: //00
      if (OldEnc == 2) TicPot++; //10
      if (OldEnc == 1) TicPot--; //01
      break;
    case 1: //01
      if (OldEnc == 0) TicPot++; //00
      if (OldEnc == 3 ) TicPot--; //11
      break;
    case 2: //10
      if (OldEnc == 3) TicPot++; //11
      if (OldEnc == 0 ) TicPot--; //00
      break;
    case 3: // 11
      if (OldEnc == 1) TicPot++; //01
      if (OldEnc == 2 ) TicPot--; //10
      break;
  }
  if (OldEnc != ActEnc) ControllerEvent = CONTR_SCROLL;
  OldEnc = ActEnc;
  if (TicPot >= PotDivider) {
    Pot++;
    TicPot = 0;
  } else if (TicPot <= -PotDivider)  {
    Pot--;
    TicPot = 0;
  }
  if (!bitRead(IntA, 0) && bitRead(PortA, 0)) EncClick = 1;	else EncClick = 0;
  if (!bitRead(IntA, 3) && bitRead(PortA, 3)) StartStop = 1; 					//P1 = 1;			else P1 = 0;
  if (!bitRead(IntA, 5) && bitRead(PortA, 5)) {StartStop = 0; StartWlC = 0;}	//P2 = 1;			else P2 = 0;
  if (!bitRead(IntA, 7) && bitRead(PortA, 7)) StartWlC = 1;						//else P3 = 0;
  if (StartWlC) {																//Disable panel navigation during the WLC cycle
	  Pot = 0;
	  EncClick = 0;
  }
  //cleanMCPInterrupts();
  //we set callback for the arduino INT handler.
  OpTime = 0;
  attachInterrupt(ArdMCPInterrupt, MCPintCallBack, FALLING);
}

void MCPintCallBack() {     //Low priority interrupt, the callback simply set a variable for interrupt handling fuctions in main loop.
  AwakenByMCPInterrupt = true;
}

void ZeroCCallBack() {      //High priority interrupt, only minimal operation and no time consumption routine
  detachInterrupt(ArdZeroCInterrupt);
  sbi(PINB, 5);             //Defined by macro on top for fast toggle pin D13 = DEBUGLED
  PhaseCounter++;
  if (PhaseCounter >= PidTime ) {
    OpTime--;
    PhaseCounter = 0;
    DoPid = 1;
  }
  /*
    TCNT_timer=65413; // 500us
    TCNT_timer=65288; // 1000us
    TCNT_timer=65038; // 2ms
    TCNT_timer=64538; // 4ms
    TCNT_timer=63538; // 8ms
  */
  TCNT1H = TCNT_timer >> 8;
  TCNT1L = TCNT_timer & 0x00FF;
  TIMSK1 |= (1 << TOIE1);
  //we set callback for the arduino INT handler.
  attachInterrupt(ArdZeroCInterrupt, ZeroCCallBack, RISING);
}

ISR(TIMER1_OVF_vect) { //timer1 overflow
  if (digitalRead(GATE) == 0) {
    TCNT1H = 0xFF;
    TCNT1L = 0xFF - PULSE;
    if (StartStop == 1 && Pid_Res > 10 && AirFlow > D_AirFlowMin && TempGun != 0) { //Check if StartStop var is active > welding active and some controls to vars
      digitalWrite(GATE, HIGH); //turn on TRIAC gate
    }
  } else {
    digitalWrite(GATE, LOW); //turn off TRIAC gate
    TIMSK1 &= ~(1 << TOIE1);
  }
}

#if defined S_Debug
void Debug() {
  GG = ~GG;
  contr.WriteLed(LED_4, GG);
  //Serial.println(Menu,DEC);
  //Serial.println(WeldCycle,DEC);
  //digitalWrite(LedV, !(digitalRead(LedV)));
  // Serial.println("Debug Programma");
  // Serial.print("MenuDec");
  // Serial.println(MenuDec);
  // Serial.print("MenuUnit");
  // Serial.println(MenuUnit );
  // Serial.print("Pot");
  // Serial.println(Pot);

  // Serial.print(KP, DEC);
  // Serial.print("\t");
  // Serial.println(KI, DEC);
  // Serial.print("\t");
  // Serial.println(KD, DEC);
}
#endif

void ModParMenu() {
  if (!InitPar) {           //First: set Modval with actual value of parameter
    switch (Menu) {
      case 1:   ModVal = TempGunApp;	break;
      case 2:   ModVal = AirFlowApp;	break;
      case 11:  ModVal = KP;			break;
      case 12:  ModVal = KI;			break;
      case 13:  ModVal = KD;			break;
      case 14:  ModVal = MinT;			break;
      case 15:  ModVal = MaxT;			break;
      case 32:  ModVal = WeldCurv;		break;
      case 33:  ModVal = Target1;		break;
      case 34:  ModVal = Time1;			break;
      case 35:  ModVal = Target2;		break;
      case 36:  ModVal = Time2;			break;	  
      case 41:  ModVal = AutoOffTime;	break;
    }
    InitPar = true;
    OldModVal = ModVal + 1;
    contr.setCursor(0, 1);
    contr.print(ModVal);
    //contr.setCursor(3, 1);
    contr.print(" ==>");
  }
  else {                   //Second: check if value is valid
    if (Menu == 41) ModVal = ModVal + (Pot * 100);
    else ModVal = ModVal + Pot;
    Pot = 0;
    switch (Menu) {
	case 1:   if (ModVal < 30) ModVal = 30;				if (ModVal  > D_MaxT) ModVal = D_MaxT;				break;
	case 2:   if (ModVal < D_AirFlowMin) ModVal = D_AirFlowMin; 	if (ModVal > D_AirFlowMax) ModVal = D_AirFlowMax;		break;
	case 11:  if (ModVal < D_Min_Pid)  ModVal = D_Min_Pid;		if (ModVal > D_Max_Pid) ModVal = D_Max_Pid;			break;
	case 12:  if (ModVal < D_Min_Pid)  ModVal = D_Min_Pid;		if (ModVal > D_Max_Pid) ModVal = D_Max_Pid;			break;
	case 13:  if (ModVal < D_Min_Pid)  ModVal = D_Min_Pid;		if (ModVal > D_Max_Pid) ModVal = D_Max_Pid;			break;
	case 14:  if (ModVal < 30) ModVal = 30 ;			if (ModVal  > D_MaxT) ModVal = D_MaxT;				break;
	case 15:  if (ModVal < 30) ModVal = 30 ;			if (ModVal  > D_MaxT) ModVal = D_MaxT;				break;
	case 32:  if (ModVal < 1) ModVal = 1;				if (ModVal  > 2) ModVal = 2;					break;
	case 33:  if (ModVal < 30) ModVal = 30;				if (ModVal  > D_MaxT) ModVal = D_MaxT;				break;
	case 34:  if (ModVal < 1) ModVal = 1;				if (ModVal  > 250) ModVal = 250;				break;
	case 35:  if (ModVal < 30) ModVal = 30;				if (ModVal  > D_MaxT) ModVal = D_MaxT;				break;
	case 36:  if (ModVal < 1) ModVal = 1;				if (ModVal  > 250) ModVal = 250;				break;
	case 41:  if (ModVal < -32000) ModVal = -32000;			if (ModVal  > -100) ModVal = -100;				break;
    }
  }
  if (OldModVal != ModVal) {
    contr.setCursor(7, 1);
    contr.print("   ");
    contr.setCursor(8, 1);
    contr.print(ModVal);
    OldModVal = ModVal;
  }
}

void SaveParMenu() {
  switch (Menu) {   //Third: save valid modified parameter on eeprom
    case 1:  TempGunApp = ModVal;  EEPROM.write(M_Temp, TempGunApp);     EEPROM.write(M_Temp1, (TempGunApp >> 8)); break;          //Save long value  to two eeprom memory bytes.
    case 2:  AirFlowApp = AirFlow = ModVal;	EEPROM.write(M_AirFlow, AirFlowApp);		break;
    case 11:  KP  = ModVal;		EEPROM.write(M_KP, KP);					break;
    case 12:  KI  = ModVal;		EEPROM.write(M_KI, KI);					break;
    case 13:  KD  = ModVal;		EEPROM.write(M_KD, KD);					break;
    case 14:  MinT  = ModVal;		EEPROM.write(M_MinT, MinT);				break;
    case 15:  MaxT  = ModVal;		EEPROM.write(M_MaxT, MaxT); EEPROM.write(M_MaxT1, (MaxT >> 8));		    break;
    case 32:  WeldCurv = ModVal;  EEPROM.write(M_WeldCurv  , WeldCurv);				break;
    case 33:  Target1 = ModVal;  EEPROM.write(M_Target1  , Target1);     EEPROM.write(M_Target11 , (Target1 >> 8)); break;
    case 34:  Time1  = ModVal;		EEPROM.write(M_Time1, Time1);				break;
    case 35:  Target2 = ModVal;  EEPROM.write(M_Target2  , Target2);     EEPROM.write(M_Target21 , (Target2 >> 8)); break;
    case 36:  Time2  = ModVal;		EEPROM.write(M_Time2, Time2);				break;
    case 41:  AutoOffTime = ModVal;	EEPROM.write(M_AutoOffTime, AutoOffTime); EEPROM.write(M_AutoOffTime1, (AutoOffTime >> 8)); AutoOffTime = 1 - AutoOffTime; break; //Save long value  to two eeprom memory bytes.
  }
  InitPar = false;
}

void SetMac() {
  switch (Menu) {
    case 21:  TempGunApp = 300; AirFlowApp = 80;	break;         //Example for predefined temp and air flow for Sn
    case 22:  TempGunApp = 120; AirFlowApp = 100;	break;        //Example for predefined temp and air flow for a specific function heat shrink tubing
    case 23:  TempGunApp = 270; AirFlowApp = 100;	break;        //Example for predefined temp and air flow for a specific function weld LDPE
    case 24:  TempGunApp = 300; AirFlowApp = 100; 	break;        //Example for predefined temp and air flow for a specific function weld PP,Hard PVC, Hard PE
    case 25:  TempGunApp = 350; AirFlowApp = 100; 	break;        //Example for predefined temp and air flow for a specific function weld ABS, PC, Soft PVC
    case 31:  WeldCycle = ~ WeldCycle;  		break;        //Set WeldCycle var now at every loop weldCurve routine is checked to the end of temperature weld cycle
    case 42:  DefVal(); 				break;
  }
  InitPar = false;
}

void weldCurve() {
 //Verifica stabiità temperature 
 Delta = abs( ActTemp - TempGun);
  if ( millis()> TimeStabl) TempInDelt =1; 
  if (Delta < Del_WlC) {
    if (!TimeStabl && !StartWlC) TimeStabl = (millis() + 5000);
	else if (!TimeStabl) TimeStabl = (millis() + 2000);
   }
  else { TimeStabl = 0;  TempInDelt = 0;}
 if (!StartWlC && StartStop ) {
	 TempGunApp = Target1;
	 if( TempInDelt && ( millis() > TimeBuzz )) {
		  contr.buzz(200, 1500);
		  TimeBuzz = millis()+ TBuzz;
	    }	
	}
 if (StartWlC){
	 if (!TimeTarg1) TimeTarg1 = ( millis() + (Time1 *1000));
	 if (millis() > TimeTarg1 && !LiftUp ) {
		 LiftUp = 1;
		 contr.buzz(200, 1500);
	 }	 
	 if(( TempGunApp < Target2) && LiftUp){
		if(( WeldCurv == 1) && TempInDelt ) {
		  TempGunApp++;
		  contr.buzz(100, 1500);
		}
	    if(( WeldCurv == 2) && (millis() > TimeRamp)){
		  TempGunApp++;
		  contr.buzz(100, 1500);
		  TimeRamp = millis() + TRamp;
	    }
	 }
	 //Check weldcurve end monitoring target2 temperature and time2 time then reset all vars
	 if (TempGunApp == Target2){
		 if ( ! TimeTarg2) TimeTarg2 = millis()+(Time2 * 1000);
		 if (millis() > TimeTarg2){
			 StartStop = 0;
			 StartWlC =0;
			 LiftUp = 0;
			 TempInDelt = 0;
			 WeldCycle=0;
			 TimeStabl = TimeBuzz = TimeRamp = TimeTarg1 = TimeTarg2 = 0 ;
			 TempGunApp = EEPROM.read(M_Temp1);
             TempGunApp = (TempGunApp << 8) + EEPROM.read(M_Temp);
			 contr.buzz(500, 1500);
			}
		}
	}
}

void PID (void){    //Controllo PID
  //if (OpTime & 1) ActTemp = TempC();
  ActTemp = TempC();
  //Int_Res = Dev_Res = 0;
  Err += (TempGun - ActTemp);
  if (Err > Pid_Out_Max / KP) Err = Pid_Out_Max / KP;
  else if (Err < Pid_Out_Min) Err = Pid_Out_Min;

  Int_Res += (KI * Err);
  if (Int_Res > Pid_Out_Max / KI) Int_Res = Pid_Out_Max / KI;
  else if (Int_Res < Pid_Out_Min) Int_Res = Pid_Out_Min;
  int Dev_Res = (ActTemp - LastActTemp);

  Pid_Res = KP * Err + Int_Res - KD * Dev_Res;
  if (Pid_Res > Pid_Out_Max) Pid_Res = Pid_Out_Max;
  else if (Pid_Res < Pid_Out_Min) Pid_Res = Pid_Out_Min;

  LastActTemp = ActTemp;

  //TCNT_timer = 63060 + Pid_Res;
  TCNT_timer = 63400 + Pid_Res;
  LastPIDTime = millis();
}

byte TC_Read(void) {
  int i;
  byte d = 0;
  for (i = 7; i >= 0; i--) {
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

signed int TempC() {
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
  return v * 0.25;
}

bool checkemptyeeprom() {
  int x = 0;
  int param = 0;
  int retval = 0;
  for (x = 0; x < EEPROM.length(); x++) {
    param = EEPROM.read(x);
    if (param != 0) {
      retval = 1;
    }
  }
  return retval;
}

bool checkerror() {
  if (ActTemp == -200) {
    contr.setCursor(0, 1);
    contr.print("E: MAX6675 timeout");
    Serial.println("Error: MAX6675 IC not responding!");
    return 1;
  }
  if (ActTemp == -100) {
    contr.setCursor(0, 1);
    contr.print("E: Temp sens. disc");
    Serial.println("Error: Temperature sensor disconnected!");
    return 1;
  }
  if (ActTemp >= MaxT) {
    contr.setCursor(0, 1);
    contr.print("E: Temp > Max_T");
    Serial.println("Error: Temperature too high > MaxT!");
    return 1;
  }
  if (millis() > 3000 && millis() >= LastPIDTime + 2000 && StartStop == 1) {
    contr.setCursor(0, 1);
    contr.print("E: Zero cross KO");
#if defined F_Debug
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(LastPIDTime);
    Serial.print("\t");
    Serial.println("Error: Zero crossing circuit error!");
#endif
    return 1;
  }
  return 0;
}

void startstop() {            //Handler for start/stop button
#ifdef STARTSTOP
  StartStop = digitalRead(STARTSTOP);
#endif
  if (StartStop != PrevStartStop) {   //StartStop button change, status check
    bitWrite(WeldStatus, 0, StartStop);
    bitWrite(WeldStatus, 1, PrevStartStop);
    switch (WeldStatus) {
      case 1: //01
        //AirFlow = AirFlowApp;
        AirFlow = 100;
        if (RelayStartTime == 0) {
          RelayStartTime = millis() + 2000; //Calculate delay from now and relay activation
        }
        break;
      case 2: //10
        RelayStartTime = 0; //Reset Relay activation time count
        TempGun = TStop;    //Set TempGun to 0 -> shutdown triac, do not change
        delay(150);
        digitalWrite(EMERG_RELAY, LOW); //Disable power to gun
        AirFlow = 100;
        if (ActTemp < TStop) { //When weld cicle finish (button set to stop) wait for cold temperature and stop air flow
          AirFlow = 0;
	  TempGun = 0;
        }
        break;
    }
    PrevStartStop = StartStop;
  }
  if(WeldStatus == 1 && millis() > RelayStartTime){
	  if(!digitalRead(EMERG_RELAY)){
		  digitalWrite(EMERG_RELAY, HIGH);
		  delay(100);
	    }
	  if (!digitalRead(HOLDER_SENS)) {
		  digitalWrite(EMERG_RELAY, HIGH);        //Enable power to gun
		  TempGun = D_MinT;
		  RelayStartTime = 0;
		  AirFlow = AirFlowApp;
	    }
	  if (digitalRead(HOLDER_SENS)) {
		  TempGun = TempGunApp;
          AirFlow = AirFlowApp;
	    }
    }
  if (WeldStatus == 2 && ActTemp < TStop) {
	AirFlow = 0;
	TempGun = 0;
  }
}

void DefVal() {
  ModVal = D_TempGunApp;
  Menu = 1; SaveParMenu();
  ModVal = D_AirFlowApp;
  Menu = 2; SaveParMenu();
  ModVal = D_KP;
  Menu = 11; SaveParMenu();
  ModVal = D_KI;
  Menu = 12; SaveParMenu();
  ModVal = D_KD;
  Menu = 13; SaveParMenu();
  ModVal = D_MinT;
  Menu = 14; SaveParMenu();
  ModVal = D_MaxT;
  Menu = 15; SaveParMenu();
  ModVal = D_WeldCurv;
  Menu = 32; SaveParMenu();
  ModVal = D_Target1;
  Menu = 33; SaveParMenu();
  ModVal = D_Time1;
  Menu = 34; SaveParMenu();
  ModVal = D_Target2;
  Menu = 35; SaveParMenu();
  ModVal = D_Time2;
  Menu = 36; SaveParMenu();
  ModVal = D_AutoOffTime;
  Menu = 41; SaveParMenu();
}

void setup() {
  //#ifdef F_Debug || defined S_Debug || SerialPlot
  Serial.begin(2000000);
  Serial.print("Startup");
 // #endif					// set up the LCD's number of rows and columns:
  contr.begin(16, 2);									// Print a message to the LCD.
  contr.clear();
  contr.setBacklight(HIGH);
  contr.setCursor(0, 0);
  contr.print("Hot Air Gun");
  contr.setCursor(0, 1);
  contr.print(String(__DATE__));
  contr.print("Wait....");
  contr.buzz(500, 1000);
  if (checkemptyeeprom() == 0) {
    DefVal();
#if defined F_Debug
    Serial.println("EEPROM empty! Saving default values");
#endif
  };
  TempGunApp = EEPROM.read(M_Temp1);
  TempGunApp = (TempGunApp << 8) + EEPROM.read(M_Temp);
  TempGun = 0;
  AirFlowApp = EEPROM.read(M_AirFlow);
#if defined F_Debug
  //Serial.print("Setup AirFlow:");
  //Serial.println(AirFlow);
#endif
  KP = EEPROM.read(M_KP);
  KI = EEPROM.read(M_KI);
  KD = EEPROM.read(M_KD);
  //LcdCont=EEPROM.read(M_Cont);
  MinT = EEPROM.read(M_MinT);
  MaxT = EEPROM.read(M_MaxT1);
  MaxT = (MaxT << 8) + EEPROM.read(M_MaxT);
  WeldCurv = EEPROM.read(M_WeldCurv);
  Target1 = EEPROM.read(M_Target11);
  Target1 = (Target1 << 8) + EEPROM.read(M_Target1);
  Time1 = EEPROM.read(M_Time1);
  Target2 = EEPROM.read(M_Target21);
  Target2 = (Target2 << 8) + EEPROM.read(M_Target2);
  Time2 = EEPROM.read(M_Time2);
  
  //  AutoOffTime = EEPROM.read(M_AutoOffTime1);
  //  AutoOffTime = (AutoOffTime << 8) + EEPROM.read(M_AutoOffTime);
  //  AutoOffTime = 1 - AutoOffTime;  //Convert to negative
  AutoOffTime = -30000;
#if defined F_Debug
  Serial.print("AOffTime");
  Serial.println(AutoOffTime);
#endif
  if (TempGunApp > D_MaxT) TempGunApp = D_MaxT;
  if (AirFlowApp < D_AirFlowMin) AirFlowApp = D_AirFlowApp;
  if (KP < D_Min_Pid) KP = D_Min_Pid;
  if (KP > D_Max_Pid) KP = D_Max_Pid;
  if (KI < D_Min_Pid) KI = D_Min_Pid;
  if (KI > D_Max_Pid) KI = D_Max_Pid;
  if (KD < D_Min_Pid) KD = D_Min_Pid;
  if (KD > D_Max_Pid) KD = D_Max_Pid;
  if (MinT < D_MinT)  MinT = D_MinT;
  if (MaxT > D_MaxT)  MaxT = D_MaxT;
  //contr.setupEncoder(EN_A,EN_B,EN_C); // Encoder setup
  //uint8_t btnCross[BTN_NUM] = { BTN_1, BTN_2, BTN_3, BTN_4, BTN_5}; // Five button on cross disposition, setup function
  //contr.setIntCross(btnCross, BTN_NUM);
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(ZeroCIntPin), ZeroCCallBack,  FALLING);
  attachInterrupt(digitalPinToInterrupt(MCPIntPin),   MCPintCallBack, FALLING);
  //interrupts();
  HandleMCPInterrupt();
  Menu = Menu_HOME;
  pinMode(HOLDER_SENS, INPUT_PULLUP);
  pinMode(GATE, OUTPUT);
  pinMode(EMERG_RELAY, OUTPUT);
  pinMode(P_FAN_PWM, OUTPUT);
#ifdef STARTSTOP
  pinMode(STARTSTOP, INPUT);
#endif
  pinMode(DEBUGLED, OUTPUT);
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
  TCCR1B |= (1 << CS11);
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
  //  if (checkerror()==1) {                  //  Error and emergengy handling function
  //    TempGun=0;
  //    AirFlow=100;
  //    digitalWrite(EMERG_RELAY,LOW);    //Disable power to gun
  //   #if defined F_Debug
  //   Serial.println("Shutdown after error!");
  //   #endif
  //   exit(0);
  //  }
  if (DoPid) {          //Check if PID recalc is needed(recalc after 1 second)
    PID();
    DoPid = 0;
#if defined SerialPlot
    Serial.print(ActTemp);
    Serial.print("\t");
    Serial.print(TempGun);
    //Serial.print("\t");
    // Serial.print(Pid_Res);
    // Serial.print("\t");
    // Serial.print(Err);
    // Serial.print("\t");
    // Serial.print(Int_Res);
    // Serial.print("\t");
    // Serial.print(Dev_Res);
    //Serial.println("\t");
    //Serial.print( Delta);
    Serial.println("\t");
#endif
  }
  startstop();
  if (AwakenByMCPInterrupt) {
    HandleMCPInterrupt();   //Handle low priority interrupt (user interface: encoder, switch) if fired
  }
  OCR2A = map(AirFlow, 0, 100, 0, 255); //Adapt percent value to 0-255 scale for timer and apply to OCR2A register for pwm generation.
#if defined S_Debug
  if (millis() > TimeDeb) {
    TimeDeb = (millis() + TDebug);
    Debug();
  }
#endif
  if (EditMode && (Menu != MenuOld)) {
    if (MenuOld != 100) contr.clear();
    contr.setCursor(0, 0);
    MenuDec = Menu / 10;
    MenuUnit = Menu % 10;
    if (EditPar && !SavePar) contr.print("Mod ");
    if (SaveConf) contr.print("Save ");
    contr.print(&MenuVoice[MenuDec][MenuUnit][1]);  //Start printing lcd from second char to hide first Menu control char
    MenuOld = Menu;
  }
  else if ( !EditMode && millis() > LcdUpd ) {    //Home Menu special handler for live update var every second
    contr.clear();
    contr.print("T:");
    contr.print(ActTemp);
    contr.print("/");
    contr.print(TempGun);
    contr.setCursor(10, 0);
    contr.print("A:");
    contr.print(AirFlow);
    contr.print("%");
    contr.setCursor(12, 1);
    if (StartStop == 1) {
	if (WeldCycle) contr.print("WelC");
	else contr.print("Weld");
    }
    else {
      contr.print("Stop");
    }
    contr.setCursor(0, 1);
    if (WeldCycle > 0) {
      contr.print("o:");
      contr.print(OpTime);
    }
    else {
      contr.print("t:");
      contr.print(OpTime);
    }
    LcdUpd = millis() + TimeLcd;
  }

  if (EncClick && MenuVoice[MenuDec][MenuUnit][0] == 'n') { //On MenuVoice=Exit goto Menu tilet`
    if (MenuDec < Menu_TITLES - 1) {              //Skip next level if not maximum level
      Menu = Menu + 10;
      if (!EditMode) {
        EditMode = 1;
        Menu = Menu_HOME;
        MenuOld = 1;
        PotDivider = 4;
      }
    }
    else  {
      Menu = Menu_HOME;       //if maximum level return to home
      EditMode = 0;
      MenuDec = MenuUnit = 0;
    }
    EncClick = 0;
  }

  if (ControllerEvent == CONTR_SCROLL && EditMode && !EditPar && !SaveConf ) {
    if (MenuVoice[MenuDec][MenuUnit][0] != 'h' && Pot == +1) { //scorre solo fino all'ultima voce del Menu
      Menu = Pot + Menu;
    }
    else if (MenuUnit > 0 && Pot == -1) { //torna indietro ruotando la rotella solo però fino alla  prima voce
      Menu = Pot + Menu;
    }
    Pot = 0;
    ControllerEvent = CONTR_NOEVENT;
  };

  if (EncClick && !EditPar && !SaveConf  && MenuVoice[MenuDec][MenuUnit][0] == 'v') { //On MenuVoice=Exit goto Menu tilet`
    EditPar = 1;
    EncClick = 0;
#ifdef F_Debug
    Serial.print("Edit");
#endif
    MenuOld = 100;
  }

  if (EncClick && !SaveConf && MenuVoice[MenuDec][MenuUnit][0] == 'v') { //On MenuVoice=Exit goto Menu tilet`
    //SavePar = 1;
    SaveConf = 1;
    EditPar = 0;
    EncClick = 0;
    PotDivider = 3;
    Pot = 2;
    PotOld = 0;
#ifdef F_Debug
    Serial.print("Save");
#endif
    MenuOld = 100;
  }

  if ( EditPar) {
    ModParMenu();
    PotDivider = 1;
  }

  if (SaveConf) {
    if (Pot != PotOld) {
      contr.setCursor(13, 1);
      if (Pot > 1) {
        contr.print("YES");
        Pot = 2;
      }
      if (Pot < -1) {
        contr.print("NO ");
        Pot = -2;
      }
      PotOld = Pot;
    }

    if (EncClick && !SetMacEn && Pot > 1) {
      SaveParMenu();
      //Menu = MenuDec = MenuUnit = 0;

      //EditMode = EditPar = SaveConf = 0;
      EditPar = SavePar = 0;
      EncClick = 0;
      SaveConf = 0;
      InitPar = 0;
      MenuOld = 0;
      contr.setCursor(0, 1);
      for (char i = 0; i <= 16; i++) contr.print(" ");
      contr.buzz(200, 1500);
    }

    if (EncClick && SetMacEn && Pot > 1) {
      SetMac();
      Menu = MenuDec = MenuUnit = 0;
      SetMacEn = 0;
      SaveConf = 0;
      EncClick = 0;
      EditMode = 0;
      contr.buzz(200, 1500);
    }

    if (EncClick && Pot < -1) {
      //SaveParMenu();
      //Menu = MenuDec = MenuUnit = 0;
      EditPar = SavePar = 0;
      EncClick = 0;
      SaveConf = 0;
      InitPar = 0;
      MenuOld = 0;
      contr.setCursor(0, 1);
      for (char i = 0; i <= 16; i++) contr.print(" ");
      contr.buzz(500, 200);
    }
  }

  if (EncClick && !SaveConf && MenuVoice[MenuDec][MenuUnit][0] == 'a') { //On MenuVoice=Exit goto Menu tilet`
    contr.setCursor(0, 1);
    contr.print("Are You Sure");
    SaveConf = 1;
    EncClick = 0;
    PotDivider = 3;
    Pot = 2;
    PotOld = 0;
    SetMacEn = 1;
  }
  if (EncClick && !SaveConf && MenuVoice[MenuDec][MenuUnit][0] == 'h') { //On MenuVoice=Exit goto Menu tilet`
    Menu = MenuDec = MenuUnit = 0;
    EditMode = EditPar = SaveConf = 0;
    EncClick = 0;
  }


  if (WeldCycle > 0) { //Check if weld temperature cycle is active and then invoke it
    weldCurve();
  }

  if (OpTime < AutoOffTime ) {  //Shutdown if inactive, other settings are required.....
    TempGun = 0;
    TempGunApp = 0;
    contr.setCursor(0, 1);
    contr.print("Auto Power OFF");
    AirFlow = 100;
    AirFlowApp = 100;
    //Serial.println("Auto Power OFF");
    if (ActTemp <= 40) {
      AirFlow = 0;  //when air on gun is lower than 40 degrees cut off pwn on fan
      OCR2A = 0;
      Serial.println("Fan stop");
      delay(500);
      exit(0);
    }
  }
}
