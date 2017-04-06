// include the library code:
#include	<Wire.h>
#include	<LiquidCrystal_I2C.h>
#include	<EEPROM.h>
#include	<max6675.h>
#include	<PID_v1.h>
#include	"Arduino.h"


//Parameter definition
#define		D_MinT		100
#define		D_MaxT		400
#define		D_AirFlow	100
#define		D_AirFlowMin	40
#define		D_AirFlowMax	100
#define		D_LcdContMin	50
#define		D_LcdContMax	255
#define 	ParMax			6
#define		ParMin			0
#define		D_Min_Pid		0
#define		D_Max_Pid		100

#define		LCD_Update	100
#define		PID_Update	100

//EEPROM data storage
#define		M_Temp		2
#define		M_Temp1		3
#define		M_AirFlow	4
#define		M_KP		5
#define		M_KI		6
#define		M_KD		7
#define		M_MinT		8
#define		M_MaxT		9
#define		M_MaxT1		10

//Nano Pin Definition
//#define		CONTRAST_PIN	9
//#define		BACKLIGHT_PIN	7
#define		Fase0			  2
#define		Encoder_A		3
#define		Encoder_B		4
#define		EncClick		A2
#define 	MAX6675_DO		11
#define 	MAX6675_CLK		12
#define 	MAX6675_CS		13
#define		P_Start			25
#define 	P_Stop			26
#define   LedV        6

//LCD iterface definition
#define I2C_ADDR    0x4E
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7


//Variabili
int	TempGun	= 0;
int	AirFlow	= 0;
int MaxT = 0;
int MinT = 30;
int LcdCont = 0;

int ActTemp, PidOut;
unsigned long time;
char KI, KD, KP;
double DActTemp, DPidOut , DTempGun;
char LcdPar, HLcdPar;
int Pot, Page, LcdPot, HLcdPot;
char ActEnc, OldEnc;
char Time1, Time2;
boolean LcdUpd, PIDUpd;
boolean ModPar, MModPar, GetVal;
boolean MPStart, MPStop, StartCiclo;
boolean state;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
MAX6675 thermocouple(MAX6675_CLK, MAX6675_CS, MAX6675_DO);
PID PID(&DActTemp, &DPidOut, &DTempGun, KP, KI, KD, DIRECT);


void debug() {
  //Serial.print("OutPid");
  //Serial.println(DPidOut);
  Serial.print("ModPar");
  Serial.println(ModPar);
  Serial.print("GetVal");
  Serial.println(GetVal);
  Serial.print("LcdPar");
  Serial.println(LcdPar, DEC);
  Serial.print("Pot");
  Serial.println(Pot);
  //Serial.print("xx");
  //Serial.println(xx);
}

void setup() {
  //pinMode(Fase0, INPUT_PULLUP);
  pinMode(Encoder_A, INPUT_PULLUP);
  pinMode(Encoder_B, INPUT_PULLUP);
  pinMode(EncClick, INPUT_PULLUP);
  pinMode(P_Start, INPUT_PULLUP);
  pinMode(P_Stop, INPUT_PULLUP);
  pinMode(LedV, OUTPUT);

  lcd.begin(16, 2);
  lcd.backlight();
  delay(2000);
  lcd.setBacklight(BACKLIGHT_OFF);
  delay(2000);
  lcd.setBacklight(BACKLIGHT_ON);

  lcd.setCursor(3, 0);
  lcd.print("HotAirGun V2");
  delay(1000);


  TempGun = EEPROM.read(M_Temp1);
  TempGun = (TempGun << 8) + EEPROM.read(M_Temp);
  AirFlow = EEPROM.read(M_AirFlow);
  KP = EEPROM.read(M_KP);
  KI = EEPROM.read(M_KI);
  KD = EEPROM.read(M_KD);
  //LcdCont=EEPROM.read(M_Cont);
  MinT = EEPROM.read(M_MinT);
  MaxT = EEPROM.read(M_MaxT1);
  MaxT = (MaxT << 8) + EEPROM.read(M_MaxT);

  lcd.clear();

  if (TempGun < D_MinT) TempGun = D_MinT;
  if (TempGun > D_MaxT) TempGun = D_MaxT;
  if (AirFlow < D_AirFlow) AirFlow = D_AirFlow;
  if (KP < D_Min_Pid) KP = D_Min_Pid;
  if (KP > D_Max_Pid) KP = D_Max_Pid;
  if (KI < D_Min_Pid) KI = D_Min_Pid;
  if (KI > D_Max_Pid) KI = D_Max_Pid;
  if (KD < D_Min_Pid) KD = D_Min_Pid;
  if (KD > D_Max_Pid) KD = D_Max_Pid;
  if (MinT < D_MinT) MinT = D_MinT;
  if (MaxT > D_MaxT) MaxT = D_MaxT;

  if (LcdCont < D_LcdContMin) LcdCont = D_LcdContMin;
  Serial.begin ( 57600 );

  //attachInterrupt(0,INT_0 , FALLING);
  attachInterrupt(1, INT_1 , CHANGE);

  PID.SetMode(AUTOMATIC);
  PID.SetOutputLimits(-255, 255);
  PID.SetSampleTime(10);

  lcd.setCursor(0, 1);
  HLcdPar = 1;
}

void loop()
{
  if (millis() > time + 1000) {
    debug();
    time = millis();
  }

  PID.Compute();
  LcdUpd++;       //Solo per debug1

  if (LcdUpd) {
    lcd.setCursor(0, 0);
    lcd.print("Act Temp C");
    lcd.setCursor(12, 0);
    lcd.print(25, DEC);
    //lcd.print(thermocouple.readCelsius(),DEC);
  }
  if (!digitalRead(EncClick)) MModPar = 1;
  if (MModPar & digitalRead(EncClick)) {
    MModPar = 0;
    ModPar = !ModPar;

  }
  if (LcdPar < ParMin) Pot = ParMin;
  if (LcdPar > ParMax) Pot = ParMax * 4;
  if (HLcdPar != LcdPar) {
    lcd.setCursor(0, 1);
    lcd.print("                ");				//Pulisce lo schermo

    switch (LcdPar) {
      case 0:	lcd.setCursor(0, 1);	lcd.print("Air Temp");	lcd.setCursor(12, 1); lcd.print(TempGun, DEC);	break;
      case 1:	lcd.setCursor(0, 1);	lcd.print("Air Flow");	lcd.setCursor(12, 1); lcd.print(AirFlow, DEC);	break;
      case 2:	lcd.setCursor(0, 1);	lcd.print("KP");		    lcd.setCursor(12, 1); lcd.print(KP, DEC);		    break;
      case 3:	lcd.setCursor(0, 1);	lcd.print("KI");		    lcd.setCursor(12, 1); lcd.print(KI, DEC);		    break;
      case 4:	lcd.setCursor(0, 1);	lcd.print("KD");		    lcd.setCursor(12, 1); lcd.print(KD, DEC);       break;
      case 5:	lcd.setCursor(0, 1);	lcd.print("Min Temp");	lcd.setCursor(12, 1); lcd.print(MinT, DEC); 	  break;
      case 6:	lcd.setCursor(0, 1);	lcd.print("Max Temp");	lcd.setCursor(12, 1); lcd.print(MaxT, DEC);		  break;
    }
    HLcdPar = LcdPar;
  }

  if (ModPar) {
    if (!GetVal) {

      switch (LcdPar) {
        case 0:	Pot = TempGun;	break;
        case 1:	Pot = AirFlow;	break;
        case 2:	Pot = KP;		break;
        case 3:	Pot = KI;		break;
        case 4:	Pot = KD;		break;
        case 5:	Pot = MinT;	break;
        case 6:	Pot = MaxT;	break;
      }
      GetVal = 1;
    }

    switch (LcdPar) {
      case 0:	if (LcdPot < MinT)LcdPot = MinT;            if (LcdPot > MaxT)LcdPot = MaxT;	                    break;
      case 1:	if (LcdPot < D_AirFlowMin)LcdPot = D_AirFlowMin; if (LcdPot > D_AirFlowMax)LcdPot = D_AirFlowMax;	break;
      case 2:	if (LcdPot < D_Min_Pid)LcdPot = D_Min_Pid;  if (LcdPot > D_Max_Pid)LcdPot = D_Max_Pid;          	break;
      case 3:	if (LcdPot < D_Min_Pid)LcdPot = D_Min_Pid;  if (LcdPot > D_Max_Pid)LcdPot = D_Max_Pid;	          break;
      case 4:	if (LcdPot < D_Min_Pid)LcdPot = D_Min_Pid;  if (LcdPot > D_Max_Pid)LcdPot = D_Max_Pid;	          break;
      case 5:	if (LcdPot < D_MinT)LcdPot = D_MinT;        if (LcdPot > D_MaxT)LcdPot = D_MaxT;	                break;
      case 6:	if (LcdPot < D_MinT)LcdPot = D_MinT;        if (LcdPot > D_MaxT)LcdPot = D_MaxT;	                break;
    }

    if (HLcdPot != LcdPot) {
      lcd.setCursor(12, 1);
      lcd.print("    ");
      lcd.setCursor(12, 1);
      lcd.print(LcdPot);
    }
    HLcdPot = LcdPot;
  }

  if ((!ModPar) & GetVal) {

    switch (LcdPar) {
      case 0: TempGun = LcdPot;	EEPROM.write(M_Temp, TempGun); EEPROM.write(M_Temp1, (TempGun >> 8));	break;
      case 1: AirFlow = LcdPot;	EEPROM.write(M_AirFlow, AirFlow); break;
      case 2: KP = LcdPot;		  EEPROM.write(M_KP, KP);			      break;
      case 3: KI = LcdPot;	  	EEPROM.write(M_KI, KI);			      break;
      case 4: KD = LcdPot;		  EEPROM.write(M_KD, KD);			      break;
      case 5: MinT = LcdPot;	  EEPROM.write(M_MinT, MinT);		    break;
      case 6: MaxT = LcdPot;    EEPROM.write(M_MaxT, MaxT);   EEPROM.write(M_MaxT1, (MaxT >> 8));     break;
    }
    Pot = LcdPar * 4;
    GetVal = 0;
  }

  if (!ModPar) LcdPar = Pot / 4;
  else LcdPot = Pot;

  if (!digitalRead(P_Start)) MPStart = 1;
  if (digitalRead(P_Start)& MPStart) {
    MPStart = 0;
    StartCiclo = 1;
  }
  if (!digitalRead(P_Stop)) MPStop = 1;
  if (digitalRead(P_Stop)& MPStop) {
    MPStop = 0;
    StartCiclo = 0;
  }

}
