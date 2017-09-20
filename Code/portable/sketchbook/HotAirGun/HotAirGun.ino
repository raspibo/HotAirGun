// include the library code:
#include <Wire.h>
#include <I2cController.h>
#include <EEPROM.h>

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
#define		D_AutoOffTime	-3600

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
#define		M_AutoOffTime   11	
#define		M_AutoOffTime1  12	

#define EN_A 1 // Encored scroll A0
#define EN_B 2 // Encoder scroll A1 
#define EN_C 0 // Encoder click  A2
#define POTDIVIDER 4 //Encoder tick divider for sensibilty regulation
#define BTN_NUM 5
#define BTN_1 3 // Single button A3
#define BTN_2 4 // Single button A4
#define BTN_3 5 // Single button A5
#define BTN_4 6 // Single button A6
#define BTN_5 7 // Single button A7


#define CONTR_NOEVENT  0 // No event detected
#define CONTR_SCROLL 1 // Encoder movement detected -Z > read Pot value
#define CONTR_CLICK  2 // Encoder click detected 
#define CONTR_BTN    3 // Button pressed

#define DEBUGLED     13 //Led used for debug purpose
#define DEBUGPIN     12 //Pin used for debug purpose with logic analyzer
#define sbi(port,bit) (port)|=(1<<(bit))
int lstate = 0;

// Connect via i2c, default address #0 (A0-A2 not jumpered)
I2cControllerLib contr(0x20);
uint8_t controllerEvent=CONTR_NOEVENT;
uint8_t encoderPos=0;
uint8_t clickButton;
int clickcount=0;
int ticPot=0;       //Used for cont of single step of rotary encoder 
int PotDivider=POTDIVIDER;   //Divider for ticPot for best regulation of sensitivity
int Pot=0;
int But;
char ActEnc, OldEnc;


// Interrupts from the MCP will be handled by this PIN on Arduino
byte MCPIntPin = 3;
// ... and this intterrupt vector
byte arduinoMCPInterrupt = 1;

// Interrupts from zero crossing circuit
byte ZeroCIntPin = 2;
// ... and this intterrupt vector
byte arduinoZeroCInterrupt = 0;

volatile boolean awakenByMCPInterrupt   = false;
unsigned long lastMillisInterrupt = 0;

uint8_t LcdPot, HLcdPot;
unsigned long LcdUpd;            //Last time lcd was updated for real time menu refresh es Home 
boolean PIDUpd;
boolean ModPar, MModPar, GetVal;

int MaxT = 0;
int MinT = 30;
int AutoOffTime;

char KI, KD, KP;
double DActTemp, DPidOut , DTempGun;
char LcdPar, HLcdPar;
char Time1, Time2;
boolean MPStart, MPStop, StartCiclo;
boolean state;

#define MENU_HOME 0
#define MENU_TITLES 12 
boolean initPar=false;

int ActTemp=145;		//Actual gun air temp
int TempGun=251;	//Target temperature for PID, at work the air flow with this temp from gun
int AirFlow=100;

int menu=MENU_HOME;
int menudec,menuunit;
int menuold=-1;
int ModVal;

int opTime=0;		//operation Timer normally equal to millis(), used for temperature welding curve time calculation, displayed in home menu
int nextEventTime=0;
int actTime=0;
uint8_t curveInd=0;     //operation index for tempCurve array
boolean weldCycle=0;    //indicate if weld cycle temp is active

int X=0;
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
	{"nHome"},													//00
	{"nSetFast"  		,"vAirTemp"	,"vAirFlow"	,"hExit"},							//10,11,12..
	{"x"        		,"mModAirTemp"	,"mModAirFlow"	},								//20,21,22..
	{"x"        		,"hSaveAirTemp"	,"hSaveAirFlow"	},								//30,31,32...
	{"nSetPar"   		,"vKP"		,"vKI"		,"vKD"		,"vMinT"		,"vMaxT"		,"hExit"},	//40,41..
	{"x"        		,"mMod KP"	,"mMod KI"	,"mMod KD"	,"mMod MinT"		,"mMod MaxT"		},		//50,51,52.....
	{"x"        		,"hSave KP"	,"hSave KI"	,"hSave KD"	,"hSave MinT"		,"hSave MaxT"		},		//60...
	{"nMaterials Presets"	,"aSn"		,"aheat shrink"	,"aLDPE"	,"aPP/Hard PVC/HDPE"	,"aABS/PC/Soft PVC"	,"hExit"},	//70..	
	{"nWeldCurve"		,"aTempCurve"	,"hExit"},											//80..	
	{"nFunct"		,"vAutoOff"	,"vPPPreset"   	,"vDefault"	,"hExit"},							//90..	
	{"x"			,"mAutoOffTime"	,"mPPPreset"	,"mSure?"},									//100..
	{"x"			,"hSaveAutoOff"	,"hSavePPPreset","hSaveDefault"},								//110..
};

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

const uint8_t tempCurve[]={     //target temp   //time to mantain temp
				150,			6,		//prehot
				155,			4,		//weld temp
				150,			5,	        //weld temp	
				180	                                //temperature after welding
			  };    


char menuDisplay[16] = "";


void handleMCPInterrupt() {
	detachInterrupt(arduinoMCPInterrupt);
//	if (millis()>lastMillisInterrupt+5) {
		//A    |¯¯|__|¯¯|__|¯
		//B  - __|¯¯|__|¯¯|__  +
		// Nel canale A pin D3 condensatore ceramico 103K 10nF
		uint8_t ActEnc=0;
		uint8_t uint8_tPin=contr.getLastInterruptPin();
		uint8_t valPin=contr.getLastInterruptPinValue();
		//Serial.print("uint8_tPin: ");
		//Serial.print(uint8_tPin);
		//Switch che  decide come gestire uint8_terrupt in base al pin che lo ha generato se A0 o A1 uint8_terpreta encoder altrimenti uint8_terpretazione come pulsante
		switch(uint8_tPin) {
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
//				Serial.print("ticPot: ");
//				Serial.println(ticPot);
//				Serial.print("Pot: ");
//				Serial.println(Pot);
				if (ticPot >= PotDivider) {
					Pot++;
					ticPot=0;
				} else if (ticPot <= -PotDivider)  {
					Pot--;
					ticPot=0;
				} 
//				Serial.print("Scaled ticPot: ");
//				Serial.println(ticPot);
				controllerEvent=CONTR_SCROLL;
				break;
			case EN_C:
				if (valPin) {
					Serial.print("click ");
					controllerEvent=CONTR_CLICK;
					clickcount++;
					Serial.println(clickcount);
				}
				break;
			default:
				if (millis() > LcdUpd+1000) {
				Serial.print("Button: A");
				Serial.println(uint8_tPin);
				controllerEvent=CONTR_BTN;
				if (intPin==4) { //Only for debug purpose simulate temp change
					ActTemp--;   //Only for debug purpose simulate temp change
				}                    //Only for debug purpose simulate temp change
                                if (intPin==6) { //Only for debug purpose simulate temp change
                                        ActTemp++;   //Only for debug purpose simulate temp change
                                }                    //Only for debug purpose simulate temp change
				}
				break;
		}

//	}


	lastMillisInterrupt=millis();
	cleanMCPInterrupts();
	//we set callback for the arduino INT handler.
	attachInterrupt(arduinoMCPInterrupt, MCPintCallBack, FALLING);
}


void MCPintCallBack() {
	awakenByMCPInterrupt = true;
}

void cleanMCPInterrupts() {
	//EIFR = 0x01;
	awakenByMCPInterrupt = false;
}

void ZeroCCallBack() {
	detachInterrupt(arduinoZeroCInterrupt);
	sbi(PINB,5);           		//Defined by macro on top for fast toggle pin D13 = DEBUGLED
	//we set callback for the arduino INT handler.
	attachInterrupt(arduinoZeroCInterrupt, ZeroCCallBack, FALLING);
}


void modParMenu() {
	if (!initPar){		        //First: set Modval with actual value of parameter
		switch (menu) {
			case 21:	ModVal  = TempGun;	break;
			case 22:	ModVal  = AirFlow;	break;
			case 51:	ModVal  = KP;		break;
			case 52:	ModVal  = KI;		break;
			case 53:	ModVal  = KD;		break;
			case 54:	ModVal  = MinT;		break;
			case 55:	ModVal  = MaxT;		break;
			case 101:	ModVal  = AutoOffTime;	break;
		}  
		initPar=true; 
		contr.setCursor(0, 1);
		contr.print(ModVal);
	}else {		                //Second: check if value is valid
		ModVal=ModVal+Pot;
		switch (menu) {
			case 21:	if (ModVal  < 30) ModVal=30 ; if (ModVal  > D_MaxT) ModVal=D_MaxT ;	break;
			case 22:	if (ModVal  < D_AirFlowMin) { ModVal= D_AirFlowMin; };  if (ModVal > D_AirFlowMax) {ModVal = D_AirFlowMax;}; break;
			case 51:	if (ModVal  < D_Min_Pid) { ModVal= D_Min_Pid; };  if (ModVal > D_Max_Pid) {ModVal = D_Max_Pid;}; break;
			case 52:	if (ModVal  < D_Min_Pid) { ModVal= D_Min_Pid; };  if (ModVal > D_Max_Pid) {ModVal = D_Max_Pid;}; break;
			case 53:	if (ModVal  < D_Min_Pid) { ModVal= D_Min_Pid; };  if (ModVal > D_Max_Pid) {ModVal = D_Max_Pid;}; break;
			case 54:	if (ModVal  < 30) ModVal=30 ; if (ModVal  > D_MaxT) ModVal=D_MaxT ;	break;
			case 55:	if (ModVal  < 30) ModVal=30 ; if (ModVal  > D_MaxT) ModVal=D_MaxT ;	break;
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
		switch (menu) {		//Third: save valid modified parameter on eeprom
			case 31: 	TempGun = ModVal;	EEPROM.write(M_Temp, TempGun); 		EEPROM.write(M_Temp1, (TempGun >> 8));	break;
			case 32: 	AirFlow = ModVal;	EEPROM.write(M_AirFlow, AirFlow); 	break;
			case 61: 	KP 	= ModVal;	EEPROM.write(M_KP, KP);			break;
			case 62: 	KI 	= ModVal;  	EEPROM.write(M_KI, KI);			break;
			case 63: 	KD 	= ModVal;	EEPROM.write(M_KD, KD);			break;
			case 64: 	MinT 	= ModVal;  	EEPROM.write(M_MinT, MinT);		break;
			case 65: 	MaxT 	= ModVal;   	EEPROM.write(M_MaxT, MaxT);   				EEPROM.write(M_MaxT1, (MaxT >> 8));     		break;
			case 111: 	AutoOffTime= ModVal;   	EEPROM.write(M_AutoOffTime, AutoOffTime);   		EEPROM.write(M_AutoOffTime1, (AutoOffTime >> 8));	break;
		}  
		initPar=false;
}

void setMac() {
		switch (menu) {
			case 71: 	TempGun = 300;	AirFlow=80; break;           //Example for predefined temp and air flow for Sn
			case 72: 	TempGun = 120;	AirFlow=100; break;          //Example for predefined temp and air flow for a specific function heat shrink tubing
			case 73: 	TempGun = 270;	AirFlow=100; break;          //Example for predefined temp and air flow for a specific function weld LDPE 
			case 74: 	TempGun = 300;	AirFlow=100; break;          //Example for predefined temp and air flow for a specific function weld PP,Hard PVC, Hard PE
			case 75:	TempGun = 350;  AirFlow=100; break;          //Example for predefined temp and air flow for a specific function weld ABS, PC, Soft PVC
			
			case 81:	weldCycle=1;	break;			     //Set weldCycle var now at every loop weldCurve routine is checked to the end of temperature weld cycle
		}  
		initPar=false;
		}

void weldCurve() {
			if (curveInd%2==0) {					     //If even element of array set temp
				TempGun = tempCurve[curveInd];
			} 
			if (curveInd%2==0 && ActTemp==TempGun) { 		    //If temp reached increment index
				curveInd++;
				opTime=tempCurve[curveInd];
			}
			if (ActTemp==TempGun && opTime<=0) {
				Serial.println("Time reached");
				curveInd++;
			}
			if (curveInd>=sizeof(tempCurve)) {
				curveInd=0;
				weldCycle=0;
				menu=0;
				Serial.println("Weld curve end");
			}
		}

void setup() {
	Serial.begin(115200);
	Serial.print("Startup");
	contr.setMCPType(LTI_TYPE_MCP23017); 
	// set up the LCD's number of rows and columns:
	contr.begin(16, 2);
	// Pruint8_t a message to the LCD.
	contr.clear();
	contr.setBacklight(HIGH);
	contr.setCursor(0, 0);
	contr.print("Hot Air Gun");
	contr.setCursor(0,1);
	contr.print("Wait....");
	TempGun = EEPROM.read(M_Temp1);
	TempGun = (TempGun << 8) + EEPROM.read(M_Temp);
	AirFlow = EEPROM.read(M_AirFlow);
	Serial.print("Setup AirFlow:");
	Serial.println(AirFlow);
	KP = EEPROM.read(M_KP);
	KI = EEPROM.read(M_KI);
	KD = EEPROM.read(M_KD);
	//LcdCont=EEPROM.read(M_Cont);
	MinT = EEPROM.read(M_MinT);
	MaxT = EEPROM.read(M_MaxT1);
	MaxT = (MaxT << 8) + EEPROM.read(M_MaxT);
	AutoOffTime = EEPROM.read(M_AutoOffTime1);
	AutoOffTime = (AutoOffTime << 8) + EEPROM.read(M_AutoOffTime);
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

	contr.setupEncoder(EN_A,EN_B,EN_C); // Encoder setup 
	uint8_t btnCross[BTN_NUM] = { BTN_1, BTN_2, BTN_3, BTN_4, BTN_5}; // Five button on cross disposition, setup function
	contr.setIntCross(btnCross, BTN_NUM);
	delay(1000);
	attachInterrupt(digitalPinToInterrupt(ZeroCIntPin), ZeroCCallBack,  FALLING);
	attachInterrupt(digitalPinToInterrupt(MCPIntPin),   MCPintCallBack, FALLING);
	interrupts();
	handleMCPInterrupt();
	
	menu=MENU_HOME;
	pinMode(DEBUGLED, OUTPUT);
	pinMode(DEBUGPIN, OUTPUT);
}

void loop() {

	if (awakenByMCPInterrupt) handleMCPInterrupt();		//Handle low priority interrupt (user interface: encoder, switch) if fired

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
		contr.setCursor(0, 1);
		if (weldCycle>0) { contr.print("t:"); contr.print(opTime); }
                opTime--;
		LcdUpd=millis();
	} 

	if (menuunit==0 && menuvoice[menudec][menuunit]=="x" ){ //Menu title on click, next menu title Home -> Set Par -> Functions, if X skip to next level
		if (menudec < MENU_TITLES-2) {                  //Skip next level if not maximum level 
			menu=menu+10;
		} else {
			menu=MENU_HOME;					//if maximum level return to home
		}
	} 
	if (controllerEvent==CONTR_CLICK && menuvoice[menudec][menuunit][0]=='h'){ //On menuvoice=Exit goto menu title
		PotDivider=POTDIVIDER;	        //Restore value encoder tick divider for sensibilty regulation (modified in m voice handler)
		saveParMenu();			//For specific voices save parameters on eeprom				
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

	if (opTime<D_AutoOffTime) {  //Shutdown if inactive, other settings are required.....
		TempGun=0;
		contr.setCursor(0, 1);
		contr.print("Auto Power OFF");
	}
}
