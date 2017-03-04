/******************************************************************************\
* Hot Air Welder  Programs control											   *
* Mcu  Pic 16F876 to 4 Mhz,  pic run the PID control to control the air        *
* temperature to the target also the PWM to control the fan speed / ari flow   *
\******************************************************************************/
// Lcd pinout settings
sbit LCD_RS at RA0_bit;
sbit LCD_EN at RA1_bit;
sbit LCD_D7 at RA5_bit;
sbit LCD_D6 at RA4_bit;
sbit LCD_D5 at RA3_bit;
sbit LCD_D4 at RA2_bit;

// Pin direction
sbit LCD_RS_Direction at TRISA0_bit;
sbit LCD_EN_Direction at TRISA1_bit;
sbit LCD_D7_Direction at TRISA5_bit;
sbit LCD_D6_Direction at TRISA4_bit;
sbit LCD_D5_Direction at TRISA3_bit;
sbit LCD_D4_Direction at TRISA2_bit;

#define FaseDetect  PORTB.F0   // Interrupt RB0 Detect the 0 point on 220V
#define Heater      PORTB.F1   // Output for the SCR
#define Start       PORTB.F6   // Start Button
#define Stop        PORTB.F2   // Stop Button
#define Fase_A      PORTB.F4   // Warming!!!! Encoder signal Pull-Up
#define Fase_B      PORTB.F5   // Warming!!!! Encoder signal Pull-Up
#define SwitchEnc   PORTB.F3
#define GunStand    PORTB.F7

#define CS          PORTC.F0   // For Max6675
#define PWMFan      PORTC.F1   // PWM2
#define CSK         PORTC.F2   // For Max6675
#define SO          PORTC.F3   // For Max6675
//#define PORTC.F
#define LedR        PORTC.F5   //System Run
#define LedV        PORTC.F6   // Interrupt Access
//#define         PORTC.F7

//Bit Programma
#define Edit       VarBool.F0
#define MemSw      VarBool.F1
#define StartCicle VarBool.F2
#define DoPid      VarBool.F3
#define Tc_Allarm  VarBool.F4
#define Time_Stop  VarBool.F5
//#define VarBool.F6
//#define VarBool.F7


// Costanti Programma
#define Eprom_Add 0
#define Min_Row   10
#define Max_Row   11
#define Min_Temp  100
#define Max_Temp  400
#define TempStop  40
#define Min_Flow  1
#define Max_Flow  100
#define PotScale  1
#define PidTime   100      //Period 1 sec
#define TStop     25
#define Kp           1
#define Ki           2
#define Kd           6
#define SumE_Min     -1200
#define SumE_Max     1200
#define PidOutMin    0
#define PidOutMax    1200


//Pid Var
signed int  SumE, Int_Res, Dev_Res, Err, Err1;
signed long Pid_Res;
// Program Var
char VarBool;
unsigned int PulseTime;
unsigned char Stop_Loop;
char PidCount;
//Menu & Parameter
signed char Pot;
char Act_PortB, Old_PortB;		//Dig Pot
unsigned char LcdRow,LcdRowOld,Loop;
unsigned int TempGun,ActTemp; 	
unsigned char AirFlow;			



//Lcd Mess
const char Logo[]=   "Msystem";
const char Logo1[]=  "HotAirGun V1.0";
const char StatMen[]="Act Temp  C";
const char Menu1[]=  "TempGun   C";
const char Menu2[]=  "AirFlow   %";
const char All[]=    "ThermoC Error!!";

//void PID ();
void Init();
void Lcd_3Dig(char Row,char Col,int Dat);
void Lcd_Fl_Str(char Row, char Col, unsigned Text);
void TC_Read();
signed int TempC();
void PID();


void main(){
 Init();
 VarBool=0;
 LcdRow=Min_Row;
 LcdRowOld=Max_Row;
 Stop_Loop=0;
 Lcd_Fl_Str(1,5,Logo);
 Lcd_Fl_Str(2,1,Logo1);
 Delay_ms(1000);
 Lcd_Cmd(_LCD_CLEAR);
 Lcd_Fl_Str(1,2,StatMen);


 //Read Data Fron EEprom
 //TempGun=80;//EEprom_read(Eprom_Add);
 TempGun=EEprom_read(Eprom_Add);
 TempGun=(TempGun<<8)+EEprom_read(Eprom_Add+1);
 AirFlow=EEprom_read(Eprom_Add+2);


 while (1){


 if(DoPid){
   DoPid=0;
   //LedV=~LedV;
   if(Stop_Loop>=1) Stop_Loop-=1;     //CoutDown Stop 1Sec
   ActTemp=TempC();
   if(ActTemp==-1) Tc_Allarm=1;
     else ActTemp=((ActTemp*8)/10);
   Lcd_3Dig(1,13,ActTemp);
   if(StartCicle)PID();
   //if(StartCicle==0){ Err=Err1=SumE=0;}
   //Lcd_3Dig(2,3,Pid_Res);
   }
 //Digital Potentiometer
   Act_PortB = PORTB& 0X30;
   if (Act_PortB!=Old_PortB){
    switch(Old_PortB)
     {
    case 0:     //0b0000 0000
      if((Act_PortB.F4==1)&&(Act_PortB.F5==0)) Pot-=1;
      if((Act_PortB.F4==0)&&(Act_PortB.F5==1)) Pot+=1;
    break;
    /*case 16:    //0b0001 0000
      if((Act_PortB.F4==1)&&(Act_PortB.F5==1)) Pot-=1;
      if((Act_PortB.F4==0)&&(Act_PortB.F5==0)) Pot+=1;
    break;*/
    case 48:    //0b0011 0000
      if((Act_PortB.F4==0)&&(Act_PortB.F5==1)) Pot-=1;
      if((Act_PortB.F4==1)&&(Act_PortB.F5==0)) Pot+=1;
    break;
    /*case 32:    //0b0010 0000
      if((Act_PortB.F4==0)&&(Act_PortB.F5==0)) Pot-=1;
      if((Act_PortB.F4==1)&&(Act_PortB.F5==1)) Pot+=1;
    break;*/
   }
    Old_PortB= Act_PortB;
  }

   if((SwitchEnc==0)&&(Edit==0)&&(MemSw==0))  {
    Edit=1;
    MemSw=1;
   }
   if((SwitchEnc==0)&&(Edit)&&(MemSw==0))  {
    Edit=0;
    MemSw=1;
   switch(LcdRow)
   {
    case 0x0A: EEprom_write(Eprom_Add+1,TempGun+1);EEprom_write(Eprom_Add,(TempGun>>8));break;
    case 0x0B: EEprom_write(Eprom_Add+2,AirFlow);break;
   } //End Switch
  }

  if(SwitchEnc) MemSw=0;

  if (Edit==0){     //Change row
    Loop+=Pot;
    if((Loop>2)||(Loop<-2)) {
       LcdRow+=Pot;
       Loop=0;
      }
    Pot=0;
  }
  if (LcdRow<Min_Row)  LcdRow=Min_Row;
  if (LcdRow>Max_Row)  LcdRow=Max_Row;


  // Dynamic pages

   if(LcdRow!=LcdRowOld){
     switch(LcdRow)
     {
      case 0x0A:
        Lcd_Fl_Str(2,2,Menu1);
        Lcd_3Dig(2,13,TempGun);
      break;
      case 0x0B:
        Lcd_Fl_Str(2,2,Menu2);
        Lcd_3Dig(2,13,AirFlow);
      break;

    } //End Switch
    LcdRowOld=LcdRow;
  }  //End Schermate Dinamiche

  if (Tc_Allarm) Lcd_Fl_Str(2,1,All);    //Termocouple Allarm

  if ((Edit)&&(Pot!=0))    //Variables
  {
   switch(LcdRow){
      case 0x0A:
         TempGun+=Pot;
         Pot=0;
         if (TempGun<Min_Temp) TempGun=Min_Temp;
         if (TempGun>Max_Temp) TempGun=Max_Temp;
         Lcd_3Dig(2,13,TempGun);
      break;
      case 0x0B:
         AirFlow+=Pot;
         Pot=0;
         if (AirFlow<Min_Flow) AirFlow=Min_Flow;
         if (AirFlow>Max_Flow) AirFlow=Max_Flow;
         Lcd_3Dig(2,13,AirFlow);
      break;
   }
  }  

  // Cicle Start Stop
  if (Start==0) {
   PWM2_Start();
   PWM2_Set_Duty(255);
   Delay_ms(2000);
   StartCicle=1;
  }
 if ((Stop==0)||(Tc_Allarm)) {
   StartCicle=0;

   if (Time_Stop==0){
    Time_Stop=1;
    Stop_Loop=TStop;
   }
  }
  LedR=StartCicle;
  if(StartCicle)PWM2_Set_Duty(154+AirFlow);
  if((StartCicle==0)&&(Time_Stop==1)){
   PWM2_Set_Duty(254);
   if (ActTemp<=TempStop){
     PWM2_Stop();
     PWMFan=0;
     Time_Stop=0;
     }
   }

 }    // Emd While
}     // End Main





void interrupt (){

 //INTCON.GIE=0;
 if(INTCON.INTF){
  INTCON.INTF=0;
  Heater=0;
  PidCount+=1;
  if(PidCount>=PidTime){
     DoPid=1;

     PidCount=0;
     }
 // LedV=~LedV;
  T1CON.B0=0;
  TMR1L=(PulseTime);          //It's calculated by subtracting the overflow
  TMR1H=(PulseTime>>8);
  T1CON.B0=1;
 }// //End Interrupt RB0

  if(PIR1.B0){   // Tmr1 Interrupt to generate the pulse fase
   PIR1.TMR1IF=0;
   LedV=~LedV;
   if (StartCicle) Heater=1;
   //Heater=~Heater;
   //Heater=1;
   Delay_us(10);
   Heater=0;
  }
 INTCON      =0b11010000;
 }//End Interrupt routine






void Init(){

  TRISA=0b000000;
  TRISB=0b01111101;
  TRISC=0b00001000;
  PORTA=0;
  PORTB=0;
  PORTC=0;
  //Reset Flag & StartUp
  //PCON = 0;               // Reset on StartUp
  //PIR1 = 0;

  ADCON1 = 6;               // Disable ADCON I/O general

                            //Prescaler TMR0 /32 are 312 pulse on 10 ms 250 + 62                            
							//Enabel Interrupt on RB4-RB5 and TMR2

  INTCON      =0b11010000;  // D1=INTF: RB0/INT External Interrupt Flag bit
                            // D4=INTE: RB0/INT External Interrupt Enable bit
                            // D6=PIE Periferical interrupt
                            // D7=GIE: Global Interrupt Enable bit
  OPTION_REG  =0b00000000;  // D7=0 PORTB pull-ups are enabled by individual port latch values
                            // D6=RBO/INT  EdgDetect "INTEDG" 1= Rising 0= Fallin
  PIR1        =0b00000000;  // D0=Tmr1Flag
  PIE1        =0b00000001;  // D0=Enable interrupt on Tmr1
  T1CON       =0b00100001;  // D0=TME1ON
                            // D4-D5 = Prescaler 1-2-4-8  Fosc4= 1000000/4 /50Hz /2 = 2500 half wave

                    //Abilitazioni periferiche Hw
  //UART1_Init(19200);
  PWM2_Init (15000);
  Lcd_Init();
  Lcd_Cmd(_LCD_CURSOR_OFF);
  Lcd_Cmd(_LCD_CLEAR);
  INTCON.B7=1;
  INTCON.B4=1;
 }



 void PID (void)    //Controllo PID
{
        Int_Res = Dev_Res = 0;
        Err1 = Err;
        Err = (TempGun-ActTemp);
        // Integrale
        SumE = SumE + Err;                      // SumE is the summation of the error terms
        if(SumE > SumE_Max)SumE = SumE_Max;
        if(SumE < SumE_Min)SumE = SumE_Min;

        //Int_Res = SumE / 10;                 // Ki*SumE/(Kp*Fs*X) where X is an unknown scaling factor
        Int_Res = SumE * Ki;                   // combination of scaling factor and Kp
        //Int_Res = Int_Res ;// / 16;

        // Calculate the derivative term
       // Dev_Res = Err - Err1;
        /*if(Dev_Res > 120)Dev_Res = 120;
        if(Dev_Res < -120)Dev_Res = -120;*/

        Dev_Res =   Kd*(Err - Err1);               // Derivative Kd(en0-en3)/(Kp*X*3*Ts)
       //Dev_Res = Dev_Res /2;

        if(Dev_Res > 120)Dev_Res = 120;
        if(Dev_Res < -120)Dev_Res = -120;


        // C(n) = K(E(n) + (Ts/Ti)SumE + (Td/Ts)[E(n) - E(n-1)])
        Pid_Res = Err + Int_Res + Dev_Res;        // Sum the terms
        Pid_Res = Pid_Res * Kp>>1;                // multiply by Kp then scale
        if(Pid_Res> PidOutMax)  Pid_Res=PidOutMax;
        if(Pid_Res< PidOutMin)  Pid_Res=PidOutMin;
        PulseTime=63340+Pid_Res;
}



void Lcd_3Dig(char Row,char Col,int Dat){
 char e,txt[7];
  Col-=3;
  IntToStr(Dat,txt);
  for(e=2;e<=5;e++){
  Lcd_Chr(Row,(Col+e),txt[e]);
  }

}


void Lcd_Fl_Str(char Row, char Col, unsigned Text)
{
char c,n;
 n=0;
 while(c=Flash_Read(Text+n)) {
 Lcd_Chr(Row,(Col+n),c);
 n++;
 }
}

char TC_Read() {
  char d;
  signed char i;
  d = 0;

   for (i=7; i>=0; i--)
  {
    CSK=1;
    Delay_ms(1);
    if (SO) {
      d |= (1 << i);
    }
    CSK=0;
    Delay_ms(1);
  }
  return d;
}

signed int TempC(){
  signed int Read;
  CS=0;
  Delay_ms(1);
  Read=0;
  Read = TC_Read();
  Read <<= 8;
  Read |= TC_Read();
  CS=1;
  if ((Read & 0x4)>>2) {  // Probe disconnected
  return (-1);
  }
  Read >>= 3;
  return (Read*0.25);

}