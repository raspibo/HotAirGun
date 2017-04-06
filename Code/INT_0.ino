void INT_0()
{
Time1++;
if (Time1>=LCD_Update) {
	LcdUpd=1;
	Time1=0;
}
Time2++;
if (Time2>=PID_Update) {
	PIDUpd=1;
	Time2=0;
}



}
