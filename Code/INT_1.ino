void INT_1()
{ 
       
//A    |¯¯|__|¯¯|__|¯
//B  - __|¯¯|__|¯¯|__  +
// Nel canale A pin D3 condensatore ceramico 103K 10nF
 ActEnc=0;
 ActEnc= digitalRead(Encoder_A);
 ActEnc= (ActEnc<<1)+digitalRead(Encoder_B);
 switch(ActEnc)
 {
  case 0: //00
  //if(OldEnc==2) Pot++;  //10
  if(OldEnc==3) Pot--;  //01
  break;
  case 1: //01
  if(OldEnc==2) Pot++;  //10
  //if(OldEnc==0) Pot--;  //00
  //if(OldEnc==1) Pot--;  //01
  break;
  case 2: //10
  if(OldEnc==1) Pot++;  //01
  //if(OldEnc==3) Pot--;  //11
  break;
  case 3: // 11
  //if(OldEnc==1) Pot--;  //01
  if(OldEnc==0) Pot--;  //00
  break;
 }
 
 OldEnc=ActEnc;
}

