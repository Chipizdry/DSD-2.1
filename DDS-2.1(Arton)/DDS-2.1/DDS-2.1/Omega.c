/*
 * Omega.c
 *
 * Created: 07.04.2021 19:02:41
 *  Author: Red
 */ 

 #include "Omega.h"
 void Omega_slave() {
 
 
// PORTD|= (1 <<PD0); // ��������������� �����-��������� ��������� 
 stats= (PINB & 0b0000001); //������ ��������� �����
 stats1=(PINB & 0b0000100);
 if (( stats ==1)&&(bit_flag==0))
 {
	 
	// if((8<low)&& (low<11)){low=0;}    // ������������� ����
	// if((17<low)&& (low<21)){low=1;}   // ������������� �������
	// if((26<low)&& (low<35)){low=2;tct=0; input_bit=0;adress_t=0;directive=0;}  // ������� ������ ����� 
		 
		 
	  if((5<=low)&& (low<=7)){low=0;}    // ������������� ����
	  if((10<=low)&& (low<12)){low=1;}   // ������������� �������
	  if((14<low)&& (low<35)){low=2;tct=0; input_bit=0;adress_t=0;directive=0;}  // ������� ������ �����	 
		 
	 if(low>1000){tct=0; input_bit=0;adress_t=0;directive=0;low=0;external=0;}
	 detect[tct]=low;
	 
	 if (tct==8)
	 {
		 adress_t|= (detect[2]<<6)|(detect[3]<<5)|(detect[4]<<4)|(detect[5]<<3)|(detect[6]<<2)|(detect[7]<<1)|(detect[8]) ;
		 if(adress_t==0){adres_call=0;}
	 }

	 if((adress_t==adress)&&(tct>=8)) {protocol();}
	 
	   
	 bit_flag=1;
	 hi_flag=0;
	 low=0;
	 temp_ID=0;
	 if((detect[1]==1)&&(detect[2]==0)&&(tct==2))
	  {
		 adres_call=adres_call+1;
		 if(adres_call==adress)
		    {
			 adress_t=adres_call;{  PORTB |= (1 <<PB1); PORTB|= (1 <<PB4);} 	          
		    }
	  }
		 tct=tct+1;
	 }
	 
	 
	 if (( stats ==1)&&(bit_flag==1))
	 {
		 active =active+1;	 
	 }
	 
	 
	 if (( stats ==0)&&(hi_flag==0))
	 {
		  PORTB &=~ (1 <<PB1);                // digitalWrite(13,LOW);
		  PORTB&=~ (1 <<PB4);
		 hi_flag=1;
		 bit_flag=0;
		 active=0;
		 input_bit=0;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 }
	 
	 if (( stats ==0)&&(hi_flag==1))
	 {	 
		 active=0;
		 low=low+1;
	 }

	// PORTD &=~(1 <<PD0);  // ��������������� �����-��������� ��������� 
	 
	 }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	 
	 void protocol(void)
	 {
		 if (tct==13)
		 {
			 directive|=(detect[10]<<3)|(detect[11]<<2)|(detect[12]<<1)|(detect[13]) ; //����������� ���� ������� 
		 }
		 
		if(tct==9) {  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}   //OWR � �������� ��������� ��� ������� �� ����  
		if(tct==13){  PORTB|= (1 <<PB1); PORTB|= (1 <<PB4);}   //OWR � �������� ��������� ��� ������� �� �������������/����
		if((tct==14)&&((stat[0]==1)||(external==2)))   {  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}   //��������� ������� 
		 
		 
		if(tct>=13)
		{ 
			
			if(tct==22){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==31){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==40){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==49){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==58){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==67){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==76){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
		    if(tct==76){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==85){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			if(tct==94){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
						
							
				
		  switch(directive)
		   {
			   
		   case 0 :
		   if(tct==9) {  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}                 //OWR � �������� ��������� ��� ������� �� ����
		   if(tct==13){  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}                  //OWR � �������� ��������� ��� ������� �� �������������/����   
		   if((tct==14)&&((stat[0]==1)||(external==2)))   {  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}   //��������� �������
			   
			   if(tct==22){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			   if(tct==31){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			   if(tct==40){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			   if(tct==49){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			   if(tct==58){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			   if(tct==67){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			   if(tct==76){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
			   
		   break;
		   
		   case 2 :
		  
		   if(tct==13){  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}                  //OWR � �������� ��������� ��� ������� �� �������������/����
		   if((tct==14)&&((stat[0]==1)||(external==2)))   {  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}   //��������� �������
		   break;
		   
		   case 3 :
		 
		   if(tct==13){  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}                  //OWR � �������� ��������� ��� ������� �� �������������/����
		   if((tct==14)&&((stat[0]==1)||(external==2)))   {  PORTB|= (1 <<PB1); PORTC|= (1 <<PC5);}   //��������� �������
		
		   if(tct==22){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� ������������� 
		   if(tct==31){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
		   if(tct==40){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
		   if(tct==49){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
		   if(tct==58){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
		   if(tct==67){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
		   if(tct==76){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  // ������� �������������
		
	   	   if((tct>=14)&&(tct<22))   //�������������� ����������
			    {
				 temp_ID|=(device_ID>>(21-tct))&(0b1);
				 if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				 if(temp_ID==0){  PORTB &=~ (1 <<PB1);}
			    }  
				
			if((tct==30)&&(tct<32))  //��������� ������ -�������
			{
				temp_ID|=((external)&(0b1));
				if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				if(temp_ID==0){  PORTB &=~(1 <<PB1);}
			}	
				
				
				
		   if((tct>=34)&&(tct<36))  //������ ����� (�� ����)
			    {
				 temp_ID|=((mode)&(0b1));
				 if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				 if(temp_ID==0){  PORTB &=~ (1 <<PB1);}
			    }
       ///////////////////////////////////////////////////////////////////////////// 
			if((tct>=36)&&(tct<40))  //��������� ������ -�������
			{
				temp_ID|=((external)&(0b1));
				if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				if(temp_ID==0){  PORTB &=~(1 <<PB1);}
			}
		////////////////////////////////////////////////////////////////////////////		 
		/*	 
			if((tct>=42)&&(tct<46))  //������ �������.�������
			{
				temp_ID|=((1)&(0b1));
				if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				if(temp_ID==0){  PORTB &=~(1 <<PB1);}
			}
				 
			if((tct>=49)&&(tct<50))  //������ �������??????
			{
				temp_ID|=((1)&(0b1));
				if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				if(temp_ID==0){  PORTB &=~(1 <<PB1);}
			}
		*/	
				 
		   if((tct>=39)&&(tct<40))  //������ ������� DHT
			    {
				 temp_ID|=((receivemode)&(0b1));
				 if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				 if(temp_ID==0){  PORTB &=~(1 <<PB1);}
			    }   
			 
		   if((tct>=51)&&(tct<57))  // ������� ���� ������� 
			    {
				  temp_ID|=((fire/4)>>(57-tct))&(0b1);
				  if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
				  if(temp_ID==0){  PORTB &=~ (1 <<PB1);}          
			    }
				
		   if((tct>=60)&&(tct<67)) // �������� (�����������)
				{
				  temp_ID|=(tmp>>(66-tct))&(0b1);
				  if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                  
				  if(temp_ID==0){  PORTB &=~ (1 <<PB1);}                
				}
		   if((tct>=69)&&(tct<76))  //����������� 
			    {
				temp_ID|=((25)>>(75-tct))&(0b1);
				if(temp_ID==1){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                
				if(temp_ID==0){  PORTB &=~ (1 <<PB1);}               
			    } 
			   
		   break;
		   
		   case 14 :
		   if(tct==13){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}                 
		   if(tct==30){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
		   if(tct==45){  PORTB |= (1 <<PB1); PORTC|= (1 <<PC5);}
		   if(tct==46)
		   {
			   if((detect[35]==1)&&(detect[42]==1)) {external=0;}
			   if((detect[36]==1)&&(detect[43]==1)) {external=0;}
			   if((detect[38]==1)&&(detect[45]==1)) {external=1;}
			   if((detect[37]==1)&&(detect[44]==1)) {external=0;}
		   }
		   break;
		   
		 }
	  }
		 
 }