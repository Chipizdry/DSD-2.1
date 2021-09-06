#include <avr/io.h>
//#define F_CPU              16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <EEPROM.h>

#define OMEGA_PORT        PORTC
#define OMEGA_DDR         DDRC
#define OMEGA_PIN         PINC
#define OMEGA_DQ             5

int stats;
int timing=0;
int milis;
int active;
int low;
unsigned char bit_flag;
unsigned char hi_flag;
int tct;
int input_bit;
int adres_call;
long rd;
int frame=0;
int answer=0;
int retn=0;
int adress_t;
int adress=3;
int new_adress;
int new_adress_test;
bool isolator;
bool led_status;
bool alarm_status;
int directive=0;
int device_ID=26;// 157-блок коммутации ,89-СПРА ,108-БСА,25-СПДОТА
int temp_ID;
int result[180];
int out[180];
int detect[180]={0,0,0,0,0};
//int detect_2[180]={0,0,0,0,0};
int bffr[180];
//int bffr_2[180];
int cnt_buff=0;
void protocol();
void strobe_tact();
void strobe_command();
void strobe_diagnostic();
void strobe_tact(void){
  
    if(tct==13){digitalWrite(13, HIGH);}
    if(tct==22){digitalWrite(13, HIGH);}
    if(tct==31){digitalWrite(13, HIGH);}
    if(tct==40){digitalWrite(13, HIGH);}
    if(tct==49){digitalWrite(13, HIGH);}
    if(tct==58){digitalWrite(13, HIGH);}
    if(tct==67){digitalWrite(13, HIGH);}
    if(tct==76){digitalWrite(13, HIGH);}
    if(tct==85){digitalWrite(13, HIGH);}
    if(tct==94){digitalWrite(13, HIGH);}
    if(tct==103){digitalWrite(13, HIGH);}
    if(tct==112){digitalWrite(13, HIGH);}
    if(tct==121){digitalWrite(13, HIGH);}
    if(tct==130){digitalWrite(13, HIGH);}
  }


void strobe_command(void){
  
    if(tct==13){digitalWrite(13, HIGH);}
    if(tct==30){digitalWrite(13, HIGH);}
    if(tct==45){digitalWrite(13, HIGH);}
    if(tct==54){digitalWrite(13, HIGH);}
    if(tct==63){digitalWrite(13, HIGH);}
    if(tct==72){digitalWrite(13, HIGH);}
    if(tct==81){digitalWrite(13, HIGH);}
    if(tct==90){digitalWrite(13, HIGH);}
    if(tct==99){digitalWrite(13, HIGH);}
    if(tct==108){digitalWrite(13, HIGH);}
    if(tct==117){digitalWrite(13, HIGH);}
    if(tct==126){digitalWrite(13, HIGH);}
  
  }

void strobe_diagnostic(void){
  
    if(tct==13){digitalWrite(13, HIGH);}
    if(tct==22){digitalWrite(13, HIGH);}
    if(tct==31){digitalWrite(13, HIGH);}
    if(tct==40){digitalWrite(13, HIGH);}
    if(tct==48){digitalWrite(13, HIGH);}
    if(tct==50){digitalWrite(13, HIGH);}
    if(tct==52){digitalWrite(13, HIGH);}
    if(tct==55){digitalWrite(13, HIGH);}
    if(tct==58){digitalWrite(13, HIGH);}
    if(tct==59){digitalWrite(13, HIGH);}
    if(tct==68){digitalWrite(13, HIGH);}
    if(tct==72){digitalWrite(13, HIGH);}
    if(tct==73){digitalWrite(13, HIGH);}
    if(tct==74){digitalWrite(13, HIGH);}
    if(tct==75){digitalWrite(13, HIGH);}
    if(tct==76){digitalWrite(13, HIGH);}
     if((tct>=31)&&(tct<150))//   ID 
             {         
              
             digitalWrite(13, HIGH); 
             }
  }

void protocol(void){


   if (tct==13)
     {
       directive|=((detect[10]<<3)|(detect[11]<<2)|(detect[12]<<1)|(detect[13])) ; //Определение типа запроса 
     }
     
   switch(directive)
      {


    case(0):
           if((tct>=14)&&(tct<22))//   ID  устройства 
             {         
              temp_ID|=(device_ID>>(21-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);} 
              if(temp_ID==0){digitalWrite(13,LOW);} 
             }
      strobe_tact();
    
     break;   

        case(1):
        led_status=1;
        strobe_tact();
        
         break; 
     
          case(2):
          
           if((tct>=14)&&(tct<22))//   ID  устройства 
             {         
              temp_ID|=(device_ID>>(21-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);} 
              if(temp_ID==0){digitalWrite(13,LOW);} 
             }
     strobe_tact();
     break;

     
     case(3):
      strobe_tact();
  
            if((tct>=14)&&(tct<22))//   ID  устройства 
             {         
              temp_ID|=(device_ID>>(21-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);} 
              if(temp_ID==0){digitalWrite(13,LOW);} 
             }

             if((tct>=23)&&(tct<=25))  //Загрязнение камеры %%
             {   
                //32 Камера 50%
                //64 Камера 25%
              temp_ID|=((6)>>(25-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  

///////////////////////////////////////////////////////////////////
              if(tct==27)  //Изолятор устройства
             {   
                //1 Тревога 
                //2 Светодиод
                //4 Изолятор разомкнут
                //8 Програмное размыкание 
                //16 EEPROM  
                //isolator =1;
              // isolator = EEPROM.read(1);
              if(isolator==1){digitalWrite(13, HIGH);}
              if(isolator==0){digitalWrite(13,LOW);}
             }  
           ///////////////////////////////////////////////////

            if(tct==29)  //Светодиод 
             {   
              if(led_status==1){digitalWrite(13, HIGH);}
              if(led_status==0){digitalWrite(13,LOW);}
             }  
           ///////////////////////////////////////////////////

             if(tct==30)  //Тревога  
            // alarm_status=1;
             {   
              alarm_status=1;
              if(alarm_status==1){digitalWrite(13, HIGH);}
              if(alarm_status==0){digitalWrite(13,LOW);}
             }  
           ///////////////////////////////////////////////////
           
           /*
             if((tct>=26)&&(tct<=30))  //Состояния устройства
             {   
                //1 Тревога 
                //2 Светодиод
                //4 Изолятор разомкнут
                //8 Програмное размыкание 
                //16 EEPROM  
              temp_ID|=((0)>>(30-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  
           */  
////////////////////////////////////////////////////////////////           
             if((tct>=32)&&(tct<=39))  //ЗАГРЯЗНЕНИЕ КАМЕРЫ
             {   
        
              temp_ID|=((10)>>(39-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  

             if((tct>=41)&&(tct<=48))  //Уровень дыма
             {   
        
              temp_ID|=((38)>>(48-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  
             
         if((tct>=50)&&(tct<=57))  //Уровень нуля
             {   
        
              temp_ID|=((218)>>(57-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  

          if((tct>=59)&&(tct<=66))  //Смещение
             {   
        
              temp_ID|=((46)>>(66-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  

          
          if((tct>=68)&&(tct<=75))  //Температура
             {   
        
              temp_ID|=((24)>>(75-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  
           

         
          
           if((tct>=77)&&(tct<=84))  //Ток светодиода
          {
          
        // int current=0;
            temp_ID|=((118)>>(84-tct))&(0b1);
            if(temp_ID==1){digitalWrite(13, HIGH);}
            if(temp_ID==0){digitalWrite(13,LOW);}
          }
       
         
          if((tct>=86)&&(tct<=93))  //Напряжение питания
             {   
         int   volts=255;
              temp_ID|=((volts)>>(93-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  
             

    break;


        case(6):
         strobe_tact();
             if((tct>=14)&&(tct<=21))//Мощность импульса ?????????????????
             {         
              temp_ID|=((7)>>(21-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);} 
              if(temp_ID==0){digitalWrite(13,LOW);} 
             }
       
          if((tct>=23)&&(tct<=30))  //Фон при 0дБ/м
             {   
        
              temp_ID|=((27)>>(30-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  

              if((tct>=32)&&(tct<=39))  //Уровень тревоги
             {   
        
              temp_ID|=((98)>>(39-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  
             if((tct>=41)&&(tct<=48))  //Ток светодиода
             {   
              temp_ID|=((161)>>(48-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  


             if((tct>=50)&&(tct<=57))  //Служебная информация/Компенсация тем-ры/Комп.загрязнения/
             {   
              temp_ID|=((63)>>(57-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  

              if((tct>=59)&&(tct<=62))  //Служебная информация???????
             {   
              temp_ID|=((11)>>(62-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  

               if((tct>=63)&&(tct<=66))  //Мощность импульса
             {   
              temp_ID|=((4)>>(66-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);}
              if(temp_ID==0){digitalWrite(13,LOW);}
             }  
              
         break;




        case(7):    
         strobe_tact();
          strobe_diagnostic();
          if((tct>=14)&&(tct<22))//   ID  устройства 
             {         
              temp_ID|=(device_ID>>(21-tct))&(0b1);
              if(temp_ID==1){digitalWrite(13, HIGH);} 
              if(temp_ID==0){digitalWrite(13,LOW);} 
             }
         break;
              
         case(8):    //Смена адреса 
         strobe_command();

           if (tct==39)
         {
         new_adress_test|=((detect[32]<<6)|(detect[33]<<5)|(detect[34]<<4)|(detect[35]<<3)|(detect[36]<<2)|(detect[37]<<1)|(detect[38])) ; //Определение типа запроса 
         
         }

           if (tct==46)
         {
         new_adress|=((detect[39]<<6)|(detect[40]<<5)|(detect[41]<<4)|(detect[42]<<3)|(detect[43]<<2)|(detect[44]<<1)|(detect[45])) ; //Определение типа запроса 
          if(new_adress_test==new_adress){
         adress=new_adress;
          EEPROM.put(0,adress);new_adress=0;new_adress_test=0; }
        }
       
         
         break;
            
       
         case(9):    
         strobe_command();
         break;

         case(14):    //Активация изолятора 
         strobe_command();  

            if (tct==46)
         {
           if((detect[36]==1)&&(detect[43]==1)){isolator=1;EEPROM.put(1,isolator);  }
           if((detect[36]==0)&&(detect[43]==0)){isolator=0;EEPROM.put(1,isolator);  }
         }
          break;
         
         
      }
  
  }

ISR(TIMER0_COMPA_vect){
 digitalWrite(4,HIGH);
  
    
  timing++;
  stats = digitalRead(A3);
  answer = digitalRead(A5);
  if (( stats ==1)&&(bit_flag==0))
  {   
    
    if((4<low)&& (low<17)){low=0;}  // if((8<low)&& (low<11)){low=0;}
    if((17<low)&& (low<=27)){low=1;} // if((17<low)&& (low<21)){low=1;}
    if((28<low)&& (low<35)){low=2;tct=0;adress_t=0;directive=0;}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(low==2){cnt_buff++;}
    if(cnt_buff==2){ cnt_buff=0;}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
      detect[tct]=low; 


    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    if (tct==8)
    {
      adress_t=0;
      adress_t|= (detect[2]<<6)|(detect[3]<<5)|(detect[4]<<4)|(detect[5]<<3)|(detect[6]<<2)|(detect[7]<<1)|(detect[8]) ;
      if(adress_t==0){adres_call=0;}
    }

    if((adress_t==adress)&&(tct==9)){digitalWrite(12, HIGH);}

         // if((adress_t==adress)&&(tct==14)){digitalWrite(13, HIGH);}//СОСТОЯНИЕ ТРЕВОГИ 
    if((adress_t==adress)&&(tct>=9)){protocol();}

    bit_flag=1;
    hi_flag=0;
    low=0;
     temp_ID=0;  
    if((detect[1]==1)&&(detect[2]==0)&&(tct==2))
          {
            adres_call=adres_call+1;
            if(adres_call==adress){
             adress_t=adres_call;  
            
            digitalWrite(13, HIGH); }}
    tct=tct+1;
  }
  
  
  if (( stats ==1)&&(bit_flag==1))
  {   
    active =active+1; 

     
     if(answer==1){retn=retn+1;}
     
    if((33<=active) &&(tct>3)) //23<225
      {
      low=3;
      detect[tct]=low;
      tct=tct+1;
      active=0;
      if((detect[0]==2)&&(tct>=2))
         {
         frame=tct;
         }
      }  
   }
  
  
  if (( stats ==0)&&(hi_flag==0))
  {
    digitalWrite(13,LOW);
    digitalWrite(12,LOW);
    hi_flag=1;
    bit_flag=0; 
    active=0;
    //input_bit=0;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  }
  
  if (( stats ==0)&&(hi_flag==1))
  {
     if((retn>11)&(retn<21))       // if((retn<300)&&(retn>16))
     {
     result[tct]=1;
     }
    // else {result[tct]=0;}
    
    active=0;
    retn=0;
    low=low+1;
  }
  
   if((tct>=2)&&(frame==tct))
  {
    tct=0;
    
   digitalWrite(4,HIGH);  
    for(int i=0;i<frame;i++)
  {  
  bffr[i]=detect[i];
  out[i]=result[i];
  result[i]=0;
  detect[i]=0;
  }
  digitalWrite(4,LOW);
 input_bit=1;
  }

 // digitalWrite(4,LOW);
}




int main(void)
{
  TCCR0A  = 0x00;
  TCCR0B |= (0 << CS12)|(1 << CS11)|(0 << CS10)|(1 << WGM12); //предделитель clk/1024, режим таймера СТС
  TCNT0 = 0x00;
  OCR0A = 255; // максимальный предел счета
  TIMSK0 = (1 << OCIE0A); // разрешение прерывания по совпадению

   // Настройка портов ввода/вывода
   DDRC = 0b00000000;
   DDRB = 0b00000000;
   PORTB= 0b00000000;
   DDRD = 0b11111110;
   PORTC= 0b00001000;
   Serial.begin(115200);
   pinMode(4, OUTPUT);           // назначить выводу порт ввода
   pinMode(A5,INPUT);
   pinMode(A3,INPUT);
   pinMode(13, OUTPUT); 
   digitalWrite(13, LOW);       // вsключить подтягивающий резистор
   //digitalWrite(A3,HIGH);
   adress = EEPROM.read(0);
   isolator = EEPROM.read(1);
 //  led_status = EEPROM.read(2);
   if (adress==255){adress=0;}
   if (isolator>=2){isolator=0;}
   if (led_status>=2){led_status=0;}
    sei();
     OMEGA_DDR &= ~(1 << OMEGA_DQ); // вход
     
    while (1) 
    {
      
   for(int g=0;g<frame;g++)
   {
    
    Serial.print(g);
    Serial.print(":");
   
    if(out[g]==0)
    {
    if(g==9){Serial.print( bffr[g]);Serial.print(";Adr:");Serial.println(adress_t);}
    if((g!=14)&&(g!=9))Serial.println( bffr[g]);
    if(g==14){Serial.print( bffr[g]);Serial.print(";Cmd:");Serial.println(directive);}}
    if(out[g]==1)
    { Serial.print( bffr[g]);
  
     if((g==14)||(g==23)||(g==32)||(g==41)||(g==50)||(g==59)||(g==68)||(g==77)||(g==86)||(g==95)||(g==104)||(g==113)||(g==122)||(g==131)||(g==140)||(g==149))
      {
       
       if(g!=14)Serial.println(";strobe");
       if(g==14){Serial.print(";strobe;Cmd:");Serial.println(directive);};
      }

      else{ 
      Serial.print(";ans:");Serial.println( out[g]) ; }
      }
    
     if(g==(frame-1)){rd=rd+1;Serial.print("Search:");Serial.println(adres_call);Serial.print("Cmd:");Serial.println(directive);Serial.print("Cnt:");Serial.println(cnt_buff); Serial.print("Rst:");Serial.println(rd);}
   } 
   
   frame=0;
   input_bit=0;
    }
    
}
