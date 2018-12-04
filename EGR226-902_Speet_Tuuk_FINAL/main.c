#include "msp.h"
#include <stdio.h>
#include <string.h>
#define HALFPLUS 3000000
#define MAX_NOTE 2 // How many notes are in the song below
#define E4 329.63
#define REST 0
#define BREATH_TIME 50000

float speed = 1;

/*
 * Dylan Speet and Zachary Tuuk
 * 11/27/2018 Start
 * Alarm Clock Final Project for EGR 226-902
 * Uses timerA, Real time clock, Interrupts, DAC, and ADC
 * Buttons, LEDs, LCD, Potentiometer, Speaker
 */
enum states{
    setalarm,
    settime,
    clock,
    snooze,
    alarm
};
enum states state = clock;
int time_update = 0, alarm_update = 0, i = 0, time_set = 0, ampm = 1, hours, mins, set_time = 0, houradjust = 0, minadjust = 0, set_alarm = 0, alarm_status = 1;
uint8_t  secs, hour_update;
int alarmhours = 14, alarmmins = 46, status = 0, wakeupsequence = 0, alarm_sound;
char m, a;
float volt, celsius, fahrenheit, duty = 0.0004;
char temperature[12];
char astatus[10];
int note = 0;       //The note in the music sequence we are on
int breath = 0;

float music_note_sequence[][2] = {
                                 {E4,HALFPLUS},
                                 {REST,HALFPLUS},
};


void P2_Init();
void SetupTimer32s();
void alarm_Status();
void wake_Up_Lights();
void return_ADC();
void P4_Init_ADC();
void ADC14_Init();
void initialization();
void LCD_init();
void RTC_Init();
void commandWrite(uint8_t command);   //command function
void pushByte(uint8_t byte);           //pushByte function
void dataWrite(uint8_t data);        //dataWrite
void pushNibble (uint8_t nibble);      //push over four bits
void PulseEnablePin();                  //Pulse Enable (E)
void delay_ms(unsigned ms);             //delay for a set millisecond delay
void delay_micro(unsigned microsec);      //delay for a microsecond delay
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
char currenttime[11];
char alarmtime[14];
char doublespace[2] = "  ";                                //used to write blanks
    initialization();                               //initialize all pins, timers, and interrupts
    RTC_Init();
    __enable_interrupt();
    LCD_init();
    P4_Init_ADC();
    ADC14_Init();
    SetupTimer32s();

while (1){
    wake_Up_Lights();
    return_ADC();
    alarm_Status();
    switch (state){

    case clock:
        play_alarm();
        if(time_update){
            commandWrite(0x80);
             if(hours<10){
                 sprintf(currenttime," %01d:%02d:%02d %cM",hours,mins,secs, m);
             }
             else
                 sprintf(currenttime,"%02d:%02d:%02d %cM",hours,mins,secs, m);
             while(!(currenttime[i]=='\0')){                            //print time until null
                              dataWrite(currenttime[i]);
                              i++;
                              }
                              i=0;
                }
        commandWrite(0x90);
        if((alarmhours)<10){
            if(alarmhours == 0){
                sprintf(alarmtime,"ALARM:%02d:%02d AM",alarmhours+12,alarmmins);
            }
            else
                sprintf(alarmtime,"ALARM: %01d:%02d AM",alarmhours,alarmmins);
        }
        if(alarmhours >= 10 && alarmhours <=12){
            if(alarmhours == 12)
                sprintf(alarmtime,"ALARM:%02d:%02d PM",alarmhours,alarmmins);
            else
              sprintf(alarmtime,"ALARM:%02d:%02d AM",alarmhours,alarmmins);
        }
        if(alarmhours >= 12)
        {
            if((alarmhours-12)>=10)
             sprintf(alarmtime,"ALARM:%02d:%02d PM",alarmhours-12,alarmmins);
            if((alarmhours-12)<10)
                if((alarmhours-12)==0)
                    sprintf(alarmtime,"ALARM:%02d:%02d PM",(alarmhours-12)+12,alarmmins);
                else
                sprintf(alarmtime,"ALARM: %01d:%02d PM",alarmhours-12,alarmmins);
        }
        while(!(alarmtime[i]=='\0')){
                                dataWrite(alarmtime[i]);
                                        i++;
                                        }
                                        i=0;
                if(alarm_update){
                    if(alarm_status==0)
                        alarm_update=0;
                    printf("ALARM\n");
                }
        break;

    case settime:
        if(set_time == 1 && houradjust == 0){
            commandWrite(0x80);
            while(!(doublespace[i]=='\0')){                            //print doublespace until null
                 dataWrite(doublespace[i]);
                 i++;
                 }
                 i=0;
        }
        if(houradjust == 1 && set_time == 1)
        {
            commandWrite(0x80);
        if(hours<10){
            sprintf(currenttime," %01d:%02d:%02d %cM",hours,mins,secs, m);
        }
        else
            sprintf(currenttime,"%02d:%02d:%02d %cM",hours,mins,secs, m);
        while(!(currenttime[i]=='\0')){                            //print time until null
                         dataWrite(currenttime[i]);
                         i++;
                         }
                         i=0;
        }
        if(set_time == 2 && minadjust == 0){
            commandWrite(0x83);
                       while(!(doublespace[i]=='\0')){                            //print doublespace until null
                            dataWrite(doublespace[i]);
                            i++;
                            }
                            i=0;
        }
        if(set_time == 2 && minadjust ==1)
        {
               commandWrite(0x80);
           if(hours<10){
               sprintf(currenttime," %01d:%02d:00 %cM",hours,mins, m);
           }
           else
               sprintf(currenttime,"%02d:%02d:00 %cM",hours,mins, m);
           while(!(currenttime[i]=='\0')){                            //print time until null
                            dataWrite(currenttime[i]);
                            i++;
                            }
                            i=0;
           }
        break;
    case setalarm:
        if(time_update){
                   commandWrite(0x80);
                    if(hours<10){
                        sprintf(currenttime," %01d:%02d:%02d %cM",hours,mins,secs, m);
                    }
                    else
                        sprintf(currenttime,"%02d:%02d:%02d %cM",hours,mins,secs, m);
                    while(!(currenttime[i]=='\0')){                            //print time until null
                                     dataWrite(currenttime[i]);
                                     i++;
                                     }
                                     i=0;
                       }
        if(set_alarm == 1 && houradjust == 0){
                commandWrite(0x96);
                while(!(doublespace[i]=='\0')){                            //print doublespace until null
                     dataWrite(doublespace[i]);
                     i++;
                     }
                     i=0;
            }
            if(houradjust == 1 && set_alarm == 1)
            {
                commandWrite(0x90);
                      if((alarmhours)<10){
                          if(alarmhours == 0){
                              sprintf(alarmtime,"ALARM:%02d:%02d AM",alarmhours+12,alarmmins);
                          }
                          else
                              sprintf(alarmtime,"ALARM: %01d:%02d AM",alarmhours,alarmmins);
                      }
                      if(alarmhours >= 10 && alarmhours <=12){
                          if(alarmhours == 12)
                              sprintf(alarmtime,"ALARM:%02d:%02d PM",alarmhours,alarmmins);
                          else
                            sprintf(alarmtime,"ALARM:%02d:%02d AM",alarmhours,alarmmins);
                      }
                      if(alarmhours >= 12)
                      {
                          if((alarmhours-12)>=10)
                           sprintf(alarmtime,"ALARM:%02d:%02d PM",alarmhours-12,alarmmins);
                          if((alarmhours-12)<10)
                              if((alarmhours-12)==0)
                                  sprintf(alarmtime,"ALARM:%02d:%02d PM",(alarmhours-12)+12,alarmmins);
                              else
                              sprintf(alarmtime,"ALARM: %01d:%02d PM",alarmhours-12,alarmmins);
                      }
                      while(!(alarmtime[i]=='\0')){
                                              dataWrite(alarmtime[i]);
                                                      i++;
                                                      }
                                                      i=0;
            }
            if(set_alarm == 2 && minadjust == 0){
                commandWrite(0x99);
                           while(!(doublespace[i]=='\0')){                            //print doublespace until null
                                dataWrite(doublespace[i]);
                                i++;
                                }
                                i=0;
            }
            if(set_alarm == 2 && minadjust ==1)
            {
                commandWrite(0x90);
                      if((alarmhours)<10){
                          if(alarmhours == 0){
                              sprintf(alarmtime,"ALARM:%02d:%02d AM",alarmhours+12,alarmmins);
                          }
                          else
                              sprintf(alarmtime,"ALARM: %01d:%02d AM",alarmhours,alarmmins);
                      }
                      if(alarmhours >= 10 && alarmhours <=12){
                          if(alarmhours == 12)
                              sprintf(alarmtime,"ALARM:%02d:%02d PM",alarmhours,alarmmins);
                          else
                            sprintf(alarmtime,"ALARM:%02d:%02d AM",alarmhours,alarmmins);
                      }
                      if(alarmhours >= 12)
                      {
                          if((alarmhours-12)>=10)
                           sprintf(alarmtime,"ALARM:%02d:%02d PM",alarmhours-12,alarmmins);
                          if((alarmhours-12)<10)
                              if((alarmhours-12)==0)
                                  sprintf(alarmtime,"ALARM:%02d:%02d PM",(alarmhours-12)+12,alarmmins);
                              else
                              sprintf(alarmtime,"ALARM: %01d:%02d PM",alarmhours-12,alarmmins);
                      }
                      while(!(alarmtime[i]=='\0')){
                                              dataWrite(alarmtime[i]);
                                                      i++;
                                                      }
                                                      i=0;
            }
}
}
}
void RTC_Init(){
    //Initialize time to 2:45:55 pm
//    RTC_C->TIM0 = 0x2D00;  //45 min, 0 secs
    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = 45<<8 | 45;//45 min, 55 secs
    RTC_C->TIM1 = 1<<8 | 14;  //Monday, 2 pm
    RTC_C->YEAR = 2018;
    //Alarm at 2:46 pm
    RTC_C->AMINHR = alarmhours<<8 | alarmmins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b0010;  //1/64 second interrupt is 0b0010 a 1 second interupt is

    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;
    NVIC_EnableIRQ(RTC_C_IRQn);
}

void RTC_C_IRQHandler()
{
    if(RTC_C->PS1CTL & BIT0){
        hours = RTC_C->TIM1 & 0x00FF;
        mins = (RTC_C->TIM0 & 0xFF00) >> 8;
        if(mins>59)
            RTC_C->TIM0 = 0<<8;
        secs = RTC_C->TIM0 & 0x00FF;
    }
    if(time_set == 1)
    {
        if(mins>59){
            RTC_C->TIM1 = ((RTC_C->TIM1 & 0x00FF)+1);
        }
        if(hours>24)
            RTC_C->TIM1 = 1;
        if(hours>=12 && hours<24){
            m = 'P';
        }
        else
            m = 'A';
        if(hours>12){
            hours = hours-12;
        }
        if(secs != 59){
            RTC_C->TIM0 = RTC_C->TIM0 + 1;
        }
        else {
            RTC_C->TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;
            time_update = 1;
        }
        RTC_C->PS1CTL &= ~BIT0;
    }
    if(time_set == 0)
    {
        if(hours>24)
            RTC_C->TIM1 = 1;
        if(hours>=12 && hours<24){
            m = 'P';
        }
        else
            m = 'A';
        if(hours>12){
            hours = hours-12;
        }
        time_update = 1;
        RTC_C->PS1CTL &= ~BIT0;
    }
    if(RTC_C->CTL0 & BIT1)
    {
        alarm_update = 1;
        RTC_C->CTL0 = (0xA500) | BIT5;
    }
}
void PORT3_IRQHandler()
{
    status = P3->IFG;
    P3->IFG = 0;
    if(status & BIT2) //second timing
    {
        //sets the RTC to have 1 second real time = 1 second clock time
        time_set=0;
    }
    if(status & BIT3) //minute timing
    {
        //sets the RTC so that 1 second real time = 1 minute clock time
        time_set=1;
    }
    if(status & BIT5) //set alarm
    {
        state = setalarm;
        set_alarm++;
        if(set_alarm == 2){
            houradjust = 0;
        }
        if(set_alarm == 3){
            minadjust = 0;
            state = clock;
            set_alarm = 0;
        }
    }
    if(status & BIT6) //set time
    {
        //sets the current time for the RTC
        state = settime;
       set_time++;
       if(set_time == 2){
           houradjust = 0;
       }
       if(set_time == 3)
       {
           minadjust = 0;
           state = clock;
           set_time = 0;
       }
    }
    if(status & BIT7) //snooze/down
    {
        //sets the alarm for 10 minutes later for the snooze function
        //acts as the DOWN button for when times are entered
        if(alarm_update==1)
        {
            int first_snooze=0;
            if(alarmmins <= 49 && first_snooze==0){
                alarmmins = alarmmins +10;
                first_snooze = 1;
            }
            if(alarmmins >=50 && alarmmins!=60 && first_snooze==0){
                alarmmins = alarmmins-50;
                first_snooze==1;
            if(alarmhours <= 23){
                alarmhours = alarmhours +1;
            }
            if (alarmhours == 24){
                alarmhours = 0;
              }
            }
            if(alarmmins == 60 && first_snooze==0){
                alarmmins == 0;
                first_snooze==1;
            if(alarmhours <= 23){
                alarmhours = alarmhours +1;
            }
            if (alarmhours == 24){
                alarmhours = 0;
              }
            }
            alarm_update=0;
            RTC_C->AMINHR = alarmhours<<8 | alarmmins | BIT(15) | BIT(7);
            state = clock;
        }
        if(state == settime){
        if( set_time == 1){
            houradjust=1;

         if(RTC_C->TIM1 > 1){
             RTC_C->TIM1 = ((RTC_C->TIM1 & 0x00FF)-1);
                             }
         if(RTC_C->TIM1 == 1)
         {
             RTC_C->TIM1 = 24;
         }
        }
        if( set_time == 2)
        {
            minadjust = 1;

            if((RTC_C->TIM0 & 0xFF00) > 0<<8){
              RTC_C->TIM0 = ((RTC_C->TIM0 & 0xFF00)-1);
                                }
            if((RTC_C->TIM0 & 0xFF00) == 0<<8)
            {
                RTC_C->TIM0 = ((RTC_C->TIM0<<8 & 0xFF00)+59);
            }
        }
        }
        if(state == setalarm){
            if(set_alarm == 1){
                houradjust = 1;
                if(alarmhours >= 0){
               alarmhours = alarmhours - 1;
                }
                if(alarmhours < 0){
                    alarmhours = 23;
                }
            }
            if(set_alarm == 2){
                minadjust = 1;
                if(alarmmins > 0 )
                {
                    alarmmins = alarmmins -1;
                }
               if(alarmmins == 0){
                   alarmmins = 59;
               }
            }
RTC_C->AMINHR = alarmhours<<8 | alarmmins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
        }
    }
    if(status & BIT0) //On/Off/Up
    {
        int next_time = 1;
//        turns the alarm on/off when it hasn't sounded yet
//        turns the alarm off if it is going off
//        turns off the alarm if warm up lights sequence has started 5 min before alarm time
//        acts as the UP button for when times are entered
        if(alarm_update==1)
        {
            TIMER_A0->CCR[0] = 0;                                   //Set output of TimerA to 0
            TIMER_A0->CCR[1] = 0;
            TIMER_A0->CCR[2] = 0;
            alarm_update=0;
            state = clock;
        }
        if(state == settime && set_time == 1){
            houradjust=1;
                if((RTC_C->TIM1>>8) < 24){
                    RTC_C->TIM1 = ((RTC_C->TIM1 & 0x00FF)+1);
                                    }
                if(RTC_C->TIM1 > 24)
                {
                    RTC_C->TIM1 = 1;
                }
               }
        if(state == settime && set_time == 2)
        {
            minadjust = 1;
            if((RTC_C->TIM0 & 0xFF00) <= 59<<8){
                          RTC_C->TIM0 = ((RTC_C->TIM0)+(1<<8));
                                            }
                        if((RTC_C->TIM0 & 0xFF00) == 60<<8)
                        {
                            RTC_C->TIM0 = 0<<8;
                        }
        }
        if(state == setalarm && set_alarm == 1){
            houradjust = 1;
            if(alarmhours <= 23){
                alarmhours = alarmhours +1;
            }
            if (alarmhours == 24){
                alarmhours = 0;
            }
        }
        if(state == setalarm && set_alarm == 2){
            minadjust = 1;
            if(alarmmins <= 59){
                alarmmins = alarmmins +1;
            }
            if(alarmmins == 60){
                alarmmins = 0;
            }
        }
        if(state != settime && state != setalarm)
        {
            if(alarm_status == 1 && next_time==1)
            {
                next_time=0;
                alarm_status = 0;
                RTC_C->AMINHR &= ~(BIT(15)|BIT7);
            }
            if(alarm_status == 0 && next_time==1)
            {
                next_time=0;
                alarm_status = 1;
                RTC_C->AMINHR |= (BIT(15)|BIT7);
            }
        }
RTC_C->AMINHR = alarmhours<<8 | alarmmins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits

    }
}
void initialization(){
SysTick -> CTRL = 0;                    //Systic Timer
SysTick -> LOAD = 0x00FFFFFF;
SysTick -> VAL = 0;
SysTick -> CTRL = 0x00000005;

//Following for LCD************************************************************************************************
P6->SEL0 &= BIT7;                         //DB pins for the LCD screen
P6->SEL1 &= BIT7;
P6->DIR  |= BIT7;
P6->OUT &= ~BIT7;
P6->SEL0 &= BIT6;
P6->SEL1 &= BIT6;
P6->DIR  |= BIT6;
P6->OUT &= ~BIT6;
P6->SEL0 &= BIT5;
P6->SEL1 &= BIT5;
P6->DIR  |= BIT5;
P6->OUT &= ~BIT5;
P6->SEL0 &= BIT4;
P6->SEL1 &= BIT4;
P6->DIR  |= BIT4;
P6->OUT &= ~BIT4;
P1->SEL0 &= BIT5;                                //Enable Pin
P1->SEL1 &= BIT5;
P1->DIR  |= BIT5;
P1->OUT &= ~BIT5;
P1->SEL0 &= BIT6;                                    //Rs
P1->SEL1 &= BIT6;
P1->DIR  |= BIT6;
P1->OUT &=~BIT6;
//LCD done ******************************************************************************************

//Push Buttons***************************************************************************************
//P3.2 second timing    BLACK
//P3.3 minute timing    WHITE
//P3.5 Set Alarm        BLUE sanded
//P3.6 Set Time         GREEN
//P3.7 Snooze/Down      RED
//P3.0 On/Off/Up        BLUE
    P3->SEL0 &= ~(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P3->SEL1 &= ~(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P3->DIR  &= ~(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P3->REN  |=  (BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P3->OUT  |=  (BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P3->IES  |=  (BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P3->IE   |=  (BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);

    P3->IFG = 0;
    NVIC_EnableIRQ(PORT3_IRQn);
//Buttons done***************************************************************************************

//Wake up lights***********************************************************************************
    P7->SEL0 |=  (BIT4|BIT5);                         //Timer A pin
    P7->SEL1 &= ~(BIT4|BIT5);
    P7->DIR  |=  (BIT4|BIT5);

    TIMER_A1->CTL       = 0b0000001000010100;
    TIMER_A1->CCR[0]    = 2999;
    TIMER_A1->CCR[3]    = (3000*duty)-1;
    TIMER_A1->CCR[4]    = (3000*duty)-1;
    TIMER_A1->CCTL[3]   = 0xE0;
    TIMER_A1->CCTL[4]   = 0xE0;

}
void commandWrite(uint8_t command)
{

P1->OUT &= ~BIT6;                            //RS Pin to 0
pushByte(command);

}
/*
 * dataWrite is used to send symbols to the LCD
 */
void dataWrite(uint8_t data)
{

P1->OUT |= BIT6;                               //RS Pin to 1
pushByte(data);
}
void pushByte(uint8_t byte)                                           //referenced from Kandalaf lecture
{
uint8_t nibble;
nibble = (byte & 0xF0)>>4;         //copy in most significant bits by anding with 11110000b
pushNibble(nibble);
nibble = byte & 0x0F;              //copy in least significant bits by anding with 00001111b
pushNibble(nibble);
delay_micro(1000);                  //delay for 100 microseconds
}
void pushNibble (uint8_t nibble)                                          //referenced from Kandalaf Lecture
{
P6->OUT &=~(BIT7|BIT6|BIT5|BIT4); //BIT7|BIT6|BIT5|BIT4
P6->OUT |= (nibble & 0x0F)<<4; //and nibble with 1111b to copy it and then shift it to the left 4 bits

PulseEnablePin();
}
/*
 * PulseEnablePin() allowed for data to be read
 * on the LCD every 10,000 microseconds
 */
void PulseEnablePin()                                                      //referenced from Kandalaf Lecture
{
int microsecond = 10;            //delay 10000 microseconds
P1->OUT &= ~BIT5;               //Enable is zero
delay_micro(microsecond);       //wait 10000 microseconds
P1->OUT |= BIT5;                //Enable is 1
delay_micro(microsecond);       //wait 10000 microseconds
P1->OUT &= ~BIT5;               //enable is 0
delay_micro(microsecond);       //wait 10000 microseconds
}
void delay_ms(unsigned ms)
{
    SysTick -> LOAD = ((ms*3000)-1);   // ms second countdown to 0;
    SysTick -> VAL =0;                    //any write to CVR clears it

       while((SysTick -> CTRL & 0x00010000)==0);

}
void delay_micro(unsigned microsec)
{
    SysTick -> LOAD = ((microsec*3)-1);   // microsecond second countdown to 0;
    SysTick -> VAL =0;                    //any write to CVR clears it

       while((SysTick -> CTRL & 0x00010000)==0);
}
void LCD_init()
{
    commandWrite(3);                               //LCD_init referenced from Lab Write-up
    delay_ms(10);
    commandWrite(3);
    delay_micro(100);
    commandWrite(3);
    delay_ms(10);

    commandWrite(2);
    delay_micro(100);
    commandWrite(2);
    delay_micro(100);

    commandWrite(8);
    delay_micro(100);
    commandWrite(0x0F);
    delay_micro(100);
    commandWrite(1);
    delay_micro(100);
    commandWrite(6);
    delay_ms(10);

    commandWrite(0x0C);
}
void P4_Init_ADC()
{
    P4->SEL0 |= BIT1;                                          //initializes P4.1 to Analog input
    P4->SEL1 |= BIT1;
}
void ADC14_Init()
{
    ADC14->CTL0    =  0;                                       //disables ADC for setup
    ADC14->CTL0   |=  0b10000100001000000000001100010000;      //S/H pulse mode, 16 sample clocks on SMCLK
    ADC14->CTL1    =  0b110000;                                //14 bit resolution
    ADC14->CTL1   |=  0;                                       //converts to MEM0 register
    ADC14->MCTL[0] =  14;                                      //MEM[0] has the value ADC14INCHx = 0
    ADC14->CTL0   |=  0b10;                                    //Enables the ADC
}
void return_ADC()
{
    static volatile uint16_t result;

    ADC14->CTL0 |= ADC14_CTL0_SC;                                       //starts conversion sequence
    while(!(ADC14->IFGR0 & BIT0));
    result = ADC14->MEM[0];                                    //stores ADC value into result
    volt   = (result*3.3)/16384;                               //calculates voltage from ADC value
    celsius = ((volt*1000)-500)/10;
    fahrenheit = ((celsius*9)/5)+32;
    sprintf(temperature," Temp: %.1f F ",fahrenheit);
    commandWrite(0xCF);
    while(!(temperature[i]=='\0'))
    {
        dataWrite(temperature[i]);
        i++;
    }
    i=0;
}
//void ADC_IRQHandler()
//{
//    if(ADC14->IFGR0 & BIT0)                             // Table 20-14. ADC14IFGR0 Register Description of Reference Manual says interrupt flag will be at BIT0 for ADC14MEM0
//       {
//           ADC14->MCTL[1]       =   0;
//           ADC14->MCTL[2]       =   22;
//           ADC14->CLRIFGR0     &=  ~BIT0;                  // Clear MEM0 interrupt flag
//       }
//    ADC14->CLRIFGR1     &=    ~0b1111110;                 // Clear all IFGR1 Interrupts (Bits 6-1.  These could trigger an interrupt and we are checking them for now.)
//}
void wake_Up_Lights()
{
    //float duty = 0.01;
    int x = RTC_C->TIM1 & 0x00FF;
    int y = (alarmmins - ((RTC_C->TIM0 & 0xFF00)>>8));
    int w = (RTC_C->TIM0 & 0x00FF);
    TIMER_A1->CCR[3] = (3000*duty)-1;
    TIMER_A1->CCR[4] = (3000*duty)-1;
    if(x == alarmhours && y <= 5 )
    {
        wakeupsequence = 1;
    }


    if(wakeupsequence)
    {
//               static int check = 0;
//               check = (alarmmins*60) - ((RTC_C->TIM0 & 0x00FF));
//               if(check%3 = 0)
        if(w%3 == 0){
               duty+=0.01;
//               TIMER_A1->CCR[3] = (3000*duty)-1;
//               TIMER_A1->CCR[4] = (3000*duty)-1;
        }
    }
}
void alarm_Status()
{
    if(alarm_status == 1)
    {
        sprintf(astatus, "ALARM ON ");
    }
    if(alarm_status == 0)
    {
        sprintf(astatus, "ALARM OFF");

    }
    commandWrite(0xC0);
    while(!(astatus[i]=='\0'))
    {
        dataWrite(astatus[i]);
        i++;
    }
    i=0;
}
/*-------------------------------------------------------------------------------------------------------------------------------
 *
 * void T32_INT2_IRQHandler()
 *
 * Interrupt Handler for Timer 2.  The name of this function is set in startup_msp432p401r_ccs.c
 *
 * This handler clears the status of the interrupt for Timer32_2
 *
 * Sets up the next note to play in sequence and loads it into TimerA for play back at that frequency.
 * Enables a new Timer32 value to interrupt after the note is complete.
 *
-------------------------------------------------------------------------------------------------------------------------------*/

void T32_INT2_IRQHandler()
{
    TIMER32_2->INTCLR = 1;                                      //Clear interrupt flag so it does not interrupt again immediately.
    if(breath) {                                                //Provides separation between notes
        TIMER_A0->CCR[0] = 0;                                   //Set output of TimerA to 0
        TIMER_A0->CCR[1] = 0;
        TIMER_A0->CCR[2] = 0;
        TIMER32_2->LOAD = BREATH_TIME;                          //Load in breath time to interrupt again
        breath = 0;                                             //Next Timer32 interrupt is no longer a breath, but is a note
    }
    else {                                                      //If not a breath (a note)
        TIMER32_2->LOAD = (music_note_sequence[note][1] - 1)*speed;     //Load into interrupt count down the length of this note
        if(music_note_sequence[note][0] == REST) {              //If note is actually a rest, load in nothing to TimerA
            TIMER_A0->CCR[0] = 0;
            TIMER_A0->CCR[1] = 0;
            TIMER_A0->CCR[2] = 0;
        }
        else {
            TIMER_A0->CCR[0] = 3000000 / music_note_sequence[note][0];  //Math in an interrupt is bad behavior, but shows how things are happening.  This takes our clock and divides by the frequency of this note to get the period.
            TIMER_A0->CCR[1] = 1500000 / music_note_sequence[note][0];  //50% duty cycle
            TIMER_A0->CCR[2] = TIMER_A0->CCR[0];                        //Had this in here for fun with interrupts.  Not used right now
        }
        note = note + 1;                                                //Next note
        if(note >= MAX_NOTE) {                                          //Go back to the beginning if at the end
            note = 0;
        }
        breath = 1;                                             //Next time through should be a breath for separation.
    }
}

/*-------------------------------------------------------------------------------------------------------------------------------
 *
 * void TA0_N_IRQHandler()
 *
 * Interrupt Handler for Timer A0.  The name of this function is set in startup_msp432p401r_ccs.c
 *
 * This handler clears the status of the interrupt for Timer32_A0 CCTL 1 and 2
 *
 * Toggles P2.0 for every interrupt (LED2 Red) through cooperation with Timer_A0
 * Turns on when CCTL1 interrupts.  Turns off with CCTL2 interrupts.
 *
-------------------------------------------------------------------------------------------------------------------------------*/

void TA0_N_IRQHandler()
{
    if(TIMER_A0->CCTL[1] & BIT0) {                  //If CCTL1 is the reason for the interrupt (BIT0 holds the flag)
    }
    if(TIMER_A0->CCTL[2] & BIT0) {                  //If CCTL1 is the reason for the interrupt (BIT0 holds the flag)
    }
}
/*-------------------------------------------------------------------------------------------------------------------------------
 *
 * void SetupTimer32s()
 *
 * Configures Timer32_1 as a single shot (runs once) timer that does not interrupt so the value must be monitored.
 * Configures Timer32_2 as a single shot (runs once) timer that does interrupt and will run the interrupt handler 1 second
 * after this function is called (and the master interrupt is enabled).
 *
-------------------------------------------------------------------------------------------------------------------------------*/

void SetupTimer32s()
{
    TIMER_A0->CCR[0] = 0;                           // Turn off timerA to start
    TIMER_A0->CCTL[1] = 0b0000000011110100;         // Setup Timer A0_1 Reset/Set, Interrupt, No Output
    TIMER_A0->CCR[1] = 0;                           // Turn off timerA to start
    TIMER_A0->CCTL[2] = 0b0000000011110100;         // Setup Timer A0_2 Reset/Set, Interrupt, No Output
    TIMER_A0->CCR[2] = 0;                           // Turn off timerA to start
    TIMER_A0->CTL = 0b0000001000010100;             // Count Up mode using SMCLK, Clears, Clear Interrupt Flag

    P2->SEL0 |= BIT4;                               // Setup the P2.4 to be an output for the song.  This should drive a sounder.
    P2->SEL1 &= ~BIT4;
    P2->DIR |= BIT4;
}

void P2_Init()
{
    P2->SEL0 |=  BIT0;
    P2->SEL1 &= ~BIT0;
    P2->DIR  |=  BIT0;
    P2->OUT  &= ~BIT0;
}
void play_alarm()
{
    int time = secs;
    if(time%2==0 && alarm_update==1 && alarm_status==1){
        TIMER_A0->CCR[0] = 9000;
        TIMER_A0->CCR[1] = 4500;
        TIMER_A0->CCR[2] = TIMER_A0->CCR[0];
    }
    else
    {
        TIMER_A0->CCR[0] = 0;
        TIMER_A0->CCR[1] = 0;
        TIMER_A0->CCR[2] = 0;
    }
}
