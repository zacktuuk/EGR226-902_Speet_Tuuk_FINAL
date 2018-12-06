#include "msp.h"
#include <stdio.h>
#include <string.h>
/*
 * Dylan Speet and Zachary Tuuk
 * 11/27/2018 Start 12/5/2018 End
 * Alarm Clock Final Project for EGR 226-902
 * Uses timerA, Real time clock, Interrupts, DAC, and ADC
 * Buttons, LEDs, LCD, Potentiometer, Speaker
 * Varios codes from Prof. Zuidema, Prof Kandalaft and Prof. Brakora
 * were referenced to write this code.
 */
enum states{
    setalarm,
    settime,
    clock,
    snooze,
    alarm
};

// Making a buffer of 100 characters for serial to store to incoming serial data
#define BUFFER_SIZE 100
char INPUT_BUFFER[BUFFER_SIZE];
// initializing the starting position of used buffer and read buffer
uint8_t storage_location = 0; // used in the interrupt to store new data
uint8_t read_location = 0; // used in the main application to read valid data that hasn't been read yet

void writeOutput(char *string); // write output charactrs to the serial port
void readInput(char* string); // read input characters from INPUT_BUFFER that are valid
void setup(); // Sets up P1.0 as an output to drive the on board LED
void setupSerial(); // Sets up serial for use and enables interrupts

char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands
    char amin[3];
    char ahour[3];
    char cmin[3];
    char chour[3];
    char csec[3];
    int currentmin, currenthour, currentsec;
    int b = 0, c = 0, newcomm = 0;


enum states state = clock;
int time_update = 0, alarm_update = 0, i = 0, time_set = 0, ampm = 1, hours, mins, set_time = 0, houradjust = 0, minadjust = 0, set_alarm = 0, alarm_status = 1;
uint8_t  secs, hour_update;
int alarmhours = 14, alarmmins = 46, status = 0, wakeupsequence = 0, alarm_sound;
char m, a;
float volt, celsius, fahrenheit, duty = 0.0004;
char temperature[12];
char astatus[10];
char currenttime[11];
char alarmtime[14];

void get_serial();
void Setupspeaker();
void alarm_Status();
void return_ADC();
void P6_Init_ADC();
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

char doublespace[2] = "  ";                                //used to write blanks
    initialization();                               //initialize all pins, timers, and interrupts
    RTC_Init();
    P6_Init_ADC();
    ADC14_Init();
    setupSerial();
      INPUT_BUFFER[0]= '\0';
    __enable_interrupt();
    LCD_init();
    Setupspeaker();


while (1){
    if(newcomm){                          //Checks if line feed from the serial port
    get_serial();
    newcomm = 0;
    }

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
                              sprintf(alarmtime,"ALARM: %01d:%02d AM",alarmhours,alarmmins);       //print alarm till null
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
/*
 * RTC_Init()
 * Written by Zack Tuuk and Dylan Speet Referenced from Prof Zuidemas Example Code
 * Initializes the Real time Clock and its alarm, including interrupts.
 */
void RTC_Init(){
    //Initialize time to 2:45:55 pm
//    RTC_C->TIM0 = 0x2D00;  //45 min, 0 secs
    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = 40<<8 | 45;//45 min, 55 secs
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
/*
 * RTC_C_IRQHandler()
 * Interrupt Handler was taken from msp.h
 * Contents written by Zack Tuuk and Dylan Speet, some bit shifting referenced from Prof. Zuidema
 * Makes sure the time values are stored correctly and increments them accordingly.
 */
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
        if(hours==RTC_C->AMINHR )
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
        //if()
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
/*
 * Port3_IRQHandler()
 * Function created in library
 * Contents written by Zack Tuuk and Dylan Speet
 * This handler determines which button was pressed and based on which button was pressed
 * if statements set flags in order to set the time,snooze,set the alarm, turn off the alarm,
 * turn on the alarm etc..
 */
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
            alarm_status = 2;
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
            if((alarm_status == 1 || alarm_status == 2) && next_time==1 )
            {
                next_time=0;
                alarm_status = 0;
                RTC_C->AMINHR &= ~(BIT(15)|BIT7);
            }
            if((alarm_status == 0 || alarm_status == 2) && next_time==1)
            {
                next_time=0;
                alarm_status = 1;
                RTC_C->AMINHR |= (BIT(15)|BIT7);
            }

        }
RTC_C->AMINHR = alarmhours<<8 | alarmmins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits

    }
}
/*
 * initialization()
 * Written by Zack Tuuk and Dylan Speet
 * Initializes systick timer, LCD pins and button pins
 */
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

}
/*
 * commandWrite()
 * Referenced from Prof. Zuidema written by Zack Tuuk and Dylan Speet
 * Sends a write command to the LCD.
 */
void commandWrite(uint8_t command)
{

P1->OUT &= ~BIT6;                            //RS Pin to 0
pushByte(command);

}
/*
 * dataWrite()
 * Referenced from Prof. Zuidema written by Zack Tuuk and Dylan Speet
 * dataWrite is used to send symbols to the LCD
 */
void dataWrite(uint8_t data)
{

P1->OUT |= BIT6;                               //RS Pin to 1
pushByte(data);
}
/*
 * pushByte()
 * Referenced from Prof. Zuidema and written by Zack Tuuk and Dylan Speet
 * Sends a byte of data to the LCD
 */
void pushByte(uint8_t byte)                                           //referenced from Kandalaf lecture
{
uint8_t nibble;
nibble = (byte & 0xF0)>>4;         //copy in most significant bits by anding with 11110000b
pushNibble(nibble);
nibble = byte & 0x0F;              //copy in least significant bits by anding with 00001111b
pushNibble(nibble);
delay_micro(1000);                  //delay for 100 microseconds
}
/*
 * pushNibble()
 * Referenced from Prof. Zuidema and written by Zack Tuuk and Dylan Speet
 * Sends a nibble to the LCD
 */
void pushNibble (uint8_t nibble)                                          //referenced from Kandalaf Lecture
{
P6->OUT &=~(BIT7|BIT6|BIT5|BIT4); //BIT7|BIT6|BIT5|BIT4
P6->OUT |= (nibble & 0x0F)<<4; //and nibble with 1111b to copy it and then shift it to the left 4 bits

PulseEnablePin();
}
/*
 * PulseEnablePin()
 * Referenced from Prof. Zuidema and written by Zack Tuuk and Dylan Speet
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
/*
 * delay_ms
 * Written by Zack Tuuk and Dylan Speet
 * delay for a millisecond amount
 */
void delay_ms(unsigned ms)
{
    SysTick -> LOAD = ((ms*3000)-1);   // ms second countdown to 0;
    SysTick -> VAL =0;                    //any write to CVR clears it

       while((SysTick -> CTRL & 0x00010000)==0);

}
/*
 * delay_micro()
 * Written by Zack Tuuk and Dylan Speet
 * Delay for a microsecond amount
 */
void delay_micro(unsigned microsec)
{
    SysTick -> LOAD = ((microsec*3)-1);   // microsecond second countdown to 0;
    SysTick -> VAL =0;                    //any write to CVR clears it

       while((SysTick -> CTRL & 0x00010000)==0);
}
/*
 * LCD_init()
 * Referenced from Prof. Zuidema, Prof. Brakora and Prof. Kandalaft, written by Zack Tuuk and Dylan Speet
 * Initializes the LCD and blinks the cursor.
 * Cursor is turned off at the end.
 */
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

    commandWrite(0x0C);                                      // makes cursor invisible
}
/*
 * P6_Init_ADC()
 * Written by Zack Tuuk and Dylan Speet
 * Intializes P6.1 to analog read
 */
void P6_Init_ADC()
{
    P6->SEL0 |= BIT1;                                          //initializes P6.1 to Analog input
    P6->SEL1 |= BIT1;
}
/*
 * ADC14_Init()
 * Written by Zack Tuuk and Dylan Speet referenced from Prof. Zuidemas lecture code
 * Sets up ADC to read temperature sensor.
 */
void ADC14_Init()
{
    ADC14->CTL0    =  0;                                       //disables ADC for setup
    ADC14->CTL0   |=  0b10000100001000000000001100010000;      //S/H pulse mode, 16 sample clocks on SMCLK
    ADC14->CTL1    =  0b110000;                                //14 bit resolution
    ADC14->CTL1   |=  0;                                       //converts to MEM0 register
    ADC14->MCTL[0] =  14;                                      //MEM[0] has the value ADC14INCHx = 0
    ADC14->CTL0   |=  0b10;                                    //Enables the ADC
    NVIC_EnableIRQ(ADC14_IRQn);
}
/*
 * return_ADC()
 * Referenced from Prof. Zuidema and written by Dylan Speet and Zack Tuuk
 * Gets a reading in from MEM[0] and converts it to voltage and then to fahrenheit
 */
void return_ADC()
{
    static volatile uint16_t result;
    ADC14->CTL0 |= BIT0;
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
/*
 * ADC_IRQHandler()
 * Written by Zack Tuuk and Dylan Speet
 * Gets the readings from the ADC
 */
void ADC_IRQHandler()
{
    if(ADC14->IFGR0 & BIT0)                             // Table 20-14. ADC14IFGR0 Register Description of Reference Manual says interrupt flag will be at BIT0 for ADC14MEM0
       {
           ADC14->MCTL[1]       =   0;
           ADC14->MCTL[2]       =   22;
           ADC14->CLRIFGR0     &=  ~BIT0;                  // Clear MEM0 interrupt flag
       }
    ADC14->CLRIFGR1     &=    ~0b1111110;                 // Clear all IFGR1 Interrupts (Bits 6-1.  These could trigger an interrupt and we are checking them for now.)
}
/*
 * alarm_Status()
 * Written by Dylan Speet and Zack Tuuk
 * Depending on what the Alarm status is, the function prints
 * the status to the LCD.
 */
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
    if(alarm_status == 2){
        sprintf(astatus, "SNOOZE   ");
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
 * void Setupspeaker()
 *
 *Referenced from Prof. Zuidema Assignment 5, Written by Zack Tuuk and Dylan Speet
 *
 * Configures Timer32_1 as a single shot (runs once) timer that does not interrupt so the value must be monitored.
 * Configures Timer32_2 as a single shot (runs once) timer that does interrupt and will run the interrupt handler 1 second
 * after this function is called (and the master interrupt is enabled).
 *
-------------------------------------------------------------------------------------------------------------------------------*/

void Setupspeaker()
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
/*
 * play_alarm()
 * Written by Zack Tuuk and Dylan Speet
 * Plays the alarm sound
 */
void play_alarm()
{
    int time = secs;
    if(time%2==0 && alarm_update==1 && (alarm_status==1 || alarm_status == 2)){
        TIMER_A0->CCR[0] = 9000;
        TIMER_A0->CCR[1] = 4500;
    }
    else
    {
        TIMER_A0->CCR[0] = 9000;
        TIMER_A0->CCR[1] = 0;
    }
}
/*----------------------------------------------------------------
 * void writeOutput(char *string)
 *
 *Referenced from Prof. Zuidema, Written by Dylan Speet and Zack Tuuk
 *
 * Description:  This is a function similar to most serial port
 * functions like printf.  Written as a demonstration and not
 * production worthy due to limitations.
 * One limitation is poor memory management.
 * Inputs: Pointer to a string that has a string to send to the serial.
 * Outputs: Places the data on the serial output.
----------------------------------------------------------------*/
void writeOutput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to

    while(string[i] != '\0') {
        EUSCI_A0->TXBUF = string[i];
        i++;
        while(!(EUSCI_A0->IFG & BIT1));
    }
}

/*----------------------------------------------------------------
 * void readInput(char *string)
 *
 *Referenced from Prof. Zuidema, Written by Dylan Speet and Zack Tuuk
 *
 * Description:  This is a function similar to most serial port
 * functions like ReadLine.  Written as a demonstration and not
 * production worthy due to limitations.
 * One of the limitations is that it is BLOCKING which means
 * it will wait in this function until there is a \n on the
 * serial input.
 * Another limitation is poor memory management.
 * Inputs: Pointer to a string that will have information stored
 * in it.
 * Outputs: Places the serial data in the string that was passed
 * to it.  Updates the global variables of locations in the
 * INPUT_BUFFER that have been already read.
----------------------------------------------------------------*/
void readInput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to

    // One of the few do/while loops I've written, but need to read a character before checking to see if a \n has been read
    do
    {
        // If a new line hasn't been found yet, but we are caught up to what has been received, wait here for new data
        while(read_location == storage_location && INPUT_BUFFER[read_location] != '\n');
        string[i] = INPUT_BUFFER[read_location];  // Manual copy of valid character into "string"
        INPUT_BUFFER[read_location] = '\0';
        i++; // Increment the location in "string" for next piece of data
        read_location++; // Increment location in INPUT_BUFFER that has been read
        if(read_location == BUFFER_SIZE)  // If the end of INPUT_BUFFER has been reached, loop back to 0
            read_location = 0;
    }
    while(string[i-1] != '\n'); // If a \n was just read, break out of the while loop

    string[i-1] = '\0'; // Replace the \n with a \0 to end the string when returning this function
}

/*----------------------------------------------------------------
 * void EUSCIA0_IRQHandler(void)
 *
 *Referenced from Prof Zuidema, written by Dylan Speet and Zack Tuuk
 *
 * Description: Interrupt handler for serial communication on EUSCIA0.
 * Stores the data in the RXBUF into the INPUT_BUFFER global character
 * array for reading in the main application
 * Inputs: None (Interrupt)
 * Outputs: Data stored in the global INPUT_BUFFER. storage_location
 * in the INPUT_BUFFER updated.
----------------------------------------------------------------*/
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & BIT0)  // Interrupt on the receive line
    {
        INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF; // store the new piece of data at the present location in the buffer
        if(EUSCI_A0->RXBUF == '\n'){
            newcomm = 1;
        }
        EUSCI_A0->IFG &= ~BIT0; // Clear the interrupt flag right away in case new data is ready
        storage_location++; // update to the next position in the buffer
        if(storage_location == BUFFER_SIZE) // if the end of the buffer was reached, loop back to the start
            storage_location = 0;
    }
}
/*----------------------------------------------------------------
 * void setupSerial()
 *
 * Referenced from Prof. Zuidema and Written by Zack Tuuk and Dylan Speet
 *
 * Sets up the serial port EUSCI_A0 as 115200 8E2 (8 bits, Even parity,
 * two stops bit.)  Enables the interrupt so that received data will
 * results in an interrupt.
 * Description:
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void setupSerial()
{
    P1->SEL0 |=  (BIT2 | BIT3); // P1.2 and P1.3 are EUSCI_A0 RX
    P1->SEL1 &= ~(BIT2 | BIT3); // and TX respectively.

    EUSCI_A0->CTLW0  = BIT0; // Disables EUSCI. Default configuration is 8N1
    EUSCI_A0->CTLW0 |= BIT7; // Connects to SMCLK BIT[7:6] = 10
//    EUSCI_A0->CTLW0 |= (BIT(15)|BIT(14)|BIT(11));  //BIT15 = Parity, BIT14 = Even, BIT11 = Two Stop Bits
    // Baud Rate Configuration
    // 3000000/(16*115200) = 1.628  (3 MHz at 115200 bps is fast enough to turn on over sampling (UCOS = /16))
    // UCOS16 = 1 (0ver sampling, /16 turned on)
    // UCBR  = 1 (Whole portion of the divide)
    // UCBRF = .628 * 16 = 10 (0x0A) (Remainder of the divide)
    // UCBRS = 3000000/115200 remainder=0.04 -> 0x01 (look up table 22-4)
    EUSCI_A0->BRW = 19;  // UCBR Value from above
    EUSCI_A0->MCTLW = 0xAA81; //UCBRS (Bits 15-8) & UCBRF (Bits 7-4) & UCOS16 (Bit 0)

    EUSCI_A0->CTLW0 &= ~BIT0;  // Enable EUSCI
    EUSCI_A0->IFG &= ~BIT0;    // Clear interrupt
    EUSCI_A0->IE |= BIT0;      // Enable interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);
}
/*
 * get_serial()
 * Written by Dylan Speet and Zack Tuuk
 * Gets whatever is sent on the serial port and sets the time or alarm.
 * Also sends back to the serial what is currently on clock and alarm.
 */
void get_serial(){
    readInput(string); // Read the input up to \n, store in string.  This function doesn't return until \n is received
        if(string[0] != '\0'){ // if string is not empty, check the inputted data.
                 if(string[0] == 'S'){
                     if(string[3] == 'A'){
                         //set the alarm
                        for(b=9,c=0;b<=10,c<=1;b++,c++){
                            ahour[c]=string[b];
                        }
                          alarmhours = atoi(ahour);
                        for(b=12,c=0;b<=13,c<=1;b++,c++){
                            amin[c] = string[b];
                        }
                        alarmmins = atoi(amin);

                        RTC_C->AMINHR = alarmhours<<8 | alarmmins | BIT(15) | BIT(7);

                     }
                     if(string[3] == 'T'){
                         //set the time
                         for(b=8,c=0;b<=9,c<=1;b++,c++)
                        {
                            chour[c] = string[b];
                        }
                         currenthour = atoi(chour);

                         for(b=11,c=0;b<=12,c<=1;b++,c++)
                         {
                             cmin[c] = string[b];
                         }
                         currentmin = atoi(cmin);

                         for(b=14,c=0;b<=15,c<=1;b++,c++){
                             csec[c] = string[b];
                         }
                         currentsec = atoi(csec);

                         RTC_C->TIM0 = currentmin<<8 | currentsec;//45 min, 55 secs
                         RTC_C->TIM1 = 1<<8 | currenthour;  //Monday, 2 pm
                     }
                 }
                  if(string[0] == 'R'){
                      if(string[4] == 'A'){
                          //write output the current alarm
                          writeOutput(alarmtime);
                      }
                      if(string[4] == 'T'){
                          //write output the current time
                          writeOutput(currenttime);
                      }
                  }
               }
}
