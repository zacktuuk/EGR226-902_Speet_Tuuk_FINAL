#include "msp.h"
#include <stdio.h>
#include <string.h>
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
    snooze
};
enum states state = clock;
int time_update = 0, alarm_update = 0, i = 0, time_set = 0;
uint8_t hours, mins, secs, hour_update;

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
char temperature[11];
char alarmset[11];
    initialization();                               //initialize all pins, timers, and interrupts
    RTC_Init();
    __enable_interrupt();
    LCD_init();

while (1){
    switch (state){
    case clock:
        if(time_update){
            commandWrite(0xC1);

                sprintf(currenttime,"%02d:%02d:%02d XM",hours,mins,secs);
                while(!(currenttime[i]=='\0')){                            //print my name until null
                                 dataWrite(currenttime[i]);
                                 i++;
                                 }
                                 i=0;
                    printf("%02d:%02d:%02d\n",hours,mins,secs);
                }
                if(alarm_update){
                    printf("ALARM\n");
                    alarm_update = 0;
                }
        break;
    }
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
delay_micro(10000);                  //delay for 100 microseconds
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
}
void RTC_Init(){
    //Initialize time to 2:45:55 pm
//    RTC_C->TIM0 = 0x2D00;  //45 min, 0 secs
    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = 45<<8 | 55;//45 min, 55 secs
    RTC_C->TIM1 = 1<<8 | 14;  //Monday, 2 pm
    RTC_C->YEAR = 2018;
    //Alarm at 2:46 pm
    RTC_C->AMINHR = 14<<8 | 46 | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b0010;  //1/64 second interrupt is 0b0010 a 1 second interupt is

    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;
    NVIC_EnableIRQ(RTC_C_IRQn);

}

void RTC_C_IRQHandler()
{
    if(time_set == 1)
    {
        if(mins>59)
        {
        hour_update = hours;
        hour_update++;
        if(hour_update>23)
            hour_update=0;

        if(RTC_C->PS1CTL & BIT0){
            hours = RTC_C->TIM1 & 0x00FF;
            mins = (RTC_C->TIM0 & 0xFF00) >> 8;
            secs = RTC_C->TIM0 & 0x00FF;
            if(secs != 59){
                RTC_C->TIM0 = RTC_C->TIM0 + 1;
        }
        else {
            RTC_C->TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;
            time_update = 1;
        }
        RTC_C->PS1CTL &= ~BIT0;
        }
    }
    if(time_set == 0)
    {
        if(RTC_C->PS1CTL & BIT0){
            hours = RTC_C->TIM1 & 0x00FF;
            mins = (RTC_C->TIM0 & 0xFF00) >> 8;
            secs = RTC_C->TIM0 & 0x00FF;
                time_update = 1;
            RTC_C->PS1CTL &= ~BIT0;
        }
    }
    if(RTC_C->CTL0 & BIT1)
    {
        alarm_update = 1;
        RTC_C->CTL0 = (0xA500) | BIT5;
    }
}
void PORT3_IRQHandler(){
    int status = P3->IFG;
    //int time_set = 1;
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
        //sets the alarm time for the RTC
    }
    if(status & BIT6) //set time
    {
        //sets the current time for the RTC
    }
    if(status & BIT7) //snooze/down
    {
        //sets the alarm for 10 minutes later for the snooze function
        //acts as the DOWN button for when times are entered
    }
    if(status & BIT0) //On/Off/Up
    {
        //turns the alarm on/off when it hasn't sounded yet
        //turns the alarm off if it is going off
        //turns pff the alarm if warm up lights sequence has started 5 min before alarm time
        //acts as the UP button for when times are entered
    }
}
