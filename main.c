// Periodic timer example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "uart4.h"
#include "uart0.h"
#include "wait.h"
#include "eeprom.h"
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint32_t n1;
uint32_t n2;
uint32_t i;
uint16_t device_add;
uint16_t rampDevice_add;
uint16_t pulseDevice_add;
uint16_t time;
uint16_t difference;
uint16_t start[512];
uint16_t stop[512];
uint16_t first;
uint16_t last;
uint16_t pulseCount;
volatile uint16_t rampCount[512];
uint16_t count;
volatile uint16_t updatePerCyc[512];
uint32_t pulsetime;
uint16_t DMX_Data[512] = {0,};
volatile char RX_buffer[512];
volatile bool on = 0;
uint32_t index2 = 0;
volatile bool brk = false;
volatile bool sc = false;
volatile uint16_t DMX_max = 512;
#define MAX_CHARS 80
#define MAX_FIELDS 6
typedef struct USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

// Portc masks

#define DE_LED_MASK 64

#define DE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) // pc6

// PortF masks
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8

void getsUart0(USER_DATA *data);



void parseFields(USER_DATA *data)
{
    char PType = 'd';
    data->fieldCount = 0;
    uint8_t i = 0;

    while(true)
    {
        if(data->buffer[i]=='\0'){
            break;
        }
        char currentChar = data->buffer[i];
        char CType = 'd';
        if((currentChar >= 65 && currentChar<= 90)|| (currentChar >= 97 && currentChar <=122)){
            CType = 'a';
        }
        else if (currentChar >= 48 && currentChar <= 57){
            CType = 'n';
        }
        else{
            data->buffer[i] = '\0';
            //CType='d';
        }
        if (CType != PType)
        {
            if (data->fieldCount < MAX_FIELDS && CType != 'd')
            {
                // Record the type and position of the field
                data->fieldType[data->fieldCount] = CType;
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
            }
            PType = CType; // Update previous type

        }
        i++;
    }
}
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{

    if (fieldNumber <= data->fieldCount)
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return NULL;
    }
}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if (fieldNumber < data->fieldCount && data->fieldType[fieldNumber] == 'n')
    {

        int32_t fieldValue = atoi(&data->buffer[data->fieldPosition[fieldNumber]]);

        return fieldValue;
    }
    else
    {
        return 0;
    }
}

bool compare_string(char str1[], char str2[]){

    uint8_t i=0, j=0;

    while (str1[i] != '\0' && str2[j] != '\0')
    {
        if (str1[i] != str2[j])
            return false;
        i++;
        j++;
    }

    if(i == j)
        return true;
    else
        return false;
}

bool isCommand(USER_DATA* data, char strCommand[], uint8_t minArguments)
{
    if (data->fieldCount >= minArguments + 1)
    {
        if(compare_string(strCommand, &data->buffer[data->fieldPosition[0]]))
        {
            return true;
        }
    }

    return false;
}
void clear(){
    uint16_t i;

    for(i = 0; i < 512; i++)
    {
        DMX_Data[i] = 0x00;
    }
}

// Initialize Hardware
void initLED()
{ // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_AFSEL_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb

    PWM1_2_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)

    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}
void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}
// oneshot timer
void initHwT()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure LED pin
    GPIO_PORTC_DIR_R |= DE_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTC_DR2R_R |= DE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= DE_LED_MASK;  // enable LEDs

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_16_BIT;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for ONE SHOT MODE
    TIMER1_TAILR_R = 3680;                       // set load value to 40e6 for  92us interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
}
void ininHwT2()
{
    // Enable clocks
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
        _delay_cycles(3);

        // Configure Timer 1 as the time base
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER2_CFG_R = TIMER_CFG_16_BIT;           // configure as 32-bit timer (A+B)
        TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for periodic mode (count down)
        TIMER2_TAILR_R = 40000;                       // set load value to 40e6 for 1 Hz interrupt rate
        TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
        //TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
        NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC

}
void ininHwT3()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER3_CFG_R = TIMER_CFG_16_BIT;           // configure as 32-bit timer (A+B)
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for periodic mode (count down)
    TIMER3_TAILR_R = 40000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER3_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN1_R |= 1 << (INT_TIMER3A-16-32);              // turn-on interrupt 37 (TIMER1A) in NVIC
}
void timer3Isr()
{
    UART4_LCRH_R &= ~UART_LCRH_BRK; //MAB
    waitMicrosecond(12);
    while(UART4_DR_R & UART_FR_BUSY);

    UART4_DR_R = 0X00;   ///startcode
    while(UART4_DR_R & UART_FR_BUSY);



    int16_t index4;
    for (index4 = 0; index4 <= DMX_max; index4++)
    {
      while (UART4_FR_R & UART_FR_BUSY);
      UART4_DR_R = DMX_Data[index4];
      setRgbColor(0, 1023, 0);
    }
        while(UART4_DR_R & UART_FR_BUSY);
    waitMicrosecond(pulsetime);
    if(DMX_Data[rampDevice_add] == first)
    {
        DMX_Data[rampDevice_add] =last;
    }
    else
    {
        DMX_Data[rampDevice_add] = first;
    }
    pulseCount++;
    if(pulseCount <= count){
    if(on == 1)
      {
        UART4_LCRH_R |=  UART_LCRH_BRK;   //brk
        TIMER3_CTL_R |= TIMER_CTL_TAEN;
      }
    }

       TIMER3_ICR_R |= TIMER_ICR_TATOCINT;// clear interrupt flag
       setRgbColor(0,0,0);
}
void timer2Isr()
{
    UART4_LCRH_R &= ~UART_LCRH_BRK; //MAB
    waitMicrosecond(12);
    while(UART4_DR_R & UART_FR_BUSY);

    UART4_DR_R = 0X00;   ///startcode
    while(UART4_DR_R & UART_FR_BUSY);

    int16_t index3;
    for (index3 = 0; index3 <= DMX_max; index3++)
    {
      while (UART4_FR_R & UART_FR_BUSY);
      UART4_DR_R = DMX_Data[index3];
      setRgbColor(0,1023, 0);
    }
    while(UART4_DR_R & UART_FR_BUSY);
    int16_t j;

    for(j = 0 ; j <= 512; j++){

    if(DMX_Data[j] <= stop[j])
    {
        DMX_Data[j] = DMX_Data[j] + updatePerCyc[j];

    }
    if(DMX_Data[j] == stop[j])
    {
        DMX_Data[j] = start[j];
        rampCount[j] = rampCount[j] - 1;
    }

   if(rampCount[j] == 0)
    {
        updatePerCyc[j] = 0;
    }
   }
    if(on == 1)
    {
        UART4_LCRH_R |=  UART_LCRH_BRK;   //brk
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
    }

     TIMER2_ICR_R |= TIMER_ICR_TATOCINT;// clear interrupt flag
     setRgbColor(0,0,0);
}

// timerISR
void timer1Isr()
{
    UART4_LCRH_R &= ~UART_LCRH_BRK; //MAB
        waitMicrosecond(12);

    while(UART4_DR_R & UART_FR_BUSY);

    UART4_DR_R = 0X00;   ///startcode
    while(UART4_DR_R & UART_FR_BUSY);
    int16_t index;

       for (index = 0; index <= DMX_max; index++)
       {
           while (UART4_FR_R & UART_FR_BUSY);
           setRgbColor(0, 1023, 0);
           UART4_DR_R = DMX_Data[index];
       }
       while(UART4_DR_R & UART_FR_BUSY);
       setRgbColor(0, 0, 0);

       if(on == 1)
       {
           UART4_LCRH_R |=  UART_LCRH_BRK;   //brk
           TIMER1_CTL_R |= TIMER_CTL_TAEN;
       }

    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;// clear interrupt flag
    setRgbColor(0,0,0);

}
void uart4Isr()
{
    if(UART4_DR_R & UART_DR_BE)
    {
        brk = true;
        sc = false;
        index2 = 0;
       // UART4_ICR_R = UART_ICR_BEIC;
    }
    else
    {
        if((brk == true) && (sc == true))
        {

            if(index2 == device_add)
            {
                RX_buffer[index2] = UART4_DR_R & 0xFF;
                setRgbColor(RX_buffer[device_add],0,0);
            }
            //
            index2++;
        }
        else
        {
            if((brk == true) && ((UART4_DR_R & 0xFF) == 0x00))
            {
                sc = true;
            }
        }
    }

    //setRgbColor(0,0,0);
    UART4_ICR_R = UART_ICR_RXIC;

}

/**
 * main.c
 */
int main(void)
{
       initSystemClockTo40Mhz();
       initHwT();
       initUart0();
       initUart4();
       initEeprom();
       initLED();
       ininHwT2();
       ininHwT3();
       // Initialize system clock to 40 MHz
       setUart0BaudRate(115200, 40e6);
       DMX_max = 512;
       USER_DATA data;

       //set default to controller mode
       writeEeprom(1,0);
       if(readEeprom(1) == 0){
           putsUart0("in controller mode\r\n");
       }
       // Endless loop
           while (true)
           {
               if (kbhitUart0() == 1)
                   //setRgbColor(1023,0,0);
               {
                   //putsUart0("enter text:\n");
                   getsUart0(&data);
                   parseFields(&data);
                   setRgbColor(1023, 0, 0);

                   if(isCommand(&data,"controller",0))
                   {
                       DE_LED = 1;
                       waitMicrosecond(12000);
                       writeEeprom(1,0);  //writeEeprom for controller mode
                       putsUart0("in controller mode");
                       UART4_CTL_R &= ~(UART_CTL_RXE);                   //off Rx
                       UART4_CTL_R |= UART_CTL_TXE;        //on Tx
                   }
                   if(isCommand(&data,"pulse",5))
                   {
                       pulseCount = 0;
                       rampDevice_add = getFieldInteger(&data,1);
                       time = getFieldInteger(&data,2);
                       first = getFieldInteger(&data,3);
                       last = getFieldInteger(&data,4);
                       count = getFieldInteger(&data,5);
                       count = count*2;
                       DMX_Data[rampDevice_add] = first;
                       pulsetime = time*1000;
                       on = 1;
                       if(readEeprom(1) == 0)
                       {
                           UART4_CTL_R &= ~(UART_CTL_RXE);                   //off Rx
                           UART4_CTL_R |= UART_CTL_TXE;        //on Tx
                           DE_LED = 1;
                           waitMicrosecond(1000000);
                           putsUart0("ok");
                           UART4_LCRH_R |=  UART_LCRH_BRK;     //brk
                           TIMER3_CTL_R |= TIMER_CTL_TAEN;     //TURN-ON TIMER
                       }

                   }
                   if(isCommand(&data,"ramp",5))
                   {
                       rampDevice_add = getFieldInteger(&data,1);
                       time = getFieldInteger(&data,2);
                       start[rampDevice_add] = getFieldInteger(&data,3);
                       stop[rampDevice_add] = getFieldInteger(&data,4);
                       rampCount[rampDevice_add] = 0;
                       rampCount[rampDevice_add] = getFieldInteger(&data,5);
                       difference = stop[rampDevice_add] - start[rampDevice_add];
                       updatePerCyc[rampDevice_add] = difference / time;
                       DMX_Data[rampDevice_add] = start[rampDevice_add];
                       on = 1;
                       if(readEeprom(1) == 0)
                       {
                        UART4_CTL_R &= ~(UART_CTL_RXE);                   //off Rx
                        UART4_CTL_R |= UART_CTL_TXE;        //on Tx
                        DE_LED = 1;
                        waitMicrosecond(1000000);
                        putsUart0("ok");
                        UART4_LCRH_R |=  UART_LCRH_BRK;     //brk
                        TIMER2_CTL_R |= TIMER_CTL_TAEN;     //TURN-ON TIMER
                       }

                   }

                   if(isCommand(&data,"on",0))
                   {
                       on = 1;
                       // its in controller mode or not
                       if(readEeprom(1) == 0)
                       {
                           UART4_CTL_R &= ~(UART_CTL_RXE);                   //off Rx
                           UART4_CTL_R |= UART_CTL_TXE;        //on Tx
                           DE_LED = 1;
                           waitMicrosecond(1000000);
                           putsUart0("ok");
                           UART4_LCRH_R |=  UART_LCRH_BRK;     //brk
                           TIMER1_CTL_R |= TIMER_CTL_TAEN;     //TURN-ON TIMER
                         //  putsUart0("in controller mode on\r\n");
                       }
                       else
                       {
                           putsUart0("mode = device so invalid command\r\n");
                       }
                   }

                   if(isCommand(&data,"off",0))
                   {
                       // its in controller mode or not
                       on = 0;
                   }

                   if(isCommand(&data,"device",1))
                   {
                       on = 0; //stop timerisr
                       DE_LED = 0; //off data enable pin
                       waitMicrosecond(1000000);
                       writeEeprom(1,1);  //write in eeprom that device in device mode

                       device_add = getFieldInteger(&data,1);
                       writeEeprom(2,device_add);  //in eep store device add
                       putsUart0("in device mode");
                       UART4_CTL_R &= ~(UART_CTL_TXE);        //off Tx
                       UART4_CTL_R |= UART_CTL_RXE;                   //on Rx
                   }

                   if(isCommand(&data, "clear", 0))
                   {
                       clear();
                   }

                   if(isCommand(&data,"set", 2))
                   {
                       n1 = getFieldInteger(&data,1);
                       n2 = getFieldInteger(&data, 2);
                       DMX_Data[n1] = n2;
                       putsUart0("ok");
                   }

                   if (isCommand(&data, "get", 1))
                       {
                           uint16_t p=0;
                           p = getFieldInteger(&data, 1);

                           char s[20];
                           snprintf(s, sizeof(s), "%d", DMX_Data[p]);
                           putsUart0(s);
                       }

                   if(isCommand(&data,"max", 1))
                   {
                       char max[10];
                       uint16_t maxnum;
                       maxnum = getFieldInteger(&data,1);

                       if(maxnum < 512)
                       {
                           DMX_max = maxnum;
                           sprintf(max,"%d",maxnum);
                           putsUart0(&max[0]);
                       }
                       else
                       {
                           putsUart0("not valid");
                       }
                   }
               }
                setRgbColor( 0, 0, 0);
           }
       }
void getsUart0(USER_DATA *data)
{
    uint8_t count = 0;
    char c;

    while (true)
    {
        c = getcUart0();

        if (c == 8 || c == 127) //8 or 127 for backspace
        {
            if (count > 0)
            {
                count--;
            }
        }
        else if (c == 13) //13 for enter
        {
            data->buffer[count] = '\0';
            return;
        }
        else if (c >= 32) //32 for character
        {
            if (count < MAX_CHARS)
            {
                data->buffer[count] = c;
                count++;
            }
            else
            {
                data->buffer[count] = '\0';
                return;
            }
        }
    }
}
