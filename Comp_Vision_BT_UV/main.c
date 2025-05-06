#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "clock.h"
#include "nvic.h"
#include "gpio.h"
#include "uart0.h"
#include "uart2.h"

// H-Bridge Motor/PWM Pins
// PortE masks
#define RIGHT_WHEEL_FORWARD 32 // PE5
// PortF masks
#define LEFT_WHEEL_BACKWARD 2 // PF1
#define LEFT_WHEEL_FORWARD 4 // PF2
#define RIGHT_WHEEL_BACKWARD 8 // PF3
// H-Bridge Sleep pin
#define SLP PORTE,3 // GPO

// IR Sensor Pin
#define IR PORTD,0 // WT2CCP0

// PIR Sensor
#define PIR PORTB,4 // T1CCPO

// Buzzer
#define BUZZ PORTD,2 // GPO

// Right Photo transistor
#define OPBL PORTC,6 // WT1CCPO

// Left Photo transistor
#define OPBR PORTC,7 // WT1CCP1

// Ultrasonic Sensor
#define ECHO PORTC,5 // WT0CCP1
#define TRIG PORTC,4 // WT0CCP0

// NEC IR Code
#define MAX_EDGES 34

// Global Variables
// IR Parse
float edgeTimings[MAX_EDGES];
uint32_t bitStream[32];
uint32_t edgeCount = 0;
uint32_t data_bin = 0b00000000;
char flag = 'N';
char button_pressed = 'N';
char balance = 'B';
char flag2 = 'N';
char bt_flag = 'N';
bool bt_received = true;

// PIR
int8_t pir_flag = 0;

// Optical Interrupters
uint32_t RPM_L = 0;
uint32_t RPM_R = 0;

// Ultrasonic sensors
uint32_t ultrasonic_distance = 0;

// Odometry
uint32_t angle = 0;
uint32_t distance = 0;
uint32_t count_limit = 0;
uint8_t autonomous = 0;

// Parse UART string
#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

void getsUart0(USER_DATA *data)
{
    int count = 0;
    while(true)
    {
        char c = getcUart0();
        if(((c == 8) || (c == 127)) && (count > 0))
        {
            count--;
        }
        else if(c == 13)
        {
            data->buffer[count] = '\0';
            return;
        }
        else
        {
            if(count == MAX_CHARS)
            {
                data->buffer[MAX_CHARS] = '\0';
                return;
            }
            data->buffer[count] = c;
            count++;
        }
    }
}

void parseFields(USER_DATA *data)
{
    char alpha = 'a';
    char num = 'n';
    char delim = 'd';
    char temp[MAX_CHARS+1];
    data->fieldCount = 0;
    temp[0] = 'd';

    //while true, increment count, store in field type at count
    int it = 0;
    while (data->buffer[it] != '\0')
    {
        if(((data->buffer[it] >= 65) && (data->buffer[it] <= 90)) || ((data->buffer[it] >= 97) && (data->buffer[it] <= 122)))
        {
            temp[it+1] = alpha;
        }
        else if(((data->buffer[it] >= 48) && (data->buffer[it] <= 57)) || (data->buffer[it] == 45) || (data->buffer[it] == 46))
        {
            temp[it+1] = num;
        }
        else
        {
                temp[it+1] = delim;
        }
        it++;
    }
    temp[it+1] = '\0';

    int i = 0;
    while (temp[i] != '\0')
    {
        if(temp[i] == delim)
        {
            if(temp[i+1] != delim)
            {
                data->fieldType[data->fieldCount] = temp[i+1];
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
            }
        }
        i++;
    }

    int count = 0;
    while(temp[count] != '\0')
    {
        temp[count] =  temp[count+1];
        count++;
    }

    int x = 0;
    while (data->buffer[x] != '\0')
    {
        if(temp[x] == delim)
        {
            data->buffer[x] = '\0';
        }
        x++;
    }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
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
    if((fieldNumber < data->fieldCount) && (data->fieldType[fieldNumber] == 'n'))
    {
        return atoi(&(data->buffer[data->fieldPosition[fieldNumber]]));
    }
    else
    {
        return 0;
    }
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    int i = 0;
    bool valid = 0;
    //i = data->fieldPosition[0];

    while (data->buffer[i] != '\0')
    {
        if ((data->buffer[i] == strCommand[i]) && ((data->fieldCount-1) >= minArguments))
        {
            valid = 1;
            i++;
        }
        else
        {
            valid = 0;
            return valid;
        }
    }
    return valid;
}

int strcmp(const char first[], const char second[])
{
    int i = 0;
    int valid = 0;
    while ((first[i] != '\0') && (second[i] != '\0'))
    {
        if (first[i] == second[i])
        {
            valid = 0;
            i++;
        }
        else if (first[i] < second[i])
        {
            valid = -1;
            return valid;
        }
        else if (first[i] > second[i])
        {
            valid = 1;
            return valid;
        }
     }
     return valid;
}

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable Clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2;

    // Enable Ports
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

    // Configure Output Pins
    selectPinPushPullOutput(SLP);
    selectPinPushPullOutput(BUZZ);
    selectPinPushPullOutput(TRIG);
    selectPinPushPullOutput(PORTE, 5);
    selectPinPushPullOutput(PORTF, 1);
    selectPinPushPullOutput(PORTF, 2);
    selectPinPushPullOutput(PORTF, 3);
    setPinValue(SLP, 1);
    setPinValue(BUZZ, 0);
    setPinValue(ECHO, 0);

    // Configure Input Pins
    selectPinDigitalInput(IR);
    selectPinDigitalInput(PIR);
    selectPinDigitalInput(ECHO);
    selectPinDigitalInput(OPBL);
    selectPinDigitalInput(OPBR);
    // Enable Interrupts for PIR
    enablePinInterrupt(PIR);
    enableNvicInterrupt(INT_GPIOB);
    clearPinInterrupt(PIR);
    //selectPinInterruptHighLevel(PIR);
    selectPinInterruptRisingEdge(PIR);
    // Enable Auxiliary Timer Functions of Pins
    setPinAuxFunction(OPBL, 7);
    setPinAuxFunction(OPBR, 7);
    setPinAuxFunction(IR, 7);
    setPinAuxFunction(ECHO, 7);
}

void enableTimer()
{
    // Configure Timer 1A as the time base for Optical Interrupters
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 20E3;  // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = 0;                 // turn-on interrupts
//    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
//    NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

    // Configure Timer 2A as the time base for Ultrasonic sensors
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 40E5;                           // set load value to 40e6 for 1 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    enableNvicInterrupt(INT_TIMER2A);                // turn-on interrupt

    // Configure Wide Timer 0A as the time base for ECHO
    WTIMER0_CTL_R &= ~TIMER_CTL_TBEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TBMR_R = TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR;
                                                     // configure for edge time mode, count up
    WTIMER0_CTL_R = TIMER_CTL_TBEVENT_BOTH;           // measure time from positive edge to positive edge
    WTIMER0_IMR_R = TIMER_IMR_CBEIM;                 // turn-on interrupts
    WTIMER0_TBV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TBEN;                 // turn-on counter
    enableNvicInterrupt(INT_WTIMER0B);                // turn-on interrupt

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;           // count positive edges
    WTIMER1_IMR_R |= 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    //NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);

    // Configure Wide Timer 1 as counter of external events on CCP1 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TBEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TBMR_R = TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R |= TIMER_CTL_TBEVENT_NEG;           // count positive edges
    WTIMER1_IMR_R |= 0;                               // turn-off interrupts
    WTIMER1_TBV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TBEN;                 // turn-on counter
    //NVIC_EN3_R = 1 << (INT_WTIMER1B-16-96);

    // Configure IR Timer
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER2_CTL_R |= TIMER_CTL_TAEVENT_NEG;          // measure time from negative edge to negative edge
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

    NVIC_EN3_R = 1 << (INT_WTIMER2A-16-96);          // turn-on interrupt
}

void initMotor()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTF_DEN_R |= LEFT_WHEEL_FORWARD | RIGHT_WHEEL_BACKWARD | LEFT_WHEEL_BACKWARD;
    GPIO_PORTF_AFSEL_R |= LEFT_WHEEL_FORWARD | RIGHT_WHEEL_BACKWARD | LEFT_WHEEL_BACKWARD;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;
    // Configure three LEDs
    GPIO_PORTE_DEN_R |= RIGHT_WHEEL_FORWARD;
    GPIO_PORTE_AFSEL_R |= RIGHT_WHEEL_FORWARD;
    GPIO_PORTE_PCTL_R &= ~(GPIO_PCTL_PE5_M);
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE5_M0PWM5;

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM0_2_CTL_R = 0;
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;

    PWM1_2_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)
    PWM0_2_LOAD_R = 1024;

    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off
    PWM0_2_CMPB_R = 0;

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
    PWM0_ENABLE_R = PWM_ENABLE_PWM5EN;
                                                     // enable outputs
}

void right_f(float pwm)
{
    PWM1_2_CMPB_R = pwm;
}
void left_b(float pwm) //
{
    PWM1_3_CMPA_R = pwm;
}
void left_f(float pwm)
{
    PWM1_3_CMPB_R = pwm;
}
void right_b(float pwm)
{
    PWM0_2_CMPB_R = pwm;
}

void pirISR()
{
    pir_flag = 1;
    clearPinInterrupt(PIR);
}

void getIR()
{
    int i = 0;
    int time = 0;
    // Convert from time to bits
    if (edgeCount == MAX_EDGES)
    {
        data_bin = 0b00000000;
        for(i = 1; i < MAX_EDGES-1; i++)
        {
            if( (((edgeTimings[i+1] - edgeTimings[i]) * 2) >= 0.5) && (((edgeTimings[i+1] - edgeTimings[i]) * 2) <= 1.7) )
            {
                bitStream[time] = 0;
            }
            else if( (((edgeTimings[i+1] - edgeTimings[i]) * 2) >= 1.8) && (((edgeTimings[i+1] - edgeTimings[i]) * 2) <= 3.0) )
            {
                bitStream[time] = 1;
            }
            time++;
        }
        time = 0;
        flag = 'T';
        int x;
        int z = 7;
        // Get data from stream
        for (x = 16; x <= 23; x++)
        {
            data_bin |= bitStream[x] << z;
            z--;
        }
    }
    if(edgeCount == MAX_EDGES)
    {
        edgeCount = 0;
    }
}

void parseIR()
{
    //if (data_bin == 152 || data_bin == 148)
    if (data_bin == 152)
    {
        distance = 35;
        flag = 'F'; // forward
    }
    else if (data_bin == 204)
    {
        distance = 35;
        flag = 'B'; // backwards
    }
    else if (data_bin == 180)
    {
        angle = 90;
        flag = 'C'; // CW
    }
    else if (data_bin == 120)
    {
        angle = 90;
        flag = 'W'; // CCW
    }
    else if (data_bin == 30)
    {
        angle = 360;
        flag = 'C'; // 360 CW
    }
    else if (data_bin == 44)
    {
        angle = 180;
        flag = 'W'; // 180 CCW
    }
    else if (data_bin == 170)
    {
        angle = 180;
        flag = 'C'; // 180 CW
    }
    else if (data_bin == 134)
    {
        flag = 'S'; // Stop
    }
    else if (data_bin == 232)
    {
        flag = 'A'; // Autonomous mode on/off
    }
    else
    {
        flag = 'N'; // NULL
    }
}

void irWTISR()
{
    // if (button_pressed != Y)
    // get all falling edges
    if (edgeCount < MAX_EDGES)
    {
        edgeTimings[edgeCount] = (WTIMER2_TAR_R * (0.000000025 * 1000)) / 2;
        edgeCount++;
    }
    // checksum of data bits
    if (edgeCount == 34)
    {
        button_pressed = 'Y';
    }
    WTIMER2_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void ultrasonicISR()
{
    setPinValue(TRIG, 1);
    waitMicrosecond(10);
    setPinValue(TRIG, 0);
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

void wideTimer0Isr()
{
    ultrasonic_distance = WTIMER0_TBV_R;                        // read counter input
    ultrasonic_distance = (ultrasonic_distance * (345 * 0.000000025 * 1000)) / 2 ; // convert to distance
    WTIMER0_TBV_R = 0;                           // zero counter for next edge
    // turn 90 (don't use command because not fast enough)
    if (autonomous == 1 && ultrasonic_distance < 300)
    {
        right_f(0);
        right_b(0);
        left_f(0);
        left_b(0);
        waitMicrosecond(200000);
        right_f(0);
        right_b(856);
        left_f(870);
        left_b(0);
        waitMicrosecond(1000000);
        right_b(0);
        left_f(0);
        waitMicrosecond(200000);
        left_f(1015);
        right_f(1015);
    }
    WTIMER0_ICR_R = TIMER_ICR_CBECINT;           // clear interrupt flag
}

// Not used
void opblISR()
{
    RPM_L += (WTIMER1_TAR_R);
    WTIMER1_TAV_R = 0;
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void modISR()
{
    RPM_L += (WTIMER1_TAV_R);
    RPM_R += (WTIMER1_TBV_R);

    WTIMER1_TAV_R = 0;                           // reset counter for next period
    WTIMER1_TBV_R = 0;                           // reset counter for next period
    if(RPM_L >= count_limit+1)
    {
        RPM_L = 0;
        RPM_R = 0;
        left_f(0);
        right_f(0);
        left_b(0);
        right_b(0);
        TIMER1_IMR_R = 0;                 // turn-on interrupts
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-on timer
    }

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void bt_input(uint8_t bt_value)
{
    if(bt_value < 6)
    {
        bt_received = true;
        if(bt_value == 0) // Stop
        {
            bt_flag = 'S';
        }
        else if(bt_value == 1) // GO -> Auto mode
        {
            bt_flag = 'A';
        }
        else if(bt_value == 2) // 360
        {
            angle = 360;
            bt_flag = 'C';
        }
        else if(bt_value == 3) // CCW
        {
            angle = 90;
            bt_flag = 'W';
        }
        else if(bt_value == 4) // CW
        {
            angle = 90;
            bt_flag = 'C';
        }
        else if(bt_value == 5) // Forward
        {
            distance = 35;
            bt_flag = 'F';
        }
    }
}

int main(void)
{
    initHw();
    initUart0();
    initUart2();
    setUart2BaudRate(9600, 40e6);
    initMotor();
    enableTimer();
    USER_DATA data;
    bool valid = false;
    bool command = false;
    char str[40];
    uint8_t bt_value = 0;

    while(true)
    {
        if(kbhitUart0())
        {
            getsUart0(&data);
            parseFields(&data);

            if(isCommand(&data, "reset", 0))
            {
                valid = true;
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }
            else if(isCommand(&data, "forward", 1))
            {
                distance = getFieldInteger(&data, 1);
                valid = true;
                flag2 = 'F';
                command = true;
            }
            else if(isCommand(&data, "reverse", 0))
            {
                distance = getFieldInteger(&data, 1);
                valid = true;
                flag2 = 'B';
                command = true;
            }
            else if(isCommand(&data, "CW", 1))
            {
                angle = getFieldInteger(&data, 1);
                valid = true;
                flag2 = 'C';
                command = true;
            }
            else if(isCommand(&data, "CCW", 1))
            {
                angle = getFieldInteger(&data, 1);
                valid = true;
                flag2 = 'W';
                command = true;
            }
            else if(isCommand(&data, "auto", 0))
            {
                valid = true;
                flag2 = 'A';
                command = true;
            }
            if (!valid)
            {
                putsUart0("Invalid command\n");
            }
            valid = false;
        }

        if (button_pressed  == 'Y' || command == true || bt_received == true)
        {
            getIR();
            parseIR();
            if ((flag == 'F' || flag2 == 'F' || bt_flag == 'F') && (autonomous == 0))
            {
                count_limit = (float)distance / 1.4;
                TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
                NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                left_f(1012);
                right_f(1012);
            }
            else if ((flag == 'B' || flag2 == 'B'  || bt_flag == 'B') && (autonomous == 0))
            {
                count_limit = (float)distance / 1.4;
                TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
                NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                right_b(1012);
                left_b(1012);
            }
            else if ((flag == 'C' || flag2 == 'C' || bt_flag == 'C') && (autonomous == 0))
            {
                count_limit = (float)angle / 12;
                TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
                NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                right_b(880);
                left_f(890);
            }
            else if ((flag == 'W' || flag2 == 'W' || bt_flag == 'W') && (autonomous == 0))
            {
                count_limit = (float)angle / 12;
                TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
                NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                right_f(880);
                left_b(890);
            }
            else if (flag == 'A' || flag2 == 'A' || bt_flag == 'A')
            {
                autonomous = autonomous ^ 1;
                if (autonomous)
                {
                    right_f(1023);
                    left_f(1015);
                }
                else
                {
                    right_f(0);
                    left_f(0);
                }
            }
            else if (flag == 'S' || bt_flag == 'S')
            {
                autonomous = 0;
                count_limit = 0;
                right_f(0);
                left_f(0);
                right_b(0);
                left_b(0);
            }
            data_bin = 0b00000000;
            flag = 'N';
            flag2 = 'N';
            bt_flag = 'N';
            button_pressed = 'N';
            command = false;
            bt_received = false;
            waitMicrosecond(500000);
        }

        if (pir_flag == 1)
        {
            pir_flag = 0;
            putsUart0("Intruder Detected! \n");
            while((getPinValue(PIR) == 1) && (autonomous == 1))
            {
                setPinValue(BUZZ, 1);
                waitMicrosecond(1500);
                setPinValue(BUZZ, 0);
                waitMicrosecond(1200);
            }
        }

        if(kbhitUart2())
        {
            bt_value = getcUart2();
            bt_input(bt_value - 48);
        }

        snprintf(str, sizeof(str), "ultrasonic distance: %d mm \n", ultrasonic_distance);
        putsUart0(str);
        snprintf(str, sizeof(str), "Left wheel speed: %d \n", RPM_L);
        putsUart0(str);
        snprintf(str, sizeof(str), "Right wheel speed: %d \n", RPM_R);
        putsUart0(str);
        waitMicrosecond(1000000);
    }
}
// distance and angle can be specified so use opb
// wheel radius is 4.5cm so circumference is 28.27
// 28.27 / 20 = 1.4 cm for each hole counted
// i.e 50 / 1.4 = 35.7 = 36, so 50 cm is 36 counts
// formula = distance / 1.4; while TnV register value is less than formula drive wheels else stop
// diameter between both wheels is 13.5 cm so radius is 6.75 cm and circumference is 42.41 cm
// 360 / (42.41 / 1.4) = 6 degrees per count?
// formula = angle / 6
