// #############################################################################
//  FILE:   LAB6starter_main.c
//
//  TITLE:  Lab6 Starter
// #############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI 3.1415926535897932384626433832795
#define TWOPI 6.283185307179586476925286766559
#define HALFPI 1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

#include "F28379dCAN.h"

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);

// Lab7 Vars

float k1 = -60;
float k2 = -4.5;
float k3 = -1.1;
float k4 = -0.1;

float ubal = 0.0;


float LeftWheelPrev = 0;
float RightWheelPrev = 0;

float velLeftPrev = 0;
float velRightPrev = 0;

float velLeft = 0;
float velRight = 0;

float gyro_rate_prev = 0;
float gyro_rate = 0;
float gyro_value_prev = 0;

float WhlDiff = 0;
float WhlDiff_1 = 0;
float vel_WhlDiff = 0;
float vel_WhlDiff_1 = 0;
float turnref = 0;
float turnrefPrev = 0;
float errorDiff = 0;
float errorDiffPrev = 0;
float intDiff = 0;
float intDiffPrev = 0;
float turnrate = 0;
float turnratePrev = 0;
float averagewheelvelocity=0.0;
float segbot_ref_speed=0.0;
float espeed=0.0;
float Kpspeed=0.35;
float Kispeed=1.5;
float IK_espeed=0.0;
float ForwardBackwardCommand=0.0;
float IK_espeed_1=0.0;
float espeed_1=0.0;


// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.76;
int16 IMU_data[9];
uint16_t temp = 0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0;
float RightWheel = 0;
float LeftWheelArray[4] = {0, 0, 0, 0};
float RightWheelArray[4] = {0, 0, 0, 0};
// Kalman Filter vars
float T = 0.001; // sample rate, 1ms
float Q = 0.01;  // made global to enable changing in runtime
float R = 25000; // 50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t AverageIndex = -1;
float pred_P = 0;
float kalman_K = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

void setupSpib(void);
// st52_qiyuanx3: encoder feedback function predefinition
void init_eQEPs(void);

// st52_qiyuanx3: encoder reading definitions
float readEncLeft(void);
float readEncRight(void);

// st52_qiyuanx3: Motor function definitions
void setEPWM2A(float);
void setEPWM2B(float);

// st52_qiyuanx3: external varaibles for the status of the distance sensor and the distance it produces

extern uint32_t dis_1;
extern uint32_t dis_3;

extern uint32_t measure_status_1;
extern uint32_t measure_status_3;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// st52_qiyuanx3: Raw IMU data
int16_t accelx_raw = 0;
int16_t accely_raw = 0;
int16_t accelz_raw = 0;
int16_t gyrox_raw = 0;
int16_t gyroy_raw = 0;
int16_t gyroz_raw = 0;

// st52_qiyuanx3: Scaled IMU data
float accelx = 0;
float accely = 0;
float accelz = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

// st52_qiyuanx3: number of times spib called
int32_t SpibNumCalls = 0;

// st52_qiyuanx3: adc results for joystick
uint16_t adca2result;
uint16_t adca3result;
float adcina2_scaled;
float adcina3_scaled;

float controleffort = 0.0;

// st52_qiyuanx3: All the global variables are defined below here

// st52_qiyuanx3:left and right wheel encoder values
float rightwheel = 0.0;
float leftwheel = 0.0;

// st52_qiyuanx3: control effort for left and right wheel
float uLeft = 5.0;
float uRight = 5.0;

// st52_qiyuanx3: position readings at current and previous timestep
float posleft_k = 0.0;
float posleft_k1 = 0.0;
float posright_k = 0.0;
float posright_k1 = 0.0;

// st52_qiyuanx3: velocity of left and right wheels
float vLeftk = 0.0;
float vRightk = 0.0;

// st52_qiyuanx3:reference velocity
float Vref = 0.25;

// st52_qiyuanx3: velocity error
float erLeft = 0.0;
float erRight = 0.0;

// st52_qiyuanx3: velocity error in previous timestep
float erk1Left = 0.0;
float erk1Right = 0.0;

// st52_qiyuanx3: integral at current and previous timestep
float IkLeft = 0.0;
float Ik1Left = 0.0;
float IkRight = 0.0;
float Ik1Right = 0.0;

// st52_qiyuanx3: P and I gains for velocity control
float Kd = 0.08;
float Kp = 3.0;
float Ki = 20.0;

// st52_qiyuanx3: turn and turn error, turn P gain
float eturn = 0.0;
float turn = 0.0;
float Kp_turn = 3.0;

// st52_qiyuanx3: width of robot and diameter of wheel
float width = 0.173;
float wheel = 0.0593;

// st52_qiyuanx3: left and right wheel angle and angular velocity
float theta_left = 0.0;
float theta_right = 0.0;
float theta_dot_right = 0.0;
float theta_dot_left = 0.0;

// st52_qiyuanx3: left and right angle on timestep back
float theta_right_prev = 0.0;
float theta_left_prev = 0.0;

// st52_qiyuanx3: current and previous robot x,y velocity
float x_dot = 0.0;
float y_dot = 0.0;
float x_dot_prev = 0.0;
float y_dot_prev = 0.0;

// st52_qiyuanx3: average angle and angular velocity
float theta_avg = 0.0;
float theta_dot_avg = 0.0;

// st52_qiyuanx3: right wall follow and front obstacle detected turn gain
float Kp_right = 0.001;
float Kp_front = 0.0002;

// st52_qiyuanx3: right follow and front obstacle state distance reference
float ref_right = 200.0;
float ref_front = 1400.0;

// st52_qiyuanx3: thresholds to switch states
float threshold1 = 450.0;
float threshold2 = 650.0;

// st52_qiyuanx3: distance to right and front
float distright = 0.0;
float distfront = 0.0;

// st52_qiyuanx3: reference velocity of right wall follow and front obstacle status
float vel_right = 0.25;
float vel_front = 0.25;

// st52_qiyuanx3:turn saturation
float turn_sat = 2.0;

// st52_qiyuanx3: for python UI
float printPY3 = 0;
float printPY4 = 0;
float printPY5 = 0;
float printPY6 = 0;
float printPY7 = 0;
float printPY8 = 0;
float x = 0;
float y = 0;
float bearing = 0;

extern uint16_t NewPYData;
extern float fromPYvalues[PYNUM_TOFROM_FLOATS];
extern PYSendFloats_t DataToPython;
extern char PYsenddata[PYNUM_TOFROM_FLOATS * 4 + 2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

int rightwallfollow = 1; // st52_qiyuanx3: State 1 is that is follows the right wall and State 2 is that it looks ahead

uint32_t fir_order = 22;
float xk3_prev[21] = {0};
float xk2_prev[21] = {0};

float b[22] = {-4.3821884069440866e-04, 6.0807670026235064e-04, 3.1406253912868204e-03, 8.7768009786225315e-03,
               1.8745713870486094e-02,  3.3384353372803752e-02, 5.1837442737665093e-02, 7.2071154861486783e-02,
               9.1222809663499521e-02,  1.0620981157506326e-01, 1.1444142968951818e-01, 1.1444142968951818e-01,
               1.0620981157506326e-01,  9.1222809663499521e-02, 7.2071154861486783e-02, 5.1837442737665093e-02,
               3.3384353372803752e-02,  1.8745713870486094e-02, 8.7768009786225315e-03, 3.1406253912868204e-03,
               6.0807670026235064e-04,  -4.3821884069440866e-04};

// st52_qiyuanx3: Global variables for simple 5 tap averaging filter
float xk3 = 0;
float xk2 = 0;

// yk is the filtered value

float yk3 = 0;
float yk2 = 0;

float distanceLeft = 0.0;
float distanceRight = 0.0;

void main(void) {
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    // WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    // ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    // SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    // DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    // DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    // DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    // MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    // WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    // PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    // PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    // PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    // PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    // Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Initialize the CAN controller
    InitCANB();

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.CANB0_INT = &can_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    EDIS; // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA, 115200);
    init_eQEPs();

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3;    // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2;    // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event (“pulse” is the same as“trigger”)
    EPwm5Regs.TBCTR = 0x0;              // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0;      // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0;     // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000;            // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;  // enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; // unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    // qiyuanx3: EPWM2A and B settings
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 1250;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.CMPB.bit.CMPB = 1250;
    EDIS;

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);

    EALLOW;                            // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    EDIS;

    EALLOW;
    // write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // read calibration settings

    // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    // power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    // Select the channels to convert and end of conversion flag
    // Many statements commented out, To be used when using ADCA or ADCB.

    // ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;     // st52_qiyuanx3: SOC0 convert ADCINA2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99;    // sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;  // st52_qiyuanx3: trigger using EPWM5
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;     // st52_qiyuanx3: SOC0 convert ADCINA3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99;    // sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;  // st52_qiyuanx3: trigger using EPWM5
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; // st52_qiyuanx3: SOC1 is the last
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // make sure INT1 flag is cleared
    EDIS;

    setupSpib();

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT6;
    IER |= M_INT8; // SCIC SCID
    IER |= M_INT9; // SCIA  CANB
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1; // SPiB
    // Enable CANB in the PIE: Group 9 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;
    // Enable the CAN interrupt signal
    CanbRegs.CAN_GLB_INT_EN.bit.GLBINT0_EN = 1;
    // st52_qiyuanx3: Enable ADCA1 in PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    init_serialSCIC(&SerialC, 115200);
    init_serialSCID(&SerialD, 115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT; // Enable Global interrupt INTM
    ERTM; // Enable Global realtime interrupt DBGM

    SetCANMessagesForIRSensor();

    // IDLE loop. Just sit and loop forever (optional):
    while (1) {

        if (UARTPrint == 1) {
            // serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",numTimer2calls,numRXA);
            serial_printf(&SerialA, "a:%.3f,%.3f,%.3f  g:%.3f,%.3f,%.3f, TiltVal:%.3f, GyroVal:%.3f, LeftWheel:%.3f, RightWheel:%.3f\r\n", accelx, accely, accelz, gyrox, gyroy,
                          gyroz, tilt_value, gyro_value, LeftWheel, RightWheel);
            // serial_printf(&SerialA, "D1 %ld D2 %ld\r\n", dis_1, dis_3);
            // serial_printf(&SerialA, " St1 %ld St2 %ld\n\r", measure_status_1, measure_status_3);
            // serial_printf(&SerialA, "Joystick X %f Joystick Y %f\r\n", xk3, yk2);
            serial_printf(&SerialA, "Rightwheel v %f Leftwheel v%f\r\n", vRightk, vLeftk);

            UARTPrint = 0;
        }
    }
}

// // SWI_isr,  Using this interrupt as a Software started interrupt
// __interrupt void SWI_isr(void) {

//     // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
//     // making it lower priority than all other Hardware interrupts.
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
//     asm("       NOP"); // Wait one cycle
//     EINT;              // Clear INTM to enable interrupts

//     // Insert SWI ISR Code here.......

//     // st52_qiyuanx3: this code below checks for the validity of the distane sensor. It ensures that hte robot
//     doesn't keep spinning in a circle due to bad data

// //     if (measure_status_1 == 0) {
// //         distright = dis_1;
// //     } else {
// //         distright = 1400; // set to max reading if error
// //     }
// //     if (measure_status_3 == 0) {
// //         distfront = dis_3;
// //     } else {
// //         distfront = 1400; // set to max reading if error
// //     }

// //     // st52_qiyuanx3: Below is the wall-following algorithm which is a simple state machine that has 2 states to
// turn when the front distance it soo small and to follow the wall and not veer away from it.

// //     if (rightwallfollow) {
// //         turn = Kp_right * (ref_right - distright); // st52_qiyuanx3: turn if needed to maintain distance to the
// right
// //         Vref = vel_right; // st52_qiyuanx3: right follow velocity reference
// //         if (distfront < threshold1) { // st52_qiyuanx3: stop wall follow when obstacle in front
// //             rightwallfollow = 0;
// //         }
// //     }
// //     else {
// //         turn = Kp_front * (ref_front - distfront);
// //         Vref = vel_front;
// //         if (distfront > threshold2) { // st52_qiyuanx3: switch back to right wall follow if no obstacle in front
// //             rightwallfollow = 1;
// //         }
// //     }
// //     // st52_qiyuanx3: turn saturation
// //     if (turn > turn_sat) {
// //         turn = turn_sat;
// //     }
// //     if (turn < -turn_sat) {
// //         turn = -turn_sat;
// //     }

// //     // st52_qiyuanx3: left and right wheel angle from encoder
// //     leftwheel = -1 * readEncLeft();
// //     rightwheel = readEncRight();

// //     // st52_qiyuanx3: tranform from angle to distance
// //     distanceLeft = leftwheel / 16.34;
// //     distanceRight = rightwheel / 16.34;
// //     posright_k = distanceRight;
// //     posleft_k = distanceLeft;

// //     // st52_qiyuanx3: velocity calculation
// //     vLeftk = (posleft_k - posleft_k1) / 0.004;
// //     vRightk = (posright_k - posright_k1) / 0.004;
// //     posleft_k1 = posleft_k;
// //     posright_k1 = posright_k;

// //     // st52_qiyuanx3: We are propagating the wheel states with trapezoid rule integration and getting the control
// u with the PI controller.
// //     // st52_qiyuanx3:turn error
// //     eturn = turn + (vLeftk - vRightk);
// //     // st52_qiyuanx3: left and right wheel error
// //     erLeft = Vref - vLeftk - Kp_turn * eturn;
// //     erRight = Vref - vRightk + Kp_turn * eturn;

// //     // st52_qiyuanx3: trapezoid rule integration
// //     IkRight = Ik1Right + 0.004 * (erRight + erk1Right) / 2.0;
// //     IkLeft = Ik1Left + 0.004 * (erLeft + erk1Left) / 2.0;

// //     // st52_qiyuanx3: PI controller
// //     uRight = Kp * erRight + Ki * Ik1Right;
// //     uLeft = Kp * erLeft + Ki * Ik1Left;

// //     // st52_qiyuanx3: prevent integral windup : if control effort > 10, integral gets multiplied by 0.95
// //     if (fabs(uRight) > 10.0) {
// //         IkRight = 0.95 * Ik1Right;
// //     }

// //     if (fabs(uLeft) > 10.0) {
// //         IkLeft = 0.95 * Ik1Left;
// //     }

// //     // st52_qiyuanx3: set up integral and error for 1 timestep back
// //     Ik1Right = IkRight;
// //     Ik1Left = IkLeft;
// //     erk1Right = erRight;
// //     erk1Left = erLeft;

// //     // st52_qiyuanx3:Robot pose calculation

// //     // st52_qiyuanx3: wheels angle 1 timestep back
// //     theta_left_prev = theta_left;
// //     theta_right_prev = theta_right;

// //     // st52_qiyuanx3: wheel angles
// //     theta_left = leftwheel;
// //     theta_right = rightwheel;

// //     // st52_qiyuanx3: wheel angular velocity
// //     theta_dot_right = (theta_right - theta_right_prev) / 0.004;
// //     theta_dot_left = (theta_left - theta_left_prev) / 0.004;

// //     // st52_qiyuanx3: average angle and angular velocity
// //     theta_dot_avg = 0.5 * (theta_dot_right + theta_dot_left);
// //     theta_avg = 0.5 * (theta_left + theta_right);

// //     // st52_qiyuanx3: robot pose angle
// //     bearing = (wheel / width) * (theta_right - theta_left);

// //     // st52_qiyuanx3: previous x,y velocity
// //     x_dot_prev = x_dot;
// //     y_dot_prev = y_dot;

// //     // st52_qiyuanx3: x,y velocity
// //     x_dot = wheel * theta_dot_avg * cosf(bearing);
// //     y_dot = wheel * theta_dot_avg * sinf(bearing);

// //     // st52_qiyuanx3: x,y position
// //     x = x + (x_dot + x_dot_prev) / 2.0 * 0.004;
// //     y = y + (y_dot + y_dot_prev) / 2.0 * 0.004;

// //    // st52_qiyuanx3: send control effort to motors
// //     setEPWM2A(uRight);
// //     setEPWM2B(uLeft);

// //     // st52_qiyuanx3: communication with python
// //     if (NewPYData == 1) {
// //         NewPYData = 0;
// //         Vref = fromPYvalues[0];
// //         turn = fromPYvalues[1];
// //         printPY3 = fromPYvalues[2];
// //         printPY4 = fromPYvalues[3];
// //         printPY5 = fromPYvalues[4];
// //         printPY6 = fromPYvalues[5];
// //         printPY7 = fromPYvalues[6];
// //         printPY8 = fromPYvalues[7];
// //     }
// //     if ((numSWIcalls % 13) == 0) { // 4*13 = 52ms
// //         DataToPython.floatData[0] = x;
// //         DataToPython.floatData[1] = y;
// //         DataToPython.floatData[2] = bearing;
// //         DataToPython.floatData[3] = 2.0 * ((float)numTimer0calls) * .001; // Next 5 are up to you.
// //         DataToPython.floatData[4] = 3.0 * ((float)numTimer0calls) * .001; // Here I am just sending
// //         DataToPython.floatData[5] = (float)numTimer0calls;                // something to check
// //         DataToPython.floatData[6] = (float)numTimer0calls * 4.0;
// //         DataToPython.floatData[7] = (float)numTimer0calls * 5.0;
// //         PYsenddata[0] = '*'; // header for PYdata
// //         PYsenddata[1] = '$';
// //         for (int i = 0; i < PYNUM_TOFROM_FLOATS * 4; i++) {
// //             if (i % 2 == 0) {
// //                 PYsenddata[i + 2] = DataToPython.rawData[i / 2] & 0xFF;
// //             } else {
// //                 PYsenddata[i + 2] = (DataToPython.rawData[i / 2] >> 8) & 0xFF;
// //             }
// //         }
// //         serial_sendSCID(&SerialD, PYsenddata, 4 * PYNUM_TOFROM_FLOATS + 2);
// //     }
// //     numSWIcalls++;

// //     DINT;
// }

// SWI_isr, Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {
    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm(" NOP"); // Wait one cycle
    EINT;        // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    

    LeftWheel = -1.0 * readEncLeft();
    RightWheel = readEncRight();

    velLeft = 100.0 * (LeftWheel - LeftWheelPrev) + 0.6 * velLeftPrev;
    velRight = 100.0 * (RightWheel - RightWheelPrev) + 0.6 * velRightPrev;

    gyro_rate = 100.0 * (gyro_value - gyro_value_prev) + 0.6 * gyro_rate_prev;

    ubal = -k1 * tilt_value - k2 * gyro_value - k3 * (velLeft + velRight) / 2.0 - k4 * gyro_rate;

    WhlDiff = (RightWheel - LeftWheel);

    turnref = turnrefPrev + 0.004 * (turnrate + turnratePrev) / 2.0;

    vel_WhlDiff = 166.667 * (WhlDiff - WhlDiff_1) + 0.333 * vel_WhlDiff_1;
    errorDiff = turnref - WhlDiff;
    intDiff = intDiffPrev + 0.004 * (errorDiff + errorDiffPrev) / 2.0;

    turn = Kp * errorDiff + Ki * intDiff - Kd * vel_WhlDiff;

    if (fabs(turn) > 4.0) {
        intDiff = intDiffPrev * 0.95;
        turn = 4.0 * (fabs(turn) / turn);
    }
    
    espeed=(segbot_ref_speed-(velLeft+velRight)/2.0);
    IK_espeed=IK_espeed_1+0.002*(espeed+espeed_1);
    ForwardBackwardCommand = Kpspeed*espeed + Kispeed*IK_espeed;

    if (fabs(ForwardBackwardCommand) > 4.0) {
        IK_espeed = IK_espeed_1 * 0.95;
        ForwardBackwardCommand = 4.0 * (fabs(ForwardBackwardCommand) / ForwardBackwardCommand);
    }

    
    uLeft = 0.5 * ubal - turn-ForwardBackwardCommand;
    uRight = 0.5 * ubal + turn-ForwardBackwardCommand;

    setEPWM2A(uRight);
    setEPWM2B(uLeft);
    espeed_1=espeed;
    IK_espeed_1=IK_espeed;
    errorDiffPrev = errorDiff;
    intDiffPrev = intDiff;
    vel_WhlDiff_1 = vel_WhlDiff;
    WhlDiff_1 = WhlDiff;

    LeftWheelPrev = LeftWheel;
    RightWheelPrev = RightWheel;

    gyro_value_prev = gyro_value;
    gyro_rate_prev = gyro_rate;

    velLeftPrev = velLeft;
    velRightPrev = velRight;

    turnratePrev = turnrate;
    turnrefPrev = turnref;

    numSWIcalls++;

    DINT;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void) {
    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls % 250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) { // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    // //GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    // //    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    // //    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    // //    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
    // //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    // //    SpibRegs.SPITXBUF = 0xDA;
    // //    SpibRegs.SPITXBUF = 500; // PWM value
    // //    SpibRegs.SPITXBUF = 2200; // PWM Value
    // GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    // SpibRegs.SPITXBUF = 0xBA00;
    // SpibRegs.SPITXBUF = 0x0000;
    // SpibRegs.SPITXBUF = 0x0000;
    // SpibRegs.SPITXBUF = 0x0000;
    // SpibRegs.SPITXBUF = 0x0000;
    // SpibRegs.SPITXBUF = 0x0000;
    // SpibRegs.SPITXBUF = 0x0000;
    // SpibRegs.SPITXBUF = 0x0000;

    // //if ((numTimer0calls%50) == 0) {
    // //	// Blink LaunchPad Red LED
    // //	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    // //}

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void) { numTimer1calls++; }

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void) {
    // Blink LaunchPad Blue LED
    // GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    numTimer2calls++;

    if ((numTimer2calls % 10) == 0) {
        //		UARTPrint = 1;
    }
}

void setupSpib(void) // Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    // Step 1.
    //  cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in between each
    //  transfer to 0. Also donâ€™t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a
    //  part of the SPIB setup.
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);               // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1;                   // Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);               // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;                   // Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); // Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); // Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); // Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1;    // This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;  // The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;    // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1;         // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1;         // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0;    // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLKâ€™s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1;     // Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;   // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0;     // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;    // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL

    // SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPIFFCT.bit.TXDLY = 0;

    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1;      // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1;    // Enables SPI interrupt. !! I dont think this is needed. Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 16; // Interrupt Level to 16 words or more received into FIFO causes interrupt. This
                                      // is just the initial setting for the register. Will be changed below

    //-----------------------------------------------------------------------------------------------------------------

    // Step 2.
    //  perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    //  0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    //  some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16-bit
    // transfers, so two registers at a time after the first 16-bit transfer. To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x1300 | 0x0000);
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = (0x0000 | 0x0013);
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = (0x0200 | 0x0000);
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = (0x0800 | 0x0006);
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 7)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 3.
    //  perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    //  0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    //  some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = (0x2300 | 0x0000);
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = (0x4000 | 0x008C);
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = (0x0200 | 0x0088);
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A);

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 4)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 4.
    //  perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);
    // wait for one byte to be received
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0016); // 0x7700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x00C4); // 0x7800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E8); // 0x7A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00F0); // 0x7B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001C); // 0x7D00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0062); // 0x7E00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
__interrupt void SPIB_isr(void) {
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    spivalue1 = SpibRegs.SPIRXBUF;
    accelx_raw = SpibRegs.SPIRXBUF;
    accely_raw = SpibRegs.SPIRXBUF;
    accelz_raw = SpibRegs.SPIRXBUF;
    spivalue2 = SpibRegs.SPIRXBUF;
    gyrox_raw = SpibRegs.SPIRXBUF;
    gyroy_raw = SpibRegs.SPIRXBUF;
    gyroz_raw = SpibRegs.SPIRXBUF;

    accelx = accelx_raw * (4.0 / 32767.0);
    accely = accely_raw * (4.0 / 32767.0);
    accelz = accelz_raw * (4.0 / 32767.0);

    gyrox = gyrox_raw * (250.0 / 32767.0);
    gyroy = gyroy_raw * (250.0 / 32767.0);
    gyroz = gyroz_raw * (250.0 / 32767.0);

    // This post of the SWI interrupt is only needed in Lab 6
    // Comment out these next eight lines for Lab 7, the given code in Lab 7 performs the SWI Post and blinks the LEDs
    // if ((SpibNumCalls % 4) == 0) {
    //     PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Software Trigger, Post, SWI interrupt.
    // }
    // if ((numTimer0calls % 50) == 0) {
    //     // Blink LaunchPad LEDs
    //     GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    //     GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    // }

    if (calibration_state == 0) {
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if (calibration_state == 1) {
        accelx_offset += accelx;
        accely_offset += accely;
        accelz_offset += accelz;
        gyrox_offset += gyrox;
        gyroy_offset += gyroy;
        gyroz_offset += gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset /= 2000.0;
            accely_offset /= 2000.0;
            accelz_offset /= 2000.0;
            gyrox_offset /= 2000.0;
            gyroy_offset /= 2000.0;
            gyroz_offset /= 2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if (calibration_state == 2) {
        accelx -= (accelx_offset);
        accely -= (accely_offset);
        accelz -= (accelz_offset - accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------
        ------------------*/
        float tiltrate = (gyrox * PI) / 180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T * tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P / S;
        kalman_tilt = pred_tilt + kalman_K * y;
        kalman_P = (1 - kalman_K) * pred_P;
        AverageIndex++;
        // Kalman Filter used
        tilt_array[AverageIndex] = kalman_tilt;
        gyro_array[AverageIndex] = tiltrate;
        LeftWheelArray[AverageIndex] = readEncLeft();
        RightWheelArray[AverageIndex] = readEncRight();
        if (AverageIndex >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3]) / 4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3]) / 4.0;
            LeftWheel = (LeftWheelArray[0] + LeftWheelArray[1] + LeftWheelArray[2] + LeftWheelArray[3]) / 4.0;
            RightWheel = (RightWheelArray[0] + RightWheelArray[1] + RightWheelArray[2] + RightWheelArray[3]) / 4.0;
            AverageIndex = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }
    if ((SpibNumCalls % 200) == 0) {
        if (doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
    }

    // In Lab7, Cut and Paste the Lab 7 code right here

    if ((SpibNumCalls % 200) == 0) {
        UARTPrint = 1;
    }
    SpibNumCalls++;

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

__interrupt void ADCA_ISR(void) {
    adca2result = AdcaResultRegs.ADCRESULT0;
    adca3result = AdcaResultRegs.ADCRESULT1;
    // Here covert ADCIND0, ADCIND1 to volts
    adcina2_scaled = adca2result * 3.0 / 4096.0;
    adcina3_scaled = adca3result * 3.0 / 4096.0;

    // st52_qiyuanx3: xk sets to current read value, yk initialize to 0
    xk3 = adcina3_scaled;
    yk3 = 0;

    // st52_qiyuanx3: calculates the output voltage yk from coefficients b, current value xk, and previous 21 values
    for (int i = 0; i < fir_order; i++) {
        if (i == 0) {
            yk3 += b[i] * xk3;
        } else {
            yk3 += b[i] * xk3_prev[i - 1];
        }
    }

    // st52_qiyuanx3: update the past states, start from the end, the value updates to the value of the index 1 before
    for (int i = fir_order - 2; i > 0; i--) {
        xk3_prev[i] = xk3_prev[i - 1];
    }
    xk3_prev[0] = xk3; // st52_qiyuanx3: the first value will update to xk

    // st52_qiyuanx3: xk sets to current read value, yk initialize to 0
    xk2 = adcina2_scaled;
    yk2 = 0;

    // st52_qiyuanx3: calculates the output voltage yk from coefficients b, current value xk, and previous 21 values
    for (int i = 0; i < fir_order; i++) {
        if (i == 0) {
            yk2 += b[i] * xk2;
        } else {
            yk2 += b[i] * xk2_prev[i - 1];
        }
    }

    // st52_qiyuanx3: update the past states, start from the end, the value updates to the value of the index 1 before
    for (int i = fir_order - 2; i > 0; i--) {
        xk2_prev[i] = xk2_prev[i - 1];
    }
    xk2_prev[0] = xk2; // st52_qiyuanx3: the first value will update to xk

    // Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    // GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    //     SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //     SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //     SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
    //     SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //     SpibRegs.SPITXBUF = 0xDA;
    //     SpibRegs.SPITXBUF = 500; // PWM value
    //     SpibRegs.SPITXBUF = 2200; // PWM Value
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = 0xBA00;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;

    // if ((numTimer0calls%50) == 0) {
    //	// Blink LaunchPad Red LED
    //	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    // }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    // Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;   // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;   // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0;      // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0;     // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0;        // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0;        // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0;          // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;     // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    // setup QEP2 pins for input
    EALLOW;
    // Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;   // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;   // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0;          // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0;         // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0;            // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0;            // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0;              // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF;         // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;     // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; // 4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts/one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw * (2 * PI / 12000));
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; // 4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts/one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw * (2 * PI / 12000));
}

void setEPWM2A(float controleffort) {
    // -10 mean 0 and 10 mean 1250 and 0 means 1250
    if (controleffort > 10.0) {
        controleffort = 10.0;
    }
    if (controleffort < -10.0) {
        controleffort = -10.0;
    }

    float scaled_val = (controleffort + 10.0) / 20.0;

    EPwm2Regs.CMPA.bit.CMPA = (float)(EPwm2Regs.TBPRD) * scaled_val;
}

void setEPWM2B(float controleffort) {
    if (controleffort > 10.0) {
        controleffort = 10.0;
    }
    if (controleffort < -10.0) {
        controleffort = -10.0;
    }

    float scaled_val = (-controleffort + 10.0) / 20.0;

    EPwm2Regs.CMPB.bit.CMPB = (float)(EPwm2Regs.TBPRD) * scaled_val;
}