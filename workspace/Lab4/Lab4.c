// #############################################################################
//  FILE:   LABstarter_main.c
//
//  TITLE:  Lab Starter
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

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCD_ISR(void);
__interrupt void ADCA_ISR(void);
__interrupt void ADCB_ISR(void);

// st52_qiyuanx3: Global variables for simple 5 tap averaging filter

float xk3 = 0;
float xk2 = 0;


float xk_1 = 0;
float xk_2 = 0;
float xk_3 = 0;
float xk_4 = 0;
// yk is the filtered value

float yk3 = 0;
float yk2 = 0;

uint32_t fir_order = 201;
// b is the filter coefficients
// float b[5] = {0.2, 0.2, 0.2, 0.2, 0.2}; // 0.2 is 1/5th therefore a 5 point average
float b[201]={	0.0000000000000000e+00,
	5.9310714489336756e-06,
	-3.1622967769327651e-05,
	-4.8555068453528292e-05,
	2.5439713965310787e-05,
	1.0636311700700322e-04,
	4.0943002353631231e-05,
	-1.3032220577822534e-04,
	-1.5574891529465397e-04,
	7.0196727362914057e-05,
	2.6540563723290351e-04,
	9.5064239493256373e-05,
	-2.8658107227730101e-04,
	-3.2812869717536937e-04,
	1.4279219385660838e-04,
	5.2410188833392109e-04,
	1.8294277247625515e-04,
	-5.3896326106340239e-04,
	-6.0433916889949103e-04,
	2.5796617561388676e-04,
	9.2991400672171173e-04,
	3.1911873330115218e-04,
	-9.2506458625588820e-04,
	-1.0213653948466521e-03,
	4.2955897881453775e-04,
	1.5265299949334193e-03,
	5.1669962831012530e-04,
	-1.4780355805854024e-03,
	-1.6110575355229172e-03,
	6.6918892199691841e-04,
	2.3496157697640721e-03,
	7.8606155208158456e-04,
	-2.2232290083023733e-03,
	-2.3968374448656050e-03,
	9.8502121171656831e-04,
	3.4229316777166867e-03,
	1.1336843329482866e-03,
	-3.1752530451682115e-03,
	-3.3908650312083569e-03,
	1.3807326691208015e-03,
	4.7551466832362818e-03,
	1.5612214506753590e-03,
	-4.3356876674700917e-03,
	-4.5919145822890450e-03,
	1.8547647431397482e-03,
	6.3376381887008809e-03,
	2.0648899499666564e-03,
	-5.6916767433324135e-03,
	-5.9841621743526663e-03,
	2.3999370146009441e-03,
	8.1434953680532256e-03,
	2.6352425605786147e-03,
	-7.2155447007137578e-03,
	-7.5370184926852038e-03,
	3.0034667501062182e-03,
	1.0127854273180437e-02,
	3.2573555775766913e-03,
	-8.8655093390223368e-03,
	-9.2060620474939629e-03,
	3.6474090771480448e-03,
	1.2229590813161625e-02,
	3.9114340065166935e-03,
	-1.0587477461206914e-02,
	-1.0935042168719135e-02,
	4.3094994905231235e-03,
	1.4374291177478096e-02,
	4.5738026088351863e-03,
	-1.2317824362199525e-02,
	-1.2658836216425077e-02,
	4.9643483253135274e-03,
	1.6478317025112887e-02,
	5.2182204687492078e-03,
	-1.3986978809293973e-02,
	-1.4307168102935730e-02,
	5.5849081756906699e-03,
	1.8453693204446470e-02,
	5.8174301026213534e-03,
	-1.5523568556509704e-02,
	-1.5808831901388178e-02,
	6.1441123839398966e-03,
	2.0213476370381896e-02,
	6.3448321692533404e-03,
	-1.6858833233933300e-02,
	-1.7096120342050141e-02,
	6.6165676009443137e-03,
	2.1677219511235468e-02,
	6.7761652158435911e-03,
	-1.7930985773174448e-02,
	-1.8109137162612209e-02,
	6.9801773411530743e-03,
	2.2776133888324963e-02,
	7.0910676403608403e-03,
	-1.8689202686073811e-02,
	-1.8799676503551140e-02,
	7.2175770031319480e-03,
	2.3457567606047107e-02,
	7.2744064368391557e-03,
	-1.9096947799859182e-02,
	-1.9134381704657800e-02,
	7.3172737971483352e-03,
	2.3688467798905991e-02,
	7.3172737971483352e-03,
	-1.9134381704657800e-02,
	-1.9096947799859182e-02,
	7.2744064368391557e-03,
	2.3457567606047107e-02,
	7.2175770031319480e-03,
	-1.8799676503551140e-02,
	-1.8689202686073811e-02,
	7.0910676403608403e-03,
	2.2776133888324963e-02,
	6.9801773411530743e-03,
	-1.8109137162612209e-02,
	-1.7930985773174448e-02,
	6.7761652158435911e-03,
	2.1677219511235468e-02,
	6.6165676009443137e-03,
	-1.7096120342050141e-02,
	-1.6858833233933300e-02,
	6.3448321692533404e-03,
	2.0213476370381896e-02,
	6.1441123839398966e-03,
	-1.5808831901388178e-02,
	-1.5523568556509704e-02,
	5.8174301026213534e-03,
	1.8453693204446470e-02,
	5.5849081756906699e-03,
	-1.4307168102935730e-02,
	-1.3986978809293973e-02,
	5.2182204687492078e-03,
	1.6478317025112887e-02,
	4.9643483253135274e-03,
	-1.2658836216425077e-02,
	-1.2317824362199525e-02,
	4.5738026088351863e-03,
	1.4374291177478096e-02,
	4.3094994905231235e-03,
	-1.0935042168719135e-02,
	-1.0587477461206914e-02,
	3.9114340065166935e-03,
	1.2229590813161625e-02,
	3.6474090771480448e-03,
	-9.2060620474939629e-03,
	-8.8655093390223368e-03,
	3.2573555775766913e-03,
	1.0127854273180437e-02,
	3.0034667501062182e-03,
	-7.5370184926852038e-03,
	-7.2155447007137578e-03,
	2.6352425605786147e-03,
	8.1434953680532256e-03,
	2.3999370146009441e-03,
	-5.9841621743526663e-03,
	-5.6916767433324135e-03,
	2.0648899499666564e-03,
	6.3376381887008809e-03,
	1.8547647431397482e-03,
	-4.5919145822890450e-03,
	-4.3356876674700917e-03,
	1.5612214506753590e-03,
	4.7551466832362818e-03,
	1.3807326691208015e-03,
	-3.3908650312083569e-03,
	-3.1752530451682115e-03,
	1.1336843329482866e-03,
	3.4229316777166867e-03,
	9.8502121171656831e-04,
	-2.3968374448656050e-03,
	-2.2232290083023733e-03,
	7.8606155208158456e-04,
	2.3496157697640721e-03,
	6.6918892199691841e-04,
	-1.6110575355229172e-03,
	-1.4780355805854024e-03,
	5.1669962831012530e-04,
	1.5265299949334193e-03,
	4.2955897881453775e-04,
	-1.0213653948466521e-03,
	-9.2506458625588820e-04,
	3.1911873330115218e-04,
	9.2991400672171173e-04,
	2.5796617561388676e-04,
	-6.0433916889949103e-04,
	-5.3896326106340239e-04,
	1.8294277247625515e-04,
	5.2410188833392109e-04,
	1.4279219385660838e-04,
	-3.2812869717536937e-04,
	-2.8658107227730101e-04,
	9.5064239493256373e-05,
	2.6540563723290351e-04,
	7.0196727362914057e-05,
	-1.5574891529465397e-04,
	-1.3032220577822534e-04,
	4.0943002353631231e-05,
	1.0636311700700322e-04,
	2.5439713965310787e-05,
	-4.8555068453528292e-05,
	-3.1622967769327651e-05,
	5.9310714489336756e-06,
	0.0000000000000000e+00};


float xk3_prev[200] = {0};
float xk2_prev[200] = {0};

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numTimer2calls = 0;

uint32_t numADCDcalls = 0;
uint32_t numADCAcalls = 0;

uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// ADCD global variables
uint16_t adcd0result;
uint16_t adcd1result;
float adcind0_scaled;


// st52_qiyuanx3: definding sclaing for for adca results as well as adcb results
uint16_t adca2result;
uint16_t adca3result;

float adcina2_scaled;
float adcina3_scaled;

uint16_t adcb0result;
float adcb0_scaled;

// This function sets DACA to the voltage between 0.0V and 3.0V passed to this function.
// If outside 0.0V to 3.0V the output is saturated at 0.0V to 3.0V
// Example code
// float myu = 2.25;
// setDACA(myu); // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta0 * 4096.0 / 3.0; // perform scaling of 0.0 – almost 3.0V to 0 - 4095
    if (DACOutInt > 4095)
        DACOutInt = 4095;
    if (DACOutInt < 0)
        DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}
void setDACB(float dacouta1) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta1 * 4096.0 / 3.0; // perform scaling of 0.0 – almost 3.0V to 0 - 4095
    if (DACOutInt > 4095)
        DACOutInt = 4095;
    if (DACOutInt < 0)
        DACOutInt = 0;
    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}

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


    //GPIO52
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.ADCD1_INT = &ADCD_ISR;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.ADCB1_INT = &ADCB_ISR;
    EDIS; // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 100000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA, 115200);

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
    EPwm7Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    EPwm7Regs.TBCTL.bit.CTRMODE = 3;    // freeze counter
    EPwm7Regs.ETSEL.bit.SOCASEL = 2;    // Select Event when counter equal to PRD
    EPwm7Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event (“pulse” is the same as“trigger”)
    EPwm7Regs.TBCTR = 0x0;              // Clear counter
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTL.bit.PHSEN = 0;      // Disable phase loading
    EPwm7Regs.TBCTL.bit.CLKDIV = 0;     // divide by 1 50Mhz Clock
    EPwm7Regs.TBPRD = 5000;            // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm7Regs.ETSEL.bit.SOCAEN = 1;  // enable SOCA
    EPwm7Regs.TBCTL.bit.CTRMODE = 0; // unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    // write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;                                 // set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;                                 // set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;                                 // set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6;                                 // set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // read calibration settings
    // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    // power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    // delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    // Select the channels to convert and end of conversion flag
    // Many statements commented out, To be used when using ADCA or ADCB.

    // ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have tobe A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose willtrigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will setINT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
   
    // ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have tobe B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 17; // EPWM7
    // AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have tobe B1
    // AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    // AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose willtrigger SOC1
    // AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have tobe B2
    // AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    // AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose willtrigger SOC2
    // AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have tobe B3
    // AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    // AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose willtrigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will setINT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    // ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;    // set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99;   // sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;    // set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99;   // sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC1
    // AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    // AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    // AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    // AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    // AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    // AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; // set to SOC1, the last converted, and it will set INT1flag ADCD1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;   // enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // make sure INT1 flag is cleared
    EDIS;

    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0;   // load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1;  // use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;   // load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;  // use ADC VREF as reference voltage
    EDIS;
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT8; // SCIC SCID
    IER |= M_INT9; // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Enable ADCD1 in PIE: Group 1 interrupt 6
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

    // st52_qiyuanx3: Enable ADCA1 in PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // st52_qiyuanx3: Enable ADCB1 in PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;


    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    init_serialSCIC(&SerialC, 115200);
    init_serialSCID(&SerialD, 115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT; // Enable Global interrupt INTM
    ERTM; // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while (1) {
        if (UARTPrint == 1) {
            // serial_printf(&SerialA, "Num Timer2:%ld Num SerialRX: %ld\r\n", numTimer2calls, numRXA);
            serial_printf(&SerialA, "Microphone:%f\r\n", yk3);

            UARTPrint = 0;
        }
    }
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP"); // Wait one cycle
    EINT;              // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    numSWIcalls++;

    DINT;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void) {
    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls % 25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) { // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls % 50) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void) { numTimer1calls++; }

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void) {
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    // numTimer2calls++;

    // if ((numTimer2calls % 1) == 0) {
    //     UARTPrint = 1;
    // }
}

// adcd1 pie interrupt
// __interrupt void ADCD_ISR(void) {
//     adcd0result = AdcdResultRegs.ADCRESULT0;
//     adcd1result = AdcdResultRegs.ADCRESULT1;
//     // Here covert adcd0result to volts, saving in a global float variable

//     adcind0_scaled = adcd0result * 3.0 / 4096.0;

//     // Here write voltage value to DACA

//     setDACA(adcind0_scaled);

//     // Print ADCIND0’s voltage value to Tera Term every 100ms

//     numADCDcalls += 1;

//     if ((numADCDcalls % 100) == 0) {
//         UARTPrint = 1;
//     }

//     // Acknowledge that interrupt function is finished
//     // You should not add any code after these two lines.
//     AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear interrupt flag
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
// }

// __interrupt void ADCD_ISR(void) {
//     adcd0result = AdcdResultRegs.ADCRESULT0;
//     adcd1result = AdcdResultRegs.ADCRESULT1;

//     adcind0_scaled = adcd0result * 3.0 / 4096.0;

//     // Here covert ADCIND0, ADCIND1 to volts
//     xk = adcind0_scaled;
//     yk = b[0] * xk + b[1] * xk_1 + b[2] * xk_2 + b[3] * xk_3 + b[4] * xk_4;
//     // Save past states before exiting from the function so that next sample they are the older state
//     xk_4 = xk_3;
//     xk_3 = xk_2;
//     xk_2 = xk_1;
//     xk_1 = xk;
//     // Here write yk to DACA channel
//     setDACA(yk);

//     numADCDcalls += 1;
//     if ((numADCDcalls % 100) == 0) {
//         UARTPrint = 1;
//     }
//     // Print ADCIND0 and ADCIND1’s voltage value to Tera Term every 100ms
//     AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear interrupt flag
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
// }


__interrupt void ADCB_ISR(void) {
    adcb0result = AdcbResultRegs.ADCRESULT0;


    // Here covert ADCIND0, ADCIND1 to volts
    adcb0_scaled = adcb0result * 3.0 / 4096.0;

    GpioDataRegs.GPBSET.bit.GPIO52 = 1;
        //st52_qiyuanx3: xk sets to current read value, yk initialize to 0
    xk3 = adcb0_scaled;
    yk3 = 0;

    //st52_qiyuanx3: calculates the output voltage yk from coefficients b, current value xk, and previous 21 values 
    for (int i = 0; i < fir_order; i++) {
        if (i == 0) {
            yk3 += b[i] * xk3;
        } else {
            yk3 += b[i] * xk3_prev[i - 1];
        }
    }

    //st52_qiyuanx3: update the past states, start from the end, the value updates to the value of the index 1 before
    for (int i = fir_order - 2; i > 0; i--) {
        xk3_prev[i] = xk3_prev[i - 1];
    }
    xk3_prev[0] = xk3; // st52_qiyuanx3: the first value will update to xk

    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    setDACA(yk3+1.5);

    numADCAcalls += 1;
    if ((numADCAcalls % 100) == 0) {
        UARTPrint = 1;
    }
    // Print ADCIND0 and ADCIND1’s voltage value to Tera Term every 100ms
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void ADCA_ISR(void) {
    adca2result = AdcaResultRegs.ADCRESULT0;
    adca3result = AdcaResultRegs.ADCRESULT1;

    // // Here covert ADCIND0, ADCIND1 to volts
    // adcina2_scaled = adca2result * 3.0 / 4096.0;
    // adcina3_scaled = adca3result * 3.0 / 4096.0;

    
    
    // //st52_qiyuanx3: xk sets to current read value, yk initialize to 0
    // xk3 = adcina3_scaled;
    // yk3 = 0;

    // //st52_qiyuanx3: calculates the output voltage yk from coefficients b, current value xk, and previous 21 values 
    // for (int i = 0; i < fir_order; i++) {
    //     if (i == 0) {
    //         yk3 += b[i] * xk3;
    //     } else {
    //         yk3 += b[i] * xk3_prev[i - 1];
    //     }
    // }

    // //st52_qiyuanx3: update the past states, start from the end, the value updates to the value of the index 1 before
    // for (int i = fir_order - 2; i > 0; i--) {
    //     xk3_prev[i] = xk3_prev[i - 1];
    // }
    // xk3_prev[0] = xk3; // st52_qiyuanx3: the first value will update to xk

    


    // //st52_qiyuanx3: xk sets to current read value, yk initialize to 0
    // xk2 = adcina2_scaled;
    // yk2 = 0;

    // //st52_qiyuanx3: calculates the output voltage yk from coefficients b, current value xk, and previous 21 values 
    // for (int i = 0; i < fir_order; i++) {
    //     if (i == 0) {
    //         yk2 += b[i] * xk2;
    //     } else {
    //         yk2 += b[i] * xk2_prev[i - 1];
    //     }
    // }

    // //st52_qiyuanx3: update the past states, start from the end, the value updates to the value of the index 1 before
    // for (int i = fir_order - 2; i > 0; i--) {
    //     xk2_prev[i] = xk2_prev[i - 1];
    // }
    // xk2_prev[0] = xk2; // st52_qiyuanx3: the first value will update to xk

    // numADCAcalls += 1;
    // if ((numADCAcalls % 100) == 0) {
    //     UARTPrint = 1;
    // }
    // Print ADCIND0 and ADCIND1’s voltage value to Tera Term every 100ms
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


__interrupt void ADCD_ISR(void) {
    adcd0result = AdcdResultRegs.ADCRESULT0;
    adcd1result = AdcdResultRegs.ADCRESULT1;

    // // Here covert ADCIND0, ADCIND1 to volts
    // adcind0_scaled = adcd0result * 3.0 / 4096.0;

    
    
    // //st52_qiyuanx3: xk sets to current read value, yk initialize to 0
    // xk3 = adcind0_scaled;
    // yk3 = 0;

    // //st52_qiyuanx3: calculates the output voltage yk from coefficients b, current value xk, and previous 21 values 
    // for (int i = 0; i < fir_order; i++) {
    //     if (i == 0) {
    //         yk3 += b[i] * xk3;
    //     } else {
    //         yk3 += b[i] * xk_prev[i - 1];
    //     }
    // }

    // //st52_qiyuanx3: update the past states, start from the end, the value updates to the value of the index 1 before
    // for (int i = fir_order - 2; i > 0; i--) {
    //     xk_prev[i] = xk_prev[i - 1];
    // }
    // xk_prev[0] = xk3; // st52_qiyuanx3: the first value will update to xk

    // // st52_qiyuanx3:  write yk to DACA channel
    // setDACA(yk3);

    // numADCDcalls += 1;
    // if ((numADCDcalls % 100) == 0) {
    //     UARTPrint = 1;
    // }
    // Print ADCIND0 and ADCIND1’s voltage value to Tera Term every 100ms
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
