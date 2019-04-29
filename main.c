/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>
#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "oscilloscope.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "sysctl_pll.h"

//......DEFINES......//
#define ADC_OFFSET 2048
#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
#define ADC_BITS 12
#define VIN_RANGE 3.3
#define PIXELS_PER_DIV 20
#define ADC_BITS 12
#define ADC_OFFSET 2048
#define SYSTEM_CLOCK_MHZ 120            // [MHz] system clock frequency
#define EVENT1_PERIOD 5
#define TIMER1_PERIOD (SYSTEM_CLOCK_MHZ * EVENT1_PERIOD)


//......GLOBALS......//
volatile uint32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;  // Latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];            // Circular buffer
volatile uint16_t gSpectrum[1024];
volatile uint16_t spectrumDraw[1024];
volatile uint32_t gADCErrors;
volatile uint32_t ADCIndex;
volatile uint16_t triggerDirection = 0;
volatile uint32_t triggerVoltage = 2048;
volatile uint16_t samplingRateState = 11;
volatile uint16_t mode = 1; // Mode 1: Oscilloscope, Mode 0: Spectrum
volatile float VOLTS_PER_DIV = 0.5;
volatile uint16_t ADCDraw[128];
volatile uint16_t ADCDrawScaled[128];
uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second
volatile uint16_t voltageScaleState = 2;
uint32_t buttonResponseTime = 0;
uint32_t buttonLatency = 0;
uint32_t buttonMissedDeadlines = 0;
char string[30];
uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
uint32_t gpressed = 0;
////......FUNCTION PROTOTYPES......//
void ADCInit(void);
void GetWaveform(int Direction, uint16_t Voltage);
void initMain();
void drawGrid(tContext sContext);

#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

//......MAIN......//
int main(void)
{
    IntMasterDisable();
    initMain();
    /* Start BIOS */
    BIOS_start();
    return (0);
}

void ClockTask(UArg arg1, UArg arg2)
{
    Semaphore_post(Button_Semaphore); // Post to Button Semaphore
}
void ButtonTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    uint32_t t;
    uint32_t timer0_period = TimerLoadGet(TIMER0_BASE, TIMER_A) + 1;
    while (true)
    {
        Semaphore_pend(Button_Semaphore, BIOS_WAIT_FOREVER);
//        TIMER0_ICR_R = TIMER_ICR_TATOCINT; // clear interrupt flag
        t = timer0_period - TimerValueGet(TIMER0_BASE, TIMER_A);
        if (t > buttonLatency)
            buttonLatency = t; // measure latency
        gpressed = getButton();
        if (gpressed != 0)
        {
            Mailbox_post(Button_Mailbox, &gpressed, BIOS_WAIT_FOREVER);
        }
        if (Semaphore_getCount(Button_Semaphore))
        { // next event occurred
            buttonMissedDeadlines++;
            t = 2 * timer0_period; // timer overflowed since last event
        }
        else
        {
            t = timer0_period;
        }
        t -= TimerValueGet(TIMER0_BASE, TIMER_A);
        if (t > buttonResponseTime)
        {
            buttonResponseTime = t;
        }
    }
}
void UserInputTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    while (true)
    {
        uint32_t presses;
        Mailbox_pend(Button_Mailbox, &presses, BIOS_WAIT_FOREVER);
        ButtonPress(presses);
        Semaphore_post(Display_Semaphore);
    }
}
void DisplayTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontCmss12);     // Select font
    tRectangle rectFullScreen = { 0, 0, GrContextDpyWidthGet(&sContext) - 1,
                                  GrContextDpyHeightGet(&sContext) - 1 };
    while (true)
    {
        Semaphore_pend(Display_Semaphore, BIOS_WAIT_FOREVER);
        // Local variables to prevent shared data issues
        uint16_t voltageScaleStateLocal = voltageScaleState;
        uint16_t samplingRateStateLocal = samplingRateState;
        uint16_t triggerDirectionLocal = triggerDirection;
        if (mode)
        {
            // Fill screen with black
            GrContextForegroundSet(&sContext, ClrBlack);
            GrRectFill(&sContext, &rectFullScreen);

            // Draw all elements on the LCD screen
            drawWaveform(sContext, triggerDirectionLocal,
                         voltageScaleStateLocal, samplingRateStateLocal);

            // Draw waveform
            GrContextForegroundSet(&sContext, ClrYellow); // Yellow waveform
            int i;
            for (i = 0; i < 127; i++)
            {
                int currentY = ADCDrawScaled[i];
                int nextY = ADCDrawScaled[i + 1];
                GrLineDraw(&sContext, i, currentY, i + 1, nextY);
            }
            GrFlush(&sContext); // Update the LCD display
        }
        else
        {
            GrContextForegroundSet(&sContext, ClrBlack);
            GrRectFill(&sContext, &rectFullScreen);
            drawSpectrum(sContext, samplingRateStateLocal);
            GrFlush(&sContext);
        }
    }
}
void WaveformTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    while (true)
    {
        Semaphore_pend(Waveform_Semaphore, BIOS_WAIT_FOREVER);
        if (mode)
        {
            uint16_t triggerDirectionLocal = triggerDirection;
            getWaveform(triggerDirectionLocal, triggerVoltage);
        }
        else
        {
            getSpectrum();
        }
        Semaphore_post(Processing_Semaphore); // Signal Processing task
    }
}
void ProcessingTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // Complex waveform and spectrum buffers
    int i;
    static float w[NFFT]; // Window function

    for (i = 0; i < NFFT; i++)
    {
        // Blackman window
        w[i] = 0.42f - 0.5f * cosf(2 * PI * i / (NFFT - 1))
                + 0.08f * cosf(4 * PI * i / (NFFT - 1));
    }

    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    while (true)
    {
        Semaphore_pend(Processing_Semaphore, BIOS_WAIT_FOREVER);
        if (mode)
        {
            int i;
            float VOLTS_PER_DIV_LOCAL = VOLTS_PER_DIV;
            float fScale = (VIN_RANGE * PIXELS_PER_DIV)
                    / ((1 << ADC_BITS) * VOLTS_PER_DIV_LOCAL);
            for (i = 0; i < 128; i++) //calculate y coordinate of each point in the buffer
            {
                ADCDrawScaled[i] = LCD_VERTICAL_MAX / 2
                        - (int) roundf(fScale * (ADCDraw[i] - ADC_OFFSET));
            }
        }
        else
        {
            for (i = 0; i < NFFT; i++)
            { // generate an input waveform
                in[i].r = gSpectrum[i] * sinf(20 * PI * i / NFFT) * w[i]; // real part of waveform
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            for (i = 0; i < NFFT; i++)
            {
                //Calculate magnitude = sqrt(r^2+i^2), then convert to dB
                float dBMagnitude = log10f(
                        sqrtf(out[i].r * out[i].r + out[i].i * out[i].i));
                spectrumDraw[i] = 120 - (int) roundf(17 * dBMagnitude); //round values, and apply an offset so they will accurately display on screen. convert to decibel
            }
        }
        Semaphore_post(Display_Semaphore); // Post to Display_Semaphore
        Semaphore_post(Waveform_Semaphore); // Post to Waveform_Semaphore after DisplayTask is finished
    }
}

// Initialize ADC
void ADCInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
                      pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
                      pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0); // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0,
    ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END); // in the 0th step, sample channel 3 (AIN3)

    // Enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0); // enable the sequence.  it is now sampling
    ADCIntEnable(ADC1_BASE, 0); // enable sequence 0 interrupt in the ADC1 peripheral
    IntPrioritySet(INT_ADC1SS0, 0);    // set ADC1 sequence 0 interrupt priority
    IntEnable(INT_ADC1SS0); // enable ADC1 sequence 0 interrupt in int. controller

    // Setup timer for ADC conversions trigger
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerEnable(TIMER2_BASE, TIMER_A);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);

}

// ISR for ADC
void ADC_ISR(void)
{
    ADC1_ISC_R = ADC_ISC_IN0; // clear ADC1 sequence0 interrupt flag in the ADCISC register
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) // check for ADC FIFO overflow
    {
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }
    gADCBuffer[gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)] =
    ADC1_SSFIFO0_R;         // read sample from the ADC1 sequence 0 FIFO
}

// Initialize hardware
void initMain()
{
    IntMasterDisable();
    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();
    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                                      120000000);
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation
    ButtonInit(); //Initialize buttons
    ADCInit(); //Initialize ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock / 100 - 1); // .01 sec interval
    IntMasterEnable();
}

