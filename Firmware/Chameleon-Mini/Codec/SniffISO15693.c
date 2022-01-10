/*
 * SniffISO15693.c
 *
 *  Created on: 25.01.2017
 *      Author: ceres-c & MrMoDDoM
 */

#include "SniffISO15693.h"
#include "Codec.h"
#include "../System.h"
#include "../Application/Application.h"
#include "LEDHook.h"
#include "AntennaLevel.h"
#include "Terminal/Terminal.h"


#define SOC_1_OF_4_CODE         0x7B
#define SOC_1_OF_256_CODE       0x7E
#define SOC_ONE_SUBCARRIER      0xFF
#define SOC_TWO_SUBCARRIER      0x1FF
#define EOC_CODE                0xDF
#define ISO15693_READER_SAMPLE_CLK      TC_CLKSEL_DIV2_gc // 13.56MHz
#define ISO15693_READER_SAMPLE_PERIOD   128 // 9.4us
#define ISO15693_CARD_SAMPLE_CLK        TC_CLKSEL_DIV4_gc /* Max possible sampling resolution */
#define ISO15693_CARD_SAMPLE_PERIOD     148 // 18.5us

#define WRITE_GRID_CYCLES       4096
#define SUBCARRIER_1            32
#define SUBCARRIER_2            28
#define SUBCARRIER_OFF          0
#define SOF_PATTERN             0x1D // 0001 1101 
#define EOF_PATTERN             0xB8 // 1011 1000

// These registers provide quick access but are limited
// so global vars will be necessary
#define DataRegister            Codec8Reg0
#define StateRegister           Codec8Reg1
#define ModulationPauseCount    Codec8Reg2
#define SampleRegister          CodecCount16Register1
// #define SampleRegister          Codec8Reg3
// #define BitSent                 CodecCount16Register1
#define BitSampleCount          CodecCount16Register2
#define CodecBufferPtr          CodecPtrRegister1

#define CODEC_18uS_SLOT_TIMER CODEC_READER_TIMER

static volatile struct {
    volatile bool ReaderDemodFinished;
    volatile bool CardDemodFinished;
} Flags = { 0 };

typedef enum {
    DEMOD_SOC_STATE,
    DEMOD_1_OUT_OF_4_STATE,
    DEMOD_1_OUT_OF_256_STATE
} DemodStateType;

static volatile DemodStateType DemodState;
static volatile uint8_t ShiftRegister;
static volatile uint8_t ByteCount;
static volatile uint8_t bDualSubcarrier;
static volatile uint16_t DemodByteCount;
static volatile uint16_t AppReceivedByteCount;
static volatile uint16_t BitRate1;
static volatile uint16_t BitRate2;
static volatile uint16_t SampleDataCount;
/* First part of SOC is 24 bits long if in single subcarrier mode or 27 bits if in double subcarrier mode.
 * The CardSOCBitsCount variable is 8 if in single subcarrier or 9 if in double subcarrier,
 * and represents the size of a unit (how many bits we're checking at a time).
 */
static volatile uint8_t CardSOCBitsCount;
/* The CardSOCBytesCounter variable holds the number of units (of which the length is defined by CardSOCBitsCount)
 * we have already checked during Card data demodulation.
 */
static volatile uint8_t CardSOCBytesCounter;

INLINE void SNIFF_ISO15693_READER_EOC_VCD(void);
INLINE void CardSniffInit(void);

/////////////////////////////////////////////////
// VCD->VICC
/////////////////////////////////////////////////

/* This function implements CODEC_DEMOD_IN_INT0_VECT interrupt vector.
 * It is called when a pulse is detected in CODEC_DEMOD_IN_PORT (PORTB).
 * The relevatn interrupt vector is registered to CODEC_DEMOD_IN_MASK0 (PIN1) via:
 * CODEC_DEMOD_IN_PORT.INT0MASK = CODEC_DEMOD_IN_MASK0;
 * and unregistered writing the INT0MASK to 0
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_DEMOD_READER_IN_INT0_VECT(void) {
    /* Start sample timer CODEC_TIMER_SAMPLING (TCD0).
     * Set Counter Channel C (CCC) with relevant bitmask (TC0_CCCIF_bm),
     * the period for clock sampling is specified in StartSniffISO15693Demod.
     */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;
    /* Sets register INTCTRLB to TC_CCCINTLVL_HI_gc = (0x03<<4) to enable compare/capture for high level interrupts on Channel C (CCC) */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCCINTLVL_HI_gc;

    /* Disable this interrupt as we've already sensed the relevant pulse and will use our internal clock from now on */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;
}

/* This function is registered to CODEC_TIMER_SAMPLING (TCD0)'s Counter Channel C (CCC).
 * When the timer is enabled, this is called on counter's overflow
 * 
 * It demodulates bits received from the reader and saves them in CodecBuffer.
 * 
 * It disables its own interrupt when receives an EOF (calling ISO15693_EOC) or when it receives garbage
 */
ISR_SHARED SNIFF_ISO15693_READER_CODEC_TIMER_SAMPLING_CCC_VECT(void) {
    /* Shift demod data */
    SampleRegister = (SampleRegister << 1) | (!(CODEC_DEMOD_IN_PORT.IN & CODEC_DEMOD_IN_MASK) ? 0x01 : 0x00);

    if (++BitSampleCount == 8) {
        BitSampleCount = 0;
        switch (DemodState) {
            case DEMOD_SOC_STATE:
                if (SampleRegister == SOC_1_OF_4_CODE) {
                    DemodState = DEMOD_1_OUT_OF_4_STATE;
                    SampleDataCount = 0;
                    ModulationPauseCount = 0;
                } else if (SampleRegister == SOC_1_OF_256_CODE) {
                    DemodState = DEMOD_1_OUT_OF_256_STATE;
                    SampleDataCount = 0;
                } else { // No SOC. Restart and try again, we probably received garbage.
                    Flags.ReaderDemodFinished = 1;
                    Flags.CardDemodFinished = 1;
                    /* Sets timer off for CODEC_TIMER_SAMPLING (TCD0) disabling clock source */
                    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
                    /* Sets register INTCTRLB to 0 to disable all compare/capture interrupts */
                    CODEC_TIMER_SAMPLING.INTCTRLB = 0;
                }
                break;

            case DEMOD_1_OUT_OF_4_STATE:
                if (SampleRegister == EOC_CODE) {
                    SNIFF_ISO15693_READER_EOC_VCD();
                } else {
                    uint8_t SampleData = ~SampleRegister;
                    if (SampleData == (0x01 << 6)) {
                        /* ^_^^^^^^ -> 00 */
                        ModulationPauseCount++;
                        DataRegister >>= 2;

                    } else if (SampleData == (0x01 << 4)) {
                        /* ^^^_^^^^ -> 01 */
                        ModulationPauseCount++;
                        DataRegister >>= 2;
                        DataRegister |= 0b01 << 6;

                    } else if (SampleData == (0x01 << 2)) {
                        /* ^^^^^_^^ -> 10 */
                        ModulationPauseCount++;
                        DataRegister >>= 2;
                        DataRegister |= 0b10 << 6;

                    } else if (SampleData == (0x01 << 0)) {
                        /* ^^^^^^^_ -> 11 */
                        ModulationPauseCount++;
                        DataRegister >>= 2;
                        DataRegister |= 0b11 << 6;
                    }

                    if (ModulationPauseCount == 4) {
                        ModulationPauseCount = 0;
                        *CodecBufferPtr = DataRegister;
                        ++CodecBufferPtr;
                        ++ByteCount;
                    }
                }
                break;

            case DEMOD_1_OUT_OF_256_STATE:
                if (SampleRegister == EOC_CODE) {
                    SNIFF_ISO15693_READER_EOC_VCD();
                } else {
                    uint8_t Position = ((SampleDataCount / 2) % 256) - 1;
                    uint8_t SampleData = ~SampleRegister;

                    if (SampleData == (0x01 << 6)) {
                        /* ^_^^^^^^ -> N-3 */
                        DataRegister = Position - 3;
                        ModulationPauseCount++;

                    } else if (SampleData == (0x01 << 4)) {
                        /* ^^^_^^^^ -> N-2 */
                        DataRegister = Position - 2;
                        ModulationPauseCount++;

                    } else if (SampleData == (0x01 << 2)) {
                        /* ^^^^^_^^ -> N-1 */
                        DataRegister = Position - 1;
                        ModulationPauseCount++;

                    } else if (SampleData == (0x01 << 0)) {
                        /* ^^^^^^^_ -> N-0 */
                        DataRegister = Position - 0;
                        ModulationPauseCount++;
                    } 

                    if (ModulationPauseCount == 1) {
                        ModulationPauseCount = 0;
                        *CodecBufferPtr = DataRegister;
                        ++CodecBufferPtr;
                        ++ByteCount;
                    } 
                }
                break;
        }
        SampleRegister = 0;
    }
    SampleDataCount++;
}

/* This function is called from isr_ISO15693_CODEC_TIMER_SAMPLING_CCC_VECT
 * when we have 8 bits in SampleRegister and they represent an end of frame.
 */
INLINE void SNIFF_ISO15693_READER_EOC_VCD(void) {
    /* Now we setup the free running timer to timestamp the events on TCD0.
     * The period is set to 1 ms: on overflow we assume a NO_RESPONSE from the card (timeout).
     * So we set an interrupt to call a function to revert the chameleon's state to sniff the reader.
     * 
     * The overflow should ~~never~~ be reached since the other counter will reset this timer on every
     * bit-frame (17 pulses).
     */
    CODEC_TIMER_SAMPLING.CNT = 0;
    CODEC_TIMER_SAMPLING.PER = 13560; // 1 ms
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_DIV2_gc; 
    CODEC_TIMER_SAMPLING.CTRLE = 0 ; // Normal 16-bit mode
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH7_gc ;
    /* ENable Overflow IRQ */
    CODEC_TIMER_SAMPLING.CNT = 0;
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_OVFIF_bm; /* Register overflow handler to revert to reader sniff (card did not answer) */
    CODEC_TIMER_SAMPLING.INTCTRLA = TC_OVFINTLVL_HI_gc;
    /* Overflow ISR is registered below */

    /* Finally mark reader data as read */
    Flags.ReaderDemodFinished = 1;

    /* And initialize VICC->VCD sniffer */
    CardSniffInit(); // TODO_sniff can this be moved to CodecTask or would it be too slow and we'd loose some bits?
}

/* This function is registered to CODEC_TIMER_SAMPLING (TCD0)'s period overflow
 * after EOF is received from the reader. Given how TCD0 is used to wait for
 * the first card modulation pulse, this interrupt is called when the card does
 * not answer the reader. It then reinits the chameleon to sniff reader data.
 */
ISR(CODEC_TIMER_SAMPLING_OVF_VECT) {
    CODEC_TIMER_SAMPLING.INTCTRLA = 0;
    isr_func_TCD0_CCC_vect = &SNIFF_ISO15693_READER_CODEC_TIMER_SAMPLING_CCC_VECT;
    CODEC_TIMER_SAMPLING.CTRLA = ISO15693_READER_SAMPLE_CLK;
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;  // TODO Why writing to a FLAG register? We need only to READ this...

    Flags.CardDemodFinished = 1;
}

/////////////////////////////////////////////////
// VICC->VCD
/////////////////////////////////////////////////

/* Currently implemented only single subcarrier SOF detection
 * TODO extract to single subcarrier specific function and call this or double subcarrier
 */
INLINE void CardSniffInit(void) {
    DACB.CH0DATA = 16*50; // TODO sniff remove this forced threshold, identifiy a value automatically

    /* Enable the analog comparator AC interrupt */
    ACA.AC1CTRL = 0; // TODO_sniff why resetting AC1 CTRL? We're not using it
    ACA.STATUS = AC_AC0IF_bm; /* Analog Comparator 0 Interrupt Flag bit mask. */
    /* enable AC | high speed mode | no hysteresis | sample on rising edge | interrupt level high */
    ACA.AC0CTRL = AC_ENABLE_bm | AC_HSMODE_bm | AC_HYSMODE_NO_gc | AC_INTMODE_RISING_gc | AC_INTLVL_HI_gc;

    /* Use the event system for resetting the pause-detecting timer CODEC_TIMER_LOADMOD (TCE0). */
    EVSYS.CH2MUX = EVSYS_CHMUX_ACA_CH0_gc; // on every ACA_AC0 INT
    EVSYS.CH2CTRL = EVSYS_DIGFILT_1SAMPLE_gc;

    /**
     * CODEC_TIMER_TIMESTAMPS (TCD1) will be used to count the peaks identified by ACA while sniffing VICC data
     */
    CODEC_TIMER_TIMESTAMPS.CTRLA = TC_CLKSEL_EVCH2_gc; /* Using Event channel 2 as an input */
    CODEC_TIMER_TIMESTAMPS.CNT = 0; /* Clear counter */ // TODO clear this counter if we happen to find some interference before actually hitting the SOF
    CODEC_TIMER_TIMESTAMPS.PER = 0xFFFF; /* Don't limit this value, will be cleared manually */
    CODEC_TIMER_TIMESTAMPS.CCB = 0xFFFF; /* Don't limit this value, will be cleared manually */
    CODEC_TIMER_TIMESTAMPS.INTCTRLA = 0; /* No error/overflow interrupt */
    // CODEC_TIMER_TIMESTAMPS.INTFLAGS = TC1_CCBIF_bm; // Clear interrupt flag // TODO_sniff we don't need interrupts on this counter, right?
    CODEC_TIMER_TIMESTAMPS.INTCTRLB = TC_CCBINTLVL_HI_gc; /* Register to high interrupt level */

    /**
     * CODEC_TIMER_LOADMOD (TCE0) will be used to identify the pause in SOF when in single subcarrier mode
     * SOF: 0000000000000000000000001111111111111111111111110000000011111111
     *                                 |rrrrrrrrrrrrrrrrrrrrO|
     *                                 |                    The timer will overflow here because modulations stopped
     *                                 |                    and it was not reset. We can start the 18,88 us half-bit timer.
     *                                 |
     *                                 We start catching modulations somewhere around here and enable this timer.
     *                                 Every modulation resets this timer
     */
    CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_DIV1_gc; /* Clocked at 27.12 MHz */
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH2_gc; /* Restart this timer on every event on channel 2 (pulse detected by AC) */
    CODEC_TIMER_LOADMOD.CNT = 0; /* Clear counter */
    CODEC_TIMER_LOADMOD.PER = 0xffff; /* Stupid high value because we're going to use CCB. CCB is already shared and won't conflict with other codecs */
    CODEC_TIMER_LOADMOD.CCB = 64; /* Given clock = 27.12 MHz, this is exactly the duration of 1 pulse at f_c/32 */
    CODEC_TIMER_LOADMOD.INTCTRLA = 0; /* No error/overflow interrupt */
    // TODO_sniff we could use an interrupt on overflow, but our ISR has to cooperate with other codecs which might overflow this counter as well
    CODEC_TIMER_LOADMOD.INTFLAGS = TC1_CCBIF_bm; /* Register interrupt to Compare/Capture channel B */
    /* Interrupt will be enabled later, writing INTFLAGS register, when we're done sensing data from VCD */

}

INLINE void CardSniffDeinit(void) {
    // Reset ACA AC0 to default setting
    ACA.AC0MUXCTRL = AC_MUXPOS_DAC_gc | AC_MUXNEG_PIN7_gc;
    ACA.AC0CTRL = CODEC_AC_DEMOD_SETTINGS;

    // TODO disable evenst system

    // TODO disable all other interrupts
}

// This interrupt waits for Card -> Reader SOC
ISR_SHARED isr_SNIFF_ISO15693_ACA_AC0_VECT(void) { // this interrupt either finds the SOC or gets triggered before
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    /* TODO peak threshold finding idea:
     *  - get "silence" level during the pause after we reader EOC
     *  - set silence*3 (or appropriate value) as ACA threshold
     * Then use event routing system to automatically update this value:
     *  - on n-th ACA hit, trigger ADC sampling => will fetch the most recent "high" value  (check if we have enough clock cycles)
     *  - when ADC is done sampling, trigger ADC ISR to multiply that value by 0,5 and save it as ACA threshold
    */

    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCBINTLVL_HI_gc; /* Actually enable CCB interrupt (handled by isr_func_CODEC_TIMER_LOADMOD_CCB_VECT) */

    ACA.AC0CTRL &= ~AC_INTLVL_HI_gc; /* Disable this interrupt */

    // TODO_sniff re-enable this interrupt somewhere else if no SOF match after this interrupt signals it as theoretically found (false positive)

    // TODO to use AC as pulse counter:
    //  1) change event trigger to falling edge (currently using raising edge)
    //  2) route this to source a counter (or set it to 1 if already routed). Better to have it already routed so we can count this pulse as well.

    // TODO stop TCD0 or disable overflow interrupt (we have found a SOF)
}

/**
 * Find the modulation pause inside VICC->VCD SOF when in single subcarrier
 * SOF: 0000000000000000000000001111111111111111111111110000000011111111
 *                                                      ^
 *                                                      We're here in the SOF now
 * Actually we're late by 64 clock cycles from the latest pulse due to CCB = 64.
 * TODO We should account for those lost 64 cycles setting the CNT value for the next interrupt accordingly
 */
ISR_SHARED isr_SNIFF_ISO15693_SOF_SSC_PAUSE(void) { // Registered to CODEC_TIMER_LOADMOD CCB
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    // TODO this value now contains the number of received pulses (if a proper threshold has been set). Use it.
    CODEC_TIMER_TIMESTAMPS.CNT; // TODO there are some fluctuations in this value, 24 < CNT < 26. Probably due to threshold identification

    // TODO:
    //  1) start the 18,88us timer (account for 64 lost cycles)
    //  2) Check and clear CODEC_TIMER_TIMESTAMPS.CNT
    //  3) Set codec demod status to SSC_SOF
    //  4) Start actual demod (skipping first bit which is still part of the SOF)

    /* Disable this interrupt */
    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCDINTLVL_OFF_gc;
}


















// /* This is a test function to check if interrupts work
//  * Performs action on every 18.5us
//  */
// ISR_SHARED isr_SNIFF_ISO15693_CARD_CODEC_TIMER_SAMPLING_CCC_VECT(void) {
//     LEDHook(LED_CODEC_RX, LED_PULSE);
//     if(Flags.CardDemodFinished == 0){
//         if (EVSYS.CH7MUX == EVSYS_CHMUX_TCE0_OVF_gc) {
//             EVSYS.CH7MUX = EVSYS_CHMUX_TCE0_CCA_gc;
//         } else if (EVSYS.CH7MUX == EVSYS_CHMUX_TCE0_CCA_gc) {
//             EVSYS.CH7MUX = EVSYS_CHMUX_TCE0_CCB_gc;
//         } else if (EVSYS.CH7MUX == EVSYS_CHMUX_TCE0_CCB_gc) {
//             EVSYS.CH7MUX = EVSYS_CHMUX_TCE0_CCA_gc;
//         }

//        /* All this logic is not working */
//        /* So just reverting back to Reader sniffing */
//         CODEC_TIMER_TIMESTAMPS.CTRLA = 0;
//         isr_func_TCD0_CCC_vect = &SNIFF_ISO15693_READER_CODEC_TIMER_SAMPLING_CCC_VECT;
//         CODEC_TIMER_SAMPLING.CTRLA = ISO15693_READER_SAMPLE_CLK;
//         CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;
//         CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;  // TODO Why writing to a FLAG register? We need only to READ this...

//         Flags.CardDemodFinished = 1;

//         CODEC_TIMER_LOADMOD.CNT = 0;
//         CODEC_TIMER_SAMPLING.CNT = 0;
//         CODEC_TIMER_TIMESTAMPS.CNT = 0;
//     }
// }


// /* This is a test function to check if interrupts work 
//  * It waits until a pulse is detected and then enables timers/counters
//  */
// ISR_SHARED isr_SNIFF_ISO15693_CODEC_DEMOD_CARD_IN_INT0_VECT(void) {
//     // TODO timeout and disable interrupts if no data is sent from the card
//     // TODO we might need to set here the pulse counter to 1 because the first pulse could be missed

//     /* Start sample timer CODEC_TIMER_SAMPLING (TCD0).
//      * Set Counter Channel C (CCC) with relevant bitmask (TC0_CCCIF_bm),
//      * the period for clock sampling is specified in SNIFF_ISO15693_READER_EOC_VCD.
//      */
//     CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;
//     /* Sets register INTCTRLB to TC_CCCINTLVL_HI_gc = (0x03<<4) to enable compare/capture for high level interrupts on Channel C (CCC) */
//     CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCCINTLVL_HI_gc;

//     /* Disable this interrupt as we've already sensed the relevant pulse and will use our internal clock from now on */
//     CODEC_DEMOD_IN_PORT.INT0MASK = 0;
// }

/* This function is called once we have received the card SOF and
 * sets the corect value and interrupt in the counter
 */
void isr_SNIFF_ISO15693_CARD_SNIFF_PREPARE_COUNTERS(){
    CODEC_TIMER_LOADMOD.PER = 17; // bit-frame's pulses
    ; // First interrupt to read the time after 8 pulses
    ; // Second interrupt to read the time after 16 pulses and compare the two value
}

/* This functions resets all global variables used in the codec and enables interrupts to wait for reader data */
void StartSniffISO15693Demod(void) {
    /* Reset global variables to default values */
    CodecBufferPtr = CodecBuffer;
    Flags.ReaderDemodFinished = 0;
    Flags.CardDemodFinished = 0;
    DemodState = DEMOD_SOC_STATE;
    // StateRegister = LOADMOD_WAIT; // TODO set to card demod or reader demod
    DataRegister = 0;
    SampleRegister = 0;
    BitSampleCount = 0;
    SampleDataCount = 0;
    ModulationPauseCount = 0;
    ByteCount = 0;
    ShiftRegister = 0;

    /* Activate Power for demodulator */
    CodecSetDemodPower(true);

    /* Configure sampling-timer free running and sync to first modulation-pause. */
    /* Resets the counter to 0 */
    CODEC_TIMER_SAMPLING.CNT = 0;
    /* Set the period for CODEC_TIMER_SAMPLING (TCD0) to ISO15693_READER_SAMPLE_PERIOD - 1 because PER is 0-based */
    CODEC_TIMER_SAMPLING.PER = ISO15693_READER_SAMPLE_PERIOD - 1;
    /* Set Counter Channel C (CCC) register with half bit period - 1. (- 14 to compensate ISR timing overhead) */
    CODEC_TIMER_SAMPLING.CCC = ISO15693_READER_SAMPLE_PERIOD / 2 - 14 - 1;
    /* Set timer for CODEC_TIMER_SAMPLING (TCD0) to ISO15693_READER_SAMPLE_CLK = TC_CLKSEL_DIV2_gc = System Clock / 2
     *
     * TODO Why system clock / 2 and not iso period?
     */
    CODEC_TIMER_SAMPLING.CTRLA = ISO15693_READER_SAMPLE_CLK;
    /* Set event action for CODEC_TIMER_SAMPLING (TCD0) to restart and trigger CODEC_TIMER_MODSTART_EVSEL = TC_EVSEL_CH0_gc = Event Channel 0 */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;
    /* Set Counter Channel C (CCC) with relevant bitmask (TC0_CCCIF_bm), the period for clock sampling is specified above */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;  // TODO Why writing to a FLAG register? We need only to READ this...
    /* Sets register INTCTRLB to TC_CCCINTLVL_OFF_gc = (0x00<<4) to disable compare/capture C interrupts
     * It is now turned off in order to wait for the first pulse and start sampling once it occurs,
     * will be enabled in isr_ISO15693_CODEC_DEMOD_IN_INT0_VECT
     */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCCINTLVL_OFF_gc;

    /* Set event action for CODEC_TIMER_LOADMOD (TCE0) to restart and trigger CODEC_TIMER_MODSTART_EVSEL = TC_EVSEL_CH0_gc = Event Channel 0 */
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;
    /* Set the period for CODEC_TIMER_LOADMOD (TCE0) to... some magic numbers?
     * Using PER instead of PERBUF breaks it when receiving ISO15693_APP_NO_RESPONSE from Application.
     *
     * TODO What are these numbers?
     */
    CODEC_TIMER_LOADMOD.PERBUF = 4192 + 128 + 128 - 1;
    /* Sets register INTCTRLA to 0 to disable timer error or overflow interrupts */
    CODEC_TIMER_LOADMOD.INTCTRLA = 0;
    /* Sets register INTCTRLB to 0 to disable all compare/capture interrupts */
    CODEC_TIMER_LOADMOD.INTCTRLB = 0;
    /* Set timer for CODEC_TIMER_SAMPLING (TCD0) to TC_CLKSEL_EVCH6_gc = Event Channel 6 */
    CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_EVCH6_gc;

    /* Start looking out for modulation pause via interrupt. */
    /* Sets register INTFLAGS to PORT_INT0LVL_HI_gc = (0x03<<0) to enable compare/capture for high level interrupts on CODEC_DEMOD_IN_PORT (PORTB) */
    CODEC_DEMOD_IN_PORT.INTFLAGS = PORT_INT0LVL_HI_gc;
    /* Sets INT0MASK to CODEC_DEMOD_IN_MASK0 = PIN1_bm to use it as source for port interrupt 0 */
    CODEC_DEMOD_IN_PORT.INT0MASK = CODEC_DEMOD_IN_MASK0;
}

void SniffISO15693CodecInit(void) {
    // TODO_sniff temp code start
    PORTE.DIR = PIN0_bm;
    CodecThresholdReset(); // TODO_sniff do we need this?
    isr_func_ACA_AC0_vect = &isr_SNIFF_ISO15693_ACA_AC0_VECT; // TODO move to SSC Card init

    isr_func_CODEC_TIMER_LOADMOD_CCB_VECT = &isr_SNIFF_ISO15693_SOF_SSC_PAUSE; // Pause finder in SOC // TODO move to SSC init
    // TODO_sniff temp code end

    CodecInitCommon();

    // TODO move below code to Reader init
    /* Register isr_ISO15693_CODEC_TIMER_SAMPLING_CCC_VECT function
     * to CODEC_TIMER_SAMPLING (TCD0)'s Counter Channel C (CCC)
     */
    isr_func_TCD0_CCC_vect = &SNIFF_ISO15693_READER_CODEC_TIMER_SAMPLING_CCC_VECT;
    /* Register isr_ISO15693_CODEC_DEMOD_IN_INT0_VECT function
     * to CODEC_DEMOD_IN_PORT (PORTB) interrupt 0
     */
    isr_func_CODEC_DEMOD_IN_INT0_VECT = &isr_SNIFF_ISO15693_CODEC_DEMOD_READER_IN_INT0_VECT;

    StartSniffISO15693Demod();
}

void SniffISO15693CodecDeInit(void) {
    /* Gracefully shutdown codec */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;

    /* Reset global variables to default values */
    CodecBufferPtr = CodecBuffer;
    Flags.ReaderDemodFinished = 0;
    Flags.CardDemodFinished = 0;
    DemodState = DEMOD_SOC_STATE;
    // StateRegister = LOADMOD_WAIT; // TODO can this be removed?
    DataRegister = 0;
    SampleRegister = 0;
    BitSampleCount = 0;
    SampleDataCount = 0;
    ModulationPauseCount = 0;
    ByteCount = 0;
    ShiftRegister = 0;

    /* Disable sample timer */
    /* Sets timer off for CODEC_TIMER_SAMPLING (TCD0) disabling clock source */
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
    /* Disable event action for CODEC_TIMER_SAMPLING (TCD0) */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc;
    /* Sets register INTCTRLB to TC_CCCINTLVL_OFF_gc = (0x00<<4) to disable compare/capture C interrupts */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCCINTLVL_OFF_gc;
    /* Restore Counter Channel C (CCC) interrupt mask (TC0_CCCIF_bm) */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;

    CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OFF, 0);
    CodecSetDemodPower(false);
}


void DebugPrintTimerRegs(void){ // TODO remove this function

    char log_text[100];
    log_text[99] = '\0';
    snprintf(log_text, 99, "CODEC_TIMER_LOADMOD.CNT=%d", CODEC_TIMER_LOADMOD.CNT);
    LogEntry(LOG_INFO_GENERIC, log_text, strlen(log_text));
    snprintf(log_text, 99, "CODEC_TIMER_LOADMOD.PER=%d", CODEC_TIMER_LOADMOD.PER);
    LogEntry(LOG_INFO_GENERIC, log_text, strlen(log_text));
    snprintf(log_text, 99, "CODEC_TIMER_SAMPLING.CNT=%d", CODEC_TIMER_SAMPLING.CNT);
    LogEntry(LOG_INFO_GENERIC, log_text, strlen(log_text));
    snprintf(log_text, 99, "CODEC_TIMER_SAMPLING.PER=%d", CODEC_TIMER_SAMPLING.PER);
    LogEntry(LOG_INFO_GENERIC, log_text, strlen(log_text));

}

void SniffISO15693CodecTask(void) {
    if (Flags.ReaderDemodFinished) {
        
        Flags.ReaderDemodFinished = 0;

        DemodByteCount = ByteCount;
        AppReceivedByteCount = 0;
        bDualSubcarrier = 0;
        CardSOCBitsCount = 8;

        if (DemodByteCount > 0) {
            LogEntry(LOG_INFO_CODEC_SNI_READER_DATA, CodecBuffer, DemodByteCount);
            LEDHook(LED_CODEC_RX, LED_PULSE);

            if (CodecBuffer[0] & ISO15693_REQ_SUBCARRIER_DUAL) {
                bDualSubcarrier = 1;
                CardSOCBitsCount = 9;
            }
            // TODO enable card demodulation
            // TODO set DemodState as DEMOD_SOC_STATE

        }

    }

    if(Flags.CardDemodFinished) {
        Flags.CardDemodFinished = 0;
        StartSniffISO15693Demod();
    }
}
