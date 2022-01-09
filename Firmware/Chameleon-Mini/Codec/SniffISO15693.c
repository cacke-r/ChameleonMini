/*
 * SniffISO15693.c
 *
 *  Created on: 25.01.2017
 *      Author: ceres-c & MrMoDDoM
 */

#include "SniffISO15693.h"
#include "../System.h"
#include "../Application/Application.h"
#include "LEDHook.h"
#include "AntennaLevel.h"
#include "Terminal/Terminal.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>


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

/////////////////////////////////////////////////
// VCD->VICC
/////////////////////////////////////////////////

INLINE void SNIFF_ISO15693_READER_EOC_VCD(void);

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























/* This is a test function to check if interrupts work
 * Performs action on every 18.5us
 */
ISR_SHARED isr_SNIFF_ISO15693_CARD_CODEC_TIMER_SAMPLING_CCC_VECT(void) {
    LEDHook(LED_CODEC_RX, LED_PULSE);
    if(Flags.CardDemodFinished == 0){
        if (EVSYS.CH7MUX == EVSYS_CHMUX_TCE0_OVF_gc) {
            EVSYS.CH7MUX = EVSYS_CHMUX_TCE0_CCA_gc;
        } else if (EVSYS.CH7MUX == EVSYS_CHMUX_TCE0_CCA_gc) {
            EVSYS.CH7MUX = EVSYS_CHMUX_TCE0_CCB_gc;
        } else if (EVSYS.CH7MUX == EVSYS_CHMUX_TCE0_CCB_gc) {
            EVSYS.CH7MUX = EVSYS_CHMUX_TCE0_CCA_gc;
        }

       /* All this logic is not working */
       /* So just reverting back to Reader sniffing */
        CODEC_TIMER_TIMESTAMPS.CTRLA = 0;
        isr_func_TCD0_CCC_vect = &SNIFF_ISO15693_READER_CODEC_TIMER_SAMPLING_CCC_VECT;
        CODEC_TIMER_SAMPLING.CTRLA = ISO15693_READER_SAMPLE_CLK;
        CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;
        CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;  // TODO Why writing to a FLAG register? We need only to READ this...

        Flags.CardDemodFinished = 1;

        CODEC_TIMER_LOADMOD.CNT = 0;
        CODEC_TIMER_SAMPLING.CNT = 0;
        CODEC_TIMER_TIMESTAMPS.CNT = 0;
    }
}


/* This is a test function to check if interrupts work 
 * It waits until a pulse is detected and then enables timers/counters
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_DEMOD_CARD_IN_INT0_VECT(void) {
    // TODO timeout and disable interrupts if no data is sent from the card
    // TODO we might need to set here the pulse counter to 1 because the first pulse could be missed

    /* Start sample timer CODEC_TIMER_SAMPLING (TCD0).
     * Set Counter Channel C (CCC) with relevant bitmask (TC0_CCCIF_bm),
     * the period for clock sampling is specified in SNIFF_ISO15693_READER_EOC_VCD.
     */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;
    /* Sets register INTCTRLB to TC_CCCINTLVL_HI_gc = (0x03<<4) to enable compare/capture for high level interrupts on Channel C (CCC) */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCCINTLVL_HI_gc;

    /* Disable this interrupt as we've already sensed the relevant pulse and will use our internal clock from now on */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;
}

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
    PORTE.DIR = PIN0_bm;
    CodecInitCommon();

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
