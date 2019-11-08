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
#define ISO15693_SAMPLE_CLK     TC_CLKSEL_DIV2_gc // 13.56MHz
#define ISO15693_SAMPLE_PERIOD  128 // 9.4us

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

/* This function implements CODEC_DEMOD_IN_INT0_VECT interrupt vector.
 * It is called when a pulse is detected in CODEC_DEMOD_IN_PORT (PORTB).
 * The relevatn interrupt vector is registered to CODEC_DEMOD_IN_MASK0 (PIN1) via:
 * CODEC_DEMOD_IN_PORT.INT0MASK = CODEC_DEMOD_IN_MASK0;
 * and unregistered writing the INT0MASK to 0
 */
// ISR(CODEC_DEMOD_IN_INT0_VECT)
void isr_SNIFF_ISO15693_CODEC_DEMOD_IN_INT0_VECT(void) {
    LEDHook(LED_CODEC_RX, LED_PULSE);

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

/* This function is called from isr_ISO15693_CODEC_TIMER_SAMPLING_CCC_VECT
 * when we have 8 bits in SampleRegister and they represent an end of frame.
 */
INLINE void SNIFF_ISO15693_EOC_VCD(void) {
    /* Set bitrate required by the reader on SOF for our following response */
    BitRate1 = 256 * 4; // 256 * 4 - 1
    if (CodecBuffer[0] & ISO15693_REQ_DATARATE_HIGH)
        BitRate1 = 256;

    if (CodecBuffer[0] & ISO15693_REQ_SUBCARRIER_DUAL) {
        BitRate2 = 252 * 4; // 252 * 4 - 3
        if (CodecBuffer[0] & ISO15693_REQ_DATARATE_HIGH)
            BitRate2 = 252;
    } else {
        BitRate2 = BitRate1;
    }

    Flags.ReaderDemodFinished = 1;
    /* Sets timer off for CODEC_TIMER_SAMPLING (TCD0) disabling clock source */
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
    /* Sets register INTCTRLB to 0 to disable all compare/capture interrupts */
    CODEC_TIMER_SAMPLING.INTCTRLB = 0;
}

/* This function is registered to CODEC_TIMER_SAMPLING (TCD0)'s Counter Channel C (CCC).
 * When the timer is enabled, this is called on counter's overflow
 * 
 * It demodulates bits received from the card and saves them in CodecBuffer.
 * 
 * It disables its own interrupt when receives an EOF (calling ISO15693_EOC) or when it receives garbage
 */
void isr_SNIFF_ISO15693_CARD_CODEC_TIMER_SAMPLING_CCC_VECT(void) {
    /* Shift demod data */
    SampleRegister = (SampleRegister << 1) | (!(CODEC_DEMOD_IN_PORT.IN & CODEC_DEMOD_IN_MASK) ? 0x01 : 0x00);

    if (++BitSampleCount == CardSOCBitsCount) { /* CardSOCBitsCount can be 8 if using one subcarrier or 9 if using two */
        BitSampleCount = 0;
        switch (DemodState) {
            case DEMOD_SOC_STATE:
                CardSOCBytesCounter += 1;
                if (SampleRegister == SOC_ONE_SUBCARRIER) {
                    if (CardSOCBytesCounter == 3) { /* We have checked 3*8=24 pulses */
                        CardSOCBytesCounter = 0;
                        DemodState = DEMOD_1_OUT_OF_4_STATE;
                    }
                    SampleDataCount = 0;
                    ModulationPauseCount = 0;
                } else if (SampleRegister == SOC_TWO_SUBCARRIER) { /* We have checked 3*9=27 pulses */
                    if (CardSOCBytesCounter == 3) {
                        CardSOCBytesCounter = 0;
                        DemodState = DEMOD_1_OUT_OF_256_STATE;
                    }
                    SampleDataCount = 0;
                } else { // No SOC. Restart and try again, we probably received garbage.
                    CardSOCBytesCounter = 0;
                    Flags.ReaderDemodFinished = 1;
                    /* Sets timer off for CODEC_TIMER_SAMPLING (TCD0) disabling clock source */
                    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
                    /* Sets register INTCTRLB to 0 to disable all compare/capture interrupts */
                    CODEC_TIMER_SAMPLING.INTCTRLB = 0;
                }
                break;

            case DEMOD_1_OUT_OF_4_STATE:
                if (SampleRegister == EOC_CODE) {
                    SNIFF_ISO15693_EOC_VCD();
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
                    SNIFF_ISO15693_EOC_VCD();
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
    }
    SampleDataCount++;
}


/* This function is registered to CODEC_TIMER_SAMPLING (TCD0)'s Counter Channel C (CCC).
 * When the timer is enabled, this is called on counter's overflow
 * 
 * It demodulates bits received from the reader and saves them in CodecBuffer.
 * 
 * It disables its own interrupt when receives an EOF (calling ISO15693_EOC) or when it receives garbage
 */
// ISR(CODEC_TIMER_SAMPLING_CCC_VECT) // Reading data sent from the reader
void SNIFF_ISO15693_READER_CODEC_TIMER_SAMPLING_CCC_VECT(void) {
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
                    /* Sets timer off for CODEC_TIMER_SAMPLING (TCD0) disabling clock source */
                    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
                    /* Sets register INTCTRLB to 0 to disable all compare/capture interrupts */
                    CODEC_TIMER_SAMPLING.INTCTRLB = 0;
                }
                break;

            case DEMOD_1_OUT_OF_4_STATE:
                if (SampleRegister == EOC_CODE) {
                    SNIFF_ISO15693_EOC_VCD();
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
                    SNIFF_ISO15693_EOC_VCD();
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
    /* Set the period for CODEC_TIMER_SAMPLING (TCD0) to ISO15693_SAMPLE_PERIOD - 1 because PER is 0-based */
    CODEC_TIMER_SAMPLING.PER = ISO15693_SAMPLE_PERIOD - 1;
    /* Set Counter Channel C (CCC) register with half bit period - 1. (- 14 to compensate ISR timing overhead) */
    CODEC_TIMER_SAMPLING.CCC = ISO15693_SAMPLE_PERIOD / 2 - 14 - 1;
    /* Set timer for CODEC_TIMER_SAMPLING (TCD0) to ISO15693_SAMPLE_CLK = TC_CLKSEL_DIV2_gc = System Clock / 2
     *
     * TODO Why system clock / 2 and not iso period?
     */
    CODEC_TIMER_SAMPLING.CTRLA = ISO15693_SAMPLE_CLK;
    /* Set event action for CODEC_TIMER_SAMPLING (TCD0) to restart and trigger CODEC_TIMER_MODSTART_EVSEL = TC_EVSEL_CH0_gc = Event Channel 0 */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;
    /* Set Counter Channel C (CCC) with relevant bitmask (TC0_CCCIF_bm), the period for clock sampling is specified above */
    // CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;  // TODO Why writing to a FLAG register? We need only to READ this...
    /* Sets register INTCTRLB to TC_CCCINTLVL_OFF_gc = (0x00<<4) to disable compare/capture C interrupts
     *
     * TODO Why turn it off?
     */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCCINTLVL_OFF_gc;

    /* Start looking out for modulation pause via interrupt. */
    /* Sets register INTFLAGS to PORT_INT0LVL_HI_gc = (0x03<<0) to enable compare/capture for high level interrupts on CODEC_DEMOD_IN_PORT (PORTB) */
    CODEC_DEMOD_IN_PORT.INTFLAGS = PORT_INT0LVL_HI_gc;
    /* Sets INT0MASK to CODEC_DEMOD_IN_MASK0 = PIN1_bm to use it as source for port interrupt 0 */
    CODEC_DEMOD_IN_PORT.INT0MASK = CODEC_DEMOD_IN_MASK0;
}

void SniffISO15693CodecInit(void) {
    CodecInitCommon();

    /* Register isr_ISO15693_CODEC_TIMER_SAMPLING_CCC_VECT function
     * to CODEC_TIMER_SAMPLING (TCD0)'s Counter Channel C (CCC)
     */
    isr_func_TCD0_CCC_vect = &SNIFF_ISO15693_READER_CODEC_TIMER_SAMPLING_CCC_VECT;
    /* Register isr_ISO15693_CODEC_DEMOD_IN_INT0_VECT function
     * to CODEC_DEMOD_IN_PORT (PORTB) interrupt 0
     */
    isr_func_CODEC_DEMOD_IN_INT0_VECT = &isr_SNIFF_ISO15693_CODEC_DEMOD_IN_INT0_VECT;

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

void StartSniffISO15693Card_Dual_Demod(void) {

    CODEC_18uS_SLOT_TIMER.CNT = 0;
    CODEC_18uS_SLOT_TIMER.CTRLA = TC_CLKSEL_DIV4_gc;
    CODEC_18uS_SLOT_TIMER.CTRLB = 0; // Normal mode
    CODEC_18uS_SLOT_TIMER.CTRLC = 0; // No waveform generator in use
    CODEC_18uS_SLOT_TIMER.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH7_gc ; // On CH7 (8 or 9 pulses by TCE0) restart the counter 
    CODEC_18uS_SLOT_TIMER.CTRLE = 0; // Normal mode - simple counter
    CODEC_18uS_SLOT_TIMER.CCC = 148; //18.5 us


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

        StartSniffISO15693Demod();
    }
}
