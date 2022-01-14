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

/**
 * INTERRUPT SETUP:
 *  - CODEC_TIMER_TIMESTAMPS:
 *      Counts pulses received from the reader in a given time span.
 *      TODO counter channel B to get field amplitude every 4 pulses
 *  - CODEC_TIMER_LOADMOD:
 *      Analyze single pulses (max duration)
 *      TODO timeout on missing pulse end, restart sampling
 *  - TODO ADC sample field on first pulse via event system
 *  - TODO ADC sample field every n-th pulse via event system
 *  - TODO another timer every 18,88 us to get bits
 */

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

    /**
     * Route events from the analog comparator on the event system using channel 2
     * (channel 0 and 1 are used by the antenna).
     * These events will be counted by CODEC_TIMER_TIMESTAMPS and will (sometimes)
     * reset CODEC_TIMER_LOADMOD.
     */
    EVSYS.CH2MUX = EVSYS_CHMUX_ACA_CH0_gc; /* Route analog comparator channel 0 events on event system channel 2 mux*/
    EVSYS.CH2CTRL = EVSYS_DIGFILT_1SAMPLE_gc; /* Ignore first event */ // TODO why are we ignoring the first event?

    /**
     * Prepare ADC for antenna field sampling
     * Use conversion channel (0 is already used for reader field RSSI sensing), attached to
     * CPU pin 42 (DEMOD-READER/2.3C).
     *
     * Values from DEMOD-READER/2.3C will be used to identify a suitable threshold.
     *
     * The ADC will also be used to detect when the peak amplitude is decreasing via the compare register:
     * the sensed signal amplitude will be used to mantain a valid threshold when signal
     */
    ADCA.PRESCALER = ADC_PRESCALER_DIV4_gc; /* Increase clock speed from settings in AntennaLevel.h */
    ADCA.CTRLB = ADC_FREERUN_bm; /* Set ADC as free running */
    ADCA.EVCTRL = ADC_SWEEP_01_gc; /* Enable free running sweep on channel 0 and 1 */
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc; /* Sample PORTA Pin 2 (DEMOD-READER/2.3C) in channel 1 */
    ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; /* Single-ended input, no gain */
    /* Later on we'll configure ADCA.CMP register with the threshold value */
    /* Later on we'll enable interrupts on channel 1 when amplitude goes
       below threshold ADCA.CH1.INTCTRL = ADC_CH_INTMODE_BELOW_gc | ADC_CH_INTLVL_HI_gc; */
    // TODO trigger ADC sampling in AC interrupt (can't use events here, there would be too many)
    // TODO trigger ADC sampling with some kind counter (every x pulses?)
    // TODO store threshold value*1,25 in ADC compare register and trigger new sampling when going below that 

    /**
     * CODEC_TIMER_TIMESTAMPS (TCD1) will be used to count the peaks identified by ACA while sniffing VICC data
     * TODO can this be cleared automatically with an event generated by CODEC_TIMER_LOADMOD CCA?
     */
    CODEC_TIMER_TIMESTAMPS.CTRLA = TC_CLKSEL_EVCH2_gc; /* Using Event channel 2 as an input */
    CODEC_TIMER_TIMESTAMPS.CNT = 0; /* Clear counter */
    CODEC_TIMER_TIMESTAMPS.PER = 0xFFFF; /* Don't limit this value, will be cleared manually */
    CODEC_TIMER_TIMESTAMPS.INTCTRLA = RTC_OVFINTLVL_OFF_gc; /* No error/overflow interrupt */

    /**
     * CODEC_TIMER_LOADMOD (TCE0) has multiple usages: its period overflow handles SOF timeout,
     * compare channel A generates a half-bit every 18,88 us and compare channel B filters
     * erroneous VICC->VCD SOF identification.
     *
     * PER = maximum card wait time (1000 us), if no data recived by this moment, restart sampling reader data
     * CCA = half-bit duration (18,88 us)
     * CCB = duration of a single clock cycle (64 us)
     */
    CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_DIV1_gc; /* Clocked at 27.12 MHz */
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH2_gc; /* Restart this timer on every event on channel 2 (pulse detected by AC) */
    CODEC_TIMER_LOADMOD.CNT = 0; /* Clear counter */
    CODEC_TIMER_LOADMOD.PER = 27120; /* Max wait time after reader modulation end */
    CODEC_TIMER_LOADMOD.CCA = 512; /* Half-bit period (To measure half-bit, we have to disable restart on Channel 2 events, will do it later) */
    CODEC_TIMER_LOADMOD.CCB = 64; /* Single pulse width */
    CODEC_TIMER_LOADMOD.INTCTRLA = RTC_OVFINTLVL_HI_gc; /* Enable overflow interrupt to handle SOC timeout */
    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCAINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc; /* Keep CCA/CCB interrupts disabled (will be enabled on demand) */

    /* Register CODEC_TIMER_LOADMOD shared interrupt handlers */
    isr_func_CODEC_TIMER_LOADMOD_OVF_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT; /* Restart VCD sniff on VICC SOC timeout */
    isr_func_CODEC_TIMER_LOADMOD_CCA_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCA_VECT; /* Decode data every half-bit period */
    isr_func_CODEC_TIMER_LOADMOD_CCB_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCB_VECT; /* Handle spurious SOF (noise) detection */
}

INLINE void CardSniffDeinit(void) {
    // Reset ACA AC0 to default setting
    ACA.AC0MUXCTRL = AC_MUXPOS_DAC_gc | AC_MUXNEG_PIN7_gc;
    ACA.AC0CTRL = CODEC_AC_DEMOD_SETTINGS;

    /* Restore ADC clock */
    ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc; /* Restore same clock as in AntennaLevel.h */

    // TODO disable evenst system

    // TODO disable all other interrupts
}

/**
 * This interrupt is called on every rising edge sensed by the analog comparator
 * and it enables the spurious pulse filtering interrupt (CODEC_TIMER_LOADMOD CCB).
 * If we did not receive a SOF but, indeed, noise, this interrupt will be enabled again.
 */
ISR_SHARED isr_SNIFF_ISO15693_ACA_AC0_VECT(void) { // this interrupt either finds the SOC or gets triggered before
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCBINTLVL_HI_gc; /* Enable level 0 CCB interrupt to filter spurious pulses */
    // TODO TEMP CODE BELOW, PLEASE REMOVE, JUST TO TEST 18,88 us PULSES
    // CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc; /* Don't restart this counter automatically on every pulse anymore */
    // CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCAINTLVL_HI_gc; /* Enable level 0 CCB interrupt to filter spurious pulses */
    // END TEMP CODE

    ACA.AC0CTRL &= ~AC_INTLVL_HI_gc; /* Disable this interrupt */

    // TODO to use AC as pulse counter:
    //  1) change event trigger to falling edge (currently using raising edge)
    //  2) route this to source a counter (or set it to 1 if already routed). Better to have it already routed so we can count this pulse as well.

    // TODO stop TCD0 or disable overflow interrupt (we have found a SOF)
}

/**
 * This interrupt handles VICC->VCD SOF timeout and restarts VCD->VICC sniffing
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT(void) { // Registered to CODEC_TIMER_LOADMOD CCB
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    // TODO Do something here
}

/**
 * This interrupt is called every half-bit period (18,88 us) to sample received data
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCA_VECT(void) {
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    // TODO DON'T FORGET TO DISABLE TIMER_LOADMOD RESETTING ON EVERY PULSE AFTER SOC

}

/**
 * This interrupt is called every (f_c/32)/8 (once per pulse when in single subcarrier mode),
 * unless the timer is reset. This means it will be invoked only if, after a pulse, we don't receive any more.
 * Classical scenario: spurious pulse detected as SOF start.
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCB_VECT(void) {
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    ACA.AC0CTRL |= AC_INTLVL_HI_gc; /* Re-enable analog comparator interrupt to search for another pulse */

    CODEC_TIMER_TIMESTAMPS.CNT = 0; /* Clear pulses counter */

    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_OFF_gc; /* Disable this interrupt */

    char tmpBuf[10]; // TODO remove this debug code
    snprintf(tmpBuf, 10, "ADC: %d\n", ADCA.CH1RES);
    TerminalSendString(tmpBuf);

    // TODO decomment below ADC flush once it is fixed (currently triggering BADISTR)
    // ADCA.CTRLA = ADC_FLUSH_bm; /* Flush ADC pipeline (this is not a valid threshold) */
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
