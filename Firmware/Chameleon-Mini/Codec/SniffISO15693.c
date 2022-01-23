/*
 * SniffISO15693.c
 *
 *  Created on: 25.01.2017
 *      Author: ceres-c
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

#define SUBCARRIER_1            32
#define SUBCARRIER_2            28
#define SUBCARRIER_OFF          0

// These registers provide quick access but are limited
// so global vars will be necessary
#define DataRegister            Codec8Reg0
// #define StateRegister           Codec8Reg1
#define ModulationPauseCount    Codec8Reg2
#define BitSampleCount          Codec8Reg3 /* Store the amount of received half-bits */
#define SampleRegister          CodecCount16Register1 /* Store a byte of logical bits, 16 half-bits */
#define FloorNoiseLevelDelta    CodecCount16Register2 /* Use register because it will be subtracted from every ADC reading */
#define CodecBufferPtr          CodecPtrRegister1

static volatile struct {
    volatile bool ReaderDemodFinished;
    volatile bool CardDemodFinished;
} Flags = { 0 };

typedef enum {
    DEMOD_SOC_STATE,
    DEMOD_1_OUT_OF_4_STATE,
    DEMOD_1_OUT_OF_256_STATE
} DemodStateType;

static volatile uint16_t DemodFloorNoiseLevel;

static volatile DemodStateType DemodState;
static volatile uint8_t ShiftRegister;
static volatile uint8_t ByteCount;
static volatile uint8_t bDualSubcarrier;
static volatile uint16_t DemodByteCount;
static volatile uint16_t SampleDataCount;

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
    /* Mark reader data as received */
    Flags.ReaderDemodFinished = 1;

    /* And initialize VICC->VCD sniffer */
    CardSniffInit(); // TODO_sniff can this be moved to CodecTask or would it be too slow and we'd loose some bits?
}

/////////////////////////////////////////////////
// VICC->VCD
/////////////////////////////////////////////////

/**
 * INTERRUPT SETUP:
 *  - CODEC_TIMER_TIMESTAMPS:
 *      Counts pulses received from the reader.
 *      Overflow when SOC is finished
 *      Counter channel A to get updated pulses amplitude after 3 pulses.
 *  - CODEC_TIMER_LOADMOD:
 *      Analyze pulse timings.
 *      Overflow when no SOC pulse is received after 1000 us (card did not respond)
 *      Counter channel A called every half-bit during data transmission
 *      Counter channel B called every pulse to validate SOC reception
 *  - Analog Comparator channel 0:
 *      Update pulse amplitude on first pulse
 */

/* Currently implemented only single subcarrier SOC detection
 * TODO extract to single subcarrier specific function and call this or double subcarrier
 */
INLINE void CardSniffInit(void) {
    /**
     * Route events from the analog comparator (which will be configured later) on the event system
     * using channel 2 (channel 0 and 1 are used by the antenna).
     * These events will be counted by CODEC_TIMER_TIMESTAMPS and will (sometimes)
     * reset CODEC_TIMER_LOADMOD.
     */
    EVSYS.CH2MUX = EVSYS_CHMUX_ACA_CH0_gc; /* Route analog comparator channel 0 events on event system channel 2 */

    /**
     * Prepare ADC for antenna field sampling
     * Use conversion channel 1 (0 is already used for reader field RSSI sensing), attached to
     * CPU pin 42 (DEMOD-READER/2.3C).
     * Use conversion channel 2, attached to CPU pin 3 (DEMOD/2.3C).
     *
     * Values from channel 1 (DEMOD-READER/2.3C) will be used to determine pulses levels. This channel is used
     * because the signal shape from DEMOD-READER is easier to analyze with our limited resources and will
     * more likely yield useful values.
     * Values from channel 2 (DEMOD/2.3C) will be used to identify the first pulse threshold: ADC channel 2
     * is sampling on the same channel where analog comparator is comparing values, thus the value from
     * channel 2 is the only useful threshold to identify the first pulse.
     */
    ADCA.PRESCALER = ADC_PRESCALER_DIV4_gc; /* Increase ADC clock speed from default setting in AntennaLevel.h */
    ADCA.CTRLB |= ADC_FREERUN_bm; /* Set ADC as free running */
    ADCA.EVCTRL = ADC_SWEEP_012_gc; /* Enable free running sweep on channel 0, 1 and 2 */
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc; /* Sample PORTA Pin 2 (DEMOD-READER/2.3C) in channel 1 (same pin the analog comparator is comparing to) */
    ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; /* Single-ended input, no gain */
    ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN7_gc; /* Sample PORTA Pin 7 (DEMOD/2.3C) in channel 2 */
    ADCA.CH2.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; /* Single-ended input, no gain */

    /**
     * CODEC_TIMER_TIMESTAMPS (TCD1) will be used to count the peaks identified by ACA while sniffing VICC data
     *
     * PER = 24 pulses, to mark SOC complete reception
     * CCA = 3 pulses, to update the threshold to a more suitable value for upcoming pulses
     * CCA = 5 pulses, to update the threshold again
     */
    CODEC_TIMER_TIMESTAMPS.CTRLA = TC_CLKSEL_EVCH2_gc; /* Using Event channel 2 as an input */
    CODEC_TIMER_TIMESTAMPS.PER = 23; /* 24 - 1 (constantly and reliably missing first pulse): SOC completed */
    CODEC_TIMER_TIMESTAMPS.CCA = 3;
    CODEC_TIMER_TIMESTAMPS.CCB = 5;
    CODEC_TIMER_TIMESTAMPS.INTCTRLA = TC_OVFINTLVL_HI_gc; /* Enable overflow interrupt to handle SOC reception */
    CODEC_TIMER_TIMESTAMPS.INTCTRLB = TC_CCAINTLVL_HI_gc | TC_CCBINTLVL_HI_gc; /* Enable CCA/CCB interrupt */
    CODEC_TIMER_TIMESTAMPS.CTRLFSET = TC_CMD_RESTART_gc; /* Reset counter once it has been configured */

    /* Register CODEC_TIMER_TIMESTAMPS shared interrupt handlers */
    isr_func_CODEC_TIMER_TIMESTAMPS_CCA_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_TIMESTAMPS_CCA_VECT;
    isr_func_CODEC_TIMER_TIMESTAMPS_CCB_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_TIMESTAMPS_CCB_VECT;

    /**
     * CODEC_TIMER_LOADMOD (TCE0) has multiple usages:
     *  - Period overflow handles:
     *      - SOC timeout - until the SOC has been received
     *      - Second half-bit (called 37,76 us after bit start) - while decoding data
     *  - Compare channel A samples the first half-bit (called 18,88 us after bit start) - while decoding data
     *  - Compare channel B filters erroneous VICC->VCD SOC identification - until the SOF has been received
     *
     * It is restarted on every event on event channel 2 until the first 24 pulses of the SOC have been received.
     * This is needed to filter out spurious pulses thanks to CCB. In fact, when CCB is called, it means we are
     * not receiving a pulse train, but just some noise in the field. When receiving a pulse train, this timer
     * will be restarted BEFORE the 64 clock cycles expire, thus CCB won't be called at all as long as we're
     * still receiving pulses. Once 24 pulses have been received, then finally CCB can be disabled (no need to
     * filter anymore) and restart itself can be disabled to scan every data bit.
     *
     * PER = maximum card wait time (1000 us). If no data recived by this moment, restart sampling reader data
     * CCA = half-bit duration (18,88 us)
     * CCB = duration of a single pulse (2,36 us: half hi + half low)
     */
    CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_DIV1_gc; /* Clocked at 27.12 MHz */
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH2_gc; /* Restart this timer on every event on channel 2 (pulse detected by AC) */
    CODEC_TIMER_LOADMOD.PER = 27120; /* 1000 us card response timeout */
    CODEC_TIMER_LOADMOD.CCA = 512 - 10 - 18; /* Half-bit period - 10 clock cycles for shared ISR compensation - 18 to hit right half-bit (checked with scope) */
    CODEC_TIMER_LOADMOD.CCB = 64; /* Single pulse width */
    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_HI_gc; /* Enable overflow (SOF timeout) interrupt */
    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCAINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc; /* Keep interrupt disabled, they will be enabled later on */
    CODEC_TIMER_LOADMOD.CTRLFSET = TC_CMD_RESTART_gc; /* Reset timer */

    /* Register CODEC_TIMER_LOADMOD shared interrupt handlers */
    isr_func_CODEC_TIMER_LOADMOD_CCA_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCA_VECT; /* First half-bit */
    isr_func_CODEC_TIMER_LOADMOD_CCB_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCB_VECT; /* Handle spurious SOC (noise) detection */
    isr_func_CODEC_TIMER_LOADMOD_OVF_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_timeout; /* VICC SOC timeout */

    /**
     * Get current signal amplitude and record it as "silence"
     *
     * This can't be moved closer to ADC configuration since the ADC has to be populated with valid
     * values and it takes 7*4 clock cycles to do so (7 ADC stages * 4 ADC clock prescaler)
     */
    uint32_t accumulator;
    accumulator = ADCA.CH2RES - ANTENNA_LEVEL_OFFSET; /* PORTA Pin 7 (DEMOD/2.3C) - CPU pin 7 */
    for (uint8_t i = 0; i < 127; i++) { /* Add 127 more values */
        accumulator += ADCA.CH2RES - ANTENNA_LEVEL_OFFSET;
    }
    DemodFloorNoiseLevel = (uint16_t)(accumulator >> 7); /* Divide by 2^7=128 */
    /**
     * Typical values with my CR95HF reader:
     * ReaderFloorNoiseLevel: ~900 (measured on ADCA.CH1RES)
     * DemodFloorNoiseLevel: ~1500
    */

    /* Reconfigure ADC to sample only the first 2 channels from now on (no need for CH2 once the noise level has been found) */
    ADCA.EVCTRL = ADC_SWEEP_01_gc;

    /* Write threshold to the DAC channel 0 (connected to Analog Comparator positive input) */
    DACB.CH0DATA = DemodFloorNoiseLevel + (DemodFloorNoiseLevel >> 3); /* Slightly increase DAC output to ease triggering */

    /**
     * Finally, now that we have the DAC set up, configure analog comparator A (the only one in this MCU)
     * channel 0 (the only one that can be connecte to the DAC) to recognize carrier pulses modulated by
     * the VICC against the correct threshold coming from the DAC.
     */
    /* Register ACA channel 0 interrupt handler */
    isr_func_ACA_AC0_vect = &isr_SNIFF_ISO15693_ACA_AC0_VECT;

    ACA.AC0MUXCTRL = AC_MUXPOS_DAC_gc | AC_MUXNEG_PIN7_gc; /* Tigger when DAC signal is above PORTA Pin 7 (DEMOD/2.3C) */
    /* enable AC | high speed mode | large hysteresis | sample on rising edge | high level interrupts */
    /* Hysteresis is not actually needed, but appeared to be working and sounds like it might be more robust */
    ACA.AC0CTRL = AC_ENABLE_bm | AC_HSMODE_bm | AC_HYSMODE_LARGE_gc | AC_INTMODE_RISING_gc | AC_INTLVL_HI_gc;

    /* This function ends ~116 us after last VCD pulse, still in the 300 us period given by ISO15693 section 8.5 */
}

INLINE void CardSniffDeinit(void) {
    /* Restore ADC clock */
    ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc; /* Restore same clock as in AntennaLevel.h */
    ADCA.CTRLB &= ~ADC_FREERUN_bm; /* Stop free running ADC */
    ADCA.EVCTRL &= ~ADC_SWEEP_01_gc; /* Stop channel sweep */
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc; /* Sample PORTA Pin 2 (DEMOD-READER/2.3C) in channel 1 */
    ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; /* Single-ended input, no gain */

    // TODO disable event system

    // TODO disable all other interrupts

    /* Reset ACA AC0 to default setting */
    ACA.AC0MUXCTRL = AC_MUXPOS_DAC_gc | AC_MUXNEG_PIN7_gc; /* This actually was unchanged */
    ACA.AC0CTRL = CODEC_AC_DEMOD_SETTINGS;
}


/**
 * This interrupt is called on every rising edge sensed by the analog comparator
 * and it enables the spurious pulse filtering interrupt (CODEC_TIMER_LOADMOD CCB).
 * If we did not receive a SOC but, indeed, noise, this interrupt will be enabled again.
 */
ISR_SHARED isr_SNIFF_ISO15693_ACA_AC0_VECT(void) {
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCBINTLVL_HI_gc; /* Enable level 0 CCB interrupt to filter spurious pulses */

    ACA.AC0CTRL = AC_ENABLE_bm | AC_HSMODE_bm | AC_HYSMODE_LARGE_gc | AC_INTMODE_RISING_gc | AC_INTLVL_OFF_gc; /* Disable this interrupt */

    DACB.CH0DATA = (DemodFloorNoiseLevel << 1) - (DemodFloorNoiseLevel >> 2); /* Blindly increase threshold after 1 pulse */
    /* Note: by the time the DAC has changed its output, we're already after the 2nd pulse */
}

/**
 * This interrupt is called after 3 subcarrier pulses and increases the threshold
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_TIMESTAMPS_CCA_VECT(void) {
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    uint16_t temp_read = ADCA.CH1RES - ANTENNA_LEVEL_OFFSET;
    if (temp_read > 0xfff) { /* Naive overflow check */
        DACB.CH0DATA = 0xfff;
    } else {
        DACB.CH0DATA = temp_read; /* Further increase DAC output after 3 pulses */
    }

    CODEC_TIMER_TIMESTAMPS.INTCTRLB &= TC_CCAINTLVL_OFF_gc; /* Disable this interrupt */
}

/**
 * This interrupt is called after 5 subcarrier pulses and increases the threshold even more
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_TIMESTAMPS_CCB_VECT(void) {
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    uint16_t temp_read = ADCA.CH1RES - ANTENNA_LEVEL_OFFSET;
    if (temp_read + (temp_read >> 2) + (temp_read >> 3) > 0xfff) { /* Naive overflow check */
        DACB.CH0DATA = 0xfff;
    } else {
        DACB.CH0DATA = temp_read + (temp_read >> 2) + (temp_read >> 3); /* Further increase DAC output after 5 pulses */
    }

    CODEC_TIMER_TIMESTAMPS.INTCTRLB &= TC_CCBINTLVL_OFF_gc; /* Disable this interrupt */
}

/**
 * This interrupt is called after the first continuous 24 pulses (SOC).
 * The upcoming pause and 8 pulses (logic 1) in SOC will be handled as a normal logic 1 bit.
 *
 * This interrupt is called right at the end of the 24 pulses of the sof, which has the following structure (0 is no pulse, 1 is a pulse)
 *
 * 0000000000000000000000001111111111111111111111110000000011111111xxxxxxxxyyyyyyyyzzzzzzzz...
 *                                                 ^
 *                                                 This interrupt is called here
 *
 * The following interupt that will be called will be CODEC_TIMER_LOADMOD.CCA_vect after 512 clock cycles (18,88 us)
 * and it has to check we've received no pulses (unmodulated 18,88 us period in SOF), then CODEC_TIMER_LOADMOD.PER_vect
 * will be called and it has to check we've received 8 pulses in the period (8 pulses modulated 18,88 us period in SOF).
 * If both these conditions stand, we have correctly received a full SOF and can finally start to decode data
 * (represented with x, y and z in the above sequence).
 */
ISR(CODEC_TIMER_TIMESTAMPS_OVF_VECT) {
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    /* Enable CCA (first half-bit decoding), disable CCB (spurious pulse timeout) */
    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCAINTLVL_HI_gc | TC_CCBINTLVL_OFF_gc;
    /* Disable timer resetting on every AC0 event */
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_OFF_gc;
    /* Change CODEC_TIMER_LOADMOD period to one full logic bit length */
    CODEC_TIMER_LOADMOD.PER = 1024 + 26; /* First period is slightly longer to accomodate for this interrupt, which is triggered slightly earlier than it should have  */
    CODEC_TIMER_LOADMOD.PERBUF = 1024; /* Store actual period in buffered register (it will be used once this period is over) */

    /* Update overflow handler to bit decode function */
    isr_func_CODEC_TIMER_LOADMOD_OVF_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_decode;

    CODEC_TIMER_TIMESTAMPS.INTCTRLA = TC_OVFINTLVL_OFF_gc; /* Disable this interrupt */
}

/**
 * This interrupt is called on the first half-bit
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCA_VECT(void) {
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    PORTE.OUT = (CODEC_TIMER_TIMESTAMPS.CNT > 2); /* Theoretically we should shift this, but since we're writing PIN0, no need to shift by 0 (<< PIN0_bp) */
    /* Using 2 as a discriminating factor to allow for slight errors in pulses counting. */
    CODEC_TIMER_TIMESTAMPS.CNT = 0; /* Clear count register for next half-bit */
}

/**
 * This interrupt is called every 64 clock cycles (= once per pulse, when in single subcarrier mode),
 * unless the timer is reset. This means it will be invoked only if, after a pulse, we don't receive another one.
 * Classical scenario: spurious pulse detected as SOC start.
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCB_VECT(void) {
    CODEC_TIMER_LOADMOD.INTCTRLB &= TC_CCBINTLVL_OFF_gc; /* Disable this interrupt */

    // DemodFloorNoiseLevel += CODEC_THRESHOLD_CALIBRATE_STEPS; /* Slightly increase DAC output value */ // TODO check if this actually helps or is a trouble with low signals

    DACB.CH0DATA = DemodFloorNoiseLevel + (DemodFloorNoiseLevel >> 3); /* Restore DAC output (AC negative comparation threshold) to pre-AC0 interrupt update value */

    ACA.AC0CTRL |= AC_INTLVL_HI_gc; /* Re-enable analog comparator interrupt to search for another pulse */

    CODEC_TIMER_TIMESTAMPS.CTRLFSET = TC_CMD_RESTART_gc; /* Clear pulses counter (we received garbage) */
    CODEC_TIMER_TIMESTAMPS.INTCTRLB = TC_CCAINTLVL_HI_gc | TC_CCBINTLVL_HI_gc; /* Re enable CCA/CCB interrupt in case they were triggered and are now disabled */
}

/**
 * This interrupt handles the VICC SOF timeout when the card does not answer
 * and restarts reader sniffing
 * 
 * It will be replaced by isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_decode once
 * a SOF is received
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_timeout(void) {
    // TODO restart reader sniffing
}

/**
 * This interrupt is called on the second bit-half to decode it.
 * It replaces isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_timeout once the SOF is received
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_decode(void) {
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    PORTE.OUT = (CODEC_TIMER_TIMESTAMPS.CNT > 2); /* Theoretically we should shift this, but since we're writing PIN0, no need to shift by 0 (<< PIN0_bp) */
    /* Using 2 as a discriminating factor to allow for slight errors in pulses counting. */
    CODEC_TIMER_TIMESTAMPS.CNT = 0; /* Clear count register for next half-bit */
}

















/* This functions resets all global variables used in the codec and enables interrupts to wait for reader data */
void StartSniffISO15693Demod(void) {
    /* Reset global variables to default values */
    CodecBufferPtr = CodecBuffer;
    Flags.ReaderDemodFinished = 0;
    Flags.CardDemodFinished = 0;
    DemodState = DEMOD_SOC_STATE;
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

    /* Change DAC reference source to Internal 1V (same as ADC source) */
    DACB.CTRLC = DAC_REFSEL_INT1V_gc;

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

    /* Restore default DAC reference to Codec.h setting */
    DACB.CTRLC = DAC_REFSEL_AVCC_gc;
}

void SniffISO15693CodecTask(void) {
    if (Flags.ReaderDemodFinished) {

        Flags.ReaderDemodFinished = 0;

        DemodByteCount = ByteCount;
        bDualSubcarrier = 0;

        if (DemodByteCount > 0) {
            LogEntry(LOG_INFO_CODEC_SNI_READER_DATA, CodecBuffer, DemodByteCount);
            LEDHook(LED_CODEC_RX, LED_PULSE);

            if (CodecBuffer[0] & ISO15693_REQ_SUBCARRIER_DUAL) {
                bDualSubcarrier = 1;
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
