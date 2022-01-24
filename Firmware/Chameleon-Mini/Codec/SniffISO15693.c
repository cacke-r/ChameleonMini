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
#include "Terminal/Terminal.h" // TODO_sniff remove


#define VCD_SOC_1_OF_4_CODE         0x7B
#define VCD_SOC_1_OF_256_CODE       0x7E
#define VCD_EOC_CODE                0xDF
#define VICC_SOC_CODE               0b00011101 /* = 0x1D 3 unmodulated periods, 3 modulated periods, 1 unmodulated, 1 modulated */
#define VICC_EOC_CODE               0b10111000 /* = 0xB8 1 modulated period, 1 unmodulated period, 3 modulated, 3 unmodulated */
#define ISO15693_READER_SAMPLE_CLK      TC_CLKSEL_DIV2_gc // 13.56MHz
#define ISO15693_READER_SAMPLE_PERIOD   128 // 9.4us
#define ISO15693_CARD_SAMPLE_CLK        TC_CLKSEL_DIV4_gc /* Max possible sampling resolution */
#define ISO15693_CARD_SAMPLE_PERIOD     148 // 18.5us

// These registers provide quick access but are limited
// so global vars will be necessary
#define DataRegister            Codec8Reg0
#define StateRegister           Codec8Reg1
#define ModulationPauseCount    Codec8Reg2
#define BitSampleCount          Codec8Reg3 /* Store the amount of received half-bits */
#define SampleRegister          CodecCount16Register1
#define SampleRegisterH         GPIOR4 /* CodecCount16Register1 is the composition of GPIOR4 and GPIOR5, divide them for easier read access */
#define SampleRegisterL         GPIOR5
// #define FloorNoiseLevelDelta    CodecCount16Register2
#define CodecBufferPtr          CodecPtrRegister1

static volatile struct {
    volatile bool ReaderDemodFinished;
    volatile bool CardDemodFinished;
} Flags = { 0 };

typedef enum {
    DEMOD_VCD_SOC_STATE,
    DEMOD_VCD_1_OUT_OF_4_STATE,
    DEMOD_VCD_1_OUT_OF_256_STATE,
    DEMOD_VICC_SOC_STATE,
    DEMOD_VICC_DATA,
} DemodSniffStateType;

static volatile uint16_t DemodFloorNoiseLevel;

static volatile DemodSniffStateType DemodState;
static volatile uint8_t ShiftRegister;
static volatile uint8_t ByteCount;
static volatile uint8_t ReaderByteCount;
static volatile uint8_t CardByteCount;
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
            case DEMOD_VCD_SOC_STATE:
                if (SampleRegister == VCD_SOC_1_OF_4_CODE) {
                    DemodState = DEMOD_VCD_1_OUT_OF_4_STATE;
                    SampleDataCount = 0;
                    ModulationPauseCount = 0;
                } else if (SampleRegister == VCD_SOC_1_OF_256_CODE) {
                    DemodState = DEMOD_VCD_1_OUT_OF_256_STATE;
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

            case DEMOD_VCD_1_OUT_OF_4_STATE:
                if (SampleRegister == VCD_EOC_CODE) {
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

            case DEMOD_VCD_1_OUT_OF_256_STATE:
                if (SampleRegister == VCD_EOC_CODE) {
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
    ReaderByteCount = ByteCount; /* Copy to direction-specific variable */

    /* Sets timer off for TCD0, disabling clock source. We're done receiving data from reader and don't need to probe the antenna anymore - From 14.12.1 [8331F–AVR–04/2013] */
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
    /* Disable event action for CODEC_TIMER_SAMPLING (TCD0) - From 14.12.4 [8331F–AVR–04/2013] */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc;
    /* Disable compare/capture interrupts on Channel C - From 14.12.7 [8331F–AVR–04/2013] */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCCINTLVL_OFF_gc;
    /* Clear Compare Channel C (CCC) interrupt Flags - From 14.12.10 [8331F–AVR–04/2013] */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCCIF_bm;

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
     * CCA = 3 pulses, to update the threshold to a more suitable value for upcoming pulses
     */
    // TODO stop using this timer as a counter (not needed): move thresh update to CODEC_TIMER_LOADMOD and use this as a timeout
    // TODO remove shared interrupt CCA/CCB
    CODEC_TIMER_TIMESTAMPS.CTRLA = TC_CLKSEL_EVCH2_gc; /* Using Event channel 2 as an input */
    CODEC_TIMER_TIMESTAMPS.CCA = 3;
    CODEC_TIMER_TIMESTAMPS.INTCTRLB = TC_CCAINTLVL_HI_gc; /* Enable CCA/CCB interrupt */
    CODEC_TIMER_TIMESTAMPS.CTRLFSET = TC_CMD_RESTART_gc; /* Reset counter once it has been configured */

    /* Register CODEC_TIMER_TIMESTAMPS shared interrupt handlers */
    isr_func_CODEC_TIMER_TIMESTAMPS_CCA_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_TIMESTAMPS_CCA_VECT;

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

    /* Reinit state variables */
    CodecBufferPtr = CodecBuffer;
    ByteCount = 0; // TODO use register?

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
    // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCBINTLVL_HI_gc; /* Enable level 0 CCB interrupt to filter spurious pulses and find SOC */

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

    DACB.CH0DATA = ADCA.CH1RES - ANTENNA_LEVEL_OFFSET; /* Further increase DAC output after 3 pulses with value from PORTA Pin 2 (DEMOD-READER/2.3) */

    CODEC_TIMER_TIMESTAMPS.INTCTRLB = TC_CCAINTLVL_OFF_gc; /* Disable this interrupt */
}

/**
 * This interrupt is called on the first half-bit
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCA_VECT(void) {
    /**
     * This interrupt is called on every odd half-bit, thus we don't need to do any check,
     * just appen to the sample register and increment number of received bits.
     */
    SampleRegister = (SampleRegister << 1) | (CODEC_TIMER_TIMESTAMPS.CNT > 4); /* Using 3 as a discriminating factor to allow for slight errors in pulses counting. */
    /* Don't increase BitSampleCount, since this is only the first half of a bit */

    CODEC_TIMER_TIMESTAMPS.CNT = 0; /* Clear count register for next half-bit */
}

/**
 * This interrupt is called every 64 clock cycles (= once per pulse) unless the timer is reset.
 * This means it will be invoked only if, after a pulse, we don't receive another one.
 *
 * This will either find a noise or the end of the SOC. If we've received noise, most likely,
 * we received few pulses. The soc is made of 24 pulses, so if VICC modulation actually started,
 * we should have a relevant number of pulses
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_CCB_VECT(void) {
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

    if (CODEC_TIMER_TIMESTAMPS.CNT < 15) {
        /* We most likely received garbage */

        CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCBINTLVL_OFF_gc; /* Disable all compare interrupts, including this one */

        // DemodFloorNoiseLevel += CODEC_THRESHOLD_CALIBRATE_STEPS; /* Slightly increase DAC output value */ // TODO check if this actually helps or is a trouble with low signals

        DACB.CH0DATA = DemodFloorNoiseLevel + (DemodFloorNoiseLevel >> 3); /* Restore DAC output (AC negative comparation threshold) to pre-AC0 interrupt update value */
        ACA.AC0CTRL |= AC_INTLVL_HI_gc; /* Re-enable analog comparator interrupt to search for another pulse */

        CODEC_TIMER_TIMESTAMPS.INTCTRLB = TC_CCAINTLVL_HI_gc; /* Re enable CCA interrupt in case it was triggered and then is now disabled */
    } else {
        /* Got actual data, prepare for sniffing */

        // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

        /* Enable CCA (first half-bit decoding), disable CCB (spurious pulse timeout) */
        CODEC_TIMER_LOADMOD.INTCTRLB = TC_CCAINTLVL_HI_gc;
        /* Disable timer resetting on every AC0 event */
        CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_OFF_gc;
        /* Change CODEC_TIMER_LOADMOD to compensate for delay */
        CODEC_TIMER_LOADMOD.CCA = 512 - 8; /* First CCA is slightly shorter to accommodate for this interrupt, which is triggered 64 clock cycles after SOC ends */
        CODEC_TIMER_LOADMOD.CCABUF = 512; /* After first CCA, use actual CCA period (will be written when UPDATE condition is met, thus after first period hit) */
        /* Change CODEC_TIMER_LOADMOD period to one full logic bit length */
        CODEC_TIMER_LOADMOD.PER = 1024 - 8; /* One full logic bit duration */
        CODEC_TIMER_LOADMOD.PERBUF = 1024 - 1; /* One full logic bit duration - 1 (accommodate for slight clock drift) */
        /* Change overflow handler to bit decode function */
        isr_func_CODEC_TIMER_LOADMOD_OVF_VECT = &isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_decode;

        /**
         * Prepare registers for actual data demodulation
         * This is a bit like cheating. Since we assume this is a real SOC, we can pretend we've already received
         * the first 3 unmodulated and 3 modulated periods (binary: 0b000111). The two remaining periods (0b01) will be
         * shifted in while data decoding.
         */
        SampleRegister = 0x07; /* = 0b000111 */
        BitSampleCount = 4 + 3; /* Pretend we've already received 8 empty half-bits and the above 6 half-bits */
        StateRegister = DEMOD_VICC_SOC_STATE;
        // PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    }

    CODEC_TIMER_TIMESTAMPS.CTRLFSET = TC_CMD_RESTART_gc; /* Clear pulses counter nonetheless */
}

/**
 * This interrupt handles the VICC SOF timeout when the card does not answer
 * and restarts reader sniffing
 * 
 * It will be replaced by isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_decode once
 * a SOF is received
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_timeout(void) {
    // TODO DO NOT USE THIS INTERRUPT
    // This is being reset every pulse, thus continuous noise (threshold too low) would make it restart forever, losing all future comms
    // Need to setup yet another timer to count to 27120
}

/**
 * This interrupt is called on the second bit-half to decode it.
 * It replaces isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_timeout once the SOF is received
 */
ISR_SHARED isr_SNIFF_ISO15693_CODEC_TIMER_LOADMOD_OVF_VECT_decode(void) {
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code


    /**
     * This interrupt is called on every even half-bit, we then need to check the content of the register
     */
    SampleRegister = (SampleRegister << 1) | (CODEC_TIMER_TIMESTAMPS.CNT > 4); /* Using 3 as a discriminating factor to allow for slight errors in pulses counting. */
    BitSampleCount++;
    CODEC_TIMER_TIMESTAMPS.CNT = 0; /* Clear count register for next half-bit */

    // char tmpBuf[10];
    // snprintf(tmpBuf, 10, "BSC: %d\n", BitSampleCount);
    // TerminalSendString(tmpBuf);

    if (BitSampleCount == 8) { /* We have 16 half-bits in SampleRegister at this point */
        BitSampleCount = 0;

        switch (StateRegister) {
            case DEMOD_VICC_SOC_STATE:
                if (SampleRegister == VICC_SOC_CODE) {
                    StateRegister = DEMOD_VICC_DATA;
                } else { // No SOC. Restart and try again, we probably received garbage.
                    Flags.ReaderDemodFinished = 1;
                    Flags.CardDemodFinished = 1;

                    // TODO disable interrupts and cleanup to start VICC data reception again
                    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
                    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code
                }

                break;
            case DEMOD_VICC_DATA:
                if (SampleRegisterL == VICC_EOC_CODE) {
                    Flags.CardDemodFinished = 1;
                    PORTE.OUTTGL = PIN0_bm; // TODO_sniff remove this testing code

                    CardByteCount = ByteCount; /* Copy to direction-specific variable */

                    // TODO disable everything sniff, restart reader
                } else {
                    DataRegister  = ( (SampleRegisterL & 0b00000011) == 0b00000001) << 3;
                    DataRegister |= ( (SampleRegisterL & 0b00001100) == 0b00000100) << 2;
                    DataRegister |= ( (SampleRegisterL & 0b00110000) == 0b00010000) << 1;
                    DataRegister |= ( (SampleRegisterL & 0b11000000) == 0b01000000); /* Bottom 2 bits */
                    DataRegister |= ( (SampleRegisterH & 0b00000011) == 0b00000001) << 7;
                    DataRegister |= ( (SampleRegisterH & 0b00001100) == 0b00000100) << 6;
                    DataRegister |= ( (SampleRegisterH & 0b00110000) == 0b00010000) << 5;
                    DataRegister |= ( (SampleRegisterH & 0b11000000) == 0b01000000) << 4;

                    *CodecBufferPtr = DataRegister;
                    CodecBufferPtr++;
                    ByteCount++;
                }
        }
    }

}

















/* This functions resets all global variables used in the codec and enables interrupts to wait for reader data */
void StartSniffISO15693Demod(void) {
    /* Reset global variables to default values */
    CodecBufferPtr = CodecBuffer;
    Flags.ReaderDemodFinished = 0;
    Flags.CardDemodFinished = 0;
    DemodState = DEMOD_VCD_SOC_STATE;
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
    DemodState = DEMOD_VCD_SOC_STATE;
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

        DemodByteCount = ReaderByteCount;
        bDualSubcarrier = 0;

        if (DemodByteCount > 0) {
            LogEntry(LOG_INFO_CODEC_SNI_READER_DATA, CodecBuffer, DemodByteCount);
            LEDHook(LED_CODEC_RX, LED_PULSE);

            if (CodecBuffer[0] & ISO15693_REQ_SUBCARRIER_DUAL) {
                bDualSubcarrier = 1;
            }
        }

    }

    if(Flags.CardDemodFinished) {
        Flags.CardDemodFinished = 0;

        DemodByteCount = CardByteCount;

        if (DemodByteCount > 0) {
            LogEntry(LOG_INFO_CODEC_SNI_CARD_DATA, CodecBuffer, DemodByteCount);
            LEDHook(LED_CODEC_RX, LED_PULSE);

        }

        StartSniffISO15693Demod();
    }
}
