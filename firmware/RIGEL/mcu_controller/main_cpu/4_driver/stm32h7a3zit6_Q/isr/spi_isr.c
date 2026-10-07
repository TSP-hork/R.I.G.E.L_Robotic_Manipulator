#include "stm32h7xx.h"
#include "types.h"
#include "config.h"
#include "spi.h"
#include "foc.h"
#include <math.h>

// Diagnostic counter — incremented every SPI frame, used for LED blink timing
static volatile uint32_t diag_counter = 0;

// ============================================================================
// EXTI4_IRQHandler: interrupt triggered by FPGA's SPI chip-select (CS) on PA4.
//
// The FPGA (SPI master) drives CS low to start a transaction and high when done.
// This handler fires on BOTH edges:
//   • Falling edge (CS low):  configure DMA and enable SPI peripheral for reception.
//   • Rising edge  (CS high): SPI transfer complete — process received data,
//                              compute FOC, fill TX buffer for next frame, and
//                              reset SPI for the next transaction.
//
// This runs at the PWM rate (~20 kHz). All FOC computation happens inside
// the rising-edge branch, keeping the latency to one PWM cycle.
// ============================================================================
void EXTI4_IRQHandler(void) {
    // Clear the EXTI pending flag for line 4 (PA4 = SPI CS from FPGA)
    EXTI->PR1 = (1u << 4);

    if ((GPIOA->IDR & (1u << 4)) == 0) {
        // ================================================================
        // CS FALLING EDGE: FPGA just asserted CS — set up DMA for this
        // SPI transaction so data flows automatically during clocking.
        // ================================================================

        // Clear all DMA stream 0 and stream 1 interrupt flags in one write
        DMA1->LIFCR = 0x0F7D0F7Du;

        // Configure DMA Stream 0 (RX): destination = RX buffer, transfer size = packet
        DMA1_Stream0->M0AR = (uint32_t)spi_get_rx();
        DMA1_Stream0->NDTR = SPI_PACKET_SIZE;
        // Configure DMA Stream 1 (TX): source = TX buffer, transfer size = packet
        DMA1_Stream1->M0AR = (uint32_t)spi_get_tx();
        DMA1_Stream1->NDTR = SPI_PACKET_SIZE;

        // Enable TX DMA first, then RX DMA (TX must be ready before clocks arrive)
        DMA1_Stream1->CR |= DMA_SxCR_EN;
        DMA1_Stream0->CR |= DMA_SxCR_EN;

        // Enable the SPI peripheral — it will now respond to SCK from FPGA
        SPI3->CR1 |= SPI_CR1_SPE;

    } else {
        // ================================================================
        // CS RISING EDGE: FPGA released CS — the full-duplex SPI transfer
        // is complete. Disable SPI/DMA, process received data, compute
        // FOC outputs, and prepare the TX buffer for the next frame.
        // ================================================================

        // Disable SPI peripheral immediately (no more clocks expected)
        SPI3->CR1 &= ~SPI_CR1_SPE;

        // Disable both DMA streams and wait until the hardware confirms they are stopped
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream0->CR & DMA_SxCR_EN);  // Spin until RX DMA fully stopped
        while (DMA1_Stream1->CR & DMA_SxCR_EN);  // Spin until TX DMA fully stopped

        // Get pointers to the RX (sensor data from FPGA) and TX (commands to FPGA) buffers
        volatile spi_rx_packet_t* rx = spi_get_rx();
        volatile spi_tx_packet_t* tx = spi_get_tx();

        diag_counter++;  // Count processed frames (used for LED timing)

        // ============================================================
        // LED DIAGNOSTICS (active in ALL run modes)
        // PE1 toggles every packet — fast blinking = communication alive
        // ============================================================
        GPIOE->ODR ^= (1u << 1);

        // ============================================================
        // RUN MODES: compile-time selection of operating behavior.
        // RUN_MODE is defined in config.h.
        // ============================================================

#if RUN_MODE == 0
        // ============================================================
        // MODE 0: OPEN-LOOP ROTATION
        // Generates a slowly rotating voltage vector (no current feedback).
        // Used for initial motor spin-up test and ADC range diagnostics.
        // ============================================================
        {
            // Slowly advancing electrical angle (open-loop, no encoder)
            static float ol_angle = 0.0f;
            ol_angle += 0.01f;                              // Increment angle each packet (~20 kHz)
            if (ol_angle > TWO_PI_F) ol_angle -= TWO_PI_F;  // Wrap to [0, 2π)

            // Fixed voltage amplitude (25% of bus voltage)
            float amplitude = 0.25f;
            float v_alpha = amplitude * cosf(ol_angle);     // Stationary α-axis voltage
            float v_beta  = amplitude * sinf(ol_angle);     // Stationary β-axis voltage
            svpwm_output_t pwm = foc_calc_svpwm(v_alpha, v_beta);  // Convert to PWM ticks

            // Send PWM commands for axis 0
            tx->axis[0].pwm_a = pwm.pwm_a;
            tx->axis[0].pwm_b = pwm.pwm_b;
            tx->axis[0].pwm_c = pwm.pwm_c;
            tx->axis[0].flags = 1;  // Enable axis 0 gate drivers

            // --- ADC diagnostics: track min/max of phase-A current reading ---
            static uint16_t min_ia = 4096;  // Running minimum (starts above max ADC value)
            static uint16_t max_ia = 0;     // Running maximum (starts below min ADC value)
            uint16_t ia = rx->axis[0].i_a_raw;
            if (ia < min_ia) min_ia = ia;
            if (ia > max_ia) max_ia = ia;
            
            uint16_t range = max_ia - min_ia;  // Peak-to-peak ADC swing (indicates current flow)
            
            // Turn all diagnostic LEDs off first
            GPIOB->BSRR = (1u << (0+16));    // Green OFF (PB0 reset)
            GPIOE->BSRR = (1u << (1+16));    // Yellow OFF (PE1 reset)
            GPIOB->BSRR = (1u << (14+16));   // Red OFF (PB14 reset)
            
            // Light one LED based on measured current range (indicates ADC health)
            if (range > 500) {
                GPIOB->BSRR = (1u << 0);     // Green ON — large swing, ADC working well
            } else if (range > 200) {
                GPIOE->BSRR = (1u << 1);     // Yellow ON — moderate swing
            } else {
                GPIOB->BSRR = (1u << 14);    // Red ON — tiny or no swing, possible ADC issue
            }

            // Disable all other axes (only axis 0 active in this mode)
            for (int i = 1; i < AXIS_COUNT; i++) {
                tx->axis[i].pwm_a = 0;
                tx->axis[i].pwm_b = 0;
                tx->axis[i].pwm_c = 0;
                tx->axis[i].flags = 0;
            }
        }

#elif RUN_MODE == 1
        // ============================================================
        // MODE 1: ENCODER READBACK TEST
        // Motor stays disabled; only reads encoder to verify wiring
        // and signal integrity. LED indicates encoder stability.
        // ============================================================
        {
            uint16_t ia = rx->axis[0].i_a_raw;    // Read ADC (not used, but available)
            uint32_t enc = rx->axis[0].enc_pos;    // Current encoder position from FPGA
            
            // Compare with previous reading to detect erratic jumps
            static uint32_t prev_enc = 0;
            int32_t diff = (int32_t)(enc - prev_enc);  // Signed difference (handles wrap-around)
            prev_enc = enc;
            
            // If encoder jumps more than 100 ticks in one packet period (~50 µs)
            // it likely indicates a bad connection or noise on the encoder lines
            if (diff > 100 || diff < -100) {
                GPIOB->BSRR = (1u << (14+16));   // LED OFF — encoder jump detected (bad contact)
            } else {
                GPIOB->BSRR = (1u << 14);        // LED ON — encoder reading is stable
            }
            
            // Motor disabled: zero PWM and clear enable flag
            tx->axis[0].pwm_a = 0;
            tx->axis[0].pwm_b = 0;
            tx->axis[0].pwm_c = 0;
            tx->axis[0].flags = 0;
            
            // Disable all other axes as well
            for (int i = 1; i < AXIS_COUNT; i++) {
                tx->axis[i].pwm_a = 0;
                tx->axis[i].pwm_b = 0;
                tx->axis[i].pwm_c = 0;
                tx->axis[i].flags = 0;
            }
        }

#elif RUN_MODE == 2
        // ============================================================
        // MODE 2: ALIGNMENT (static d-axis vector)
        // Applies a constant voltage vector at electrical angle = 0,
        // which pulls the rotor to a known d-axis position.
        // The encoder reading at this position becomes the electrical
        // offset (read back via axis[1] fields for debugging).
        // ============================================================
        {
            // Apply a fixed vector along d-axis: angle=0 → cos(0)=1, sin(0)=0
            float align_amplitude = 0.15f;        // Low amplitude to gently pull rotor
            float v_alpha = align_amplitude;       // cos(0) = 1
            float v_beta  = 0.0f;                  // sin(0) = 0
            svpwm_output_t pwm = foc_calc_svpwm(v_alpha, v_beta);

            tx->axis[0].pwm_a = pwm.pwm_a;
            tx->axis[0].pwm_b = pwm.pwm_b;
            tx->axis[0].pwm_c = pwm.pwm_c;
            tx->axis[0].flags = 1;  // Enable gate drivers

            // Pack the current encoder position into axis 1's unused TX fields
            // so it can be observed externally (e.g. via FPGA debug LEDs or SPI readback)
            tx->axis[1].pwm_a = (uint16_t)(rx->axis[0].enc_pos & 0xFFFF);         // Lower 16 bits
            tx->axis[1].pwm_b = (uint16_t)((rx->axis[0].enc_pos >> 16) & 0xFFFF); // Upper 16 bits
            tx->axis[1].pwm_c = 0;
            tx->axis[1].flags = 0;

            // Axis 2 disabled
            tx->axis[2].pwm_a = 0;
            tx->axis[2].pwm_b = 0;
            tx->axis[2].pwm_c = 0;
            tx->axis[2].flags = 0;

            // LED: slow blink while alignment is active (visual confirmation)
            if (diag_counter & 0x100) {
                GPIOB->BSRR = (1u << 14);        // Red LED ON
            } else {
                GPIOB->BSRR = (1u << (14+16));   // Red LED OFF
            }
        }

#elif RUN_MODE == 3
        // ============================================================
        // MODE 3: ALIGN → CLOSED-LOOP FOC (single axis)
        //
        // Two-phase operation:
        //   Phase 0 (first ~2 seconds): hold rotor at d-axis angle = 0
        //            to establish the encoder reference point.
        //   Phase 1 (afterwards): run full closed-loop FOC with the
        //            calibrated electrical offset.
        //
        // LED colors:
        //   Red    = aligning
        //   Green  = FOC running
        // ============================================================
        {
            static uint32_t phase = 0;          // 0 = aligning, 1 = FOC running
            static uint32_t counter = 0;        // Packet counter within current phase
            static uint32_t enc_at_align = 0;   // Encoder snapshot taken at end of alignment
            
            // Pre-calibrated electrical offset for this motor (240° = 4π/3 radians)
            float current_offset = 4.18879f;

            // ADC zero-current offset (mid-scale for 12-bit ADC = 2048)
            float real_offset_a = CURRENT_OFFSET;
            float real_offset_c = CURRENT_OFFSET;
            int32_t enc_direction = 1;  // Encoder counting direction (+1 or -1)

            uint32_t enc_now = rx->axis[0].enc_pos;  // Current encoder position
            counter++;

            // =============================================
            // PHASE 0: ALIGNMENT (~2 seconds)
            // Apply a static d-axis voltage vector to lock
            // the rotor at a known electrical angle.
            // Red LED is ON during this phase.
            // =============================================
            if (phase == 0) {
                // Static voltage vector at angle = 0 (d-axis)
                float v_alpha = ALIGN_AMPLITUDE;  // cos(0) × amplitude
                float v_beta  = 0.0f;              // sin(0) × amplitude
                svpwm_output_t pwm = foc_calc_svpwm(v_alpha, v_beta);

                tx->axis[0].pwm_a = pwm.pwm_a;
                tx->axis[0].pwm_b = pwm.pwm_b;
                tx->axis[0].pwm_c = pwm.pwm_c;
                tx->axis[0].flags = 1;  // Gates enabled

                // After ALIGN_TIME packets (~2 sec), capture encoder and switch to FOC
                if (counter >= ALIGN_TIME) {
                    enc_at_align = enc_now;         // Snapshot: this encoder value = electrical angle 0
                    phase = 1;                      // Transition to closed-loop FOC
                    counter = 0;
                    foc_init();                     // Re-initialize PI controllers (clear integrators)
                    foc_set_iq_ref(0, TEST_IQ_REF); // Set the torque current reference for testing
                }

                GPIOB->BSRR = (1u << 14);          // Red LED ON (aligning)
                GPIOE->BSRR = (1u << (1 + 16));    // Yellow LED OFF
                GPIOB->BSRR = (1u << (0 + 16));    // Green LED OFF

            // =============================================
            // PHASE 1: CLOSED-LOOP FOC
            // Full current control with encoder feedback.
            // Green LED is ON during this phase.
            // =============================================
            } else {
                
                // --- Read phase currents from ADC and reconstruct third phase ---
                // ADC measures phases A and C; phase B = -(A + C) by Kirchhoff
                float i_a = ((int32_t)rx->axis[0].i_a_raw - real_offset_a) * CURRENT_SCALE;
                float i_c = ((int32_t)rx->axis[0].i_b_raw - real_offset_c) * CURRENT_SCALE;
                float i_b = -i_a - i_c;  // Reconstruct phase B current

                // --- Compute electrical angle from encoder ---
                // delta_enc: encoder ticks relative to the alignment snapshot
                int32_t delta_enc = enc_direction * (int32_t)(enc_now - enc_at_align);
                // Wrap to one mechanical revolution [0, ENCODER_CPR)
                int32_t ticks_mod = delta_enc % ENCODER_CPR;
                if (ticks_mod < 0) ticks_mod += ENCODER_CPR;
                
                // Convert ticks to mechanical angle in radians
                float mech_angle = (float)ticks_mod * (TWO_PI_F / (float)ENCODER_CPR);
                // Multiply by pole pairs and add calibrated offset to get electrical angle
                float elec_angle = mech_angle * (float)POLE_PAIRS + current_offset;
                
                // Wrap electrical angle to [0, 2π)
                while (elec_angle >= TWO_PI_F) elec_angle -= TWO_PI_F;
                while (elec_angle < 0.0f)      elec_angle += TWO_PI_F;

                // --- Execute FOC: Clarke → Park → PI → inverse Park ---
                foc_output_t foc = foc_step(0, i_a, i_b, elec_angle);
                // --- Convert αβ voltages to PWM timer ticks via SVPWM ---
                svpwm_output_t pwm = foc_calc_svpwm(foc.v_alpha, foc.v_beta);
                
                tx->axis[0].pwm_a = pwm.pwm_a;
                tx->axis[0].pwm_b = pwm.pwm_b;
                tx->axis[0].pwm_c = pwm.pwm_c;
                tx->axis[0].flags = 1;  // Gates enabled

                // LED indication: Green = FOC running normally
                GPIOB->BSRR = (1u << (14 + 16));   // Red OFF
                GPIOE->BSRR = (1u << (1 + 16));    // Yellow OFF
                GPIOB->BSRR = (1u << 0);           // Green ON
            }

            // Disable all other axes (only axis 0 used in this mode)
            for (int i = 1; i < AXIS_COUNT; i++) {
                tx->axis[i].pwm_a = 0;
                tx->axis[i].pwm_b = 0;
                tx->axis[i].pwm_c = 0;
                tx->axis[i].flags = 0;
            }
        }

#elif RUN_MODE == 4
        // ============================================================
        // MODE 4: FULL CLOSED-LOOP FOC (all axes, production mode)
        // Runs FOC on every axis using pre-calibrated constants from
        // config.h. An outer position/velocity loop (not yet implemented)
        // would call foc_set_iq_ref() to command torque.
        // ============================================================
        {
            for (int i = 0; i < AXIS_COUNT; i++) {
                // Convert raw ADC readings to amperes (subtract zero offset, scale)
                float i_a = ((int32_t)rx->axis[i].i_a_raw - CURRENT_OFFSET)
                           * CURRENT_SCALE;
                float i_b = ((int32_t)rx->axis[i].i_b_raw - CURRENT_OFFSET)
                           * CURRENT_SCALE;

                // Compute mechanical angle from encoder (modulo one revolution)
                uint32_t ticks_in_rev = rx->axis[i].enc_pos % ENCODER_CPR;
                float mech_angle = (float)ticks_in_rev
                                 * (TWO_PI_F / (float)ENCODER_CPR);
                // Derive electrical angle: mechanical × pole_pairs + calibrated offset
                float elec_angle = mech_angle * (float)POLE_PAIRS
                                 + ELECTRICAL_OFFSET;

                // Run FOC current loop and SVPWM modulator
                foc_output_t foc = foc_step(i, i_a, i_b, elec_angle);
                svpwm_output_t pwm = foc_calc_svpwm(foc.v_alpha, foc.v_beta);

                tx->axis[i].pwm_a = pwm.pwm_a;
                tx->axis[i].pwm_b = pwm.pwm_b;
                tx->axis[i].pwm_c = pwm.pwm_c;
                tx->axis[i].flags = 1;  // All axes enabled
            }
        }
#endif

        // ============================================================
        // COMMON FOOTER: applied after every run mode
        // ============================================================

        // Write the magic response header so the FPGA can verify link_ok
        tx->magic = SPI_MAGIC;
        // Increment and store the host-side frame counter for diagnostics
        spi_increment_count();
        tx->status = spi_get_packet_count();

        // ============================================================
        // SPI PERIPHERAL RESET
        // The STM32H7 SPI3 must be fully reset between transactions to
        // clear internal FIFOs and state. This is done by toggling the
        // peripheral's reset bit in the RCC register.
        // ============================================================
        RCC->APB1LRSTR |= (1u << 15);    // Assert SPI3 reset
        __DSB();                           // Ensure the write completes before continuing
        for (volatile int i = 0; i < 16; i++);  // Short delay for reset to take effect
        RCC->APB1LRSTR &= ~(1u << 15);   // Release SPI3 reset
        __DSB();
        for (volatile int i = 0; i < 16; i++);  // Short delay for peripheral to stabilize

        // Re-configure SPI3 registers after reset:
        //   DSIZE = 7 (8-bit data frames), RX/TX DMA enabled
        SPI3->CFG1 = (7u << SPI_CFG1_DSIZE_Pos)
                   | SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN;
        //   SSM = software slave management (CS handled by EXTI, not SPI hardware)
        SPI3->CFG2 = SPI_CFG2_SSM;
        //   CR2 = 0: no specific transfer size limit (DMA controls the count)
        SPI3->CR2  = 0;
    }
}