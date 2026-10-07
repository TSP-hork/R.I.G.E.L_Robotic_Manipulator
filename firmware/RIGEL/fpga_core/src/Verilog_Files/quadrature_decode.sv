// ============================================================================
// ABZ Encoder Decoder with Glitch Filter and Precise Velocity Measurement.
//
// Features:
// - Glitch Filter (removes noise from wires).
// - X4 Decoding (counts every edge of A and B).
// - Index (Z) channel support for homing.
// - Velocity measurement via "Time between pulses" (Period).
// - 32-bit Position Counter.
//
// Typical usage: connect to a rotary incremental encoder (motor or output
// shaft). The module outputs absolute tick count, rotation direction, period
// between edges (for speed calculation on the host MCU), and an index-found
// latch for homing sequences.
// ============================================================================


module quadrature_decoder #(
    parameter int CLK_FREQ = 27_000_000, // System clock frequency (used externally for speed scaling)
    parameter int FILTER_LEN = 4         // Glitch-filter depth: input must be stable this many clocks to be accepted
)(
    input  logic        clk,      // System clock
    input  logic        rst_n,    // Active-low asynchronous reset
    
    // --- Physical Inputs (directly from encoder pads) ---
    input  logic        a_in,   // Quadrature phase A
    input  logic        b_in,   // Quadrature phase B
    input  logic        z_in,   // Index pulse Z (one pulse per full revolution)

    // --- Control Interface (directly from register bridge / software) ---
    input  logic        set_zero,      // When pulsed high, forces position counter to zero
    input  logic        homing_mode,   // When 1, a Z-pulse automatically resets position to zero (hardware homing)

    // --- Outputs to System Storage ---
    output logic [31:0] position,      // Accumulated absolute position in encoder ticks (signed interpretation by SW)
    output logic [31:0] period,        // Clock cycles elapsed between the last two quadrature edges (inverse of velocity)
    output logic        dir_fwd,       // Direction flag: 1 = forward (CW), 0 = backward (CCW)
    output logic        index_found    // Sticky latch: set to 1 the first time a Z pulse is detected after reset
);

    // -----------------------------------------------------------------------
    // 1. GLITCH FILTER
    // -----------------------------------------------------------------------
    // Each input (A, B, Z) is passed through a shift register of depth
    // FILTER_LEN. The filtered ("clean") value only changes when the entire
    // shift register is unanimously 0 or 1, rejecting short glitches/noise.
    // -----------------------------------------------------------------------
    logic [FILTER_LEN-1:0] a_filter, b_filter, z_filter; // Shift-register histories
    logic a_clean, b_clean, z_clean;                      // Debounced (clean) versions

    // Shift new samples into the MSB of each filter register every clock
    always_ff @(posedge clk) begin
        a_filter <= {a_filter[FILTER_LEN-2:0], a_in};
        b_filter <= {b_filter[FILTER_LEN-2:0], b_in};
        z_filter <= {z_filter[FILTER_LEN-2:0], z_in};
    end

    // Strict-stability filter function:
    //   - If every bit in the history is 1 → output 1
    //   - If every bit in the history is 0 → output 0
    //   - Otherwise (mixed / transitioning) → hold previous value (reject noise)
    function automatic logic filter_logic(logic [FILTER_LEN-1:0] hist, logic prev);
        if (&hist) return 1'b1;           // All bits are 1 → accept high
        else if (|hist == 1'b0) return 1'b0; // All bits are 0 → accept low
        else return prev;                 // Mixed → keep previous (noise rejection)
    endfunction

    // Apply the filter function to produce clean signals, reset them to 0
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_clean <= 0; b_clean <= 0; z_clean <= 0;
        end else begin
            a_clean <= filter_logic(a_filter, a_clean);
            b_clean <= filter_logic(b_filter, b_clean);
            z_clean <= filter_logic(z_filter, z_clean);
        end
    end

    // -----------------------------------------------------------------------
    // 2. EDGE DETECTION & X4 QUADRATURE DECODING
    // -----------------------------------------------------------------------
    // The two-bit state {A, B} follows a Gray-code sequence as the shaft
    // rotates. By detecting every valid single-step transition we achieve
    // X4 resolution (four counts per full A/B cycle).
    //
    // Forward  (CW)  sequence: 00→10→11→01→00 …
    // Backward (CCW) sequence: 00→01→11→10→00 …
    //
    // Any transition not in these sets is ignored (invalid / skipped step).
    // -----------------------------------------------------------------------
    logic [1:0] current_state, prev_state; // {A, B} now and one cycle ago
    logic       pulse_event;               // Pulses high for exactly 1 clock on each valid quadrature step
    logic       direction;                 // Latched direction: 1 = forward, 0 = backward

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position      <= 0;
            prev_state    <= 0;
            index_found   <= 0;
            pulse_event   <= 0;
            direction     <= 1;            // Default direction: forward
        end else begin
            // Capture the current filtered quadrature state and remember previous
            current_state <= {a_clean, b_clean};
            prev_state    <= current_state;
            pulse_event   <= 0; // Default: no event this cycle

            // --- Software zero-reset command ---
            if (set_zero) position <= 0;

            // --- Z-Index pulse handling ---
            if (z_clean) begin
                index_found <= 1;              // Sticky flag: stays 1 until external reset
                if (homing_mode) position <= 0; // In homing mode, Z resets position automatically
            end

            // --- Quadrature state-transition decode (X4) ---
            case ({prev_state, current_state})
                // Forward transitions: 00→10, 10→11, 11→01, 01→00
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: begin
                    position    <= position + 1;  // Increment position
                    pulse_event <= 1;             // Signal a valid step occurred
                    direction   <= 1;             // Mark direction as forward
                end
                
                // Backward transitions: 00→01, 01→11, 11→10, 10→00
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: begin
                    position    <= position - 1;  // Decrement position
                    pulse_event <= 1;             // Signal a valid step occurred
                    direction   <= 0;             // Mark direction as backward
                end
                // All other transitions (no change, or illegal skip) are silently ignored
            endcase
        end
    end

    // Expose internal direction latch as a module output
    assign dir_fwd = direction;

    // -----------------------------------------------------------------------
    // 3. VELOCITY UNIT (Period / Time-Stamping)
    // -----------------------------------------------------------------------
    // Measures the number of system-clock ticks between consecutive valid
    // quadrature edges. A small period value means high rotational speed;
    // a large value means low speed. The host MCU computes:
    //     velocity = K / period   (where K depends on CLK_FREQ and encoder CPR)
    //
    // If no edges occur for a very long time the timer saturates at 0xFFFFFFFF,
    // effectively reporting "speed ≈ 0" (motor stopped or stalled).
    // -----------------------------------------------------------------------
    
    logic [31:0] timer_cnt;   // Free-running clock-tick counter between edges
    logic [31:0] period_reg;  // Captured period value from the most recent edge pair

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer_cnt  <= 0;
            period_reg <= 32'hFFFFFFFF; // Initialize to max (velocity = 0 / stopped)
        end else begin
            // Increment the timer every clock, but saturate at max to avoid wrap-around
            if (timer_cnt != 32'hFFFFFFFF) begin
                timer_cnt <= timer_cnt + 1;
            end

            if (pulse_event) begin
                // A valid quadrature edge just happened:
                // snapshot the elapsed time as the new period and restart the timer
                period_reg <= timer_cnt;
                timer_cnt  <= 0;
            end
        end
    end

    // Drive the period output continuously from the captured register.
    // The host MCU reads this value and computes: velocity = K / period.
    // If the timer has saturated (motor stopped), period equals 0xFFFFFFFF.
    assign period = period_reg;

endmodule