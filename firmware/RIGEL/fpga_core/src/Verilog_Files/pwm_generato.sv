// ============================================================================
// pwm_generator: center-aligned (triangle/up-down) PWM generator.
//
// Produces a symmetric PWM waveform by counting up to 'period', then back
// down to 0, repeating continuously. The output is high while the counter
// value is below 'duty'. A single-cycle 'center_event' pulse is emitted
// each time the counter reaches the top (peak), useful for synchronizing
// ADC sampling or control-loop updates at the PWM center point.
//
// Effective PWM frequency = clk / (2 × period).
// ============================================================================
module pwm_generator (
    input  logic        clk,          // System clock
    input  logic        rst_n,        // Active-low asynchronous reset
    input  logic        enable,       // Master enable: when low, counter is held at 0 and output is forced low
    input  logic [15:0] period,       // Counter ceiling value — sets the PWM carrier frequency (e.g. 27000 for ~500 Hz at 27 MHz)
    input  logic [15:0] duty,         // Duty-cycle threshold: pwm_out is high while counter < duty
    output logic        pwm_out,      // Raw PWM output signal (active high)
    output logic        center_event  // Single-cycle pulse at the counter peak (top of triangle), for ADC / control sync
);

    logic [15:0] cnt;  // Free-running up/down counter (forms the triangle carrier)
    logic        dir;  // Current count direction: 0 = counting up, 1 = counting down

    // -----------------------------------------------------------------------
    // Up-down (triangle) counter
    // Counts 0 → period (up phase), then period → 0 (down phase), producing
    // a symmetric triangle waveform. This yields center-aligned PWM which
    // minimizes current ripple in motor-drive applications.
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 0;   // Reset counter to bottom
            dir <= 0;   // Start in up-counting direction
        end else begin
            if (enable) begin
                // --- Up-Down Counter Logic (Triangle waveform) ---
                if (dir == 0) begin
                    // Counting UP
                    if (cnt >= period) begin
                        dir <= 1;              // Reached the top → reverse to count down
                        cnt <= cnt - 16'd1;    // Begin descending immediately
                    end else begin
                        cnt <= cnt + 16'd1;    // Continue counting up
                    end
                end else begin
                    // Counting DOWN
                    if (cnt == 0) begin
                        dir <= 0;              // Reached the bottom → reverse to count up
                        cnt <= cnt + 1;        // Begin ascending immediately
                    end else begin
                        cnt <= cnt - 16'd1;    // Continue counting down
                    end
                end
            end else begin
                // Module disabled: hold counter at zero (safe state, output will be low)
                cnt <= 0;
                dir <= 0;
            end
        end
    end

    // PWM output: high when counter is below the duty threshold AND module is enabled
    assign pwm_out = (cnt < duty) && enable;
    
    // Center-event flag: pulses high for one clock when the counter hits the peak.
    // Useful for triggering ADC conversions or control-loop calculations at the
    // exact center of the PWM ON-time (best measurement point for phase currents).
    assign center_event = (cnt == period);

endmodule