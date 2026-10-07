// ============================================================================
// deadtime: inserts dead-time (break-before-make) between complementary
// high-side and low-side gate-drive signals for a half-bridge leg.
//
// Takes a single PWM input and produces two non-overlapping outputs:
//   pwm_h — drives the high-side MOSFET/IGBT
//   pwm_l — drives the low-side MOSFET/IGBT (complementary)
//
// When the input transitions, the outgoing signal is turned OFF immediately,
// but the incoming signal is held OFF for 'dt_ticks' clock cycles before
// being allowed ON. This guarantees that both transistors in a half-bridge
// leg are never conducting simultaneously (shoot-through protection).
//
// Timing example (dt_ticks = 3):
//   pwm_in: ____/‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//   pwm_h:  ________/‾‾‾‾‾‾‾‾‾\_____   (turn-on delayed by 3 clocks)
//   pwm_l:  ‾‾‾‾\________/‾‾‾‾‾‾‾‾‾‾   (turn-on delayed by 3 clocks)
//                 ↑dead↑       ↑dead↑
// ============================================================================
module deadtime (
    input  logic       clk,       // System clock
    input  logic       rst_n,     // Active-low asynchronous reset
    input  logic       pwm_in,    // Raw PWM signal from the PWM generator
    input  logic [7:0] dt_ticks,  // Dead-time duration in clock cycles (e.g. 50 → ~1.85 µs at 27 MHz)
    output logic       pwm_h,     // Gate drive for the HIGH-side transistor (active high)
    output logic       pwm_l      // Gate drive for the LOW-side transistor  (active high, complementary)
);

    // Delay counters: track how long the input has been stable in each state
    logic [7:0] rise_cnt;  // Counts clocks since pwm_in went high  (for high-side turn-on delay)
    logic [7:0] fall_cnt;  // Counts clocks since pwm_in went low   (for low-side  turn-on delay)
    
    // Internal registered outputs (directly mapped to pwm_h / pwm_l)
    logic h_out, l_out;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // On reset: both outputs off, counters cleared — safe state
            rise_cnt <= 0;
            fall_cnt <= 0;
            h_out <= 0;
            l_out <= 0;
        end else begin
            // =============================================================
            // High-side logic: turn-on is DELAYED, turn-off is IMMEDIATE
            // =============================================================
            if (pwm_in == 1'b1) begin
                // Input is high → we WANT high-side on, but wait for dead-time first
                if (rise_cnt < dt_ticks) rise_cnt <= rise_cnt + 8'd1; // Still in dead-time window, keep counting
                else                     h_out    <= 1'b1;            // Dead-time elapsed → enable high-side
            end else begin
                // Input went low → immediately turn off high-side (no delay on turn-off)
                rise_cnt <= 0;       // Reset the delay counter for the next rising edge
                h_out    <= 1'b0;    // Instant OFF — prevents shoot-through
            end

            // =============================================================
            // Low-side logic: complementary to high-side, with the same
            // delayed-turn-on / instant-turn-off behaviour
            // =============================================================
            if (pwm_in == 1'b0) begin
                // Input is low → we WANT low-side on (complement), but wait for dead-time first
                if (fall_cnt < dt_ticks) fall_cnt <= fall_cnt + 8'd1; // Still in dead-time window
                else                     l_out    <= 1'b1;            // Dead-time elapsed → enable low-side
            end else begin
                // Input went high → immediately turn off low-side
                fall_cnt <= 0;       // Reset the delay counter for the next falling edge
                l_out    <= 1'b0;    // Instant OFF — prevents shoot-through
            end
        end
    end

    // Drive module outputs from the registered internal signals
    assign pwm_h = h_out; // High-side gate command
    assign pwm_l = l_out; // Low-side gate command

endmodule