// ============================================================================
// system_storage: centralized register file for a multi-axis motor controller.
// Provides software-writable configuration registers (enable, PWM duty values)
// and read-only access to hardware feedback (encoder positions/periods/flags,
// ADC current measurements) for each axis.
// ============================================================================
module system_storage #(
    parameter AXIS_COUNT = 3 // Number of motor axes supported
)(
    input  logic clk,        // System clock
    input  logic rst_n,      // Active-low asynchronous reset
    
    // Register-bus interface (directly from SPI bridge or similar)
    input  logic        write_en,  // Write-enable strobe
    input  logic [7:0]  addr,      // Register address: upper nibble = axis, lower nibble = register index
    input  logic [31:0] wdata,     // Data to write
    output logic [31:0] rdata,     // Data read back (active on current addr, no latency)

    // Per-axis outputs derived from writable registers
    output logic [AXIS_COUNT-1:0] enables,              // Axis enable bits (one per axis)
    output logic [15:0] pwm_a_0, pwm_a_1, pwm_a_2,     // Phase-A PWM duty cycle for axes 0/1/2
    output logic [15:0] pwm_b_0, pwm_b_1, pwm_b_2,     // Phase-B PWM duty cycle for axes 0/1/2
    output logic [15:0] pwm_c_0, pwm_c_1, pwm_c_2,     // Phase-C PWM duty cycle for axes 0/1/2
    
    // Encoder feedback — motor-side position (accumulated pulse count)
    input  logic [31:0] enc_motor_pos_0,  enc_motor_pos_1,  enc_motor_pos_2,
    // Encoder feedback — output-side (load) position
    input  logic [31:0] enc_output_pos_0, enc_output_pos_1, enc_output_pos_2,
    // Encoder feedback — motor-side period (time between edges, for speed)
    input  logic [31:0] enc_motor_per_0,  enc_motor_per_1,  enc_motor_per_2,
    // Encoder feedback — output-side period
    input  logic [31:0] enc_output_per_0, enc_output_per_1, enc_output_per_2,
    // Encoder direction flags (1 bit per axis): current rotation direction
    input  logic [2:0]  enc_motor_dir,
    input  logic [2:0]  enc_output_dir,
    // Encoder index pulse flags (1 bit per axis): asserted when index mark seen
    input  logic [2:0]  enc_motor_idx,
    input  logic [2:0]  enc_output_idx,

    // ADC data from G4 (2×16-bit currents packed into 32 bits, per axis)
    input  logic [31:0] adc_data_0,
    input  logic [31:0] adc_data_1,
    input  logic [31:0] adc_data_2
);

    // -----------------------------------------------------------------------
    // Software-writable register bank: 3 axes × 4 registers each.
    //   reg[axis][0] — control/enable register (bit 0 = axis enable)
    //   reg[axis][1] — phase-A PWM duty (lower 16 bits used)
    //   reg[axis][2] — phase-B PWM duty (lower 16 bits used)
    //   reg[axis][3] — phase-C PWM duty (lower 16 bits used)
    // -----------------------------------------------------------------------
    logic [31:0] regs [0:2][0:3]; 

    // Split the 8-bit address into axis selector and register selector
    logic [3:0] axis_id; // Upper nibble: selects axis (0, 1, or 2)
    logic [3:0] reg_id;  // Lower nibble: selects register within the axis
    assign axis_id = addr[7:4];
    assign reg_id  = addr[3:0];

    // --- WRITE logic: store incoming data into the register bank ---
    // Synchronous write with asynchronous reset; ignores writes to
    // out-of-range addresses (axis >= 3 or reg >= 4).
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Clear all writable registers on reset
            for (int i = 0; i < 3; i++)
                for (int r = 0; r < 4; r++)
                    regs[i][r] <= 32'd0;
        end else if (write_en && (axis_id < 3) && (reg_id < 4)) begin
            regs[axis_id][reg_id] <= wdata;
        end
    end

    // --- READ logic: combinationally mux the selected register onto rdata ---
    // Addresses 0x00-0x03 per axis map to writable regs; 0x04-0x09 map to
    // read-only hardware feedback signals.
    always_comb begin
        rdata = 32'd0; // Default: return zero for unmapped addresses
        case ({axis_id, reg_id})
            // ============== Axis 0 ==============
            // Software-writable registers
            {4'd0, 4'h0}: rdata = regs[0][0]; // Control/enable register
            {4'd0, 4'h1}: rdata = regs[0][1]; // Phase-A PWM duty
            {4'd0, 4'h2}: rdata = regs[0][2]; // Phase-B PWM duty
            {4'd0, 4'h3}: rdata = regs[0][3]; // Phase-C PWM duty
            // Hardware read-only registers
            {4'd0, 4'h4}: rdata = enc_motor_pos_0;  // Motor encoder position
            {4'd0, 4'h5}: rdata = enc_output_pos_0; // Output encoder position
            {4'd0, 4'h6}: rdata = enc_motor_per_0;  // Motor encoder period (speed)
            {4'd0, 4'h7}: rdata = enc_output_per_0; // Output encoder period
            // Packed encoder status flags: [3]=output_idx, [2]=motor_idx, [1]=output_dir, [0]=motor_dir
            {4'd0, 4'h8}: rdata = {28'd0, enc_output_idx[0], enc_motor_idx[0], enc_output_dir[0], enc_motor_dir[0]};
            {4'd0, 4'h9}: rdata = adc_data_0; // ADC phase-current measurements

            // ============== Axis 1 ==============
            {4'd1, 4'h0}: rdata = regs[1][0]; // Control/enable register
            {4'd1, 4'h1}: rdata = regs[1][1]; // Phase-A PWM duty
            {4'd1, 4'h2}: rdata = regs[1][2]; // Phase-B PWM duty
            {4'd1, 4'h3}: rdata = regs[1][3]; // Phase-C PWM duty
            {4'd1, 4'h4}: rdata = enc_motor_pos_1;  // Motor encoder position
            {4'd1, 4'h5}: rdata = enc_output_pos_1; // Output encoder position
            {4'd1, 4'h6}: rdata = enc_motor_per_1;  // Motor encoder period
            {4'd1, 4'h7}: rdata = enc_output_per_1; // Output encoder period
            // Encoder status flags for axis 1
            {4'd1, 4'h8}: rdata = {28'd0, enc_output_idx[1], enc_motor_idx[1], enc_output_dir[1], enc_motor_dir[1]};
            {4'd1, 4'h9}: rdata = adc_data_1; // ADC phase-current measurements

            // ============== Axis 2 ==============
            {4'd2, 4'h0}: rdata = regs[2][0]; // Control/enable register
            {4'd2, 4'h1}: rdata = regs[2][1]; // Phase-A PWM duty
            {4'd2, 4'h2}: rdata = regs[2][2]; // Phase-B PWM duty
            {4'd2, 4'h3}: rdata = regs[2][3]; // Phase-C PWM duty
            {4'd2, 4'h4}: rdata = enc_motor_pos_2;  // Motor encoder position
            {4'd2, 4'h5}: rdata = enc_output_pos_2; // Output encoder position
            {4'd2, 4'h6}: rdata = enc_motor_per_2;  // Motor encoder period
            {4'd2, 4'h7}: rdata = enc_output_per_2; // Output encoder period
            // Encoder status flags for axis 2
            {4'd2, 4'h8}: rdata = {28'd0, enc_output_idx[2], enc_motor_idx[2], enc_output_dir[2], enc_motor_dir[2]};
            {4'd2, 4'h9}: rdata = adc_data_2; // ADC phase-current measurements

            default: rdata = 32'd0; // Unmapped address: read as zero
        endcase
    end

    // --- OUTPUT ROUTING: fan-out writable register fields to module outputs ---

    // Enable bit for each axis is bit 0 of that axis's control register (reg 0)
    assign enables[0] = regs[0][0][0];
    assign enables[1] = regs[1][0][0];
    assign enables[2] = regs[2][0][0];
    
    // Phase-A/B/C PWM duty values (lower 16 bits of regs 1/2/3 per axis)
    assign pwm_a_0 = regs[0][1][15:0]; // Axis 0, phase A
    assign pwm_b_0 = regs[0][2][15:0]; // Axis 0, phase B
    assign pwm_c_0 = regs[0][3][15:0]; // Axis 0, phase C
    assign pwm_a_1 = regs[1][1][15:0]; // Axis 1, phase A
    assign pwm_b_1 = regs[1][2][15:0]; // Axis 1, phase B
    assign pwm_c_1 = regs[1][3][15:0]; // Axis 1, phase C
    assign pwm_a_2 = regs[2][1][15:0]; // Axis 2, phase A
    assign pwm_b_2 = regs[2][2][15:0]; // Axis 2, phase B
    assign pwm_c_2 = regs[2][3][15:0]; // Axis 2, phase C

endmodule