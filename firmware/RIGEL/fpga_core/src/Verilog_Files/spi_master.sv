// ============================================================================
// spi_master: configurable-width SPI master with full-duplex shift register.
// Designed to communicate with an STM32 (or similar) acting as SPI slave.
// The CS falling edge serves as an EXTI trigger so the slave can set up DMA
// before clocking begins. Supports arbitrary packet widths (e.g. 384 bits)
// and a programmable SCK frequency via CLK_DIV.
// ============================================================================
module spi_master #(
    parameter int DATA_WIDTH = 384, // Total number of bits per SPI transaction
    parameter int CLK_DIV    = 2,   // SCK half-period in system clock cycles (fSCK = fCLK / (2*(CLK_DIV+1)))
    parameter int CS_WAIT    = 10   // Number of system clocks to wait after CS assert before clocking (gives slave time for DMA setup)
)(
    input  logic                    clk,    // System clock
    input  logic                    rst_n,  // Active-low asynchronous reset

    // --- Internal (fabric-side) interface ---
    input  logic                    start,      // Single-cycle pulse to begin a transaction
    input  logic [DATA_WIDTH-1:0]   tx_data,    // Parallel TX payload to shift out on MOSI
    output logic [DATA_WIDTH-1:0]   rx_data,    // Parallel RX payload captured from MISO (valid when done=1)
    output logic                    busy,       // High while a transaction is in progress
    output logic                    done,       // Single-cycle pulse indicating transaction complete & rx_data valid

    // --- Physical SPI signals ---
    output logic spi_cs_n,   // Chip-select (active low); directly drives the pad
    output logic spi_sck,    // Serial clock output
    output logic spi_mosi,   // Master-Out-Slave-In: we transmit data to the slave
    input  logic spi_miso    // Master-In-Slave-Out: we receive data from the slave
);

    // FSM state encoding — explicit 2-bit constants for Gowin EDA compatibility
    localparam IDLE      = 2'd0; // Idle: CS high, waiting for start pulse
    localparam LEAD_WAIT = 2'd1; // Lead-in wait: CS just asserted, waiting CS_WAIT cycles
    localparam TRANSFER  = 2'd2; // Transfer: shifting DATA_WIDTH bits in/out
    localparam DONE_ST   = 2'd3; // Done: latch result, de-assert CS, pulse done

    logic [1:0] state; // Current FSM state

    logic [15:0] timer;        // General-purpose timer used during LEAD_WAIT
    logic [15:0] bit_counter;  // Number of bits remaining to transfer
    logic [15:0] clk_counter;  // Counts system clocks to derive SCK timing
    
    logic [DATA_WIDTH-1:0] shift_rx; // Shift register collecting incoming MISO bits
    logic [DATA_WIDTH-1:0] shift_tx; // Shift register feeding outgoing MOSI bits
    logic sck_internal;              // Internal SCK phase tracker (0=low, 1=high)

    // MOSI is always the MSB of the TX shift register (MSB-first transmission)
    assign spi_mosi = shift_tx[DATA_WIDTH-1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all outputs and internal state to safe defaults
            state        <= IDLE;
            spi_cs_n     <= 1'b1;   // CS inactive (high)
            spi_sck      <= 1'b0;   // SCK idle low (CPOL=0)
            sck_internal <= 1'b0;
            done         <= 1'b0;
            busy         <= 1'b0;
            rx_data      <= '0;
            shift_tx     <= '0;
            shift_rx     <= '0;
        end else begin
            done <= 1'b0; // done is a single-cycle pulse; clear it every clock

            case (state)
                // ----------------------------------------------------------
                // IDLE: wait for the start pulse to kick off a transaction
                // ----------------------------------------------------------
                IDLE: begin
                    spi_cs_n     <= 1'b1; // Keep CS de-asserted
                    spi_sck      <= 1'b0; // SCK idle low
                    sck_internal <= 1'b0;
                    busy         <= 1'b0;
                    
                    if (start) begin
                        state    <= LEAD_WAIT;
                        spi_cs_n <= 1'b0;      // Assert CS (triggers EXTI on STM32 slave)
                        shift_tx <= tx_data;    // Latch parallel TX data into shift register
                        timer    <= 0;
                        busy     <= 1'b1;       // Signal that SPI is now occupied
                    end
                end

                // ----------------------------------------------------------
                // LEAD_WAIT: hold CS low for CS_WAIT clocks so the slave
                // microcontroller has time to configure its DMA/SPI peripheral
                // ----------------------------------------------------------
                LEAD_WAIT: begin
                    if (timer >= CS_WAIT) begin
                        // Wait complete — proceed to bit transfer
                        state       <= TRANSFER;
                        bit_counter <= DATA_WIDTH; // Load the full bit count
                        clk_counter <= 0;
                    end else begin
                        timer <= timer + 16'd1; // Keep counting
                    end
                end

                // ----------------------------------------------------------
                // TRANSFER: generate SCK edges and shift data in/out.
                // Uses CPOL=0 / CPHA=0 style:
                //   • Rising SCK edge  → sample MISO (capture)
                //   • Falling SCK edge  → advance MOSI (shift TX register)
                // clk_counter throttles edge generation to achieve CLK_DIV.
                // ----------------------------------------------------------
                TRANSFER: begin
                    if (clk_counter >= CLK_DIV) begin
                        clk_counter <= 0;
                        // Toggle internal SCK and drive the external pin
                        sck_internal <= ~sck_internal;
                        spi_sck      <= ~sck_internal;

                        if (sck_internal == 1'b0) begin
                            // === RISING EDGE of SCK ===
                            // Sample MISO and shift it into the LSB of the RX register
                            shift_rx <= {shift_rx[DATA_WIDTH-2:0], spi_miso};
                        end else begin
                            // === FALLING EDGE of SCK ===
                            // Check whether this was the last bit
                            if (bit_counter == 1) begin
                                // All bits transferred — go to DONE
                                state    <= DONE_ST;
                                spi_sck  <= 1'b0; // Return SCK to idle-low
                            end else begin
                                // Shift the TX register left (next bit appears on MOSI via assign)
                                shift_tx    <= {shift_tx[DATA_WIDTH-2:0], 1'b0};
                                bit_counter <= bit_counter - 16'd1;
                            end
                        end
                    end else begin
                        clk_counter <= clk_counter + 16'd1; // Wait for next SCK edge time
                    end
                end

                // ----------------------------------------------------------
                // DONE_ST: de-assert CS, publish the received data on rx_data,
                // pulse 'done', and return to IDLE.
                // ----------------------------------------------------------
                DONE_ST: begin
                    spi_cs_n <= 1'b1;      // Release CS (transaction over)
                    rx_data  <= shift_rx;   // Latch the fully received word to output
                    done     <= 1'b1;       // Single-cycle completion flag
                    busy     <= 1'b0;       // SPI is free again
                    state    <= IDLE;
                end
                
                // Defensive default: recover to IDLE if FSM enters illegal state
                default: state <= IDLE;
            endcase
        end
    end

endmodule