// =============================================================================
// tb_uart_term.v  -  behavioural "PC terminal" on the other end of the UART
// -----------------------------------------------------------------------------
//   send(byte, bad_parity, bad_stop)  transmit one frame (errors on request)
//   send_break(bit_times)             hold the line low
//   every frame the DUT sends is decoded into rx_mem[] / rx_cnt
//   frame_lvls[] holds the line levels of the last frame (start..stop)
// The frame format (bit_ns, nbits, par_en, par_odd, stop2) is set by the
// testbench through hierarchical references, e.g.  term.bit_ns = 1000.0;
// =============================================================================
`timescale 1ns/1ps
module tb_uart_term (
    input  wire rx,          // from the DUT's TXD
    output reg  tx           // to the DUT's RXD
);
    real    bit_ns  = 8680.556;     // 115200 baud
    integer nbits   = 8;
    reg     par_en  = 1'b0;
    reg     par_odd = 1'b0;
    reg     stop2   = 1'b0;

    reg [7:0] rx_mem   [0:4095];
    reg       rx_perr  [0:4095];
    reg       rx_ferr  [0:4095];
    integer   rx_cnt   = 0;
    reg       frame_lvls [0:11];    // start, data..., [parity], stop
    integer   frame_len = 0;

    initial tx = 1'b1;

    task send;
        input [7:0] b;
        input       bad_parity;
        input       bad_stop;
        integer i;
        reg     p;
        begin
            p = (^(b & ((9'd1 << nbits) - 9'd1))) ^ par_odd ^ bad_parity;
            tx = 1'b0;                     #(bit_ns);
            for (i = 0; i < nbits; i = i + 1) begin
                tx = b[i];                 #(bit_ns);
            end
            if (par_en) begin tx = p;      #(bit_ns); end
            tx = bad_stop ? 1'b0 : 1'b1;   #(bit_ns);
            if (stop2)  begin tx = 1'b1;   #(bit_ns); end
            tx = 1'b1;
        end
    endtask

    task send_break;
        input integer bit_times;
        begin
            tx = 1'b0;
            #(bit_ns * bit_times);
            tx = 1'b1;
            #(bit_ns * 2);
        end
    endtask

    // ---------------- receiver ----------------
    initial begin : rx_loop
        integer i;
        reg [7:0] d;
        reg       p, s;
        forever begin
            @(negedge rx);
            #(bit_ns / 2.0);
            if (rx === 1'b0) begin                    // a real start bit
                frame_len = 0;
                frame_lvls[frame_len] = rx; frame_len = frame_len + 1;
                d = 8'd0;
                for (i = 0; i < nbits; i = i + 1) begin
                    #(bit_ns);
                    d[i] = rx;
                    frame_lvls[frame_len] = rx; frame_len = frame_len + 1;
                end
                p = 1'b0;
                if (par_en) begin
                    #(bit_ns);
                    p = rx;
                    frame_lvls[frame_len] = rx; frame_len = frame_len + 1;
                end
                #(bit_ns);
                s = rx;
                frame_lvls[frame_len] = rx; frame_len = frame_len + 1;
                rx_mem[rx_cnt]  = d;
                rx_perr[rx_cnt] = par_en && (p !== ((^d) ^ par_odd));
                rx_ferr[rx_cnt] = (s !== 1'b1);
                rx_cnt = rx_cnt + 1;
                // we are now in the middle of the stop bit
            end
        end
    end
endmodule
