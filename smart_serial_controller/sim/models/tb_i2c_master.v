// =============================================================================
// tb_i2c_master.v  -  a second I2C master on the same bus (100 kHz)
// -----------------------------------------------------------------------------
//   transaction(b) : wait for a free bus, START, send b, STOP
//   join_start(b)  : wait for somebody else's START and start at the same
//                    instant, then send b - used to force an arbitration
//                    contest with the DUT
// It follows the wired-AND rules: after letting SCL go it waits until SCL is
// really high, so the two masters' clocks synchronise.
// =============================================================================
`timescale 1ns/1ps
module tb_i2c_master (
    inout wire scl,
    inout wire sda
);
    reg sda_low = 1'b0;
    reg scl_low = 1'b0;
    assign sda = sda_low ? 1'b0 : 1'bz;
    assign scl = scl_low ? 1'b0 : 1'bz;

    integer tq         = 2500;      // quarter period in ns (100 kHz)
    reg     won        = 1'b1;      // we never saw a bit different from ours
    reg     got_ack    = 1'b0;
    integer done_count = 0;

    task send_byte;                 // SCL is low when this starts
        input [7:0] b;
        integer i;
        begin
            won = 1'b1;
            for (i = 7; i >= 0; i = i - 1) begin
                #(tq); sda_low = ~b[i];
                #(tq); scl_low = 1'b0;
                wait (scl === 1'b1);
                #(tq);
                if (sda !== b[i]) won = 1'b0;
                #(tq); scl_low = 1'b1;
            end
            #(tq); sda_low = 1'b0;            // release SDA for the ACK
            #(tq); scl_low = 1'b0;
            wait (scl === 1'b1);
            #(tq);
            got_ack = (sda === 1'b0);
            #(tq); scl_low = 1'b1;
        end
    endtask

    task stop;
        begin
            #(tq); sda_low = 1'b1;
            #(tq); scl_low = 1'b0;
            wait (scl === 1'b1);
            #(tq); sda_low = 1'b0;            // STOP
            #(2 * tq);
        end
    endtask

    task transaction;
        input [7:0] b;
        begin
            wait (scl === 1'b1 && sda === 1'b1);
            sda_low = 1'b1;                    // START
            #(tq); scl_low = 1'b1;
            send_byte(b);
            stop;
            done_count = done_count + 1;
        end
    endtask

    task join_start;
        input [7:0] b;
        begin
            @(negedge sda);                    // somebody's START
            sda_low = 1'b1;                    // ... and ours, same instant
            #(tq); scl_low = 1'b1;
            send_byte(b);
            stop;
            done_count = done_count + 1;
        end
    endtask
endmodule
