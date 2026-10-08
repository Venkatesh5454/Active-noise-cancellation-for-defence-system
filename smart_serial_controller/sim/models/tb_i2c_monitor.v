// =============================================================================
// tb_i2c_monitor.v  -  listens to the I2C bus and logs what happened
// -----------------------------------------------------------------------------
// ev[] codes:  1000 = START, 1001 = repeated START, 1002 = STOP,
//              0..511 = byte, bit 8 set when the 9th bit was NACK
// show() prints the log like "S 96 A 00 A Sr 97 A 0C A 80 N P".
// =============================================================================
`timescale 1ns/1ps
module tb_i2c_monitor (
    input wire scl,
    input wire sda
);
    integer   ev [0:1023];
    integer   nev = 0;
    reg       in_xfer = 1'b0;
    integer   bitc = 0;
    reg [8:0] sh;

    always @(negedge sda) if (scl === 1'b1) begin
        ev[nev] = in_xfer ? 1001 : 1000;
        nev     = nev + 1;
        in_xfer = 1'b1;
        bitc    = 0;
    end

    always @(posedge sda) if (scl === 1'b1) begin
        ev[nev] = 1002;
        nev     = nev + 1;
        in_xfer = 1'b0;
    end

    always @(posedge scl) if (in_xfer) begin
        sh   = {sh[7:0], (sda === 1'b0) ? 1'b0 : 1'b1};
        bitc = bitc + 1;
        if (bitc == 9) begin
            ev[nev] = {23'd0, sh[0], sh[8:1]};
            nev     = nev + 1;
            bitc    = 0;
        end
    end

    task clear;
        begin
            nev  = 0;
            bitc = 0;
        end
    endtask

    task show;
        integer i;
        reg [7:0] b;
        begin
            $write("        bus:");
            for (i = 0; i < nev; i = i + 1) begin
                if (ev[i] == 1000)      $write(" S");
                else if (ev[i] == 1001) $write(" Sr");
                else if (ev[i] == 1002) $write(" P");
                else begin
                    b = ev[i] & 255;
                    $write(" %h %s", b, (ev[i] > 255) ? "N" : "A");
                end
            end
            $write("\n");
        end
    endtask
endmodule
