// =============================================================================
// tb_spi_slave.v  -  generic SPI slave: any mode, 4..32 bits, MSB or LSB first
// -----------------------------------------------------------------------------
// Records every word it receives in got[] and answers with resp (resp is
// increased by resp_inc after every word).  Settings are written by the
// testbench: gslv.cpol = 1; gslv.nbits = 16; ...
// =============================================================================
`timescale 1ns/1ps
module tb_spi_slave (
    input  wire cs_n,
    input  wire sclk,
    input  wire mosi,
    output wire miso
);
    reg        cpol     = 1'b0;
    reg        cpha     = 1'b0;
    reg        lsb      = 1'b0;
    integer    nbits    = 8;
    reg [31:0] resp     = 32'h0000_00A5;
    reg [31:0] resp_inc = 32'd0;

    reg [31:0] got [0:1023];
    integer    ngot = 0;
    reg [31:0] rx_word;
    integer    krx, ktx;
    reg        miso_r = 1'b0;

    assign miso = (cs_n === 1'b0) ? miso_r : 1'bz;

    function integer idx;              // wire position of the k-th bit
        input integer k;
        idx = lsb ? k : (nbits - 1 - k);
    endfunction

    task sample;
        begin
            rx_word[idx(krx)] = mosi;
            krx = krx + 1;
            if (krx == nbits) begin
                got[ngot] = rx_word;
                ngot      = ngot + 1;
                krx       = 0;
                rx_word   = 32'd0;
                resp      = resp + resp_inc;
            end
        end
    endtask

    task change_cpha0;                 // CPHA=0: next bit on the trailing edge
        begin
            ktx = ktx + 1;
            if (ktx == nbits) ktx = 0;
            miso_r = resp[idx(ktx)];
        end
    endtask

    task change_cpha1;                 // CPHA=1: bit on the leading edge
        begin
            miso_r = resp[idx(ktx)];
            ktx = ktx + 1;
            if (ktx == nbits) ktx = 0;
        end
    endtask

    always @(negedge cs_n) begin
        krx     = 0;
        ktx     = 0;
        rx_word = 32'd0;
        if (!cpha) miso_r = resp[idx(0)];
    end

    always @(posedge sclk) if (cs_n === 1'b0) begin
        if (!cpol) begin if (!cpha) sample; else change_cpha1; end   // leading
        else       begin if (!cpha) change_cpha0; else sample; end   // trailing
    end

    always @(negedge sclk) if (cs_n === 1'b0) begin
        if (!cpol) begin if (!cpha) change_cpha0; else sample; end   // trailing
        else       begin if (!cpha) sample; else change_cpha1; end   // leading
    end
endmodule
