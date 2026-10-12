// =============================================================================
// tb_spi_flash.v  -  behavioural model of the PmodSF3 serial NOR flash
// -----------------------------------------------------------------------------
// Micron N25Q256 / MT25QL256 style, SPI mode 0 or 3, 3-byte addresses.
// Supported commands:
//   0x9F READ ID        -> 0x20 0xBA 0x19 (Micron, 3 V, 256 Mbit)
//   0x05 READ STATUS    -> bit0 WIP (write in progress), bit1 WEL
//   0x06 / 0x04         write enable / disable
//   0x20 SUBSECTOR ERASE 4 KB (sets bytes to 0xFF, takes 100 us)
//   0x02 PAGE PROGRAM   (can only clear bits, takes 50 us)
//   0x03 READ           continuous read
// Only 4 KB of memory is modelled (address bits 11:0).
// =============================================================================
`timescale 1ns/1ps
module tb_spi_flash (
    input  wire cs_n,
    input  wire sclk,
    input  wire mosi,
    output wire miso
);
    reg [7:0]  mem [0:4095];
    reg        wel = 1'b0;
    reg        wip = 1'b0;
    reg [7:0]  cmd;
    reg [7:0]  in_byte;
    reg [7:0]  out_byte;
    reg [23:0] addr;
    integer    nbit, nbyte, obit, id_idx;
    reg [7:0]  pp_buf [0:255];
    integer    pp_n;
    reg        drive  = 1'b0;
    reg        miso_r = 1'b0;
    integer    busy_ns;
    integer    k;
    event      start_busy;

    assign miso = drive ? miso_r : 1'bz;

    initial for (k = 0; k < 4096; k = k + 1) mem[k] = 8'hFF;

    always @(start_busy) begin
        wip = 1'b1;
        #(busy_ns);
        wip = 1'b0;
        wel = 1'b0;
    end

    always @(negedge cs_n) begin
        nbit     = 0;
        nbyte    = 0;
        obit     = 8;
        out_byte = 8'h00;
        pp_n     = 0;
        cmd      = 8'h00;
        drive    = 1'b1;
    end

    always @(posedge cs_n) begin
        drive = 1'b0;
        if (nbyte >= 1 && !wip) begin
            case (cmd)
                8'h06: wel = 1'b1;
                8'h04: wel = 1'b0;
                8'h20: if (wel && nbyte >= 4) begin
                           for (k = 0; k < 4096; k = k + 1) mem[k] = 8'hFF;
                           busy_ns = 100000;
                           -> start_busy;
                       end
                8'h02: if (wel && nbyte >= 4) begin
                           for (k = 0; k < pp_n; k = k + 1)
                               mem[{addr[11:8], addr[7:0] + k[7:0]}] =
                                   mem[{addr[11:8], addr[7:0] + k[7:0]}] & pp_buf[k];
                           busy_ns = 50000;
                           -> start_busy;
                       end
                default: ;
            endcase
        end
    end

    task byte_done;
        input [7:0] b;
        begin
            if (nbyte == 0) begin
                cmd = b;
                case (b)
                    8'h9F: begin out_byte = 8'h20; id_idx = 1; end
                    8'h05: out_byte = {6'd0, wel, wip};
                    default: out_byte = 8'h00;
                endcase
            end else begin
                case (cmd)
                    8'h9F: begin
                        out_byte = (id_idx == 1) ? 8'hBA : (id_idx == 2) ? 8'h19 : 8'h00;
                        id_idx   = id_idx + 1;
                    end
                    8'h05: out_byte = {6'd0, wel, wip};
                    8'h03: begin
                        if (nbyte <= 3) addr = {addr[15:0], b};
                        if (nbyte >= 3) begin
                            out_byte = mem[addr[11:0]];
                            addr     = addr + 24'd1;
                        end
                    end
                    8'h02: begin
                        if (nbyte <= 3) addr = {addr[15:0], b};
                        else begin
                            pp_buf[pp_n] = b;
                            pp_n = pp_n + 1;
                        end
                    end
                    8'h20: if (nbyte <= 3) addr = {addr[15:0], b};
                    default: out_byte = 8'h00;
                endcase
            end
        end
    endtask

    always @(posedge sclk) if (cs_n === 1'b0) begin
        in_byte = {in_byte[6:0], mosi};
        nbit    = nbit + 1;
        if (nbit == 8) begin
            nbit = 0;
            byte_done(in_byte);
            nbyte = nbyte + 1;
            obit  = 0;                 // next falling edge sends bit 7
        end
    end

    always @(negedge sclk) if (cs_n === 1'b0) begin
        if (obit < 8) begin
            miso_r = out_byte[7 - obit];
            obit   = obit + 1;
        end
    end
endmodule
