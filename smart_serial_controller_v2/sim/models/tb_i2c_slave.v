// =============================================================================
// tb_i2c_slave.v  -  behavioural I2C slave with 16 registers
// -----------------------------------------------------------------------------
// With the default parameters it behaves like the PmodTMP2 (ADT7420) at 0x4B:
//   write: first data byte = register pointer, next bytes go to regs[ptr++]
//   read : returns regs[ptr], regs[ptr+1], ... until the master NACKs
//   regs[0], regs[1] = 0x0C, 0x80 -> 25.0 C
// TEN = 1 makes it a 10-bit-address device (ADDR is then 10 bits).
// stretch_ns > 0 makes it hold SCL low after every ACK (clock stretching).
// =============================================================================
`timescale 1ns/1ps
module tb_i2c_slave #(
    parameter [9:0] ADDR = 10'h04B,
    parameter       TEN  = 0
) (
    inout wire scl,
    inout wire sda
);
    reg sda_low = 1'b0;
    reg scl_low = 1'b0;
    assign sda = sda_low ? 1'b0 : 1'bz;
    assign scl = scl_low ? 1'b0 : 1'bz;

    reg [7:0] regs [0:15];
    reg [3:0] ptr = 4'd0;
    integer   stretch_ns    = 0;
    integer   stretch_count = 0;
    reg [7:0] wlog [0:255];          // every data byte the master wrote
    integer   nwlog = 0;
    integer   nread = 0;             // bytes sent to the master
    reg       present = 1'b1;        // 0 = "unplugged": never answers

    localparam IDLE = 0, ABYTE = 1, A2BYTE = 2, WBYTE = 3, RBYTE = 4,
               ACK_DRV = 5, ACK_HOLD = 6, ACK_REL = 7,
               MACK_REL = 8, MACK_WAIT = 9, MACK_NEXT = 10;
    localparam N_A2 = 0, N_WR = 1, N_RD = 2;

    integer   st = IDLE;
    integer   bitc, obit, nxt;
    reg [7:0] sh, tx;
    reg       first_data;
    reg       addressed10 = 1'b0;
    integer   k;

    initial begin
        regs[0] = 8'h0C;  regs[1] = 8'h80;  regs[2] = 8'h00;  regs[3] = 8'h00;
        for (k = 4; k < 16; k = k + 1) regs[k] = 8'h40 + k;
        regs[11] = 8'hCB;                 // ADT7420 ID register
    end

    task do_stretch;
        begin
            if (stretch_ns > 0) begin
                scl_low = 1'b1;
                #(stretch_ns);
                scl_low = 1'b0;
                stretch_count = stretch_count + 1;
            end
        end
    endtask

    // START or repeated START
    always @(negedge sda) if (scl === 1'b1) begin
        st      = ABYTE;
        bitc    = 0;
        sda_low = 1'b0;
    end

    // STOP
    always @(posedge sda) if (scl === 1'b1) begin
        st          = IDLE;
        sda_low     = 1'b0;
        addressed10 = 1'b0;
    end

    always @(posedge scl) begin
        case (st)
            ABYTE: begin
                sh   = {sh[6:0], sda};
                bitc = bitc + 1;
                if (bitc == 8) begin
                    if (!present) st = IDLE;
                    else if (!TEN) begin
                        if (sh[7:1] == ADDR[6:0]) begin
                            nxt = sh[0] ? N_RD : N_WR;
                            first_data = 1'b1;
                            st  = ACK_DRV;
                        end else st = IDLE;
                    end else begin
                        if (sh[7:3] == 5'b11110 && sh[2:1] == ADDR[9:8]) begin
                            if (!sh[0])           begin nxt = N_A2; st = ACK_DRV; end
                            else if (addressed10) begin nxt = N_RD; st = ACK_DRV; end
                            else st = IDLE;
                        end else st = IDLE;
                    end
                end
            end
            A2BYTE: begin
                sh   = {sh[6:0], sda};
                bitc = bitc + 1;
                if (bitc == 8) begin
                    if (sh == ADDR[7:0]) begin
                        addressed10 = 1'b1;
                        first_data  = 1'b1;
                        nxt = N_WR;
                        st  = ACK_DRV;
                    end else st = IDLE;
                end
            end
            WBYTE: begin
                sh   = {sh[6:0], sda};
                bitc = bitc + 1;
                if (bitc == 8) begin
                    wlog[nwlog] = sh;
                    nwlog = nwlog + 1;
                    if (first_data) begin
                        ptr = sh[3:0];
                        first_data = 1'b0;
                    end else begin
                        regs[ptr] = sh;
                        ptr = ptr + 4'd1;
                    end
                    nxt = N_WR;
                    st  = ACK_DRV;
                end
            end
            ACK_HOLD:  st = ACK_REL;
            RBYTE: begin
                bitc = bitc + 1;
                if (bitc == 8) st = MACK_REL;
            end
            MACK_WAIT: st = (sda === 1'b0) ? MACK_NEXT : IDLE;   // NACK: done
            default: ;
        endcase
    end

    always @(negedge scl) begin
        case (st)
            ACK_DRV: begin
                sda_low = 1'b1;               // ACK
                st = ACK_HOLD;
                do_stretch;
            end
            ACK_REL: begin
                if (nxt == N_RD) begin
                    tx = regs[ptr];
                    sda_low = ~tx[7];
                    obit = 6;
                    bitc = 0;
                    nread = nread + 1;
                    st = RBYTE;
                end else begin
                    sda_low = 1'b0;
                    bitc = 0;
                    st = (nxt == N_A2) ? A2BYTE : WBYTE;
                end
            end
            RBYTE: if (obit >= 0) begin
                sda_low = ~tx[obit];
                obit = obit - 1;
            end
            MACK_REL: begin
                sda_low = 1'b0;               // let the master answer
                st = MACK_WAIT;
            end
            MACK_NEXT: begin
                ptr = ptr + 4'd1;
                tx = regs[ptr];
                sda_low = ~tx[7];
                obit = 6;
                bitc = 0;
                nread = nread + 1;
                st = RBYTE;
            end
            default: ;
        endcase
    end
endmodule
