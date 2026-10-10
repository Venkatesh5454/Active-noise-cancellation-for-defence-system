// =============================================================================
// tb_zed2_standalone.v  -  the complete Way 1 ZedBoard image (no ARM)
// -----------------------------------------------------------------------------
// zed2_top_standalone with the same devices as on the bench:
//     JA : PC terminal   JB : SPI flash   JC : PmodTMP2   JD : logic analyser
// ssc2_boot sets everything up by itself, then:
//   [1] the boot list finishes and the loop runs
//   [2] PC sends 01 9F 03 -> 20 BA 19 (slide 17, FRAME mode)
//   [3] the hub's TMP2 records land in the block RAM (slide 18)
//   [4] LD0..3 show the record count / 4
//   [5] SW2..0 = 4: JD switches to SM0, which sends the last value 0C 80
//       as UART bytes (hub task 1 -> serial engine -> crossbar -> JD2)
//   [6] SW7 = 1: ADDRESSED frames; 05 4B 01 00 02 reads the TMP2 -> 0C 80
// Time is compressed: 1 "ms" = 50 us (core and boot sequencer).
// =============================================================================
`timescale 1ns/1ps
module tb_zed2_standalone;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg btn_reset = 1'b1;
    reg [7:0] sw = 8'h00;

    integer errors = 0, checks = 0;

    wire [3:0] ja, jb, jc, jd, jb_lo, jd_lo, oled;
    wire       oled_vdd, oled_vbat;
    wire [7:0] led;

    zed2_top_standalone dut (
        .clk_100m(clk), .btn_reset(btn_reset), .btn(4'b0000), .sw(sw),
        .ja(ja), .jb(jb), .jb_lo(jb_lo), .jc(jc), .jd(jd), .jd_lo(jd_lo),
        .oled(oled), .oled_vdd(oled_vdd), .oled_vbat(oled_vbat), .led(led)
    );
    defparam dut.u_core.US_PER_MS = 50;           // 1 "ms" = 50 us
    defparam dut.u_boot.CLK_HZ    = 5_000_000;    // boot WAIT: 1 "ms" = 5000 clocks

    pullup (ja[0]); pullup (ja[1]); pullup (ja[2]); pullup (ja[3]);
    pullup (jb[0]); pullup (jb[1]); pullup (jb[2]); pullup (jb[3]);
    pullup (jc[0]); pullup (jc[1]); pullup (jc[2]); pullup (jc[3]);
    pullup (jd[0]); pullup (jd[1]); pullup (jd[2]); pullup (jd[3]);

    wire pc_tx, la_tx_unused;
    tb_uart_term pc (.rx(ja[1]), .tx(pc_tx));
    assign ja[2] = pc_tx;
    tb_spi_flash flash (.cs_n(jb[0]), .sclk(jb[3]), .mosi(jb[1]), .miso(jb[2]));
    tb_i2c_slave #(.ADDR(10'h04B), .TEN(0)) tmp2 (.scl(jc[2]), .sda(jc[3]));
    tb_uart_term la (.rx(jd[1]), .tx(la_tx_unused));

    task check;
        input cond;
        input [8*72-1:0] what;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("    FAIL  %0s", what);
            end else
                $display("    ok    %0s", what);
        end
    endtask

    task wait_us;
        input integer us;
        begin
            #(us * 1000);
        end
    endtask

    // record i, word w from the block RAM
    function [31:0] rec_word;
        input integer i, w;
        begin
            rec_word = dut.u_mem.mem[i*4 + w];
        end
    endfunction

    integer i, k, n0, good;
    reg [15:0] seq;

    initial begin
        $display("==============================================================");
        $display(" Way 1 (FPGA only) ZedBoard image");
        $display("==============================================================");
        #200 btn_reset = 1'b0;

        $display("[1] boot sequencer");
        k = 0;
        while (!dut.u_boot.running && k < 2000) begin #1000; k = k + 1; end
        check(dut.u_boot.running, "set-up list done, loop running");

        $display("[2] PC reads the flash ID through the NoC (FRAME mode)");
        n0 = pc.rx_cnt;
        pc.send(8'h01, 0, 0); pc.send(8'h9F, 0, 0); pc.send(8'h03, 0, 0);
        k = 0; while (pc.rx_cnt < n0 + 3 && k < 3000) begin #1000; k = k + 1; end
        check(pc.rx_cnt == n0 + 3 && pc.rx_mem[n0] == 8'h20 && pc.rx_mem[n0+1] == 8'hBA &&
              pc.rx_mem[n0+2] == 8'h19, "PC got 20 BA 19");

        $display("[3] hub records in the block RAM");
        // 4 readings at 100 "ms" = 5 ms each
        wait_us(22000);
        good = 0;
        for (i = 0; i < 4; i = i + 1) begin
            seq = i + 1;
            if (rec_word(i, 1) == {seq, 8'd2, 8'd3} && rec_word(i, 2) == 32'h0000_800C &&
                rec_word(i, 3) == 32'd0)
                good = good + 1;
            else
                $display("    record %0d: %h %h %h %h", i, rec_word(i, 0), rec_word(i, 1),
                         rec_word(i, 2), rec_word(i, 3));
        end
        check(good == 4, "4 records: source 3, len 2, seq 1..4, data 0C 80");
        check(rec_word(1, 0) - rec_word(0, 0) >= 4500 && rec_word(1, 0) - rec_word(0, 0) <= 5500,
              "timestamps 100 \"ms\" (5000 us) apart");

        $display("[4] LEDs");
        check(led[0] === 1'b1, "LD0 = record count / 4 bit 0");

        $display("[5] SW = 4: JD driven by SM0, hub task 1 sends the last value");
        sw = 8'h04;
        n0 = la.rx_cnt;
        k = 0; while (la.rx_cnt < n0 + 2 && k < 12000) begin #1000; k = k + 1; end
        check(la.rx_cnt >= n0 + 2 && la.rx_mem[n0] == 8'h0C && la.rx_mem[n0+1] == 8'h80,
              "JD2 shows the temperature bytes 0C 80 as UART frames");

        $display("[6] SW7 = 1: ADDRESSED frame to the I2C node");
        sw = 8'h84;
        wait_us(1000);                                // loop copies SW7 within 10 "ms"
        n0 = pc.rx_cnt;
        pc.send(8'h05, 0, 0); pc.send(8'h4B, 0, 0); pc.send(8'h01, 0, 0);
        pc.send(8'h00, 0, 0); pc.send(8'h02, 0, 0);
        k = 0; while (pc.rx_cnt < n0 + 2 && k < 5000) begin #1000; k = k + 1; end
        check(pc.rx_cnt == n0 + 2 && pc.rx_mem[n0] == 8'h0C && pc.rx_mem[n0+1] == 8'h80,
              "PC got the temperature 0C 80");

        $display("==============================================================");
        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $display("==============================================================");
        $finish;
    end

    initial begin
        #200_000_000;
        $display("FAIL: watchdog");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule
