// =============================================================================
// tb_zed2_gate.v  -  board-level test that also works on the gate netlist
// -----------------------------------------------------------------------------
// The other testbenches look inside the design (hierarchical names) and
// shorten time with parameters.  A synthesised or implemented netlist has
// neither: it is just LUTs and flip-flops with the top-level pins.  So this
// testbench uses ONLY the pins of zed2_top_standalone, at real speed, and
// can run as
//     Behavioral Simulation                 (the Verilog)
//     Post-Synthesis Functional / Timing    (after Run Synthesis)
//     Post-Implementation Functional / Timing  (after Run Implementation)
// in Vivado with zed2_top_standalone as the synthesis top.  It runs 2 ms of
// board time (a few minutes behavioural, longer on a netlist; timing
// simulation is the slowest).
//
// Set-up, as on the bench: PC terminal on JA, SPI flash on JB, PmodTMP2 on
// JC, switches SW7 = 1 (UART NI in ADDRESSED mode) and SW2..0 = 010 (JD
// shows a copy of the SPI pins).
//   [1] PC sends 04 00 01 9F 03 (to node 4 = SPI, CS0, write 9F, read 3)
//       -> must receive 20 BA 19: the flash ID through the network
//   [2] JD1 (CS#) went low and JD4 (SCK) toggled: the crossbar copy works
//   [3] PC sends 05 4B 01 00 02 (to node 5 = I2C, address 0x4B, write 00,
//       read 2) -> must receive 0C 80: the temperature through the network
//   [4] JD7..10 probe copies moved (UART TXD, I2C SCL)
//   [5] fixed outputs: JB7..10 high, OLED supplies off, LD7 (IRQ) off
// =============================================================================
`timescale 1ns/1ps
module tb_zed2_gate;
    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz, like GCLK Y9
    reg       btn_reset = 1'b1;
    reg [7:0] sw = 8'h82;                       // SW7 = 1, SW2..0 = 2 (SPI)

    integer errors = 0, checks = 0;

    wire [3:0] ja, jb, jc, jd, jb_lo, jd_lo, oled;
    wire       oled_vdd, oled_vbat;
    wire [7:0] led;

    zed2_top_standalone dut (
        .clk_100m(clk), .btn_reset(btn_reset), .btn(4'b0000), .sw(sw),
        .ja(ja), .jb(jb), .jb_lo(jb_lo), .jc(jc), .jd(jd), .jd_lo(jd_lo),
        .oled(oled), .oled_vdd(oled_vdd), .oled_vbat(oled_vbat), .led(led)
    );

    // the pull-ups of zed2_pins.xdc (a netlist does not contain them)
    pullup (ja[0]); pullup (ja[1]); pullup (ja[2]); pullup (ja[3]);
    pullup (jb[0]); pullup (jb[1]); pullup (jb[2]); pullup (jb[3]);
    pullup (jc[0]); pullup (jc[1]); pullup (jc[2]); pullup (jc[3]);
    pullup (jd[0]); pullup (jd[1]); pullup (jd[2]); pullup (jd[3]);

    // devices on the Pmods
    wire pc_tx;
    tb_uart_term pc (.rx(ja[1]), .tx(pc_tx));         // JA2 = our TXD, JA3 = our RXD
    assign ja[2] = pc_tx;
    tb_spi_flash flash (.cs_n(jb[0]), .sclk(jb[3]), .mosi(jb[1]), .miso(jb[2]));
    tb_i2c_slave #(.ADDR(10'h04B), .TEN(0)) tmp2 (.scl(jc[2]), .sda(jc[3]));

    // edge counters on the copies (JD1 = CS#, JD4 = SCK, JD7 = TXD, JD9 = SCL)
    integer cs_low = 0, sck_edges = 0, txd_edges = 0, scl_edges = 0;
    always @(negedge jd[0])    cs_low    = cs_low + 1;
    always @(posedge jd[3])    sck_edges = sck_edges + 1;
    always @(negedge jd_lo[0]) txd_edges = txd_edges + 1;
    always @(negedge jd_lo[2]) scl_edges = scl_edges + 1;

    task check;
        input cond;
        input [8*72-1:0] what;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("    FAIL  %0s (t=%0t)", what, $time);
            end else
                $display("    ok    %0s", what);
        end
    endtask

    // wait until the PC has n bytes in total, at most max_us microseconds
    task pc_wait;
        input integer n, max_us;
        integer t;
        begin
            t = 0;
            while (pc.rx_cnt < n && t < max_us) begin #1000; t = t + 1; end
        end
    endtask

    integer n0;
    initial begin
        $display("==============================================================");
        $display(" Way 1 image, pins only (works on the post-synthesis and");
        $display(" post-implementation netlists too)");
        $display("==============================================================");
        #1000 btn_reset = 1'b0;                 // longer than the 100 ns GSR of a netlist
        #100000;                                // boot list + first loop pass (~3 us)

        $display("[1] flash ID through the NoC: 04 00 01 9F 03");
        n0 = pc.rx_cnt;
        pc.send(8'h04, 0, 0); pc.send(8'h00, 0, 0); pc.send(8'h01, 0, 0);
        pc.send(8'h9F, 0, 0); pc.send(8'h03, 0, 0);
        pc_wait(n0 + 3, 3000);
        check(pc.rx_cnt == n0 + 3, "PC received 3 bytes");
        check(pc.rx_mem[n0] == 8'h20 && pc.rx_mem[n0+1] == 8'hBA && pc.rx_mem[n0+2] == 8'h19,
              "they are 20 BA 19");

        $display("[2] crossbar: JD shows a copy of the SPI pins (SW2..0 = 2)");
        check(cs_low >= 1, "JD1 (CS#) went low");
        check(sck_edges >= 32, "JD4 (SCK) clocked at least 4 bytes");

        $display("[3] temperature through the NoC: 05 4B 01 00 02");
        n0 = pc.rx_cnt;
        pc.send(8'h05, 0, 0); pc.send(8'h4B, 0, 0); pc.send(8'h01, 0, 0);
        pc.send(8'h00, 0, 0); pc.send(8'h02, 0, 0);
        pc_wait(n0 + 2, 4000);
        check(pc.rx_cnt == n0 + 2, "PC received 2 bytes");
        check(pc.rx_mem[n0] == 8'h0C && pc.rx_mem[n0+1] == 8'h80, "they are 0C 80 (25.00 C)");

        $display("[4] probe copies on JD7..10");
        check(txd_edges >= 5, "JD7 (UART TXD copy) moved");
        check(scl_edges >= 18, "JD9 (I2C SCL copy) moved");

        $display("[5] fixed outputs");
        check(jb_lo === 4'b1111, "JB7..10 (CS1#, RST#, WP#, HOLD#) high");
        check(oled_vdd === 1'b1 && oled_vbat === 1'b1, "OLED supplies off");
        check(led[7] === 1'b0, "LD7 (interrupt line) off");

        $display("==============================================================");
        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $display("==============================================================");
        $finish;
    end

    initial begin
        #20_000_000;
        $display("FAIL: watchdog (20 ms)");
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule
