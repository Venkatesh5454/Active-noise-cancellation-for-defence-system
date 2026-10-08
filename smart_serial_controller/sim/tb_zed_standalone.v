// =============================================================================
// tb_zed_standalone.v  -  simulates the complete "Way 1" FPGA image
// -----------------------------------------------------------------------------
// This is the bitstream you will load on the ZedBoard, with fake Pmods:
//     JA  PC terminal (115200 8N1)       JB  PmodSF3 flash model
//     JC  PmodTMP2 model at 0x4B         (pull-ups on SCL/SDA)
// The testbench "types" keys and checks what the terminal prints:
//     banner, 't' -> "Temp = +25.0000 C", 'f' -> "Flash ID = 20 BA 19",
//     a negative temperature, an unplugged sensor, and the echo of other keys.
// The whole terminal session is printed at the end.
// =============================================================================
`timescale 1ns/1ps
module tb_zed_standalone;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg btn = 1'b1;                          // hold reset at start

    wire uart_txd, uart_rxd, uart_rts_n;
    wire spi_cs_n, spi_mosi, spi_miso, spi_sclk;
    wire spi_wp_n, spi_hold_n, spi_cs1_n, spi_cs2_n;
    wire i2c_scl, i2c_sda;
    wire [7:0] la, led;

    pullup (i2c_scl);
    pullup (i2c_sda);
    pullup (spi_miso);

    zed_top_standalone dut (
        .clk_100m(clk), .btn_reset(btn),
        .uart_cts_n(1'b0), .uart_txd(uart_txd), .uart_rxd(uart_rxd), .uart_rts_n(uart_rts_n),
        .spi_cs_n(spi_cs_n), .spi_mosi(spi_mosi), .spi_miso(spi_miso), .spi_sclk(spi_sclk),
        .spi_wp_n(spi_wp_n), .spi_hold_n(spi_hold_n), .spi_cs1_n(spi_cs1_n), .spi_cs2_n(spi_cs2_n),
        .i2c_scl(i2c_scl), .i2c_sda(i2c_sda),
        .la(la), .led(led)
    );

    tb_uart_term term  (.rx(uart_txd), .tx(uart_rxd));
    tb_spi_flash flash (.cs_n(spi_cs_n), .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso));
    tb_i2c_slave #(.ADDR(10'h04B), .TEN(0)) tmp2 (.scl(i2c_scl), .sda(i2c_sda));

    integer errors = 0, checks = 0;
    integer base, i, j, found;

    // wait until the terminal has printed the prompt "> " again
    task wait_prompt;
        input integer timeout_ms;
        integer tend;
        begin
            tend = $time + timeout_ms * 1000000;
            #(100000);
            while (!(term.rx_cnt >= base + 2 && term.rx_mem[term.rx_cnt - 2] == ">" &&
                     term.rx_mem[term.rx_cnt - 1] == " ") && $time < tend)
                #(10000);
            #(200000);
        end
    endtask

    // look for txt (len characters) in what the terminal received since base
    task expect_text;
        input [8*40-1:0] txt;
        input integer    len;
        begin
            found = 0;
            for (i = base; i + len <= term.rx_cnt && !found; i = i + 1) begin
                found = 1;
                for (j = 0; j < len; j = j + 1)
                    if (term.rx_mem[i + j] !== txt[8 * (len - 1 - j) +: 8]) found = 0;
            end
            checks = checks + 1;
            if (!found) begin
                errors = errors + 1;
                $display("    FAIL  terminal did not print \"%0s\"", txt);
            end else
                $display("    ok    terminal printed \"%0s\"", txt);
        end
    endtask

    task press;
        input [7:0] key;
        begin
            base = term.rx_cnt;
            term.send(key, 1'b0, 1'b0);
            wait_prompt(30);
        end
    endtask

    task show_session;
        begin
            $display("\n----------------- terminal session -----------------");
            for (i = 0; i < term.rx_cnt; i = i + 1)
                if (term.rx_mem[i] != 8'h0D) $write("%c", term.rx_mem[i]);
            $display("\n----------------------------------------------------");
        end
    endtask

    initial begin
        $display("==============================================================");
        $display(" Way 1 (FPGA only) - full ZedBoard image simulation");
        $display("==============================================================");
        base = 0;
        #(1000);
        btn = 1'b0;                                  // release BTNC

        wait_prompt(40);
        expect_text("Smart Serial Controller", 23);
        expect_text("t = temperature", 15);

        press("t");
        expect_text("Temp = +25.0000 C", 17);
        checks = checks + 1;
        if (led[6] !== 1'b0) begin errors = errors + 1; $display("    FAIL  error LED on"); end

        press("f");
        expect_text("Flash ID = 20 BA 19", 19);

        tmp2.regs[0] = 8'hF3; tmp2.regs[1] = 8'h80;  // 0xF380 >> 3 = -400 -> -25.0 C
        press("t");
        expect_text("Temp = -25.0000 C", 17);

        tmp2.regs[0] = 8'h01; tmp2.regs[1] = 8'h88;  // 0x0188 >> 3 = 49 -> 3.0625 C
        press("t");
        expect_text("Temp = +3.0625 C", 16);

        tmp2.present = 1'b0;                         // pull the Pmod out
        press("t");
        expect_text("No ACK from PmodTMP2", 20);
        checks = checks + 1;
        if (led[6] !== 1'b1) begin errors = errors + 1; $display("    FAIL  error LED not on"); end
        tmp2.present = 1'b1;

        base = term.rx_cnt;
        term.send("x", 1'b0, 1'b0);
        #(300000);
        expect_text("x", 1);

        show_session;
        $display("==============================================================");
        if (errors == 0) $display(" ALL %0d CHECKS PASSED", checks);
        else             $display(" %0d OF %0d CHECKS FAILED", errors, checks);
        $display("==============================================================");
        $finish;
    end

    initial begin
        #(300_000_000);
        $display("GLOBAL TIME-OUT");
        show_session;
        $finish;
    end
endmodule
