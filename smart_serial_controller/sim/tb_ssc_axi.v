// =============================================================================
// tb_ssc_axi.v  -  the controller as the ARM sees it in Way 2 (AXI4-Lite)
// -----------------------------------------------------------------------------
// Drives ssc_axi_top with real AXI4-Lite handshakes (write address and write
// data arriving in either order, back-to-back accesses) and repeats the
// register sequences that sw/ssc_driver.c uses: ID check, UART loop-back,
// SPI flash JEDEC ID, PmodTMP2 read with repeated START, interrupt line.
// =============================================================================
`timescale 1ns/1ps
module tb_ssc_axi;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg aresetn = 1'b0;

    reg  [11:0] awaddr = 0;  reg awvalid = 0;  wire awready;
    reg  [31:0] wdata  = 0;  reg wvalid  = 0;  wire wready;
    wire [1:0]  bresp;       wire bvalid;      reg bready = 0;
    reg  [11:0] araddr = 0;  reg arvalid = 0;  wire arready;
    wire [31:0] rdata;       wire [1:0] rresp; wire rvalid;  reg rready = 0;
    wire        irq;

    wire uart_txd, uart_rts_n;
    wire spi_sclk, spi_mosi, spi_miso;
    wire [3:0] spi_cs_n;
    wire spis_miso, spis_miso_oe, spi_slave_mode;
    wire scl, sda, scl_oe, sda_oe;
    pullup (scl);
    pullup (sda);
    pullup (spi_miso);
    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;

    ssc_axi_top dut (
        .s_axi_aclk(clk), .s_axi_aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .irq(irq),
        .uart_rxd(1'b1), .uart_txd(uart_txd), .uart_cts_n(1'b0), .uart_rts_n(uart_rts_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso), .spi_cs_n(spi_cs_n),
        .spis_sclk(1'b0), .spis_mosi(1'b0), .spis_cs_n(1'b1),
        .spis_miso(spis_miso), .spis_miso_oe(spis_miso_oe), .spi_slave_mode(spi_slave_mode),
        .i2c_scl_in(scl), .i2c_scl_oe(scl_oe), .i2c_sda_in(sda), .i2c_sda_oe(sda_oe)
    );

    tb_spi_flash flash (.cs_n(spi_cs_n[0]), .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso));
    tb_i2c_slave #(.ADDR(10'h04B), .TEN(0)) tmp2 (.scl(scl), .sda(sda));

    integer errors = 0, checks = 0, i;
    integer wskew = 0;                 // >0: W arrives later than AW, <0: earlier
    reg [31:0] r, b0, b1;

    task check_eq;
        input [31:0] got, exp;
        input [8*60-1:0] what;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("    FAIL  %0s: got 0x%0h expected 0x%0h", what, got, exp);
            end else
                $display("    ok    %0s = 0x%0h", what, got);
        end
    endtask

    // ---------------- AXI4-Lite master ----------------
    task axi_wr;
        input [11:0] a;
        input [31:0] d;
        begin
            fork
                begin                                           // write address
                    if (wskew < 0) repeat (-wskew) @(posedge clk);
                    @(posedge clk); awaddr <= a; awvalid <= 1'b1;
                    @(posedge clk); while (!awready) @(posedge clk);
                    awvalid <= 1'b0;
                end
                begin                                           // write data
                    if (wskew > 0) repeat (wskew) @(posedge clk);
                    @(posedge clk); wdata <= d; wvalid <= 1'b1;
                    @(posedge clk); while (!wready) @(posedge clk);
                    wvalid <= 1'b0;
                end
            join
            bready <= 1'b1;
            @(posedge clk); while (!bvalid) @(posedge clk);
            bready <= 1'b0;
            if (bresp !== 2'b00) $display("    FAIL  BRESP = %b", bresp);
        end
    endtask

    task axi_rd;
        input  [11:0] a;
        output [31:0] d;
        begin
            @(posedge clk); araddr <= a; arvalid <= 1'b1;
            @(posedge clk); while (!arready) @(posedge clk);
            arvalid <= 1'b0;
            rready  <= 1'b1;
            @(posedge clk); while (!rvalid) @(posedge clk);
            d = rdata;
            rready <= 1'b0;
        end
    endtask

    task wait_done;                     // like ssc_wait_events(SSC_INT_I2C_DONE)
        begin
            axi_rd(12'h004, r);
            while (!r[9]) axi_rd(12'h004, r);
            axi_wr(12'h004, 32'h0000_0200);
        end
    endtask

    initial begin
        $display("==============================================================");
        $display(" Way 2 hardware path: AXI4-Lite -> APB bridge -> controller");
        $display("==============================================================");
        repeat (5) @(posedge clk);
        aresetn = 1'b1;

        axi_rd(12'h000, r);  check_eq(r, 32'h5353_4301, "ID over AXI");
        wskew = 3;   axi_wr(12'h014, 32'h1111_2222);   // data arrives late
        axi_rd(12'h014, r);  check_eq(r, 32'h1111_2222, "scratch, W after AW");
        wskew = -3;  axi_wr(12'h014, 32'h3333_4444);   // data arrives first
        axi_rd(12'h014, r);  check_eq(r, 32'h3333_4444, "scratch, W before AW");
        wskew = 0;

        // ssc_uart_init(921600) + LOOP, then putc/getc
        axi_wr(12'h104, {12'd0, 4'd13, 16'd6});         // 6 + 13/16
        axi_wr(12'h100, 32'h0003_010F);                 // TX, RX, 8N1, LOOP, flush
        b0 = 0;
        for (i = 0; i < 16; i = i + 1) begin
            axi_wr(12'h10C, i * 37 + 11);
            axi_rd(12'h110, r);
            while (r[31]) axi_rd(12'h110, r);
            if (r[7:0] === ((i * 37 + 11) & 255)) b0 = b0 + 1;
        end
        check_eq(b0, 16, "UART loop-back bytes correct");

        // ssc_spi_init(1 MHz, mode 0) + flash_read_id
        axi_wr(12'h204, 32'd49);
        axi_wr(12'h200, 32'h0003_0871);                 // EN, 8 bit, CS manual, flush
        axi_wr(12'h200, 32'h0000_1871);                 // CS0 low
        axi_wr(12'h20C, 32'h9F); axi_wr(12'h20C, 0); axi_wr(12'h20C, 0); axi_wr(12'h20C, 0);
        axi_rd(12'h208, r);
        while (r[20:16] < 4 || r[4]) axi_rd(12'h208, r);
        axi_wr(12'h200, 32'h0000_0871);                 // CS0 high
        axi_rd(12'h210, r);
        axi_rd(12'h210, r); b0 = r;
        axi_rd(12'h210, r); b1 = r;
        axi_rd(12'h210, r);
        check_eq({b0[7:0], b1[7:0], r[7:0]}, 24'h20BA19, "flash JEDEC ID");

        // tmp2_read: write pointer (no STOP), read 2 with STOP; IRQ on I2C_DONE
        axi_wr(12'h008, 32'h0000_0200);                 // INT_ENABLE = I2C_DONE
        axi_wr(12'h304, 32'd62);                        // 400 kHz
        axi_wr(12'h300, 32'h0003_0001);
        axi_wr(12'h308, 32'h4B);
        axi_wr(12'h314, 32'h00);
        axi_wr(12'h30C, 32'h0000_0001);
        wait (irq === 1'b1);
        checks = checks + 1;
        $display("    ok    IRQ line raised by I2C_DONE");
        wait_done;
        #(100);
        checks = checks + 1;
        if (irq !== 1'b0) begin errors = errors + 1; $display("    FAIL  IRQ still high"); end
        else $display("    ok    IRQ line cleared by write-1-to-clear");
        axi_wr(12'h30C, 32'h0000_0302);
        wait_done;
        axi_rd(12'h318, b0);
        axi_rd(12'h318, b1);
        check_eq({b0[7:0], b1[7:0]}, 16'h0C80, "PmodTMP2 bytes (25.0 C)");
        axi_rd(12'h310, r);
        check_eq(r[7:4], 4'b0000, "I2C idle, no NACK, no ARB_LOST");

        $display("==============================================================");
        if (errors == 0) $display(" ALL %0d CHECKS PASSED", checks);
        else             $display(" %0d OF %0d CHECKS FAILED", errors, checks);
        $display("==============================================================");
        $finish;
    end

    initial begin
        #(50_000_000);
        $display("GLOBAL TIME-OUT");
        $finish;
    end
endmodule
