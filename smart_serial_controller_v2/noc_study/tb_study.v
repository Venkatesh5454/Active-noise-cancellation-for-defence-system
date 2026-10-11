// =============================================================================
// tb_study.v  -  NoC scalability study: traffic test for bus, crossbar, mesh
// -----------------------------------------------------------------------------
// Drives one fabric (study_bus, study_xbar or study_mesh) with N nodes with
// uniform random traffic and measures latency and throughput.  Every fabric
// has the same node interface (tx_* / rx_*, 34-bit flits, credits, DEPTH = 4
// buffers), so all of them see exactly the same traffic.
//
// Parameters (iverilog -P tb_study.FABRIC=2 -P tb_study.N=16 ...):
//   FABRIC  0 bus, 1 crossbar, 2 mesh, 3 all three side by side (default)
//   N       number of nodes, 2 .. 16 (default 4)
//
// Run modes (vvp plusargs):
//   (none)        quick sanity run (the unit test): rate 0.1 with ideal
//                 sinks, then rate 1.0 with 20 % urgent packets and sinks
//                 that stall 30 % of the time
//   +sweep        the study: rates 0.02 0.05 0.1 0.2 0.3 0.5 0.7 1.0
//   +rate=R       one rate, R in flits/node/clock x 1000 (e.g. +rate=100)
//   +warm=C +win=C +seed=S   warm-up clocks, measurement window, seed
//
// Traffic (the study_traffic module below, one per fabric):
//   * Each clock every node makes a new packet with probability rate/4
//     (packets are 1 HEAD + 2 BODY + 1 TAIL = 4 flits), so the offered load
//     is "rate" flits per node per clock.  The destination is uniform random
//     among the other N-1 nodes.
//   * New packets wait in a source queue (up to QCAP packets per node), so
//     a busy network cannot lower the offered load.  The source sends the
//     front flit whenever it has a credit (one flit per clock at most).
//   * Every node has a DEPTH-flit receive buffer (modelled as a counter): it
//     takes one flit per clock (or with probability sink_pct) and returns a
//     credit for each.
//   * Head flit data = {src[3:0], dest[3:0], 0, prio, 6'd12, seq[15:0]}, so
//     dest is at DEST_LSB = 24 and prio at PRIO_BIT = 22.  BODY/TAIL data
//     = {src, flit index, seq[7:0]^8'h5A, seq}: the sink can rebuild every
//     flit and check it.
//
// Measurement of one rate:
//   warm-up (WARM clocks) -> window (WIN clocks: packets made now are
//   "tagged", flits that arrive now are counted) -> keep the traffic going
//   until every tagged packet has arrived -> stop making packets and wait
//   until the fabric is empty again.
//   latency  = clock the TAIL arrives - clock the packet was made (this
//              includes the time in the source queue)
//   net lat. = clock the TAIL arrives - clock the HEAD left the source
//   accepted = flits that arrived in the window / (N * WIN)
//   sat = 1  = the fabric could not keep up (source queue full, accepted
//              < 90 % of offered, or tagged packets still missing after
//              DRAIN clocks); its latency is then not meaningful
//
// Checks for each fabric and rate (the scoreboard):
//   1. no packet at the wrong node, no duplicate, no corrupted or missing
//      flit, no two packets interleaved, no receive-buffer overflow
//   2. every packet made was delivered (exactly once, together with 1.)
//   3. at the end every credit is back (source counters at DEPTH, every
//      receive buffer empty)
//   (sanity run only) 4. no fabric saturates at a load of 0.1 with N = 4
//
// Output: one machine-readable line per fabric and rate, e.g.
//   RESULT fabric=mesh n=4 rate=0.100 offered=0.0990 accepted=0.0985 ...
// and at the end "ALL n CHECKS PASSED" or "m OF n CHECKS FAILED".
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// study_traffic: N traffic sources, N sinks and the scoreboard for one fabric
// -----------------------------------------------------------------------------
module study_traffic #(
    parameter N     = 4,
    parameter DEPTH = 4,
    parameter PKT   = 4,             // flits per packet
    parameter QCAP  = 256,           // source queue size, packets per node
    parameter MAXP  = 1024           // packet records per node (> QCAP)
) (
    input  wire            clk, rst_n,
    // node interface, connected to the fabric
    output reg  [N-1:0]    tx_valid,
    output reg  [N*34-1:0] tx_flit,
    input  wire [N-1:0]    tx_credit,
    input  wire [N-1:0]    rx_valid,
    input  wire [N*34-1:0] rx_flit,
    output reg  [N-1:0]    rx_credit,
    // control from the test
    input  wire            gen_on,       // make new packets
    input  wire            win_on,       // measurement window
    input  wire            clr,          // clear the window statistics
    input  wire            reseed,       // restart the traffic random numbers
    input  wire [31:0]     seed_val,
    input  wire [31:0]     rate_pm,      // flits/node/clock x 1000
    input  wire [31:0]     prio_pct,     // % of packets with prio = 1
    input  wire [31:0]     sink_pct,     // % chance a node takes a flit per clock
    // results (see the list at the bottom)
    output wire [16*32-1:0] stats
);
    localparam FW = 34;
    localparam [1:0] K_HEAD = 2'b00, K_BODY = 2'b01, K_TAIL = 2'b10;

    // ---------------- packet records (circular, MAXP per source) ----------
    integer   r_dest [0:N*MAXP-1];
    integer   r_gen  [0:N*MAXP-1];       // clock it was made
    integer   r_inj  [0:N*MAXP-1];       // clock its HEAD left the source
    integer   r_seq  [0:N*MAXP-1];
    reg       r_tag  [0:N*MAXP-1];       // made in the window
    reg       r_prio [0:N*MAXP-1];
    reg [1:0] r_st   [0:N*MAXP-1];       // 0 free, 1 on its way, 2 delivered

    // ---------------- per node state ----------------
    integer q_wr [0:N-1];                // next packet number to make
    integer q_rd [0:N-1];                // next packet number to send
    integer fpos [0:N-1];                // next flit of that packet
    integer cred [0:N-1];                // source credits
    integer sbuf [0:N-1];                // flits in the node's rx buffer
    reg     c_open [0:N-1];              // sink: inside a packet
    integer c_src  [0:N-1];
    integer c_seq  [0:N-1];
    integer c_idx  [0:N-1];
    integer c_rec  [0:N-1];

    // ---------------- counters ----------------
    integer cyc = 0;
    integer gseed = 1, sseed = 12345;
    integer errs = 0, n_gen = 0, n_del = 0, pend_tag = 0, prog = 0;
    integer off_w = 0, acc_w = 0, refused = 0;
    integer lat_cnt = 0, lat_sum = 0, lat_max = 0, lat_min = 0, net_sum = 0;
    integer qmax = 0;
    reg     idle = 1'b1;

    integer i0;
    initial begin
        tx_valid  = {N{1'b0}};
        tx_flit   = {N*FW{1'b0}};
        rx_credit = {N{1'b0}};
        lat_min   = 32'h7fffffff;
        for (i0 = 0; i0 < N; i0 = i0 + 1) begin
            q_wr[i0] = 0; q_rd[i0] = 0; fpos[i0] = 0; cred[i0] = DEPTH;
            sbuf[i0] = 0; c_open[i0] = 1'b0; c_src[i0] = 0; c_seq[i0] = 0;
            c_idx[i0] = 0; c_rec[i0] = 0;
        end
        for (i0 = 0; i0 < N*MAXP; i0 = i0 + 1) begin
            r_st[i0] = 2'd0; r_seq[i0] = -1; r_dest[i0] = 0; r_gen[i0] = 0;
            r_inj[i0] = 0; r_tag[i0] = 1'b0; r_prio[i0] = 1'b0;
        end
        if (N > 16) begin
            $display("FAIL: tb_study supports N up to 16 (4-bit src/dest fields)");
            $finish;
        end
    end

    // the flit number idx of packet seq from s to dst
    function [FW-1:0] mk_flit;
        input integer s, sq, idx, dst;
        input         pr;
        reg [3:0]  s4, d4, i4;
        reg [15:0] q16;
        begin
            s4 = s; d4 = dst; i4 = idx; q16 = sq;
            if (idx == 0)
                mk_flit = {K_HEAD, s4, d4, 1'b0, pr, 6'd12, q16};
            else
                mk_flit = {(idx == PKT - 1) ? K_TAIL : K_BODY, s4, i4,
                           q16[7:0] ^ 8'h5A, q16};
        end
    endfunction

    task fail(input [8*48-1:0] what, input integer node, input [FW-1:0] f);
        begin
            errs = errs + 1;
            if (errs <= 10)
                $display("  ERROR node %0d: %0s, flit %h (t=%0t)", node, what, f, $time);
        end
    endtask

    always @(posedge clk) begin : run
        integer s, d, i, r, dst, sq, lat;
        reg     pr, go, cons, ok;
        reg [FW-1:0] f;
        if (rst_n) begin
            cyc = cyc + 1;
            if (reseed) gseed = seed_val;
            if (clr) begin
                off_w = 0; acc_w = 0; refused = 0; lat_cnt = 0; lat_sum = 0;
                lat_max = 0; lat_min = 32'h7fffffff; net_sum = 0; qmax = 0;
            end

            // ---- 1. make new packets (Bernoulli, rate/PKT per clock) ----
            for (s = 0; s < N; s = s + 1) begin
                if (gen_on) begin
                    r = {$random(gseed)} % (1000 * PKT);
                    if (r < rate_pm) begin
                        dst = {$random(gseed)} % (N - 1);
                        if (dst >= s) dst = dst + 1;          // never to itself
                        pr  = ({$random(gseed)} % 100) < prio_pct;
                        if (win_on) off_w = off_w + PKT;
                        if (q_wr[s] - q_rd[s] >= QCAP) begin
                            if (win_on) refused = refused + 1;
                        end else begin
                            i = s * MAXP + (q_wr[s] % MAXP);
                            if (r_st[i] == 2'd1) fail("packet records overrun", s, 0);
                            r_st[i]   = 2'd1;
                            r_seq[i]  = q_wr[s] % 65536;
                            r_dest[i] = dst;
                            r_gen[i]  = cyc;
                            r_tag[i]  = win_on;
                            r_prio[i] = pr;
                            if (win_on) pend_tag = pend_tag + 1;
                            q_wr[s] = q_wr[s] + 1;
                            n_gen   = n_gen + 1;
                        end
                    end
                end
                if (win_on && q_wr[s] - q_rd[s] > qmax) qmax = q_wr[s] - q_rd[s];
            end

            // ---- 2. sources: send the next flit when there is a credit ----
            for (s = 0; s < N; s = s + 1) begin
                go = (q_rd[s] != q_wr[s]) && (cred[s] > 0);
                tx_valid[s] <= go;
                if (go) begin
                    i = s * MAXP + (q_rd[s] % MAXP);
                    if (fpos[s] == 0) r_inj[i] = cyc;
                    tx_flit[FW*s +: FW] <= mk_flit(s, r_seq[i], fpos[s], r_dest[i], r_prio[i]);
                    if (fpos[s] == PKT - 1) begin
                        fpos[s] = 0;
                        q_rd[s] = q_rd[s] + 1;
                    end else begin
                        fpos[s] = fpos[s] + 1;
                    end
                end
                cred[s] = cred[s] - go + tx_credit[s];
                if (cred[s] > DEPTH) fail("too many tx credits returned", s, 0);
            end

            // ---- 3. sinks: receive buffer, credits and the scoreboard ----
            for (d = 0; d < N; d = d + 1) begin
                cons = 1'b0;
                if (sbuf[d] > 0)
                    cons = (sink_pct >= 100) ? 1'b1 : (({$random(sseed)} % 100) < sink_pct);
                rx_credit[d] <= cons;
                sbuf[d] = sbuf[d] - cons;
                if (rx_valid[d]) begin
                    f = rx_flit[FW*d +: FW];
                    sbuf[d] = sbuf[d] + 1;
                    if (sbuf[d] > DEPTH) fail("rx buffer overflow (credit ignored)", d, f);
                    prog = prog + 1;
                    if (win_on) acc_w = acc_w + 1;
                    if (f[33:32] == K_HEAD) begin
                        if (c_open[d]) fail("new head inside a packet (interleaved)", d, f);
                        c_open[d] = 1'b0;
                        s  = f[31:28];
                        sq = f[15:0];
                        i  = s * MAXP + (sq % MAXP);
                        ok = 1'b0;
                        if (s >= N)
                            fail("head from a node that does not exist", d, f);
                        else if (r_st[i] != 2'd1 || r_seq[i] != sq)
                            fail("unknown or duplicate packet", d, f);
                        else if (r_dest[i] != d)
                            fail("packet delivered to the wrong node", d, f);
                        else if (f !== mk_flit(s, sq, 0, d, r_prio[i]))
                            fail("head flit corrupted", d, f);
                        else
                            ok = 1'b1;
                        if (ok) begin
                            c_open[d] = 1'b1;
                            c_src[d]  = s;
                            c_seq[d]  = sq;
                            c_idx[d]  = 1;
                            c_rec[d]  = i;
                        end
                    end else if (!c_open[d]) begin
                        fail("payload flit without a head", d, f);
                    end else if (f !== mk_flit(c_src[d], c_seq[d], c_idx[d], d, 1'b0)) begin
                        fail("payload flit wrong or out of order", d, f);
                        c_open[d] = 1'b0;
                    end else if (c_idx[d] == PKT - 1) begin
                        // TAIL: the packet is complete
                        i = c_rec[d];
                        r_st[i]   = 2'd2;
                        n_del     = n_del + 1;
                        c_open[d] = 1'b0;
                        if (r_tag[i]) begin
                            lat      = cyc - r_gen[i];
                            lat_sum  = lat_sum + lat;
                            net_sum  = net_sum + (cyc - r_inj[i]);
                            lat_cnt  = lat_cnt + 1;
                            pend_tag = pend_tag - 1;
                            if (lat > lat_max) lat_max = lat;
                            if (lat < lat_min) lat_min = lat;
                        end
                    end else begin
                        c_idx[d] = c_idx[d] + 1;
                    end
                end
            end

            // ---- 4. idle: nothing on its way and every credit back ----
            idle = (n_del == n_gen);
            for (s = 0; s < N; s = s + 1)
                if (q_rd[s] != q_wr[s] || fpos[s] != 0 || cred[s] != DEPTH ||
                    sbuf[s] != 0 || c_open[s] || rx_credit[s])
                    idle = 1'b0;
        end
    end

    // results for the test (32 bits each)
    assign stats[32*0  +: 32] = errs;        // scoreboard errors
    assign stats[32*1  +: 32] = n_gen;       // packets made (total)
    assign stats[32*2  +: 32] = n_del;       // packets delivered (total)
    assign stats[32*3  +: 32] = pend_tag;    // tagged packets not yet delivered
    assign stats[32*4  +: 32] = off_w;       // flits offered in the window
    assign stats[32*5  +: 32] = acc_w;       // flits delivered in the window
    assign stats[32*6  +: 32] = lat_cnt;     // tagged packets delivered
    assign stats[32*7  +: 32] = lat_sum;
    assign stats[32*8  +: 32] = lat_max;
    assign stats[32*9  +: 32] = net_sum;
    assign stats[32*10 +: 32] = refused;     // packets refused (queue full)
    assign stats[32*11 +: 32] = {31'd0, idle};
    assign stats[32*12 +: 32] = prog;        // flits delivered (total)
    assign stats[32*13 +: 32] = lat_min;
    assign stats[32*14 +: 32] = qmax;        // longest source queue (packets)
    assign stats[32*15 +: 32] = 32'd0;
endmodule

// -----------------------------------------------------------------------------
// tb_study: the fabrics, the run sequence, the checks and the result lines
// -----------------------------------------------------------------------------
module tb_study;
    parameter FABRIC = 3;            // 0 bus, 1 crossbar, 2 mesh, 3 all three
    parameter N      = 4;            // nodes: 2 .. 16 (mesh: 2, 4, 6, 8, 12, 16)
    parameter SEED   = 7;
    localparam DEPTH = 4;
    localparam PKT   = 4;
    localparam SW    = 16 * 32;      // width of one stats bus
    localparam [2:0] PRESENT = (FABRIC == 3) ? 3'b111 : (3'b001 << FABRIC);

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;            // 100 MHz

    integer checks = 0;
    integer errors = 0;
    task check(input cond, input [8*72-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    // ---------------- control, shared by every fabric ----------------
    reg        gen_on = 1'b0, win_on = 1'b0, clr = 1'b0, reseed = 1'b0;
    reg [31:0] seed_val = 32'd1, rate_pm = 32'd0, prio_pct = 32'd0, sink_pct = 32'd100;
    wire [3*SW-1:0] st_all;          // stats of fabric f at [SW*f +: SW]

    // ---------------- the fabrics, each with its own traffic ----------------
    generate
        if (PRESENT[0]) begin : g_bus
            wire [N-1:0]    tv, tc, rv, rc;
            wire [N*34-1:0] tf, rf;
            study_bus #(.N(N), .DEST_LSB(24), .PRIO_BIT(22), .DEPTH(DEPTH)) u_fab (
                .clk(clk), .rst_n(rst_n),
                .tx_valid(tv), .tx_flit(tf), .tx_credit(tc),
                .rx_valid(rv), .rx_flit(rf), .rx_credit(rc));
            study_traffic #(.N(N), .DEPTH(DEPTH), .PKT(PKT)) u_trf (
                .clk(clk), .rst_n(rst_n),
                .tx_valid(tv), .tx_flit(tf), .tx_credit(tc),
                .rx_valid(rv), .rx_flit(rf), .rx_credit(rc),
                .gen_on(gen_on), .win_on(win_on), .clr(clr), .reseed(reseed),
                .seed_val(seed_val), .rate_pm(rate_pm), .prio_pct(prio_pct),
                .sink_pct(sink_pct), .stats(st_all[SW*0 +: SW]));
        end else begin : g_no_bus
            assign st_all[SW*0 +: SW] = {SW{1'b0}};
        end

        if (PRESENT[1]) begin : g_xbar
            wire [N-1:0]    tv, tc, rv, rc;
            wire [N*34-1:0] tf, rf;
            study_xbar #(.N(N), .DEST_LSB(24), .PRIO_BIT(22), .DEPTH(DEPTH)) u_fab (
                .clk(clk), .rst_n(rst_n),
                .tx_valid(tv), .tx_flit(tf), .tx_credit(tc),
                .rx_valid(rv), .rx_flit(rf), .rx_credit(rc));
            study_traffic #(.N(N), .DEPTH(DEPTH), .PKT(PKT)) u_trf (
                .clk(clk), .rst_n(rst_n),
                .tx_valid(tv), .tx_flit(tf), .tx_credit(tc),
                .rx_valid(rv), .rx_flit(rf), .rx_credit(rc),
                .gen_on(gen_on), .win_on(win_on), .clr(clr), .reseed(reseed),
                .seed_val(seed_val), .rate_pm(rate_pm), .prio_pct(prio_pct),
                .sink_pct(sink_pct), .stats(st_all[SW*1 +: SW]));
        end else begin : g_no_xbar
            assign st_all[SW*1 +: SW] = {SW{1'b0}};
        end

        if (PRESENT[2]) begin : g_mesh
            wire [N-1:0]    tv, tc, rv, rc;
            wire [N*34-1:0] tf, rf;
            study_mesh #(.N(N), .DEST_LSB(24), .PRIO_BIT(22), .DEPTH(DEPTH)) u_fab (
                .clk(clk), .rst_n(rst_n),
                .tx_valid(tv), .tx_flit(tf), .tx_credit(tc),
                .rx_valid(rv), .rx_flit(rf), .rx_credit(rc));
            study_traffic #(.N(N), .DEPTH(DEPTH), .PKT(PKT)) u_trf (
                .clk(clk), .rst_n(rst_n),
                .tx_valid(tv), .tx_flit(tf), .tx_credit(tc),
                .rx_valid(rv), .rx_flit(rf), .rx_credit(rc),
                .gen_on(gen_on), .win_on(win_on), .clr(clr), .reseed(reseed),
                .seed_val(seed_val), .rate_pm(rate_pm), .prio_pct(prio_pct),
                .sink_pct(sink_pct), .stats(st_all[SW*2 +: SW]));
        end else begin : g_no_mesh
            assign st_all[SW*2 +: SW] = {SW{1'b0}};
        end
    endgenerate

    // stat k of fabric f
    function integer gs;
        input integer f, k;
        begin
            gs = st_all[SW*f + 32*k +: 32];
        end
    endfunction

    function [8*4-1:0] fname;
        input integer f;
        begin
            fname = (f == 0) ? "bus" : (f == 1) ? "xbar" : "mesh";
        end
    endfunction

    // every present fabric is empty / has all its tagged packets
    function all_idle;
        input dummy;
        integer f;
        begin
            all_idle = 1'b1;
            for (f = 0; f < 3; f = f + 1)
                if (PRESENT[f] && gs(f, 11) == 0) all_idle = 1'b0;
        end
    endfunction

    reg [2:0] sat;                   // fabric f could not keep up
    function all_tagged;
        input dummy;
        integer f;
        begin
            all_tagged = 1'b1;
            for (f = 0; f < 3; f = f + 1)
                if (PRESENT[f] && !sat[f] && gs(f, 3) != 0) all_tagged = 1'b0;
        end
    endfunction

    // ---------------- deadlock watchdog ----------------
    integer cyc = 0, last_prog = 0, prog_prev = -1;
    always @(posedge clk) begin : wdog
        integer tot;
        cyc = cyc + 1;
        tot = gs(0, 12) + gs(1, 12) + gs(2, 12);
        if (tot != prog_prev || all_idle(1'b0)) begin
            prog_prev = tot;
            last_prog = cyc;
        end else if (cyc - last_prog > 20000) begin
            $display("FAIL: DEADLOCK - no flit delivered for 20000 clocks (t=%0t)", $time);
            $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
            $finish;
        end
    end

    // ---------------- one rate: warm-up, window, drain, report ----------------
    integer warm, win, drain_max, seed;
    localparam IDLE_MAX = 400000;

    task run_rate(input integer rpm, input integer ppct, input integer spct,
                  input sanity);
        integer f, t, off, acc, lc;
        real    offr, accr, lat_avg, net_avg;
        reg     tmo;
        begin
            // set up: same random traffic for every fabric at this rate
            @(posedge clk);
            rate_pm  <= rpm;
            prio_pct <= ppct;
            sink_pct <= spct;
            seed_val <= seed + rpm;
            reseed   <= 1'b1;
            clr      <= 1'b1;
            @(posedge clk);
            reseed <= 1'b0;
            clr    <= 1'b0;
            gen_on <= 1'b1;
            repeat (warm) @(posedge clk);
            win_on <= 1'b1;
            repeat (win) @(posedge clk);
            win_on <= 1'b0;
            @(posedge clk);

            // clearly saturated fabrics do not have to deliver their tagged
            // packets before the traffic stops
            for (f = 0; f < 3; f = f + 1) begin
                off = gs(f, 4);
                acc = gs(f, 5);
                sat[f] = PRESENT[f] && (gs(f, 10) > 0 || 10 * acc < 9 * off);
            end
            // keep the traffic going until every tagged packet has arrived
            t = 0;
            while (t < drain_max && !all_tagged(1'b0)) begin
                @(posedge clk);
                t = t + 1;
            end
            for (f = 0; f < 3; f = f + 1)
                if (PRESENT[f] && gs(f, 3) != 0) sat[f] = 1'b1;
            // stop the traffic and let every fabric run empty
            gen_on <= 1'b0;
            t = 0;
            while (t < IDLE_MAX && !all_idle(1'b0)) begin
                @(posedge clk);
                t = t + 1;
            end
            tmo = (t >= IDLE_MAX);
            repeat (20) @(posedge clk);

            // results and checks
            for (f = 0; f < 3; f = f + 1) begin
                if (PRESENT[f]) begin
                    offr    = gs(f, 4) * 1.0 / (N * win);
                    accr    = gs(f, 5) * 1.0 / (N * win);
                    lc      = gs(f, 6);
                    lat_avg = (lc > 0) ? gs(f, 7) * 1.0 / lc : 0.0;
                    net_avg = (lc > 0) ? gs(f, 9) * 1.0 / lc : 0.0;
                    $display("RESULT fabric=%0s n=%0d rate=%0.3f offered=%0.4f accepted=%0.4f total=%0.3f lat_avg=%0.2f lat_max=%0d net_avg=%0.2f lat_min=%0d pkts=%0d qmax=%0d sat=%0d",
                             fname(f), N, rpm / 1000.0, offr, accr, accr * N,
                             lat_avg, gs(f, 8), net_avg, (lc > 0) ? gs(f, 13) : 0,
                             lc, gs(f, 14), sat[f]);
                    check(gs(f, 0) == 0,
                          "scoreboard: no lost/duplicate/corrupt/misrouted/interleaved flit");
                    check(!tmo && gs(f, 1) > 0 && gs(f, 2) == gs(f, 1),
                          "every packet made was delivered exactly once");
                    check(gs(f, 11) == 1, "fabric empty and every credit returned");
                    if (sanity && rpm <= 100)
                        check(!sat[f], "no saturation at a load of 0.1");
                    if (gs(f, 0) != 0 || gs(f, 2) != gs(f, 1) || gs(f, 11) != 1)
                        $display("  %0s: made %0d delivered %0d errors %0d idle %0d",
                                 fname(f), gs(f, 1), gs(f, 2), gs(f, 0), gs(f, 11));
                end
            end
        end
    endtask

    // ---------------- main sequence ----------------
    integer rate_one, k;
    reg     do_sweep;
    initial begin
        warm = 1000; win = 4000; drain_max = 20000; seed = SEED;
        rate_one = 0;
        do_sweep = $test$plusargs("sweep");
        if (!$value$plusargs("rate=%d", rate_one)) rate_one = 0;
        if (rate_one == 0 && !do_sweep) begin
            warm = 500;                  // the quick sanity run
            win  = 2000;
        end
        if ($value$plusargs("warm=%d", k))  warm = k;
        if ($value$plusargs("win=%d", k))   win = k;
        if ($value$plusargs("drain=%d", k)) drain_max = k;
        if ($value$plusargs("seed=%d", k))  seed = k;
        sat = 3'b000;

        $display("tb_study: FABRIC=%0d (0 bus, 1 xbar, 2 mesh, 3 all) N=%0d DEPTH=%0d, %0d-flit packets, warm %0d, window %0d",
                 FABRIC, N, DEPTH, PKT, warm, win);
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (5) @(posedge clk);

        if (do_sweep) begin
            run_rate(20,   0, 100, 1'b0);
            run_rate(50,   0, 100, 1'b0);
            run_rate(100,  0, 100, 1'b0);
            run_rate(200,  0, 100, 1'b0);
            run_rate(300,  0, 100, 1'b0);
            run_rate(500,  0, 100, 1'b0);
            run_rate(700,  0, 100, 1'b0);
            run_rate(1000, 0, 100, 1'b0);
        end else if (rate_one != 0) begin
            run_rate(rate_one, 0, 100, 1'b0);
        end else begin
            // sanity: light load, then overload with urgent packets and
            // receivers that stall
            run_rate(100,  0, 100, 1'b1);
            run_rate(1000, 20, 70, 1'b1);
        end

        if (errors == 0)
            $display("ALL %0d CHECKS PASSED", checks);
        else
            $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end

    // hard watchdog: never hang
    initial begin
        #(200_000_000);
        $display("FAIL: watchdog timeout (t=%0t)", $time);
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule
