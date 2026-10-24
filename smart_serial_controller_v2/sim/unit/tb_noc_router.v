// =============================================================================
// tb_noc_router.v  -  unit test: one noc_router driven with raw flits
// -----------------------------------------------------------------------------
// Part 1 (routing) uses ten small routers at different places in different
// meshes (all six routers of the 3x2 mesh, plus 4x4, 4x2, 2x2 and 3x3
// positions with IDW = 2..4).  Every input of every router gets one packet
// for every possible dest value (SINGLE, or HEAD + TAIL):
//   * the flit must leave on the XY output port (dest >= W*H -> LOCAL),
//     unchanged, and the TAIL must follow its HEAD on the same port,
//   * the per-hop latency (in_valid -> out_valid) must be 2 clocks (<= 3),
//   * in_credit of that input must pulse exactly once per flit.
//
// Part 2 uses the centre router (1,1) of a 3x3 mesh, so all five outputs are
// reachable.  Every input has a source (a flit queue that obeys the credits)
// and every output has a sink (a model of a DEPTH-flit buffer that returns a
// credit for each flit it frees, at a random rate or not at all).  A
// scoreboard checks every flit that leaves: right output, unchanged, packets
// never interleaved, in order per (input, output) flow, delivered once.
//   T1 throughput: single streams and five parallel streams at 1 flit/clock
//   T2 prio beats round-robin (order compared with a reference model)
//   T3 round-robin fairness (5 inputs, then 2 streams that must alternate)
//   T4 wormhole: competing multi-flit packets never interleave, and a prio
//      packet waits for the TAIL of the packet that holds the output
//   T5 credit stall: with no credits returned exactly DEPTH flits leave
//   T6 random stress: 2000 packets, random rates and stalls on all ports
//   T7 a stray BODY flit at an idle input is dropped (and its credit given)
//   T8 end: in_credit pulses = flits sent, every credit counter = DEPTH
// =============================================================================
`timescale 1ns / 1ps
module tb_noc_router;
    localparam DEPTH = 4;
    localparam FW    = 34;
    localparam [1:0] K_HEAD = 2'b00, K_BODY = 2'b01, K_TAIL = 2'b10, K_SINGLE = 2'b11;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer seed;
    integer cyc = 0;                 // clock counter (cycle number)
    always @(posedge clk) cyc <= cyc + 1;

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

    // expected XY output port (0 LOCAL 1 NORTH 2 EAST 3 SOUTH 4 WEST)
    function integer exp_port;
        input integer x, y, w, h, d;
        integer dx, dy;
        begin
            dx = d % w;
            dy = d / w;
            if (d >= w * h)  exp_port = 0;
            else if (dx > x) exp_port = 2;
            else if (dx < x) exp_port = 4;
            else if (dy > y) exp_port = 3;
            else if (dy < y) exp_port = 1;
            else             exp_port = 0;
        end
    endfunction

    // =========================================================================
    // Part 1: ten routers for the routing test
    // =========================================================================
    //  k : 0..5 = 3x2 mesh nodes, 6 = 4x4 (2,1), 7 = 4x2 (0,1), 8 = 2x2 (1,1),
    //      9 = 3x3 (1,1).  Tables are packed, k = 0 in the low bits.
    localparam NR = 10;
    localparam [4*NR-1:0] T_X = {4'd1, 4'd1, 4'd0, 4'd2, 4'd2, 4'd1, 4'd0, 4'd2, 4'd1, 4'd0};
    localparam [4*NR-1:0] T_Y = {4'd1, 4'd1, 4'd1, 4'd1, 4'd1, 4'd1, 4'd1, 4'd0, 4'd0, 4'd0};
    localparam [4*NR-1:0] T_W = {4'd3, 4'd2, 4'd4, 4'd4, 4'd3, 4'd3, 4'd3, 4'd3, 4'd3, 4'd3};
    localparam [4*NR-1:0] T_H = {4'd3, 4'd2, 4'd2, 4'd4, 4'd2, 4'd2, 4'd2, 4'd2, 4'd2, 4'd2};
    localparam [4*NR-1:0] T_I = {4'd4, 4'd2, 4'd4, 4'd4, 4'd3, 4'd3, 4'd3, 4'd3, 4'd3, 4'd3};
    localparam [8*NR-1:0] T_D = {8'd24, 8'd26, 8'd24, 8'd24, 8'd26, 8'd26, 8'd26, 8'd26, 8'd26, 8'd26};

    reg  [NR*5-1:0]    rt_iv = {NR*5{1'b0}};
    reg  [NR*5*FW-1:0] rt_if = {NR*5*FW{1'b0}};
    wire [NR*5-1:0]    rt_ic;
    wire [NR*5-1:0]    rt_ov;
    wire [NR*5*FW-1:0] rt_of;
    reg  [NR*5-1:0]    rt_oc = {NR*5{1'b0}};
    wire [NR*40-1:0]   rt_dcred;
    wire [NR*40-1:0]   rt_dlevel;

    genvar g;
    generate
        for (g = 0; g < NR; g = g + 1) begin : g_rt
            noc_router #(
                .X(T_X[4*g +: 4]), .Y(T_Y[4*g +: 4]), .W(T_W[4*g +: 4]),
                .H(T_H[4*g +: 4]), .IDW(T_I[4*g +: 4]), .DEST_LSB(T_D[8*g +: 8]),
                .PRIO_BIT(22), .DEPTH(DEPTH)
            ) u_r (
                .clk(clk), .rst_n(rst_n),
                .in_valid(rt_iv[5*g +: 5]), .in_flit(rt_if[5*FW*g +: 5*FW]),
                .in_credit(rt_ic[5*g +: 5]),
                .out_valid(rt_ov[5*g +: 5]), .out_flit(rt_of[5*FW*g +: 5*FW]),
                .out_credit(rt_oc[5*g +: 5]));
            assign rt_dcred[40*g +: 40]  = u_r.dbg_cred;
            assign rt_dlevel[40*g +: 40] = u_r.dbg_level;
        end
    endgenerate

    // ideal sink: every flit that leaves gives its credit back one clock later
    always @(posedge clk) rt_oc <= rt_ov;

    // monitor: records what router mon_k does (sampled at the falling edge)
    reg         mon_on = 1'b0;
    integer     mon_k = 0, mon_p = 0;
    integer     mon_n = 0, mon_cr = 0, mon_badcr = 0;
    integer     mon_q [0:7];
    integer     mon_t [0:7];
    reg [FW-1:0] mon_f [0:7];
    always @(negedge clk) begin : mon
        integer q;
        if (mon_on) begin
            for (q = 0; q < 5; q = q + 1) begin
                if (rt_ov[5*mon_k + q]) begin
                    if (mon_n < 8) begin
                        mon_q[mon_n] = q;
                        mon_t[mon_n] = cyc;
                        mon_f[mon_n] = rt_of[FW*(5*mon_k + q) +: FW];
                    end
                    mon_n = mon_n + 1;
                end
                if (rt_ic[5*mon_k + q]) begin
                    if (q == mon_p) mon_cr    = mon_cr + 1;
                    else            mon_badcr = mon_badcr + 1;
                end
            end
        end
    end

    integer lat_min = 99, lat_max = 0, n_routed = 0, n_beyond = 0;

    // send one packet (SINGLE, or HEAD + TAIL) into input p of router k
    task route_one(input integer k, input integer p, input integer d);
        integer i, t0, ep, two, idw, dl, nf;
        reg [FW-1:0] f, tl;
        begin
            idw = T_I[4*k +: 4];
            dl  = T_D[8*k +: 8];
            two = {$random(seed)} % 2;
            f   = {two ? K_HEAD : K_SINGLE, $random(seed)};
            for (i = 0; i < idw; i = i + 1) f[dl + i] = d[i];
            tl  = {K_TAIL, $random(seed)};
            ep  = exp_port(T_X[4*k +: 4], T_Y[4*k +: 4], T_W[4*k +: 4],
                           T_H[4*k +: 4], d);
            nf  = two ? 2 : 1;
            if (d >= T_W[4*k +: 4] * T_H[4*k +: 4]) n_beyond = n_beyond + 1;

            mon_k = k; mon_p = p; mon_n = 0; mon_cr = 0; mon_badcr = 0;
            @(negedge clk);
            mon_on = 1'b1;
            rt_iv[5*k + p] = 1'b1;
            rt_if[FW*(5*k + p) +: FW] = f;
            t0 = cyc;
            if (two) begin
                @(negedge clk);
                rt_if[FW*(5*k + p) +: FW] = tl;
            end
            @(negedge clk);
            rt_iv[5*k + p] = 1'b0;
            repeat (6) @(negedge clk);
            mon_on = 1'b0;

            check(mon_n == nf && mon_q[0] == ep && mon_f[0] == f &&
                  (!two || (mon_q[1] == ep && mon_f[1] == tl)),
                  "R: flit leaves unchanged on the XY port");
            check(mon_t[0] - t0 == 2 && (!two || mon_t[1] - t0 == 3),
                  "R: per-hop latency is 2 clocks");
            check(mon_cr == nf && mon_badcr == 0, "R: one in_credit pulse per flit");
            if (mon_n != nf || mon_q[0] != ep)
                $display("  router %0d in %0d dest %0d: %0d flits, port %0d, want %0d",
                         k, p, d, mon_n, mon_q[0], ep);
            if (mon_n > 0) begin
                if (mon_t[0] - t0 < lat_min) lat_min = mon_t[0] - t0;
                if (mon_t[0] - t0 > lat_max) lat_max = mon_t[0] - t0;
            end
            n_routed = n_routed + 1;
        end
    endtask

    // =========================================================================
    // Part 2: the main router with sources, sinks and a scoreboard
    // =========================================================================
    // centre of a 3x3 mesh: dest 4 LOCAL, 1 NORTH, 2/5 EAST, 7 SOUTH, 0/3/6 WEST
    localparam MX = 1, MY = 1, MW = 3, MH = 3;
    localparam QN = 8192;            // flits per source queue / sink log
    localparam PN = 2048;            // packets per source

    reg  [4:0]      m_iv = 5'd0;
    reg  [5*FW-1:0] m_if = {5*FW{1'b0}};
    wire [4:0]      m_ic;
    wire [4:0]      m_ov;
    wire [5*FW-1:0] m_of;
    reg  [4:0]      m_oc = 5'd0;

    noc_router #(
        .X(MX), .Y(MY), .W(MW), .H(MH), .IDW(3), .DEST_LSB(26), .PRIO_BIT(22),
        .DEPTH(DEPTH)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m_iv), .in_flit(m_if), .in_credit(m_ic),
        .out_valid(m_ov), .out_flit(m_of), .out_credit(m_oc));

    // ---------------- sources (one per input) ----------------
    reg  [FW-1:0] sq [0:5*QN-1];     // flit queue of input p at [QN*p ..]
    integer sq_wr  [0:4];            // flits written into the queue
    integer sq_rd  [0:4];            // flits sent
    integer s_cred [0:4];            // credit counter (starts at DEPTH)
    integer s_crx  [0:4];            // in_credit pulses seen
    integer s_rate [0:4];            // % of clocks the source tries to send
    integer pk_n   [0:4];            // packets queued per input
    integer pk_start [0:5*PN-1];     // first flit of each packet in sq
    integer pk_len   [0:5*PN-1];     // flits in the packet
    integer pk_out   [0:5*PN-1];     // output it must leave on

    always @(posedge clk) begin : src
        integer p;
        reg go;
        for (p = 0; p < 5; p = p + 1) begin
            go = 1'b0;
            if (rst_n && sq_rd[p] < sq_wr[p] && s_cred[p] > 0)
                go = ({$random(seed)} % 100) < s_rate[p];
            m_iv[p] <= go;
            if (go) begin
                m_if[FW*p +: FW] <= sq[QN*p + sq_rd[p]];
                sq_rd[p] = sq_rd[p] + 1;
            end
            s_cred[p] = s_cred[p] - go + m_ic[p];
            if (m_ic[p]) s_crx[p] = s_crx[p] + 1;
        end
    end

    // queue one packet of nfl flits (nfl = 1 -> SINGLE) at input p
    task add_pkt(input integer p, input integer dest, input integer nfl, input prio);
        integer i, s, a;
        reg [FW-1:0] f;
        begin
            s = pk_n[p];
            a = sq_wr[p];
            pk_start[PN*p + s] = a;
            pk_len[PN*p + s]   = nfl;
            pk_out[PN*p + s]   = exp_port(MX, MY, MW, MH, dest);
            f = {(nfl == 1) ? K_SINGLE : K_HEAD, $random(seed)};
            f[28:26] = dest;        // dest field
            f[25:23] = p;           // input it came from (scoreboard)
            f[22]    = prio;
            f[15:0]  = s;           // packet number of this input
            sq[QN*p + a] = f;
            for (i = 1; i < nfl; i = i + 1)
                sq[QN*p + a + i] = {(i == nfl - 1) ? K_TAIL : K_BODY, $random(seed)};
            pk_n[p]  = s + 1;
            sq_wr[p] = a + nfl;
        end
    endtask

    // ---------------- sinks (one per output) + scoreboard ----------------
    reg  [4:0]    k_en = 5'h1F;      // 0: the sink frees nothing (no credits)
    integer k_rate [0:4];            // % of clocks the sink frees a flit
    integer k_cnt  [0:4];            // flits in the modelled downstream buffer
    integer k_rx   [0:4];            // flits received
    reg  [FW-1:0] kq [0:5*QN-1];     // received flits per output
    integer kq_t   [0:5*QN-1];       // cycle it was on out_valid
    integer kq_src [0:5*QN-1];       // input it came from
    integer k_over = 0;              // buffer over-fills (must stay 0)
    integer cur_src  [0:4];          // packet in progress on each output
    integer cur_ptr  [0:4];
    integer cur_left [0:4];          // its flits still to come
    integer fptr [0:24];             // flow pointer per (output, input)
    integer sb_err = 0, sb_pkts = 0, sb_flits = 0;

    always @(posedge clk) begin : sink
        integer q, p, i;
        reg [FW-1:0] f;
        reg cons;
        for (q = 0; q < 5; q = q + 1) begin
            cons = 1'b0;
            if (rst_n && k_cnt[q] > 0 && k_en[q])
                cons = ({$random(seed)} % 100) < k_rate[q];
            m_oc[q] <= cons;
            k_cnt[q] = k_cnt[q] - cons;
            if (m_ov[q]) begin
                f = m_of[FW*q +: FW];
                k_cnt[q] = k_cnt[q] + 1;
                if (k_cnt[q] > DEPTH) k_over = k_over + 1;
                sb_flits = sb_flits + 1;
                if (f[33:32] == K_HEAD || f[33:32] == K_SINGLE) begin
                    if (cur_left[q] != 0) begin
                        sb_err = sb_err + 1;
                        $display("  out %0d: new head while a packet is open (t=%0t)", q, $time);
                    end
                    p = f[25:23];
                    i = (p < 5) ? fptr[5*q + p] : 0;
                    if (p < 5)
                        while (i < pk_n[p] && pk_out[PN*p + i] != q) i = i + 1;
                    if (p >= 5 || i >= pk_n[p] || i != f[15:0]) begin
                        sb_err = sb_err + 1;
                        cur_left[q] = 0;
                        $display("  out %0d: unexpected head %h (t=%0t)", q, f, $time);
                    end else begin
                        fptr[5*q + p] = i + 1;
                        if (f !== sq[QN*p + pk_start[PN*p + i]]) begin
                            sb_err = sb_err + 1;
                            $display("  out %0d: head changed %h (t=%0t)", q, f, $time);
                        end
                        cur_src[q]  = p;
                        cur_ptr[q]  = pk_start[PN*p + i] + 1;
                        cur_left[q] = pk_len[PN*p + i] - 1;
                        if (cur_left[q] == 0) sb_pkts = sb_pkts + 1;
                    end
                end else begin
                    if (cur_left[q] == 0) begin
                        sb_err = sb_err + 1;
                        $display("  out %0d: payload flit outside a packet (t=%0t)", q, $time);
                    end else begin
                        if (f !== sq[QN*cur_src[q] + cur_ptr[q]]) begin
                            sb_err = sb_err + 1;
                            $display("  out %0d: payload flit wrong %h (t=%0t)", q, f, $time);
                        end
                        cur_ptr[q]  = cur_ptr[q] + 1;
                        cur_left[q] = cur_left[q] - 1;
                        if (cur_left[q] == 0) sb_pkts = sb_pkts + 1;
                    end
                end
                kq[QN*q + k_rx[q]]     = f;
                kq_t[QN*q + k_rx[q]]   = cyc;
                kq_src[QN*q + k_rx[q]] = cur_src[q];
                k_rx[q] = k_rx[q] + 1;
            end
        end
    end

    // random rates and stalls while "chaos" is on (T6)
    reg chaos = 1'b0;
    always @(posedge clk) begin : chaos_gen
        integer p;
        if (chaos && (cyc % 32) == 0) begin
            for (p = 0; p < 5; p = p + 1) begin
                k_en[p]   = ({$random(seed)} % 5) != 0;
                k_rate[p] = 20 + {$random(seed)} % 81;
                s_rate[p] = 20 + {$random(seed)} % 81;
            end
        end
    end

    // ---------------- helpers ----------------
    task set_rates(input integer sr, input integer kr);
        integer p;
        begin
            for (p = 0; p < 5; p = p + 1) begin
                s_rate[p] = sr;
                k_rate[p] = kr;
            end
        end
    endtask

    function all_idle;
        input dummy;
        integer p;
        begin
            all_idle = (m_iv == 5'd0) && (m_ov == 5'd0) && (m_ic == 5'd0) && (m_oc == 5'd0);
            for (p = 0; p < 5; p = p + 1) begin
                if (sq_rd[p] != sq_wr[p] || k_cnt[p] != 0 || s_cred[p] != DEPTH ||
                    dut.dbg_level[8*p +: 8] != 0 || dut.dbg_cred[8*p +: 8] != DEPTH)
                    all_idle = 1'b0;
            end
        end
    endfunction

    task wait_idle(input integer maxc);
        integer c;
        begin
            c = 0;
            while (!all_idle(1'b0) && c < maxc) begin
                @(negedge clk);
                c = c + 1;
            end
            repeat (3) @(negedge clk);
            check(all_idle(1'b0), "router back to idle");
        end
    endtask

    task wait_rx(input integer q, input integer n, input integer maxc);
        integer c;
        begin
            c = 0;
            while (k_rx[q] < n && c < maxc) begin
                @(negedge clk);
                c = c + 1;
            end
            if (k_rx[q] < n)
                $display("  timeout: output %0d has %0d of %0d flits", q, k_rx[q], n);
        end
    endtask

    function integer total_pkts;
        input dummy;
        integer p;
        begin
            total_pkts = 0;
            for (p = 0; p < 5; p = p + 1) total_pkts = total_pkts + pk_n[p];
        end
    endfunction

    // ---------------- reference arbiter model (T2, T3) ----------------
    integer rm_pr [0:24];            // prio of flit i of input p at [5*p + i]
    integer rm_n  [0:4];
    integer rm_h  [0:4];
    integer rm_order [0:24];
    task ref_order(input integer last0, input integer total);
        integer n, k, idx, pick, anyp, last;
        begin
            last = last0;
            for (k = 0; k < 5; k = k + 1) rm_h[k] = 0;
            for (n = 0; n < total; n = n + 1) begin
                anyp = 0;
                for (k = 0; k < 5; k = k + 1)
                    if (rm_h[k] < rm_n[k] && rm_pr[5*k + rm_h[k]]) anyp = 1;
                pick = -1;
                for (k = 1; k <= 5; k = k + 1) begin
                    idx = (last + k) % 5;
                    if (pick < 0 && rm_h[idx] < rm_n[idx] &&
                        (!anyp || rm_pr[5*idx + rm_h[idx]])) pick = idx;
                end
                rm_order[n] = pick;
                rm_h[pick]  = rm_h[pick] + 1;
                last = pick;
            end
        end
    endtask

    // T2/T3: block LOCAL, fill all five inputs with 4 SINGLE flits each
    // (prio pattern: bit i of pat[4p +: 4] = prio of flit i of input p),
    // release, and compare the grant order with the model.
    integer arb_ok;
    task arb_test(input [19:0] pat);
        integer p, i, n0, ok;
        begin
            set_rates(100, 100);
            k_en[0] = 1'b0;
            n0 = k_rx[0];
            for (i = 0; i < 4; i = i + 1) add_pkt(0, 4, 1, 1'b0);   // use up the credits
            wait_rx(0, n0 + 4, 100);
            repeat (5) @(negedge clk);
            check(dut.dbg_cred[7:0] == 0, "arb: LOCAL output has no credits");
            for (p = 0; p < 5; p = p + 1) begin
                rm_n[p] = 4;
                for (i = 0; i < 4; i = i + 1) begin
                    rm_pr[5*p + i] = pat[4*p + i];
                    add_pkt(p, 4, 1, pat[4*p + i]);
                end
            end
            repeat (12) @(negedge clk);
            ok = 1;
            for (p = 0; p < 5; p = p + 1) if (dut.dbg_level[8*p +: 8] != DEPTH) ok = 0;
            check(ok, "arb: all five input FIFOs full");
            k_en[0] = 1'b1;
            wait_rx(0, n0 + 24, 200);
            ref_order(0, 20);         // the 4 fillers came from input 0
            arb_ok = 1;
            for (i = 0; i < 20; i = i + 1)
                if (kq_src[n0 + 4 + i] != rm_order[i]) arb_ok = 0;
            if (!arb_ok) begin
                $write("  got :");
                for (i = 0; i < 20; i = i + 1) $write(" %0d", kq_src[n0 + 4 + i]);
                $write("\n  want:");
                for (i = 0; i < 20; i = i + 1) $write(" %0d", rm_order[i]);
                $write("\n");
            end
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    integer i, j, k, p, q, n0, n1, c, ok, first, last, cycles;
    integer nb [0:4];
    integer sent0;
    reg [19:0] pat;
    initial begin
        seed = 20260;
        for (p = 0; p < 5; p = p + 1) begin
            sq_wr[p] = 0; sq_rd[p] = 0; s_cred[p] = DEPTH; s_crx[p] = 0;
            s_rate[p] = 100; pk_n[p] = 0;
            k_rate[p] = 100; k_cnt[p] = 0; k_rx[p] = 0;
            cur_src[p] = 0; cur_ptr[p] = 0; cur_left[p] = 0;
        end
        for (i = 0; i < 25; i = i + 1) fptr[i] = 0;
        #33 rst_n = 1'b1;
        repeat (3) @(negedge clk);

        // ---------------------------------------------------------------
        // Part 1: routing from every input to every dest, ten routers
        // ---------------------------------------------------------------
        for (k = 0; k < NR; k = k + 1)
            for (p = 0; p < 5; p = p + 1)
                for (j = 0; j < (1 << T_I[4*k +: 4]); j = j + 1)
                    route_one(k, p, j);
        ok = 1;
        for (k = 0; k < NR; k = k + 1)
            for (q = 0; q < 5; q = q + 1)
                if (rt_dcred[40*k + 8*q +: 8] != DEPTH || rt_dlevel[40*k + 8*q +: 8] != 0)
                    ok = 0;
        check(ok, "R: all credit counters back to DEPTH, FIFOs empty");
        check(n_beyond == 5 * (2 * 6 + 8 + 7), "R: dest >= W*H cases were sent");
        $display("R: %0d packets routed (%0d with dest >= W*H -> LOCAL), latency %0d..%0d clocks per hop",
                 n_routed, n_beyond, lat_min, lat_max);
        check(lat_max <= 3, "R: latency <= 3 clocks per hop");

        // ---------------------------------------------------------------
        // T1: throughput
        // ---------------------------------------------------------------
        set_rates(100, 100);
        n0 = k_rx[2];
        add_pkt(0, 5, 200, 1'b0);                      // LOCAL -> EAST, 200 flits
        wait_rx(2, n0 + 200, 1000);
        cycles = kq_t[QN*2 + n0 + 199] - kq_t[QN*2 + n0] + 1;
        $display("T1: one 200-flit packet LOCAL->EAST in %0d clocks = %0d.%03d flits/clock",
                 cycles, 200 / cycles, (200 * 1000 / cycles) % 1000);
        check(cycles == 200, "T1: 200-flit packet at 1 flit/clock");

        n0 = k_rx[1];
        for (i = 0; i < 100; i = i + 1) add_pkt(4, 1, 1, 1'b0);   // WEST -> NORTH SINGLEs
        wait_rx(1, n0 + 100, 1000);
        cycles = kq_t[QN*1 + n0 + 99] - kq_t[QN*1 + n0] + 1;
        check(cycles == 100, "T1: 100 SINGLE flits at 1 flit/clock");

        n0 = k_rx[0];
        for (i = 0; i < 50; i = i + 1) add_pkt(3, 4, 4, 1'b0);    // SOUTH -> LOCAL, 4-flit pkts
        wait_rx(0, n0 + 200, 1000);
        cycles = kq_t[n0 + 199] - kq_t[n0] + 1;
        check(cycles == 200, "T1: 50 back-to-back 4-flit packets, no bubble");

        // five streams at once (a permutation): 5 flits/clock in total
        for (q = 0; q < 5; q = q + 1) nb[q] = k_rx[q];
        for (i = 0; i < 25; i = i + 1) begin
            add_pkt(0, 5, 4, 1'b0);    // LOCAL -> EAST
            add_pkt(1, 7, 4, 1'b0);    // NORTH -> SOUTH
            add_pkt(2, 3, 4, 1'b0);    // EAST  -> WEST
            add_pkt(3, 1, 4, 1'b0);    // SOUTH -> NORTH
            add_pkt(4, 4, 4, 1'b0);    // WEST  -> LOCAL
        end
        for (q = 0; q < 5; q = q + 1) wait_rx(q, nb[q] + 100, 1000);
        ok = 1;
        for (q = 0; q < 5; q = q + 1)
            if (kq_t[QN*q + nb[q] + 99] - kq_t[QN*q + nb[q]] + 1 != 100) ok = 0;
        check(ok, "T1: five parallel streams, each at 1 flit/clock");
        wait_idle(200);
        check(sb_err == 0 && k_over == 0, "T1: scoreboard clean");

        // ---------------------------------------------------------------
        // T2: prio beats round-robin
        // ---------------------------------------------------------------
        // in0 0000, in1 0010 (2nd flit urgent), in2 0000, in3 0011, in4 1000
        pat = {4'b1000, 4'b0011, 4'b0000, 4'b0010, 4'b0000};
        n0 = k_rx[0];
        arb_test(pat);
        check(kq_src[n0 + 4] == 3 && kq_src[n0 + 5] == 3, "T2: urgent flits of input 3 go first");
        check(arb_ok, "T2: grant order = prio first, then round-robin");
        wait_idle(200);

        // ---------------------------------------------------------------
        // T3: round-robin fairness
        // ---------------------------------------------------------------
        n0 = k_rx[0];
        arb_test(20'd0);
        check(arb_ok, "T3: grant order = round-robin");
        ok = 1;
        for (i = 0; i < 16; i = i + 1)
            for (j = 1; j < 5; j = j + 1)
                for (k = 0; k < j; k = k + 1)
                    if (kq_src[n0 + 4 + i + j] == kq_src[n0 + 4 + i + k]) ok = 0;
        check(ok, "T3: every 5 grants in a row serve 5 different inputs");
        wait_idle(200);

        // two full-rate streams into one output must alternate exactly
        n0 = k_rx[2];
        for (i = 0; i < 100; i = i + 1) begin
            add_pkt(1, 5, 1, 1'b0);    // NORTH -> EAST
            add_pkt(4, 5, 1, 1'b0);    // WEST  -> EAST
        end
        wait_rx(2, n0 + 200, 1000);
        ok = 1;
        for (i = 1; i < 200; i = i + 1)
            if (kq_src[QN*2 + n0 + i] == kq_src[QN*2 + n0 + i - 1]) ok = 0;
        check(ok, "T3: two competing streams alternate 1:1");
        cycles = kq_t[QN*2 + n0 + 199] - kq_t[QN*2 + n0] + 1;
        check(cycles == 200, "T3: shared output still runs at 1 flit/clock");
        wait_idle(200);

        // ---------------------------------------------------------------
        // T4: wormhole
        // ---------------------------------------------------------------
        n0 = k_rx[4];
        add_pkt(0, 3, 7, 1'b0);        // LOCAL -> WEST, 7 flits
        add_pkt(2, 3, 7, 1'b0);        // EAST  -> WEST, 7 flits
        wait_rx(4, n0 + 14, 200);
        ok = 1;
        for (i = 0; i < 14; i = i + 1) begin
            if (kq[QN*4 + n0 + i][33:32] !=
                ((i % 7 == 0) ? K_HEAD : (i % 7 == 6) ? K_TAIL : K_BODY)) ok = 0;
            if (kq_src[QN*4 + n0 + i] != kq_src[QN*4 + n0 + 7 * (i / 7)]) ok = 0;
        end
        check(ok && kq_src[QN*4 + n0] != kq_src[QN*4 + n0 + 7],
              "T4: two 7-flit packets leave one after the other");
        wait_idle(200);

        // three inputs, slow random rates, 3 x 6-flit packets each
        set_rates(40, 60);
        n0 = sb_pkts;
        for (i = 0; i < 3; i = i + 1) begin
            add_pkt(0, 0, 6, 1'b0);
            add_pkt(2, 6, 6, 1'b0);
            add_pkt(3, 3, 6, 1'b0);
        end
        c = 0;
        while (sb_pkts < n0 + 9 && c < 2000) begin @(negedge clk); c = c + 1; end
        check(sb_pkts == n0 + 9 && sb_err == 0, "T4: 9 competing packets, never interleaved");
        wait_idle(200);

        // a prio packet must wait for the TAIL of the packet holding the output
        set_rates(100, 100);
        s_rate[1] = 20;                // slow source keeps the lock for long
        n0 = k_rx[4];
        add_pkt(1, 3, 20, 1'b0);       // NORTH -> WEST, 20 flits
        wait_rx(4, n0 + 1, 200);
        add_pkt(0, 3, 1, 1'b1);        // urgent SINGLE LOCAL -> WEST
        wait_rx(4, n0 + 21, 1000);
        check(kq_src[QN*4 + n0 + 20] == 0 && kq[QN*4 + n0 + 20][22] == 1'b1 &&
              kq[QN*4 + n0 + 19][33:32] == K_TAIL,
              "T4: urgent packet waits for the TAIL (lock not broken)");
        set_rates(100, 100);
        wait_idle(200);

        // ---------------------------------------------------------------
        // T5: credit stall
        // ---------------------------------------------------------------
        k_en[2] = 1'b0;                // EAST sink frees nothing
        n0 = k_rx[2];
        sent0 = sq_rd[0];
        n1 = s_crx[0];
        add_pkt(0, 5, 12, 1'b0);       // LOCAL -> EAST, 12 flits
        repeat (100) @(negedge clk);
        check(k_rx[2] - n0 == DEPTH, "T5: exactly DEPTH flits leave without credits");
        check(dut.dbg_cred[8*2 +: 8] == 0, "T5: EAST credit counter is 0");
        check(dut.dbg_level[7:0] == DEPTH, "T5: LOCAL input FIFO full");
        check(sq_rd[0] - sent0 == 2 * DEPTH && s_cred[0] == 0,
              "T5: source stopped by its own credits after 2*DEPTH flits");
        check(s_crx[0] - n1 == DEPTH, "T5: in_credit pulsed once per flit that left");
        repeat (100) @(negedge clk);
        check(k_rx[2] - n0 == DEPTH && k_over == 0, "T5: still DEPTH after 100 more clocks");
        k_en[2] = 1'b1;
        wait_rx(2, n0 + 12, 200);
        check(k_rx[2] - n0 == 12 && sb_err == 0, "T5: all 12 flits arrive after release, none lost");
        wait_idle(200);

        // ---------------------------------------------------------------
        // T6: random stress on all ports
        // ---------------------------------------------------------------
        n0 = sb_pkts;
        n1 = total_pkts(1'b0);
        for (p = 0; p < 5; p = p + 1)
            for (i = 0; i < 400; i = i + 1) begin
                j = {$random(seed)} % 4;
                add_pkt(p, {$random(seed)} % 8,
                        (j == 0) ? 1 : 2 + {$random(seed)} % 7,
                        ({$random(seed)} % 5) == 0);
            end
        chaos = 1'b1;
        c = 0;
        while (sb_pkts < n0 + 2000 && c < 200000) begin @(negedge clk); c = c + 1; end
        chaos = 1'b0;
        k_en = 5'h1F;
        set_rates(100, 100);
        wait_idle(500);
        $display("T6: %0d random packets (%0d flits total so far) in %0d clocks",
                 sb_pkts - n0, sb_flits, c);
        check(sb_pkts - n0 == 2000 && total_pkts(1'b0) - n1 == 2000,
              "T6: all 2000 random packets delivered");
        check(sb_err == 0, "T6: unchanged, right output, in order, no interleave");
        check(k_over == 0, "T6: downstream buffers never over-filled");
        ok = 1;
        for (q = 0; q < 5; q = q + 1)
            for (p = 0; p < 5; p = p + 1) begin
                i = fptr[5*q + p];
                while (i < pk_n[p] && pk_out[PN*p + i] != q) i = i + 1;
                if (i != pk_n[p]) ok = 0;
            end
        check(ok, "T6: no packet missing in any (input, output) flow");

        // ---------------------------------------------------------------
        // T7: stray BODY flit at an idle input
        // ---------------------------------------------------------------
        n0 = sb_flits;
        n1 = s_crx[3];
        sq[QN*3 + sq_wr[3]] = {K_BODY, 32'hDEAD_BEEF};
        sq_wr[3] = sq_wr[3] + 1;
        repeat (20) @(negedge clk);
        check(sb_flits == n0 && s_crx[3] == n1 + 1 && dut.dbg_level[8*3 +: 8] == 0,
              "T7: stray BODY dropped, credit returned");
        add_pkt(3, 4, 3, 1'b0);
        repeat (20) @(negedge clk);
        check(sb_flits == n0 + 3 && sb_err == 0, "T7: next packet on that input is fine");

        // ---------------------------------------------------------------
        // T8: end state
        // ---------------------------------------------------------------
        wait_idle(200);
        ok = 1;
        for (p = 0; p < 5; p = p + 1)
            if (s_crx[p] != sq_rd[p] || s_cred[p] != DEPTH) ok = 0;
        check(ok, "T8: in_credit pulses = flits sent on every input");
        ok = 1;
        for (q = 0; q < 5; q = q + 1)
            if (dut.dbg_cred[8*q +: 8] != DEPTH || dut.dbg_level[8*q +: 8] != 0) ok = 0;
        check(ok, "T8: all credit counters = DEPTH, all FIFOs empty");
        check(sb_pkts == total_pkts(1'b0) && sb_err == 0 && k_over == 0,
              "T8: every packet delivered exactly once");
        $display("T8: %0d packets, %0d flits through the main router", sb_pkts, sb_flits);

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end

    // watchdog: never hang
    initial begin
        #(5_000_000);
        $display("FAIL: watchdog timeout (t=%0t)", $time);
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule
