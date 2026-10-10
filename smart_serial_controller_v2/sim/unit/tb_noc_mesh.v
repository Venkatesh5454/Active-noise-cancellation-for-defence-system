// =============================================================================
// tb_noc_mesh.v  -  unit test: the 3x2 NoC mesh with real packet helpers,
//                   and a 4x4 mesh (IDW = 4) driven with raw flits
// -----------------------------------------------------------------------------
// Part A: noc_mesh with the SPEC parameters (W 3, H 2, IDW 3, DEST_LSB 26,
// PRIO_BIT 22, DEPTH 4).  Every node has a noc_pkt_tx (producer) and a
// noc_pkt_rx (consumer).  Each node owns a list of packets; a consumer
// checks every packet it gets against the list of the sender:
//   right node, header unchanged, payload unchanged, and in order per
//   (source, dest) flow.  Total received = total sent proves "exactly once".
//   A1 zero-load latency for all 36 (source, dest) pairs: 2 clocks per router
//   A2 slide 16: node 5 (I2C) -> node 1 (hub), a 3-byte XFER_RESP (2 hops)
//   A3 > 5000 random packets: all-to-all (dest = own node too), length
//      0..63, random prio/type/tag/arg, random producer gaps and consumer
//      stalls.  A deadlock watchdog stops the run if nothing arrives for
//      20000 clocks.
//   A4 hot spot: every node sends 100 packets to node 1
//   A5 end: every credit counter back to DEPTH, every FIFO empty, no flit
//      ever left the mesh on a boundary port
// Part B: 4x4 mesh, IDW 4, DEST_LSB 24, PRIO_BIT 22, raw flit sources and
//   sinks (the same models as tb_noc_router) with random rates and stalls,
//   3200 packets of 1..8 flits: every packet delivered once, unchanged, in
//   order per flow, never interleaved; all credits back to DEPTH.
// =============================================================================
`timescale 1ns / 1ps
module tb_noc_mesh;
    localparam DEPTH = 4;
    localparam FW    = 34;
    localparam W = 3, H = 2, N = 6;
    localparam PN = 1024;            // packets per node list (part A)
    localparam NA1 = 6;              // A1 packets per node (idx 0..5)
    localparam A3_END = 857;         // A3 packets: idx 6 (7 for node 5) .. 856
    localparam A4_END = 957;         // A4 hot spot: idx 857 .. 956
    localparam HOT = 1;              // hot-spot node
    localparam [1:0] K_HEAD = 2'b00, K_BODY = 2'b01, K_TAIL = 2'b10, K_SINGLE = 2'b11;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    integer seed;
    integer cyc = 0;
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

    // =========================================================================
    // Part A: 3x2 mesh + noc_pkt_tx / noc_pkt_rx at every node
    // =========================================================================
    wire [N-1:0]    tx_valid;
    wire [N*FW-1:0] tx_flit;
    wire [N-1:0]    tx_credit;
    wire [N-1:0]    rx_valid;
    wire [N*FW-1:0] rx_flit;
    wire [N-1:0]    rx_credit;

    noc_mesh #(.W(W), .H(H), .N(N), .IDW(3), .DEST_LSB(26), .PRIO_BIT(22),
               .DEPTH(DEPTH)) u_mesh (
        .clk(clk), .rst_n(rst_n),
        .tx_valid(tx_valid), .tx_flit(tx_flit), .tx_credit(tx_credit),
        .rx_valid(rx_valid), .rx_flit(rx_flit), .rx_credit(rx_credit));

    // ---------------- packet lists (node n, packet i at N*.. PN*n + i) ----
    reg [2:0] p_type [0:N*PN-1];
    reg [2:0] p_dest [0:N*PN-1];
    reg       p_prio [0:N*PN-1];
    reg [5:0] p_len  [0:N*PN-1];
    reg [7:0] p_tag  [0:N*PN-1];
    reg [7:0] p_arg  [0:N*PN-1];
    reg [7:0] p_pay  [0:N*PN*64-1];
    integer   p_cnt  [0:N-1];        // packets in the list of node n

    integer prod_limit [0:N-1];      // node n may send packets 0 .. limit-1
    integer prod_rate  [0:N-1];      // % chance per clock to offer data
    integer cons_rate  [0:N-1];      // % chance per clock to be ready
    reg     stall_en = 1'b0;         // random long consumer stalls
    integer tx_started [0:N-1];      // packets started at node n
    integer rx_done    [0:N-1];      // packets fully received at node n
    integer rx_bytes   [0:N-1];
    integer t_start    [0:N-1];      // clock of the last start at node n
    integer t_done     [0:N-1];      // clock of the last packet end at node n
    integer fptr [0:N*N-1];          // flow pointer, (src s, dest d) at N*s + d
    integer sb_err = 0;

    genvar n;
    generate
        for (n = 0; n < N; n = n + 1) begin : g_n
            localparam [2:0] MY = n;

            // ------------- producer -> noc_pkt_tx -------------
            reg        start = 1'b0;
            wire       hdr_ready;
            reg  [2:0] t_type = 3'd0, t_dest = 3'd0;
            reg        t_prio = 1'b0;
            reg  [5:0] t_len = 6'd0;
            reg  [7:0] t_tag = 8'd0, t_arg = 8'd0;
            reg        b_valid = 1'b0;
            reg  [7:0] b_data = 8'd0;
            wire       b_ready;

            noc_pkt_tx #(.DEPTH(DEPTH)) u_tx (
                .clk(clk), .rst_n(rst_n),
                .start(start), .hdr_ready(hdr_ready),
                .ptype(t_type), .dest(t_dest), .src(MY), .prio(t_prio),
                .len(t_len), .tag(t_tag), .arg(t_arg),
                .b_valid(b_valid), .b_data(b_data), .b_ready(b_ready),
                .out_valid(tx_valid[n]), .out_flit(tx_flit[FW*n +: FW]),
                .out_credit(tx_credit[n]));

            integer tx_pkt = 0;
            integer tx_idx = 0;
            reg     tx_phase = 1'b0;     // 0 header, 1 payload
            always @(posedge clk) begin
                if (rst_n) begin
                    if (!tx_phase) begin
                        if (start && hdr_ready) begin
                            start <= 1'b0;
                            t_start[n]    = cyc;
                            tx_started[n] = tx_started[n] + 1;
                            if (p_len[PN*n + tx_pkt] == 0) begin
                                tx_pkt <= tx_pkt + 1;
                            end else begin
                                tx_phase <= 1'b1;
                                tx_idx   <= 0;
                                b_valid  <= ({$random(seed)} % 100) < prod_rate[n];
                                b_data   <= p_pay[(PN*n + tx_pkt)*64];
                            end
                        end else if (!start && tx_pkt < prod_limit[n] &&
                                     ({$random(seed)} % 100) < prod_rate[n]) begin
                            start  <= 1'b1;
                            t_type <= p_type[PN*n + tx_pkt];
                            t_dest <= p_dest[PN*n + tx_pkt];
                            t_prio <= p_prio[PN*n + tx_pkt];
                            t_len  <= p_len[PN*n + tx_pkt];
                            t_tag  <= p_tag[PN*n + tx_pkt];
                            t_arg  <= p_arg[PN*n + tx_pkt];
                        end
                    end else begin
                        if (b_valid && b_ready) begin
                            if (tx_idx + 1 == p_len[PN*n + tx_pkt]) begin
                                b_valid  <= 1'b0;
                                tx_phase <= 1'b0;
                                tx_pkt   <= tx_pkt + 1;
                            end else begin
                                tx_idx  <= tx_idx + 1;
                                b_valid <= ({$random(seed)} % 100) < prod_rate[n];
                                b_data  <= p_pay[(PN*n + tx_pkt)*64 + tx_idx + 1];
                            end
                        end else begin
                            b_valid <= ({$random(seed)} % 100) < prod_rate[n];
                            b_data  <= p_pay[(PN*n + tx_pkt)*64 + tx_idx];
                        end
                    end
                end
            end

            // ------------- noc_pkt_rx -> consumer + scoreboard -------------
            wire       r_hv;
            wire [2:0] r_type, r_dest, r_src;
            wire       r_prio;
            wire [5:0] r_len;
            wire [7:0] r_tag, r_arg;
            reg        r_hready = 1'b0;
            wire       r_bv;
            wire [7:0] r_bd;
            wire       r_blast;
            reg        r_bready = 1'b0;

            noc_pkt_rx #(.DEPTH(DEPTH)) u_rx (
                .clk(clk), .rst_n(rst_n),
                .in_valid(rx_valid[n]), .in_flit(rx_flit[FW*n +: FW]),
                .in_credit(rx_credit[n]),
                .hdr_valid(r_hv), .ptype(r_type), .dest(r_dest), .src(r_src),
                .prio(r_prio), .len(r_len), .tag(r_tag), .arg(r_arg),
                .hdr_ready(r_hready),
                .b_valid(r_bv), .b_data(r_bd), .b_last(r_blast), .b_ready(r_bready));

            reg     rx_phase = 1'b0;
            reg     pkt_ok = 1'b1;
            integer rx_idx = 0;
            integer cur_len = 0;
            integer cur_base = 0;        // payload index of the expected packet
            integer stall = 0;
            always @(posedge clk) begin : cons
                integer s, i, e;
                if (rst_n) begin
                    if (!rx_phase) begin
                        if (r_hv && r_hready) begin
                            // which packet of sender s should this be?
                            s = r_src;
                            i = 0;
                            if (s < N) begin
                                i = fptr[N*s + n];
                                while (i < p_cnt[s] && p_dest[PN*s + i] != n) i = i + 1;
                            end
                            if (s >= N || i >= p_cnt[s]) begin
                                pkt_ok = 1'b0;
                                cur_base = 0;
                            end else begin
                                fptr[N*s + n] = i + 1;
                                e = PN*s + i;
                                pkt_ok = (r_dest == n) && (r_type == p_type[e]) &&
                                         (r_dest == p_dest[e]) && (r_prio == p_prio[e]) &&
                                         (r_len  == p_len[e])  && (r_tag  == p_tag[e])  &&
                                         (r_arg  == p_arg[e]);
                                cur_base = e * 64;
                            end
                            if (!pkt_ok)
                                $display("  node %0d: header mismatch src %0d len %0d tag %h (t=%0t)",
                                         n, s, r_len, r_tag, $time);
                            cur_len = r_len;
                            if (r_len == 0) begin
                                if (!pkt_ok) sb_err = sb_err + 1;
                                rx_done[n] = rx_done[n] + 1;
                                t_done[n]  = cyc;
                            end else begin
                                rx_phase <= 1'b1;
                                rx_idx   <= 0;
                            end
                        end
                    end else begin
                        if (r_bv && r_bready) begin
                            if (r_bd !== p_pay[cur_base + rx_idx] ||
                                r_blast !== (rx_idx + 1 == cur_len)) begin
                                if (pkt_ok)
                                    $display("  node %0d: byte %0d wrong: %h (t=%0t)",
                                             n, rx_idx, r_bd, $time);
                                pkt_ok = 1'b0;
                            end
                            rx_bytes[n] = rx_bytes[n] + 1;
                            if (rx_idx + 1 == cur_len) begin
                                if (!pkt_ok) sb_err = sb_err + 1;
                                rx_done[n] = rx_done[n] + 1;
                                t_done[n]  = cyc;
                                rx_phase <= 1'b0;
                            end else begin
                                rx_idx <= rx_idx + 1;
                            end
                        end
                    end
                    // ready signals: random gaps plus rare long stalls
                    if (stall > 0)
                        stall = stall - 1;
                    else if (stall_en && ({$random(seed)} % 400) == 0)
                        stall = {$random(seed)} % 100;
                    r_hready <= (stall == 0) && (({$random(seed)} % 100) < cons_rate[n]);
                    r_bready <= (stall == 0) && (({$random(seed)} % 100) < cons_rate[n]);
                end
            end
        end
    endgenerate

    // ---------------- link monitor: injection / ejection times ----------------
    integer inj_head [0:N-1];        // clock the last HEAD/SINGLE of node s entered
    integer inj_tail [0:N-1];        // clock the last TAIL of node s entered
    integer ej_head  [0:N-1];        // clock the last HEAD/SINGLE left at node d
    integer ej_tail  [0:N-1];        // clock the last TAIL left at node d
    integer lat_head [0:N-1];        // network latency of that head
    integer flits_in = 0, flits_out = 0;
    always @(posedge clk) begin : linkmon
        integer s, d;
        reg [FW-1:0] f;
        for (s = 0; s < N; s = s + 1) begin
            if (tx_valid[s]) begin
                f = tx_flit[FW*s +: FW];
                flits_in = flits_in + 1;
                if (f[33:32] == K_HEAD || f[33:32] == K_SINGLE) inj_head[s] = cyc;
                if (f[33:32] == K_TAIL) inj_tail[s] = cyc;
            end
        end
        for (d = 0; d < N; d = d + 1) begin
            if (rx_valid[d]) begin
                f = rx_flit[FW*d +: FW];
                flits_out = flits_out + 1;
                if (f[33:32] == K_HEAD || f[33:32] == K_SINGLE) begin
                    ej_head[d]  = cyc;
                    lat_head[d] = cyc - inj_head[f[25:23]];
                end
                if (f[33:32] == K_TAIL) ej_tail[d] = cyc;
            end
        end
    end

    // flits must never leave the mesh on a boundary port
    reg  [5*N-1:0] bnd_mask;
    integer bnd_flits = 0;
    always @(posedge clk) if ((u_mesh.r_out_valid & bnd_mask) != 0) bnd_flits = bnd_flits + 1;

    // debug views of every router: credits and FIFO levels
    wire [N*40-1:0] a_cred, a_level;
    generate
        for (n = 0; n < N; n = n + 1) begin : g_dbg
            assign a_cred[40*n +: 40]  = u_mesh.g_node[n].u_router.dbg_cred;
            assign a_level[40*n +: 40] = u_mesh.g_node[n].u_router.dbg_level;
        end
    endgenerate

    // =========================================================================
    // Part B: 4x4 mesh with raw flit sources and sinks
    // =========================================================================
    localparam W4 = 4, H4 = 4, N4 = 16;
    localparam Q4 = 2048;            // flits per source queue
    localparam P4 = 256;             // packets per source

    reg  [N4-1:0]    m4_tv = {N4{1'b0}};
    reg  [N4*FW-1:0] m4_tf = {N4*FW{1'b0}};
    wire [N4-1:0]    m4_tc;
    wire [N4-1:0]    m4_rv;
    wire [N4*FW-1:0] m4_rf;
    reg  [N4-1:0]    m4_rc = {N4{1'b0}};

    noc_mesh #(.W(W4), .H(H4), .N(N4), .IDW(4), .DEST_LSB(24), .PRIO_BIT(22),
               .DEPTH(DEPTH)) u_mesh4 (
        .clk(clk), .rst_n(rst_n),
        .tx_valid(m4_tv), .tx_flit(m4_tf), .tx_credit(m4_tc),
        .rx_valid(m4_rv), .rx_flit(m4_rf), .rx_credit(m4_rc));

    reg  [FW-1:0] q4 [0:N4*Q4-1];    // flit queue of node s at [Q4*s ..]
    integer q4_wr [0:N4-1];
    integer q4_rd [0:N4-1];
    integer c4    [0:N4-1];          // source credit counters
    integer c4rx  [0:N4-1];          // tx_credit pulses seen
    integer s4_rate [0:N4-1];
    integer pk4_n [0:N4-1];
    integer pk4_start [0:N4*P4-1];
    integer pk4_len   [0:N4*P4-1];
    integer pk4_dest  [0:N4*P4-1];

    always @(posedge clk) begin : src4
        integer s;
        reg go;
        for (s = 0; s < N4; s = s + 1) begin
            go = 1'b0;
            if (rst_n && q4_rd[s] < q4_wr[s] && c4[s] > 0)
                go = ({$random(seed)} % 100) < s4_rate[s];
            m4_tv[s] <= go;
            if (go) begin
                m4_tf[FW*s +: FW] <= q4[Q4*s + q4_rd[s]];
                q4_rd[s] = q4_rd[s] + 1;
            end
            c4[s] = c4[s] - go + m4_tc[s];
            if (m4_tc[s]) c4rx[s] = c4rx[s] + 1;
        end
    end

    // HEAD data = {src[31:28], dest[27:24], x, prio[22], x[21:16], number[15:0]}
    task add_pkt4(input integer s, input integer dest, input integer nfl, input prio);
        integer i, k, a;
        reg [FW-1:0] f;
        begin
            k = pk4_n[s];
            a = q4_wr[s];
            pk4_start[P4*s + k] = a;
            pk4_len[P4*s + k]   = nfl;
            pk4_dest[P4*s + k]  = dest;
            f = {(nfl == 1) ? K_SINGLE : K_HEAD, $random(seed)};
            f[31:28] = s;
            f[27:24] = dest;
            f[22]    = prio;
            f[15:0]  = k;
            q4[Q4*s + a] = f;
            for (i = 1; i < nfl; i = i + 1)
                q4[Q4*s + a + i] = {(i == nfl - 1) ? K_TAIL : K_BODY, $random(seed)};
            pk4_n[s] = k + 1;
            q4_wr[s] = a + nfl;
        end
    endtask

    reg  [N4-1:0] k4_en = {N4{1'b1}};
    integer k4_rate [0:N4-1];
    integer k4_cnt  [0:N4-1];        // modelled node buffer (DEPTH flits)
    integer cs4 [0:N4-1];            // packet in progress at node d: source
    integer cp4 [0:N4-1];            //   next flit position in its queue
    integer cl4 [0:N4-1];            //   flits still to come
    integer f4  [0:N4*N4-1];         // flow pointer (src s, dest d) at N4*s + d
    integer sb4_err = 0, sb4_pkts = 0, sb4_flits = 0, k4_over = 0;

    always @(posedge clk) begin : sink4
        integer d, s, i;
        reg [FW-1:0] f;
        reg cons;
        for (d = 0; d < N4; d = d + 1) begin
            cons = 1'b0;
            if (rst_n && k4_cnt[d] > 0 && k4_en[d])
                cons = ({$random(seed)} % 100) < k4_rate[d];
            m4_rc[d] <= cons;
            k4_cnt[d] = k4_cnt[d] - cons;
            if (m4_rv[d]) begin
                f = m4_rf[FW*d +: FW];
                k4_cnt[d] = k4_cnt[d] + 1;
                if (k4_cnt[d] > DEPTH) k4_over = k4_over + 1;
                sb4_flits = sb4_flits + 1;
                if (f[33:32] == K_HEAD || f[33:32] == K_SINGLE) begin
                    if (cl4[d] != 0) begin
                        sb4_err = sb4_err + 1;
                        $display("  4x4 node %0d: packets interleaved (t=%0t)", d, $time);
                    end
                    s = f[31:28];
                    i = f4[N4*s + d];
                    while (i < pk4_n[s] && pk4_dest[P4*s + i] != d) i = i + 1;
                    if (i >= pk4_n[s] || i != f[15:0] ||
                        f !== q4[Q4*s + pk4_start[P4*s + i]]) begin
                        sb4_err = sb4_err + 1;
                        cl4[d] = 0;
                        $display("  4x4 node %0d: unexpected head %h (t=%0t)", d, f, $time);
                    end else begin
                        f4[N4*s + d] = i + 1;
                        cs4[d] = s;
                        cp4[d] = pk4_start[P4*s + i] + 1;
                        cl4[d] = pk4_len[P4*s + i] - 1;
                        if (cl4[d] == 0) sb4_pkts = sb4_pkts + 1;
                    end
                end else begin
                    if (cl4[d] == 0 || f !== q4[Q4*cs4[d] + cp4[d]]) begin
                        sb4_err = sb4_err + 1;
                        $display("  4x4 node %0d: payload flit wrong %h (t=%0t)", d, f, $time);
                    end
                    if (cl4[d] != 0) begin
                        cp4[d] = cp4[d] + 1;
                        cl4[d] = cl4[d] - 1;
                        if (cl4[d] == 0) sb4_pkts = sb4_pkts + 1;
                    end
                end
            end
        end
    end

    // random rates and stalls while chaos4 is on
    reg chaos4 = 1'b0;
    always @(posedge clk) begin : chaos_gen
        integer s;
        if (chaos4 && (cyc % 32) == 0) begin
            for (s = 0; s < N4; s = s + 1) begin
                k4_en[s]   = ({$random(seed)} % 5) != 0;
                k4_rate[s] = 20 + {$random(seed)} % 81;
                s4_rate[s] = 20 + {$random(seed)} % 81;
            end
        end
    end

    wire [N4*40-1:0] b_cred, b_level;
    generate
        for (n = 0; n < N4; n = n + 1) begin : g_dbg4
            assign b_cred[40*n +: 40]  = u_mesh4.g_node[n].u_router.dbg_cred;
            assign b_level[40*n +: 40] = u_mesh4.g_node[n].u_router.dbg_level;
        end
    endgenerate

    // =========================================================================
    // Deadlock watchdog: something must arrive at least every 20000 clocks
    // while packets are outstanding
    // =========================================================================
    integer last_prog = 0, prog_prev = -1;
    reg     wd_on = 1'b0;
    always @(posedge clk) begin : wdog
        integer d, tot, busy;
        tot = sb4_flits;
        for (d = 0; d < N; d = d + 1) tot = tot + rx_done[d] + rx_bytes[d];
        // outstanding work: part A packets not yet received, part B flits queued
        busy = (sum_done(1'b0) != sum_started(1'b0));
        for (d = 0; d < N4; d = d + 1) if (q4_rd[d] != q4_wr[d] || cl4[d] != 0) busy = 1;
        if (tot != prog_prev || !wd_on || !busy) begin
            prog_prev = tot;
            last_prog = cyc;
        end else if (cyc - last_prog > 20000) begin
            $display("FAIL: DEADLOCK - nothing delivered for 20000 clocks (t=%0t)", $time);
            for (d = 0; d < N; d = d + 1)
                $display("  node %0d: started %0d of %0d, received %0d",
                         d, tx_started[d], prod_limit[d], rx_done[d]);
            $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
            $finish;
        end
    end

    // =========================================================================
    // Helpers
    // =========================================================================
    function integer sum_started;
        input dummy;
        integer d;
        begin
            sum_started = 0;
            for (d = 0; d < N; d = d + 1) sum_started = sum_started + prod_limit[d];
        end
    endfunction

    function integer sum_done;
        input dummy;
        integer d;
        begin
            sum_done = 0;
            for (d = 0; d < N; d = d + 1) sum_done = sum_done + rx_done[d];
        end
    endfunction

    function integer hops;
        input integer s, d;
        integer dx, dy;
        begin
            dx = (s % W) - (d % W);
            dy = (s / W) - (d / W);
            hops = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
        end
    endfunction

    // wait until every sent packet has been received (or maxc clocks)
    task wait_all(input integer maxc, output integer used);
        begin
            used = 0;
            while (sum_done(1'b0) < sum_started(1'b0) && used < maxc) begin
                @(negedge clk);
                used = used + 1;
            end
        end
    endtask

    task set_rates(input integer pr, input integer cr);
        integer d;
        begin
            for (d = 0; d < N; d = d + 1) begin
                prod_rate[d] = pr;
                cons_rate[d] = cr;
            end
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    integer i, j, s, d, e, c, ok, nb, n0, t0, tot_pk, tot_by;
    integer lat_tab [0:N*N-1];
    integer hop_lat [0:3];
    initial begin
        seed = 4242;
        // ---------------- build the packet lists ----------------
        for (s = 0; s < N; s = s + 1) begin
            p_cnt[s] = A4_END;
            prod_limit[s] = 0; tx_started[s] = 0; rx_done[s] = 0; rx_bytes[s] = 0;
            t_start[s] = 0; t_done[s] = 0;
            inj_head[s] = 0; inj_tail[s] = 0; ej_head[s] = 0; ej_tail[s] = 0;
            lat_head[s] = 0;
            for (d = 0; d < N; d = d + 1) fptr[N*s + d] = 0;
            for (i = 0; i < A4_END; i = i + 1) begin
                e = PN*s + i;
                p_type[e] = $random(seed);
                p_prio[e] = ({$random(seed)} % 4) == 0;
                p_tag[e]  = $random(seed);
                p_arg[e]  = $random(seed);
                p_len[e]  = $random(seed);
                p_dest[e] = (i < A3_END) ? {$random(seed)} % N : HOT;
                for (j = 0; j < 64; j = j + 1) p_pay[e*64 + j] = $random(seed);
                if (i < NA1) begin           // A1: one empty packet to each node
                    p_dest[e] = i;
                    p_len[e]  = 0;
                    p_prio[e] = 1'b0;
                end
            end
        end
        // A2 (slide 16): I2C node 5 -> hub node 1, XFER_RESP [status 0][r0][r1]
        e = PN*5 + 6;
        p_type[e] = 3'd2; p_dest[e] = 3'd1; p_prio[e] = 1'b0; p_len[e] = 6'd3;
        p_pay[e*64] = 8'h00; p_pay[e*64 + 1] = 8'h1A; p_pay[e*64 + 2] = 8'h80;
        // corner lengths early in A3
        for (s = 0; s < N; s = s + 1) begin
            p_len[PN*s + 10] = 1;  p_len[PN*s + 11] = 4;  p_len[PN*s + 12] = 5;
            p_len[PN*s + 13] = 63; p_len[PN*s + 14] = 0;  p_len[PN*s + 15] = 63;
            p_dest[PN*s + 16] = s; p_len[PN*s + 16] = 63;  // to itself
        end
        // boundary output ports of the 3x2 mesh
        bnd_mask = {5*N{1'b0}};
        for (d = 0; d < N; d = d + 1) begin
            if (d / W == 0)     bnd_mask[5*d + 1] = 1'b1;   // NORTH
            if (d % W == W - 1) bnd_mask[5*d + 2] = 1'b1;   // EAST
            if (d / W == H - 1) bnd_mask[5*d + 3] = 1'b1;   // SOUTH
            if (d % W == 0)     bnd_mask[5*d + 4] = 1'b1;   // WEST
        end
        // part B state
        for (s = 0; s < N4; s = s + 1) begin
            q4_wr[s] = 0; q4_rd[s] = 0; c4[s] = DEPTH; c4rx[s] = 0; s4_rate[s] = 100;
            pk4_n[s] = 0; k4_rate[s] = 100; k4_cnt[s] = 0;
            cs4[s] = 0; cp4[s] = 0; cl4[s] = 0;
            for (d = 0; d < N4; d = d + 1) f4[N4*s + d] = 0;
        end

        set_rates(100, 100);
        #33 rst_n = 1'b1;
        repeat (3) @(negedge clk);
        wd_on = 1'b1;

        // ---------------------------------------------------------------
        // A1: zero-load latency, one packet at a time
        // ---------------------------------------------------------------
        for (i = 0; i < 4; i = i + 1) hop_lat[i] = -1;
        ok = 1;
        for (s = 0; s < N; s = s + 1)
            for (d = 0; d < N; d = d + 1) begin
                nb = rx_done[d];
                prod_limit[s] = d + 1;
                c = 0;
                while (rx_done[d] == nb && c < 200) begin @(negedge clk); c = c + 1; end
                repeat (4) @(negedge clk);
                lat_tab[N*s + d] = lat_head[d];
                if (rx_done[d] != nb + 1 || lat_head[d] != 2 * (hops(s, d) + 1)) ok = 0;
                if (hop_lat[hops(s, d)] < 0) hop_lat[hops(s, d)] = lat_head[d];
                else if (hop_lat[hops(s, d)] != lat_head[d]) ok = 0;
            end
        check(ok, "A1: zero-load latency = 2 clocks per router for all 36 pairs");
        check(sb_err == 0 && sum_done(1'b0) == 36, "A1: 36 single-flit packets delivered");
        $display("A1: zero-load network latency (head flit: tx_valid -> rx_valid), clocks:");
        $display("      dest:  0  1  2  3  4  5");
        for (s = 0; s < N; s = s + 1)
            $display("    src %0d:  %0d  %0d  %0d  %0d  %0d  %0d", s,
                     lat_tab[N*s], lat_tab[N*s+1], lat_tab[N*s+2],
                     lat_tab[N*s+3], lat_tab[N*s+4], lat_tab[N*s+5]);
        $display("A1: hops 0/1/2/3 -> %0d/%0d/%0d/%0d clocks = 2 + 2 per hop (2 per router)",
                 hop_lat[0], hop_lat[1], hop_lat[2], hop_lat[3]);
        check(hop_lat[1] - hop_lat[0] <= 3 && hop_lat[3] - hop_lat[2] <= 3,
              "A1: per-hop latency <= 3 clocks");

        // ---------------------------------------------------------------
        // A2: slide 16 path, node 5 -> node 1, 3-byte reply
        // ---------------------------------------------------------------
        nb = rx_done[1];
        prod_limit[5] = 7;
        c = 0;
        while (rx_done[1] == nb && c < 200) begin @(negedge clk); c = c + 1; end
        repeat (4) @(negedge clk);
        check(rx_done[1] == nb + 1 && sb_err == 0, "A2: 3-byte XFER_RESP 5 -> 1 delivered");
        check(ej_tail[1] - inj_tail[5] == 6 && lat_head[1] == 6,
              "A2: head and tail each take 6 clocks (3 routers, 2 hops)");
        $display("A2: node 5 -> node 1 (2 hops), 3-byte XFER_RESP = HEAD + TAIL:");
        $display("    head in -> head out %0d clocks, head in -> tail out %0d clocks",
                 lat_head[1], ej_tail[1] - inj_head[5]);
        $display("    start at node 5 -> last byte taken at node 1: %0d clocks",
                 t_done[1] - t_start[5]);

        // ---------------------------------------------------------------
        // A3: long random all-to-all run
        // ---------------------------------------------------------------
        n0 = sum_done(1'b0);
        nb = flits_in;
        for (s = 0; s < N; s = s + 1) begin
            prod_rate[s] = 50 + {$random(seed)} % 51;
            cons_rate[s] = 60 + {$random(seed)} % 41;
        end
        stall_en = 1'b1;
        t0 = cyc;
        for (s = 0; s < N; s = s + 1) prod_limit[s] = A3_END;
        wait_all(2000000, c);
        tot_pk = sum_done(1'b0) - n0;
        $display("A3: %0d random packets, %0d flits, in %0d clocks", tot_pk,
                 flits_in - nb, cyc - t0);
        check(tot_pk == sum_started(1'b0) - n0 && tot_pk >= 5000,
              "A3: >= 5000 packets, every one arrived");
        check(sb_err == 0, "A3: right node, unchanged, in order per flow");

        // ---------------------------------------------------------------
        // A4: hot spot, all nodes -> node 1
        // ---------------------------------------------------------------
        stall_en = 1'b0;
        set_rates(100, 100);
        repeat (200) @(negedge clk);
        n0 = sum_done(1'b0);
        nb = flits_out;
        tot_by = rx_bytes[HOT];
        t0 = cyc;
        for (s = 0; s < N; s = s + 1) prod_limit[s] = A4_END;
        wait_all(200000, c);
        tot_pk = sum_done(1'b0) - n0;
        $display("A4: hot spot: %0d packets (%0d bytes) into node %0d in %0d clocks = %0d.%02d bytes/clock",
                 tot_pk, rx_bytes[HOT] - tot_by, HOT, cyc - t0,
                 (rx_bytes[HOT] - tot_by) / (cyc - t0),
                 ((rx_bytes[HOT] - tot_by) * 100 / (cyc - t0)) % 100);
        check(tot_pk == 6 * (A4_END - A3_END), "A4: all 600 hot-spot packets arrived");
        check(sb_err == 0, "A4: hot-spot packets unchanged and in order");

        // ---------------------------------------------------------------
        // A5: end state of the 3x2 mesh
        // ---------------------------------------------------------------
        repeat (50) @(negedge clk);
        ok = 1;
        for (i = 0; i < 5 * N; i = i + 1)
            if (a_cred[8*i +: 8] != DEPTH || a_level[8*i +: 8] != 0) ok = 0;
        check(ok, "A5: all 30 router credit counters = DEPTH, FIFOs empty");
        check(g_n[0].u_tx.cred == DEPTH && g_n[1].u_tx.cred == DEPTH &&
              g_n[2].u_tx.cred == DEPTH && g_n[3].u_tx.cred == DEPTH &&
              g_n[4].u_tx.cred == DEPTH && g_n[5].u_tx.cred == DEPTH,
              "A5: all noc_pkt_tx credit counters = DEPTH");
        check(flits_in == flits_out, "A5: flits into the mesh = flits out of it");
        check(bnd_flits == 0, "A5: no flit ever left on a boundary port");
        ok = 1;
        for (s = 0; s < N; s = s + 1)
            for (d = 0; d < N; d = d + 1) begin
                i = fptr[N*s + d];
                while (i < A4_END && p_dest[PN*s + i] != d) i = i + 1;
                if (i != A4_END) ok = 0;
            end
        check(ok && sum_done(1'b0) == N * A4_END, "A5: every packet of every flow received once");
        $display("A: %0d packets, %0d flits through the 3x2 mesh", sum_done(1'b0), flits_out);

        // ---------------------------------------------------------------
        // B: 4x4 mesh, IDW 4, raw flits
        // ---------------------------------------------------------------
        for (s = 0; s < N4; s = s + 1)
            for (i = 0; i < 200; i = i + 1) begin
                j = {$random(seed)} % 4;
                add_pkt4(s, {$random(seed)} % N4,
                         (j == 0) ? 1 : 2 + {$random(seed)} % 7,
                         ({$random(seed)} % 5) == 0);
            end
        t0 = cyc;
        chaos4 = 1'b1;
        c = 0;
        while (sb4_pkts < N4 * 200 && c < 400000) begin @(negedge clk); c = c + 1; end
        chaos4 = 1'b0;
        k4_en = {N4{1'b1}};
        for (s = 0; s < N4; s = s + 1) begin s4_rate[s] = 100; k4_rate[s] = 100; end
        repeat (100) @(negedge clk);
        $display("B: 4x4 mesh: %0d packets, %0d flits in %0d clocks", sb4_pkts, sb4_flits, c);
        check(sb4_pkts == N4 * 200, "B: all 3200 packets delivered");
        check(sb4_err == 0, "B: unchanged, right node, in order per flow, no interleave");
        check(k4_over == 0, "B: node buffers never over-filled");
        ok = 1;
        for (s = 0; s < N4; s = s + 1)
            for (d = 0; d < N4; d = d + 1) begin
                i = f4[N4*s + d];
                while (i < pk4_n[s] && pk4_dest[P4*s + i] != d) i = i + 1;
                if (i != pk4_n[s]) ok = 0;
            end
        check(ok, "B: no packet missing in any of the 256 flows");
        ok = 1;
        for (i = 0; i < 5 * N4; i = i + 1)
            if (b_cred[8*i +: 8] != DEPTH || b_level[8*i +: 8] != 0) ok = 0;
        for (s = 0; s < N4; s = s + 1)
            if (c4[s] != DEPTH || c4rx[s] != q4_rd[s] || q4_rd[s] != q4_wr[s]) ok = 0;
        check(ok, "B: all 80 router + 16 source credit counters = DEPTH");

        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end

    // watchdog: never hang
    initial begin
        #(40_000_000);
        $display("FAIL: watchdog timeout (t=%0t)", $time);
        $display("%0d OF %0d CHECKS FAILED", errors + 1, checks + 1);
        $finish;
    end
endmodule
