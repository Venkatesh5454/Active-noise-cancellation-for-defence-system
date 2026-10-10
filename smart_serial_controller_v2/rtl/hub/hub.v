// =============================================================================
// hub.v  -  sensor hub (NoC node 1): runs up to 8 "tasks" on its own schedule
// -----------------------------------------------------------------------------
// The hub replaces the CPU for repetitive sensor work (slides 18-20).  Each
// task is one small transaction that the hub sends into the network, for
// example "every 100 ms ask the I2C node to read 2 bytes from the TMP2 at
// 0x4B and make a time-stamped RECORD for the DMA writer".
//
// Task table: 8 tasks x 4 words at 0x40 + 16*t + 4*w (docs/SPEC.md section 7)
//   W0 TASK_CFG [0] EN [3:1] dest [6:4] type (1 XFER_REQ, 0 DATA) [7] prio
//               [15:8] arg [19:16] wlen [23:20] rlen [24] RECORD [25] SEND_LAST
//               [26] ADDR_INC [27] TRIG_RECORDS [28] APPEND_REC
//   W1 / W2     write bytes 0..3 / 4..7 (byte 0 in [7:0])
//   W3 TIMING   [15:0] period (ms, or records), [31:16] phase (ms, 0 = one period)
//
// Scheduling (one small block per task, all 8 work in parallel):
//   * EN rising or a W3 write loads the countdown with phase (or period if
//     phase = 0).  Every ms_tick it counts down; when it reaches 0 the task
//     becomes "due" and the countdown reloads with the period.
//   * TRIG_RECORDS tasks instead count the RECORDs the hub sends; they become
//     due when PERIOD records were sent since the task last ran.
//   * HUB_CTRL[8+t] (write 1) makes an enabled task due at once (RUN_NOW).
//   * Due tasks run one at a time, the lowest index first.  HUB_CTRL.EN = 0
//     stops new tasks from starting (a running task still finishes).
//
// Running a task (the main state machine):
//   IDLE -> HDR -> PAY -> WAIT -> RESP -> (HDR -> PAY for the RECORD) -> IDLE
//   1. t_start = time_us, tag = {roll[4:0], t[2:0]}, roll counts every run
//   2. send the request: XFER_REQ payload [wtotal][rlen][w][last value][last
//      record] (SEND_LAST / APPEND_REC add the two last parts); a DATA task
//      sends the same bytes without the two length bytes and is then done
//   3. wait for the XFER_RESP with the same tag (everything else that arrives
//      is read and thrown away), or time out after HUB_TIMEOUT ms
//   4. status 0: last value = response data; if RECORD, send a 16-byte RECORD
//      {t_start, code, n, seq, data} to HUB_REC_DEST, then seq++, ev_record
//      status != 0 or time-out: ev_error and an error counter, no record
//
// Design choices (where the SPEC leaves room):
//   * Period 0 means "never periodic": the task then only runs by RUN_NOW or
//     (TRIG_RECORDS) never.  A phase with period 0 gives one delayed run.
//   * TRIG_RECORDS tasks ignore the phase field.
//   * A due flag is not a queue: if a task is due again before it ran, it
//     still runs only once.  Disabling a task (EN = 0) clears its due flag;
//     RUN_NOW is ignored for disabled tasks.  Countdowns keep running while
//     HUB_CTRL.EN = 0, so due tasks wait and start when EN returns.
//   * wlen 9..15 acts as 8 (only 8 write bytes exist).  rlen is sent as
//     written; only the first 8 response data bytes are kept (length 8).
//   * seq and ev_record move only when a RECORD packet is really sent.  A
//     good response of a task without RECORD only updates the last value and
//     the RESP_OK counter.
//   * Time-out: counted in ms_ticks from the moment the request was sent; it
//     fires at the (HUB_TIMEOUT+1)-th tick, so the hub waits at least
//     HUB_TIMEOUT ms (0 = about 1 ms).  HUB_LAST.status then reads 0xFF.
//   * A matching XFER_RESP with no status byte (length 0) counts as status 4.
//   * ADDR_INC adds 256 to the address in W1 bytes 1..3 when the task starts
//     (after the bytes for this run were copied), so the register already
//     shows the next address while the run is in progress.
//   * HUB_LAST (status/task) is written by every XFER_REQ run (OK, error or
//     time-out); DATA runs leave it alone.
//   * The RECORD packet carries the run's tag and arg = task index.
//
// Registers (region 0x600, reg_addr[7:0]):
//   0x00 HUB_CTRL     [0] EN; W1 [15:8] RUN_NOW (reads back the due flags)
//   0x04 HUB_STATUS   RO [0] busy [3:1] task [8] waiting for a response
//   0x08 HUB_REC_DEST [2:0] node for RECORD packets (reset 2)
//   0x0C HUB_TIMEOUT  [15:0] ms (reset 50)
//   0x10 HUB_SEQ      RO next record sequence number (reset 1)
//   0x14 HUB_LAST     RO [3:0] last length [15:8] status [23:16] task
//   0x18 HUB_LAST_LO  RO last value bytes 0..3     0x1C HUB_LAST_HI  bytes 4..7
//   0x20 REQ  0x24 RESP_OK  0x28 ERR  0x2C TMO  0x30 REC   (RO counters)
//   0x40.. task table
// =============================================================================
`timescale 1ns / 1ps
module hub (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] time_us,
    input  wire        ms_tick,
    input  wire [2:0]  my_id,
    // register port
    input  wire        reg_we,
    input  wire        reg_re,
    input  wire [7:0]  reg_addr,
    input  wire [31:0] reg_wdata,
    output reg  [31:0] reg_rdata,
    // from the network
    input  wire        in_valid,
    input  wire [33:0] in_flit,
    output wire        in_credit,
    // to the network
    output wire        out_valid,
    output wire [33:0] out_flit,
    input  wire        out_credit,
    // events
    output reg         ev_error,
    output reg         ev_record
);
    // packet types (SPEC 3.2); type 0 = DATA
    localparam [2:0] T_XFER = 3'd1, T_RESP = 3'd2, T_REC = 3'd3, T_ALARM = 3'd4;

    // main state machine
    localparam [2:0] S_IDLE = 3'd0, S_HDR = 3'd1, S_PAY = 3'd2,
                     S_WAIT = 3'd3, S_RESP = 3'd4;

    // =========================================================================
    // global registers
    // =========================================================================
    reg        hub_en;
    reg [2:0]  rec_dest;
    reg [15:0] timeout_ms;
    reg [15:0] seq;               // next record sequence number

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hub_en     <= 1'b0;
            rec_dest   <= 3'd2;
            timeout_ms <= 16'd50;
        end else if (reg_we) begin
            case (reg_addr[7:2])
                6'h00: hub_en     <= reg_wdata[0];
                6'h02: rec_dest   <= reg_wdata[2:0];
                6'h03: timeout_ms <= reg_wdata[15:0];
                default: ;
            endcase
        end
    end

    // =========================================================================
    // task table address decode: 0x40..0xBF, task = addr[7:4] - 4
    // =========================================================================
    wire       a_task   = (reg_addr[7:6] == 2'b01) || (reg_addr[7:6] == 2'b10);
    wire [3:0] a_t4     = reg_addr[7:4] - 4'd4;
    wire [2:0] a_t      = a_t4[2:0];
    wire [1:0] a_w      = reg_addr[3:2];
    wire [7:0] a_onehot = 8'd1 << a_t;
    wire       we_task  = reg_we && a_task;
    wire       we_ctrl  = reg_we && (reg_addr[7:2] == 6'h00);   // HUB_CTRL (RUN_NOW)

    // =========================================================================
    // signals between the task slots and the main state machine
    // =========================================================================
    reg  [2:0]     state;
    wire [8*29-1:0] cfg_v;        // W0 of every task (29 used bits)
    wire [8*32-1:0] w1_v, w2_v, w3_v;
    wire [7:0]     due_v;         // due flags
    wire           rec_evt;       // a RECORD packet was just sent

    // pick the lowest-numbered due task
    function [2:0] first_one(input [7:0] v);
        integer i;
        begin
            first_one = 3'd0;
            for (i = 7; i >= 0; i = i - 1)
                if (v[i]) first_one = i[2:0];
        end
    endfunction

    wire [2:0] pick     = first_one(due_v);
    wire       start_go = (state == S_IDLE) && hub_en && (due_v != 8'd0);
    wire [7:0] start_oh = start_go ? (8'd1 << pick) : 8'd0;

    // =========================================================================
    // the 8 task slots: configuration words + scheduler (countdown, due flag)
    // =========================================================================
    genvar gt;
    generate
        for (gt = 0; gt < 8; gt = gt + 1) begin : g_task
            reg  [28:0] cfg;
            reg  [31:0] w1, w2, w3;
            reg  [15:0] cd;           // ms countdown (0 = stopped)
            reg  [15:0] rc;           // records sent since this task last ran
            reg         due;

            wire we0 = we_task && a_onehot[gt] && (a_w == 2'd0);
            wire we1 = we_task && a_onehot[gt] && (a_w == 2'd1);
            wire we2 = we_task && a_onehot[gt] && (a_w == 2'd2);
            wire we3 = we_task && a_onehot[gt] && (a_w == 2'd3);

            wire        en     = cfg[0];
            wire        trig   = cfg[27];
            wire [15:0] period = w3[15:0];
            wire [15:0] phase  = w3[31:16];

            // (re)start the schedule: EN goes 0 -> 1, or W3 is written
            wire        en_rise  = we0 && reg_wdata[0] && !en;
            wire        load     = en_rise || we3;
            wire [15:0] new_per  = we3 ? reg_wdata[15:0]  : period;
            wire [15:0] new_ph   = we3 ? reg_wdata[31:16] : phase;
            wire [15:0] load_val = (new_ph != 16'd0) ? new_ph : new_per;

            wire        starting = start_oh[gt];
            wire [15:0] rc_inc   = (rc == 16'hFFFF) ? rc : rc + 16'd1;

            // the three ways to become due
            wire tick_due = ms_tick && en && !trig && (cd == 16'd1) && !load;
            wire rec_due  = rec_evt && en && trig && (period != 16'd0) &&
                            (rc_inc >= period) && !load;
            wire now_due  = we_ctrl && reg_wdata[8+gt] && en;

            // ADDR_INC: bytes 1..3 = big-endian address; +256 = +1 on bytes 1..2
            wire [15:0] a_next = {w1[15:8], w1[23:16]} + 16'd1;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cfg <= 29'd0;
                    w1  <= 32'd0;
                    w2  <= 32'd0;
                    w3  <= 32'd0;
                    cd  <= 16'd0;
                    rc  <= 16'd0;
                    due <= 1'b0;
                end else begin
                    // configuration words (the CPU always wins)
                    if (we0) cfg <= reg_wdata[28:0];
                    if (we1)
                        w1 <= reg_wdata;
                    else if (starting && cfg[26])
                        w1 <= {w1[31:24], a_next[7:0], a_next[15:8], w1[7:0]};
                    if (we2) w2 <= reg_wdata;
                    if (we3) w3 <= reg_wdata;

                    // ms countdown (timed tasks only)
                    if (load)
                        cd <= load_val;
                    else if (ms_tick && en && !trig && (cd != 16'd0))
                        cd <= (cd == 16'd1) ? period : cd - 16'd1;

                    // record counter (TRIG_RECORDS tasks)
                    if (load || starting)
                        rc <= 16'd0;
                    else if (rec_evt && en && trig)
                        rc <= rc_inc;

                    // due flag
                    if (we0 && !reg_wdata[0])
                        due <= 1'b0;                      // task switched off
                    else if (tick_due || rec_due || now_due)
                        due <= 1'b1;
                    else if (starting)
                        due <= 1'b0;
                end
            end

            assign cfg_v[29*gt +: 29] = cfg;
            assign w1_v[32*gt +: 32]  = w1;
            assign w2_v[32*gt +: 32]  = w2;
            assign w3_v[32*gt +: 32]  = w3;
            assign due_v[gt]          = due;
        end
    endgenerate

    // =========================================================================
    // network helpers
    // =========================================================================
    wire        tx_start;
    wire        tx_hdr_ready;
    wire [2:0]  tx_type, tx_dest;
    wire        tx_prio;
    wire [5:0]  tx_len;
    wire [7:0]  tx_arg;
    wire        tx_b_valid;
    wire [7:0]  tx_b_data;
    wire        tx_b_ready;
    reg  [7:0]  cur_tag;

    noc_pkt_tx #(.DEPTH(4)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(tx_start), .hdr_ready(tx_hdr_ready),
        .ptype(tx_type), .dest(tx_dest), .src(my_id), .prio(tx_prio),
        .len(tx_len), .tag(cur_tag), .arg(tx_arg),
        .b_valid(tx_b_valid), .b_data(tx_b_data), .b_ready(tx_b_ready),
        .out_valid(out_valid), .out_flit(out_flit), .out_credit(out_credit));

    wire        rx_hdr_valid;
    wire [2:0]  rx_type, rx_src;
    wire [5:0]  rx_len;
    wire [7:0]  rx_tag;
    wire        rx_b_valid;
    wire [7:0]  rx_b_data;
    wire        rx_b_last;

    // the hub always takes headers and bytes, so it never blocks the network
    noc_pkt_rx #(.DEPTH(4)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_flit(in_flit), .in_credit(in_credit),
        .hdr_valid(rx_hdr_valid), .ptype(rx_type), .dest(), .src(rx_src),
        .prio(), .len(rx_len), .tag(rx_tag), .arg(), .hdr_ready(1'b1),
        .b_valid(rx_b_valid), .b_data(rx_b_data), .b_last(rx_b_last),
        .b_ready(1'b1));

    // =========================================================================
    // current run
    // =========================================================================
    reg  [2:0]   cur_t;           // task being run
    reg  [28:0]  cur_cfg;         // copy of its W0 ...
    reg  [63:0]  cur_w;           // ... and of its 8 write bytes
    reg  [31:0]  t_start;         // time_us when it started
    reg  [4:0]   roll;            // rolling part of the tag
    reg          rec_mode;        // 1: sending the RECORD packet
    reg  [1:0]   seg;             // payload part: 0 lengths 1 w 2 last 3 record
    reg  [3:0]   idx;             // byte inside the part
    reg  [15:0]  tmo_cnt;         // ms ticks left before the time-out

    reg  [63:0]  last_val;        // last value (HUB_LAST_LO/HI)
    reg  [3:0]   last_len;
    reg  [7:0]   last_status;
    reg  [2:0]   last_task;
    reg  [127:0] last_rec;        // last RECORD sent, as its 16 bytes

    reg  [31:0]  cnt_req, cnt_ok, cnt_err, cnt_tmo, cnt_rec;

    // response being received
    reg          rsp_active;      // inside the matching response's payload
    reg          rsp_done;        // 1-clock pulse: the whole response is in
    reg  [5:0]   rsp_cnt;         // payload bytes taken so far
    reg  [7:0]   rsp_status;
    reg  [3:0]   rsp_n;           // data bytes kept (0..8)
    reg  [63:0]  rsp_data;
    reg  [2:0]   rsp_src;

    // ---- request payload sizes (from the copied W0) ----
    wire        is_xfer = (cur_cfg[6:4] == T_XFER);
    wire [3:0]  wl      = (cur_cfg[19:16] > 4'd8) ? 4'd8 : cur_cfg[19:16];
    wire [3:0]  ll      = cur_cfg[25] ? last_len : 4'd0;
    wire [4:0]  al      = cur_cfg[28] ? 5'd16 : 5'd0;
    wire [5:0]  wtotal  = {2'b00, wl} + {2'b00, ll} + {1'b0, al};
    wire [5:0]  req_len = wtotal + (is_xfer ? 6'd2 : 6'd0);

    // ---- header for noc_pkt_tx: the request, or the RECORD ----
    assign tx_start = (state == S_HDR) && tx_hdr_ready;
    assign tx_type  = rec_mode ? T_REC : cur_cfg[6:4];
    assign tx_dest  = rec_mode ? rec_dest : cur_cfg[3:1];
    assign tx_prio  = rec_mode ? 1'b0 : (cur_cfg[7] || (cur_cfg[6:4] == T_ALARM));
    assign tx_len   = rec_mode ? 6'd16 : req_len;
    assign tx_arg   = rec_mode ? {5'd0, cur_t} : cur_cfg[15:8];

    // ---- payload generator: walk the 4 parts, skip the empty ones ----
    reg [4:0] seg_len;
    always @(*) begin
        if (rec_mode)
            seg_len = (seg == 2'd3) ? 5'd16 : 5'd0;      // record = last_rec
        else begin
            case (seg)
                2'd0:    seg_len = is_xfer ? 5'd2 : 5'd0;
                2'd1:    seg_len = {1'b0, wl};
                2'd2:    seg_len = {1'b0, ll};
                default: seg_len = al;
            endcase
        end
    end

    reg [7:0] pay_byte;
    always @(*) begin
        case (seg)
            2'd0:    pay_byte = idx[0] ? {4'd0, cur_cfg[23:20]} : {2'd0, wtotal};
            2'd1:    pay_byte = cur_w[8*idx[2:0] +: 8];
            2'd2:    pay_byte = last_val[8*idx[2:0] +: 8];
            default: pay_byte = last_rec[8*idx +: 8];
        endcase
    end

    wire seg_empty = (seg_len == 5'd0);
    wire seg_end   = ({1'b0, idx} == seg_len - 5'd1);
    assign tx_b_valid = (state == S_PAY) && !seg_empty;
    assign tx_b_data  = pay_byte;
    wire b_take   = tx_b_valid && tx_b_ready;
    wire pay_done = (state == S_PAY) && (seg == 2'd3) && (seg_empty || (b_take && seg_end));
    assign rec_evt = pay_done && rec_mode;

    // ---- response matching ----
    wire rsp_match = rx_hdr_valid && (state == S_WAIT) &&
                     (rx_type == T_RESP) && (rx_tag == cur_tag);

    // engine code of the responding node (SPEC 3.3)
    function [7:0] src_code(input [2:0] s);
        begin
            case (s)
                3'd3:    src_code = 8'd1;     // UART
                3'd4:    src_code = 8'd2;     // SPI
                3'd5:    src_code = 8'd3;     // I2C
                3'd0:    src_code = 8'd4;     // serial engine
                default: src_code = 8'd0;
            endcase
        end
    endfunction

    // =========================================================================
    // main state machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            cur_t       <= 3'd0;
            cur_cfg     <= 29'd0;
            cur_w       <= 64'd0;
            cur_tag     <= 8'd0;
            t_start     <= 32'd0;
            roll        <= 5'd0;
            rec_mode    <= 1'b0;
            seg         <= 2'd0;
            idx         <= 4'd0;
            tmo_cnt     <= 16'd0;
            seq         <= 16'd1;
            last_val    <= 64'd0;
            last_len    <= 4'd0;
            last_status <= 8'd0;
            last_task   <= 3'd0;
            last_rec    <= 128'd0;
            cnt_req     <= 32'd0;
            cnt_ok      <= 32'd0;
            cnt_err     <= 32'd0;
            cnt_tmo     <= 32'd0;
            cnt_rec     <= 32'd0;
            ev_error    <= 1'b0;
            ev_record   <= 1'b0;
        end else begin
            ev_error  <= 1'b0;
            ev_record <= 1'b0;

            case (state)
                // ---- pick the lowest due task and copy its settings ----
                S_IDLE: if (start_go) begin
                    cur_t    <= pick;
                    cur_cfg  <= cfg_v[29*pick +: 29];
                    cur_w    <= {w2_v[32*pick +: 32], w1_v[32*pick +: 32]};
                    t_start  <= time_us;
                    cur_tag  <= {roll, pick};
                    roll     <= roll + 5'd1;
                    rec_mode <= 1'b0;
                    state    <= S_HDR;
                end

                // ---- hand the header to noc_pkt_tx ----
                S_HDR: if (tx_hdr_ready) begin
                    seg <= 2'd0;
                    idx <= 4'd0;
                    if (!rec_mode) cnt_req <= cnt_req + 32'd1;
                    // only a DATA task can have no payload at all
                    state <= (tx_len == 6'd0) ? S_IDLE : S_PAY;
                end

                // ---- give the payload bytes, part by part ----
                S_PAY: begin
                    if (seg_empty) begin
                        seg <= seg + 2'd1;
                        idx <= 4'd0;
                    end else if (b_take) begin
                        if (seg_end) begin
                            seg <= seg + 2'd1;
                            idx <= 4'd0;
                        end else begin
                            idx <= idx + 4'd1;
                        end
                    end

                    if (pay_done) begin
                        if (rec_mode) begin                 // RECORD is out
                            seq       <= seq + 16'd1;
                            cnt_rec   <= cnt_rec + 32'd1;
                            ev_record <= 1'b1;
                            rec_mode  <= 1'b0;
                            state     <= S_IDLE;
                        end else if (is_xfer) begin         // wait for the reply
                            tmo_cnt <= timeout_ms;
                            state   <= S_WAIT;
                        end else begin                      // DATA: no reply
                            state <= S_IDLE;
                        end
                    end
                end

                // ---- wait for the XFER_RESP with our tag ----
                S_WAIT: begin
                    if (rsp_match) begin
                        state <= S_RESP;
                    end else if (ms_tick) begin
                        if (tmo_cnt == 16'd0) begin         // time-out
                            cnt_tmo     <= cnt_tmo + 32'd1;
                            ev_error    <= 1'b1;
                            last_status <= 8'hFF;
                            last_task   <= cur_t;
                            state       <= S_IDLE;
                        end else begin
                            tmo_cnt <= tmo_cnt - 16'd1;
                        end
                    end
                end

                // ---- the response is coming in; judge it when complete ----
                S_RESP: if (rsp_done) begin
                    last_status <= rsp_status;
                    last_task   <= cur_t;
                    if (rsp_status == 8'd0) begin
                        cnt_ok <= cnt_ok + 32'd1;
                        if (rsp_n != 4'd0) begin
                            last_val <= rsp_data;
                            last_len <= rsp_n;
                        end
                        if (cur_cfg[24]) begin              // RECORD
                            last_rec <= {rsp_data, seq, 4'd0, rsp_n,
                                         src_code(rsp_src), t_start};
                            rec_mode <= 1'b1;
                            state    <= S_HDR;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        cnt_err  <= cnt_err + 32'd1;
                        ev_error <= 1'b1;
                        state    <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // receive side: every packet is read; only the matching response is kept
    // =========================================================================
    wire [2:0] rsp_di = rsp_cnt[2:0] - 3'd1;    // data byte index (cnt 1..8)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsp_active <= 1'b0;
            rsp_done   <= 1'b0;
            rsp_cnt    <= 6'd0;
            rsp_status <= 8'd0;
            rsp_n      <= 4'd0;
            rsp_data   <= 64'd0;
            rsp_src    <= 3'd0;
        end else begin
            rsp_done <= 1'b0;
            if (rsp_match) begin
                rsp_src  <= rx_src;
                rsp_data <= 64'd0;
                rsp_cnt  <= 6'd0;
                // n = length - 1 (the status byte), at most 8 bytes kept
                rsp_n    <= (rx_len == 6'd0) ? 4'd0 :
                            (rx_len > 6'd9)  ? 4'd8 : rx_len[3:0] - 4'd1;
                if (rx_len == 6'd0) begin
                    rsp_status <= 8'd4;          // no status byte: bad response
                    rsp_done   <= 1'b1;
                end else begin
                    rsp_active <= 1'b1;
                end
            end else if (rx_b_valid && rsp_active) begin
                if (rsp_cnt == 6'd0)
                    rsp_status <= rx_b_data;
                else if (rsp_cnt <= 6'd8)
                    rsp_data[8*rsp_di +: 8] <= rx_b_data;
                rsp_cnt <= rsp_cnt + 6'd1;
                if (rx_b_last) begin
                    rsp_active <= 1'b0;
                    rsp_done   <= 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // register read (combinational)
    // =========================================================================
    wire busy    = (state != S_IDLE);
    wire waiting = (state == S_WAIT) || (state == S_RESP);

    always @(*) begin
        reg_rdata = 32'd0;
        if (a_task) begin
            case (a_w)
                2'd0:    reg_rdata = {3'd0, cfg_v[29*a_t +: 29]};
                2'd1:    reg_rdata = w1_v[32*a_t +: 32];
                2'd2:    reg_rdata = w2_v[32*a_t +: 32];
                default: reg_rdata = w3_v[32*a_t +: 32];
            endcase
        end else if (reg_addr[7:6] == 2'b00) begin
            case (reg_addr[5:2])
                4'h0: reg_rdata = {16'd0, due_v, 7'd0, hub_en};
                4'h1: reg_rdata = {23'd0, waiting, 4'd0, cur_t, busy};
                4'h2: reg_rdata = {29'd0, rec_dest};
                4'h3: reg_rdata = {16'd0, timeout_ms};
                4'h4: reg_rdata = {16'd0, seq};
                4'h5: reg_rdata = {13'd0, last_task, last_status, 4'd0, last_len};
                4'h6: reg_rdata = last_val[31:0];
                4'h7: reg_rdata = last_val[63:32];
                4'h8: reg_rdata = cnt_req;
                4'h9: reg_rdata = cnt_ok;
                4'hA: reg_rdata = cnt_err;
                4'hB: reg_rdata = cnt_tmo;
                4'hC: reg_rdata = cnt_rec;
                default: reg_rdata = 32'd0;
            endcase
        end
    end
endmodule
