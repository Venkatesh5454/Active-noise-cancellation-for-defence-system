// =============================================================================
// tb_noc_router_equiv.v  -  unit test: the timing-optimised noc_router must
// behave exactly like the simple reference router, clock by clock
// -----------------------------------------------------------------------------
// rtl/noc/noc_router.v keeps a register copy of each FIFO front, works out
// the route when a flit is written and uses one-hot arbiters, so that it
// meets 100 MHz.  sim/models/noc_router_ref.v is the first, simple version
// (front read straight from the FIFO memory).  Both get the same inputs and
// every output (out_valid, out_flit, in_credit) is compared at every clock.
//
// Six pairs run at once, one for each router position of the 3 x 2 mesh
// (the route table depends on X, Y).  Each pair has its own random stimulus,
// in phases of 2000 clocks with a new mix each time:
//   mode A  anything goes: random in_valid and out_credit (FIFO overrun,
//           credit counters that wrap, stray BODY/TAIL flits)
//   mode B  like a real network: senders obey the credits, a downstream
//           buffer model returns credits, so outputs stay locked and FIFOs
//           stay full for long stretches
// with random flit kinds, dest values (0..7, also the unused 6 and 7),
// a random share of prio flits, and now and then a reset in mid-traffic.
//
// Checks per position: no mismatch, traffic really flowed, outputs really
// locked.  Default 50 000 clocks per position (about a minute in Icarus);
// for a longer run:  iverilog ... -P tb_noc_router_equiv.CYCLES=1000000
// =============================================================================
`timescale 1ns / 1ps
module tb_noc_router_equiv;
    parameter CYCLES = 50000;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    integer checks = 0;
    integer errors = 0;
    task check(input cond, input [8*64-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL: %0s (t=%0t)", what, $time);
            end
        end
    endtask

    wire [5:0] done;
    genvar gp;
    generate
        for (gp = 0; gp < 6; gp = gp + 1) begin : g_pos
            pair_run #(.X(gp % 3), .Y(gp / 3), .SEED(1000 + 77 * gp), .CYCLES(CYCLES))
                u (.clk(clk), .done(done[gp]));
        end
    endgenerate

    initial begin
        wait (&done);
        @(negedge clk);
        `define POS_CHECK(i) \
            $display("(%0d,%0d): %0d clocks, %0d with flits out, %0d with a locked output, %0d mismatches", \
                     g_pos[i].u.X, g_pos[i].u.Y, CYCLES, g_pos[i].u.busy_cyc, g_pos[i].u.locked_cyc, \
                     g_pos[i].u.mism); \
            check(g_pos[i].u.mism == 0, "outputs equal to the reference at every clock"); \
            check(g_pos[i].u.busy_cyc > CYCLES / 10, "traffic flowed"); \
            check(g_pos[i].u.locked_cyc > CYCLES / 10, "wormhole locks were exercised");
        `POS_CHECK(0)
        `POS_CHECK(1)
        `POS_CHECK(2)
        `POS_CHECK(3)
        `POS_CHECK(4)
        `POS_CHECK(5)
        if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
        else             $display("%0d OF %0d CHECKS FAILED", errors, checks);
        $finish;
    end
endmodule

// one router position: reference and optimised router side by side
module pair_run #(parameter X = 0, parameter Y = 0, parameter SEED = 1,
                  parameter CYCLES = 1000) (
    input  wire clk,
    output reg  done
);
    reg          rst_n;
    reg  [4:0]   in_valid;
    reg  [169:0] in_flit;
    reg  [4:0]   out_credit;
    wire [4:0]   c_ref, c_dut, v_ref, v_dut;
    wire [169:0] f_ref, f_dut;

    noc_router_ref #(.X(X), .Y(Y)) u_ref (.clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_flit(in_flit), .in_credit(c_ref),
        .out_valid(v_ref), .out_flit(f_ref), .out_credit(out_credit));
    noc_router     #(.X(X), .Y(Y)) u_dut (.clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_flit(in_flit), .in_credit(c_dut),
        .out_valid(v_dut), .out_flit(f_dut), .out_credit(out_credit));

    integer seed, cyc, i, mode, pv, pc, ph_hi, ph_kind, t;
    integer mism, busy_cyc, locked_cyc;
    integer up_cred [0:4];      // mode B: credits the sender of input i holds
    integer dn_used [0:4];      // mode B: flits sitting in the buffer after output i
    reg [31:0] d;
    reg [1:0]  k;

    // compare at every rising edge (outputs are registers)
    always @(posedge clk) begin
        if (rst_n === 1'b1 && {c_ref, v_ref, f_ref} !== {c_dut, v_dut, f_dut}) begin
            mism = mism + 1;
            if (mism <= 3)
                $display("MISMATCH (%0d,%0d) t=%0t: credit %b/%b valid %b/%b\n  ref %h\n  dut %h",
                         X, Y, $time, c_ref, c_dut, v_ref, v_dut, f_ref, f_dut);
        end
    end

    initial begin
        seed = SEED;
        done = 1'b0; rst_n = 1'b0;
        in_valid = 5'd0; in_flit = 170'd0; out_credit = 5'd0;
        mism = 0; busy_cyc = 0; locked_cyc = 0;
        mode = 0; pv = 50; pc = 50; ph_hi = 20; ph_kind = 1;
        for (i = 0; i < 5; i = i + 1) begin up_cred[i] = 4; dn_used[i] = 0; end
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        for (cyc = 0; cyc < CYCLES; cyc = cyc + 1) begin
            if (cyc % 2000 == 0) begin                  // a new phase
                t       = {$random(seed)} % 8;
                mode    = (t < 3) ? 0 : 1;
                pv      = 5 + {$random(seed)} % 96;     // % of clocks with a flit offered
                pc      = 5 + {$random(seed)} % 96;     // % of clocks with a credit back
                ph_hi   = (t == 7) ? 0 : {$random(seed)} % 60;
                ph_kind = {$random(seed)} % 3;
            end
            @(negedge clk);
            // what happened at the last edge (mode B bookkeeping, coverage)
            for (i = 0; i < 5; i = i + 1) begin
                if (c_ref[i]) up_cred[i] = up_cred[i] + 1;
                if (v_ref[i]) dn_used[i] = dn_used[i] + 1;
            end
            if (|v_ref)            busy_cyc   = busy_cyc + 1;
            if (|u_ref.o_locked)   locked_cyc = locked_cyc + 1;
            // new inputs
            for (i = 0; i < 5; i = i + 1) begin
                d = $random(seed);
                t = {$random(seed)} % 100;
                if (ph_kind == 0)      k = d[31:30];                                   // any kind
                else if (ph_kind == 1) k = (t < 40) ? 2'b00 : (t < 70) ? 2'b01 :
                                           (t < 85) ? 2'b10 : 2'b11;                  // long packets
                else                   k = (t < 50) ? 2'b11 : (t < 75) ? 2'b00 : 2'b10; // short ones
                d[22] = ({$random(seed)} % 100) < ph_hi;                              // prio
                in_flit[34*i +: 34] = {k, d};
                if (mode == 0) begin
                    in_valid[i]   = ({$random(seed)} % 100) < pv;
                    out_credit[i] = ({$random(seed)} % 100) < pc;
                end else begin
                    in_valid[i] = (up_cred[i] > 0) && (({$random(seed)} % 100) < pv);
                    if (in_valid[i]) up_cred[i] = up_cred[i] - 1;
                    out_credit[i] = (dn_used[i] > 0) && (({$random(seed)} % 100) < pc);
                    if (out_credit[i]) dn_used[i] = dn_used[i] - 1;
                end
            end
            if (mode == 0)
                for (i = 0; i < 5; i = i + 1) begin up_cred[i] = 4; dn_used[i] = 0; end
            // a reset in the middle of traffic, now and then
            if (({$random(seed)} % 20000) == 0) begin
                rst_n = 1'b0;
                @(negedge clk);
                rst_n = 1'b1;
                for (i = 0; i < 5; i = i + 1) begin up_cred[i] = 4; dn_used[i] = 0; end
            end
        end
        done = 1'b1;
    end
endmodule
