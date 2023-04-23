`default_nettype none
// Timescale change to provide resolution to realistic pulse size
`timescale 100ps/1ps
//
// MetaData-1.0::
// Module-Name: sr_latch_nand
// Module-Language: Verilog-200?
// Output-Count: 2
// Output-1: q Q
// Output-2: qn Q-inverted
// Input-Count: 2
// input-1: r Reset
// input-2: s Set
//
//
//  SR Latch (using NAND, active LOW)
//
//  Level Sensitive
//
//  S  R    Q  Qn
//  0  0    X   X  (ILLEGAL state)
//          1   1  (likely output seen from illegal state)
//  0  1    1   0  (set active)
//  1  0    0   1  (reset active)
//  1  1  n/c n/c  (no change in state)
//          0   1  (possible no change state)
//          1   0  (possible no change state)
//
//
module sr_latch_nand (
    output		q,
    output		qn,
    input		r,
    input		s
);

    // SR-latch / sr_latch_nand
    //
    // Q     = NAND(R, Qn)
    // Qn    = NAND(S, Q)
    //

    //
    //   +---------+
    //   | S     Q |
    //   |         |
    //   |         |
    //   |         |
    //   | R     Qn|
    //   +---------+
    //
    
    initial begin
        // FIXME we want to allow startup policy to be X, 0, 1, random here.
`ifndef __ICARUS__
        q = 0;
        qn = 1;
`endif
    end

    // SR latch NAND
    wire q_int, qn_int;
    assign #1 q_int = ~(s & qn_int);		// NAND
    assign #1 qn_int = ~(r & q_int);		// NAND
    assign q = q_int;
    assign qn = qn_int;

`ifndef __ICARUS__
    asrt_setrst : assert(!r && !s)
     else $error("reset and set can not be high at the same time.");
`endif

endmodule
