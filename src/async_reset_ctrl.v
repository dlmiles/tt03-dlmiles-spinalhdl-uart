`default_nettype none
`timescale 1ns/1ps

//
//  This circuit converts an async level or glitch into a sync reset high signal
//
//
module async_reset_ctrl (
    output			reset_out,
    input			clk,
    input			async_reset_in
//`ifdef COCOTB_SIM
//`ifndef GL_TEST
    // ifdef SIMULATOR
    // This input is provided to allow the simulator to put the state X into a
    //  none X state at startup.  Ideally we'd like this to be 0,1,random.
,   input			sim_resetn
,   input			sim_set
//`endif
//`endif
);

    // The instance naming suffix 1,2,3 relates to the sequence the signal passed to get towards 'reset_out'

    wire rs1q;
    wire dff2q;

    dff dff2 (
      .clk (clk),
      .d   (rs1q),
      .q   (dff2q)
    );

    wire dff3q;		// aka reset_out

    dff dff3 (
      .clk (clk),
      .d   (dff2q),
      .q   (dff3q)
    );

    wire posedge_det_glitcher;
    wire async_reset_in_inverted;
    // How do you simulate a glitch, well you can instruct the simulator the inverter step has a delay
    assign #2 async_reset_in_inverted = !async_reset_in;
    assign #1 posedge_det_glitcher = async_reset_in & async_reset_in_inverted;
    // assign posedge_det_glitcher  = async_reset_in & !async_reset_in;

    wire rs_reset_priority;
    wire dff3q_inverted;
    assign #2 dff3q_inverted = !dff3q;
    assign #1 rs_reset_priority = posedge_det_glitcher & dff3q_inverted;
    //assign rs_reset_priority = posedge_det_glitcher & !dff3q;

    wire rs_reset_priority_inverted;
    assign #1 rs_reset_priority_inverted = !rs_reset_priority;	// effective NAND due to active LOW sr_latch_nand

    wire reset;		// net exists for sim_reset connection
//`ifdef COCOTB_SIM
//`ifndef GL_TEST
    assign reset = sim_resetn & dff3q_inverted;	// COCOTB_SIM && !GL_TEST
    wire set;
    assign set = sim_set | rs_reset_priority_inverted;
//`else
//    assign reset = dff3q_inverted;		// COCOTB_SIM && GL_TEST
//`endif
//`else
//    assign reset = dff3q_inverted;		// !COCOTB_SIM
//`endif

    // using dff3q_inverted instead of dff3q as it is sr_latch_nand so active low

    sr_latch_nand sr_latch1 (
      .s   (set),		// rs_reset_priority_inverted
      .r   (reset),		// dff3q_inverted
      .clr (sim_resetn),
      .q   (rs1q)
    );

    assign reset_out = dff3q;

endmodule
