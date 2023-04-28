`default_nettype none
`timescale 1ns/1ps

//
//
//
//
module top_tt03_dlmiles_spinalhdl_uart (
    output		[7:0]		io_out,
    input		[7:0]		io_in
`ifdef COCOTB_SIM
`ifndef GL_TEST
,   input				sim_reset
`endif
`endif
);

    wire clk;
    assign clk = io_in[0];
    wire [6:0] in7; // = io_in[7:1];
    assign in7 = io_in[7:1];
    wire [7:0] out8; // = out8;
    assign io_out[7:0] = out8;

    // Active high resets
    wire sync_reset;
    wire async_reset;

    initial begin
    end

    wire resetCommandStrobe;

    wire sim_resetnn;
    wire sim_sett;

    inverter_reg_ladder #(.STAGES(3)) irl_sim_resetn (.clk(clk), .i(1'b1), .o(sim_resetnn), .taps(/*nc*/));
    inverter_reg_ladder #(.STAGES(6)) irl_sim_sett   (.clk(clk), .i(1'b1), .o(sim_sett),    .taps(/*nc*/));

    // This exists outside the SpinalHDL project as it has async properties
    // SpinalHDL is based around sync design principles that make up the bulk (99.9%) of digital designs.
    async_reset_ctrl__dff_async_set async_reset_ctrl (
        .reset_out  	(sync_reset),
        .clk        	(clk),
        .async_reset_in (async_reset)
//`ifdef COCOTB_SIM
//`ifndef GL_TEST
///        , .sim_resetn	(sim_resetn)
///        , .sim_set	(sim_set)
//`endif
//`endif
    );

    wire gatedTxdStopBitSupport;

    // This is the SpinalHDL project: uart
    Uart uart (
        .clk                       (clk),
        .reset                     (sync_reset),
        .io_out8                   (out8),
        .io_in7	                   (in7),
        .io_resetCommandStrobe     (async_reset),
        .io_gatedTxdStopBitSupport (gatedTxdStopBitSupport)
    );

endmodule
