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

    reg sim_resetn;
    reg sim_resetn$1;
    reg sim_resetn$2;
    reg sim_resetn$3;
//    reg sim_resetn$4;

    reg sim_set;
    reg sim_set$1;
    reg sim_set$2;
    reg sim_set$3;
    reg sim_set$4;
    reg sim_set$5;
//    task clear;
//     begin
//        sim_resetn = 1;
//     end
//    endtask
//
//    initial clear;

    initial begin
        sim_resetn$1 = 1;
        sim_resetn$2 = 0;
        sim_set$3 = 1;
        sim_set$4 = 0;
    end

    wire resetCommandStrobe;
    always @ (posedge clk) begin
        sim_resetn   <= ~sim_resetn$1;		// 1
        sim_resetn$1 <= ~sim_resetn$2;		// 0
        sim_resetn$2 <= 1; //~sim_resetn$3;	// 1
        //sim_resetn$3 <= ~sim_resetn$4;	// 0
        //sim_resetn$4 <= 1;			// 1

        sim_set   <= ~sim_set$1;	// 0
        sim_set$1 <= ~sim_set$2;	// 1
        sim_set$2 <= ~sim_set$3;	// 0
        sim_set$3 <= ~sim_set$4;	// 1
        sim_set$4 <= ~sim_set$5;	// 0
        sim_set$5 <= 1;
    end

    // This exists outside the SpinalHDL project as it has async properties
    // SpinalHDL is based around sync design principles that make up the bulk (99.9%) of digital designs.
    async_reset_ctrl async_reset_ctrl (
        .reset_out  	(sync_reset),
        .clk        	(clk),
        .async_reset_in (async_reset)
//`ifdef COCOTB_SIM
//`ifndef GL_TEST
        , .sim_resetn	(sim_resetn)
        , .sim_set	(sim_set)
//`endif
//`endif
    );

    // This is the SpinalHDL project: uart
    Uart uart (
        .clk			(clk),
        .reset			(sync_reset),
        .io_out8		(out8),
        .io_in7			(in7),
        .io_resetCommandStrobe	(async_reset)
    );

endmodule
