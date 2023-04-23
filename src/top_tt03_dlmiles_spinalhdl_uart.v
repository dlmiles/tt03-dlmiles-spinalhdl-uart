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

    reg foo_reset;
    reg foo_reset$1;
    reg foo_reset$2;
    reg foo_reset$3;
//    task clear;
//     begin
//        foo_reset = 1;
//     end
//    endtask
//
//    initial clear;

    initial begin
        foo_reset$2 = 0;
        foo_reset$3 = 1;
    end

    wire resetCommandStrobe;
    always @ (posedge clk) begin
        foo_reset <= foo_reset$1;
        foo_reset$1 <= foo_reset$2;
        foo_reset$2 <= foo_reset$3;
        foo_reset$3 <= 0;
    end
    assign async_reset = resetCommandStrobe | foo_reset;

    // This exists outside the SpinalHDL project as it has async properties
    // SpinalHDL is based around sync design principles that make up the bulk (99.9%) of digital designs.
    async_reset_ctrl async_reset_ctrl (
        .reset_out  	(sync_reset),
        .clk        	(clk),
        .async_reset_in (async_reset)
`ifdef COCOTB_SIM
`ifndef GL_TEST
        , .sim_reset	(sim_reset)
`endif
`endif
    );

    // This is the SpinalHDL project: uart
    Uart uart (
        .clk			(clk),
        .reset			(sync_reset),
        .io_out8		(out8),
        .io_in7			(in7),
        .io_resetCommandStrobe	(resetCommandStrobe)	// async_reset
    );

endmodule
