`default_nettype none
`timescale 1ns/1ps

/*
this testbench just instantiates the module and makes some convenient wires
that can be driven / tested by the cocotb test.py
*/

module tb_uart (
    // testbench is controlled by test.py
    input			clk,
    input		[6:0]	in7,
    output		[7:0]	out8
`ifdef COCOTB_SIM
`ifndef GL_TEST
    , input			sim_reset
`endif
`endif
);

    // this part dumps the trace to a vcd file that can be viewed with GTKWave
    initial begin
        $dumpfile ("tb_uart.vcd");
        $dumpvars (0, tb_uart);
        #1;
    end

    // wire up the inputs and outputs
    wire [7:0] inputs = {in7, clk};
    wire [7:0] outputs;
    assign out8 = outputs[7:0];

    // instantiate the DUT
    top_tt03_dlmiles_spinalhdl_uart dut (
`ifdef GL_TEST
        .vccd1    ( 1'b1),
        .vssd1    ( 1'b0),
`endif
        .io_in    (inputs),
        .io_out   (outputs)
`ifdef COCOTB_SIM
`ifndef GL_TEST
     , .sim_reset (sim_reset)
`endif
`endif
    );

endmodule
