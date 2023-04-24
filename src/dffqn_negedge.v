// Timescale change to provide resolution to realistic pulse size
`timescale 1ns/1ps
//
// MetaData-1.0::
// Module-Name: dffqn_negedge
// Module-Language: Verilog-200?
// Output-Count: 2
// Output-1: q Q
// Output-2: qn Q-Inverted
// Input-Count: 2
// input-1: clk Clock (negedge)
// input-2: d data
//
//
//  D Flip-Flop (verilog register simulation)
//
//
//
module dffqn_negedge (
    output reg			q,
    output reg			qn,

    input			clk,
    input			d
);

    always @(negedge clk) begin
        q <= d;
        qn <= ~d;
    end

endmodule
