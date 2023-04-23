`default_nettype none
`timescale 1ns/1ps

//
//
//
//
module uart (
    input				clk,
    input				reset,
    output reg		[7:0]		out8,
    input		[6:0]		in7,
    output reg				resetCommandStrobe	// can be a wire ?
);

    localparam CMD_DATA		= 2'd0;
    localparam CMD_CONFIG	= 2'd1;
    localparam CMD_PREDIV	= 2'd2;
    localparam CMD_SPARE	= 2'd3;

    localparam CMD_CONFIG_RESET	= 5'b11000;	// c2

    wire [1:0] cmd;
    assign cmd = in7[1:0];	// io_in[2:1]
    wire [4:0] in7_3;
    assign in7_3 = in7[6:2];	// io_in[7:3]

    wire has_cmd;
    assign has_cmd = cmd == CMD_CONFIG;
    wire has_in7_3;
    assign has_in7_3 = in7_3 == CMD_CONFIG_RESET;

    always @ (posedge clk) begin
        resetCommandStrobe <= 0;
        if (cmd == CMD_CONFIG && in7_3 == CMD_CONFIG_RESET) begin
            resetCommandStrobe <= 1;
        end
    end    

    reg [7:0] count;
    reg run;

    always @ (posedge clk) begin
        if (reset) begin
            out8 <= 0;
            run <= 1;
            count <= 0;
//        end else if(in7[6]) begin
//            out8 <= 8'b10101100;
        end
    end
    
    always @ (posedge clk) begin
        count <= count - 1;
        if (count == 0) begin
            out8[6:2] <= out8[6:2] + 1;
        end
    end

endmodule
