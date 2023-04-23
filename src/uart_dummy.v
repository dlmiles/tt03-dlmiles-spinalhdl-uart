`default_nettype none
`timescale 1ns/1ps

//
//
//
//
module uart_dummy (
    input				clk,
    input				reset,
    output reg		[7:0]		io_out8,
    input		[6:0]		io_in7,
    output reg				io_resetCommandStrobe	// can be a wire ?
);

    localparam CMD_DATA		= 2'd0;
    localparam CMD_CONFIG	= 2'd1;
    localparam CMD_PREDIV	= 2'd2;
    localparam CMD_SPARE	= 2'd3;

    localparam CMD_CONFIG_RESET	= 5'b11000;	// c2

    wire [1:0] cmd;
    assign cmd = io_in7[1:0];	// io_in[2:1]
    wire [4:0] in7_3;
    assign in7_3 = io_in7[6:2];	// io_in[7:3]

    wire has_cmd;
    assign has_cmd = cmd == CMD_CONFIG;
    wire has_in7_3;
    assign has_in7_3 = in7_3 == CMD_CONFIG_RESET;

    always @ (posedge clk) begin
        io_resetCommandStrobe <= 0;
        if (cmd == CMD_CONFIG && in7_3 == CMD_CONFIG_RESET) begin
            io_resetCommandStrobe <= 1;
        end
    end    

    reg [7:0] count;
    reg run;

    always @ (posedge clk) begin
        if (reset) begin
            io_out8 <= 8'h0;
            run <= 1;
            count <= 8'h0;
        end else if (io_in7[6] && io_in7[5] && cmd == CMD_CONFIG) begin
            io_out8 <= 8'b10101100;
        end else if (count == 8'h0) begin
            io_out8[6:2] <= io_out8[6:2] + 1;
        end else begin
            count <= count - 1;
        end
    end

endmodule
