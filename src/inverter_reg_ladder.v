
//
//  I used this to generate an edge after clock startup.
//
//  For this use an ODD number for STAGES, use a constant for 'i'.
//  Do this twice or more to generate a posedge or negedge, etc..
//
//
module inverter_reg_ladder #(
    parameter	STAGES = 	1
) (
    input			clk,
    input			i,	// taps[STAGES]

    output			o,	// taps[0]
    output   	[STAGES-1:0]	taps
);

  reg  [STAGES-1  :0] rtaps;
  wire [STAGES-1+1:0] wtaps;	// +1 for input

  assign wtaps[STAGES] = i;	// input

  genvar stage;

  generate
    for (stage = STAGES - 1; stage >= 0; stage = stage - 1) begin : stinv
      assign wtaps[stage] = ~rtaps[stage];	// inverted
      assign taps[stage]  =  rtaps[stage];
    end
  endgenerate

//  always @ (posedge clk) begin
    //generate
//      for (stage = STAGES - 1; stage > 0; stage = stage - 1) begin : st		// STAGES-1 .. 0 (iterates STAGES times)
//        rtaps[stage] <= wtaps[stage+1];	// ladder
//      end
    //endgenerate
//  end

  generate
    for (stage = STAGES - 1; stage >= 0; stage = stage - 1) begin : st		// STAGES-1 .. 0 (iterates STAGES times)
      always @ (posedge clk) begin
        rtaps[stage] <= wtaps[stage+1];	// ladder
      end
    end
  endgenerate

  assign o = rtaps[0];

endmodule
