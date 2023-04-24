
//
//	This will glitch free stretch the 'clock_in'
//
//	sel == 0: clock_in normal (no invertion at clock_out)
//	sel == 1: clock_in is inverted at clock_out
//
//	The sel input can be changed every other clock cycle (or less often).
//
//	The transition is managed glitch free.
//	
//
module clock_stretch_invert_mux (
    output                              clock_out,
    input				clock_in,
    input				sel
);

    reg sel_delay1half;
    always @(negedge clock_in) begin
        sel_delay1half <= sel;
    end

    wire sel_delay1half_inverted;
    assign sel_delay1half_inverted = ~sel_delay1half;

    reg sel_delay2half;
    always @(posedge clock_in) begin
        sel_delay2half <= sel;
    end

    reg sel_delay3half;
    always @(negedge clock_in) begin
        sel_delay3half <= sel_delay2half;
    end    

    wire sel_edge;
    assign sel_edge = sel ^ sel_delay1half;
    wire sel_negedge;
    assign sel_negedge = sel_edge & sel_delay1half;

    // Below is the critical part, that can be made into primitive ?
    // That has good clock characteristics, sharp rise/fall? Consistent tPHL, tPLH ?
    
    // primitive_clock_stretch(
    //   .clock_out(),
    //   .clock_in(),
    //   .sel_negedge(),
    //   .sel_delay1half_inverted(),
    //   .sel_delay3half()
    // )

    wire mux_or[2:0];

    wire clock_in_inverted;
    assign clock_in_inverted = ~clock_in;

    // Normal High state
    assign mux_or[0] = clock_in & sel_delay1half_inverted;

    // dff_negedge_hold_high_state
    // This is only high when we are stretching the high clock_out
    //  state and need to ensure mux_or remains high.
    reg hold_high_state;
    always @(negedge clock_in) begin
        hold_high_state <= sel_negedge;
    end
    assign mux_or[1] = hold_high_state;

    // Inverted High state
    assign mux_or[2] = clock_in_inverted & sel_delay3half;

    sky130_fd_sc_hd__clkbuf_8 myclkbuf8(
        .A(mux_or[0] | mux_or[1] | mux_or[2]),
        .X(clock_out)
    );

    // Glitch free clock mux output final 3-input OR
    //assign clock_out = mux_or[0] | mux_or[1] | mux_or[2];	// need for iverilog
    //assign clock_out = |mux_or[2:0];		// reduction operator
    //assign clock_out = |mux_or;		// reduction operator

endmodule

`ifndef SYNTHESIS
`ifndef GL_TEST
module sky130_fd_sc_hd__clkbuf_8 (
    output X,
    input A
);

    assign X = A;

endmodule
`endif
`endif
