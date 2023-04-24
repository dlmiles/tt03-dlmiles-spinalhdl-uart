// Generator : SpinalHDL dev    git head : ab2403fd43aa92126d173aaad3edce3eac04ef89
// Component : Uart
// Git hash  : b79a254f8ef9eb05864c432f913087cb0687d2e3

`timescale 1ns/1ps

module Uart (
  input      [6:0]    io_in7,
  output reg [7:0]    io_out8,
  output reg          io_resetCommandStrobe,
  output              io_gatedTxdStopBitSupport,
  input               clk,
  input               reset
);
  localparam UartCtrlRxState_WAITMARK = 3'd0;
  localparam UartCtrlRxState_IDLE = 3'd1;
  localparam UartCtrlRxState_START = 3'd2;
  localparam UartCtrlRxState_DATA = 3'd3;
  localparam UartCtrlRxState_PARITY = 3'd4;
  localparam UartCtrlRxState_STOP = 3'd5;
  localparam UartParityType_NONE = 2'd0;
  localparam UartParityType_ODD = 2'd1;
  localparam UartParityType_EVEN = 2'd2;

  wire                clockPrescalerRipplerMux_bit0_0_1_sel;
  wire                clockPrescalerRipplerMux_bit0_2_3_sel;
  wire                clockPrescalerRipplerMux_bit1_01_23_sel;
  wire                stretcherClockIn_sel;
  wire                area_samplingTicker_samplingTickerInputTick_sel;
  wire                area_rxStateMachine_shifter_io_en;
  wire                clockPrescalerRipplerMux_bit0_0_1_clk_out;
  wire                clockPrescalerRipplerMux_bit0_2_3_clk_out;
  wire                clockPrescalerRipplerMux_bit1_01_23_clk_out;
  wire                stretcherClockIn_clk_out;
  wire                clockPrescalerRetarder_stretcher_clock_out;
  wire                flowCCByToggle_1_io_output_valid;
  wire       [7:0]    flowCCByToggle_1_io_output_payload;
  wire                area_samplingTicker_samplingTickerInputTick_clk_out;
  wire                rxd_buffercc_io_dataOut;
  wire       [6:0]    area_rxStateMachine_shifter_io_output7;
  wire       [7:0]    area_rxStateMachine_shifter_io_output8;
  wire       [3:0]    _zz_when_Uart_l512;
  wire                _zz_area_sampler_value;
  wire                _zz_area_sampler_value_1;
  wire                _zz_area_sampler_value_2;
  wire                _zz_area_sampler_value_3;
  wire                _zz_area_sampler_value_4;
  wire                _zz_area_sampler_value_5;
  wire                _zz_area_sampler_value_6;
  wire       [7:0]    _zz_area_rxStateMachine_shifterOutputSized;
  wire       [3:0]    _zz_when_Uart_l844;
  wire       [3:0]    _zz_when_Uart_l868;
  wire       [1:0]    cmd;
  reg                 rxdLast;
  reg                 rxd;
  wire                when_Uart_l360;
  reg                 modeData7;
  reg        [1:0]    modeParity;
  reg        [1:0]    modeStop;
  reg        [2:0]    dataLengthZeroBased;
  reg        [0:0]    stopBitsZeroBased;
  wire                stopBitsIsHalf;
  wire                when_Uart_l384;
  reg        [1:0]    regPredivRpl;
  reg        [2:0]    regPredivCtr;
  reg                 regPreretEna;
  reg        [3:0]    regPreretCtr;
  reg        [7:0]    bufVec_0;
  reg                 bufPresent_0;
  reg                 rxBreakLatch;
  wire                rxErrorParity;
  wire                rxErrorStop;
  reg                 rxMasterClock;
  wire                when_Uart_l420;
  wire                when_Uart_l428;
  wire                when_Uart_l434;
  wire                when_Uart_l437;
  wire                clockPrescalerRippler_taps_0;
  wire                clockPrescalerRippler_taps_1;
  wire                clockPrescalerRippler_taps_2;
  wire                clockPrescalerRippler_taps_3;
  reg        [0:0]    _zz_clockPrescalerRippler_taps_1;
  reg        [0:0]    _zz_clockPrescalerRippler_taps_2;
  reg        [0:0]    _zz_clockPrescalerRippler_taps_3;
  reg        [3:0]    clockPrescalerCounter_counter;
  wire                clockPrescalerCounter_initialState;
  reg                 clockPrescalerCounter_tick;
  wire                when_Uart_l511;
  wire                when_Uart_l512;
  reg        [3:0]    clockPrescalerRetarder_counter;
  reg                 clockPrescalerRetarder_sel;
  wire                when_Uart_l545;
  wire                when_Uart_l547;
  reg                 flowPush_valid;
  reg        [7:0]    flowPush_payload;
  reg        [0:0]    area_samplingTicker_counter;
  reg                 area_samplingTicker_tick;
  wire                when_Uart_l599;
  wire                when_Uart_l601;
  wire                area_samplingTick;
  wire                area_samplingTickCmdData;
  wire                area_sampler_synchroniser;
  wire                area_sampler_samples0_0;
  reg                 area_sampler_samples0_1;
  reg                 area_sampler_samples0_2;
  reg                 area_sampler_samples0_3;
  reg                 area_sampler_samples0_4;
  wire                area_sampler_inputBuffer;
  reg                 area_sampler_regSamples_0;
  reg                 area_sampler_regSamples_1;
  reg                 area_sampler_regSamples_2;
  reg                 area_sampler_regSamples_3;
  reg                 area_sampler_regSamples_4;
  reg                 area_sampler_value;
  reg                 area_sampler_tick;
  reg        [2:0]    area_rxBitTimer_counter;
  reg                 area_rxBitTimer_tick;
  wire                when_Uart_l703;
  reg        [3:0]    area_rxBitCounter_value;
  reg        [6:0]    area_rxBreak_counter;
  wire                area_rxBreak_valid;
  wire                when_Uart_l753;
  reg                 area_rxBreak_valid_regNext;
  wire                when_Uart_l757;
  reg        [2:0]    area_rxStateMachine_state;
  reg                 area_rxStateMachine_parity;
  wire       [7:0]    area_rxStateMachine_shifterOutputSized;
  wire                when_Uart_l798;
  wire                when_Uart_l811;
  wire                when_Uart_l834;
  wire                when_Uart_l844;
  wire                when_Uart_l845;
  wire                when_Uart_l859;
  wire                when_Uart_l866;
  wire                when_Uart_l868;
  wire                when_Uart_l893;
  `ifndef SYNTHESIS
  reg [63:0] area_rxStateMachine_state_string;
  `endif


  assign _zz_when_Uart_l512 = {1'd0, regPredivCtr};
  assign _zz_area_rxStateMachine_shifterOutputSized = {1'd0, area_rxStateMachine_shifter_io_output7};
  assign _zz_when_Uart_l844 = {1'd0, dataLengthZeroBased};
  assign _zz_when_Uart_l868 = {3'd0, stopBitsZeroBased};
  assign _zz_area_sampler_value = ((((1'b0 || ((_zz_area_sampler_value_1 && area_sampler_regSamples_1) && area_sampler_regSamples_2)) || (((_zz_area_sampler_value_2 && area_sampler_regSamples_0) && area_sampler_regSamples_1) && area_sampler_regSamples_3)) || (((1'b1 && area_sampler_regSamples_0) && area_sampler_regSamples_2) && area_sampler_regSamples_3)) || (((1'b1 && area_sampler_regSamples_1) && area_sampler_regSamples_2) && area_sampler_regSamples_3));
  assign _zz_area_sampler_value_3 = (((1'b1 && area_sampler_regSamples_0) && area_sampler_regSamples_1) && area_sampler_regSamples_4);
  assign _zz_area_sampler_value_4 = ((1'b1 && area_sampler_regSamples_0) && area_sampler_regSamples_2);
  assign _zz_area_sampler_value_5 = (1'b1 && area_sampler_regSamples_1);
  assign _zz_area_sampler_value_6 = 1'b1;
  assign _zz_area_sampler_value_1 = (1'b1 && area_sampler_regSamples_0);
  assign _zz_area_sampler_value_2 = 1'b1;
  glitch_free_clock_mux clockPrescalerRipplerMux_bit0_0_1 (
    .clk_out (clockPrescalerRipplerMux_bit0_0_1_clk_out), //o
    .sel     (clockPrescalerRipplerMux_bit0_0_1_sel    ), //i
    .clk_0   (clockPrescalerRippler_taps_0             ), //i
    .clk_1   (clockPrescalerRippler_taps_1             )  //i
  );
  glitch_free_clock_mux clockPrescalerRipplerMux_bit0_2_3 (
    .clk_out (clockPrescalerRipplerMux_bit0_2_3_clk_out), //o
    .sel     (clockPrescalerRipplerMux_bit0_2_3_sel    ), //i
    .clk_0   (clockPrescalerRippler_taps_2             ), //i
    .clk_1   (clockPrescalerRippler_taps_3             )  //i
  );
  glitch_free_clock_mux clockPrescalerRipplerMux_bit1_01_23 (
    .clk_out (clockPrescalerRipplerMux_bit1_01_23_clk_out), //o
    .sel     (clockPrescalerRipplerMux_bit1_01_23_sel    ), //i
    .clk_0   (clockPrescalerRipplerMux_bit0_0_1_clk_out  ), //i
    .clk_1   (clockPrescalerRipplerMux_bit0_2_3_clk_out  )  //i
  );
  glitch_free_clock_mux stretcherClockIn (
    .clk_out (stretcherClockIn_clk_out                   ), //o
    .sel     (stretcherClockIn_sel                       ), //i
    .clk_0   (clockPrescalerRipplerMux_bit1_01_23_clk_out), //i
    .clk_1   (clockPrescalerCounter_tick                 )  //i
  );
  clock_stretch_invert_mux clockPrescalerRetarder_stretcher (
    .clock_out (clockPrescalerRetarder_stretcher_clock_out), //o
    .clock_in  (stretcherClockIn_clk_out                  ), //i
    .sel       (clockPrescalerRetarder_sel                )  //i
  );
  FlowCCByToggle flowCCByToggle_1 (
    .io_input_valid    (flowPush_valid                            ), //i
    .io_input_payload  (flowPush_payload[7:0]                     ), //i
    .io_output_valid   (flowCCByToggle_1_io_output_valid          ), //o
    .io_output_payload (flowCCByToggle_1_io_output_payload[7:0]   ), //o
    .clock_out         (clockPrescalerRetarder_stretcher_clock_out), //i
    .reset             (reset                                     ), //i
    .clk               (clk                                       )  //i
  );
  glitch_free_clock_mux area_samplingTicker_samplingTickerInputTick (
    .clk_out (area_samplingTicker_samplingTickerInputTick_clk_out), //o
    .sel     (area_samplingTicker_samplingTickerInputTick_sel    ), //i
    .clk_0   (clockPrescalerRetarder_stretcher_clock_out         ), //i
    .clk_1   (clockPrescalerCounter_tick                         )  //i
  );
  BufferCC rxd_buffercc (
    .io_dataIn  (rxd                                       ), //i
    .io_dataOut (rxd_buffercc_io_dataOut                   ), //o
    .clock_out  (clockPrescalerRetarder_stretcher_clock_out), //i
    .reset      (reset                                     )  //i
  );
  ShiftRegisterSerialInParallelOut area_rxStateMachine_shifter (
    .io_clk     (area_rxBitTimer_tick                       ), //i
    .io_input   (area_sampler_value                         ), //i
    .io_en      (area_rxStateMachine_shifter_io_en          ), //i
    .io_output7 (area_rxStateMachine_shifter_io_output7[6:0]), //o
    .io_output8 (area_rxStateMachine_shifter_io_output8[7:0]), //o
    .reset      (reset                                      )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(area_rxStateMachine_state)
      UartCtrlRxState_WAITMARK : area_rxStateMachine_state_string = "WAITMARK";
      UartCtrlRxState_IDLE : area_rxStateMachine_state_string = "IDLE    ";
      UartCtrlRxState_START : area_rxStateMachine_state_string = "START   ";
      UartCtrlRxState_DATA : area_rxStateMachine_state_string = "DATA    ";
      UartCtrlRxState_PARITY : area_rxStateMachine_state_string = "PARITY  ";
      UartCtrlRxState_STOP : area_rxStateMachine_state_string = "STOP    ";
      default : area_rxStateMachine_state_string = "????????";
    endcase
  end
  `endif

  assign io_gatedTxdStopBitSupport = 1'b0;
  assign cmd = io_in7[1 : 0];
  assign when_Uart_l360 = (cmd == 2'b00);
  always @(*) begin
    if(when_Uart_l360) begin
      rxd = io_in7[6];
    end else begin
      rxd = rxdLast;
    end
  end

  always @(*) begin
    if(modeData7) begin
      dataLengthZeroBased = 3'b110;
    end else begin
      dataLengthZeroBased = 3'b111;
    end
  end

  assign stopBitsIsHalf = modeStop[1];
  assign when_Uart_l384 = (modeStop == 2'b00);
  always @(*) begin
    if(when_Uart_l384) begin
      stopBitsZeroBased = 1'b0;
    end else begin
      stopBitsZeroBased = 1'b1;
    end
  end

  always @(*) begin
    io_out8 = 8'h00;
    if(when_Uart_l420) begin
      if(when_Uart_l428) begin
        io_out8[4] = 1'b1;
      end
      io_out8[5] = 1'b1;
    end
    io_out8[1] = clockPrescalerRetarder_stretcher_clock_out;
    io_out8[2] = area_sampler_tick;
    io_out8[3] = area_rxBitTimer_tick;
    io_out8[7 : 4] = area_rxBitCounter_value;
    io_out8[0] = area_rxBreak_valid;
    if(when_Uart_l893) begin
      if(bufPresent_0) begin
        io_out8 = bufVec_0;
      end
    end
  end

  always @(*) begin
    io_resetCommandStrobe = 1'b0;
    if(when_Uart_l420) begin
      if(when_Uart_l428) begin
        io_resetCommandStrobe = 1'b1;
      end
    end
  end

  always @(*) begin
    rxMasterClock = 1'b0;
    if(!when_Uart_l420) begin
      if(!when_Uart_l434) begin
        if(!when_Uart_l437) begin
          rxMasterClock = clk;
        end
      end
    end
  end

  assign when_Uart_l420 = (cmd == 2'b01);
  assign when_Uart_l428 = (io_in7[6 : 5] == 2'b11);
  assign when_Uart_l434 = (cmd == 2'b10);
  assign when_Uart_l437 = (cmd == 2'b11);
  assign clockPrescalerRippler_taps_0 = rxMasterClock;
  assign clockPrescalerRippler_taps_1 = _zz_clockPrescalerRippler_taps_1[0];
  assign clockPrescalerRippler_taps_2 = _zz_clockPrescalerRippler_taps_2[0];
  assign clockPrescalerRippler_taps_3 = _zz_clockPrescalerRippler_taps_3[0];
  assign clockPrescalerRipplerMux_bit0_0_1_sel = (regPredivRpl[0] ^ reset);
  assign clockPrescalerRipplerMux_bit0_2_3_sel = (regPredivRpl[0] ^ reset);
  assign clockPrescalerRipplerMux_bit1_01_23_sel = (regPredivRpl[1] ^ reset);
  assign clockPrescalerCounter_initialState = 1'b1;
  assign when_Uart_l511 = (regPredivCtr != 3'b000);
  assign when_Uart_l512 = (clockPrescalerCounter_counter == _zz_when_Uart_l512);
  assign stretcherClockIn_sel = ((regPredivCtr != 3'b000) ^ reset);
  assign when_Uart_l545 = (regPreretCtr != 4'b0000);
  assign when_Uart_l547 = (clockPrescalerRetarder_counter == 4'b0000);
  always @(*) begin
    flowPush_valid = 1'b0;
    case(area_rxStateMachine_state)
      UartCtrlRxState_WAITMARK : begin
      end
      UartCtrlRxState_IDLE : begin
      end
      UartCtrlRxState_START : begin
      end
      UartCtrlRxState_DATA : begin
      end
      UartCtrlRxState_PARITY : begin
      end
      default : begin
        if(area_rxBitTimer_tick) begin
          if(!when_Uart_l866) begin
            if(when_Uart_l868) begin
              flowPush_valid = 1'b1;
            end
          end
        end
      end
    endcase
  end

  always @(*) begin
    flowPush_payload = 8'bxxxxxxxx;
    case(area_rxStateMachine_state)
      UartCtrlRxState_WAITMARK : begin
      end
      UartCtrlRxState_IDLE : begin
      end
      UartCtrlRxState_START : begin
      end
      UartCtrlRxState_DATA : begin
      end
      UartCtrlRxState_PARITY : begin
      end
      default : begin
        if(area_rxBitTimer_tick) begin
          if(!when_Uart_l866) begin
            if(when_Uart_l868) begin
              flowPush_payload = area_rxStateMachine_shifterOutputSized;
            end
          end
        end
      end
    endcase
  end

  assign area_samplingTicker_samplingTickerInputTick_sel = ((regPredivCtr != 3'b000) ^ reset);
  assign when_Uart_l599 = 1'b1;
  assign when_Uart_l601 = (area_samplingTicker_counter == 1'b0);
  assign area_samplingTick = clockPrescalerRetarder_stretcher_clock_out;
  assign area_samplingTickCmdData = (area_samplingTick && (cmd == 2'b00));
  assign area_sampler_synchroniser = rxd_buffercc_io_dataOut;
  assign area_sampler_samples0_0 = area_sampler_synchroniser;
  assign area_sampler_inputBuffer = area_sampler_synchroniser;
  always @(*) begin
    area_rxBitTimer_tick = 1'b0;
    if(area_sampler_tick) begin
      if(when_Uart_l703) begin
        area_rxBitTimer_tick = 1'b1;
      end
    end
  end

  assign when_Uart_l703 = (area_rxBitTimer_counter == 3'b000);
  assign area_rxBreak_valid = (area_rxBreak_counter == 7'h68);
  assign when_Uart_l753 = (area_samplingTickCmdData && (! area_rxBreak_valid));
  assign when_Uart_l757 = (area_rxBreak_valid && (! area_rxBreak_valid_regNext));
  assign area_rxStateMachine_shifter_io_en = (area_rxStateMachine_state == UartCtrlRxState_DATA);
  assign area_rxStateMachine_shifterOutputSized = (modeData7 ? _zz_area_rxStateMachine_shifterOutputSized : area_rxStateMachine_shifter_io_output8);
  assign when_Uart_l798 = (area_sampler_value == 1'b1);
  assign when_Uart_l811 = (area_sampler_value == 1'b0);
  assign when_Uart_l834 = (area_sampler_value == 1'b1);
  assign when_Uart_l844 = (area_rxBitCounter_value == _zz_when_Uart_l844);
  assign when_Uart_l845 = (modeParity == UartParityType_NONE);
  assign when_Uart_l859 = (area_rxStateMachine_parity != area_sampler_value);
  assign when_Uart_l866 = (! area_sampler_value);
  assign when_Uart_l868 = (area_rxBitCounter_value == _zz_when_Uart_l868);
  assign when_Uart_l893 = (cmd == 2'b00);
  always @(posedge clk or posedge reset) begin
    if(reset) begin
      rxdLast <= 1'b1;
      modeData7 <= 1'b0;
      modeParity <= 2'b00;
      modeStop <= 2'b00;
      regPredivRpl <= 2'b00;
      regPredivCtr <= 3'b000;
      regPreretEna <= 1'b0;
      regPreretCtr <= 4'b0000;
      bufPresent_0 <= 1'b0;
      rxBreakLatch <= 1'b0;
    end else begin
      if(when_Uart_l360) begin
        rxdLast <= io_in7[6];
      end
      if(when_Uart_l420) begin
        modeData7 <= io_in7[2];
        modeParity <= io_in7[4 : 3];
        modeStop <= io_in7[6 : 5];
      end else begin
        if(when_Uart_l434) begin
          regPredivRpl <= io_in7[3 : 2];
          regPredivCtr <= io_in7[6 : 4];
        end else begin
          if(when_Uart_l437) begin
            regPreretEna <= io_in7[6];
            regPreretCtr <= io_in7[5 : 2];
          end
        end
      end
      if(!area_sampler_value) begin
        if(when_Uart_l757) begin
          rxBreakLatch <= 1'b1;
        end
      end
      case(area_rxStateMachine_state)
        UartCtrlRxState_WAITMARK : begin
        end
        UartCtrlRxState_IDLE : begin
        end
        UartCtrlRxState_START : begin
        end
        UartCtrlRxState_DATA : begin
          bufPresent_0 <= 1'b0;
        end
        UartCtrlRxState_PARITY : begin
        end
        default : begin
        end
      endcase
      if(flowCCByToggle_1_io_output_valid) begin
        bufPresent_0 <= 1'b1;
      end
    end
  end

  always @(posedge clockPrescalerRippler_taps_0 or posedge reset) begin
    if(reset) begin
      _zz_clockPrescalerRippler_taps_1 <= 1'b0;
    end else begin
      _zz_clockPrescalerRippler_taps_1 <= (~ _zz_clockPrescalerRippler_taps_1);
    end
  end

  always @(posedge clockPrescalerRippler_taps_1 or posedge reset) begin
    if(reset) begin
      _zz_clockPrescalerRippler_taps_2 <= 1'b0;
    end else begin
      _zz_clockPrescalerRippler_taps_2 <= (~ _zz_clockPrescalerRippler_taps_2);
    end
  end

  always @(posedge clockPrescalerRippler_taps_2 or posedge reset) begin
    if(reset) begin
      _zz_clockPrescalerRippler_taps_3 <= 1'b0;
    end else begin
      _zz_clockPrescalerRippler_taps_3 <= (~ _zz_clockPrescalerRippler_taps_3);
    end
  end

  always @(posedge clockPrescalerRipplerMux_bit1_01_23_clk_out or posedge reset) begin
    if(reset) begin
      clockPrescalerCounter_counter <= 4'b0000;
      clockPrescalerCounter_tick <= clockPrescalerCounter_initialState;
    end else begin
      if(when_Uart_l511) begin
        if(when_Uart_l512) begin
          clockPrescalerCounter_counter <= 4'b0001;
          clockPrescalerCounter_tick <= (! clockPrescalerCounter_tick);
        end else begin
          clockPrescalerCounter_counter <= (clockPrescalerCounter_counter + 4'b0001);
        end
      end else begin
        clockPrescalerCounter_counter <= 4'b0001;
        clockPrescalerCounter_tick <= clockPrescalerCounter_initialState;
      end
    end
  end

  always @(posedge stretcherClockIn_clk_out or posedge reset) begin
    if(reset) begin
      clockPrescalerRetarder_counter <= 4'b0000;
      clockPrescalerRetarder_sel <= 1'b0;
    end else begin
      if(when_Uart_l545) begin
        clockPrescalerRetarder_counter <= (clockPrescalerRetarder_counter - 4'b0001);
        if(when_Uart_l547) begin
          clockPrescalerRetarder_counter <= regPreretCtr;
          clockPrescalerRetarder_sel <= (! clockPrescalerRetarder_sel);
        end
      end
    end
  end

  always @(posedge clockPrescalerRetarder_stretcher_clock_out or posedge reset) begin
    if(reset) begin
      area_samplingTicker_counter <= 1'b0;
      area_samplingTicker_tick <= 1'b0;
      area_sampler_regSamples_0 <= 1'b0;
      area_sampler_regSamples_1 <= 1'b1;
      area_sampler_regSamples_2 <= 1'b0;
      area_sampler_regSamples_3 <= 1'b1;
      area_sampler_regSamples_4 <= 1'b0;
      area_sampler_value <= 1'b0;
      area_sampler_tick <= 1'b0;
      area_rxBitTimer_counter <= 3'b000;
      area_rxBitCounter_value <= 4'b0000;
      area_rxBreak_counter <= 7'h00;
      area_rxStateMachine_state <= UartCtrlRxState_WAITMARK;
    end else begin
      area_samplingTicker_tick <= 1'b0;
      if(when_Uart_l599) begin
        area_samplingTicker_counter <= (area_samplingTicker_counter - 1'b1);
        if(when_Uart_l601) begin
          area_samplingTicker_tick <= 1'b1;
        end
      end
      if(area_samplingTickCmdData) begin
        area_sampler_regSamples_0 <= area_sampler_inputBuffer;
      end
      if(area_samplingTickCmdData) begin
        area_sampler_regSamples_1 <= area_sampler_regSamples_0;
      end
      if(area_samplingTickCmdData) begin
        area_sampler_regSamples_2 <= area_sampler_regSamples_1;
      end
      if(area_samplingTickCmdData) begin
        area_sampler_regSamples_3 <= area_sampler_regSamples_2;
      end
      if(area_samplingTickCmdData) begin
        area_sampler_regSamples_4 <= area_sampler_regSamples_3;
      end
      area_sampler_value <= ((((((_zz_area_sampler_value || _zz_area_sampler_value_3) || (_zz_area_sampler_value_4 && area_sampler_regSamples_4)) || ((_zz_area_sampler_value_5 && area_sampler_regSamples_2) && area_sampler_regSamples_4)) || (((_zz_area_sampler_value_6 && area_sampler_regSamples_0) && area_sampler_regSamples_3) && area_sampler_regSamples_4)) || (((1'b1 && area_sampler_regSamples_1) && area_sampler_regSamples_3) && area_sampler_regSamples_4)) || (((1'b1 && area_sampler_regSamples_2) && area_sampler_regSamples_3) && area_sampler_regSamples_4));
      area_sampler_tick <= area_samplingTickCmdData;
      if(area_sampler_tick) begin
        area_rxBitTimer_counter <= (area_rxBitTimer_counter - 3'b001);
      end
      if(area_rxBitTimer_tick) begin
        area_rxBitCounter_value <= (area_rxBitCounter_value + 4'b0001);
      end
      if(area_sampler_value) begin
        area_rxBreak_counter <= 7'h00;
      end else begin
        if(when_Uart_l753) begin
          area_rxBreak_counter <= (area_rxBreak_counter + 7'h01);
        end
      end
      case(area_rxStateMachine_state)
        UartCtrlRxState_WAITMARK : begin
          if(area_sampler_tick) begin
            if(when_Uart_l798) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        UartCtrlRxState_IDLE : begin
          if(area_sampler_tick) begin
            if(area_rxBreak_valid) begin
              area_rxStateMachine_state <= UartCtrlRxState_WAITMARK;
            end else begin
              if(when_Uart_l811) begin
                area_rxStateMachine_state <= UartCtrlRxState_START;
                area_rxBitTimer_counter <= 3'b101;
              end
            end
          end
        end
        UartCtrlRxState_START : begin
          if(area_rxBitTimer_tick) begin
            area_rxStateMachine_state <= UartCtrlRxState_DATA;
            area_rxBitCounter_value <= 4'b0000;
            if(when_Uart_l834) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        UartCtrlRxState_DATA : begin
          if(area_rxBitTimer_tick) begin
            if(when_Uart_l844) begin
              if(when_Uart_l845) begin
                area_rxStateMachine_state <= UartCtrlRxState_STOP;
                area_rxBitCounter_value <= 4'b0000;
              end else begin
                area_rxStateMachine_state <= UartCtrlRxState_PARITY;
              end
            end
          end
        end
        UartCtrlRxState_PARITY : begin
          if(area_rxBitTimer_tick) begin
            area_rxStateMachine_state <= UartCtrlRxState_STOP;
            area_rxBitCounter_value <= 4'b0000;
            if(when_Uart_l859) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        default : begin
          if(area_rxBitTimer_tick) begin
            if(when_Uart_l866) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end else begin
              if(when_Uart_l868) begin
                area_rxStateMachine_state <= UartCtrlRxState_IDLE;
              end else begin
                if(stopBitsIsHalf) begin
                  area_rxBitTimer_counter <= 3'b100;
                end
              end
            end
          end
        end
      endcase
    end
  end

  always @(posedge clockPrescalerRetarder_stretcher_clock_out) begin
    if(area_samplingTickCmdData) begin
      area_sampler_samples0_1 <= area_sampler_samples0_0;
    end
    if(area_samplingTickCmdData) begin
      area_sampler_samples0_2 <= area_sampler_samples0_1;
    end
    if(area_samplingTickCmdData) begin
      area_sampler_samples0_3 <= area_sampler_samples0_2;
    end
    if(area_samplingTickCmdData) begin
      area_sampler_samples0_4 <= area_sampler_samples0_3;
    end
    if(area_rxBitTimer_tick) begin
      area_rxStateMachine_parity <= (area_rxStateMachine_parity ^ area_sampler_value);
    end
    case(area_rxStateMachine_state)
      UartCtrlRxState_WAITMARK : begin
      end
      UartCtrlRxState_IDLE : begin
      end
      UartCtrlRxState_START : begin
        if(area_rxBitTimer_tick) begin
          area_rxStateMachine_parity <= (modeParity == UartParityType_ODD);
        end
      end
      UartCtrlRxState_DATA : begin
      end
      UartCtrlRxState_PARITY : begin
      end
      default : begin
      end
    endcase
  end

  always @(posedge clockPrescalerRetarder_stretcher_clock_out) begin
    area_rxBreak_valid_regNext <= area_rxBreak_valid;
  end

  always @(posedge clk) begin
    if(flowCCByToggle_1_io_output_valid) begin
      bufVec_0 <= flowCCByToggle_1_io_output_payload;
    end
  end


endmodule

module ShiftRegisterSerialInParallelOut (
  input               io_clk,
  input               io_input,
  input               io_en,
  output     [6:0]    io_output7,
  output     [7:0]    io_output8,
  input               reset
);

  reg                 shiftRegs_vec_0;
  reg                 shiftRegs_vec_1;
  reg                 shiftRegs_vec_2;
  reg                 shiftRegs_vec_3;
  reg                 shiftRegs_vec_4;
  reg                 shiftRegs_vec_5;
  reg                 shiftRegs_vec_6;
  reg                 shiftRegs_vec_7;

  assign io_output7 = {shiftRegs_vec_0,{shiftRegs_vec_1,{shiftRegs_vec_2,{shiftRegs_vec_3,{shiftRegs_vec_4,{shiftRegs_vec_5,shiftRegs_vec_6}}}}}};
  assign io_output8 = {shiftRegs_vec_0,{shiftRegs_vec_1,{shiftRegs_vec_2,{shiftRegs_vec_3,{shiftRegs_vec_4,{shiftRegs_vec_5,{shiftRegs_vec_6,shiftRegs_vec_7}}}}}}};
  always @(posedge io_clk) begin
    if(io_en) begin
      shiftRegs_vec_7 <= shiftRegs_vec_6;
      shiftRegs_vec_6 <= shiftRegs_vec_5;
      shiftRegs_vec_5 <= shiftRegs_vec_4;
      shiftRegs_vec_4 <= shiftRegs_vec_3;
      shiftRegs_vec_3 <= shiftRegs_vec_2;
      shiftRegs_vec_2 <= shiftRegs_vec_1;
      shiftRegs_vec_1 <= shiftRegs_vec_0;
      shiftRegs_vec_0 <= io_input;
    end
  end


endmodule

module BufferCC (
  input               io_dataIn,
  output              io_dataOut,
  input               clock_out,
  input               reset
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge clock_out) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule

module FlowCCByToggle (
  input               io_input_valid,
  input      [7:0]    io_input_payload,
  output              io_output_valid,
  output     [7:0]    io_output_payload,
  input               clock_out,
  input               reset,
  input               clk
);

  wire                bufferCC_3_io_dataIn;
  wire                bufferCC_3_io_dataOut;
  wire                inputArea_target_buffercc_io_dataOut;
  wire                reset_syncronized;
  reg                 inputArea_target;
  reg        [7:0]    inputArea_data;
  wire                outputArea_target;
  reg                 outputArea_hit;
  wire                outputArea_flow_valid;
  wire       [7:0]    outputArea_flow_payload;
  reg                 outputArea_flow_m2sPipe_valid;
  (* async_reg = "true" *) reg        [7:0]    outputArea_flow_m2sPipe_payload;

  BufferCC_1 bufferCC_3 (
    .io_dataIn  (bufferCC_3_io_dataIn ), //i
    .io_dataOut (bufferCC_3_io_dataOut), //o
    .clk        (clk                  ), //i
    .reset      (reset                )  //i
  );
  BufferCC_2 inputArea_target_buffercc (
    .io_dataIn         (inputArea_target                    ), //i
    .io_dataOut        (inputArea_target_buffercc_io_dataOut), //o
    .clk               (clk                                 ), //i
    .reset_syncronized (reset_syncronized                   )  //i
  );
  assign bufferCC_3_io_dataIn = (1'b0 ^ 1'b0);
  assign reset_syncronized = bufferCC_3_io_dataOut;
  assign outputArea_target = inputArea_target_buffercc_io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload = inputArea_data;
  assign io_output_valid = outputArea_flow_m2sPipe_valid;
  assign io_output_payload = outputArea_flow_m2sPipe_payload;
  always @(posedge clock_out or posedge reset) begin
    if(reset) begin
      inputArea_target <= 1'b0;
    end else begin
      if(io_input_valid) begin
        inputArea_target <= (! inputArea_target);
      end
    end
  end

  always @(posedge clock_out) begin
    if(io_input_valid) begin
      inputArea_data <= io_input_payload;
    end
  end

  always @(posedge clk or posedge reset_syncronized) begin
    if(reset_syncronized) begin
      outputArea_flow_m2sPipe_valid <= 1'b0;
      outputArea_hit <= 1'b0;
    end else begin
      outputArea_hit <= outputArea_target;
      outputArea_flow_m2sPipe_valid <= outputArea_flow_valid;
    end
  end

  always @(posedge clk) begin
    if(outputArea_flow_valid) begin
      outputArea_flow_m2sPipe_payload <= outputArea_flow_payload;
    end
  end


endmodule

module BufferCC_2 (
  input               io_dataIn,
  output              io_dataOut,
  input               clk,
  input               reset_syncronized
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge clk or posedge reset_syncronized) begin
    if(reset_syncronized) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

module BufferCC_1 (
  input               io_dataIn,
  output              io_dataOut,
  input               clk,
  input               reset
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge clk or posedge reset) begin
    if(reset) begin
      buffers_0 <= 1'b1;
      buffers_1 <= 1'b1;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule
