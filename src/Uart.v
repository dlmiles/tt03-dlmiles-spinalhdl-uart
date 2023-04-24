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

  wire                area_rxStateMachine_shifter_io_en;
  wire                clockPrescalerRetarder_stretcher_clock_out;
  wire                rxd_buffercc_io_dataOut;
  wire       [6:0]    area_rxStateMachine_shifter_io_output7;
  wire       [7:0]    area_rxStateMachine_shifter_io_output8;
  reg                 _zz_clockPrescaledRipplerOut;
  wire       [3:0]    _zz_when_Uart_l505;
  wire                _zz_area_sampler_value;
  wire                _zz_area_sampler_value_1;
  wire                _zz_area_sampler_value_2;
  wire                _zz_area_sampler_value_3;
  wire                _zz_area_sampler_value_4;
  wire                _zz_area_sampler_value_5;
  wire                _zz_area_sampler_value_6;
  wire       [7:0]    _zz_area_rxStateMachine_shifterOutputSized;
  wire       [3:0]    _zz_when_Uart_l823;
  wire       [3:0]    _zz_when_Uart_l847;
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
  wire                when_Uart_l419;
  wire                when_Uart_l427;
  wire                when_Uart_l433;
  wire                when_Uart_l436;
  wire                clockPrescalerRippler_taps_0;
  wire                clockPrescalerRippler_taps_1;
  wire                clockPrescalerRippler_taps_2;
  wire                clockPrescalerRippler_taps_3;
  reg        [0:0]    _zz_clockPrescalerRippler_taps_1;
  reg        [0:0]    _zz_clockPrescalerRippler_taps_2;
  reg        [0:0]    _zz_clockPrescalerRippler_taps_3;
  wire                clockPrescaledRipplerOut;
  reg        [3:0]    clockPrescalerCounter_counter;
  wire                clockPrescalerCounter_initialState;
  reg                 clockPrescalerCounter_tick;
  wire                when_Uart_l504;
  wire                when_Uart_l505;
  wire                stretcherClockIn;
  reg        [3:0]    clockPrescalerRetarder_counter;
  reg                 clockPrescalerRetarder_sel;
  wire                when_Uart_l537;
  wire                when_Uart_l539;
  reg        [0:0]    area_samplingTicker_counter;
  reg                 area_samplingTicker_tick;
  wire                area_samplingTicker_samplingTickerInputTick;
  wire                when_Uart_l581;
  wire                when_Uart_l583;
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
  wire                when_Uart_l685;
  reg        [3:0]    area_rxBitCounter_value;
  reg        [6:0]    area_rxBreak_counter;
  wire                area_rxBreak_valid;
  wire                when_Uart_l732;
  reg                 area_rxBreak_valid_regNext;
  wire                when_Uart_l736;
  reg        [2:0]    area_rxStateMachine_state;
  reg                 area_rxStateMachine_parity;
  wire       [7:0]    area_rxStateMachine_shifterOutputSized;
  wire                when_Uart_l777;
  wire                when_Uart_l790;
  wire                when_Uart_l813;
  wire                when_Uart_l823;
  wire                when_Uart_l824;
  wire                when_Uart_l838;
  wire                when_Uart_l845;
  wire                when_Uart_l847;
  wire                when_Uart_l864;
  `ifndef SYNTHESIS
  reg [63:0] area_rxStateMachine_state_string;
  `endif


  assign _zz_when_Uart_l505 = {1'd0, regPredivCtr};
  assign _zz_area_rxStateMachine_shifterOutputSized = {1'd0, area_rxStateMachine_shifter_io_output7};
  assign _zz_when_Uart_l823 = {1'd0, dataLengthZeroBased};
  assign _zz_when_Uart_l847 = {3'd0, stopBitsZeroBased};
  assign _zz_area_sampler_value = ((((1'b0 || ((_zz_area_sampler_value_1 && area_sampler_regSamples_1) && area_sampler_regSamples_2)) || (((_zz_area_sampler_value_2 && area_sampler_regSamples_0) && area_sampler_regSamples_1) && area_sampler_regSamples_3)) || (((1'b1 && area_sampler_regSamples_0) && area_sampler_regSamples_2) && area_sampler_regSamples_3)) || (((1'b1 && area_sampler_regSamples_1) && area_sampler_regSamples_2) && area_sampler_regSamples_3));
  assign _zz_area_sampler_value_3 = (((1'b1 && area_sampler_regSamples_0) && area_sampler_regSamples_1) && area_sampler_regSamples_4);
  assign _zz_area_sampler_value_4 = ((1'b1 && area_sampler_regSamples_0) && area_sampler_regSamples_2);
  assign _zz_area_sampler_value_5 = (1'b1 && area_sampler_regSamples_1);
  assign _zz_area_sampler_value_6 = 1'b1;
  assign _zz_area_sampler_value_1 = (1'b1 && area_sampler_regSamples_0);
  assign _zz_area_sampler_value_2 = 1'b1;
  clock_stretch_invert_mux clockPrescalerRetarder_stretcher (
    .clock_out (clockPrescalerRetarder_stretcher_clock_out), //o
    .clock_in  (stretcherClockIn                          ), //i
    .sel       (clockPrescalerRetarder_sel                )  //i
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
  always @(*) begin
    case(regPredivRpl)
      2'b00 : _zz_clockPrescaledRipplerOut = clockPrescalerRippler_taps_0;
      2'b01 : _zz_clockPrescaledRipplerOut = clockPrescalerRippler_taps_1;
      2'b10 : _zz_clockPrescaledRipplerOut = clockPrescalerRippler_taps_2;
      default : _zz_clockPrescaledRipplerOut = clockPrescalerRippler_taps_3;
    endcase
  end

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
    if(when_Uart_l419) begin
      if(when_Uart_l427) begin
        io_out8[4] = 1'b1;
      end
      io_out8[5] = 1'b1;
    end
    io_out8[1] = clockPrescalerRetarder_stretcher_clock_out;
    io_out8[2] = area_sampler_tick;
    io_out8[3] = area_rxBitTimer_tick;
    io_out8[7 : 4] = area_rxBitCounter_value;
    io_out8[0] = area_rxBreak_valid;
    if(when_Uart_l864) begin
      if(bufPresent_0) begin
        io_out8 = bufVec_0;
      end
    end
  end

  always @(*) begin
    io_resetCommandStrobe = 1'b0;
    if(when_Uart_l419) begin
      if(when_Uart_l427) begin
        io_resetCommandStrobe = 1'b1;
      end
    end
  end

  always @(*) begin
    rxMasterClock = 1'b0;
    if(!when_Uart_l419) begin
      if(!when_Uart_l433) begin
        if(!when_Uart_l436) begin
          rxMasterClock = clk;
        end
      end
    end
  end

  assign when_Uart_l419 = (cmd == 2'b01);
  assign when_Uart_l427 = (io_in7[6 : 5] == 2'b11);
  assign when_Uart_l433 = (cmd == 2'b10);
  assign when_Uart_l436 = (cmd == 2'b11);
  assign clockPrescalerRippler_taps_0 = rxMasterClock;
  assign clockPrescalerRippler_taps_1 = _zz_clockPrescalerRippler_taps_1[0];
  assign clockPrescalerRippler_taps_2 = _zz_clockPrescalerRippler_taps_2[0];
  assign clockPrescalerRippler_taps_3 = _zz_clockPrescalerRippler_taps_3[0];
  assign clockPrescaledRipplerOut = _zz_clockPrescaledRipplerOut;
  assign clockPrescalerCounter_initialState = 1'b1;
  assign when_Uart_l504 = (regPredivCtr != 3'b000);
  assign when_Uart_l505 = (clockPrescalerCounter_counter == _zz_when_Uart_l505);
  assign stretcherClockIn = ((regPredivCtr != 3'b000) ? clockPrescalerCounter_tick : clockPrescaledRipplerOut);
  assign when_Uart_l537 = (regPreretCtr != 4'b0000);
  assign when_Uart_l539 = (clockPrescalerRetarder_counter == 4'b0000);
  assign area_samplingTicker_samplingTickerInputTick = ((regPredivCtr != 3'b000) ? clockPrescalerCounter_tick : clockPrescalerRetarder_stretcher_clock_out);
  assign when_Uart_l581 = 1'b1;
  assign when_Uart_l583 = (area_samplingTicker_counter == 1'b0);
  assign area_samplingTick = clockPrescalerRetarder_stretcher_clock_out;
  assign area_samplingTickCmdData = (area_samplingTick && (cmd == 2'b00));
  assign area_sampler_synchroniser = rxd_buffercc_io_dataOut;
  assign area_sampler_samples0_0 = area_sampler_synchroniser;
  assign area_sampler_inputBuffer = area_sampler_synchroniser;
  always @(*) begin
    area_rxBitTimer_tick = 1'b0;
    if(area_sampler_tick) begin
      if(when_Uart_l685) begin
        area_rxBitTimer_tick = 1'b1;
      end
    end
  end

  assign when_Uart_l685 = (area_rxBitTimer_counter == 3'b000);
  assign area_rxBreak_valid = (area_rxBreak_counter == 7'h68);
  assign when_Uart_l732 = (area_samplingTickCmdData && (! area_rxBreak_valid));
  assign when_Uart_l736 = (area_rxBreak_valid && (! area_rxBreak_valid_regNext));
  assign area_rxStateMachine_shifter_io_en = (area_rxStateMachine_state == UartCtrlRxState_DATA);
  assign area_rxStateMachine_shifterOutputSized = (modeData7 ? _zz_area_rxStateMachine_shifterOutputSized : area_rxStateMachine_shifter_io_output8);
  assign when_Uart_l777 = (area_sampler_value == 1'b1);
  assign when_Uart_l790 = (area_sampler_value == 1'b0);
  assign when_Uart_l813 = (area_sampler_value == 1'b1);
  assign when_Uart_l823 = (area_rxBitCounter_value == _zz_when_Uart_l823);
  assign when_Uart_l824 = (modeParity == UartParityType_NONE);
  assign when_Uart_l838 = (area_rxStateMachine_parity != area_sampler_value);
  assign when_Uart_l845 = (! area_sampler_value);
  assign when_Uart_l847 = (area_rxBitCounter_value == _zz_when_Uart_l847);
  assign when_Uart_l864 = (cmd == 2'b00);
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
      clockPrescalerRetarder_counter <= 4'b0000;
      clockPrescalerRetarder_sel <= 1'b0;
    end else begin
      if(when_Uart_l360) begin
        rxdLast <= io_in7[6];
      end
      if(when_Uart_l419) begin
        modeData7 <= io_in7[2];
        modeParity <= io_in7[4 : 3];
        modeStop <= io_in7[6 : 5];
      end else begin
        if(when_Uart_l433) begin
          regPredivRpl <= io_in7[3 : 2];
          regPredivCtr <= io_in7[6 : 4];
        end else begin
          if(when_Uart_l436) begin
            regPreretEna <= io_in7[6];
            regPreretCtr <= io_in7[5 : 2];
          end
        end
      end
      if(when_Uart_l537) begin
        clockPrescalerRetarder_counter <= (clockPrescalerRetarder_counter - 4'b0001);
        if(when_Uart_l539) begin
          clockPrescalerRetarder_counter <= regPreretCtr;
          clockPrescalerRetarder_sel <= (! clockPrescalerRetarder_sel);
        end
      end else begin
        clockPrescalerRetarder_sel <= 1'b0;
      end
      if(!area_sampler_value) begin
        if(when_Uart_l736) begin
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
          if(area_rxBitTimer_tick) begin
            if(!when_Uart_l845) begin
              if(when_Uart_l847) begin
                bufPresent_0 <= 1'b1;
              end
            end
          end
        end
      endcase
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

  always @(posedge clockPrescaledRipplerOut or posedge reset) begin
    if(reset) begin
      clockPrescalerCounter_counter <= 4'b0000;
      clockPrescalerCounter_tick <= clockPrescalerCounter_initialState;
    end else begin
      if(when_Uart_l504) begin
        if(when_Uart_l505) begin
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
      if(when_Uart_l581) begin
        area_samplingTicker_counter <= (area_samplingTicker_counter - 1'b1);
        if(when_Uart_l583) begin
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
        if(when_Uart_l732) begin
          area_rxBreak_counter <= (area_rxBreak_counter + 7'h01);
        end
      end
      case(area_rxStateMachine_state)
        UartCtrlRxState_WAITMARK : begin
          if(area_sampler_tick) begin
            if(when_Uart_l777) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        UartCtrlRxState_IDLE : begin
          if(area_sampler_tick) begin
            if(area_rxBreak_valid) begin
              area_rxStateMachine_state <= UartCtrlRxState_WAITMARK;
            end else begin
              if(when_Uart_l790) begin
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
            if(when_Uart_l813) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        UartCtrlRxState_DATA : begin
          if(area_rxBitTimer_tick) begin
            if(when_Uart_l823) begin
              if(when_Uart_l824) begin
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
            if(when_Uart_l838) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        default : begin
          if(area_rxBitTimer_tick) begin
            if(when_Uart_l845) begin
              area_rxStateMachine_state <= UartCtrlRxState_IDLE;
            end else begin
              if(when_Uart_l847) begin
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
          if(!when_Uart_l845) begin
            if(when_Uart_l847) begin
              bufVec_0 <= area_rxStateMachine_shifterOutputSized;
            end
          end
        end
      end
    endcase
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
