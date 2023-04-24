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

  wire                clockPrescalerRetarder_stretcher_clock_out;
  wire                rxd_buffercc_io_dataOut;
  wire       [6:0]    rxStateMachine_shifter_io_output7;
  wire       [7:0]    rxStateMachine_shifter_io_output8;
  reg                 _zz_clockPrescaledOut;
  wire       [3:0]    _zz_when_Uart_l439;
  wire                _zz_sampler_value;
  wire                _zz_sampler_value_1;
  wire                _zz_sampler_value_2;
  wire                _zz_sampler_value_3;
  wire                _zz_sampler_value_4;
  wire                _zz_sampler_value_5;
  wire                _zz_sampler_value_6;
  wire       [3:0]    _zz_when_Uart_l669;
  wire       [0:0]    _zz_when_Uart_l669_1;
  wire       [1:0]    cmd;
  reg                 rxdLast;
  reg                 rxd;
  wire                when_Uart_l320;
  reg                 modeData7;
  reg        [1:0]    modeParity;
  reg        [1:0]    modeStop;
  reg        [3:0]    dataLength;
  reg        [1:0]    regPredivRpl;
  reg        [2:0]    regPredivCtr;
  reg                 regPreretEna;
  reg        [3:0]    regPreretCtr;
  reg        [7:0]    bufVec_0;
  reg                 bufPresent_0;
  wire                rxErrorParity;
  wire                rxErrorStop;
  reg                 rxMasterClock;
  wire                when_Uart_l369;
  wire                when_Uart_l377;
  wire                when_Uart_l383;
  wire                when_Uart_l386;
  wire                clockPrescalerDivider_taps_0;
  wire                clockPrescalerDivider_taps_1;
  wire                clockPrescalerDivider_taps_2;
  wire                clockPrescalerDivider_taps_3;
  reg        [0:0]    _zz_clockPrescalerDivider_taps_1;
  reg        [0:0]    _zz_clockPrescalerDivider_taps_2;
  reg        [0:0]    _zz_clockPrescalerDivider_taps_3;
  wire                clockPrescaledOut;
  reg        [3:0]    clockPrescalerCounter_counter;
  reg                 clockPrescalerCounter_tick;
  wire                clockPrescalerCounter_clockPrescalerCounterInputTick;
  wire                when_Uart_l439;
  wire                stretcherClockIn;
  reg        [3:0]    clockPrescalerRetarder_counter;
  reg                 clockPrescalerRetarder_sel;
  wire                when_Uart_l452;
  wire                when_Uart_l454;
  reg        [0:0]    samplingTicker_counter;
  reg                 samplingTicker_tick;
  wire                samplingTicker_samplingTickerInputTick;
  wire                when_Uart_l481;
  wire                when_Uart_l483;
  wire                samplingTick;
  wire                samplingTickCmdData;
  reg                 samplingTickCmdDataRegNext;
  wire                sampler_synchroniser;
  wire                sampler_samples_0;
  reg                 sampler_samples_1;
  reg                 sampler_samples_2;
  reg                 sampler_samples_3;
  reg                 sampler_samples_4;
  reg                 sampler_value;
  reg                 sampler_tick;
  reg        [2:0]    rxBitTimer_counter;
  reg                 rxBitTimer_tick;
  wire                when_Uart_l545;
  reg        [3:0]    rxBitCounter_value;
  reg        [6:0]    rxBreak_counter;
  wire                rxBreak_valid;
  wire                when_Uart_l588;
  reg        [2:0]    rxStateMachine_state;
  reg                 rxStateMachine_parity;
  reg                 rxStateMachine_shifterVec_0;
  reg                 rxStateMachine_shifterVec_1;
  reg                 rxStateMachine_shifterVec_2;
  reg                 rxStateMachine_shifterVec_3;
  reg                 rxStateMachine_shifterVec_4;
  reg                 rxStateMachine_shifterVec_5;
  reg                 rxStateMachine_shifterVec_6;
  reg                 rxStateMachine_shifterVec_7;
  wire                when_Uart_l617;
  wire                when_Uart_l622;
  wire                when_Uart_l633;
  wire       [7:0]    _zz_1;
  wire                when_Uart_l643;
  wire                when_Uart_l646;
  wire                when_Uart_l659;
  wire                when_Uart_l667;
  wire                when_Uart_l669;
  wire                when_Uart_l682;
  `ifndef SYNTHESIS
  reg [63:0] rxStateMachine_state_string;
  `endif


  assign _zz_when_Uart_l439 = {1'd0, regPredivCtr};
  assign _zz_when_Uart_l669_1 = 1'b1;
  assign _zz_when_Uart_l669 = {3'd0, _zz_when_Uart_l669_1};
  assign _zz_sampler_value = ((((1'b0 || ((_zz_sampler_value_1 && sampler_samples_1) && sampler_samples_2)) || (((_zz_sampler_value_2 && sampler_samples_0) && sampler_samples_1) && sampler_samples_3)) || (((1'b1 && sampler_samples_0) && sampler_samples_2) && sampler_samples_3)) || (((1'b1 && sampler_samples_1) && sampler_samples_2) && sampler_samples_3));
  assign _zz_sampler_value_3 = (((1'b1 && sampler_samples_0) && sampler_samples_1) && sampler_samples_4);
  assign _zz_sampler_value_4 = ((1'b1 && sampler_samples_0) && sampler_samples_2);
  assign _zz_sampler_value_5 = (1'b1 && sampler_samples_1);
  assign _zz_sampler_value_6 = 1'b1;
  assign _zz_sampler_value_1 = (1'b1 && sampler_samples_0);
  assign _zz_sampler_value_2 = 1'b1;
  clock_stretch_invert_mux clockPrescalerRetarder_stretcher (
    .clock_out (clockPrescalerRetarder_stretcher_clock_out), //o
    .clock_in  (stretcherClockIn                          ), //i
    .sel       (clockPrescalerRetarder_sel                )  //i
  );
  BufferCC rxd_buffercc (
    .io_dataIn  (rxd                    ), //i
    .io_dataOut (rxd_buffercc_io_dataOut), //o
    .clk        (clk                    ), //i
    .reset      (reset                  )  //i
  );
  ShiftRegisterSerialInParallelOut rxStateMachine_shifter (
    .io_clk     (rxBitTimer_tick                       ), //i
    .io_input   (sampler_value                         ), //i
    .io_en      (1'b1                                  ), //i
    .io_output7 (rxStateMachine_shifter_io_output7[6:0]), //o
    .io_output8 (rxStateMachine_shifter_io_output8[7:0]), //o
    .reset      (reset                                 )  //i
  );
  always @(*) begin
    case(regPredivRpl)
      2'b00 : _zz_clockPrescaledOut = clockPrescalerDivider_taps_0;
      2'b01 : _zz_clockPrescaledOut = clockPrescalerDivider_taps_1;
      2'b10 : _zz_clockPrescaledOut = clockPrescalerDivider_taps_2;
      default : _zz_clockPrescaledOut = clockPrescalerDivider_taps_3;
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(rxStateMachine_state)
      UartCtrlRxState_WAITMARK : rxStateMachine_state_string = "WAITMARK";
      UartCtrlRxState_IDLE : rxStateMachine_state_string = "IDLE    ";
      UartCtrlRxState_START : rxStateMachine_state_string = "START   ";
      UartCtrlRxState_DATA : rxStateMachine_state_string = "DATA    ";
      UartCtrlRxState_PARITY : rxStateMachine_state_string = "PARITY  ";
      UartCtrlRxState_STOP : rxStateMachine_state_string = "STOP    ";
      default : rxStateMachine_state_string = "????????";
    endcase
  end
  `endif

  assign io_gatedTxdStopBitSupport = 1'b0;
  assign cmd = io_in7[1 : 0];
  assign when_Uart_l320 = (cmd == 2'b00);
  always @(*) begin
    if(when_Uart_l320) begin
      rxd = io_in7[6];
    end else begin
      rxd = rxdLast;
    end
  end

  always @(*) begin
    if(modeData7) begin
      dataLength = 4'b0111;
    end else begin
      dataLength = 4'b1000;
    end
  end

  always @(*) begin
    io_out8 = 8'h00;
    if(when_Uart_l369) begin
      if(when_Uart_l377) begin
        io_out8[4] = 1'b1;
      end
      io_out8[5] = 1'b1;
    end
    io_out8[1] = stretcherClockIn;
    io_out8[2] = sampler_tick;
    io_out8[3] = rxBitTimer_tick;
    io_out8[7 : 4] = rxBitCounter_value;
    io_out8[0] = rxBreak_valid;
    case(rxStateMachine_state)
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
        if(rxBitTimer_tick) begin
          if(!when_Uart_l667) begin
            if(when_Uart_l669) begin
              io_out8 = rxStateMachine_shifter_io_output8;
            end
          end
        end
      end
    endcase
    if(when_Uart_l682) begin
      if(bufPresent_0) begin
        io_out8 = bufVec_0;
      end
    end
  end

  always @(*) begin
    io_resetCommandStrobe = 1'b0;
    if(when_Uart_l369) begin
      if(when_Uart_l377) begin
        io_resetCommandStrobe = 1'b1;
      end
    end
  end

  always @(*) begin
    rxMasterClock = 1'b0;
    if(!when_Uart_l369) begin
      if(!when_Uart_l383) begin
        if(!when_Uart_l386) begin
          rxMasterClock = clk;
        end
      end
    end
  end

  assign when_Uart_l369 = (cmd == 2'b01);
  assign when_Uart_l377 = (io_in7[6 : 5] == 2'b11);
  assign when_Uart_l383 = (cmd == 2'b10);
  assign when_Uart_l386 = (cmd == 2'b11);
  assign clockPrescalerDivider_taps_0 = rxMasterClock;
  assign clockPrescalerDivider_taps_1 = _zz_clockPrescalerDivider_taps_1[0];
  assign clockPrescalerDivider_taps_2 = _zz_clockPrescalerDivider_taps_2[0];
  assign clockPrescalerDivider_taps_3 = _zz_clockPrescalerDivider_taps_3[0];
  assign clockPrescaledOut = _zz_clockPrescaledOut;
  assign clockPrescalerCounter_clockPrescalerCounterInputTick = ((regPredivRpl != 2'b00) ? clockPrescaledOut : 1'b1);
  assign when_Uart_l439 = (clockPrescalerCounter_counter == _zz_when_Uart_l439);
  assign stretcherClockIn = ((regPredivCtr != 3'b000) ? clockPrescalerCounter_tick : clk);
  assign when_Uart_l452 = (regPreretCtr != 4'b0000);
  assign when_Uart_l454 = (clockPrescalerRetarder_counter == 4'b0000);
  assign samplingTicker_samplingTickerInputTick = ((regPredivCtr != 3'b000) ? clockPrescalerCounter_tick : clk);
  assign when_Uart_l481 = 1'b1;
  assign when_Uart_l483 = (samplingTicker_counter == 1'b0);
  assign samplingTick = stretcherClockIn;
  assign samplingTickCmdData = (samplingTick && (cmd == 2'b00));
  assign sampler_synchroniser = rxd_buffercc_io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @(*) begin
    rxBitTimer_tick = 1'b0;
    if(sampler_tick) begin
      if(when_Uart_l545) begin
        rxBitTimer_tick = 1'b1;
      end
    end
  end

  assign when_Uart_l545 = (rxBitTimer_counter == 3'b000);
  assign rxBreak_valid = (rxBreak_counter == 7'h68);
  assign when_Uart_l588 = (samplingTickCmdData && (! rxBreak_valid));
  assign when_Uart_l617 = (sampler_value == 1'b1);
  assign when_Uart_l622 = (sampler_value == 1'b0);
  assign when_Uart_l633 = (sampler_value == 1'b1);
  assign _zz_1 = ({7'd0,1'b1} <<< rxBitCounter_value[2 : 0]);
  assign when_Uart_l643 = (rxBitCounter_value == dataLength);
  assign when_Uart_l646 = (modeParity == UartParityType_NONE);
  assign when_Uart_l659 = (rxStateMachine_parity != sampler_value);
  assign when_Uart_l667 = (! sampler_value);
  assign when_Uart_l669 = (rxBitCounter_value == _zz_when_Uart_l669);
  assign when_Uart_l682 = (cmd == 2'b00);
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
      bufVec_0 <= 8'h00;
      bufPresent_0 <= 1'b0;
      clockPrescalerCounter_counter <= 4'b0000;
      clockPrescalerCounter_tick <= 1'b0;
      clockPrescalerRetarder_counter <= 4'b0000;
      clockPrescalerRetarder_sel <= 1'b0;
      samplingTicker_counter <= 1'b0;
      samplingTicker_tick <= 1'b0;
      sampler_samples_1 <= 1'b1;
      sampler_samples_2 <= 1'b1;
      sampler_samples_3 <= 1'b1;
      sampler_samples_4 <= 1'b1;
      sampler_value <= 1'b1;
      sampler_tick <= 1'b1;
      rxBitTimer_counter <= 3'b000;
      rxBitCounter_value <= 4'b0000;
      rxBreak_counter <= 7'h00;
      rxStateMachine_state <= UartCtrlRxState_WAITMARK;
    end else begin
      if(when_Uart_l320) begin
        rxdLast <= io_in7[6];
      end
      if(when_Uart_l369) begin
        modeData7 <= io_in7[2];
        modeParity <= io_in7[4 : 3];
        modeStop <= io_in7[6 : 5];
      end else begin
        if(when_Uart_l383) begin
          regPredivRpl <= io_in7[3 : 2];
          regPredivCtr <= io_in7[6 : 4];
        end else begin
          if(when_Uart_l386) begin
            regPreretEna <= io_in7[6];
            regPreretCtr <= io_in7[5 : 2];
          end
        end
      end
      if(clockPrescalerCounter_clockPrescalerCounterInputTick) begin
        clockPrescalerCounter_counter <= (clockPrescalerCounter_counter + 4'b0001);
        if(when_Uart_l439) begin
          clockPrescalerCounter_counter <= 4'b0000;
          clockPrescalerCounter_tick <= (! clockPrescalerCounter_tick);
        end
      end
      if(when_Uart_l452) begin
        clockPrescalerRetarder_counter <= (clockPrescalerRetarder_counter - 4'b0001);
        if(when_Uart_l454) begin
          clockPrescalerRetarder_counter <= regPreretCtr;
          clockPrescalerRetarder_sel <= (! clockPrescalerRetarder_sel);
        end
      end else begin
        clockPrescalerRetarder_sel <= 1'b0;
      end
      samplingTicker_tick <= 1'b0;
      if(when_Uart_l481) begin
        samplingTicker_counter <= (samplingTicker_counter - 1'b1);
        if(when_Uart_l483) begin
          samplingTicker_tick <= 1'b1;
        end
      end
      if(samplingTickCmdDataRegNext) begin
        sampler_samples_1 <= sampler_samples_0;
      end
      if(samplingTickCmdDataRegNext) begin
        sampler_samples_2 <= sampler_samples_1;
      end
      if(samplingTickCmdDataRegNext) begin
        sampler_samples_3 <= sampler_samples_2;
      end
      if(samplingTickCmdDataRegNext) begin
        sampler_samples_4 <= sampler_samples_3;
      end
      sampler_value <= ((((((_zz_sampler_value || _zz_sampler_value_3) || (_zz_sampler_value_4 && sampler_samples_4)) || ((_zz_sampler_value_5 && sampler_samples_2) && sampler_samples_4)) || (((_zz_sampler_value_6 && sampler_samples_0) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_1) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_2) && sampler_samples_3) && sampler_samples_4));
      sampler_tick <= samplingTickCmdDataRegNext;
      if(sampler_tick) begin
        rxBitTimer_counter <= (rxBitTimer_counter - 3'b001);
      end
      if(rxBitTimer_tick) begin
        rxBitCounter_value <= (rxBitCounter_value + 4'b0001);
      end
      if(sampler_value) begin
        rxBreak_counter <= 7'h00;
      end else begin
        if(when_Uart_l588) begin
          rxBreak_counter <= (rxBreak_counter + 7'h01);
        end
      end
      case(rxStateMachine_state)
        UartCtrlRxState_WAITMARK : begin
          if(when_Uart_l617) begin
            rxStateMachine_state <= UartCtrlRxState_IDLE;
          end
        end
        UartCtrlRxState_IDLE : begin
          if(when_Uart_l622) begin
            rxStateMachine_state <= UartCtrlRxState_START;
            rxBitTimer_counter <= 3'b010;
          end
        end
        UartCtrlRxState_START : begin
          rxStateMachine_state <= UartCtrlRxState_DATA;
          rxBitCounter_value <= 4'b0000;
          if(when_Uart_l633) begin
            rxStateMachine_state <= UartCtrlRxState_IDLE;
          end
        end
        UartCtrlRxState_DATA : begin
          bufPresent_0 <= 1'b0;
          if(rxBitTimer_tick) begin
            if(when_Uart_l643) begin
              bufVec_0 <= rxStateMachine_shifter_io_output8;
              rxBitCounter_value <= 4'b0000;
              if(when_Uart_l646) begin
                rxStateMachine_state <= UartCtrlRxState_STOP;
              end else begin
                rxStateMachine_state <= UartCtrlRxState_PARITY;
              end
            end
          end
        end
        UartCtrlRxState_PARITY : begin
          if(rxBitTimer_tick) begin
            rxStateMachine_state <= UartCtrlRxState_STOP;
            rxBitCounter_value <= 4'b0000;
            if(when_Uart_l659) begin
              rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        default : begin
          if(rxBitTimer_tick) begin
            if(when_Uart_l667) begin
              rxStateMachine_state <= UartCtrlRxState_IDLE;
            end else begin
              if(when_Uart_l669) begin
                rxStateMachine_state <= UartCtrlRxState_IDLE;
                bufPresent_0 <= 1'b1;
              end
            end
          end
        end
      endcase
    end
  end

  always @(posedge clockPrescalerDivider_taps_0 or posedge reset) begin
    if(reset) begin
      _zz_clockPrescalerDivider_taps_1 <= 1'b0;
    end else begin
      _zz_clockPrescalerDivider_taps_1 <= (~ _zz_clockPrescalerDivider_taps_1);
    end
  end

  always @(posedge clockPrescalerDivider_taps_1 or posedge reset) begin
    if(reset) begin
      _zz_clockPrescalerDivider_taps_2 <= 1'b0;
    end else begin
      _zz_clockPrescalerDivider_taps_2 <= (~ _zz_clockPrescalerDivider_taps_2);
    end
  end

  always @(posedge clockPrescalerDivider_taps_2 or posedge reset) begin
    if(reset) begin
      _zz_clockPrescalerDivider_taps_3 <= 1'b0;
    end else begin
      _zz_clockPrescalerDivider_taps_3 <= (~ _zz_clockPrescalerDivider_taps_3);
    end
  end

  always @(posedge clk) begin
    samplingTickCmdDataRegNext <= samplingTickCmdData;
    if(rxBitTimer_tick) begin
      rxStateMachine_parity <= (rxStateMachine_parity ^ sampler_value);
    end
    case(rxStateMachine_state)
      UartCtrlRxState_WAITMARK : begin
      end
      UartCtrlRxState_IDLE : begin
      end
      UartCtrlRxState_START : begin
        rxStateMachine_parity <= (modeParity == UartParityType_ODD);
      end
      UartCtrlRxState_DATA : begin
        if(rxBitTimer_tick) begin
          if(_zz_1[0]) begin
            rxStateMachine_shifterVec_0 <= sampler_value;
          end
          if(_zz_1[1]) begin
            rxStateMachine_shifterVec_1 <= sampler_value;
          end
          if(_zz_1[2]) begin
            rxStateMachine_shifterVec_2 <= sampler_value;
          end
          if(_zz_1[3]) begin
            rxStateMachine_shifterVec_3 <= sampler_value;
          end
          if(_zz_1[4]) begin
            rxStateMachine_shifterVec_4 <= sampler_value;
          end
          if(_zz_1[5]) begin
            rxStateMachine_shifterVec_5 <= sampler_value;
          end
          if(_zz_1[6]) begin
            rxStateMachine_shifterVec_6 <= sampler_value;
          end
          if(_zz_1[7]) begin
            rxStateMachine_shifterVec_7 <= sampler_value;
          end
        end
      end
      UartCtrlRxState_PARITY : begin
      end
      default : begin
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
  input               clk,
  input               reset
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule
