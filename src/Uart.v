// Generator : SpinalHDL dev    git head : ab2403fd43aa92126d173aaad3edce3eac04ef89
// Component : Uart
// Git hash  : b79a254f8ef9eb05864c432f913087cb0687d2e3

`timescale 1ns/1ps

module Uart (
  input      [6:0]    io_in7,
  output reg [7:0]    io_out8,
  output reg          io_resetCommandStrobe,
  output              io_gatedTxdStopBitSupport,
  input               reset,
  input               clk
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

  wire                rxd_buffercc_io_dataOut;
  wire       [6:0]    rxStateMachine_shifter_io_output7;
  wire       [7:0]    rxStateMachine_shifter_io_output8;
  wire                _zz_sampler_value;
  wire                _zz_sampler_value_1;
  wire                _zz_sampler_value_2;
  wire                _zz_sampler_value_3;
  wire                _zz_sampler_value_4;
  wire                _zz_sampler_value_5;
  wire                _zz_sampler_value_6;
  wire       [3:0]    _zz_when_Uart_l533;
  wire       [2:0]    _zz_when_Uart_l558;
  wire       [0:0]    _zz_when_Uart_l558_1;
  reg                 _zz_clockPrescaledOut;
  wire       [1:0]    cmd;
  wire                rxd;
  reg                 modeData7;
  reg        [1:0]    modeParity;
  reg        [1:0]    modeStop;
  reg        [3:0]    dataLength;
  reg        [1:0]    clkpre;
  reg        [2:0]    clknco;
  reg        [7:0]    bufVec_0;
  reg                 bufPresent_0;
  wire                rxErrorParity;
  wire                rxErrorStop;
  wire                when_Uart_l342;
  wire                when_Uart_l350;
  wire                when_Uart_l356;
  reg        [4:0]    samplingTicker_counter;
  reg                 samplingTicker_tick;
  wire                when_Uart_l382;
  wire                when_Uart_l384;
  wire                samplingTick;
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
  wire                when_Uart_l436;
  reg        [2:0]    rxBitCounter_value;
  reg        [6:0]    rxBreak_counter;
  wire                rxBreak_valid;
  wire                when_Uart_l479;
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
  wire                when_Uart_l508;
  wire                when_Uart_l513;
  wire                when_Uart_l524;
  wire       [7:0]    _zz_1;
  wire                when_Uart_l533;
  wire                when_Uart_l535;
  wire                when_Uart_l548;
  wire                when_Uart_l556;
  wire                when_Uart_l558;
  wire                clockPrescaler_taps_0;
  wire                clockPrescaler_taps_1;
  wire                clockPrescaler_taps_2;
  reg                 _zz_clockPrescaler_taps_1;
  reg                 _zz_clockPrescaler_taps_2;
  wire                clockPrescaledOut;
  `ifndef SYNTHESIS
  reg [63:0] rxStateMachine_state_string;
  `endif


  assign _zz_when_Uart_l533 = {1'd0, rxBitCounter_value};
  assign _zz_when_Uart_l558_1 = 1'b1;
  assign _zz_when_Uart_l558 = {2'd0, _zz_when_Uart_l558_1};
  assign _zz_sampler_value = ((((1'b0 || ((_zz_sampler_value_1 && sampler_samples_1) && sampler_samples_2)) || (((_zz_sampler_value_2 && sampler_samples_0) && sampler_samples_1) && sampler_samples_3)) || (((1'b1 && sampler_samples_0) && sampler_samples_2) && sampler_samples_3)) || (((1'b1 && sampler_samples_1) && sampler_samples_2) && sampler_samples_3));
  assign _zz_sampler_value_3 = (((1'b1 && sampler_samples_0) && sampler_samples_1) && sampler_samples_4);
  assign _zz_sampler_value_4 = ((1'b1 && sampler_samples_0) && sampler_samples_2);
  assign _zz_sampler_value_5 = (1'b1 && sampler_samples_1);
  assign _zz_sampler_value_6 = 1'b1;
  assign _zz_sampler_value_1 = (1'b1 && sampler_samples_0);
  assign _zz_sampler_value_2 = 1'b1;
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
    case(clkpre)
      2'b00 : _zz_clockPrescaledOut = clockPrescaler_taps_0;
      2'b01 : _zz_clockPrescaledOut = clockPrescaler_taps_1;
      default : _zz_clockPrescaledOut = clockPrescaler_taps_2;
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
  assign rxd = io_in7[6];
  always @(*) begin
    if(modeData7) begin
      dataLength = 4'b0111;
    end else begin
      dataLength = 4'b1000;
    end
  end

  always @(*) begin
    io_out8 = 8'h00;
    if(when_Uart_l342) begin
      if(when_Uart_l350) begin
        io_out8[4] = 1'b1;
      end
      io_out8[5] = 1'b1;
    end else begin
      if(!when_Uart_l356) begin
        io_out8[0] = bufPresent_0;
      end
    end
    io_out8[1] = samplingTicker_tick;
    io_out8[2] = sampler_tick;
    io_out8[3] = rxBitTimer_tick;
    io_out8[7 : 5] = rxBitCounter_value;
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
          if(!when_Uart_l556) begin
            if(when_Uart_l558) begin
              io_out8 = rxStateMachine_shifter_io_output8;
            end
          end
        end
      end
    endcase
  end

  always @(*) begin
    io_resetCommandStrobe = 1'b0;
    if(when_Uart_l342) begin
      if(when_Uart_l350) begin
        io_resetCommandStrobe = 1'b1;
      end
    end
  end

  assign when_Uart_l342 = (cmd == 2'b01);
  assign when_Uart_l350 = (io_in7[6 : 5] == 2'b11);
  assign when_Uart_l356 = (cmd == 2'b10);
  assign when_Uart_l382 = 1'b1;
  assign when_Uart_l384 = (samplingTicker_counter == 5'h00);
  assign samplingTick = samplingTicker_tick;
  assign sampler_synchroniser = rxd_buffercc_io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @(*) begin
    rxBitTimer_tick = 1'b0;
    if(sampler_tick) begin
      if(when_Uart_l436) begin
        rxBitTimer_tick = 1'b1;
      end
    end
  end

  assign when_Uart_l436 = (rxBitTimer_counter == 3'b000);
  assign rxBreak_valid = (rxBreak_counter == 7'h68);
  assign when_Uart_l479 = (samplingTick && (! rxBreak_valid));
  assign when_Uart_l508 = (sampler_value == 1'b1);
  assign when_Uart_l513 = (sampler_value == 1'b0);
  assign when_Uart_l524 = (sampler_value == 1'b1);
  assign _zz_1 = ({7'd0,1'b1} <<< rxBitCounter_value);
  assign when_Uart_l533 = (_zz_when_Uart_l533 == dataLength);
  assign when_Uart_l535 = (modeParity == UartParityType_NONE);
  assign when_Uart_l548 = (rxStateMachine_parity != sampler_value);
  assign when_Uart_l556 = (! sampler_value);
  assign when_Uart_l558 = (rxBitCounter_value == _zz_when_Uart_l558);
  assign clockPrescaler_taps_0 = clk;
  assign clockPrescaler_taps_1 = _zz_clockPrescaler_taps_1;
  assign clockPrescaler_taps_2 = _zz_clockPrescaler_taps_2;
  assign clockPrescaledOut = _zz_clockPrescaledOut;
  always @(posedge clk or posedge reset) begin
    if(reset) begin
      modeData7 <= 1'b0;
      modeParity <= 2'b00;
      modeStop <= 2'b00;
      clkpre <= 2'b00;
      clknco <= 3'b000;
      bufVec_0 <= 8'h00;
      bufPresent_0 <= 1'b0;
      rxBreak_counter <= 7'h00;
      rxStateMachine_state <= UartCtrlRxState_WAITMARK;
    end else begin
      if(when_Uart_l342) begin
        modeData7 <= io_in7[2];
        modeParity <= io_in7[4 : 3];
        modeStop <= io_in7[6 : 5];
      end else begin
        if(when_Uart_l356) begin
          clkpre <= io_in7[3 : 2];
          clknco <= io_in7[6 : 4];
        end
      end
      if(sampler_value) begin
        rxBreak_counter <= 7'h00;
      end else begin
        if(when_Uart_l479) begin
          rxBreak_counter <= (rxBreak_counter + 7'h01);
        end
      end
      case(rxStateMachine_state)
        UartCtrlRxState_WAITMARK : begin
          if(when_Uart_l508) begin
            rxStateMachine_state <= UartCtrlRxState_IDLE;
          end
        end
        UartCtrlRxState_IDLE : begin
          if(when_Uart_l513) begin
            rxStateMachine_state <= UartCtrlRxState_START;
          end
        end
        UartCtrlRxState_START : begin
          rxStateMachine_state <= UartCtrlRxState_DATA;
          if(when_Uart_l524) begin
            rxStateMachine_state <= UartCtrlRxState_IDLE;
          end
        end
        UartCtrlRxState_DATA : begin
          if(rxBitTimer_tick) begin
            if(when_Uart_l533) begin
              if(when_Uart_l535) begin
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
            if(when_Uart_l548) begin
              rxStateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        default : begin
          if(rxBitTimer_tick) begin
            if(when_Uart_l556) begin
              rxStateMachine_state <= UartCtrlRxState_IDLE;
            end else begin
              if(when_Uart_l558) begin
                rxStateMachine_state <= UartCtrlRxState_IDLE;
                bufPresent_0 <= 1'b1;
                bufVec_0 <= rxStateMachine_shifter_io_output8;
              end
            end
          end
        end
      endcase
    end
  end

  always @(posedge clk) begin
    if(when_Uart_l382) begin
      samplingTicker_counter <= (samplingTicker_counter - 5'h01);
      if(when_Uart_l384) begin
        samplingTicker_tick <= (! samplingTicker_tick);
        samplingTicker_counter <= 5'h1e;
      end
    end
    if(samplingTick) begin
      sampler_samples_1 <= sampler_samples_0;
    end
    if(samplingTick) begin
      sampler_samples_2 <= sampler_samples_1;
    end
    if(samplingTick) begin
      sampler_samples_3 <= sampler_samples_2;
    end
    if(samplingTick) begin
      sampler_samples_4 <= sampler_samples_3;
    end
    sampler_value <= ((((((_zz_sampler_value || _zz_sampler_value_3) || (_zz_sampler_value_4 && sampler_samples_4)) || ((_zz_sampler_value_5 && sampler_samples_2) && sampler_samples_4)) || (((_zz_sampler_value_6 && sampler_samples_0) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_1) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_2) && sampler_samples_3) && sampler_samples_4));
    sampler_tick <= samplingTick;
    if(sampler_tick) begin
      rxBitTimer_counter <= (rxBitTimer_counter - 3'b001);
    end
    if(rxBitTimer_tick) begin
      rxBitCounter_value <= (rxBitCounter_value + 3'b001);
    end
    if(rxBitTimer_tick) begin
      rxStateMachine_parity <= (rxStateMachine_parity ^ sampler_value);
    end
    case(rxStateMachine_state)
      UartCtrlRxState_WAITMARK : begin
      end
      UartCtrlRxState_IDLE : begin
        if(when_Uart_l513) begin
          rxBitTimer_counter <= 3'b010;
        end
      end
      UartCtrlRxState_START : begin
        rxBitCounter_value <= 3'b000;
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
          if(when_Uart_l533) begin
            rxBitCounter_value <= 3'b000;
          end
        end
      end
      UartCtrlRxState_PARITY : begin
        if(rxBitTimer_tick) begin
          rxBitCounter_value <= 3'b000;
        end
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
