`timescale 1ns/1ps

module Music_fpga (
    input clk,
    
    //keyboard
    input key_rst,
    inout PS2_DATA,
    inout PS2_CLK,
    
    //music
    output pmod_1, //AIN
    output pmod_2, //GAIN
    output pmod_4 //SHUTdescend_N
  );
  
//make code for keyboard keys
  parameter 
  ENTER = 9'b0_0101_1010, //5A
  ENTER_RIGHT = 9'b1_0101_1010, //E05A
  KEY_W = 9'b0_0001_1101, //1D ascend W
  KEY_S = 9'b0_0001_1011, //1B descend S
  KEY_R = 9'b0_0010_1101, //2D speed R
  DUTY_BEST = 10'd512;

  wire [511:0] key_down;
  wire [8:0] last_change;
  wire been_ready;

  assign pmod_2 = 1'b1;   // no gain (6dB)
  assign pmod_4 = 1'b1;   // turn-on

  reg rst;
  reg keyW_pressed;
  reg keyS_pressed;
  reg keyR_pressed;

  wire speed_state;    // 0: 0.5 sec, 1: 1 sec
  wire beat_freq;       // div_sig for ascending / descending

  wire [4:0] noteFreq;
  wire [31:0] freq;


  KeyboardDecoder decoder (
                    .key_down(key_down),
                    .last_change(last_change),
                    .key_valid(been_ready),
                    .PS2_DATA(PS2_DATA),
                    .PS2_CLK(PS2_CLK),
                    .clk(clk),
                    .rst(kb_rst)
                  );

  Pace_Control pace (
                    .clk(clk),
                    .rst(rst),
                    .ctrl(keyR_pressed),
                    .speed_state(speed_state)
                  );

  ClockDivider clkDiv (
                 .clk(clk),
                 .rst(rst),
                 .speed_state(speed_state),
                 .div_sig(beat_freq)
               );

  noteController note_controller (
                   .clk(clk),
                   .rst(rst),
                   .beat_freq(beat_freq),
                   .asc_ctrl(keyW_pressed),
                   .dsc_ctrl(keyS_pressed),
                   .noteFreq(noteFreq)
                 );

  noteFrequency music_sheet (
                 .noteFreq(noteFreq),
                 .note(freq)
               );

  PWM_gen note_generator (
            .clk(clk),
            .reset(rst),
            .freq(freq),
            .duty(DUTY_BEST),
            .PWM(pmod_1)
          );

  // keyboard input
  always @(posedge clk) begin
    if (been_ready && key_down[last_change] == 1'b1) begin
      rst <= (last_change == ENTER || last_change == ENTER_RIGHT) ? 1'b1 : 1'b0;
      keyW_pressed <= (last_change == KEY_W) ? 1'b1 : 1'b0;
      keyS_pressed <= (last_change == KEY_S) ? 1'b1 : 1'b0;
      keyR_pressed <= (last_change == KEY_R) ? 1'b1 : 1'b0;
    end
    else begin
      rst <= 1'b0;
      keyW_pressed <= 1'b0;
      keyS_pressed <= 1'b0;
      keyR_pressed <= 1'b0;
    end
  end

endmodule


module Pace_Control (
    input clk,
    input rst,
    input ctrl,
    output reg speed_state
  );

  always @(posedge clk) begin
    if (rst == 1'b1) speed_state <= 1'b0;
    else begin
      if (ctrl == 1'b1) speed_state <= ~speed_state;
      else speed_state <= speed_state;
   end
  end
endmodule


module ClockDivider (
    input clk,
    input rst,
    input speed_state,
    output reg div_sig
  );

  parameter [31:0] FREQ_05 = 32'd_50_000_000;   // 0.5 sec
  parameter [31:0] FREQ_1 = 32'd100_000_000;   // 1   sec

  reg [31:0] cnt;

  // counter
  always @(posedge clk) begin
    if (rst == 1'b1) cnt <= 32'b0;
    else begin
      if (speed_state == 1'b0)begin
        if (cnt >= FREQ_05 - 32'b1) cnt <= 32'b0;
        else cnt <= cnt + 32'b1;
  end
      else begin
        if (cnt >= FREQ_1 - 32'b1) cnt <= 32'b0;
        else cnt <= cnt + 32'b1;
  end
 end
  end

  // div_sig (output)
  always @(*) begin
    if (speed_state == 1'b0) begin
      if (cnt >= FREQ_05 - 32'b1) div_sig = 1'b1;
      else div_sig = 1'b0;
   end
    else begin
      if (cnt >= FREQ_1 - 32'b1) div_sig = 1'b1;
      else div_sig = 1'b0;
   end
  end

endmodule


module noteController (
    input clk,
    input rst,
    input beat_freq,
    input asc_ctrl,
    input dsc_ctrl,
    output reg [4:0] noteFreq
  );

  parameter HIGHEST_note = 5'd28;
  parameter LOWEST_note = 5'd0;

  reg [4:0] next_noteFreq;
  reg stateAsc;
  reg next_stateAsc;

  // note (update with div_sig)
  always @(posedge clk) begin
    if (rst == 1'b1) noteFreq <= 5'd0;
    else begin
      if (beat_freq == 1'b1)
        noteFreq <= next_noteFreq;
      else
        noteFreq <= noteFreq;
 end
  end

  // ascend (update with clk)
  always @(posedge clk) begin
    if (rst == 1'b1) stateAsc <= 1'b1;
    else stateAsc <= next_stateAsc;
  end

  // next ascend
  always @(*) begin
    if (asc_ctrl == 1'b1) next_stateAsc = 1'b1;
    else begin
      if (dsc_ctrl == 1'b1) next_stateAsc = 1'b0;
      else next_stateAsc = stateAsc;
   end
  end

  // next note
  always @(*) begin
    if (stateAsc == 1'b1) begin
      if (noteFreq == HIGHEST_note) next_noteFreq = noteFreq;
      else next_noteFreq = noteFreq + 5'b1;
    end
    else begin
      if (noteFreq == LOWEST_note) next_noteFreq = noteFreq;
      else next_noteFreq = noteFreq - 5'b1;
    end
  end

endmodule


module noteFrequency (
    input [4:0] noteFreq,
    output reg [31:0] note
  );

  parameter NM1 = 32'd262; // C_freq
  parameter NM2 = 32'd294; // D_freq
  parameter NM3 = 32'd330; // E_freq
  parameter NM4 = 32'd349; // F_freq
  parameter NM5 = 32'd392; // G_freq
  parameter NM6 = 32'd440; // A_freq
  parameter NM7 = 32'd494; // B_freq
  parameter NM0 = 32'd10000; // silence
  always @(*)
  begin
    case (noteFreq)
      5'd0: note = NM1;  // C4
      5'd1: note = NM2;
      5'd2: note = NM3;
      5'd3: note = NM4;
      5'd4: note = NM5;
      5'd5: note = NM6;
      5'd6: note = NM7;
      5'd7: note = NM1 << 1;  // C5
      5'd8: note = NM2 << 1;
      5'd9: note = NM3 << 1;
      5'd10: note = NM4 << 1;
      5'd11: note = NM5 << 1;
      5'd12: note = NM6 << 1;
      5'd13: note = NM7 << 1;
      5'd14: note = NM1 << 2;  // C6
      5'd15: note = NM2 << 2;
      5'd16: note = NM3 << 2;
      5'd17: note = NM4 << 2;
      5'd18: note = NM5 << 2;
      5'd19: note = NM6 << 2;
      5'd20: note = NM7 << 2;
      5'd21: note = NM1 << 3;  // C7
      5'd22: note = NM2 << 3;
      5'd23: note = NM3 << 3;
      5'd24: note = NM4 << 3;
      5'd25: note = NM5 << 3;
      5'd26: note = NM6 << 3;
      5'd27: note = NM7 << 3;
      5'd28: note = NM1 << 4;  // C8
      default: note = NM0;
    endcase
  end

endmodule