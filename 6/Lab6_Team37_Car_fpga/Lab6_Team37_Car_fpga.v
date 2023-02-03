`timescale 1ns/1ps
`define MOTOR_STOP 2'b00
`define MOTOR_FORWARD 2'b01   // [1]: backward, [0]: forward
`define MOTOR_BACKWARD 2'b10

module Top(
  input clk,
  input rst,
  input echo,
  input left_signal,
  input right_signal,
  input mid_signal,
  output trig,
  output left_speed,
  output reg [1:0] left,
  output right_speed,
  output reg [1:0] right,
  // debug
  output LED_rst,
  output [1:0] LED_left,
  output [1:0] LED_right,
  output LED_left_speed,
  output LED_right_speed,
  output LED_stop,
  output LED_left_signal,
  output LED_mid_signal,
  output LED_right_signal,
  output [6:0] seg,
  output [3:0] an
);

  wire [19:0] sonic_dis;
  wire rst_op, rst_pb;
  wire stop;
  wire [2:0] sensor_signals;

  debounce d0(rst_pb, rst, clk);
  onepulse d1(rst_pb, clk, rst_op);

  motor A(
    .clk(clk),
    .rst(rst_op),
    .mode(3'b01),  // go straight
    .pwm({left_speed, right_speed})
  );

  sonic_top B(
    .clk(clk),
    .rst(rst_op),
    .Echo(echo),
    .Trig(trig),
    .stop(stop),
    .dis(sonic_dis)
  );

  tracker_sensor C(
    .clk(clk),
    .reset(rst_op),
    .left_signal(left_signal),
    .right_signal(right_signal),
    .mid_signal(mid_signal),
    .state(sensor_signals)
   );

  // mode state
  // FIXME: make sure it's this same as mode in 'motor.v'
  parameter MODE_SIZE = 3;
  parameter STOP = 3'd0;
  parameter GO_FORWARD = 3'd1;
  parameter TURN_LEFT = 3'd2;
  parameter TURN_RIGHT = 3'd3;
  parameter GO_BACKWARD = 3'd7;

  reg [MODE_SIZE-1:0] mode;
  reg [MODE_SIZE-1:0] next_mode;

  always @(*) begin
    if (rst_op == 1'b1)
      {left, right} = {`MOTOR_FORWARD, `MOTOR_FORWARD};
    else
      if (stop == 1'b1)
        {left, right} = {`MOTOR_STOP, `MOTOR_STOP};
      else
        if (mode == GO_BACKWARD)
          {left, right} = {`MOTOR_BACKWARD, `MOTOR_BACKWARD};
        else if (mode == TURN_LEFT)
          {left, right} = {`MOTOR_STOP, `MOTOR_FORWARD};
        else if (mode == TURN_RIGHT)
          {left, right} = {`MOTOR_FORWARD, `MOTOR_STOP};
        else
          {left, right} = {`MOTOR_FORWARD, `MOTOR_FORWARD};
  end

  always @(posedge clk) begin
    if (rst)
      mode <= STOP;
    else
      mode <= next_mode;
  end

  always @(*) begin
    case (sensor_signals)
      3'b111:
        next_mode = GO_FORWARD;
      3'b110,
      3'b100:
        next_mode = TURN_LEFT;
      3'b011,
      3'b001:
        next_mode = TURN_RIGHT;
      3'b000:
        next_mode = GO_BACKWARD;
      default:
        next_mode = GO_BACKWARD;
    endcase
  end

  // debug
  assign LED_rst = rst;
  assign LED_left = left;
  assign LED_right = right;
  assign LED_left_speed = left_speed;
  assign LED_right_speed = right_speed;
  assign LED_stop = stop;
  assign LED_left_signal = left_signal;
  assign LED_mid_signal = mid_signal;
  assign LED_right_signal = right_signal;
  NumToSeg num2seg (.clk(clk), .num(sonic_dis), .seg(seg), .an(an));

endmodule

module debounce (pb_debounced, pb, clk);
  output pb_debounced;
  input pb;
  input clk;
  reg [4:0] DFF;

  always @(posedge clk) begin
    DFF[4:1] <= DFF[3:0];
    DFF[0] <= pb;
  end
  assign pb_debounced = (&(DFF));
endmodule

module onepulse (PB_debounced, clk, PB_one_pulse);
  input PB_debounced;
  input clk;
  output reg PB_one_pulse;
  reg PB_debounced_delay;

  always @(posedge clk) begin
    PB_one_pulse <= PB_debounced & (! PB_debounced_delay);
    PB_debounced_delay <= PB_debounced;
  end
endmodule

module NumToSeg (clk, num, seg, an);

parameter DIV = 32'd100_000;

input clk;
input [19:0] num;
output reg [6:0] seg; // cg~ca
output [3:0] an;

reg [3:0] digit;
reg [1:0] an_idx;
reg [32-1:0] cnt;
wire div_sig;

always @(posedge clk) begin
  if (cnt >= DIV - 32'b1)
    cnt <= 32'b0;
  else
    cnt <= cnt + 32'b1;
end
assign div_sig = (cnt == DIV - 32'b1) ? 1'b1: 1'b0;

always @(posedge clk) begin
  if (div_sig == 1'b1)
    an_idx <= an_idx + 2'b1;
  else
    an_idx <= an_idx;
end

assign an[3] = (an_idx == 2'd3) ? 1'b0 : 1'b1;
assign an[2] = (an_idx == 2'd2) ? 1'b0 : 1'b1;
assign an[1] = (an_idx == 2'd1) ? 1'b0 : 1'b1;
assign an[0] = (an_idx == 2'd0) ? 1'b0 : 1'b1;

always @(*) begin
  if (an_idx == 2'd0)
    digit = num % 20'd10;
  else if (an_idx == 2'd1)
    digit = num / 20'd10;
  else if (an_idx == 2'd2)
    digit = num / 20'd100;
  else if (an_idx == 2'd3)
    digit = num / 20'd1000;
  else
    digit = 4'd0;
end

always @(*) begin
  case (digit)
    4'd0: begin seg = 7'b1000000; end
    4'd1: begin seg = 7'b1111001; end
    4'd2: begin seg = 7'b0100100; end
    4'd3: begin seg = 7'b0110000; end
    4'd4: begin seg = 7'b0011001; end
    4'd5: begin seg = 7'b0010010; end
    4'd6: begin seg = 7'b0000010; end
    4'd7: begin seg = 7'b1111000; end
    4'd8: begin seg = 7'b0000000; end
    4'd9: begin seg = 7'b0010000; end
    // 4'ha: begin seg = 7'b0001000; end
    // 4'hb: begin seg = 7'b0000011; end
    // 4'hc: begin seg = 7'b1000110; end
    // 4'hd: begin seg = 7'b0100001; end
    // 4'he: begin seg = 7'b0000110; end
    // 4'hf: begin seg = 7'b0001110; end
    default: begin seg = 7'b1111111; end
  endcase
end

endmodule

// motor
module motor(
  input clk,
  input rst,
  input [2:0] mode,
  output [1:0] pwm  // {left, right}
);

  // mode
  parameter STOP = 3'd0;
  parameter GO_FORWARD = 3'd1;
  parameter TURN_LEFT = 3'd2;
  parameter TURN_RIGHT = 3'd3;
  parameter GO_BACKWARD = 3'd7;

  // duty
  parameter SPEED_STOP   = 10'd0;
  parameter SPEED_SLOW   = 10'd800;
  parameter SPEED_FAST   = 10'd1023;

  reg [9:0] next_left_duty, next_right_duty;
  reg [9:0] left_duty, right_duty;
  wire left_pwm, right_pwm;

  motor_pwm m0(
    .clk(clk),
    .reset(rst),
    .duty(left_duty),
    .pmod_1(left_pwm)
  );
  motor_pwm m1(
    .clk(clk),
    .reset(rst),
    .duty(right_duty),
    .pmod_1(right_pwm)
  );

  assign pwm = {left_pwm, right_pwm};

  always @(posedge clk) begin
    if (rst) begin
      left_duty <= 10'd0;
      right_duty <= 10'd0;
    end else begin
      left_duty <= next_left_duty;
      right_duty <= next_right_duty;
    end
  end

  // TODO: take the right speed for different situation
   always @(*) begin
      case (mode)
        STOP: begin
          next_left_duty = SPEED_STOP;
          next_right_duty = SPEED_STOP;
        end
        GO_FORWARD: begin
          next_left_duty = SPEED_FAST;
          next_right_duty = SPEED_FAST;
        end
        TURN_LEFT: begin
          next_left_duty = SPEED_STOP;
          next_right_duty = SPEED_FAST;
        end
        TURN_RIGHT: begin
          next_left_duty = SPEED_FAST;
          next_right_duty = SPEED_STOP;
        end
        GO_BACKWARD: begin
          next_left_duty = SPEED_SLOW;
          next_right_duty = SPEED_SLOW;
        end
        default: begin
         next_left_duty = SPEED_FAST;
         next_right_duty = SPEED_FAST;
        end
      endcase
   end

endmodule

module motor_pwm (
  input clk,
  input reset,
  input [9:0] duty,
  output pmod_1 //PWM
);

  PWM_gen pwm_0 (
    .clk(clk),
    .reset(reset),
    .freq(32'd25000),
    .duty(duty),
    .PWM(pmod_1)
  );

endmodule

//generate PWM by input frequency & duty
module PWM_gen (
  input wire clk,
  input wire reset,
  input [31:0] freq,
  input [9:0] duty,
  output reg PWM
);
  wire [31:0] count_max = 100_000_000 / freq;
  wire [31:0] count_duty = count_max * duty / 1024;
  reg [31:0] count;

  always @(posedge clk, posedge reset) begin
    if (reset) begin
      count <= 0;
      PWM <= 0;
    end else if (count < count_max) begin
      count <= count + 1;
      if(count < count_duty)
        PWM <= 1;
      else
        PWM <= 0;
    end else begin
      count <= 0;
      PWM <= 0;
    end
  end
endmodule

// sonic
module sonic_top(clk, rst, Echo, Trig, stop, dis);
  // [TO-DO] calculate the right distance to trig stop(triggered when the distance is lower than 40 cm)
  // Hint: using "dis"

	input clk, rst, Echo;
	output Trig, stop;
	output [19:0] dis;

	wire [19:0] d;
  wire clk1M;
	wire clk_2_17;

  div clk1(clk ,clk1M);
	TrigSignal u1(.clk(clk), .rst(rst), .trig(Trig));
	PosCounter u2(.clk(clk1M), .rst(rst), .echo(Echo), .distance_count(dis));

  assign stop = (dis < 20'd4000) ? 1'b1 : 1'b0;

endmodule

module PosCounter(clk, rst, echo, distance_count);
  input clk, rst, echo;
  output[19:0] distance_count;

  parameter S0 = 2'b00;
  parameter S1 = 2'b01;
  parameter S2 = 2'b10;

  wire start, finish;
  reg [1:0] curr_state, next_state;
  reg echo_reg1, echo_reg2;
  reg [19:0] count, distance_register;
  wire [19:0] distance_count;

  always@(posedge clk) begin
    if(rst) begin
      echo_reg1 <= 0;
      echo_reg2 <= 0;
      count <= 0;
      distance_register  <= 0;
      curr_state <= S0;
    end
    else begin
      echo_reg1 <= echo;
      echo_reg2 <= echo_reg1;
      case (curr_state)
        S0:begin
          if (start) curr_state <= next_state; //S1
          else count <= 0;
        end
        S1:begin
          if (finish) curr_state <= next_state; //S2
          else count <= count + 1;
        end
        S2:begin
          distance_register <= count;
          count <= 0;
          curr_state <= next_state; //S0
        end
      endcase
    end
  end

  always @(*) begin
    case (curr_state)
      // FIXME: TA use blocking assign in Combinational Circuit?
//       S0: next_state <= S1;
//       S1: next_state <= S2;
//       S2: next_state <= S0;
      S0: next_state = S1;
      S1: next_state = S2;
      S2: next_state = S0;
    endcase
  end

  /* distance_count (cm)
    c = 331.5 + 0.607 * t (m/s)
    If temperature = 20 (C degree):
      c = 0.034364 (cm/us)
          -> 29.1  (us/cm)
      distance_count = (traveled_time(us) / 2) * (c)  (cm)
                     = (traveled_time(us) / 2) / 29.1 (cm)
                     = (traveled_time(us)) * 100 / 58   (00.00cm)
  */
  assign distance_count = distance_register  * 100 / 58;
  assign start = echo_reg1 & ~echo_reg2;
  assign finish = ~echo_reg1 & echo_reg2;
endmodule

module TrigSignal(clk, rst, trig);
  input clk, rst;
  output trig;

  reg trig, next_trig;
  reg [23:0] count, next_count;

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      count <= 0;
      trig <= 0;
    end
    else begin
      count <= next_count;
      trig <= next_trig;
    end
  end

  always @(*) begin
    next_trig = trig;
    next_count = count + 1;
    if(count == 999)
      next_trig = 0;
    else if(count == 24'd9_999_999) begin
      next_trig = 1;
      next_count = 0;
    end
  end
endmodule

module div(clk, out_clk);
  input clk;
  output reg out_clk;
  // reg clkout;
  reg [6:0] cnt;

  always @(posedge clk) begin
    if(cnt < 7'd50) begin
      cnt <= cnt + 1'b1;
      out_clk <= 1'b1;
    end
    else if(cnt < 7'd100) begin
      cnt <= cnt + 1'b1;
      out_clk <= 1'b0;
    end
    else if(cnt == 7'd100) begin
      cnt <= 0;
      out_clk <= 1'b1;
    end
  end
endmodule

// tacker_sensor
module tracker_sensor(clk, reset, left_signal, right_signal, mid_signal, state);
  // [TO-DO] Receive three signals and make your own policy.
  // Hint: You can use output state to change your action.

  input clk;
  input reset;
  input left_signal, right_signal, mid_signal;
  output [2:0] state;

  assign state = {left_signal, mid_signal, right_signal};

endmodule