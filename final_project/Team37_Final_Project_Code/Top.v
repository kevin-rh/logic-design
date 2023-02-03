module Top(
    // 10-MHz clock.
    input clk,
    // buttons 
    input reset,
    input start,
    // input enter,
    // input menu,
    
    // switches.
    input [15:0] sw,

    // pmod's inputs.
    input [1:0] temp,
    input [1:0] light,
    input [1:0] soil,

    // pmod's outputs.
    // output [7:0] outA,

    // motor's output.
    output reg [1:0] left,
    output left_motor,
    output reg [1:0] right,
    output right_motor,
    
    // LEDs and 7-Segmented Displays's output.
    output [15:0] led,
    output [6:0] seg,
    output [3:0] an
);
    parameter
	SPIN_L = 3'b000,
	SPIN_R = 3'b001,
	MAINTAIN = 3'b010,
    BACK = 3'b011,
	TURN_L = 3'b100,
	TURN_R = 3'b101,
	FRONT = 3'b110,
	STOP = 3'b111;
	
    wire reset_pb, start_pb, enter_pb, menu_pb;
    debounce_onepulse #(.SIZE(8)) RESET_PB(
        .clk(clk),
        .in(reset),
        .out(reset_pb)
    );
    debounce_onepulse #(.SIZE(8)) START_PB(
        .clk(clk),
        .in(start),
        .out(start_pb)
    );
    debounce_onepulse #(.SIZE(8)) ENTER_PB(
        .clk(clk),
        .in(enter),
        .out(enter_pb)
    );
    debounce_onepulse #(.SIZE(8)) MENU_PB(
        .clk(clk),
        .in(menu),
        .out(menu_pb)
    );

    wire clk_1, clk_10, clk_100;
    clock_div #(.REFRESH(10000000)) clk_div(
        .clk(clk), 
        .reset(reset_pb), 
        .out(clk_1)
    );
    clock_div #(.REFRESH(1000000)) clk_div2(
        .clk(clk), 
        .reset(reset_pb), 
        .out(clk_10)
    );
    clock_div #(.REFRESH(100000)) clk_div3(
        .clk(clk), 
        .reset(reset_pb), 
        .out(clk_100)
    );

    wire blipped;
    blip_sensor BLIP_SENSR(
        .clk(clk),
        .clk_fast(clk_100),
        .clk_slow(clk_10),
        .reset(reset_pb),
        .light(sw[3:2]/*light*/),
        .blip(blipped)
    );
    wire [3:0] state;
    state_control STATE_CTRL(
        .clk(clk),
        .clk_sec(clk_1),
        .reset(reset_pb),
        .signal(blipped),
        .temp(/*sw[5:4]*/temp),
        .light(/*sw[3:2]*/light),
        .soil(/*sw[1:0]*/soil),
        .state(state)
    );
    wire [2:0] mode;
    motor MOTOR_CONTROL(
        .clk(clk),
        .clk_100(clk_100),  
        .rst(reset_pb),
        .state(state),
        .left_pwm(left_motor),
        .right_pwm(right_motor),
        .mode(mode)
    );
    always @(*) begin
        case(mode)
        SPIN_L: begin
            left = 2'b01;
            right = 2'b10;
        end
        SPIN_R: begin
            left = 2'b10;
            right = 2'b01;
        end
        BACK: begin
            left = 2'b10;
            right = 2'b10;
        end
        TURN_L: begin
            left = 2'b00;
            right = 2'b01;
        end
        TURN_R: begin
            left = 2'b01;
            right = 2'b00;
        end
        FRONT: begin
            left = 2'b01;
            right = 2'b01;
        end
        STOP: begin
            left = 2'b00;
            right = 2'b00;
        end
        default: begin
            left = 2'b00;
            right = 2'b00;
        end
        endcase
    end
    assign led[0]  = /*sw[0];*/soil[0];
    assign led[1]  = /*sw[1];*/soil[1];
    assign led[2]  = /*sw[2];*/light[0];
    assign led[3]  = /*sw[3];*/light[1];
    assign led[4]  = /*sw[4];*/temp[0];
    assign led[5]  = /*sw[5];*/temp[1];
    assign led[6]  = right[0];
    assign led[7]  = right[1];
    assign led[8]  = left[0];
    assign led[9]  = left[1];
    assign led[10] = clk_100;
    assign led[11] = state[0];
    assign led[12] = state[1];
    assign led[13] = state[2];
    assign led[14] = state[3];
    assign led[15] = blipped;

endmodule