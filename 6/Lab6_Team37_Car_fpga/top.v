module Top(
    input clk,
    input rst,
    input echo,
    input left_signal,
    input right_signal,
    input mid_signal,
    output trig,
    output left_motor,
    output reg [1:0]left,
    output right_motor,
    output reg [1:0]right,
    output [15:0] led
);
    
	// mode's param
	parameter
	HARD_TURN_L = 3'b000,
	HARD_TURN_R = 3'b001,
	MAINTAIN = 3'b010,
	TURN_L = 3'b100,
	TURN_R = 3'b101,
	MIDDLE = 3'b110,
	STOP = 3'b111;
	
	// state's param
	parameter
	OUT = 3'b000,	// Error
	MID = 3'b100,	// Straightforward
	R1 = 3'b101,	// state[2] == 1 -> RIGHT
	R2 = 3'b110,
	R3 = 3'b111,
	L1 = 3'b001,	// state[2] == 0 -> LEFT
	L2 = 3'b010,
	L3 = 3'b011;
	
    wire Rst_n, rst_pb, stop;
    debounce d0(rst_pb, rst, clk);
    onepulse d1(rst_pb, clk, Rst_n);
	
	wire [2:0] state;
	reg [2:0] mode, next_mode;
	reg [29:0] cnt, next_cnt;
	
    motor A(
        .clk(clk),
        .rst(Rst_n),
        .mode(mode),
        .left_pwm(left_motor),
        .right_pwm(right_motor)
    );

    sonic_top B(
        .clk(clk), 
        .rst(Rst_n), 
        .Echo(echo), 
        .Trig(trig),
        .stop(stop)
    );
    
    tracker_sensor C(
        .clk(clk), 
        .reset(Rst_n), 
        .left_signal(left_signal), 
        .right_signal(right_signal),
        .mid_signal(mid_signal), 
        .state(state)
       );
	
	always @(posedge clk) begin
        if(rst) begin
			mode <= STOP;
		end else begin
			mode <= next_mode;
		end
    end
	
    always @(*) begin
        // [TO-DO] Use left and right to set your pwm
        if(stop) {left, right} = {2'b00, 2'b00};
        else  {left, right} = {2'b01,2'b01};
    end
	
	always @(*) begin
		case (state)
		OUT: begin	// Error
			case(mode)
			HARD_TURN_R, TURN_R: begin
				next_mode = HARD_TURN_R;
			end
			HARD_TURN_L, TURN_L: begin
				next_mode = HARD_TURN_L;
			end
			default: begin
				next_mode = STOP;
			end
			endcase
		end
		MID: begin	// Straightforward
			next_mode = MIDDLE;
		end
		R1: begin	// state[2] == 1 -> RIGHT
			next_mode = TURN_R;
		end
		R2: begin
			next_mode = (mode === HARD_TURN_R)? (mode) : (TURN_R);
		end
		R3: begin
			next_mode = HARD_TURN_R;
		end
		L1: begin	// state[2] == 0 -> LEFT
			next_mode = TURN_L;
		end
		L2: begin
			next_mode = (mode === HARD_TURN_L)? (mode) : (TURN_L);
		end
		L3: begin
			next_mode = HARD_TURN_L;
		end
		default: begin
			next_mode = STOP;
		end
		endcase
	end
    
    assign led[0] = right_motor;
    assign led[1] = right[0];
    assign led[2] = right[1];
    assign led[3] = 1'b0;
    assign led[4] = left_motor;
    assign led[5] = left[0];
    assign led[6] = left[1];
    assign led[7] = 1'b0;
    assign led[8] = mode[0];
    assign led[9] = mode[1];
    assign led[10] = mode[2];
    assign led[11] = 1'b0;
    assign led[12] = stop;
    assign led[13] = state[0];
    assign led[14] = state[1];
    assign led[15] = state[2];
    
endmodule

module debounce (pb_debounced, pb, clk);
    output pb_debounced; 
    input pb;
    input clk;
    reg [8:0] DFF;
    
    always @(posedge clk) begin
        DFF[8:1] <= DFF[7:0];
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

