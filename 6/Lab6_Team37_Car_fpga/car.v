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

module motor(
    input clk,
    input rst,
    input [2:0]mode,
    output left_pwm,
    output right_pwm
);

    reg [9:0]next_left_motor, next_right_motor;
    reg [9:0]left_motor, right_motor;
    
	wire is_left_max, is_right_max;
	
	parameter
	HARD_TURN_L = 3'b000,
	HARD_TURN_R = 3'b001,
	MAINTAIN = 3'b010,
	TURN_L = 3'b100,
	TURN_R = 3'b101,
	MIDDLE = 3'b110,
	STOP = 3'b111;

    motor_pwm m0(clk, rst, left_motor, left_pwm);
    motor_pwm m1(clk, rst, right_motor, right_pwm);
	
    always@(posedge clk)begin
        if(rst)begin
            left_motor  <= 10'd0;
            right_motor <= 10'd0;
        end else begin
            left_motor  <= next_left_motor;
            right_motor <= next_right_motor;
        end
    end

    // [TO-DO] take the right speed for different situation
    
    assign is_left_max  = (left_motor===10'd1023)? 1'b1:1'b0;
    assign is_right_max = (left_motor===10'd1023)? 1'b1:1'b0;
    
    always @(*) begin
		case(mode)
		HARD_TURN_L: begin
			next_left_motor  = 10'd0;
			next_right_motor = 10'd1023;
		end
		HARD_TURN_R: begin
			next_left_motor  = 10'd1023;
			next_right_motor = 10'd0;
		end
		MAINTAIN: begin
			if(is_left_max||is_right_max) begin 
				next_left_motor  = left_motor;
				next_right_motor = right_motor;
			end else begin
				next_left_motor  = left_motor + 10'd1;
				next_right_motor = right_motor + 10'd1;
			end
		end
		TURN_L: begin
			next_left_motor  = (is_right_max)? (left_motor - 10'd2) : (is_left_max)? (left_motor) : (left_motor + 1'b1);
			next_right_motor = (is_right_max)? (right_motor) : (right_motor + 10'd1);
		end
		TURN_R: begin
			next_left_motor  = (is_left_max)? (left_motor) : (left_motor + 10'd1);
			next_right_motor = (is_left_max)? (right_motor - 10'd2) : (is_right_max)? (right_motor) : (right_motor + 1'b1);
		end
		MIDDLE: begin
		    next_left_motor  = 10'd1023;
		    next_right_motor = 10'd1023;
		end
		STOP: begin
			next_left_motor  = 10'd0;
			next_right_motor = 10'd0;
		end
		default: begin
			next_left_motor  = 10'd0;
			next_right_motor = 10'd0;
		end
		endcase
	end
	
endmodule

module motor_pwm (
    input clk,
    input reset,
    input [9:0]duty,
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

//generte PWM by input frequency & duty
module PWM_gen (
    input wire clk,
    input wire reset,
	input [31:0] freq,
    input [9:0] duty,
    output reg PWM
);
    wire [31:0] count_max = 32'd100_000_000 / freq;
    wire [31:0] count_duty = count_max * duty / 32'd1024;
    reg [31:0] count;
        
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            count <= 32'b0;
            PWM <= 1'b0;
        end else if (count < count_max) begin
            count <= count + 32'd1;
            if(count < count_duty)
                PWM <= 1'b1;
            else
                PWM <= 1'b0;
        end else begin
            count <= 32'b0;
            PWM <= 1'b0;
        end
    end
endmodule

module sonic_top(clk, rst, Echo, Trig, stop);
	input clk, rst, Echo;
	output Trig, stop;

	wire[19:0] dis;
	wire[19:0] d;
    wire clk1M;
	wire clk_2_17;

    div clk1(clk ,clk1M);
	TrigSignal u1(.clk(clk), .rst(rst), .trig(Trig));
	PosCounter u2(.clk(clk1M), .rst(rst), .echo(Echo), .distance_count(dis));

    // [TO-DO] calculate the right distance to trig stop(triggered when the distance is lower than 40 cm)
    // Hint: using "dis"
	assign stop = (dis < 20'd4000)? 1'b1 : 1'b0;
	
endmodule

module PosCounter(clk, rst, echo, distance_count); 
    input clk, rst, echo;
    output[19:0] distance_count;

    parameter S0 = 2'b00;
    parameter S1 = 2'b01; 
    parameter S2 = 2'b10;
    
    wire start, finish;
    reg[1:0] curr_state, next_state;
    reg echo_reg1, echo_reg2;
    reg[19:0] count, next_count, distance_register, next_distance;
    wire[19:0] distance_count; 

    always@(posedge clk) begin
        if(rst) begin
            echo_reg1 <= 1'b0;
            echo_reg2 <= 1'b0;
            count <= 20'b0;
            distance_register <= 20'b0;
            curr_state <= S0;
        end
        else begin
            echo_reg1 <= echo;   
            echo_reg2 <= echo_reg1; 
            count <= next_count;
            distance_register <= next_distance;
            curr_state <= next_state;
        end
    end

    always @(*) begin
        case(curr_state)
            S0: begin
                next_distance = distance_register;
                if (start) begin
                    next_state = S1;
                    next_count = count;
                end else begin
                    next_state = curr_state;
                    next_count = 20'b0;
                end 
            end
            S1: begin
                next_distance = distance_register;
                if (finish) begin
                    next_state = S2;
                    next_count = count;
                end else begin
                    next_state = curr_state;
                    next_count = count + 1'b1;
                end 
            end
            S2: begin
                next_distance = count;
                next_count = 20'b0;
                next_state = S0;
            end
            default: begin
                next_distance = 20'b0;
                next_count = 20'b0;
                next_state = S0;
            end
        endcase
    end

    assign distance_count = distance_register * 20'd100 / 20'd58; 
    assign start = echo_reg1 & ~echo_reg2;  
    assign finish = ~echo_reg1 & echo_reg2; 
endmodule

module TrigSignal(clk, rst, trig);
    input clk, rst;
    output trig;

    reg trig, next_trig;
    reg[23:0] count, next_count;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            count <= 24'b0;
            trig <= 1'b0;
        end
        else begin
            count <= next_count;
            trig <= next_trig;
        end
    end

    always @(*) begin
        next_trig = trig;
        next_count = count + 1'b1;
        if(count == 24'd999)
            next_trig = 1'b0;
        else if(count == 24'd9_999_999) begin
            next_trig = 1'b1;
            next_count = 24'd0;
        end
    end
endmodule

module div(clk ,out_clk);
    input clk;
    output out_clk;
    reg out_clk;
    reg [6:0]cnt;
    
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
            cnt <= 7'b0;
            out_clk <= 1'b1;
        end
        else begin 
            cnt <= 7'b0;
            out_clk <= 1'b1;
        end
    end
endmodule

`timescale 1ns/1ps
module tracker_sensor(clk, reset, left_signal, right_signal, mid_signal, state);
    input clk;
    input reset;
    input left_signal, right_signal, mid_signal;
    output reg [2:0] state;
	reg [2:0] next_state;
    // [TO-DO] Receive three signals and make your own policy.
    // Hint: You can use output state to change your action.
	
	parameter
	OUT = 3'b000,
	MID = 3'b100,
	R1  = 3'b101,
	R2  = 3'b110,
	R3  = 3'b111,
	L1  = 3'b001,
	L2  = 3'b010,
	L3  = 3'b011;
		
	always @(posedge clk) begin
		if(reset) begin
			state <= OUT;
		end else begin
			state <= next_state;
		end
	end
	
	always @(*) begin
		next_state = OUT;
		if(mid_signal) begin
			if(right_signal) begin
				if(left_signal)begin
					next_state = MID;
				end else begin
					next_state = R1;
				end
			end else begin
				if(left_signal)begin
					next_state = L1;
				end else begin
					next_state = OUT;
				end
			end
		end else begin
			if(right_signal) begin
				if(left_signal)begin
					next_state = OUT;
				end else begin
					next_state = R2;
				end
			end else begin
				if(left_signal)begin
					next_state = L2;
				end else begin
					case(state)
					MID: begin
						next_state = OUT;
					end
					L1,L2,L3: begin
						next_state = L3;
					end
					R1,R2,R3: begin
						next_state = R3;
					end
					default: begin
						next_state = OUT;
					end
					endcase
				end
			end
		end
	end

endmodule

