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
