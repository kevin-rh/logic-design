module motor(
    input clk,
    input clk_100,
    input rst,
    input [3:0] state,
    output left_pwm,
    output right_pwm,
    output reg [2:0] mode
);

    reg [9:0] right_motor, next_right_motor;
    reg [9:0] left_motor, next_left_motor;
    
	wire is_left_max, is_right_max;
	
    reg active_scan;
    wire enable_scan;

    parameter 
    IDLE = 4'd0,
    ROAM = 4'd1,
    SPECIAL = 4'd2,
    WET = 4'd3,
    DRY = 4'd4,
    DARK = 4'd5,
    COLD = 4'd6,
    HOT = 4'd7;

	parameter
	SPIN_L = 3'b000,
	SPIN_R = 3'b001,
	MAINTAIN = 3'b010,
    BACK = 3'b011,
	TURN_L = 3'b100,
	TURN_R = 3'b101,
	FRONT = 3'b110,
	STOP = 3'b111;

    wire [8:0] point_to;
    reg [15:0] cnt, next_cnt;
    reg [2:0] next_mode;
    
    biggest scan_lightinput (
        .clk(clk),
        .reset(rst),
        .enable(enable_scan),
        .data(light),
        .key(cnt),
        .max_key(point_to)
    );

    motor_pwm m0(clk, rst, left_motor, left_pwm);
    motor_pwm m1(clk, rst, right_motor, right_pwm);
	

    always @(posedge clk) begin
        if(rst)begin
            left_motor  <= 10'd0;
            right_motor <= 10'd0;
        end else begin
            left_motor  <= next_left_motor;
            right_motor <= next_right_motor; 
        end
    end
    always@(posedge clk)begin
        if(rst)begin
            mode <= STOP;
            cnt <= 16'd0;
        end else begin
            if(clk_100) begin
                mode <= next_mode;
                cnt = next_cnt;
            end else begin
                mode <= next_mode;
                cnt = cnt;
            end
        end
    end
    // [TO-DO] take the right speed for different situation
    
    assign is_left_max  = (left_motor===10'd1023)? 1'b1:1'b0;
    assign is_right_max = (left_motor===10'd1023)? 1'b1:1'b0;
    assign enable_scan = active_scan && clk_100;

    always @(*) begin
        next_cnt = (cnt == 16'd0)? (cnt) : (cnt - 16'd1);
        next_mode = mode;
        active_scan = 1'b0;
        case(state)
            IDLE: begin
                next_mode = STOP;
                next_cnt = 16'd0;
            end
            ROAM: begin
                if(cnt>16'd25_00) begin
                    next_mode = STOP;
                end else if(cnt>16'd22_00) begin
                    next_mode = TURN_R;
                end else if(cnt>16'd20_00) begin
                    next_mode = BACK;
                end else if(cnt>16'd18_00) begin
                    next_mode = SPIN_R;
                end else if(cnt>16'd15_00) begin
                    next_mode = TURN_R;
                end else if(cnt>16'd14_00) begin
                    next_mode = STOP;
                end else if(cnt>16'd10_00) begin
                    next_mode = TURN_L;
                end else if(cnt>16'd9_00) begin
                    next_mode = STOP;
                end else if(cnt>16'd5_00) begin
                    next_mode = FRONT;
                end else if(cnt!=16'd0)begin
                    next_mode = STOP;
                end else begin
                    next_cnt = 16'd30_00;
                end
            end
            SPECIAL: begin
                if(cnt>16'd11_00)begin              // Initial turn left for 2 second from position
                    next_mode = SPIN_L; 
                end else if(cnt>16'd7_00)begin
                    active_scan = 1'b1;
                    next_mode = SPIN_R;             // Read area from the 10cnt degree to right until 5 cnt (7.5 being in the middle)
                end else if(cnt>16'd5_00)begin
                    next_mode = SPIN_L;
                end else if (cnt>16'd4_98)begin     // Biggest data win therefore turn the other side for small amount of time (1second)
                    next_mode = (point_to>16'd7_50)? SPIN_L : SPIN_R;
                end else if (cnt>16'd3_00)begin     
                    next_mode = mode;
                end else if(cnt!=16'd0)begin
                    next_mode = FRONT;
                end else begin
                    next_cnt = 16'd13_00;
                end
            end
            WET: begin
                if(cnt>16'd20_00)begin
                    next_mode = STOP;
                end else if(cnt>16'd10_00) begin
                    next_mode = BACK;
                end else if(cnt>16'd8_00) begin
                    next_mode = STOP;
                end else if(cnt>16'd4_00)begin
                    next_mode = SPIN_L;
                end else if(cnt>16'd2_00)begin
                    next_mode = SPIN_R;
                end else if(cnt!=16'd0)begin
                    next_mode = SPIN_L;
                end else begin
                    next_cnt = 16'd22_00;
                end
            end
            DRY: begin
                if(cnt>16'd16_00)begin
                    next_mode = SPIN_R;
                end else if(cnt>16'd14_00)begin
                    next_mode = SPIN_L;
                end else if(cnt>16'd12_00)begin
                    next_mode = FRONT;
                end else if(cnt>16'd10_00) begin
                    next_mode = BACK;
                end else if(cnt>16'd8_00) begin
                    next_mode = FRONT;
                end else if(cnt>16'd4_00)begin
                    next_mode = BACK;
                end else if(cnt>16'd2_00)begin
                    next_mode = FRONT;
                end else if(cnt!=16'd0)begin
                    next_mode = BACK;
                end else begin
                    next_cnt = 16'd18_00;
                end
            end
            DARK: begin
                if(cnt>16'd25_00)begin
                    next_mode = STOP;
                end else if(cnt>16'd20_00) begin
                    next_mode = SPIN_L;
                end else if(cnt>16'd5_00) begin
                    next_mode = STOP;
                end else if(cnt!=16'd0)begin
                    next_mode = SPIN_R;
                end else begin
                    next_cnt = 16'd40_00;
                end
            end
            COLD: begin
                if(cnt>16'd16_00)begin
                    next_mode = SPIN_R;
                end else if(cnt>16'd14_00)begin
                    next_mode = SPIN_L;
                end else if(cnt>16'd12_00)begin
                    next_mode = SPIN_R;
                end else if(cnt>16'd10_00) begin
                    next_mode = SPIN_L;
                end else if(cnt>16'd8_00) begin
                    next_mode = SPIN_R;
                end else if(cnt>16'd4_00)begin
                    next_mode = SPIN_L;
                end else if(cnt>16'd2_00)begin
                    next_mode = SPIN_R;
                end else if(cnt!=16'd0)begin
                    next_mode = SPIN_L;
                end else begin
                    next_cnt = 16'd18_00;
                end
            end
            HOT: begin
                if(cnt>16'd30_00)begin
                    next_mode = FRONT;
                end else if(cnt>16'd25_00)begin
                    next_mode = BACK;
                end else if(cnt>16'd20_00) begin
                    next_mode = TURN_L;
                end else if(cnt>16'd10_00) begin
                    next_mode = SPIN_R;
                end else if(cnt!=16'd0)begin
                    next_mode = SPIN_L;
                end else begin
                    next_cnt = 16'd35_00;
                end
            end
            default: begin
                next_mode = STOP;
                next_cnt = 16'd0;
            end
        endcase
    end

    always @(*) begin
		case(mode)
		SPIN_L: begin
			next_left_motor  = 10'd1023;
			next_right_motor = 10'd1023;
		end
		SPIN_R: begin
			next_left_motor  = 10'd1023;
			next_right_motor = 10'd1023;
		end
		BACK: begin
            next_left_motor  = 10'd1023;
			next_right_motor = 10'd1023;
        end
        TURN_L: begin
            next_left_motor  = 10'd500;
			next_right_motor = 10'd1023;
        end
		TURN_R: begin
            next_left_motor  = 10'd1023;
			next_right_motor = 10'd500;
        end
		FRONT: begin
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
