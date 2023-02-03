`timescale 1ns/1ps 

module Booth_Multiplier_4bit(
	input clk,
	input rst_n,
	input start,
	input signed [3:0] a,
	input signed [3:0] b,
	output reg signed [7:0] p
	//,output [9:0] debug // DEBUG purpose
	);
    
	parameter
	WAIT = 2'b00,
	CAL = 2'b01,
	FINISH = 2'b10;
    
    parameter m = 10;
    
	reg [8-1:0] next_p;
    
	reg [3-1:0] state;
	reg [3-1:0] next_state;
	
	reg [m-1:0] calc;
	reg [m-1:0] next_calc;
	
	reg [m-1:0] a_pos, a_neg;
	reg [m-1:0] next_a_pos, next_a_neg;
	
	reg [2-1:0] cyc;
	reg [2-1:0] next_cyc;
	
    //assign debug[m-1:0] = calc[m-1:0]; // DEBUG 
    
	always@(posedge clk)begin
		if(rst_n == 1'b0)begin
			state <= WAIT;
			p <= 8'd0;
			calc <= 8'd0;
			a_pos <= 10'd0;
			a_neg <= 10'd0;
			cyc <= 2'd0;
		end else begin
			state <= next_state;
			p <= next_p;
			calc <= next_calc;
			a_pos <= next_a_pos;
			a_neg <= next_a_neg;
			cyc <= next_cyc;
		end
	end
    
	wire [1:0] LSB_calc;

	wire [m-1:0] next_a_posneg, next_calc_unshifted, next_calc_shifted;
	assign next_a_posneg = (calc[1:0] == 2'b01) ? (a_pos) : (a_neg);
	assign next_calc_unshifted = (calc[1] == calc[0])? (calc) : (next_a_posneg + calc);
	assign next_calc_shifted = {next_calc_unshifted[m-1], next_calc_unshifted[m-1:1]};
    
	wire [4:0]a_5bits, neg_a_5bits;
	assign a_5bits = {a[3], a[3:0]};
	assign neg_a_5bits = (~(a_5bits)) + 5'b00001;
	
	always @(*)begin
		case(state)
		WAIT:begin
			if(start)begin 
				next_state = CAL;
				next_p = 8'd0;
				next_calc = {5'b0000, b, 1'b0};
				next_a_pos = {a_5bits, 5'b00000};
				next_a_neg = {neg_a_5bits, 5'b00000};
				next_cyc = 2'd0;
			end else begin
				next_state = state;
				next_p = 8'd0;
				next_calc = 10'd0;
				next_a_pos = 10'd0;
				next_a_neg = 10'd0;
				next_cyc = 2'd0;
			end
		end
		CAL:begin
			if(cyc === 2'd3)begin
				next_state = FINISH;
				next_p = next_calc_shifted[8:1];
				next_a_pos = 10'd0;
				next_a_neg = 10'd0;
				next_cyc = 2'd0;
				next_calc = 10'd0;
			end else begin
				next_state = state;
				next_p = 8'd0;
				next_calc = next_calc_shifted;
				next_a_pos = a_pos;
				next_a_neg = a_neg;
				next_cyc = cyc + 2'd1;
			end
		end
		FINISH:begin
			next_state = WAIT;
			next_p = 8'd0;
			next_calc = 10'd0;
			next_a_pos = 10'd0;
			next_a_neg = 10'd0;
			next_cyc = 2'd0;
		end
		default:begin
			next_state = state;
			next_p = p;
			next_calc = calc;
			next_a_pos = a_pos;
			next_a_neg = a_neg;
			next_cyc = cyc;
		end
		endcase
	end

endmodule


/*
`timescale 1ns/1ps 

module Booth_Multiplier_4bit(clk, rst_n, start, a, b, p);
input clk;
input rst_n; 
input start;
input signed [3:0] a, b;
output reg signed [7:0] p;

parameter
WAIT = 2'b00,
CAL = 2'b01,
FINISH = 2'b10;

reg [3:0] in_b;
reg [3:0] cycle_cnt;
reg [9:0] calc;
reg [2:0] state;
reg [8:0] add_neg, add_pos;
reg [3:0] negB;

always@(*)begin
    negB = ~in_b + 1'b1;
    add_neg = {negB + calc[8:5], calc[4:0]};
    add_pos = {in_b + calc[8:5], calc[4:0]};
end

always @(posedge clk)begin
    if(rst_n == 1'b0)begin
        state <= WAIT;
        cycle_cnt <= 1'b0;
    end
    else begin
        case(state)
        WAIT:begin
            if(start == 1'b1)begin 
                calc <= {4'b0000, a[3:0], 1'b0};
                in_b <= b;
                state <= CAL;
            end
            else begin
                calc <= 9'b0;
                in_b <= 1'b0;
                state <= WAIT;
            end
            cycle_cnt <= 1'b0;
            p <= 1'b0;
        end
        CAL:begin
            if(cycle_cnt == 4'd4)begin
                cycle_cnt <= 4'b0;
                state <= FINISH;
                p <= calc[8:1];
            end
            else begin
                //10 add b complement [0:3]
                if(calc[1:0] == 2'b10)begin 
                    calc <= {add_neg[8],add_neg[8:1]};
                end
                //01 add b to [3:0]
                else if(calc[1:0] == 2'b01)begin
                    calc <= {add_pos[8],add_pos[8:1]};
                end
                else begin
                    calc <= {calc[8],calc[8:1]};
                end
                //00 & 11
                cycle_cnt <= cycle_cnt + 1'b1;
                state <= CAL;
                p <= 1'b0;  
            end
        end
        FINISH:begin
            calc <= calc;
            in_b <= in_b;
            cycle_cnt <= 1'b0;
            state <= WAIT;
            p <= 1'b0;
        end
        endcase
    end
end



endmodule

*/