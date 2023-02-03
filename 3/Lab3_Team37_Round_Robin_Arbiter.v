`timescale 1ns/1ps
`include "Lab3_Team37_FIFO_8.v"

module Round_Robin_Arbiter(clk, rst_n, wen, a, b, c, d, dout, valid);
	input clk;
	input rst_n;
	input [3:0] wen;
	input [7:0] a, b, c, d;
	output [7:0] dout;
	output valid;
	
	// 4-states
	reg [1:0] state;
	
	
	// wire for FIFO's read input
	wire [3:0] ren;
	
	// wire for FIFO's output
	wire [3:0] fifo_error;		// error
	wire [8:0] fifo_out [3:0];	// dout
	
	
	assign ren = {4'b0001<<(state)};	// decode from Read-State to Read-Enable
	assign valid = (~&fifo_error);		// one of them error -> valid = 1'b0
	
	assign dout = { {8{valid}} & {fifo_out[state]} };		// output control either 0 or 8-bits data
	
	// the changes of Read-State  : a, b, c, d, a, b, c, ...
	always @(posedge clk) begin
		if(rst_n==1'b0) begin
			state <= 2'b00;
		end
		else begin
			state <= (state==2'b11) ?  2'b00 : state + 2'b01;
		end
	end
	
	FIFO_8 fifo_a(
		.clk(clk), 
		.rst_n(rst_n), 
		.wen(wen[0]), 
		.ren(ren[0]), 
		.din(a), 
		.dout(fifo_out[0]), 
		.error(fifo_error[0])
	);
	FIFO_8 fifo_b(
		.clk(clk), 
		.rst_n(rst_n), 
		.wen(wen[1]), 
		.ren(ren[1]), 
		.din(b), 
		.dout(fifo_out[1]), 
		.error(fifo_error[1])
	);
	FIFO_8 fifo_c(
		.clk(clk), 
		.rst_n(rst_n), 
		.wen(wen[2]), 
		.ren(ren[2]), 
		.din(c), 
		.dout(fifo_out[2]), 
		.error(fifo_error[2])
	);
	FIFO_8 fifo_d(
		.clk(clk), 
		.rst_n(rst_n), 
		.wen(wen[3]), 
		.ren(ren[3]), 
		.din(d), 
		.dout(fifo_out[3]), 
		.error(fifo_error[3])
	);
	
endmodule