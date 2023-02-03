`timescale 1ns/1ps

module Round_Robin_Arbiter_t;
	reg clk = 1'b0;
	reg rst_n;
	reg [3:0] wen;
	reg [7:0] a, b, c, d;
	wire [7:0] dout;
	wire valid;
	
	parameter cycle = 10;
	
	Round_Robin_Arbiter rra(
		.clk(clk), 
		.rst_n(rst_n), 
		.wen(wen), 
		.a(a), 
		.b(b), 
		.c(c), 
		.d(d), 
		.dout(dout), 
		.valid(valid)
	);
	
	always #(cycle/2) clk=~clk;
	
	initial begin
		rst_n = 1'b0;
		$display("Testbench Starts\n");
		
		@(negedge clk)
		rst_n = 1'b1;
		$display("Reset Done\n");

		wen=4'b 1111;
		a=8'd 87;
		b=8'd 56;
		c=8'd 9;
		d=8'd 13;
		
		@(negedge clk)
		wen=4'b 1000;
		d=8'd 13;
		
		@(negedge clk)
		wen=4'b 0100;
		d=8'd 85;
		
		@(negedge clk)
		wen=4'b 0000;
		c=8'd 139;
		
		@(negedge clk)
		wen=wen;
		
		@(negedge clk)
		wen=wen;
		
		@(negedge clk)
		wen=4'b 0001;
		a=8'd 51;
		
		@(negedge clk)
		wen=4'b 0000;
		
		@(negedge clk)
		wen=wen;
		
		@(negedge clk)
		wen=wen;
		
		$finish;
	end	

endmodule