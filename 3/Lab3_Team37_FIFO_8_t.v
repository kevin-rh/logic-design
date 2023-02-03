`timescale 1ns/1ps

module FIFO_8_t;
	reg clk=1'b0;
	reg rst_n=1'b0;
	reg wen=1'b0;
	reg ren=1'b0;
	reg [7:0] din=8'b0;

	wire [7:0] dout;
	wire error;

	parameter cycle=5;

	FIFO_8 fifo8(
		.clk(clk), 
		.rst_n(rst_n), 
		.wen(wen), 
		.ren(ren), 
		.din(din), 
		.dout(dout), 
		.error(error)
	);

	always #(cycle/2) clk=~clk;

	initial begin
		@(negedge clk)
		rst_n=1'b1;
		@(negedge clk)
		din = 8'd87;
		ren = 1'b0;
		wen = 1'b1;
		@(negedge clk)
		din = 8'd87;
		ren = 1'b1;
		wen = 1'b1;
		@(negedge clk)
		din = 8'd87;
		ren = 1'b1;
		wen = 1'b0;
		@(negedge clk)
		din = 8'd85;
		ren = 1'b1;
		wen = 1'b1;
		@(negedge clk)
		din = 8'd0;
		ren = 1'b0;
		wen = 1'b1;
		@(negedge clk)
		din = 8'd77;
		ren = 1'b1;
		wen = 1'b0;
		@(negedge clk)
		din = 8'd66;
		ren = 1'b0;
		wen = 1'b1;
		@(negedge clk)
		din = 8'd89;
		ren = 1'b1;
		wen = 1'b1;
		@(negedge clk)
		din = 8'd89;
		ren = 1'b0;
		wen = 1'b1;
		@(negedge clk)
		din = 8'd66;
		ren = 1'b1;
		wen = 1'b1;
		@(negedge clk)
		ren = 1'b1;
		wen = 1'b0;
		@(negedge clk)
		$finish;
	end
endmodule 