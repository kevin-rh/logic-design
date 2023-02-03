// PARAMETERIZED PING PONG TESTBENCH
module Parameterized_Ping_Pong_Counter_t;
	reg clk=1'b1, rst_n;
	reg enable;
	reg flip;
	reg [3:0] max;
	reg [3:0] min;
	wire direction;
	wire [3:0] out;

	parameter cycle = 5;

	always #(cycle/2) clk = ~clk;

	Parameterized_Ping_Pong_Counter p_pp_c(
		.clk(clk),
		.rst_n(rst_n),
		.enable(enable),
		.flip(flip),
		.max(max),
		.min(min),
		.direction(direction),
		.out(out)
	);


	initial begin
		@(negedge clk)
		flip=1'b0;
		rst_n=1'b0;
		enable=1'b1;
		min = 4'd0;
		max =4'd8;

		@(negedge clk)
		rst_n=1'b1;
		repeat (2**5) begin
			@(negedge clk);
		end

		enable=1'b0;
		min = 4'd0;
		max =4'd8;
		repeat (2**5) begin
			@(negedge clk);
		end

		enable=1'b1;
		min = 4'd9;
		max = 4'd8;
		repeat (2**5) begin
			@(negedge clk);
		end

		rst_n=1'b0;
		@(negedge clk);
		rst_n=1'b1;
		enable=1'b1;
		min = 4'd7;
		max = 4'd15;
		@(negedge clk)

		repeat (2**5) begin
			@(negedge clk);
		end

		min = 4'd0;
		max = 4'd7;
		repeat (2**5) begin
			@(negedge clk);
		end

		rst_n=1'b0;
		@(negedge clk);
		rst_n=1'b1;
		enable=1'b1;
		min = 4'd0;
		max = 4'd15;
		@(negedge clk)
		repeat (2**2) begin
			@(negedge clk);
		end
		flip=1'b1;
		@(negedge clk)
		flip=1'b0;
		repeat (2**2) begin
			@(negedge clk);
		end
		flip=1'b1;
		@(negedge clk)
		flip=1'b0;
		repeat (2**4) begin
			@(negedge clk);
		end
		flip=1'b1;
		@(negedge clk)
		flip=1'b0;
		repeat (2**2) begin
			@(negedge clk);
		end
		flip=1'b1;
		@(negedge clk)
		flip=1'b0;
		repeat (2**4) begin
			@(negedge clk);
		end
	
		$finish;
	end
endmodule