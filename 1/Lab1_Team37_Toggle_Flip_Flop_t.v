`timescale 1ns/1ps

module Toggle_Flip_Flop_t;
reg clk = 1'b0;
reg t = 1'b0;
reg rst_n = 1'b0;

wire q;

always#(1) clk = ~clk;

Toggle_Flip_Flop m1(
	.clk(clk),
	.t(t),
	.rst_n(rst_n),
	.q(q)
);

initial begin
	rst_n = 1'b0;
	#10
	rst_n = 1'b1;
	#10
	t=1'b1;    
	#10
	rst_n = 1'b0;
	#10
	rst_n = 1'b1;
	repeat (10) begin
	#1 t=~t;
    end
    #1 $finish;
end
endmodule