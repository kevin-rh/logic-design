`timescale 10ps/1ps

module Multiplier_4bit_t;
    reg [3:0] a = 4'b0;
    reg [3:0] b = 4'b0;
    wire [7:0] p;
    
    reg cin= 1'b0;
    wire cout;
    wire [3:0] sum;
    
	reg clk= 1;
	
    Multiplier_4bit mul8bit(
        .a(a), 
        .b(b), 
        .p(p)
    );
    
    Full_Adder_4bit fa4bit (
        .a(a),
        .b(b), 
        .cin(cin),
        .cout(cout),
        .sum(sum)
    );


	always #1 clk=~clk;
	
    initial begin
	    {a, b} = 8'b00000000;
        repeat (2 ** 9) begin
            @(posedge clk) test;
			@(negedge clk) {cin,a,b} = {cin,a,b} + 8'b1;
        end
        #1 $finish;
    end
	
    reg [7:0] res;

    task test;
    begin
		res = a*b;
        if(p !== {(a * b) & 16'b0000000011111111}) begin
            $display("[ERROR]");
            $write("a: %d\t", a);
            $write("b: %d\n", b);
            $write("WRONG p    : %d\n", p);
            $write("SUPPOSED p : %d\n", res);
            $display;
        end
    end
    endtask

endmodule



