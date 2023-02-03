`timescale 1ps/1ps 

module Booth_Multiplier_4bit_t;
reg clk = 1'b0;
reg rst_n = 1'b0; 
reg start = 1'b0;
reg signed [3:0] a, b;
wire signed [7:0] p;

//wire [9:0] debug; // DEBUG purpose

parameter cyc = 2;
always #(cyc/2) clk = ~clk;
 
Booth_Multiplier_4bit BM4(
    .clk(clk), 
    .rst_n(rst_n), 
    .start(start), 
    .a(a), 
    .b(b), 
    .p(p)
    //,.debug(debug) // DEBUG purpose
);

reg signed [3:0] aa, bb;
wire signed [7:0] pp;

assign pp = aa*bb;

initial begin
    @(negedge clk) rst_n = 1'b0;
    @(negedge clk) rst_n = 1'b1;
    
    aa = -8;
    repeat ((2**4)-1) begin
        bb = -8;
        repeat (2**4) begin
            @(negedge clk) start = 1'b1;
            a = aa;
            b = bb;
            @(negedge clk) start = 1'b0;
            a = 0;
            b = 0;
            repeat (4) @(posedge clk);
            @(posedge clk);
            Test;
            bb = bb + 1;
        end
        aa = aa + 1;
    end
    $display ("Testbench Done.");
    $display ("Design is Correct!");
    $finish;
end

task Test;
begin
    if(p !== pp) begin
        $display("[ERROR]");
        $write("a      : %4b [%d]\n", aa, aa);
        $write("b      : %4b [%d]\n", bb, bb);
        $write("p      : %8b [%d]\n", p, p);
        $write("expect : %8b [%d]\n", pp, pp);
        $write ("Testbench Stopped.");
        $display;
        $finish;
    end 
end
endtask

endmodule