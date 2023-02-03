`timescale 1ns/1ps

module Greatest_Common_Divisor_t;
reg clk = 1'b0;
reg rst_n = 1'b0;
reg start = 1'b0;
reg [15:0] a = 16'd0;
reg [15:0] b = 16'd0;
wire done;
wire [15:0] gcd;

parameter cyc = 2;
always #(cyc/2) clk = ~clk;

Greatest_Common_Divisor GCD(
    .clk(clk), 
    .rst_n(rst_n), 
    .start(start), 
    .a(a), 
    .b(b), 
    .done(done), 
    .gcd(gcd)
);

initial begin
    @(negedge clk);
    @(negedge clk) rst_n = 1'b1;
    @(negedge clk);
    
    // Testcase 1: a is 0, immidiatelly output GCD
    @(negedge clk) start = 1'b1;
    a = 16'd0;
    b = 16'd10;
    @(negedge clk) start = 1'b0;
    a = 16'd1;
    b = 16'd1;
    repeat (5) @(negedge clk);
    
    // Testcase 2: wait b to 0, output GCD
    @(negedge clk) start = 1'b1;
    a = 16'd24;
    b = 16'd18;
    @(negedge clk) start = 1'b0;
    a = 16'd0;
    b = 16'd0;
    repeat (25) @(negedge clk);
    
    // Testcase 3: reset in the middle
    @(negedge clk) start = 1'b1;
    a = 16'd24;
    b = 16'd18;
    @(negedge clk) start = 1'b0;
    a = 16'd0;
    b = 16'd0;
    repeat (3) @(negedge clk);
    rst_n = 1'b0;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    repeat (10) @(negedge clk);
    
    
    $finish;
end

endmodule
