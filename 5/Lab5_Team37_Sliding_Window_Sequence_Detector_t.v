`timescale 1ns/1ps

module Sliding_Window_Sequence_Detector_t;
reg clk = 1'b0;
reg rst_n = 1'b0;
reg in = 1'b0;

wire [3:0]state;
wire dec;


Sliding_Window_Sequence_Detector SWSD(
    .clk(clk), 
    .rst_n(rst_n), 
    .in(in), 
    .dec(dec)
);

always #1 clk = ~clk;
reg [15:0] num;

integer i = 0;
integer cnt = 0;

initial begin
    @(negedge clk) rst_n = 1'b0;
    $display("Testbench Start!"); 
    @(negedge clk) rst_n = 1'b1;
    
    $display("\nTestcase1:");
    cnt = 0;
    num = 16'b1100_1010_0100_0000;
    for(i=15 ; i>=0 ; i=i-1) begin
        @(negedge clk) in = num[i];
        @(posedge clk);
        if(dec==1'b1) begin
            cnt = cnt + 1;
            $display("Found a match!");
        end
    end
    @(negedge clk); 
    if(cnt != 1) begin
        $display("[ERROR]Should found 1 match."); 
        $finish;
    end
    
    $display("\nTestcase2:");
    cnt = 0;
    num = 16'b1110_0100_1000_0000;
    for(i=15 ; i>=0 ; i=i-1) begin
        @(negedge clk) in = num[i]; 
        @(posedge clk);
        if(dec==1'b1) begin
            cnt = cnt + 1;
            $display("Found a match!");
        end
    end
    @(negedge clk);
    if(cnt != 1) begin
        $display("[ERROR]Should found 1 match."); 
        $finish;
    end 
    
    $display("\nTestcase3:");
    cnt = 0;
    num = 16'b1100_0100_0000_0000;
    for(i=15 ; i>=0 ; i=i-1) begin
        @(negedge clk) in = num[i]; 
        @(posedge clk);
        if(dec==1'b1) begin
            cnt = cnt + 1;
            $display("Found a match!");
        end
    end
    @(negedge clk);
    if(cnt != 0) begin
        $display("[ERROR]Should found NO match."); 
        $finish;
    end 
    
    $display("\nTestcase4:");
    cnt = 0;
    num = 16'b1111_1111_0010_1001;
    for(i=15 ; i>=0 ; i=i-1) begin
        @(negedge clk) in = num[i]; 
        @(posedge clk);
        if(dec==1'b1) begin
            cnt = cnt + 1;
            $display("Found a match!");
        end
    end
    @(negedge clk);
    if(cnt != 1) begin
        $display("[ERROR]Should found 1 match."); 
        $finish;
    end 
    
    $display("\nTestcase5:");
    cnt = 0;
    num = 16'b1100_1001_1100_1001;
    for(i=15 ; i>=0 ; i=i-1) begin
        @(negedge clk) in = num[i]; 
        @(posedge clk);
        if(dec==1'b1) begin
            cnt = cnt + 1;
            $display("Found a match!");
        end
    end
    @(negedge clk);
    if(cnt != 2) begin
        $display("[ERROR]Should found 2 match."); 
        $finish;
    end 
    $display("\nTestbench Done!"); 
    $display("Design is correct!"); 
    @(negedge clk);
    $finish;
end

endmodule 