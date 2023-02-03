`timescale 1ns/1ps

module Content_Addressable_Memory_t ;
    reg clk = 1'b0;
    reg wen = 1'b0;
    reg ren = 1'b0;
    reg [7:0] din = 8'd0;
    reg [3:0] addr = 4'd0;
    wire [3:0] dout;
    wire hit; 
    
    parameter cyc = 5;
    always #cyc clk=~clk;
    
    Content_Addressable_Memory CAM_test (clk, wen, ren, din, addr, dout, hit);

    initial begin
        @(posedge clk); 
        @(negedge clk); //START WRITING
        wen = 1'b1;
        addr = 4'd0;
        din = 8'd4;
        
        @(posedge clk);
        @(negedge clk);
        addr = 4'd7;
        din = 8'd8;

        @(posedge clk);
        @(negedge clk);
        addr = 4'd15;
        din = 8'd35;

        @(posedge clk);
        @(negedge clk);
        addr = 4'd9;
        din = 8'd8;

        @(posedge clk);
        @(negedge clk); // STOP WRITING
        wen = 1'b0;
        addr = 4'd0;
        din = 8'd0;
        
        @(posedge clk);
        @(negedge clk);
        

        @(posedge clk);
        @(negedge clk);
        

        @(posedge clk);
        @(negedge clk); // START READ
        ren = 1'b1;
        din = 8'd4;

        @(posedge clk);
        @(negedge clk);
        din = 8'd8;

        @(posedge clk);
        @(negedge clk);
        din = 8'd35;

        @(posedge clk);
        @(negedge clk);
        din = 8'd87;

        @(posedge clk);
        @(negedge clk);
        din = 8'd45;
        
        @(posedge clk);
        @(negedge clk); // STOP READING
        ren = 1'b0;
        din = 8'd0;
        
        @(posedge clk);
        @(negedge clk);
        
        @(posedge clk);
        @(negedge clk);
        
        @(posedge clk);
        @(negedge clk);
        
        @(posedge clk);
        @(negedge clk);
        $finish;
    end
endmodule