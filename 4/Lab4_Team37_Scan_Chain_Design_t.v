`timescale 1ns/1ps

module Scan_Chain_Design_t;
    reg clk = 1'b1;
    reg rst_n = 1'b0;
    reg scan_in = 1'b0;
    reg scan_en = 1'b0;
    wire scan_out;
    
    parameter cyc = 5;
    
    always #(cyc/2) clk=~clk;
    
    reg [3:0] a = 4'd10,
              b = 4'd10;
    integer i;
    reg [7:0] c;
    
    Scan_Chain_Design SCD_test (clk, rst_n, scan_in, scan_en, scan_out);

    initial begin
        c = a * b;
        $display("Input Number:\n");
        $write("- a     = [%04b] %d\n",a,a);
        $write("- b     = [%04b] %d\n",b,b);
        $write("- a * b = [$04b] %d\n",c,c);
        $display("Testbench Start.\n");
        @(posedge clk); 
        @(negedge clk); 
        rst_n = 1'b1;
        scan_en = 1'b1;
        
        $display("Input Start.\n\n");
        
        for(i=0;i<4;i=i+1) begin
            scan_in = b[i];
            @(posedge clk);
            @(negedge clk);
        end
        
        for(i=0;i<4;i=i+1) begin
            scan_in = a[i];
            @(posedge clk);
            @(negedge clk);
        end
        
        $display("Input Finish.\n\n");

        i=99;
        scan_en = 1'b0;
        $display("Captured!\n\n");

        @(posedge clk); // START PUTTING OUT RESULT
        
        for(i=0;i<8;i=i+1) begin
            Test;
            @(negedge clk);
            scan_en = 1'b1;
            @(posedge clk);
        end
        
        $display("Testbench Done.\n");
        $finish;
    end
    
    task Test;
        if(c[i] !== scan_out) begin
            $display("Incorrect at seq%d.\n",i);
            $display("Expected : %b\t",c[i]);
            $display("Output : %b\n",scan_out);
        end else begin
            $display("Correct at [%d]!!\n",i);
        end
    endtask
    
endmodule