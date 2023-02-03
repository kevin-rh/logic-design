`timescale 1ns/1ps

module Built_In_Self_Test_t;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg scan_en = 1'b0;
    wire scan_in; 
    wire scan_out;
    
    parameter cyc = 5;
    
    always #(cyc/2) clk = ~clk;
    
    Built_In_Self_Test bist (
        .clk(clk), 
        .rst_n(rst_n), 
        .scan_in(scan_in), 
        .scan_en(scan_en), 
        .scan_out(scan_out)
    );
    integer i;
    initial begin
        repeat(2**4) begin
            i=101;
            @(posedge clk); 
            @(negedge clk); 
            rst_n = 1'b1;
            scan_en = 1'b1;
            $display("Input Start.\n\n");
            
            for(i=0;i<4;i=i+1) begin // scan pseudo random for B
                @(posedge clk);
                @(negedge clk);
            end
            
            for(i=0;i<4;i=i+1) begin // scan psuedo random for A
                @(posedge clk);
                @(negedge clk);
            end
            $display("Input Finish.\n\n");
    
            i=99;
            scan_en = 1'b0;
            $display("Captured!\n\n");
    
            @(posedge clk); // the output
            
            for(i=0;i<8;i=i+1) begin
                @(negedge clk);
                scan_en = 1'b1;
                @(posedge clk);
            end
        
        end
    end
    
endmodule