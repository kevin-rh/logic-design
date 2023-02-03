`timescale 1ns/1ps

module Mealy_Sequence_Detector_t ();
    reg clk = 1'b0;
    reg rst_n = 1'b1;
    reg in = 1'b0;
    reg [3:0] seq = 4'b0000;
    reg [1:0] idx;
    wire dec;
    
    parameter CYC = 4;
    Mealy_Sequence_Detector q1(
      .clk(clk),
      .rst_n(rst_n),
      .in(in),
      .dec(dec)
    );
    
    always #(CYC/2) clk = ~clk;
    initial begin
      #CYC  rst_n = 1'b0;
      #CYC  rst_n = 1'b1;
    
      repeat (2 ** 4) begin
        idx = 2'b00;
        in = seq[idx];
        repeat (4) begin
          @ (posedge clk) begin
            if (idx == 2'b0)
              Test;
          end
          @ (negedge clk) begin
            idx = idx + 2'b1;
            in = seq[idx];
          end
        end
        seq = seq + 4'b1;
      end
    
      $finish;
    end
    
    task Test;
        begin
            if (seq == 4'b1011 || seq == 4'b0011 || seq == 4'b1010) begin
                if (dec != 1'b1) begin
                    $display("[ERROR], dec!=1 ");
                    $write("seq: %b\n", seq);
                    $write("dec: %d\n", dec);
                    $display;
                end
            end
            else begin
                if (dec != 1'b0) begin
                    $display("[ERROR], 'dec'!=0 ");
                    $write("seq: %b\n", seq);
                    $write("dec: %d\n", dec);
                    $display;
                end
            end
        end
    endtask

endmodule