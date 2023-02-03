`timescale 10ps/1ps

module Carry_Look_Ahead_Adder_8bit_t;
reg [7:0] a, b;
reg c0;
wire [7:0] s;
wire c8;

reg CLK = 1;

Carry_Look_Ahead_Adder_8bit cla8bit(
   .a(a), 
   .b(b), 
   .c0(c0), 
   .s(s), 
   .c8(c8)
);
/*
wire sum,prop,gen;

Full_Adder fa1bit
(.a(a[0]), .b(b[0]), .cin(c0), .sum(sum), .p(prop), .g(gen));
*/
/*
reg c_in;
reg [3:0] p, g;
wire [3:1] c_out;

Carry_Gen_4bit(
.c_in(c0),
.p(a[3:0]),
.g(b[3:0]),
.c_out(c_out)
);*/

always #1 CLK = ~CLK;
//ini juga very gak yakin soalnya yg di internet bikin reg clock???
initial begin
    {a, b} = 16'b0;
    c0 = 1'b0;
    repeat (2 ** 17) begin
        @ (posedge CLK)
        Test;
        @ (negedge CLK)
        {c0, a, b} = {c0, a, b} + 17'b1;
    end

    #1 $finish;
end

//ini maksudnya apa aaaaaaaa aku gak tau ketemu diinternet ehe 
task Test;
begin
  if (s !== (a + b + c0) & 32'b0000000011111111) begin
    $display("[ERROR] sum (s)");
    $write("a: %d\t", a);
    $write("b: %d\t", b);
    $write("c0: %d\n", c0);
    $write("s: %d\t", s);
    $write("should be: %b\n", {(a + b + c0) & 16'b0000000011111111});
    $display;
  end
  if (c8 !== &((a + b + c0) & 32'b000000100000000)) begin
    $display("[ERROR] cout (c8)");
    $write("a: %d\t", a);
    $write("b: %d\t", b);
    $write("c0: %d\n", c0);
    $write("c8: %d\t", c8);
    $write("should be: %b\n",&((a + b + c0) & 16'b000000100000000));
    $display;
  end
end
endtask

endmodule

//`timescale 1ns/1ps

//module Carry_Look_Ahead_Adder_8bit_t;
//reg [7:0] a, b;
//reg c0;
//wire [7:0] s;
//wire c8;

//reg CLK = 1;

//Carry_Look_Ahead_Adder_8bit nama(
//    .a(a), 
//    .b(b), 
//    .c0(c0), 
//    .s(s), 
//    .c8(c8)
//);

//always #1 CLK = ~CLK;

//initial begin
//    {a, b} = 16'b0;
//    c0 = 1'b0;
//    repeat (2 ** 17) begin
//        @ (posedge CLK)
//        Test;
//        @ (negedge CLK)
//        {c0, a, b} = {c0, a, b} + 16'b1;
//    end
//    #1 $finish;
//end

//task Test;
//begin
//  if (s !== (a + b + c0) & 10'b0011111111) begin
//    $display("[ERROR] sum (s)");
//    $write("a: %b\t", a);
//    $write("b: %b\t", b);
//    $write("c0: %b\n", c0);
//    $write("s: %b\t", s);
//    $write("Expected s: %b\n" , (a + b + c0) & 16'b0000000011111111 );

//    $display;
//  end
//  if (c8!==&( (a + b + c0) & 10'b0100000000)) begin
//    $display("[ERROR] cout (c8)");
//    $write("a: %b\t", a);
//    $write("b: %b\t", b);
//    $write("c0: %b\n", c0);
//    $write("c8: %b\t", c8);
//    $write("Expected c8: %b\n" , &((a + b + c0) & 9'b100000000));
//    $display;
//  end
//end
//endtask

//endmodule