`timescale 100ps/1ps

module Exhausted_Testing(a, b, cin, error, done);
output [3:0] a, b;
output cin;
output error;
output done;

// input signal to the test instance.
reg [3:0] a = 4'b0000;
reg [3:0] b = 4'b0000;
reg cin = 1'b0;

// output from the test instance.
wire [3:0] sum;
wire cout;

// instantiate the test instance.
Ripple_Carry_Adder rca(
    .a (a), 
    .b (b), 
    .cin (cin),
    .cout (cout),
    .sum (sum)
);       


initial begin
    {a,b,cin}=9'b0;
    error = 1'b0;
    done = 1'b0;
    repeat (2 ** 9) begin
        #1 
        Test;
        #4 
        {a,b,cin} = {a,b,cin} + 9'b1;
    end  
    #5 
    done = 1'b1;
    $finish;
end

//utility task for testing
    task Test;
    begin
        if({cout, sum}!==(a+b+cin)) begin
            error = 1'b1;
            $display("[ERROR]");
            $write("a:%d\t",a);
            $write("b:%d\t",b);
            $write("cin:%d\t",cin);
            $write("cout:%d\t",cout);
            $write("sum:%d\n",sum);
            $display;
        end
        else begin
            error = 1'b0;
        end
    end
    endtask

endmodule

