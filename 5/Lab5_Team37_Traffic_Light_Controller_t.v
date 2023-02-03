`timescale 1ns/1ps

module Traffic_Light_Controller_t;
reg clk = 1'b0;
reg rst_n = 1'b0;
reg lr_has_car = 1'b0;
wire [2:0] hw_light;
wire [2:0] lr_light;

parameter cyc = 2;
always #(cyc/2) clk = ~clk;

integer i=0;

initial begin
    @(negedge clk) rst_n = 1'b0;
    $display("Testbench start!");
    @(negedge clk) rst_n = 1'b1;
    
    for(i=1;i<=100;i=i+1) @(negedge clk); //wait for 100 clock cycles
    
    @(negedge clk);
    $display("Car appeared from the road!");
    lr_has_car = 1'b1;
    // Appear for a second
    @(negedge clk);
    $display("Car dissappeared from the road!");
    lr_has_car = 1'b0;
    
    for(i=1;i<=1500;i=i+1) @(negedge clk); //wait for 150 clock cycles
    
    @(negedge clk);
    $display("Car appeared from the road!");
    lr_has_car = 1'b1;
    // Car appear and stay on the road
    
    for(i=1;i<=150;i=i+1) @(negedge clk); //wait for 150 clock cycles
    @(negedge clk);
    $display("Car dissappeared from the road!");
    lr_has_car = 1'b0;
    
    for(i=1;i<=150;i=i+1) @(negedge clk); //wait for 150 clock cycles
    @(negedge clk);
    $display("Car appeared from the road!");
    lr_has_car = 1'b1;
    
    for(i=1;i<=150;i=i+1) @(negedge clk); //wait for 150 clock cycles
    @(negedge clk);
    $display("Car dissappeared from the road!");
    lr_has_car = 1'b0;
    
    $finish;
end

Traffic_Light_Controller TLC(
    .clk(clk), 
    .rst_n(rst_n), 
    .lr_has_car(lr_has_car), 
    .hw_light(hw_light), 
    .lr_light(lr_light)
);

endmodule