`timescale 1ns/1ps
`include "..\basic\Lab3_Memory.v"

module Multi_Bank_Memory (clk, ren, wen, waddr, raddr, din, dout);
input clk;
input ren, wen;
input [10:0] waddr;
input [10:0] raddr;
input [7:0] din;
output reg [7:0] dout;

parameter M=4; // Bank

reg [M-1:0] rren;
reg [M-1:0] wwen;
wire [8-1:0] wout [M-1:0];

//decode write and read [10:9] which BANK
always @ (*) begin
	if(ren) rren = {4'b0001<<raddr[10:9]};
	else rren = 4'b0000;
	
	if(wen) wwen = {4'b0001<<waddr[10:9]};
	else wwen = 4'b0000;
end

//dout READ assignment which SUB-BANK
always @(*) begin
    if(ren) dout = wout[raddr[10:9]];
end

Bank_Memory bank0 (
    .clk(clk), 
    .ren(rren[0]), 
    .wen(wwen[0]), 
    .waddr(waddr[8:0]), 
    .raddr(raddr[8:0]), 
    .din(din), 
    .dout(wout[0])
);
Bank_Memory bank1 (
    .clk(clk), 
    .ren(rren[1]), 
    .wen(wwen[1]), 
    .waddr(waddr[8:0]), 
    .raddr(raddr[8:0]), 
    .din(din), 
    .dout(wout[1])
);
Bank_Memory bank2 (
    .clk(clk), 
    .ren(rren[2]), 
    .wen(wwen[2]), 
    .waddr(waddr[8:0]), 
    .raddr(raddr[8:0]), 
    .din(din), 
    .dout(wout[2])
);
Bank_Memory bank3 (
    .clk(clk), 
    .ren(rren[3]), 
    .wen(wwen[3]), 
    .waddr(waddr[8:0]), 
    .raddr(raddr[8:0]), 
    .din(din), 
    .dout(wout[3])
);
endmodule


module Bank_Memory (clk, ren, wen, waddr, raddr, din, dout);
input clk;
input ren, wen;
input [8:0] waddr;
input [8:0] raddr;
input [7:0] din;
output reg [7:0] dout;
 
parameter M=4; // Bank

reg [6:0] addr [M-1:0] ;
reg [M-1:0] rren;
reg [M-1:0] wwen;
wire [8-1:0] wout [M-1:0];

//decode write and read [8:7] which SUB-BANK
always @ (*) begin
	if(ren) rren = {4'b0001<<raddr[8:7]};
	else rren = 4'b0000;
	
	if(wen) wwen = {4'b0001<<waddr[8:7]};
	else wwen = 4'b0000;
end

//address assignment
always @ (*) begin
	if((rren[0]==1)) addr[0] = raddr[6:0];
	else if((wwen[0]==1)) addr[0] = waddr[6:0];
	
	if((rren[1]==1)) addr[1] = raddr[6:0];
	else if((wwen[1]==1)) addr[1] = waddr[6:0];
	
	if((rren[2]==1)) addr[2] = raddr[6:0];
	else if((wwen[2]==1)) addr[2] = waddr[6:0];
	
	if((rren[3]==1)) addr[3] = raddr[6:0];
	else if((wwen[3]==1)) addr[3] = waddr[6:0];
end

//dout READ assignment which SUB-BANK
always @(*) begin
    if(ren) dout = wout[raddr[8:7]];
end

Memory sub_bank0(
    .clk(clk), 
    .ren(rren[0]), 
    .wen(wwen[0]), 
    .addr(addr[0]), 
    .din(din), 
    .dout(wout[0])
);
Memory sub_bank1(
    .clk(clk), 
    .ren(rren[1]), 
    .wen(wwen[1]), 
    .addr(addr[1]), 
    .din(din), 
    .dout(wout[1])
);
Memory sub_bank2(
    .clk(clk), 
    .ren(rren[2]), 
    .wen(wwen[2]), 
    .addr(addr[2]), 
    .din(din), 
    .dout(wout[2])
);
Memory sub_bank3(
    .clk(clk), 
    .ren(rren[3]), 
    .wen(wwen[3]), 
    .addr(addr[3]), 
    .din(din), 
    .dout(wout[3])
);
endmodule

