`timescale 1ns/1ps

module Multi_Bank_Memory_t;
	reg clk=1'b1;
	reg ren, wen;
	// Param b- w/r -adderfor bank
	reg [1:0] bwaddr;
	reg [1:0] braddr;
	// Param sb- w/r -adder for sub-bank
	reg [1:0] sbwaddr;
	reg [1:0] sbraddr;
	// Param w/r -adder for addresses 
	reg [6:0] waddr;
	reg [6:0] raddr;

	reg [7:0] din;
	wire [7:0] dout;

	parameter cycle = 10;
	/*
	Bank_Memory bank_test (
		.clk(clk), 
		.ren(ren), 
		.wen(wen), 
		.waddr({sbwaddr[1:0],waddr[6:0]}), 
		.raddr({sbraddr[1:0],raddr[6:0]}), 
		.din(din), 
		.dout(dout)
	);
	*/
	Multi_Bank_Memory multi_bank_test(
		.clk(clk), 
		.ren(ren), 
		.wen(wen), 
		.waddr({bwaddr[1:0],sbwaddr[1:0],waddr[6:0]}), 
		.raddr({braddr[1:0],sbraddr[1:0],raddr[6:0]}), 
		.din(din), 
		.dout(dout)
	);
	
/* 
	WRITE TEMPLATE
		din = 7'd ;
		wen = 1;
		waddr[6:0] = 7'd ;
		sbwaddr = 3-0;
	
	READ TEMPLATE
		ren = 1;
		raddr[6:0] = 7'd ;
		sbraddr = 3-0;
*/
	always #(cycle/2) clk=~clk;
	 
	initial begin
		bwaddr = 2'd0;
		braddr = 2'd0;
		sbwaddr = 2'd0;
		sbraddr = 2'd0;
		
		// Sub-Bank 0 scope
		sbwaddr=0;
		sbraddr=0;
		repeat (4) begin
			@(negedge clk)
			waddr[6:0] = 7'd87;
			raddr[6:0] = 7'd87;
			
			din = 120;
			ren = 1;
			wen = 0;
			// read [87]
			@(negedge clk)
			din = 130; 
			raddr[6:0] = 7'd17;
			ren = 1;
			wen = 1;
			// write [87] = 130
			// read [17]
			@(negedge clk)
			wen = 1;
			din = 140;
			waddr[6:0] = 7'd17;
			
			ren = 1;
			raddr[6:0] = 7'd87;
			// write [17] = 140
			// read [87] => 130
			
			@(negedge clk)
			raddr = 7'd17;
			din = 110;
			// write [17] = 110 cant 
			//read  [17] =>140
			@(negedge clk)
			wen = 0;
			//read [17] =>140
			
			@(negedge clk)
			sbraddr = sbraddr + 1;
			sbwaddr = sbwaddr + 1;
		end
		@(negedge clk)
		@(negedge clk)
		sbraddr = 2;
		sbwaddr = 2;
		@(negedge clk)
		@(negedge clk)
		@(negedge clk)
		sbraddr = 0;
		sbwaddr = 0;

		repeat (2**4) begin
			@(negedge clk)
			waddr[6:0] = 7'd15;
			raddr[6:0] = waddr[6:0];
			din = 8'd0;
			ren = 1'b0;
			wen = 1'b1;
			@(negedge clk)
			waddr[6:0] = 7'd127;
			raddr[6:0] = waddr[6:0];
			din = 8'd77;
			ren = 1'b1;
			wen = 1'b0;
			@(negedge clk)
			waddr[6:0] = 7'd127;
			raddr[6:0] = waddr[6:0];
			din = 8'd66;
			ren = 1'b0;
			wen = 1'b1;
			@(negedge clk)
			waddr[6:0] = 7'd127;
			raddr[6:0] = waddr[6:0];
			din = 8'd89;
			ren = 1'b1;
			wen = 1'b1;
			@(negedge clk)
			waddr[6:0] = 7'd127;
			raddr[6:0] = waddr[6:0];
			din = 8'd89;
			ren = 1'b0;
			wen = 1'b1;
			@(negedge clk)
			waddr[6:0] = 7'd15;
			raddr[6:0] = waddr[6:0];
			din = 8'd66;
			ren = 1'b1;
			wen = 1'b1;
			@(negedge clk)
			waddr[6:0] = 7'd127;
			raddr[6:0] = waddr[6:0];
			din = 8'd0;
			ren = 1'b1;
			wen = 1'b0;
		end
		$finish;
	end
endmodule