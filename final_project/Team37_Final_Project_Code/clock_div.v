module clock_div #(
    parameter REFRESH = 100_000_000	// system clock in Hz
)(
	input clk, 
	input reset, 
	output reg out
);
   	reg [26:0] clk_per_sec;
	always @(posedge clk)begin 
		if(reset) begin
			clk_per_sec <= 0;
            out <= 0;
		end else begin
			if(clk_per_sec > (REFRESH)) begin  // every half a second the clock is flipped
				clk_per_sec <= 0;
                out <= 1;
			end
			else begin
				clk_per_sec <= clk_per_sec + 1;
                out <= 0;
			end
		end
    end 

endmodule