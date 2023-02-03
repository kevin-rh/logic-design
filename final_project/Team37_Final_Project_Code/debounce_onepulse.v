module debounce_onepulse #(
    parameter SIZE = 8
)(
	input clk,
	input in,
	output reg out
);
	
	reg [SIZE-1:0] DFF;
	reg delay;
	
	wire debounced;
	assign debounced = &DFF;
	
	always @(posedge clk) begin
        DFF[SIZE-1:1] <= DFF[SIZE-2:0];
        DFF[0] <= in;
    end
    
    always @(posedge clk) begin
        if((delay===1'b0) & (debounced===1'b1)) out <= 1'b1;
        else out <= 1'b0;
        delay <= debounced;
	end
endmodule