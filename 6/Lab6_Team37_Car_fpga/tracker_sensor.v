`timescale 1ns/1ps
module tracker_sensor(clk, reset, left_signal, right_signal, mid_signal, state);
    input clk;
    input reset;
    input left_signal, right_signal, mid_signal;
    output reg [2:0] state;
	reg [2:0] next_state;
    // [TO-DO] Receive three signals and make your own policy.
    // Hint: You can use output state to change your action.
	
	parameter
	OUT = 3'b000,
	MID = 3'b100,
	R1  = 3'b101,
	R2  = 3'b110,
	R3  = 3'b111,
	L1  = 3'b001,
	L2  = 3'b010,
	L3  = 3'b011;
		
	always @(posedge clk) begin
		if(reset) begin
			state <= OUT;
		end else begin
			state <= next_state;
		end
	end
	
	always @(*) begin
		next_state = OUT;
		if(mid_signal) begin
			if(right_signal) begin
				if(left_signal)begin
					next_state = MID;
				end else begin
					next_state = R1;
				end
			end else begin
				if(left_signal)begin
					next_state = L1;
				end else begin
					next_state = OUT;
				end
			end
		end else begin
			if(right_signal) begin
				if(left_signal)begin
					next_state = OUT;
				end else begin
					next_state = R2;
				end
			end else begin
				if(left_signal)begin
					next_state = L2;
				end else begin
					case(state)
					MID: begin
						next_state = OUT;
					end
					L1,L2,L3: begin
						next_state = L3;
					end
					R1,R2,R3: begin
						next_state = R3;
					end
					default: begin
						next_state = OUT;
					end
					endcase
				end
			end
		end
	end


endmodule
