`timescale 1ns/1ps

module FPGA_UI (clk, rst_n, flip, max, min, enable, Anode, LED_out);
	input clk; 		// 100 Mhz clock
    input rst_n; 	// reset undebounced and un-one-pulsed
    input enable;
	input flip; 	// flip undebounced and un-one-pulsed
	input [3:0] max;
	input [3:0] min; 
	output reg [3:0] Anode; // anode signals of the 7-segment LED display
    output reg [6:0] LED_out;
	
	reg slow_clk;
    reg [3:0] LED_BCD;
    reg [16:0] refresh_counter;
    wire [1:0] LED_activating_counter; 
	reg [26:0] clk_per_sec;
	
	// Debounce and One-Pulse RESET, FLIP
	wire flip_db, reset_db, flip_pulse, reset_pulse;
	debounce debounce_flip (
		.pb_debounced(flip_db), 
		.pb(flip), 
		.clk(clk)
	);
	debounce debounce_reset (
		.pb_debounced(reset_db), 
		.pb(rst_n), 
		.clk(clk)
	);
	onepulse onepulse_flip (
		.PB_debounced(flip_db),
		.CLK(slow_clk), 
		.PB_one_pulse(flip_pulse)
	);
	
	onepulse onepulse_reset (
		.PB_debounced(reset_db),
		.CLK(slow_clk), 
		.PB_one_pulse(reset_pulse)
	);
	
	// Connect to Parameterized_Ping_Pong_Counter Module I/O
    wire direction;
	wire [3:0] out;
	Parameterized_Ping_Pong_Counter p_pp_c (
		.clk(slow_clk), 
		.rst_n(~reset_pulse), 
		.enable(enable), 
		.flip(flip_pulse), 
		.max(max), 
		.min(min), 
		.direction(direction), 
		.out(out)
	);
    
    // clock manipulation
	always @(posedge clk)begin 
		refresh_counter <= refresh_counter + 1;   
		
		if(clk_per_sec>=(99999999/2)) begin  // every half a second the clock is flipped
            clk_per_sec <= 0;
            slow_clk <= ~slow_clk;                
		end
		else begin
			clk_per_sec <= clk_per_sec + 1;
		end
    end 
    assign LED_activating_counter = refresh_counter[16:15]; // refresh LED every 1/2**17
    
    // LED update 4,3,2,1
    // 4    : MSB out (BSD representation)
    // 3    : LSB out (BSD representation)
    // 2,1  : Direction (up/down)
    always @(*)begin
        case(LED_activating_counter)
        2'b00: begin
            Anode = 4'b0111; 
            LED_BCD = out/10;
        end
        2'b01: begin
            Anode = 4'b1011; 
            LED_BCD = out%10;
        end
        2'b10: begin
            Anode = 4'b1101; 
            LED_BCD = {3'b111,direction};
        end
        2'b11: begin
            Anode = 4'b1110; 
            LED_BCD = {3'b111,direction};
        end
        endcase
    end
	
    // 7-segment LED display 
    // Handle each cathode to display certain number/sign
    // The MSB -> A cathode to the LSB -> G cathode
    always @(*)begin
        case(LED_BCD)
        4'b0000: LED_out = 7'b0000001; // "0"     
        4'b0001: LED_out = 7'b1001111; // "1" 
        4'b0010: LED_out = 7'b0010010; // "2" 
        4'b0011: LED_out = 7'b0000110; // "3" 
        4'b0100: LED_out = 7'b1001100; // "4" 
        4'b0101: LED_out = 7'b0100100; // "5" 
        4'b0110: LED_out = 7'b0100000; // "6" 
        4'b0111: LED_out = 7'b0001111; // "7" 
        4'b1000: LED_out = 7'b0000000; // "8"     
        4'b1001: LED_out = 7'b0000100; // "9"
		4'b1110: LED_out = 7'b1100011; // "down"
		4'b1111: LED_out = 7'b0011101; // "up"
        default: LED_out = 7'b1111111; // "dead"
        endcase
    end
		
 endmodule
 
// One-pulse circuit: Eliminate continuous signal
module onepulse (PB_debounced, CLK, PB_one_pulse);
	input PB_debounced;
	input CLK;
	output PB_one_pulse;
	
	reg PB_one_pulse;
	reg PB_debounced_delay;
	
	always @(negedge CLK) begin
		PB_one_pulse <= PB_debounced & (! PB_debounced_delay);
		PB_debounced_delay <= PB_debounced;
	end
	
endmodule

// Debounce: Eliminate any glitches
module debounce (pb_debounced, pb, clk);
	output pb_debounced;   // signal of a pushbutton after being debounced
	input pb;              // signal from a pushbutton 
	input clk; 

	reg [3:0] DFF;         // use shift_reg to filter pushbutton bounce 
	
	always @(posedge clk) begin
		DFF[3:1] <= DFF[2:0];
		DFF[0] <= pb;
	end
	
	assign pb_debounced = &DFF;

endmodule



module Parameterized_Ping_Pong_Counter (clk, rst_n, enable, flip, max, min, direction, out);
input clk, rst_n;
input enable;
input flip;
input [3:0] max;
input [3:0] min;
output direction;
output [3:0] out;

reg [3:0] counter;
reg dir;

//combination output
assign out = counter;
assign direction = dir;

//sequence memory clk
always @ (posedge clk) begin
    if(rst_n==1'b0)begin
        dir<=1'b1;
        counter<=min;
    end
    else begin
        if(enable && (max > min)) begin // operational (begin and max > min)
            if (counter == max) begin // touch max (flipped)
                counter <= max - 4'b0001;
                dir <= 1'b0;
            end
            else if (counter == min) begin // touch min (flipped)
                counter <= min + 4'b0001;
                dir <= 1'b1;
            end
            else if((min < counter )&&( counter < max)) begin // flip-able
                if(flip) begin // flipped
                    counter <= ((dir == 1'b0)? (counter + 1) : (counter - 1));
                    dir <= ~dir;
                end
                else begin // un-flipped
                    counter <= ((dir == 1'b1)? (counter + 1) : (counter - 1));
					dir <= dir;
				end
            end
            else begin // counter out of range
                counter <= counter;
                dir <= dir;
            end
        end
        // not operational (not begin or max < min)
        else begin
            dir <= dir;
            counter <= counter;
        end
    end
end

endmodule