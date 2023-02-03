`timescale 1ns/1ps

module FPGA_UI (
	input clk,
	input reset,
	input cancel,
	input add_50,
	input add_10,
	input add_5,
	inout wire PS2_DATA,
    inout wire PS2_CLK,
    input kb_rst,
	output [4-1:0] LED,
	output [7-1:0] seg,
	output [4-1:0] an
	);
	
	// Keyboard Press
	parameter KEY_A = 9'h1C;
	parameter KEY_S = 9'h1B;
	parameter KEY_D = 9'h23;
	parameter KEY_F = 9'h2B;
	wire [511:0] key_down;
	wire [8:0] last_change;
	wire been_ready;
	
	wire reset_p, cancel_p, add_50_p, add_10_p, add_5_p;
	
	wire [5-1:0] coins;
	reg [4-1:0] buy;
	
	wire en_clock_second, en_clock_update;
	
	Vending_Machine vending_machine(
		.clk(clk),
		.rst_n(~reset_p),
		.cancel(cancel_p),
		.add_50(add_50_p),
		.add_10(add_10_p),
		.add_5(add_5_p),
		.buy(buy),
		.avail(LED),
		.coins(coins)
	);
	
	// Four 7-Segment Decoder
	SevenSegmentDecoder sevseg_decoder(
		.clk(clk),
		.coins(coins),
		.an(an),
		.seg(seg)
	);
	
	// Debounced and Onepulsed Buttons
	debouncer_onepulse #(8) reset_button(
		.clk(clk),
		.in(reset),
		.out(reset_p)
	);
	debouncer_onepulse #(8) cancel_button(
		.clk(clk),
		.in(cancel),
		.out(cancel_p)
	);
	debouncer_onepulse #(8) add_50_button(
		.clk(clk),
		.in(add_50),
		.out(add_50_p)
	);
	debouncer_onepulse #(8) add_10_button(
		.clk(clk),
		.in(add_10),
		.out(add_10_p)
	);
	debouncer_onepulse #(8) add_5_button(
		.clk(clk),
		.in(add_5),
		.out(add_5_p)
	);
		
	// Keyboard Decoder
	KeyboardDecoder key_decoder (
        .key_down(key_down),
        .last_change(last_change),
        .key_valid(been_ready),
        .PS2_DATA(PS2_DATA),
        .PS2_CLK(PS2_CLK),
        .rst(kb_rst),
        .clk(clk)
    );

	// Generate Output from Keyboard
	always @(posedge clk) begin
		if (been_ready && key_down[last_change] == 1'b1) begin
			buy[3] <= (last_change == KEY_A) ? 1'b1 : 1'b0;
			buy[2] <= (last_change == KEY_S) ? 1'b1 : 1'b0;
			buy[1] <= (last_change == KEY_D) ? 1'b1 : 1'b0;
			buy[0] <= (last_change == KEY_F) ? 1'b1 : 1'b0;
		end
		else begin
			buy[3] <= 1'b0;
			buy[2] <= 1'b0;
			buy[1] <= 1'b0;
			buy[0] <= 1'b0;
		end
	end
	
endmodule

// Vending Machine Design
module Vending_Machine (
	input clk,
	input rst_n,
	input cancel,
	input add_50,
	input add_10,
	input add_5,
	input [4-1:0] buy,
	output [4-1:0] avail,
	output reg [5-1:0] coins // 1 coins = 5 dollars (max coins should be 20 coins [$100])
	);
	
	parameter
	INPUT 	= 1'b0,
	CHANGE = 1'b1;
	
	// Price each menu.
	parameter
	COFFEE 	= 5'd15, 	// 15 * $5 = $75
	COKE 	= 5'd10,	// 10 * $5 = $50
	OOLONG 	= 5'd6,		//  6 * $5 = $30
	WATER 	= 5'd5;		//  5 * $5 = $25
	
	parameter
	dollar100 = 5'd20,
	dollar50 = 5'd10,
	dollar10 = 5'd2,
	dollar5 = 5'd1,
	dollar0 = 5'd0;
	
	// Registers for state and coins.
	reg state;
	reg next_state;
	reg [5-1:0] next_coins;
	
	// Pre-calculated coin: coins + inputed coins
	reg [5-1:0] pre_coins; 

	// Users bought which items.
	wire [4-1:0] buying;
    
    // Internal clock per second.
    wire en_clock_second;
    wire reset, canceling;
    assign canceling = ((cancel === 1'b1)&&(coins !== dollar0));
    assign reset = (state === INPUT) && (canceling||(|buying));
    clk_div #(32'd100_000_000)clock_divider_second(
	    .clk(clk),
	    .reset (reset),
	    .clk_out(en_clock_second)
	);
	// Update each second the state and coins.
	always @(posedge clk) begin
		if(rst_n === 1'b0) begin
			state <= INPUT;
			coins <= dollar0;
		end else begin
		    if(state === CHANGE) begin
                if(en_clock_second) begin
                    state <= next_state;
                    coins <= next_coins;
                end else begin
                    state <= state;
                    coins <= coins;
                end
			end else begin
			    state <= next_state;
			    coins <= next_coins;
			end
		end
	end
	
	// Assign the availability which drink is affordable.
	assign avail[3] = (coins < COFFEE)	? 1'b0 : 1'b1;
	assign avail[2] = (coins < COKE)	? 1'b0 : 1'b1;
	assign avail[1] = (coins < OOLONG)	? 1'b0 : 1'b1;
	assign avail[0] = (coins < WATER)	? 1'b0 : 1'b1;
	
	// Assign which drink will be bought by the user.
	assign buying[3] = (buy[3])&(avail[3]);
	assign buying[2] = (buy[2])&(avail[2]);
	assign buying[1] = (buy[1])&(avail[1]);
	assign buying[0] = (buy[0])&(avail[0]);
	
	// Pre-calculated coins (max is 20+10 = 30 < 32 = 2**5 -> cannot overflow).
	always @(*) begin
		if(add_50) begin
			pre_coins = coins + dollar50;
		end else if (add_10) begin
			pre_coins = coins + dollar10;
		end else if (add_5) begin
			pre_coins = coins + dollar5;
		end else begin
			pre_coins = coins;
		end
	end
	
	// Calculate next_coins
	always @(*) begin
		case (state)
		INPUT: begin
            case(buying)
            4'b1000: begin				// Buy COFFEE.
                next_coins = coins - COFFEE;
            end
            4'b0100: begin	// Buy COKE.
                next_coins = coins - COKE;
            end
            4'b0010: begin	// Buy OOLONG.
                next_coins = coins - OOLONG;
            end
            4'b0001: begin	// Buy WATER.
                next_coins = coins - WATER;
            end
            default: begin
                if(pre_coins > dollar100) begin // More than $100 -> $100.
                    next_coins = dollar100;
                end else begin					// Input the pre-calculated coins.
                    next_coins = pre_coins;
                end
            end
            endcase
		end
		CHANGE: begin
			if(coins === dollar0) begin			// All money has been retrieved.
				next_coins = coins;
			end else begin
				next_coins = coins - dollar5; 	// Subtract coins by 1 ($5 dollars) each second.
			end
		end
		default: begin
			next_coins = coins;
		end
		endcase
	end
	
	// Calculate next_state
    always @(*) begin
		case (state)
		INPUT: begin
			if(canceling) begin
				if(coins === dollar0) begin	// Money is $0.
					next_state = state;
				end else begin
					next_state = CHANGE; 	// Money is not $0.
				end
			end else begin
				if(|buying) begin			// Buy COFFEE.
					next_state = CHANGE;
			    end else begin
			        next_state = state;
			    end
			end
		end
		CHANGE: begin
			if(coins === dollar0) begin			// All money has been retrieved.
				next_state = INPUT;	
			end else begin
				next_state = state;
			end
		end
		default: begin
			next_state = CHANGE;
		end
		endcase
	end
endmodule

// Seven Segment Decoder
module SevenSegmentDecoder(
	input clk,
	input [5-1:0] coins,
	input en,
	output reg [4-1:0] an,
	output reg [7-1:0] seg
	);
	
	reg [2-1:0] update;
	reg [4-1:0] BCD;
    
    // Internal Clock
    wire en_clock_update;
    clk_div #(32'd100_000)clock_divider_update (
	    .clk(clk), 
	    .clk_out(en_clock_update)
	);
	
	always @(posedge clk) begin
		if(en_clock_update) update <= update + 1;
		else update <= update;
	end
	
	// Anode
	always @(*) begin
        case(update)
            2'b10: an[3:0] = 4'b1011;
            2'b01: an[3:0] = 4'b1101;
            2'b00: an[3:0] = 4'b1110;
            default: an[3:0] = 4'b1111;
        endcase
    end
    
    // Number to BCD
	always @(*) begin
		case (update)
		2'b10: begin 	// 100's
			BCD = ((coins[5-1:0])==(5'b10100))? 4'd1 : 4'd10;
		end
		2'b01: begin 	// 10's: Implement priority decoder.
			case (coins[5-1:0])
			5'b10100: BCD = 4'd0;
			5'b10011, 5'b10010: BCD = 4'd9;
			5'b10001, 5'b10000: BCD = 4'd8;
			5'b01111, 5'b01110: BCD = 4'd7;
			5'b01101, 5'b01100: BCD = 4'd6;
			5'b01011, 5'b01010: BCD = 4'd5;
			5'b01001, 5'b01000: BCD = 4'd4;
			5'b00111, 5'b00110: BCD = 4'd3;
			5'b00101, 5'b00100: BCD = 4'd2;
			5'b00011, 5'b00010: BCD = 4'd1;
			default: BCD = 4'd10;
			endcase
		end
		2'b00: begin	// 1's: If the coins is odd, then its must be 5.
			BCD = (coins[0] === 1'b1)? 4'd5:4'd0;	
		end
		default: begin
		    BCD = 4'd11;
		end
		endcase
	end
	
	// 7-Segment NumberDecoder
	always @(*) begin
	   case(BCD)
	   4'd10: seg = 7'b1111111;
	   4'd9: seg = 7'b0000100;
	   4'd8: seg = 7'b0000000;
	   4'd7: seg = 7'b0001111;
	   4'd6: seg = 7'b0100000;
	   4'd5: seg = 7'b0100100;
	   4'd4: seg = 7'b1001100;
	   4'd3: seg = 7'b0000110;
	   4'd2: seg = 7'b0010010;
	   4'd1: seg = 7'b1001111;
	   4'd0: seg = 7'b0000001;
	   default: seg = 7'b0110000;
	   endcase
	end
	
	
endmodule

// Debouncer and OnePulse
module debouncer_onepulse #(parameter SIZE = 8)(
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
        if((delay===1'b0)& (debounced===1'b1)) out <= 1'b1;
        else out <= 1'b0;
        delay <= debounced;
	end
endmodule

// Clock Divider
// 10MHz clk to:
// 	- fast update clock
// 	- a second clock
module clk_div #(parameter MAX = 32'd100_000_000)(
	input clk,
	input reset,
	output clk_out
	);
	
	reg [32-1:0] cnt = 32'd0;
	assign clk_out = (cnt < MAX)? (1'b0) : (1'b1);
    
	always @(posedge clk) begin
		if(reset) begin
		    cnt <= 32'd0;
		end else begin
		    if(cnt < MAX) cnt <= cnt + 32'd1;
	        else          cnt <= 32'd0;
		end
	end
	
endmodule

module KeyboardDecoder(
	output reg [511:0] key_down,
	output wire [8:0] last_change,
	output reg key_valid,
	inout wire PS2_DATA,
	inout wire PS2_CLK,
	input wire rst,
	input wire clk
    );
    
    parameter [1:0] INIT			= 2'b00;
    parameter [1:0] WAIT_FOR_SIGNAL = 2'b01;
    parameter [1:0] GET_SIGNAL_DOWN = 2'b10;
    parameter [1:0] WAIT_RELEASE    = 2'b11;
    
	parameter [7:0] IS_INIT			= 8'hAA;
    parameter [7:0] IS_EXTEND		= 8'hE0;
    parameter [7:0] IS_BREAK		= 8'hF0;
    
    reg [9:0] key;		// key = {been_extend, been_break, key_in}
    reg [1:0] state;
    reg been_ready, been_extend, been_break;
    
    wire [7:0] key_in;
    wire is_extend;
    wire is_break;
    wire valid;
    wire err;
    
    wire [511:0] key_decode = 1 << last_change;
    assign last_change = {key[9], key[7:0]};
    
    KeyboardCtrl_0 inst (
		.key_in(key_in),
		.is_extend(is_extend),
		.is_break(is_break),
		.valid(valid),
		.err(err),
		.PS2_DATA(PS2_DATA),
		.PS2_CLK(PS2_CLK),
		.rst(rst),
		.clk(clk)
	);
	
	OnePulse op (
		.signal_single_pulse(pulse_been_ready),
		.signal(been_ready),
		.clock(clk)
	);
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		state <= INIT;
    		been_ready  <= 1'b0;
    		been_extend <= 1'b0;
    		been_break  <= 1'b0;
    		key <= 10'b0_0_0000_0000;
    	end else begin
    		state <= state;
			been_ready  <= been_ready;
			been_extend <= (is_extend) ? 1'b1 : been_extend;
			been_break  <= (is_break ) ? 1'b1 : been_break;
			key <= key;
    		case (state)
    			INIT : begin
    					if (key_in == IS_INIT) begin
    						state <= WAIT_FOR_SIGNAL;
    						been_ready  <= 1'b0;
							been_extend <= 1'b0;
							been_break  <= 1'b0;
							key <= 10'b0_0_0000_0000;
    					end else begin
    						state <= INIT;
    					end
    				end
    			WAIT_FOR_SIGNAL : begin
    					if (valid == 0) begin
    						state <= WAIT_FOR_SIGNAL;
    						been_ready <= 1'b0;
    					end else begin
    						state <= GET_SIGNAL_DOWN;
    					end
    				end
    			GET_SIGNAL_DOWN : begin
						state <= WAIT_RELEASE;
						key <= {been_extend, been_break, key_in};
						been_ready  <= 1'b1;
    				end
    			WAIT_RELEASE : begin
    					if (valid == 1) begin
    						state <= WAIT_RELEASE;
    					end else begin
    						state <= WAIT_FOR_SIGNAL;
    						been_extend <= 1'b0;
    						been_break  <= 1'b0;
    					end
    				end
    			default : begin
    					state <= INIT;
						been_ready  <= 1'b0;
						been_extend <= 1'b0;
						been_break  <= 1'b0;
						key <= 10'b0_0_0000_0000;
    				end
    		endcase
    	end
    end
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		key_valid <= 1'b0;
    		key_down <= 511'b0;
    	end else if (key_decode[last_change] && pulse_been_ready) begin
    		key_valid <= 1'b1;
    		if (key[8] == 0) begin
    			key_down <= key_down | key_decode;
    		end else begin
    			key_down <= key_down & (~key_decode);
    		end
    	end else begin
    		key_valid <= 1'b0;
			key_down <= key_down;
    	end
    end

endmodule

module OnePulse (
	output reg signal_single_pulse,
	input wire signal,
	input wire clock
	);

	reg signal_delay;

	always @(posedge clock) begin
		if (signal == 1'b1 & signal_delay == 1'b0)
		  signal_single_pulse <= 1'b1;
		else
		  signal_single_pulse <= 1'b0;
		signal_delay <= signal;
	end
endmodule