// 100,000,000 Hz = 10ns
// sense blip for total 3 second
//           ______          _______
//          /      \        /       \
//         |       |       |        |
// __...___|       \_______/        \___...__
`timescale 1ps/1ps

module blip_sensor #(
    parameter SENSITIVITY = 2'b01,          // LESS SENSITIVITY THEN WILL GOT 0 POINTS
    parameter HIGH = 11'b001_0000_0000,
    parameter LOW = 11'b000_0000_1000,
    parameter N = 2,
    // state's param
    parameter NO_LIGHT = 3'b000,
    parameter FIRST_LIGHT = 3'b001,
    parameter NOT_BLIP = 3'b010,
    parameter WAIT_SECOND = 3'b011,
    parameter SECOND_LIGHT = 3'b100,  
    parameter VALID_BLIP = 3'b101
)(
    input clk,
    input clk_fast,
    input clk_slow,
    input reset,
    input [N-1:0] light,
    output reg blip
);
    reg next_blip;
    reg [1023:0] light_mem;
    reg [2:0] state, next_state;

    reg [10:0] record_sum [7:0];

    wire [10:0] sum_out;
    wire [10:0] mean_sum_record;

    always @(posedge clk) begin
        if(reset) begin
            light_mem <= 1024'b0;
            state <= NO_LIGHT;
            blip <= 1'b0;
        end else begin
            if(clk_fast) begin
                record_sum[7] <= record_sum[6];
                record_sum[6] <= record_sum[5];
                record_sum[5] <= record_sum[4];
                record_sum[4] <= record_sum[3];
                record_sum[3] <= record_sum[2];
                record_sum[2] <= record_sum[1];
                record_sum[1] <= record_sum[0];
                record_sum[0] <= sum_out;
            end else begin
                record_sum[7] <= record_sum[7];
                record_sum[6] <= record_sum[6];
                record_sum[5] <= record_sum[5];
                record_sum[4] <= record_sum[4];
                record_sum[3] <= record_sum[3];
                record_sum[2] <= record_sum[2];
                record_sum[1] <= record_sum[1];
                record_sum[0] <= record_sum[0];
            end

            if(clk_slow) begin
                light_mem[0] <= (light>SENSITIVITY);
                light_mem[1023:1] <= light_mem[1022:0];
                state <= next_state;
                blip <= next_blip;
            end else begin
                light_mem [1023:0] <= light_mem [1023:0];    
                state <= state;
                blip <= blip;
            end
        end
    end
    
    sum1024 sumup(
        .in(light_mem), 
        .out(sum_out)   
    );


    assign mean_sum_record = (((record_sum[7] + record_sum[6])) + ((record_sum[5] + record_sum[4])) + ((record_sum[3] + record_sum[2]) + (record_sum[1] + record_sum[0])))>>3;
	
	always @(*) begin
        next_blip = 1'b0;
		case (state)
        NO_LIGHT: begin
            if(mean_sum_record > HIGH) begin
                next_state = FIRST_LIGHT;
            end else if(mean_sum_record > LOW) begin
                next_state = NO_LIGHT;
            end else begin
                next_state = NO_LIGHT;
            end
        end
        FIRST_LIGHT: begin
            if(mean_sum_record > HIGH) begin
                next_state = NOT_BLIP;
            end else if(mean_sum_record > LOW) begin
                next_state = NOT_BLIP;
            end else begin
                next_state = WAIT_SECOND;
            end
        end
        NOT_BLIP: begin
            if(mean_sum_record > HIGH) begin
                next_state = NOT_BLIP;
            end else if(mean_sum_record > LOW) begin
                next_state = NO_LIGHT;
            end else begin
                next_state = NO_LIGHT;
            end
        end
        WAIT_SECOND: begin
            if(mean_sum_record > HIGH) begin
                next_state = SECOND_LIGHT;
            end else if(mean_sum_record > LOW) begin
                next_state = NO_LIGHT;
            end else begin
                next_state = NO_LIGHT;
            end
        end
        SECOND_LIGHT: begin
            if(mean_sum_record > HIGH) begin
                next_state = NOT_BLIP;
            end else if(mean_sum_record > LOW) begin
                next_state = NOT_BLIP;
            end else begin
                next_state = VALID_BLIP;
                next_blip = 1'b1;
            end
        end  
        VALID_BLIP: begin
            next_blip = 1'b0;
            if(mean_sum_record > HIGH) begin
                next_state = FIRST_LIGHT;
            end else if(mean_sum_record > LOW) begin
                next_state = NO_LIGHT;
            end else begin
                next_state = NO_LIGHT;
            end
        end
        default: begin
            if(mean_sum_record > HIGH) begin
                next_state = NO_LIGHT; 
            end else if(mean_sum_record > LOW) begin
                next_state = NO_LIGHT;
            end else begin
                next_state = NO_LIGHT;
            end
        end
		endcase
	end
    
endmodule

