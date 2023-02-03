`timescale 1ns/1ps

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