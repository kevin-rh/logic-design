`timescale 1ns/1ps

module Greatest_Common_Divisor (clk, rst_n, start, a, b, done, gcd);
input clk, rst_n;
input start;
input [15:0] a;
input [15:0] b;
output done;
output reg [15:0] gcd;

parameter 
WAIT = 2'b00,
CAL = 2'b01,
FINISH = 2'b10,
DISPLAY = 2'b11;

reg [15:0] next_gcd;

reg [1:0] state;
reg [1:0] next_state;

reg [15:0] in_a, in_b;
reg [15:0] next_in_a, next_in_b;

wire zeroB, biggerA;
assign zeroB = (in_b == 16'd0);
assign biggerA = (in_a > in_b);

assign done = (state === FINISH)||(state === DISPLAY);

always@(posedge clk)begin
    if(rst_n == 1'b0)begin
        state <= WAIT;
        gcd <= 16'd0;
        in_a <= 16'd0;
        in_b <= 16'd0;
    end else begin
        state <= next_state;
        gcd <= next_gcd;
        in_a <= next_in_a;
        in_b <= next_in_b;
    end
end

always@(*)begin
    next_state = state;
    next_gcd = 16'd0;
    next_in_a = in_a;
    next_in_b = in_b;
    case(state)
    WAIT:begin
        if(start)begin
            if(a==16'd0) begin
                next_state = FINISH;
                next_gcd = b;
                next_in_a = 16'd0;
                next_in_b = 16'd0;
            end else begin
                next_state = CAL;
                next_gcd = 16'd0;
                next_in_a = a;
                next_in_b = b;
            end
        end else begin
            next_state = WAIT;
            next_gcd = 16'd0;
            next_in_a = 16'd0;
            next_in_b = 16'd0;
        end
    end
    CAL:begin
        if(zeroB)begin
            next_state = FINISH;
            next_gcd = in_a;
            next_in_a = 16'd0;
            next_in_b = 16'd0;
        end
        else begin
            if(biggerA)begin
                next_state = state;
                next_gcd = 16'd0;
                next_in_a = in_a - in_b;
                next_in_b = in_b;
            end
            else begin
                next_state = state;
                next_gcd = 16'd0;
                next_in_a = in_a;
                next_in_b = in_b - in_a;
            end
        end
    end
    FINISH:begin
        next_state = DISPLAY;
        next_gcd = gcd;
        next_in_a = 16'd0;
        next_in_b = 16'd0;
    end
    DISPLAY:begin
        next_state = WAIT;
        next_gcd = 16'd0;
        next_in_a = 16'd0;
        next_in_b = 16'd0;
    end
    endcase
end

endmodule


/*
`timescale 1ns/1ps

module Greatest_Common_Divisor (clk, rst_n, start, a, b, done, gcd);
input clk, rst_n;
input start;
input [15:0] a;
input [15:0] b;
output reg done;
output reg [15:0] gcd;

parameter WAIT = 2'b00;
parameter CAL = 2'b01;
parameter FINISH = 2'b10;

reg [1:0] state;
reg [7:0] cycle_cnt;

reg [15:0] in_a, in_b;

always@(posedge clk)begin
    if(rst_n == 1'b0)begin
        in_a <= 1'b0;
        in_b <= 1'b0;
        state <= WAIT;
        gcd <= 1'b0;
        done <= 1'b0;
        cycle_cnt <= 1'b0;
    end
    else begin
        case(state)
        WAIT:begin
            if(start==1'b0)begin
                in_a <= a;
                in_b <= b;
                state <= CAL;
            end
            else begin
                in_a <= 1'b0;
                in_b <= 1'b0;
                state <= WAIT;
            end
            gcd <= 1'b0;
            done <= 1'b0;
            cycle_cnt <= 1'b0;
        end
        CAL:begin
            if(in_a == 16'b0)begin
                in_a <= in_a;
                in_b <= in_b;
                state <= FINISH;
                gcd <= in_b;
                done <= 1'b1;
            end
            else begin
                if(in_b!=16'b0)begin
                    if(in_a > in_b) begin
                        in_a <= in_a - in_b;
                        in_b <= in_b;
                    end
                    else begin
                        in_b <= in_b - in_a;
                        in_a <= in_a;
                    end
                    state <= CAL;
                    gcd <= 1'b0;
                    done <= 1'b0;
                end
                else begin
                    state <= FINISH;
                    gcd <= in_a;
                    done <= 1'b1;
                end
            end
            cycle_cnt <= 1'b0;
        end
        FINISH:begin
            if(cycle_cnt == 2'b10)begin
                in_a <= in_a;
                in_b <= in_b;
                state <= WAIT;
                gcd <= gcd;
                done <= done;
                cycle_cnt <= 1'b0;
            end
            else begin
                in_a <= in_a;
                in_b <= in_b;
                state <= state;
                gcd <= gcd;
                done <= done;
                cycle_cnt <= cycle_cnt + 1'b1;
            end
            
        end
        endcase
    end
end
endmodule

*/