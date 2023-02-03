`timescale 1ns/1ps

module Traffic_Light_Controller (clk, rst_n, lr_has_car, hw_light, lr_light);
input clk, rst_n;
input lr_has_car;
output reg[2:0] hw_light;
output reg[2:0] lr_light;
//Green 100, Yellow 010, Red 001

parameter
Green = 3'b100,
Yellow = 3'b010,
Red = 3'b001;

parameter
hw_G = 3'd0,
hw_Y = 3'd1,
hw_R = 3'd2,
lr_G = 3'd3,
lr_Y = 3'd4,
lr_R = 3'd5;

reg [6:0] cyc = 7'd0;
reg [6:0] next_cyc;
reg [2:0] state = hw_G;
reg [2:0] next_state;

wire cyc80, cyc20;
assign cyc80 = (cyc >= 7'd79);
assign cyc20 = (cyc == 7'd19);

// Always block for update the States
always@(posedge clk) begin
    if(rst_n === 1'b0)begin
        state <= hw_G;
        cyc <= 7'd0;
    end
    else begin
        state <= next_state;
        cyc <= next_cyc;
    end
end

// Always block for the Light Ouput
always@(*) begin
    case(state)
    hw_G: begin
        hw_light = Green;
        lr_light = Red;
    end
    hw_Y: begin
        hw_light = Yellow;
        lr_light = Red;
    end
    hw_R, lr_R: begin
        hw_light = Red;
        lr_light = Red;
    end
    lr_G: begin
        hw_light = Red;
        lr_light = Green;
    end
    lr_Y: begin
        hw_light = Red;
        lr_light = Yellow;
    end
    default : begin
        hw_light = 3'b000;
        lr_light = 3'b000;
    end
    endcase
end

// Always block for calculate the Next States
always@(*) begin
    next_state = state;
    next_cyc = cyc;
    case(state)
    hw_G: begin
        if(cyc80) begin
            if(lr_has_car) begin
                next_state = hw_Y;
                next_cyc = 7'd0;
            end else begin 
                next_state = state;
                next_cyc = cyc;
            end
        end else begin
            next_state = state;
            next_cyc = cyc + 7'd1;
        end
    end
    hw_Y: begin
        if(cyc20) begin
            next_state = hw_R;
            next_cyc = 7'd0;
        end else begin
            next_state = state;
            next_cyc = cyc + 7'd1;
        end
    end
    hw_R: begin
        next_state = lr_G;
        next_cyc = 7'd0;
    end
    lr_G: begin
        if(cyc80) begin
            if(!lr_has_car) begin
                next_state = lr_Y;
                next_cyc = 7'd0;
            end else begin 
                next_state = state;
                next_cyc = cyc;
            end
        end else begin
            next_state = state;
            next_cyc = cyc + 7'd1;
        end
    end
    lr_Y: begin
        if(cyc20) begin
            next_state = lr_R;
            next_cyc = 7'd0;
        end else begin
            next_state = state;
            next_cyc = cyc + 7'd1;
        end
    end
    lr_R: begin
        next_state = hw_G;
        next_cyc = 7'd0;
    end
    default: begin
        next_state = state;
        next_cyc = cyc;
    end
    endcase
end

endmodule

/*
module Traffic_Light_Controller (clk, rst_n, lr_has_car, hw_light, lr_light);
input clk, rst_n;
input lr_has_car;
output reg [2:0] hw_light;
output reg [2:0] lr_light;
//Green 100, Yellow 010, Red 001

reg [7:0] cycle_cnt;

parameter
Green = 3'b100,
Yellow = 3'b010,
Red = 3'b001,

hw_G = 3'd0,
hw_Y = 3'd1,
hw_R = 3'd2,
lr_G = 3'd3,
lr_Y = 3'd4,
lr_R = 3'd5;

reg [2:0] state, next_state;

always@(posedge clk) begin
    if(rst_n == 1'b0)begin
        state <= hw_G;
        cycle_cnt <= 1'b0;
    end
    else begin
        if((state == lr_R))begin
            state <= next_state;
            cycle_cnt <= 1'b0;
        end
        else if((state == lr_Y) && cycle_cnt == 8'd20)begin
            state <= next_state;
            cycle_cnt <= 1'b0;
        end
        else if((state == lr_G) && cycle_cnt == 8'd80)begin
            state <= next_state;
            cycle_cnt <= 1'b0;
        end
        else if ((state == hw_R))begin
            state <= next_state;
            cycle_cnt <= 1'b0;
        end
        else if ((state == hw_Y) && cycle_cnt == 8'd20)begin
            state <= next_state;
            cycle_cnt <= 1'b0;
        end
        else if ((state == hw_G) && (cycle_cnt == 8'd80) && (lr_has_car))begin
            state <= next_state;
            cycle_cnt <= 1'b0;
        end
        else if (cycle_cnt == 8'd80) begin
            cycle_cnt <= cycle_cnt;
        end
        else cycle_cnt <= cycle_cnt + 1'b1;
    end
end

always@(*) begin
    case(state)
    hw_G: begin
        hw_light = Green;
        lr_light = Red;
        next_state = hw_Y;
    end
    hw_Y: begin
        hw_light = Yellow;
        lr_light = Red;
        next_state = hw_R;
    end
    hw_R: begin
        hw_light = Red;
        lr_light = Red;
        next_state = lr_G;
    end
    lr_G: begin
        hw_light = Red;
        lr_light = Green;
        next_state = lr_Y;
    end
    lr_Y: begin
        hw_light = Red;
        lr_light = Yellow;
        next_state = lr_R;
    end
    lr_R: begin
        hw_light = Red;
        lr_light = Red;
        next_state = hw_G;
    end
    endcase
end

endmodule
*/
