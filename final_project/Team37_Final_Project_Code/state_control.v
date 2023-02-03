module state_control #(
    parameter IDLE = 4'd0,
    parameter ROAM = 4'd1,
    parameter SPECIAL = 4'd2,
    parameter WET = 4'd3,
    parameter DRY = 4'd4,
    parameter DARK = 4'd5,
    parameter COLD = 4'd6,
    parameter HOT = 4'd7 //,
//    parameter = 4'd8,
//    parameter = 4'd9,
//    parameter = 4'd10,
//    parameter = 4'd11,
//    parameter = 4'd12,
//    parameter = 4'd13,
//    parameter = 4'd14,
//    parameter = 4'd15
    
)(
    input clk,
    input clk_sec,
    input reset,
    input signal,
    input [1:0] temp,
    input [1:0] light,
    input [1:0] soil,

    output reg [3:0] state
);
    reg [3:0] next_state;

    reg [15:0] cnt, next_cnt;
    reg [15:0] cooldown_water, next_cooldown_water;
    reg [15:0] cooldown_temp, next_cooldown_temp;
    reg [15:9] cooldown_light, next_cooldown_light;
    
    always @(posedge clk) begin
        if(reset) begin
            state <= IDLE;
            cnt <= 16'd0;
            cooldown_water <= 16'd0;
            cooldown_temp <= 16'd0;
            cooldown_light <= 16'd0;
        end else begin
            state <= next_state;
            if(clk_sec) begin // update per second for counter(s)
                cnt <= next_cnt;
                cooldown_water <= next_cooldown_water;
                cooldown_temp <= next_cooldown_temp;
                cooldown_light <= next_cooldown_light;
            end else begin
                cnt <= cnt;
                cooldown_water <= cooldown_water;
                cooldown_temp <= cooldown_temp;
                cooldown_light <= cooldown_light;
            end
        end
    end

    wire done_cnt, done_cooldown_water, done_cooldown_temp, done_cooldown_light;
    assign done_cnt = (cnt == 0);
    assign done_cooldown_water = (cooldown_water == 0);
    assign done_cooldown_temp = (cooldown_temp == 0);
    assign done_cooldown_light = (cooldown_light == 0);

    // temperature degree state step: 20, 25, 28, and 32 (celcius).
    //
    always @(*) begin
        next_state = state;
        next_cnt = (cnt == 0)? (cnt) : (cnt - 1);
        next_cooldown_water = (cooldown_water == 0)? (cooldown_water) : (cooldown_water - 1);
        next_cooldown_temp = (cooldown_temp == 0)? (cooldown_temp) : (cooldown_temp - 1);
        next_cooldown_light = (cooldown_light == 0)? (cooldown_light) : (cooldown_light - 1);

        case (state)
            IDLE: begin
                if(!signal) begin
                    if(done_cooldown_water) begin
                        if(soil == 2'b11) begin
                            next_state = WET;
                            next_cooldown_water = 16'd1800; 
                        end else if (soil == 2'b00) begin
                            next_state = DRY;
                            next_cooldown_water = 16'd30; 
                        end else begin
                            next_state = IDLE;
                            next_cooldown_water = 16'd600; 
                            next_cnt = cnt + 10'd5;
                        end
                    end else begin
                        if(done_cooldown_temp) begin 
                            if(temp == 2'b11) begin
                                next_state = HOT;
                                next_cooldown_temp = 16'd60; 
                            end else if (temp == 2'b00) begin
                                next_state = COLD;
                                next_cooldown_temp = 16'd60; 
                            end else begin
                                next_state = IDLE;
                                next_cooldown_temp = 16'd300; 
                                next_cnt = cnt + 10'd5;
                            end
                        end else begin
                            if(done_cooldown_light) begin
                                if (light <= 2'b01) begin
                                    next_state = DARK;
                                    next_cooldown_light = 16'd300; 
                                end else begin
                                    next_state = IDLE;
                                    next_cooldown_light = 16'd60; 
                                    next_cnt = cnt + 10'd5;
                                end
                            end else begin
                                if (cnt >= 10'd120) begin
                                    next_state = ROAM;
                                    next_cnt = 16'd60;
                                end else begin
                                    next_state = IDLE;
                                    next_cnt = cnt + 10'd1;
                                end
                            end
                        end
                    end
                end else begin
                    next_state = SPECIAL;
                    next_cnt = 16'd150;
                end
            end 
            SPECIAL: begin
                if(done_cnt)begin
                    next_state = IDLE;
                    next_state = IDLE;
                end else begin
                    next_state = SPECIAL;
                end
            end
            ROAM: begin
                if(done_cnt) begin
                    next_state = IDLE;
                end else begin
                    next_state = ROAM;
                end
            end
            WET: begin
                next_state = IDLE;
            end
            DRY: begin
                if(soil >= 2'b10) begin
                    next_state = IDLE;
                end else begin
                    next_state = DRY;
                end
            end
            DARK: begin
                next_state = IDLE;
            end
            COLD: begin
                if(temp != 2'b00) begin
                    next_state = IDLE;
                end else begin
                    next_state = COLD;
                end
            end
            HOT: begin
                if(temp != 2'b11) begin
                    next_state = IDLE;
                end else begin
                    next_state = HOT;
                end
            end
            default: begin
                next_state = IDLE;
                next_cnt = (cnt == 0)? (cnt) : (cnt - 1);
                next_cooldown_water = (cooldown_water == 0)? (cooldown_water) : (cooldown_water - 1);
                next_cooldown_temp = (cooldown_temp == 0)? (cooldown_temp) : (cooldown_temp - 1);
                next_cooldown_light = (cooldown_light == 0)? (cooldown_light) : (cooldown_light - 1);
            end 
        endcase
    end



endmodule