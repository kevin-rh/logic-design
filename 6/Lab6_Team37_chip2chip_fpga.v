////////////////////////////////////////////////////////////////////////////////////////////////////////////
//===================MASTER===========================================================================

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25 12:47:53
// Module Name: top
// Project Name: Chip2Chip
// Additional Comments: top module for master, pass signals and perform debounce onepulse
//
//////////////////////////////////////////////////////////////////////////////////
module debounce (pb_debounced, pb, clk);
	output pb_debounced; // signal of a pushbutton after being debounced
	input pb; // signal from a pushbutton
	input clk;

	reg [3:0] DFF;
	always @(posedge clk)begin
		DFF[3:1] <= DFF[2:0];
		DFF[0] <= pb;
	end
	assign pb_debounced = ((DFF == 4'b1111) ? 1'b1 : 1'b0);
endmodule

module onepulse (pb_debounced, clock, pb_one_pulse);
	input pb_debounced;
	input clock;
	output reg pb_one_pulse;
	reg pb_debounced_delay;
	always @(posedge clock) begin
		pb_one_pulse <= pb_debounced & (! pb_debounced_delay);
		pb_debounced_delay <= pb_debounced;
	end
endmodule

module top(clk, rst_n, in, request, notice_master, data_to_slave_o, valid, request2s, ack, seven_seg, AN);
    input clk;
    input rst_n;
    input [8-1:0] in;
    input request;
    input ack;
    output [3-1:0] data_to_slave_o;
    output notice_master;
    output valid;
    output request2s;
    output [7-1:0] seven_seg;
    output [4-1:0] AN;
    wire db_request;
    wire op_request;
    wire [3-1:0] data_to_slave;
    wire rst_n_inv;
    wire [8-1:0]slave_data_dec;
    wire db_rst_n, op_rst_n;
	wire [8-1:0]data_to_slave_dec;
    assign rst_n_inv = ~op_rst_n;
    assign AN = 4'b1110;
	assign data_to_slave_o = data_to_slave;
    encoder enc0(.in(in), .out(data_to_slave));
    debounce db_0(.pb_debounced(db_request), .pb(request), .clk(clk));
    onepulse op_0(.pb_debounced(db_request), .clock(clk), .pb_one_pulse(op_request));
    debounce db_1(.pb_debounced(db_rst_n), .pb(rst_n), .clk(clk));
    onepulse op_1(.pb_debounced(db_rst_n), .clock(clk), .pb_one_pulse(op_rst_n));
    master_control ms_ctrl_0(.clk(clk), .rst_n(rst_n_inv), .request(op_request), .ack(ack), .data_in(data_to_slave), .notice(notice_master), .data(), .valid(valid), .request2s(request2s));
	decoder dec0(.in(data_to_slave), .out(data_to_slave_dec));
    seven_segment dis_0(.in(data_to_slave_dec), .out(seven_seg));


endmodule

//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25
// Module Name: decoder and seven_segment endcoder
// Project Name:Chip2Chip
// Additional Comments: display utility modules.
//////////////////////////////////////////////////////////////////////////////////

module decoder(in, out);
	input [3-1:0] in;
	output reg [7:0] out;
	always@(*) begin
		case(in)
			3'b000: out = 8'b0000_0001;
			3'b001: out = 8'b0000_0010;
			3'b010: out = 8'b0000_0100;
			3'b011: out = 8'b0000_1000;
			3'b100: out = 8'b0001_0000;
			3'b101: out = 8'b0010_0000;
			3'b110: out = 8'b0100_0000;
			3'b111: out = 8'b1000_0000;
		endcase
	end
endmodule

module seven_segment(in, out);
    input [8-1:0] in;
    output reg [7-1:0] out;
    always@(*) begin
        out[0] = (in[1]|in[4]);
        out[1] = (in[5]|in[6]);
        out[2] = (in[2]);
        out[3] = (in[1]|in[4]|in[7]);
        out[4] = (in[1]|in[3]|in[4]|in[5]|in[7]);
        out[5] = (in[1]|in[2]|in[3]|in[7]);
        out[6] = (in[0]|in[1]|in[7]);
    end
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25
// Module Name: encoder
// Project Name:Chip2Chip
// Additional Comments: Control block for master.
// I/O:
// clk           :clock signal.
// rst_n         :reset signal, reset module when rst_n == 0.
// request       :request signal from button press by user.
// ack           :ack sent by slave.
// data_in       :data input from user.
// notice        :signal indicating the recieve of ack from slave, will be asserted for 1 sec.
// data          :data output to slave.
// valid         :signal sent to slave indicating the current data is valid and is ready to be sampled.
// request2s     :request signal output to slave, notifying slave there's data to send.
//////////////////////////////////////////////////////////////////////////////////



module master_control(clk, rst_n, request, ack, data_in, notice, data, valid, request2s);
    input clk;
    input rst_n;
    input request;
    input ack;
    input [3-1: 0] data_in;
    output reg request2s;
    output reg notice;
    output reg [3-1:0] data;
    output reg valid;

    parameter state_wait_rqst = 3'b000;  // wait for user to push btn to send request to slave.
    parameter state_wait_ack  = 3'b001;  // request sent, wait for slave to resond with an ack, if no act is received, keep sending request2s
    parameter state_wait_to_send_data = 3'b100; //illuminate leftmost LED on the board for one sec indicating ack has been recieved
    parameter state_send_data = 3'b101; // send the actual data.

    reg [3-1:0] state, next_state;
    reg next_notice;
    reg [3-1:0] next_data;
    reg next_request2s;
    reg start, next_start; // control signals of counter.
    reg next_valid;

    wire done; //ouput from counter, asserted when counter has counted for 1 sec.

    counter cnt_0(.clk(clk), .rst_n(rst_n), .start(start), .done(done));

    always@(posedge clk) begin
        if (rst_n == 0) begin
            notice = 1'b0;
            state = state_wait_rqst;
            data = 0;
            request2s = 0;
            start = 0;
            valid = 0;
        end
        else begin
            notice <= next_notice;
            state <= next_state;
            data <= next_data;
            request2s <= next_request2s;
            start <= next_start;
            valid <= next_valid;
        end
    end

    always@(*) begin
        next_state = state;
        next_notice = notice;
        next_data = data;
        next_request2s = request2s;
        next_start = start;
        next_valid = valid;
        case(state)
            state_wait_rqst: begin
                next_state = (request == 1'b1)? state_wait_ack: state_wait_rqst;
                next_notice = 1'b0;
                next_data = 3'b000;
                next_request2s = (request == 1'b1)? 1'b1: 1'b0;
                next_start = 1'b0;
                next_valid = 1'b0;
            end

            state_wait_ack: begin
                next_state = (ack == 1'b1)? state_wait_to_send_data: state_wait_ack;
                next_notice = 1'b0;
                next_data = 3'b000;
                next_request2s = (ack == 1'b1)? 1'b0: 1'b1; // if no ack is present keep sending....
                next_start = (ack == 1'b1)? 1'b1: 1'b0; // if ack recieved, start counting for 1 second with counter.
                next_valid = 1'b0;
            end
            state_wait_to_send_data: begin
                next_state = (done == 1'b1)? state_send_data: state_wait_to_send_data;
                next_notice = (done == 1'b1)? 1'b0: 1'b1; //illuminating LED.
                next_data = (done == 1'b1)? data_in: 3'b000; // time to send data!
                next_request2s = 1'b0;
                next_start = (done == 1'b1)? 1'b0: 1'b1;
                next_valid = (done == 1)? 1'b1: 1'b0; //counting done!, time to set our output data as valid
            end
            state_send_data: begin
                next_state = (ack == 1'b0)? state_wait_rqst: state_send_data;
                next_notice = 1'b0;
                next_data = (ack == 1'b0)? 3'b000: data_in;
                next_request2s = 1'b0;
                next_start = 1'b0;
                next_valid = (ack == 1'b0)? 1'b0: 1'b1;
            end
            default: begin
            end
        endcase
    end
endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25
// Module Name: encoder
// Project Name:Chip2Chip
// Additional Comments: Encoder used to encode one hot switch input to binary encoding.
// I/O:
// in        :input from swithes, in one hot.
// out       :encoded ouput, in binary.
//////////////////////////////////////////////////////////////////////////////////


module encoder(in, out);
    input [8-1:0] in;
    output reg [3-1:0] out;
    always@(*) begin
        case(in)
            8'b0000_0001: out = 3'd0;
            8'b0000_0010: out = 3'd1;
            8'b0000_0100: out = 3'd2;
            8'b0000_1000: out = 3'd3;
            8'b0001_0000: out = 3'd4;
            8'b0010_0000: out = 3'd5;
            8'b0100_0000: out = 3'd6;
            8'b1000_0000: out = 3'd7;
            default: out = 0;
         endcase
    end
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25
// Module Name: counter
// Project Name:Chip2Chip
// Additional Comments:counter used to count one second for illumination of the leftmost LED on the board.
// I/O:
// clk       :clock signal.
// rst_n     :reset signal, reset the module when rst_n == 0.
// start     :when set to 1, counter will count.
// done      :asserted when counter had counted for 1 second.
//////////////////////////////////////////////////////////////////////////////////


module counter(clk, rst_n, start, done);
    input clk;
    input rst_n;
    input start;
    output reg done;
    reg [27-1:0] count, next_count;
    always@(posedge clk) begin
        if (rst_n == 0) begin
            count = 0;
        end
        else begin
            count <= next_count;
        end
    end

    always@(*) begin
        next_count = count;
        if (start) begin
            if (count == 27'd100000000) begin
                done = 1;
                next_count = 0;
            end
            else begin
                next_count = count + 1;
                done = 0;
            end
        end
        else begin
            done = 0;
            next_count = 0;
        end
    end
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//===================SLAVE===========================================================================

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25 12:47:53
// Module Name: top
// Project Name: Chip2Chip
// Additional Comments: top module for master, pass signals and perform debounce onepulse
//////////////////////////////////////////////////////////////////////////////////
module debounce (pb_debounced, pb, clk);
	output pb_debounced; // signal of a pushbutton after being debounced
	input pb; // signal from a pushbutton
	input clk;

	reg [3:0] DFF;
	always @(posedge clk)begin
		DFF[3:1] <= DFF[2:0];
		DFF[0] <= pb;
	end
	assign pb_debounced = ((DFF == 4'b1111) ? 1'b1 : 1'b0);
endmodule

module onepulse (pb_debounced, clock, pb_one_pulse);
	input pb_debounced;
	input clock;
	output reg pb_one_pulse;
	reg pb_debounced_delay;
	always @(posedge clock) begin
		pb_one_pulse <= pb_debounced & (! pb_debounced_delay);
		pb_debounced_delay <= pb_debounced;
	end
endmodule

module top(clk, rst_n, request, valid, seven_seg, notice_slave, AN, data_in, ack);
    input clk;
    input rst_n;
    input [3-1:0]data_in;
    input request;
    input valid;
    output [7-1:0] seven_seg;
    output notice_slave;
    output [4-1:0] AN;
    output ack;

    wire rst_n_inv;
    wire [3-1:0]slave_data_o;
    wire [8-1:0]slave_data_dec;
    wire db_rst_n, op_rst_n;
    assign rst_n_inv = ~op_rst_n;
    assign AN = 4'b1110;
    debounce db_0(.pb_debounced(db_rst_n), .pb(rst_n), .clk(clk));
    onepulse op_0(.pb_debounced(db_rst_n), .clock(clk), .pb_one_pulse(op_rst_n));
    slave_control sl_ctrl_0(.clk(clk), .rst_n(rst_n_inv), .request(request), .ack(ack), .data_in(data_in), .notice(notice_slave), .valid(valid), .data(slave_data_o));
    decoder dec0(.in(slave_data_o), .out(slave_data_dec));
    seven_segment dis_0(.in(slave_data_dec), .out(seven_seg));


endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25
// Module Name:  slave_control
// Project Name: Chip2Chip
// Additional Comments: Control block for slave.
// I/O:
// clk           :clock signal.
// rst_n         :reset signal, reset module when rst_n == 0.
// request       :request signal sent by the master.
// ack           :ack output to master.
// data_in       :data input from master.
// notice        :signal indicating the receive of data or request from master, will be asserted for 1 sec.
// data          :data output to the seven segment module, in order to display the data from master.
// valid         :signal from master indicating the current data_in is valid and is ready to be sampled.
//////////////////////////////////////////////////////////////////////////////////


module slave_control(clk, rst_n, request, ack, data_in, notice, valid, data);
    input clk;
    input rst_n;
    input request;
    input [3-1:0] data_in;
    input valid;
    output reg ack;
    output reg notice;
    output reg [3-1:0] data;

    parameter state_wait_rqst = 2'b00;
    parameter state_wait_to_send_ack = 2'b01;
    parameter state_send_ack = 2'b10;
    parameter state_wait_data = 2'b11;

    reg [2-1:0] state, next_state;
    reg start, next_start;
    reg next_ack;
    reg next_notice;
    reg [3-1:0] next_data;
    wire done;
    counter cnt_0(.clk(clk), .rst_n(rst_n), .start(start), .done(done));

    always@(posedge clk) begin
        if (rst_n == 0) begin
            state = state_wait_rqst;
            notice = 0;
            ack = 0;
            data = 0;
            start = 0;
        end
        else begin
            state <= next_state;
            notice <= next_notice;
            ack <= next_ack;
            data <= next_data;
            start <= next_start;
        end
    end

    always@(*) begin
        next_state = state;
        next_notice = notice;
        next_ack = ack;
        next_data = data;
        next_start = start;
        case(state)
            state_wait_rqst: begin
                next_state = (request == 1)? state_wait_to_send_ack: state_wait_rqst;
                 next_notice = 1'b0;
                 next_ack = 1'b0;
                 next_data = data;
                 next_start = (request == 1)? 1'b1: 1'b0;
            end
            state_wait_to_send_ack: begin
                 next_state = (done == 1)? state_wait_data : state_wait_to_send_ack;
                 next_notice = 1'b1;
                 next_ack = 1'b0;
                 next_data = data;
                 next_start = 1'b1;
            end
            state_wait_data: begin
                 next_state = (valid == 1)? state_wait_rqst : state_wait_data;
                 next_notice = 1'b0;
                 next_ack = 1'b1;
                 next_data = (valid==1)?data_in : data;
                 next_start = 1'b0;
            end
            default: begin
                 next_state = state_wait_rqst ;
                 next_notice = 1'b0;
                 next_ack = 1'b0;
                 next_data = 3'b0;
                 next_start = 1'b0;
            end
        endcase
    end
endmodule

//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25
// Module Name: decoder and seven_segment endcoder
// Project Name:Chip2Chip
// Additional Comments: display utility modules.
//////////////////////////////////////////////////////////////////////////////////

module decoder(in, out);
	input [3-1:0] in;
	output reg [7:0] out;
	always@(*) begin
		case(in)
			3'b000: out = 8'b0000_0001;
			3'b001: out = 8'b0000_0010;
			3'b010: out = 8'b0000_0100;
			3'b011: out = 8'b0000_1000;
			3'b100: out = 8'b0001_0000;
			3'b101: out = 8'b0010_0000;
			3'b110: out = 8'b0100_0000;
			3'b111: out = 8'b1000_0000;
		endcase
	end
endmodule

module seven_segment(in, out);
    input [8-1:0] in;
    output reg [7-1:0] out;
    always@(*) begin
        out[0] = (in[1]|in[4]);
        out[1] = (in[5]|in[6]);
        out[2] = (in[2]);
        out[3] = (in[1]|in[4]|in[7]);
        out[4] = (in[1]|in[3]|in[4]|in[5]|in[7]);
        out[5] = (in[1]|in[2]|in[3]|in[7]);
        out[6] = (in[0]|in[1]|in[7]);
    end
endmodule

//////////////////////////////////////////////////////////////////////////////////
// Company: NTHU
// Engineer: Bob Cheng
//
// Create Date: 2019/08/25
// Module Name: counter
// Project Name:Chip2Chip
// Additional Comments:counter used to count one second for illumination of the leftmost LED on the board.
// I/O:
// clk       :clock signal.
// rst_n     :reset signal, reset the module when rst_n == 0.
// start     :when set to 1, counter will count.
// done      :asserted when counter had counted for 1 second.
//////////////////////////////////////////////////////////////////////////////////


module counter(clk, rst_n, start, done);
    input clk;
    input rst_n;
    input start;
    output reg done;
    reg [27-1:0] count, next_count;
    always@(posedge clk) begin
        if (rst_n == 0) begin
            count = 0;
        end
        else begin
            count <= next_count;
        end
    end

    always@(*) begin
        next_count = count;
        if (start) begin
            if (count == 27'd100000000) begin
                done = 1;
                next_count = 0;
            end
            else begin
                next_count = count + 1;
                done = 0;
            end
        end
        else begin
            done = 0;
            next_count = 0;
        end
    end
endmodule
