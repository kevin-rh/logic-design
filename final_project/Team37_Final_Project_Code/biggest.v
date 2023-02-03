module biggest #(
    //parameter N_DATA = 2,  // Data's binary repr length
    parameter N_KEY = 16     // Key's binary repr length
)(
    input clk,
    input reset,
    input enable,
    input [2-1:0] data,
    input [N_KEY-1:0] key,
    output reg [N_KEY-1:0] max_key
);
    reg [2*8-1:0] mem_data;

    reg [2-1:0] max_data, next_max_data;
    reg [N_KEY-1:0]  next_max_key;

    always @(posedge clk) begin
        if(reset) begin
            max_data <= {2{1'b0}}; 
            max_key <= {N_KEY{1'b0}};
        end else begin
            max_data <= next_max_data;
            max_key <= next_max_key;
        end
    end
    
    always @(posedge clk) begin
        mem_data[1:0] <= data;
        mem_data[3:2] <= mem_data[1:0];
        mem_data[5:4] <= mem_data[3:2];
        mem_data[7:6] <= mem_data[5:4];
        mem_data[9:8] <= mem_data[7:6];
        mem_data[11:10] <= mem_data[9:8];
        mem_data[13:12] <= mem_data[11:10];
        mem_data[15:14] <= mem_data[13:12];
    end
    wire [4:0] out;
    sum16 sumup (mem_data, out);
    always @(*) begin
        if(enable && (max_data < out)) begin
            next_max_data = out;
            next_max_key = key;
        end else begin
            next_max_data = max_data;
            next_max_key = max_key;
        end
    end
endmodule