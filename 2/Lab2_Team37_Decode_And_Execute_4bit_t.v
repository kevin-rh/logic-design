`timescale 10ps/1ps
module Decode_and_Execute_t;

reg CLK = 1;

reg [2:0] sel = 3'b000;
reg [3:0] rs = 4'b0000;
reg [3:0] rt = 4'b0000;

wire [3:0] rd;

Decode_And_Execute tes (
    .rs(rs), 
    .rt(rt), 
    .sel(sel), 
    .rd(rd)
);


always  #1 CLK = ~CLK;

initial begin
  sel = 3'b000;
  repeat (2 ** 3) begin
    {rs, rt} = 8'b00000000;
    repeat (2 ** 8) begin
      @ (posedge CLK) Test;
      @ (negedge CLK) {rs, rt} = {rs, rt} + 8'b00000001;
    end
    sel = sel + 3'b001;
  end
  #1 $finish;
end


task Test;
begin
    case (sel)
    3'b000: begin
        // ADD
        if (rd !== rs + rt) begin
            $display("[ERROR] ADD");
            $write("rs: %d\n", rs);
            $write("rt: %d\n", rt);
            $write("rd: %d\n", rd);
            $write("expect: %d\n", rs + rt);
            $display;
        end
    end
    3'b001: begin
        // SUB
        if (rd !== rs - rt) begin
            $display("[ERROR] SUB");
            $write("rs: %d\n", rs);
            $write("rt: %d\n", rt);
            $write("rd: %d\n", rd);
            $write("expect: %d\n", rs - rt);
            $display;
        end
    end
    3'b010: begin
        // BITWISE AND
        if (rd !== (rs & rt)) begin
            $display("[ERROR] BITWISE AND");
            $write("rs: %d\n", rs);
            $write("rt: %d\n", rt);
            $write("rd: %d\n", rd);
            $write("expect: %d\n", (rs & rt));
            $display;
        end
    end
    3'b011: begin
      // BITWISE OR
      if (rd !== (rs | rt)) begin
            $display("[ERROR] BITWISE OR");
            $write("rs: %d\n", rs);
            $write("rt: %d\n", rt);
            $write("rd: %d\n", rd);
            $write("expect: %d\n", ~(rs | rt));
            $display;
        end
    end
    3'b100: begin
        // RS CIR LEFT SHIFT 
        if (rd !== {rs[2:0], rs[3]} ) begin
            $display("[ERROR] RS CIR LEFT SHIFT");
            $write("rs: %d\n", rs);
            $write("rt: %d\n", rt);
            $write("rd: %d\n", rd);
            $write("expect: %d\n", {rs[2:0], rs[3]});
            $display;
        end
    end
    3'b101: begin
      // RT ARI RIGHT SHIFT
        if (rd !== {rt[3], rt[3:1]}) begin
            $display("[ERROR] RT ARI RIGHT SHIFT");
            $write("rs: %d\n", rs);
            $write("rt: %d\n", rt);
            $write("rd: %d\n", rd);
            $write("expect: %d\n", {rt[3], rt[3:1]});
            $display;
        end
    end
    3'b110: begin
      // COMPARE EQ
        if(rs == rt) begin
            if (rd !== 4'b1111) begin
                $display("[ERROR] COMPARE GT");
                $write("rs: %d\n", rs);
                $write("rt: %d\n", rt);
                $write("rd: %d\n", rd);
                $write("expect: %d\n", {4'b1011});
                $display;
            end
        end
        else begin
            if (rd !== {4'b1110}) begin
                $display("[ERROR] COMPARE GT");
                $write("rs: %d\n", rs);
                $write("rt: %d\n", rt);
                $write("rd: %d\n", rd);
                $write("expect: %d\n", {4'b1110});
                $display;
            end
        end
    end
    3'b111: begin
        // COMPARE GT
        if(rs > rt) begin
            if (rd !== 4'b1011) begin
                $display("[ERROR] COMPARE GT");
                $write("rs: %d\n", rs);
                $write("rt: %d\n", rt);
                $write("rd: %d\n", rd);
                $write("expect: %d\n", {4'b1011});
                $display;
            end
        end
        else begin
            if (rd !== {4'b1010}) begin
                $display("[ERROR] COMPARE GT");
                $write("rs: %d\n", rs);
                $write("rt: %d\n", rt);
                $write("rd: %d\n", rd);
                $write("expect: %d\n", {4'b1010});
                $display;
            end
        end
    end
    default: begin
        $display("[ERROR] unknown sel: %d", sel);
    end
  endcase
end
endtask

endmodule
