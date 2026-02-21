`timescale 1ps/1ps

module Decoder4to16_withE (Enable, Input, Output);
    input Enable;
    input [3:0] Input;
    output reg [15:0] Output;

    always @(Enable, Input)
    begin
        if (Enable == 1'b1) begin
            case (Input)
                4'b0000: Output = 16'b0000_0000_0000_0001;
                4'b0001: Output = 16'b0000_0000_0000_0010;
                4'b0010: Output = 16'b0000_0000_0000_0100;
                4'b0011: Output = 16'b0000_0000_0000_1000;
                4'b0100: Output = 16'b0000_0000_0001_0000;
                4'b0101: Output = 16'b0000_0000_0010_0000;
                4'b0110: Output = 16'b0000_0000_0100_0000;
                4'b0111: Output = 16'b0000_0001_0000_0000;
                4'b1000: Output = 16'b0000_0010_0000_0000;
                4'b1001: Output = 16'b0000_0100_0000_0000;
                4'b1010: Output = 16'b0001_0000_0000_0000;
                4'b1011: Output = 16'b0010_0000_0000_0000;
                4'b1100: Output = 16'b0100_0000_0000_0000;
                4'b1101: Output = 16'b1000_0000_0000_0000;
                default: Output = 16'b1111_1111_1111_1111; // Invalid input
            endcase
        end else begin
            Output = 16'b1111_1111_1111_1111; // Disable output when Enable is low
        end
    end
endmodule

module MUX16to1_8bit (Input, Select, Output);
    input [127:0] Input; // 16 inputs of 8 bits each
    input [3:0] Select; // 4-bit select line
    output reg [7:0] Output; // 8-bit output

    always @(Input, Select)
    begin
        case (Select)
            4'b0000: Output = Input[7:0];
            4'b0001: Output = Input[15:8];
            4'b0010: Output = Input[23:16];
            4'b0011: Output = Input[31:24];
            4'b0100: Output = Input[39:32];
            4'b0101: Output = Input[47:40];
            4'b0110: Output = Input[55:48];
            4'b0111: Output = Input[63:56];
            4'b1000: Output = Input[71:64];
            4'b1001: Output = Input[79:72];
            4'b1010: Output = Input[87:80];
            4'b1011: Output = Input[95:88];
            4'b1100: Output = Input[103:96];
            4'b1101: Output = Input[111:104];
            4'b1110: Output = Input[119:112];
            4'b1111: Output = Input[127:120];
            default: Output = 8'b00000000; // Default case
        endcase
    end
endmodule

////////////////////////////////////////
//////////////////////  MUX 32to1_with_E //////////////////
module MUX32to1_1bit_withE(
    input Enable,               // Enable signal
    input [31:0] Input,         // 32 input signals
    input [4:0] Select,         // 5-bit select signal to choose one of the 32 inputs
    output reg Output           // Output signal
 );

    always @(*) begin
        if (Enable) begin
            case (Select)
                5'b00000: Output = Input[0];
                5'b00001: Output = Input[1];
                5'b00010: Output = Input[2];
                5'b00011: Output = Input[3];
                5'b00100: Output = Input[4];
                5'b00101: Output = Input[5];
                5'b00110: Output = Input[6];
                5'b00111: Output = Input[7];
                5'b01000: Output = Input[8];
                5'b01001: Output = Input[9];
                5'b01010: Output = Input[10];
                5'b01011: Output = Input[11];
                5'b01100: Output = Input[12];
                5'b01101: Output = Input[13];
                5'b01110: Output = Input[14];
                5'b01111: Output = Input[15];
                5'b10000: Output = Input[16];
                5'b10001: Output = Input[17];
                5'b10010: Output = Input[18];
                5'b10011: Output = Input[19];
                5'b10100: Output = Input[20];
                5'b10101: Output = Input[21];
                5'b10110: Output = Input[22];
                5'b10111: Output = Input[23];
                5'b11000: Output = Input[24];
                5'b11001: Output = Input[25];
                5'b11010: Output = Input[26];
                5'b11011: Output = Input[27];
                5'b11100: Output = Input[28];
                5'b11101: Output = Input[29];
                5'b11110: Output = Input[30];
                5'b11111: Output = Input[31];
                default: Output = 1'b0; // Default case if Select is invalid
            endcase
        end else begin
            Output = 1'b0; // Output is 0 when Enable is low
        end
    end

endmodule

module MUX32to1_8bit (
    input [255:0] Input, // 32 inputs, each 8 bits wide (32 * 8 = 256 bits)
    input [4:0] sel,     // 5-bit selector for 32 inputs
    output reg [7:0] out // 8-bit output
);

    always @(*) begin
        case (sel)
            5'd0: out = Input[7:0];
            5'd1: out = Input[15:8];
            5'd2: out = Input[23:16];
            5'd3: out = Input[31:24];
            5'd4: out = Input[39:32];
            5'd5: out = Input[47:40];
            5'd6: out = Input[55:48];
            5'd7: out = Input[63:56];
            5'd8: out = Input[71:64];
            5'd9: out = Input[79:72];
            5'd10: out = Input[87:80];
            5'd11: out = Input[95:88];
            5'd12: out = Input[103:96];
            5'd13: out = Input[111:104];
            5'd14: out = Input[119:112];
            5'd15: out = Input[127:120];
            5'd16: out = Input[135:128];
            5'd17: out = Input[143:136];
            5'd18: out = Input[151:144];
            5'd19: out = Input[159:152];
            5'd20: out = Input[167:160];
            5'd21: out = Input[175:168];
            5'd22: out = Input[183:176];
            5'd23: out = Input[191:184];
            5'd24: out = Input[199:192];
            5'd25: out = Input[207:200];
            5'd26: out = Input[215:208];
            5'd27: out = Input[223:216];
            5'd28: out = Input[231:224];
            5'd29: out = Input[239:232];
            5'd30: out = Input[247:240];
            5'd31: out = Input[255:248];
            default: out = 8'b0; // Default case to avoid latches
        endcase
    end

endmodule

