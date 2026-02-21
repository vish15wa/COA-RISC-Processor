`timescale 1ns / 1ps
`include "extra.v"

module eightbitRegwithLoad(clk, Reset, load, Datain, Dataout);
    input clk, load, Reset;
    input [7:0] Datain;
    output reg [7:0] Dataout;

    wire [7:0] Y;

    assign Y = (load == 1'b1)? Datain: Dataout; //represents 2to1MUX_8-bit 

        // synchronous reset
    always @(posedge clk)
        begin
                if(Reset == 1'b1)
                    Dataout<=8'b0000_0000;
                else
                    Dataout<=Y;
        end
endmodule
/////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////

// Structural coding of RegisterFile module:
// 16 instances of eightbitRegwithLoad
// 1 instance of Decoder4to16_withE
// 2 instances of MUX16to1_8bit
// 2 instances of eightbitRegwithLoad

module RegisterFile(clk, Reset, RegFileRead, RegFileWrite, Datain, Source1, Source2, Destin, Dataout1, Dataout2);
    input clk, Reset, RegFileRead, RegFileWrite;
    input [7:0] Datain;
    input [3:0] Source1;
    input [3:0] Source2;
    input [3:0] Destin;
    output [7:0] Dataout1;
    output [7:0] Dataout2;

    wire [15:0] DecoderOut;
    wire [7:0] MUX1Out, MUX2Out;
    wire [7:0] RegFileOut[15:0]; // 16 registers of 8 bits each

    // Decoder4to16_withE instance
    Decoder4to16_withE Decoder1 (.Enable(RegFileWrite), .Input(Destin), .Output(DecoderOut));

    // 16 instances of eightbitRegwithLoad
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin: regfile
            eightbitRegwithLoad Reg (.clk(clk), .Reset(Reset), .load(DecoderOut[i]), .Datain(Datain), .Dataout(RegFileOut[i]));
        end
    endgenerate

    // MUX16to1_8bit instance for selecting Source1
    // MUX16to1_8bit instance for selecting Source1
    MUX16to1_8bit MUX1 (
        .Input({RegFileOut[15], RegFileOut[14], RegFileOut[13], RegFileOut[12],
                RegFileOut[11], RegFileOut[10], RegFileOut[9], RegFileOut[8],
                RegFileOut[7], RegFileOut[6], RegFileOut[5], RegFileOut[4],
                RegFileOut[3], RegFileOut[2], RegFileOut[1], RegFileOut[0]}),
        .Select(Source1),
        .Output(MUX1Out)
    );

    // MUX16to1_8bit instance for selecting Source2
    MUX16to1_8bit MUX2 (
        .Input({RegFileOut[15], RegFileOut[14], RegFileOut[13], RegFileOut[12],
                RegFileOut[11], RegFileOut[10], RegFileOut[9], RegFileOut[8],
                RegFileOut[7], RegFileOut[6], RegFileOut[5], RegFileOut[4],
                RegFileOut[3], RegFileOut[2], RegFileOut[1], RegFileOut[0]}),
        .Select(Source2),
        .Output(MUX2Out)
    );
    // Two eightbitRegwithLoad instances for storing the outputs of the MUXes
    eightbitRegwithLoad Reg1 (.clk(clk), .Reset(Reset), .load(RegFileRead), .Datain(MUX1Out), .Dataout(Dataout1));
    eightbitRegwithLoad Reg2 (.clk(clk), .Reset(Reset), .load(RegFileRead), .Datain(MUX2Out), .Dataout(Dataout2));
endmodule

/////////////////////////////////////////////////////////////////////

//////////////////////////////////// Stack module//////////////////////////////

module MUX4to1_8bit (
    input [7:0] in0,
    input [7:0] in1,
    input [7:0] in2,
    input [7:0] in3,
    input [1:0] sel,
    output reg [7:0] out
 );

    always @(*) begin
        case (sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
            default: out = 8'b0;
        endcase
    end
endmodule

module Counter (
    input clk,
    input Reset,
    input c_in,  // Increment enable
    input c_out, // Decrement enable
    output reg [7:0] SP  // Stack pointer
 );

    initial begin
        SP = 8'b0;
    end

    always @(posedge clk) begin
        if (Reset) begin
            SP <= 8'b0;
        end
        else begin
            if (c_in && !c_out) begin
                SP <= SP + 1; // Increment SP (push)
            end
            else if (c_out && !c_in) begin
                SP <= SP - 1; // Decrement SP (pop)
            end
            // If both c_in and c_out are 0, SP remains unchanged
            // If both are 1, SP remains unchanged (to avoid ambiguity)
        end
    end
endmodule


module Stack (
    input clk,
    input Reset,
    input StackRead,
    input StackWrite,
    input [7:0] Datain,
    output [7:0] Dataout
 );

    wire [7:0] SP;
    wire [7:0] SP_plus_1;
    wire [7:0] SP_minus_1;
    wire [7:0] sram_address;
    wire [7:0] sram_dout;
    wire [7:0] sp_write_ptr;
    wire [7:0] sp_read_ptr;

    wire [1:0] addr_sel;
    assign addr_sel = {StackRead, StackWrite};

    wire [7:0] zero = 8'b0;
    wire [7:0] all_ones = 8'hFF;

    assign SP_plus_1 = SP + 1;
    assign SP_minus_1 = SP - 1;

    assign sp_write_ptr = SP;
    assign sp_read_ptr = SP;

    Counter sp_counter (
        .clk(clk),
        .Reset(Reset),
        .c_in(StackWrite),
        .c_out(StackRead),
        .SP(SP)
    );

    MUX4to1_8bit addr_mux (
        .in0(zero),
        .in1(SP),
        .in2(SP_minus_1),
        .in3(zero),
        .sel(addr_sel),
        .out(sram_address)
    );

    SRAM sram (
        .clk(clk),
        .Reset(Reset),
        .Address(sram_address),
        .SRAMRead(StackRead),
        .SRAMWrite(StackWrite),
        .Datain(Datain),
        .Dataout(sram_dout)
    );

    MUX4to1_8bit output_mux (
        .in0(sram_dout),    // 00: idle (use SRAM output)
        .in1(sp_write_ptr), // 01: write (use SPWritePTR)
        .in2(sram_dout),    // 10: read (use SRAM output)
        .in3(all_ones),     // 11: invalid (use 11111111)
        .sel(addr_sel),
        .out(Dataout)
    );
endmodule

//////////////////////////////////////////////
//////////////// SRAM Module ///////////////////////
//////////////////////////////////////////////

module SRAM (
    input clk,
    input Reset,
    input [7:0] Address,
    input SRAMRead,
    input SRAMWrite,
    input [7:0] Datain,
    output reg [7:0] Dataout
    );

    // Declare the memory array: 256 locations, each 8 bits wide
    reg [7:0] datamem [0:255];

    // Initialize Dataout to 0 at the start of simulation
    initial begin
        Dataout = 8'b0;
    end

    // Main behavior on the rising edge of the clock
    always @(posedge clk) begin
        if (Reset) begin
            // On reset, set Dataout to 0
            Dataout <= 8'b0;
        end
        else begin
            // Write operation: if SRAMWrite is high, write Datain to the memory at Address
            if (SRAMWrite) begin
                datamem[Address] <= Datain;
            end

            // Read operation: if SRAMRead is high, read from memory at Address to Dataout
            if (SRAMRead) begin
                Dataout <= datamem[Address];
            end
            else if (!SRAMWrite) begin
                // If neither reading nor writing, hold the current Dataout value
                Dataout <= Dataout;
            end
        end
    end
endmodule
///////////////////////////////

////////////////// InstMEM ///////////////
//////////////////////////////////////////


module InstMEM (
    input clk,
    input Reset,
    input [7:0] Address,
    input InstRead,
    output reg [24:0] Dataout,
    output reg [4:0] Opcode,
    output reg [3:0] Destin,
    output reg [3:0] Source1,
    output reg [3:0] Source2,
    output reg [7:0] Imm
);

    reg [24:0] instmemory [0:255];
    
    
    initial begin
        $readmemb("instructions.txt", instmemory);
        // Initialize outputs to avoid undefined values
        Dataout = 25'b0;
        Opcode  = 5'b0;
        Destin  = 4'b0;
        Source1 = 4'b0;
        Source2 = 4'b0;
        Imm     = 8'b0;
    end

    always @(posedge clk) begin
    if (Reset) begin
        Dataout <= 25'b0;
        Opcode  <= 5'b0;
        Destin  <= 4'b0;
        Source1 <= 4'b0;
        Source2 <= 4'b0;
        Imm     <= 8'b0;
    end else if (InstRead) begin
        Dataout <= instmemory[Address];
        Opcode  <= instmemory[Address][24:20];
        Destin  <= instmemory[Address][19:16];
        Source1 <= instmemory[Address][15:12];
        Source2 <= instmemory[Address][11:8];
        Imm     <= instmemory[Address][7:0];
    end
end

endmodule




//////////////////////////////////////////////
//////////////// INPORT /////////////
//////////////////////////////////////////////

/*
module- INport
clk
Reset
INportRead
[7:0] InpExtWorld1
[7:0] InpExtWorld2
[7:0] InpExtWorld3
[7:0] InpExtWorld4
[7:0] Address
[7:0] Dataout

*/

module INport(clk, Reset, INportRead, InpExtWorld1, InpExtWorld2, InpExtWorld3, InpExtWorld4, Address, Dataout);
    input clk, Reset, INportRead;
    input [7:0] InpExtWorld1;
    input [7:0] InpExtWorld2;
    input [7:0] InpExtWorld3;
    input [7:0] InpExtWorld4;
    input [7:0] Address;
    output reg [7:0] Dataout;

    always @(posedge clk or posedge Reset) begin
        if (Reset) begin
            Dataout <= 8'b0; // Reset Dataout to 0
        end else if (INportRead) begin
            case (Address[1:0]) // Use only the lower 2 bits of Address
                2'b00: Dataout <= InpExtWorld1;
                2'b01: Dataout <= InpExtWorld2;
                2'b10: Dataout <= InpExtWorld3;
                2'b11: Dataout <= InpExtWorld4;
                default: Dataout <= 8'b0; // Default case to avoid latches
            endcase
        end else begin
            Dataout <= Dataout; // Hold current value if not reading
        end
    end
endmodule

//////////////////////////////////////////////
///////////////OUTPORT//////////////////////
//////////////////////////////////////////////

/*
module- OUTport

clk
Reset
[7:0] Address
[7:0] Datain
OUTportWrite
[7:0] OutExtWorld1
[7:0] OutExtWorld2
[7:0] OutExtWorld3
[7:0] OutExtWorld4
*/

module OUTport(clk, Reset, Address, Datain, OUTportWrite, OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4);
    input clk, Reset, OUTportWrite;
    input [7:0] Address;
    input [7:0] Datain;
    output reg [7:0] OutExtWorld1;
    output reg [7:0] OutExtWorld2;
    output reg [7:0] OutExtWorld3;
    output reg [7:0] OutExtWorld4;

    always @(posedge clk or posedge Reset) begin
        if (Reset) begin
            // Reset all output registers to 0
            OutExtWorld1 <= 8'b0;
            OutExtWorld2 <= 8'b0;
            OutExtWorld3 <= 8'b0;
            OutExtWorld4 <= 8'b0;
        end else if (OUTportWrite) begin
            case (Address[1:0]) // Use only the lower 2 bits of Address
                2'b00: OutExtWorld1 <= Datain;
                2'b01: OutExtWorld2 <= Datain;
                2'b10: OutExtWorld3 <= Datain;
                2'b11: OutExtWorld4 <= Datain;
                default: begin
                    // Do nothing for other addresses to avoid latches
                end
            endcase
        end else begin
            // Hold current values if not writing
            OutExtWorld1 <= OutExtWorld1;
            OutExtWorld2 <= OutExtWorld2;
            OutExtWorld3 <= OutExtWorld3;
            OutExtWorld4 <= OutExtWorld4;
        end
    end
endmodule

///////////////////////////////////////////////////
////////////////////////   ControlLogic  ///////////////////////
///////////////////////////////////////////////////

/* module- ControlLogic

clk
Reset
T1
T2
T3
T4
Zflag
Cflag
[4:0] Opcode
PCupdate
SRAMRead
SRAMWrite
StackRead
StackWrite
ALUSave
ZflagSave
CflagSave
INportRead
OUTportWrite
RegFileRead
RegFileWrite

Structural coding preferred, with some important instances:
Each control signal would be implemented using an instance of MUX32to1_1bit_withE
*/
module ControlLogic(
    input clk, Reset, T1, T2, T3, T4,
    input Zflag, Cflag,
    input [4:0] Opcode,
    output reg PCupdate, SRAMRead, SRAMWrite, StackRead, StackWrite,
    output reg ALUSave, ZflagSave, CflagSave, INportRead, OUTportWrite,
    output reg RegFileRead, RegFileWrite
);

    always @(*) begin
    PCupdate = 0;
    SRAMRead = 0;
    SRAMWrite = 0;
    StackRead = 0;
    StackWrite = 0;
    ALUSave = 0;
    ZflagSave = 0;
    CflagSave = 0;
    INportRead = 0;
    OUTportWrite = 0;
    RegFileRead = 0;
    RegFileWrite = 0;

    case (Opcode)
        5'b01010: begin // MOVI
            if (T2) ALUSave = 1;     // Compute immediate in ALU
            if (T3) RegFileWrite = 1; // Write to register
        end
        5'b00000, 5'b11000, 5'b00001, 5'b00010, 5'b11001: begin // ADD, SUB, AND, OR, Shiftright
            if (T1) RegFileRead = 1; // Fetch operands
            if (T2) ALUSave = 1;     // Compute result
            if (T3) RegFileWrite = 1; // Write back
        end
        5'b00111: begin // EXORI
            if (T1) RegFileRead = 1; // Fetch Src1
            if (T2) ALUSave = 1;     // Compute XOR with Imm
            if (T3) RegFileWrite = 1; // Write back
        end
        5'b01000: begin // INport
            if (T1) INportRead = 1;
            if (T2) RegFileWrite = 1;
        end
        5'b01001: begin // OUTport
            if (T1) RegFileRead = 1;
            if (T2) OUTportWrite = 1;
        end
        5'b10000: begin // PUSH
                if (T1) RegFileRead = 1;   // Read data from register
                if (T2) StackWrite = 1;   // Write data to stack
            end
        5'b10001: begin // POP
            if (T1) StackRead = 1;    // Read data from stack
            if (T2) RegFileWrite = 1; // Write data to register
        end
    endcase
end
endmodule

/////////////////////////////////////////
/////////////////////// ALU //////////////////////////
/////////////////////////////////////


module ALU (
    input wire clk, Reset,
    input wire [7:0] Imm7,
    input wire [7:0] Operand1, Operand2,
    input wire [4:0] Opcode,
    input wire ALUSave, ZflagSave, CflagSave,
    output reg Zflag, Cflag,
    output reg [7:0] ALUout
);

    reg [7:0] ALU_Result;
    reg CarryOut;
    wire [7:0] ImmExtended = Imm7; // No sign-extension needed for this ISA

    always @(*) begin
        case (Opcode)
            5'b00000: {CarryOut, ALU_Result} = Operand1 + Operand2; // ADD
            5'b11000: {CarryOut, ALU_Result} = Operand1 - Operand2; // SUB
            5'b00001: begin ALU_Result = Operand1 & Operand2; CarryOut = 0; end // AND
            5'b00010: begin ALU_Result = Operand1 | Operand2; CarryOut = 0; end // OR
            5'b00111: begin ALU_Result = Operand1 ^ ImmExtended; CarryOut = 0; end // EXORI
            5'b11001: begin ALU_Result = Operand1 >> Operand2; CarryOut = 0; end // Shiftright
            5'b01010: begin ALU_Result = ImmExtended; CarryOut = 0; end // MOVI
            default: begin ALU_Result = 8'd0; CarryOut = 0; end
        endcase
    end

    always @(posedge clk or posedge Reset) begin
        if (Reset) begin
            ALUout <= 8'd0;
            Zflag <= 0;
            Cflag <= 0;
        end else begin
            if (ALUSave) ALUout <= ALU_Result;
            if (ZflagSave) Zflag <= (ALU_Result == 8'd0);
            if (CflagSave) Cflag <= CarryOut;
        end
    end
endmodule

//////////////////////////////////////////////
///////////////////// TimingGEN ////////////////


module TimingGen (
    input wire clk,
    input wire Reset,
    output reg T0,
    output reg T1,
    output reg T2,
    output reg T3,
    output reg T4
);

    reg [2:0] counter;

    always @(posedge clk or posedge Reset) begin
        if (Reset)
            counter <= 3'b000;
        else
            counter <= (counter == 3'd4) ? 3'd0 : counter + 1'b1;
    end

    always @(*) begin
        T0 = (counter == 3'd0);
        T1 = (counter == 3'd1);
        T2 = (counter == 3'd2);
        T3 = (counter == 3'd3);
        T4 = (counter == 3'd4);
    end

endmodule

//////////////////////////////////////////////
////////////////////// ProgCounter ///////////

module ProgCounter (
    input wire clk, Reset, PCenable, PCupdate,
    input wire [7:0] CAddress,
    output reg [7:0] PC, PC_D2
);

    reg [7:0] PC_D1;
    wire [7:0] PCPlus1 = PC + 8'd1;
    wire [7:0] PCNext = PCenable ? PCPlus1 : PC;
    wire [7:0] PCInput = PCupdate ? CAddress : PCNext;

    always @(posedge clk or posedge Reset) begin
        if (Reset) begin
            PC <= 8'd0;
            PC_D1 <= 8'd0;
            PC_D2 <= 8'd0;
        end else begin
            PC <= PCInput;
            PC_D1 <= PC;
            PC_D2 <= PC_D1;
        end
    end
endmodule
//////////////////////////////////////////////