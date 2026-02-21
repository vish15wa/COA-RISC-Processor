`timescale 1ps/1ps
`include "subcomponents.v"

module RISCProcessor(
    input clk,
    input Reset,
    input [7:0] InpExtWorld1,
    input [7:0] InpExtWorld2,
    input [7:0] InpExtWorld3,
    input [7:0] InpExtWorld4,
    output [7:0] OutExtWorld1,
    output [7:0] OutExtWorld2,
    output [7:0] OutExtWorld3,
    output [7:0] OutExtWorld4
);

    // Internal wires
    wire [7:0] PC, PC_D2, ALUout, Dataout1, Dataout2, SRAMDataout, StackDataout, INportDataout;
    wire [7:0] Datain, CAddress;
    wire [4:0] Opcode;
    wire [3:0] Source1, Source2, Destin;
    wire [7:0] Imm7;
    wire PCupdate, SRAMRead, SRAMWrite, StackRead, StackWrite, ALUSave, ZflagSave, CflagSave, INportRead, OUTportWrite, RegFileRead, RegFileWrite;
    wire Zflag, Cflag, T0, T1, T2, T3, T4;
    wire [7:0] ImmExtended = Imm7;
    wire [7:0] Operand1 = Dataout1;
    wire [7:0] Operand2 = (Opcode == 5'b01010) ? ImmExtended : Dataout2; // Use ImmExtended for MOVI
    // Update wire widths
    wire [24:0] InstOut; // Match InstMEM's Dataout width
    
    initial begin
        $monitor("Time: %0t | PC: %h | Opcode: %b | Operand1: %h | Operand2: %h | ALUout: %h | OUTportWrite: %b", 
                 $time, PC, Opcode, Dataout1, Dataout2, ALUout, OUTportWrite);
    end


    // Correct InstMEM instantiation
    InstMEM inst_mem (
        .clk(clk),
        .Reset(Reset),
        .Address(PC),
        .InstRead(1'b1), // Assuming instruction read is always enabled
        .Dataout(InstOut), // Match 25-bit width
        .Opcode(Opcode),
        .Destin(Destin),
        .Source1(Source1),
        .Source2(Source2),
        .Imm(Imm7) // Match 8-bit width
    );

    // Program Counter (ProgCounter)
    ProgCounter prog_counter (
        .clk(clk),
        .Reset(Reset),
        .PCenable(T0),
        .PCupdate(PCupdate),
        .CAddress(CAddress),
        .PC(PC),
        .PC_D2(PC_D2)
    );

    // Timing Generator (TimingGen)
    TimingGen timing_gen (
        .clk(clk),
        .Reset(Reset),
        .T0(T0),
        .T1(T1),
        .T2(T2),
        .T3(T3),
        .T4(T4)
    );

    // Debugging block for TimingGen
    always @(posedge clk or posedge Reset) begin
        if (Reset)
            $display("Reset: Counter = %b", timing_gen.counter); // Ensure `counter` is accessible
        else
            $display("Clock Edge: Counter = %b, T0 = %b", timing_gen.counter, T0);
    end

    wire [4:0] writeback_sel;
    assign writeback_sel = (Opcode == 5'b10001) ? 5'd1 : // POP (StackDataout)
                       (Opcode == 5'b01000) ? 5'd2 : // INportRead
                       (Opcode == 5'b01001) ? 5'd3 : // OUTportWrite (ALUout)
                       (Opcode[4] == 1'b0 || Opcode == 5'b01010 || Opcode == 5'b00111) ? 5'd3 : // ALUout
                       5'd0; // Default (SRAMDataout)

    MUX32to1_8bit writeback_mux (
        .Input({SRAMDataout, StackDataout, INportDataout, ALUout, 224'b0}), // ALUout at index 3
        .sel(writeback_sel),
        .out(Datain)
    );

    // Register File
    RegisterFile reg_file (
    .clk(clk),
    .Reset(Reset),
    .RegFileRead(RegFileRead),
    .RegFileWrite(RegFileWrite),
    .Datain(Datain), // From writeback_mux
    .Source1(Source1),
    .Source2(Source2),
    .Destin(Destin),
    .Dataout1(Dataout1),
    .Dataout2(Dataout2)
);
    // ALU
    ALU alu (
        .clk(clk),
        .Reset(Reset),
        .Imm7(Imm7),
        .Operand1(Dataout1),
        .Operand2(Dataout2),
        .Opcode(Opcode[4:0]),
        .ALUSave(ALUSave),
        .ZflagSave(ZflagSave),
        .CflagSave(CflagSave),
        .Zflag(Zflag),
        .Cflag(Cflag),
        .ALUout(ALUout)
    );

    // SRAM
    SRAM sram (
        .clk(clk),
        .Reset(Reset),
        .Address(ALUout),
        .SRAMRead(SRAMRead),
        .SRAMWrite(SRAMWrite),
        .Datain(Dataout2),
        .Dataout(SRAMDataout)
    );

    // Stack
    Stack stack (
        .clk(clk),
        .Reset(Reset),
        .StackRead(StackRead),
        .StackWrite(StackWrite),
        .Datain(Dataout2),
        .Dataout(StackDataout)
    );
     // Output from the stack

    MUX32to1_8bit writeback_mux1 (
        .Input({SRAMDataout, StackDataout, INportDataout, ALUout, 224'b0}), // Add StackDataout
        .sel(writeback_sel),
        .out(Datain)
    );
    // INport
    INport in_port (
        .clk(clk),
        .Reset(Reset),
        .INportRead(INportRead),
        .InpExtWorld1(InpExtWorld1),
        .InpExtWorld2(InpExtWorld2),
        .InpExtWorld3(InpExtWorld3),
        .InpExtWorld4(InpExtWorld4),
        .Address(ALUout),
        .Dataout(INportDataout)
    );

    // OUTport
    OUTport out_port (
        .clk(clk),
        .Reset(Reset),
        .Address(ALUout),
        .Datain(ALUout), // Ensure ALUout is written to the output
        .OUTportWrite(OUTportWrite),
        .OutExtWorld1(OutExtWorld1),
        .OutExtWorld2(OutExtWorld2),
        .OutExtWorld3(OutExtWorld3),
        .OutExtWorld4(OutExtWorld4)
    );

    // Control Logic
    ControlLogic control_logic (
        .clk(clk),
        .Reset(Reset),
        .T1(T1),
        .T2(T2),
        .T3(T3),
        .T4(T4),
        .Zflag(Zflag),
        .Cflag(Cflag),
        .Opcode(Opcode),
        .PCupdate(PCupdate),
        .SRAMRead(SRAMRead),
        .SRAMWrite(SRAMWrite),
        .StackRead(StackRead),
        .StackWrite(StackWrite),
        .ALUSave(ALUSave),
        .ZflagSave(ZflagSave),
        .CflagSave(CflagSave),
        .INportRead(INportRead),
        .OUTportWrite(OUTportWrite),
        .RegFileRead(RegFileRead),
        .RegFileWrite(RegFileWrite)
    );

    

endmodule