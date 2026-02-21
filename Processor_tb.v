`include "RISCProcessor_wrapper.v"

module Processor_tb;
    reg clk, Reset;
    reg [7:0] InpExtWorld1, InpExtWorld2, InpExtWorld3, InpExtWorld4;
    wire [7:0] OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4;
    wire [7:0] Dataout1, Dataout2, ALUout;
    wire OUTportWrite;

    RISCProcessor_wrapper wrapper (
        .clk(clk), .Reset(Reset), .InpExtWorld1(InpExtWorld1), .InpExtWorld2(InpExtWorld2),
        .InpExtWorld3(InpExtWorld3), .InpExtWorld4(InpExtWorld4), .OutExtWorld1(OutExtWorld1),
        .OutExtWorld2(OutExtWorld2), .OutExtWorld3(OutExtWorld3), .OutExtWorld4(OutExtWorld4),
        .Dataout1(Dataout1), .Dataout2(Dataout2), .ALUout(ALUout), .OUTportWrite(OUTportWrite)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0; Reset = 1;
        InpExtWorld1 = 8'h04; InpExtWorld2 = 8'h0C; InpExtWorld3 = 8'h03; InpExtWorld4 = 8'h3C;
        #10 Reset = 0;
        #500 $display("Time: %0t | R1: %h | R2: %h | R3: %h | R6: %h | R10: %h | R13: %h | R14: %h | R15: %h",
            $time, wrapper.uut.reg_file.RegFileOut[1], wrapper.uut.reg_file.RegFileOut[2],
            wrapper.uut.reg_file.RegFileOut[3], wrapper.uut.reg_file.RegFileOut[6],
            wrapper.uut.reg_file.RegFileOut[10], wrapper.uut.reg_file.RegFileOut[13],
            wrapper.uut.reg_file.RegFileOut[14], wrapper.uut.reg_file.RegFileOut[15]);
        #1000 $finish;
    end

    initial $monitor("Time: %0t | PC: %h | Opcode: %b | Src1: %h | Src2: %h | Dest: %h | Dataout1: %h | Dataout2: %h | ALUout: %h | RegFileWrite: %b | T1: %b | T2: %b | T3: %b | R1: %h | R2: %h | R3: %h",
        $time, wrapper.uut.PC, wrapper.uut.Opcode, wrapper.uut.Source1, wrapper.uut.Source2, wrapper.uut.Destin,
        Dataout1, Dataout2, ALUout, wrapper.uut.RegFileWrite, wrapper.uut.T1, wrapper.uut.T2, wrapper.uut.T3,
        wrapper.uut.reg_file.RegFileOut[1], wrapper.uut.reg_file.RegFileOut[2], wrapper.uut.reg_file.RegFileOut[3]);

    initial begin
        $dumpfile("Processor_tb.vcd");
        $dumpvars(0, wrapper.uut.timing_gen); // Correctly reference TimingGen instance
        $dumpvars(0, wrapper.uut); // Dump all signals under uut
    end
endmodule