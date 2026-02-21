`timescale 1ps/1ps
`include "Processor.v"

module RISCProcessor_wrapper(
    input clk,
    input Reset,
    input [7:0] InpExtWorld1,
    input [7:0] InpExtWorld2,
    input [7:0] InpExtWorld3,
    input [7:0] InpExtWorld4,
    output [7:0] OutExtWorld1,
    output [7:0] OutExtWorld2,
    output [7:0] OutExtWorld3,
    output [7:0] OutExtWorld4,

    // Exposed internals for testbench observation
    output [7:0] Dataout1,
    output [7:0] Dataout2,
    output [7:0] ALUout,
    output       OUTportWrite
);

    // Instantiate the RISCProcessor
    RISCProcessor uut (
        .clk(clk),
        .Reset(Reset),
        .InpExtWorld1(InpExtWorld1),
        .InpExtWorld2(InpExtWorld2),
        .InpExtWorld3(InpExtWorld3),
        .InpExtWorld4(InpExtWorld4),
        .OutExtWorld1(OutExtWorld1),
        .OutExtWorld2(OutExtWorld2),
        .OutExtWorld3(OutExtWorld3),
        .OutExtWorld4(OutExtWorld4)
    );

    // Expose internal signals
    assign Dataout1 = uut.Dataout1;  // Connect to internal Dataout1
    assign Dataout2 = uut.Dataout2;  // Connect to internal Dataout2
    assign ALUout   = uut.ALUout;    // Connect to internal ALUout
    assign OUTportWrite = uut.OUTportWrite; // Connect to internal OUTportWrite

endmodule
