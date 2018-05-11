`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.04.2018 11:31:55
// Design Name: 
// Module Name: MFIFO_DataBuffer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module MFIFO_DataBuffer(
    clk,
    areset,
    data_in,
    data_out,
    full,
    empty
    );
    input clk;
    input areset;
    input data_in;
    output data_out;
    output full;
    output empty;
    
    reg[7:0] mem[0:7][0:7];
    always@(posedge clk or negedge areset)
    begin
        if(!areset)
        begin
            
        end
        else
        begin
            
        end
    end
    
    
endmodule
