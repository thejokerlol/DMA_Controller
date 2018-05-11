`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04.03.2018 15:46:53
// Design Name: 
// Module Name: Instruction_cache_tb
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


module Instruction_cache_tb(

    );
    reg clk;
    reg[31:0] address;
    reg areset;
    reg read;
    reg enable_RW;
    reg[31:0] data_in;
    wire[31:0] data_out;
    wire hit;
    
    Instruction_Cache I1(
    
        clk,
        address,
        areset,
        read,
        enable_RW,
        data_in,
        data_out,
        hit 
    
        );
        
    initial
    begin
        clk=0;
        areset=1'b1;
        address=32'd0;
        read=0;
        enable_RW=0;
        data_in=32'd0;
    end    
        
    initial
    begin
        #100 $finish();
    end    
    
    always
        #10 clk=!clk;
        
    
    
    initial
    begin
        #2 areset=1'b0;
        #5 areset=1'b1;
        #20 address=32'd89;
            enable_RW=1'b1;
            read=1'b0;
            data_in=32'd15;
         
        #20 address=32'd57;
            enable_RW=1'b1;
            read=1'b0;
         
         
         #20 address=32'd89;
            enable_RW=1'b1;
            read=1'b1;
            
         #20 enable_RW=0;   
            
    end    
endmodule
