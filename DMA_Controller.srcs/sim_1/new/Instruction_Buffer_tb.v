`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2018 15:45:58
// Design Name: 
// Module Name: Instruction_Buffer_tb
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


module Instruction_Buffer_tb(

    );
    reg clk;
    reg reset;
    reg write;
    reg read;
    reg[31:0] data_in;
    wire[31:0] data_out;
    wire empty;
    wire full;
    
    Instruction_Buffer IB(
        clk,
        reset,
        write,
        data_in,
        read,
        data_out,
        empty,
        full
    
        );
        
        initial
        begin
            clk=0;
            reset=1;
            write=0;
            read=0;
            data_in=0;
            
        end
        
        initial
        begin
            #250 $finish();
        end
        
        always
            #5 clk=!clk;
        
        initial
        begin
            #2 reset=0;
            #2 reset=1;
            
            #3 write=1;
               data_in=20;
               
            #10 write=0;
            #10 write=1;
                data_in=45;
            #10 write=1;
                data_in=56;
                
            #10 read=1;
                write=0;
                
            #10 read=1;
            #10 read=1;
            #10 read=1;
            
            #10 read=0;
            
            
            
            /*
                test for full
            */ 
            #10 write=1;
                data_in=31;
            #10 write=1;
                data_in=32;
            #10 write=1;
                data_in=33;
            #10 write=1;
                data_in=34;
            #10 write=1;
                data_in=35;
            #10 write=1;
                data_in=36;
            #10 write=1;
                data_in=37;
            #10 write=1'b1;
                data_in=38;
            #10 write=1'b1;
                data_in=39;
            #10 write=0;
                data_in=40;                                        
        end
endmodule
