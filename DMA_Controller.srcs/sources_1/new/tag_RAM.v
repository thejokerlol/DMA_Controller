`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03.03.2018 18:10:05
// Design Name: 
// Module Name: tag_RAM
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


module tag_RAM(clk,address,enable_RW,read,
    data_in,
    data_out,
    hit
    );
    input clk;
    input[31:0] address;
    input enable_RW;
    input read;
    input[31:0] data_in;
    output reg[31:0] data_out;
    output reg hit;
    
    reg[31:0] tag_mem[0:15];
    
    always@(posedge clk)
    begin
        if(enable_RW)
        begin
            if(read)
            begin
                data_out<=tag_mem[address[8:5]];
            end
            else
            begin
                tag_mem[address[8:5]]<=data_in;
            end
        end
    end
    always@(*)
    begin
        if(enable_RW)
        begin
            if(data_out[31:9]==address[31:9])
            begin
                hit=1'b1;
            end
            else
            begin
                hit=1'b0;
            end
        end
        else
        begin
            hit=0;
        end
        
    end
    
    
endmodule
