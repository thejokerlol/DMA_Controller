`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NITK
// Engineer: Gedela Uday Kiran
// 
// Create Date: 01.03.2018 13:20:20
// Design Name: 
// Module Name: Instruction_Cache
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

/*
    Let's make a Direct mapped cache
*/
module Instruction_Cache(

    clk,
    address,
    areset,
    read,
    enable_RW,
    data_in,
    data_out,
    hit 

    );
    
    input clk;
    input[31:0] address;
    input areset;
    input read;
    input enable_RW;
    input[31:0] data_in;
    output reg[31:0] data_out;
    output wire hit; 
    
    
    /*
        Tag RAM signals
    */
    reg enable_RW_tag;
    reg read_tag;
    
    wire[31:0] data_out_tag;
    reg[31:0] data_in_tag;
    
    /*
        Data RAM signals
    */
    reg enable_RW_banks[0:31];
    reg read_banks[0:31];
    
    
    wire[7:0] data_out_banks[0:31];
    reg[7:0] data_in_banks[0:31];
    
    parameter bytes_in_cache_line=4'd32;//32 bytes
    
    tag_RAM T1(clk,address,enable_RW,read,
        {address[31:9],9'd0},
        data_out_tag,hit
        );
    
    
    always@(*)
    begin
        data_in_tag=data_in;
    end
    
    genvar bank_no;
    generate
        for(bank_no=0;bank_no<32;bank_no=bank_no+1)
        begin
            Data_RAM d1(clk,address[8:5],enable_RW_banks[bank_no],read_banks[bank_no],
                data_in_banks[bank_no],
                data_out_banks[bank_no]);
                    
        end
    endgenerate
     
    integer index;
    always@(*)
    begin
       data_out=32'd0;
       for(index=0;index<8;index=index+1)
        begin            
            if(index==address[4:2])
            begin
                data_out={data_out_banks[4*index][7:0],data_out_banks[(4*index)+1][7:0],data_out_banks[(4*index)+2][7:0],data_out_banks[(4*index)+3][7:0]};
            end
            
        end
    end
    
    
    
    integer index1;
    always@(*)
    begin
        begin
            
            for(index1=0;index1<32;index1=index1+1)
            begin
                enable_RW_banks[index1]=1'b0;
                read_banks[index1]=1'b0;
                
            end
            for(index1=0;index1<8;index1=index1+1)
            begin
                if(index1==address[4:2])
                begin
                    enable_RW_banks[4*index1]=enable_RW;
                    enable_RW_banks[(4*index1)+1]=enable_RW;
                    enable_RW_banks[(4*index1)+2]=enable_RW;
                    enable_RW_banks[(4*index1)+3]=enable_RW;
                    
                    read_banks[4*index1]=read;
                    read_banks[(4*index1)+1]=read;
                    read_banks[(4*index1)+2]=read;
                    read_banks[(4*index1)+3]=read;
                    
                    
                end
                data_in_banks[4*index1]=data_in[31:24];
                data_in_banks[(4*index1)+1]=data_in[23:16];
                data_in_banks[(4*index1)+2]=data_in[15:8];
                data_in_banks[(4*index1)+3]=data_in[7:0];
            end
            
        end
    end
    
endmodule
