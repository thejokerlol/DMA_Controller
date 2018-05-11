`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.05.2018 17:00:50
// Design Name: 
// Module Name: Data_Buffer
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


module Data_Buffer(
    clk,
    areset,
    write,
    data_in,
    read,
    data_out,
    empty,
    full,
    read_pointer,
    write_pointer

);

input clk;
input areset;
input[31:0] data_in;
input write;
input read;
output reg[31:0] data_out;
output reg empty;
output reg full;

reg[7:0] mem[0:127];//128 8 bit word for each channel

reg[2:0] read_pointer;
reg[2:0] write_pointer;
reg flip;

always@(posedge clk or negedge areset)
begin
    if(!areset)
    begin
        read_pointer=0;
        write_pointer=0;
        empty=1;
        full=0;
        
    end
    else
    begin
        if(read)
        begin
            if(!empty)
            begin
                data_out<=mem[read_pointer];
                if(read_pointer==127)
                begin
                    read_pointer<=0;
                end
                else
                begin
                    read_pointer<=read_pointer+1;
                end
            end
            
        end
    end
end

//logic for flip
always@(posedge clk or negedge areset)
begin
    if(!areset)
    begin
        flip<=0;
    end
    else
    begin
       if((read==1 && read_pointer==127) || (write==1 && write_pointer==127))
       begin
            flip<=!flip;
       end
       
       
       
    end
end
//logic for full 
always@(*)
begin
    if(flip==1)
    begin
        if(read_pointer==write_pointer)
        begin
            full=1;
        end
        else
        begin
            full=0;
        end
    end
    else
    begin
        full=0;
    end
end

//logic for empty
always@(*)
begin
    if(flip==0)
    begin
        if(read_pointer==write_pointer)
        begin
            empty=1;
        end
        else
        begin
            empty=0;
        end
    end
    else
    begin
        empty=0;
    end
end


always@(posedge clk or negedge areset)
begin
    if(!areset)
    begin
        write_pointer=0;
    end
    else
    begin
        if(write)
        begin
            if(!full)
            begin
                mem[write_pointer]<=data_in;
                if(write_pointer==127)
                begin
                    write_pointer<=0;
                end
                else
                begin
                    write_pointer<=write_pointer+1;
                end
            end
            
        end           
    end
end
endmodule
