`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: NITK
// Engineer: Gedela Uday Kiran
// 
// Create Date: 27.02.2018 11:29:33
// Design Name: 
// Module Name: DMA_Controller
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

//cache line length of 32 bytes
module DMA_Controller(

    );
    //parameters
    
    parameter ID_MSB=3;
    parameter DATA_MSB=31;
    parameter STRB_MSB=3;
    parameter n=3; 
    
    /*
    AXI interface signals for the transfer
    */
    
    //Global signals
    input aclk;
    input areset;
    
    //Write Address channel
    input awready;
    output[ID_MSB:0] awid;
    output[31:0] awaddr;
    output[3:0] awlen;
    output[2:0] awsize;
    output[1:0] awburst;
    output awvalid;
    
    //write channel
    input wready;
    output[ID_MSB:0] wid;
    output[DATA_MSB:0] wdata;
    output[STRB_MSB:0] wstrb;
    output wlast;
    output wvalid;
    
    //write response channel
    input[ID_MSB:0] bid;
    input[1:0] bresp;
    input bvalid;
    output bready;
    
    //Read address channel
    input arready;
    output[ID_MSB:0] arid;
    output[31:0] araddr;
    output[3:0] arlen;
    output[2:0] arsize;
    output[1:0] arburst;
    output arvalid;
    
    //Read channel
    input[ID_MSB:0] rid;
    input[DATA_MSB:0] rdata;
    input[1:0] rresp;
    input rlast;
    input rvalid;
    output rready;
    
    
    
    /*
        peripheral request interface(currently only 1) May be use an array of signals to configure it correctly
    */
    
    input daready;
    input drlast;
    input[1:0] drtype;
    output[1:0] datype;
    output davalid;
    output drready;
    
    /*
        interrupt interface, n should be configurable
    */
    
    output[n:0] irq;
    output irq_abort;
    
    
    
    /*
        Reset initialization interface
    */
    
    input[31:0] boot_addr;
    input boot_from_pc;
    input boot_manager_ns;
    input[n:0] boot_irq_ns;
    input[n:0] boot_peripheral_ns;
    
    /*
        APB interface signals
    */
    input[31:0] paddr;
    input penable;
    input psel;
    input[31:0] pwdata;
    input pwrite;
    output[31:0] prdata;
    output pready;
    
    /*
        state values of the threads
    */
    
    parameter STOPPED=4'b0000;
    parameter EXECUTING=4'b0001;
    parameter CACHE_MISS=4'b0010;
    parameter UPDATING_PC=4'b0011;
    parameter WAITING_FOR_EVENT=4'b0100;
    parameter AT_BARRIER=4'b0101;
    parameter WAITING_FOR_PERIPHERAL=4'b0110;
    parameter FAULTING_COMPLETING=4'b0111;
    parameter FAULTING=4'b1000;
    parameter KILLING=4'b1001;
    parameter COMPLETING=4'b1010;
    
    
    parameter HIGH=1'b1;
    parameter LOW=1'b0;
    
    /*
        AXI parameters
    */
    
    parameter AXI_IDLE=4'b0000;
    
    
    
    
    
    
    /*
        AXI present state
    */
    reg[3:0] axi_state;
    
    /*
        Configuration Registers for DMA
    */
    
    reg[31:0] DS;//DMA status register(RO)
    reg[31:0] DPC;//DMA Program Counter for manager thread(RO)
    reg[31:0] INTEN;//Interrupt Enable Register(RW)
    reg[31:0] ES;//Event Status Register(RO)
    reg[31:0] INTSTATUS;//Interrupt Status Register(RO)
    reg[31:0] INTCLR;//Interrupt Clear Register(WO)
    reg[31:0] FSM;//Fault status DMA manager register(RO)
    reg[31:0] FSC;//Fault status DMA channel register(RO)
    reg[31:0] FTM;//Fault type DMA manager register(RO)
    
    //Fault type register for DMA channels(RO)
    reg[31:0] FTC[0:7];
    
    //Channel status for DMA channels(RO)
    reg[31:0] CS[0:7];
    
    //Channel program counter registers for DMA channels(RO)
    reg[31:0] CPC[0:7];
    
    //source address registers(RO)
    reg[31:0] SA[0:7];
    
    //Destination address registers(RO)
    reg[31:0] DA[0:7];
    
    //Channel control registers(RO)
    reg[31:0] CC[0:7];
    
    //AXI status and loop counter register0(RO)
    reg[31:0] LCO1[0:7];
     
    //loop counter register1(RO)
    reg[31:0] LCO2[0:7];
    
    /*
        DMAC Debug register
    */
    
    reg[31:0] DBGSTATUS;//Debug status register(RO)
    reg[31:0] DBGCMD;//Debug command register(WO)
    reg[31:0] DBGINST0;//Debug Instruction 0 register(WO)
    reg[31:0] DBGINST1;//Debug Instruction 1 regisster(WO)
    
    /*
        DMAC Configuration register
    */
    
    reg[31:0] CR0;//configuration register 0
    reg[31:0] CR1;//configuration register 1
    reg[31:0] CR2;//configuration register 2
    reg[31:0] CR3;//configuration register 3
    reg[31:0] CR4;//configuration register 4
    reg[31:0] CR5;//configuration register 5
    
    
    reg[3:0] Manager_ThreadState;
    reg[3:0] Channel_ThreadState[1:8];
    
    /*
        Temporary Register
    */
    reg[63:0] fetched_instruction;
    reg instruction_cache_miss;
    
    
    //cache signals
    reg cache_clk;
    reg[31:0] cache_addess;
    reg cache_reset;
    reg cache_read;
    reg cache_enable_RW;
    reg[31:0] cache_data_in;
    wire[31:0] cache_data_out;
    wire cache_hit;
    
    //inbetween signals
    reg[3:0] axi_read_count;
    reg[31:0] read_data;
    /*
        cache instance
    */
    Instruction_Cache I1(
    
        cache_clk,
        cache_address,
        cache_reset,
        cache_read,
        cache_enable_RW,
        cache_data_in,
        cache_data_out,
        cache_hit 
    
        );
    
    
    
    /*
        DBGCMD write operation
    */
    
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            
        end
        else
        begin
            
            
        end
   end
    
   /*
       AXI  Cache update code:- starts a transaction on cache miss
   */ 
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            axi_state=IDLE;
            axi_read_count=0;
        end
        else
        begin
            case(axi_state)
                IDLE:
                begin
                    if(cache_miss)
                    begin
                        //all address signals
                        araddr<=DPC;
                        arlen<=4'b1000;//32 bytes so burst length is 8
                        arsize<=3'b101;//5 i.e., 2^5=32
                        arburst<=2'b01;//incremental burst for cache update
                        axi_read_count<=arlen;
                        axi_state<=READ_ADDRESS_ON_BUS;
                    end
                    else
                    begin
                        axi_state<=IDLE;
                    end
                end
                READ_ADDRESS_ON_BUS:
                begin
                    if(arready==HIGH)
                    begin
                        axi_state<=READY_TO_RECEIVE_DATA;
                    end
                    else
                    begin
                        axi_state<=READ_ADDRESS_ON_BUS;
                    end
                end
                READY_TO_RECEIVE_DATA:
                begin 
                    if(rvalid==HIGH)
                    begin
                        if(axi_read_count==4'b1000)
                        begin
                            cache_address<=DPC;
                        end
                        else
                        begin
                            cache_address<=cache_address+4;
                        end
                        cache_data<=rdata;
                        axi_read_count<=axi_read_count-1;
                        axi_state<=UPDATE_CACHE;
                    end
                    else
                    begin
                       axi_state<=READY_TO_RECEIVE_DATA; 
                    end
                end
                UPDATE_CACHE:
                begin
                    if(axi_read_count!=0)
                    begin
                        axi_state<=READY_TO_RECEIVE_DATA;
                    end
                    else
                    begin
                        axi_state<=IDLE;
                    end  
                end
            endcase
        end
   end
   
   //state decoder for axi master
   always@(*)
   begin
        case(axi_state)
            IDLE:
            begin
                arvalid=LOW;
                awvalid=LOW;
                rready=LOW;
                wvalid=LOW;
                bready=LOW;
                
                //cache signals
                cache_read=LOW;
                cache_enable_RW=LOW;
            end
            READ_ADDRESS_ON_BUS:
            begin
                arvalid=HIGH;
                awvalid=LOW;
                rready=LOW;
                wvalid=LOW;
                bready=LOW;
                
                //cache signals
                cache_read=LOW;
                cache_enable_RW=LOW;
            end
            READY_TO_RECEIVE_DATA:
            begin
                arvalid=LOW;
                awvalid=LOW;
                rready=HIGH;
                wvalid=LOW;
                bready=LOW;
                
                //cache signals
                cache_read=LOW;
                cache_enable_RW=LOW;
            end
            UPDATE_CACHE:
            begin
                arvalid=LOW;
                awvalid=LOW;
                rready=LOW;
                wvalid=LOW;
                bready=LOW;
                
                //cache signals
                cache_read=LOW;
                cache_enable_RW=HIGH;
            end
        endcase
   end 
    
    /*
        state machine for each channel thread
    */
    genvar thread_no;
    generate
        for(thread_no=0;thread_no<9;thread_no=thread_no+1)
        begin
            if(thread_no==0)
            begin
                always@(posedge clk or negedge reset)
                begin
                    if(!reset)
                    begin
                        Manager_ThreadState=STOPPED;
                    end
                    else
                    begin
                        case(Manager_ThreadState)
                            STOPPED:
                            begin
                                if((boot_from_pc==HIGH) || (DBGCMD[0]==HIGH))
                                begin
                                    Manager_ThreadState=EXECUTING;
                                end
                                else
                                begin
                                    Manager_ThreadState=STOPPED;
                                end
                            end
                            EXECUTING:
                            begin
                                if(cache_miss==HIGH)
                                begin
                                    Manager_ThreadState=CACHE_MISS;
                                end
                                else
                                begin
                                    Manager_ThreadState=EXECUTING;
                                end
                            end
                            CACHE_MISS:
                            begin
                            `  
                            end
                            UPDATING_PC:
                            begin
                            
                            end
                            WAITING_FOR_EVENT:
                            begin
                                if(fetched_instruction[10:0]==11'b00000110100)//DMASEV
                                begin
                                    Manager_ThreadState=EXECUTING;
                                end
                                else
                                begin
                                    Manager_ThreadState=WAITING_FOR_EVENT;
                                end
                            end
                            FAULTING:
                            begin
                                if(DMACMD[0]==HIGH && DMAINST0[23:16]==8'd1)//DMAKILL instruction
                                begin
                                    Manager_ThreadState=STOPPED;
                                end
                                else
                                begin
                                    Manager_ThreadState=FAULTING;
                                end
                            end
                        endcase
                    end
                end
            end
            else
            begin
                always@(posedge clk or negedge reset)
                begin
                    if(!reset)
                    begin
                    end
                    else
                    begin
                        case(Channel_ThreadState[thread_no])
                            STOPPED:
                            begin
                            end
                            EXECUTING:
                            begin
                            end
                            CACHE_MISS:
                            begin
                            end
                            UPDATING_PC:
                            begin
                            end
                            WAITING_FOR_EVENT:
                            begin
                            end
                            AT_BARRIER:
                            begin
                            end
                            WAITING_FOR_PERIPHERAL:
                            begin
                            end
                            FAULTING_COMPLETING:
                            begin
                            end
                            FAULTING:
                            begin
                            end
                            KILLING:
                            begin
                            end
                            COMPLETING:
                            begin
                            end
                        endcase
                    end 
                end
            end
        end
    endgenerate
    
    
    
    
endmodule
