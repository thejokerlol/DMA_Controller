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


module DMA_Controller(

    );
    
    /*
    AXI interface signals for the transfer
    */
    
    //Global signals
    input aclk;
    input areset;
    
    //Write Address channel
    input awready;
    output[ID_MSB:0] awid;
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
                    end
                    else
                    begin
                        case(Manager_ThreadState)
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
