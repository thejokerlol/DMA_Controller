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
    aclk,
    areset,
    awready,
    awid,
    awaddr,
    awlen,
    awsize,
    awburst,
    awvalid,
    
    wready,
    wid,
    wdata,
    wstrb,
    wlast,
    wvalid,
    
    bid,
    bresp,
    bvalid,
    bready,
    
    arready,
    arid,
    araddr,
    arlen,
    arsize,
    arburst,
    arvalid,
    
    rid,
    rdata,
    rresp,
    rlast,
    rvalid,
    rready,
    
    daready,
    drlast,
    drtype,
    datype,
    davalid,
    drready,
    
    
    irq,
    irq_abort,
    
    boot_addr,
    boot_from_pc,
    boot_manager_ns,
    boot_irq_ns,
    boot_peripheral_ns,
    
    /*
        APB interface signals
    */
    paddr,
    penable,
    psel,
    pwdata,
    pwrite,
    prdata,
    pready

    );
    //parameters
    
    parameter ID_MSB=3;
    parameter DATA_MSB=64;
    parameter STRB_MSB=7;
    parameter n=3; 
    
    /*
    AXI interface signals for the transfer
    */
    
    //Global signals
    input aclk;
    input areset;
    
    //Write Address channel
    input awready;
    output reg[ID_MSB:0] awid;
    output reg[31:0] awaddr;
    output reg[3:0] awlen;
    output reg[2:0] awsize;
    output reg[1:0] awburst;
    output reg awvalid;
    
    //write channel
    input wready;
    output reg[ID_MSB:0] wid;
    output[DATA_MSB:0] wdata;//better to keep 8 bit data of different length
    output reg[STRB_MSB:0] wstrb;
    output reg wlast;
    output reg wvalid;
    
    //write response channel
    input[ID_MSB:0] bid;
    input[1:0] bresp;
    input bvalid;
    output reg bready;
    
    //Read address channel
    input arready;
    output reg[ID_MSB:0] arid;
    output reg[31:0] araddr;
    output reg[3:0] arlen;
    output reg[2:0] arsize;
    output reg[1:0] arburst;
    output reg arvalid;
    
    //Read channel
    input[ID_MSB:0] rid;
    input[DATA_MSB:0] rdata;
    input[1:0] rresp;
    input rlast;
    input rvalid;
    output reg rready;
    
    
    
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
    
    wire reset;
    wire clk;
    wire cache_miss;
    
    reg[7:0] WDATA[0:7];
    
;
    
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
        cache parameters
    */
    parameter IDLE=3'b000;//all read write and invalidate signals are low
    parameter INVALIDATE=3'b001;//invalidate signal high
    parameter READ=3'b010;//cache read
    parameter WRITE=3'b011;//cache write
    
    
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
    reg[31:0] FTC[1:8];
    
    //Channel status for DMA channels(RO)
    reg[31:0] CS[1:8];
    
    //Channel program counter registers for DMA channels(RO)
    reg[31:0] CPC[1:8];
    
    //source address registers(RO)
    reg[31:0] SA[1:8];
    
    //Destination address registers(RO)
    reg[31:0] DA[1:8];
    
    //Channel control registers(RO)
    reg[31:0] CC[1:8];
    
    //AXI status and loop counter register0(RO)
    reg[31:0] LCO1[1:8];
     
    //loop counter register1(RO)
    reg[31:0] LCO2[1:8];
    
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
    reg[1:0] inst_counter;
    
    
    //cache signals
    wire cache_clk;
    reg[31:0] cache_address;
    wire cache_reset;
    reg cache_read;
    reg cache_enable_RW;
    reg[31:0] cache_data_in;
    wire[31:0] cache_data_out;
    wire cache_hit;
    
    //inbetween signals
    reg[3:0] axi_read_count;
    reg[31:0] read_data;
    reg second_cache_access_needed;
    reg[6:0] watchdog_timer;
    
  /*
      All the control signals for execution engine
  */
  reg sr_write;
  reg dr_write;
  reg[2:0] sr_no;
  reg[15:0] sr_imm;
  reg[2:0] dr_no;
  reg[15:0] dr_imm;
  reg end_current_thread;//move the current thread to a stopped state
  
  /*
      control signals for DMAGO
  */
  reg flush_peripheral;
  reg[2:0] DMA_channel_no_to_go;
  reg[31:0] PC_value_for_go;
  /*
      control signals for DMALD and DMALDP
  */
  reg request_flag;
  reg load_data_S_B;//load data single or burst
  reg notify_peripheral;
  reg load_type;//single or burst
  reg[4:0] peripheral_number;
  /*
      control signals for DMALP and DMALPEND
  */
  reg[7:0] loop_value;
  reg reg_to_use;
  reg write_to_loop;
  reg dma_loop_end;
  reg[7:0] backward_jump_number;
  
  //for DMAKILL
  reg dma_kill;
  
  reg thread_stall;
  
  /*
        Control signals for DMAMOV
  */
  reg dma_write_register;
  reg[2:0] dma_register_type;
  reg[31:0] dma_register_value;
  
  
  /*
      Memory barriers executed
  */
  reg read_memory_barrier;
  reg write_memory_barrier;
  
  //start channel via DMAGO
  reg dma_start_channel;
  reg[3:0] dma_channel;
  
  
  /*
      DMASEV
  */
  reg[4:0] event_number;
  reg signal_event;
  
  reg invalid_instruction;
  reg inst_execution_completed;//used to stop for instructions that span multiple clock cycles
    
    
    //cache state
    reg[2:0] cache_state;
    
    //data buffer memory
    reg[7:0] channel_data_buffer[1:8][0:127]; //128 8-bit words for each channel
    reg[6:0] channel_read_pointer[1:8];//7 bit read pointer for each channel
    reg[6:0] channel_write_pointer[1:8];//7 bit write pointer for each channel
    reg[1:8] channel_buffer_empty;
    reg[1:8] channel_buffer_full;
        
    //extra signals
    assign reset=areset;
    assign clk=aclk;
    assign cache_miss=!cache_hit;
    
    
        
    //extra signals
    wire[7:0] RDATA[0:7];
    
    
    assign RDATA[0]=rdata[7:0];
    assign RDATA[1]=rdata[15:8];
    assign RDATA[2]=rdata[23:16];
    assign RDATA[3]=rdata[31:24];
    assign RDATA[4]=rdata[39:32];
    assign RDATA[5]=rdata[47:40];
    assign RDATA[6]=rdata[55:48];
    assign RDATA[7]=rdata[63:56];
       
        
    /*
        cache instance
    */
    assign cache_clk=aclk;
    assign cache_reset=areset;
    
    Instruction_Cache Instr_cache(
    
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
        Read Instruction Buffer instance
   */     
      reg read_inst_write[1:8];
      reg[31:0] read_inst_data_in[1:8];
      reg read_inst_read[1:8];
      wire[31:0] read_inst_data_out[1:8];
      wire read_inst_empty[1:8];
      wire read_inst_full[1:8];
      
      
      genvar read_buff_number;
      
      generate
        for(read_buff_number=1;read_buff_number<=8;read_buff_number=read_buff_number+1)
        begin 
          Instruction_Buffer Read_Instr_Buffer(
           clk,
           reset,
           read_inst_write[read_buff_number],
           read_inst_data_in[read_buff_number],
           read_inst_read[read_buff_number],
           read_inst_data_out[read_buff_number],
           read_inst_empty[read_buff_number],
           read_inst_full[read_buff_number]
        
           );
        end
       endgenerate
       
    /*
        Write Instruction Buffer
    */   
      reg write_inst_write[1:8];
      reg[31:0] write_inst_data_in[1:8];
      reg write_inst_read[1:8];
      wire[31:0] write_inst_data_out[1:8];
      wire write_inst_empty[1:8];
      wire write_inst_full[1:8];
      
      genvar write_buff_number;
      
      generate
        for(write_buff_number=1;write_buff_number<=8;write_buff_number=write_buff_number+1)
        begin
          Instruction_Buffer Write_Instr_Buffer(
               clk,
               reset,
               write_inst_write[write_buff_number],
               write_inst_data_in[write_buff_number],
               write_inst_read[write_buff_number],
               write_inst_data_out[write_buff_number],
               write_inst_empty[write_buff_number],
               write_inst_full[write_buff_number]
           
               ); 
        end 
      endgenerate
    
    /*
        RoundRobin Scheme signals
    */
    reg[3:0] next_thread_to_execute;//this is taken by the next executing thread
    reg[3:0] current_executing_thread;//this is used to determine the next thread to execute
    reg execute_manager;
    integer thread_number;
    
    
    //assign WDATA to different wdata values
    assign wdata[7:0]= WDATA[0];
    assign wdata[15:8]=WDATA[1];
    assign wdata[23:16]=WDATA[2];
    assign wdata[31:24]=WDATA[3];
    assign wdata[39:32]=WDATA[4];
    assign wdata[47:40]=WDATA[5];
    assign wdata[55:48]=WDATA[6];
    assign wdata[63:56]=WDATA[7];
    
   /*
        ADDRESS READ AXI STATES
   */
   reg[1:0] axi_read_address_state;
   
   /*
        ADDDRESS AXI STATES PARAMETERS
   */
   parameter AXI_ADDRESS_IDLE=2'b00;
   parameter AXI_WAITING_FOR_READY=2'b01;
   
    
   /*
       AXI  Cache update code and also DMA load requests and store requests:- starts a transaction on cache miss
   */ 
   reg dma_ld_req;
   reg[3:0] dma_ld_channel_no;

   always@(posedge aclk or negedge areset)
   begin
        if(!reset)
        begin
            axi_read_address_state<=AXI_ADDRESS_IDLE;
        end
        else
        begin
            case(axi_read_address_state)
                AXI_ADDRESS_IDLE:
                begin
                    if(axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)
                    begin
                        if(next_thread_to_execute==0)
                        begin
                            arid<=0;
                            araddr<=DPC;
                            arlen<=4'b1000;
                            arsize<=3'b101;
                            arburst<=2'b10;//wrapping burst
                            axi_read_address_state<=AXI_WAITING_FOR_READY;
                            arvalid<=HIGH;
                        end
                        else
                        begin
                            arid<=next_thread_to_execute;
                            araddr<=CPC[next_thread_to_execute];
                            arlen<=4'b1000;
                            arsize<=3'b101;
                            arburst<=2'b10;//wrapping burst
                            axi_read_address_state<=AXI_WAITING_FOR_READY;
                            arvalid<=HIGH;
                        end
                    end
                    else if(dma_ld_req==HIGH)
                    begin
                        arid<=dma_ld_channel_no;
                        araddr<=SA[dma_ld_channel_no];
                        arlen<=CC[dma_ld_channel_no][7:4];
                        arsize<=CC[dma_ld_channel_no][3:1];
                        arburst<=CC[dma_ld_channel_no][0];
                        axi_read_address_state<=AXI_WAITING_FOR_READY;     
                        arvalid<=HIGH;                  
                    end
                    else
                    begin
                        axi_read_address_state<=AXI_ADDRESS_IDLE;
                        arvalid<=LOW;
                    end
                end
                AXI_WAITING_FOR_READY:
                begin
                    if(arready==0)
                    begin
                        axi_read_address_state<=AXI_WAITING_FOR_READY;
                    end
                    else
                    begin
                        if(cache_miss==HIGH && cache_state==READ)
                        begin
                            if(next_thread_to_execute==0)
                            begin
                                araddr<=DPC;
                                arlen<=4'b1000;
                                arsize<=3'b101;
                                arburst<=2'b10;//wrapping burst
                                axi_read_address_state<=AXI_WAITING_FOR_READY;
                                arvalid<=HIGH;
                            end
                            else
                            begin
                                araddr<=CPC[next_thread_to_execute];
                                arlen<=4'b1000;
                                arsize<=3'b101;
                                arburst<=2'b10;//wrapping burst
                                axi_read_address_state<=AXI_WAITING_FOR_READY;
                                arvalid<=HIGH;
                            end
                        end
                        else if(dma_ld_req==HIGH)
                        begin
                            arid<=dma_ld_channel_no;
                            araddr<=SA[dma_ld_channel_no];
                            arlen<=CC[dma_ld_channel_no][7:4];
                            arsize<=CC[dma_ld_channel_no][3:1];
                            arburst<=CC[dma_ld_channel_no][0];
                            axi_read_address_state<=AXI_WAITING_FOR_READY;
                            arvalid<=HIGH;
                            
                        end
                        else
                        begin
                            axi_read_address_state<=AXI_ADDRESS_IDLE;
                            arvalid<=LOW;
                        end
                    end
                end       
            endcase
        end
        
   end
   /*
        ADDRESS WRITE AXI STATES
   */
   reg[1:0] axi_write_address_state;
   /*
        write_address control
   */
   reg dma_st_req;
   reg[3:0] dma_st_channel_no;  //mostly this is the next thread to execute
   
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            axi_write_address_state<=AXI_ADDRESS_IDLE;
        end
        else
        begin
            case(axi_write_address_state)
                AXI_ADDRESS_IDLE:
                begin
                    if(dma_st_req)
                    begin
                        awid<=dma_st_channel_no;
                        awaddr<=DA[dma_st_channel_no];
                        awlen<=CC[dma_st_channel_no][21:18];
                        awsize<=CC[dma_st_channel_no][17:15];
                        awburst<=CC[dma_st_channel_no][14];
                        axi_write_address_state<=AXI_WAITING_FOR_READY;
                        awvalid<=HIGH;
                    end
                    else
                    begin
                        axi_write_address_state<=AXI_ADDRESS_IDLE;
                        awvalid<=LOW;
                    end
                end
                AXI_WAITING_FOR_READY:
                begin
                    if(awready==LOW)
                    begin
                        axi_write_address_state<=AXI_WAITING_FOR_READY;
                        awvalid=LOW;
                    end
                    else
                    begin
                        if(arready==HIGH)
                        begin
                           if(dma_st_req)
                            begin
                                awid<=dma_st_channel_no;
                                awaddr<=DA[dma_st_channel_no];
                                awlen<=CC[dma_st_channel_no][21:18];
                                awsize<=CC[dma_st_channel_no][17:15];
                                awburst<=CC[dma_st_channel_no][14];
                                axi_write_address_state<=AXI_WAITING_FOR_READY;
                                awvalid<=HIGH;
                            end
                            else
                            begin
                                axi_write_address_state<=AXI_ADDRESS_IDLE;
                                awvalid<=LOW;
                            end
                        end
                        else
                        begin
                            axi_write_address_state<=AXI_WAITING_FOR_READY;
                            awvalid<=LOW;
                        end
                    end
                end
            endcase
            
        end
   end
   
      /*
        AXI write interface for dmasts(assert wvalid when necessary), drive all the write signals
   */

   reg axi_write_nos;
   parameter WRITE_IDLE=3'b000;
   parameter SEND_DATA=3'b001;
   parameter WAIT_FOR_WREADY=3'b010;
   parameter WAIT_FOR_RESPONSE=3'b011;
   reg[2:0] write_state;
   reg[2:0] completed_transfers;
   reg[31:0] write_address;
   
   integer temp_reg;
   integer temp_count;
   
   always@(posedge clk or negedge reset)
   begin
         if(!reset)
         begin
            write_state<=WRITE_IDLE;
            completed_transfers<=0;
         end
         else
         begin
            case(write_state)
                WRITE_IDLE:
                begin
                    if(dma_st_req)
                    begin
                        wstrb<=0;
                        for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                        begin
                            WDATA[temp_reg]<=0;
                        end
                        write_address<=DA[dma_st_channel_no];
                        if(CC[dma_st_channel_no][17:15]>8-(write_address%8)) //tranfer overflow for first transfer in the case of unaligned transfer
                        begin
                            temp_count=0;
                            for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                            begin
                                if(temp_reg>=write_address%8 && temp_reg<=(8-(write_address%8)))
                                begin
                                    WDATA[temp_reg]<=channel_data_buffer[dma_st_channel_no][channel_read_pointer[dma_st_channel_no]+temp_count];
                                    wstrb[temp_reg]<=1;
                                    temp_count=temp_count+1;
                                end
                                else
                                begin
                                    WDATA[temp_reg]<=0;
                                    wstrb[temp_reg]<=0;
                                end
                            end
                            wvalid=HIGH;
                            write_address<=write_address+(8-write_address%8);
                            completed_transfers<=completed_transfers+1;
                            write_state<=WAIT_FOR_WREADY;
                        end
                        else
                        begin
                            temp_count=0;
                            wstrb<=0;
                            for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                            begin
                                WDATA[temp_reg]<=0;
                            end
                            
                            for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                            begin
                                if(temp_reg>=write_address%8 && temp_reg<=((write_address%8)+CC[dma_st_channel_no][17:15]))
                                begin
                                    WDATA[temp_reg]<=channel_data_buffer[dma_st_channel_no][channel_read_pointer[dma_st_channel_no]+temp_count];
                                    wstrb[temp_reg]<=1;
                                    temp_count=temp_count+1;
                                end
                                else
                                begin
                                    WDATA[temp_reg]<=0;
                                    wstrb[temp_reg]<=0;
                                end
                            end
                            wvalid=HIGH;
                            write_address<=write_address+CC[dma_st_channel_no][17:15];
                            completed_transfers<=completed_transfers+1;
                            write_state<=WAIT_FOR_WREADY;
                        end
                        
                        
                    end
                    else
                    begin
                        write_state<=WRITE_IDLE;
                    end
                end
                WAIT_FOR_WREADY:
                begin
                    if(wready==1)
                    begin
                        completed_transfers<=completed_transfers+1;
                        if(completed_transfers==CC[dma_st_channel_no][21:18]-1)
                        begin
                            wstrb<=0;
                            for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                            begin
                                WDATA[temp_reg]<=0;
                            end
                            if(CC[dma_st_channel_no][17:15]>8-(write_address%8)) //tranfer overflow for first transfer in the case of unaligned transfer
                            begin
                                temp_count=0;
                                
                                for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                                begin
                                    if(temp_reg>=write_address%8 && temp_reg<=(8-(DA[dma_st_channel_no]%8)))
                                    begin
                                        WDATA[temp_reg]<=channel_data_buffer[dma_st_channel_no][channel_read_pointer[dma_st_channel_no]+temp_count];
                                        wstrb[temp_reg]<=1;
                                        temp_count=temp_count+1;
                                    end
                                    else
                                    begin
                                        WDATA[temp_reg]<=0;
                                        wstrb[temp_reg]<=0;
                                    end
                                end
                                wvalid<=HIGH;
                                write_state<=WAIT_FOR_WREADY;
                            end
                            else
                            begin
                                /*for(temp_reg=write_address%8;temp_reg<=CC[dma_st_channel_no][17:15];temp_reg=temp_reg+1)
                                begin
                                    wdata[7+(8*temp_reg):8*temp_reg]<=channel_data_buffer[dma_st_channel_no][channel_read_pointer[dma_st_channel_no]+temp_reg];
                                    wstrb[temp_reg]<=1;
                                end*/
                                temp_count=0;
                                for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                                begin
                                    WDATA[temp_reg]<=0;
                                end
                                
                                for(temp_reg=0;temp_reg<8;temp_reg=temp_reg+1)
                                begin
                                    if(temp_reg>=write_address%8 && temp_reg<=CC[dma_st_channel_no][17:15])
                                    begin
                                        WDATA[temp_reg]<=channel_data_buffer[dma_st_channel_no][channel_read_pointer[dma_st_channel_no]+temp_count];
                                        wstrb[temp_reg]<=1;
                                        temp_count=temp_count+1;
                                    end
                                    else
                                    begin
                                        WDATA[temp_reg]<=0;
                                        wstrb[temp_reg]<=0;
                                    end
                                end
                                
                                wvalid<=HIGH;
                                write_state<=WAIT_FOR_WREADY;
                            end
                            write_state<=WAIT_FOR_RESPONSE;
                        end
                        else
                        begin
                            write_state<=WAIT_FOR_WREADY;
                        end
                        
                    end
                    else
                    begin
                        write_state<=WAIT_FOR_WREADY;
                    end
                end
                WAIT_FOR_RESPONSE:
                begin
                    if(bvalid==1)
                    begin
                        write_state<=WAIT_FOR_RESPONSE;
                        if(bresp<=2'b00)//valid write completed
                        begin
                            DA[bid]<=write_address;  //update the destination address register
                        end
                        else//a write error an abort
                        begin
                            
                        end
                        write_address<=0;
                        completed_transfers<=0;
                        write_state<=WRITE_IDLE;
                    end
                    else
                    begin
                        write_state<=WAIT_FOR_RESPONSE; 
                    end
                end
            endcase
         end
   end 
   
   /*
        cache controller for read data logic(since rready is to be generated here only) I have to drive rready here too
   */
   reg[2:0] axi_read_state;
   parameter CACHE_IDLE=3'b000;
   parameter WAIT_FOR_RVALID=3'b001;
   parameter CHECK_FOR_CACHE_MISS=3'b010;
   parameter WAIT_FOR_CLK_CYCLE=3'b011;
   reg[3:0] no_reads[1:0];//counter to check how many read transfers have occured
   reg[31:0] cache_address_register[1:0];
   reg[1:0] req_type;
   reg[3:0] channel_waiting_for_fill[1:0];
   reg[1:0] no_of_cache_misses;
   
   
   
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            axi_read_state<=CACHE_IDLE;
            no_reads[0]<=0;
            no_reads[1]<=0;
            cache_enable_RW<=1'b0;
            no_of_cache_misses<=2'b00;
            rready=1'b1;
        end
        else
        begin
            case(axi_read_state)
                CACHE_IDLE:
                begin
                     if(next_thread_to_execute==0)
                     begin
                         if(Manager_ThreadState==EXECUTING && inst_counter==0)
                          begin
                              cache_address<=DPC;
                              cache_enable_RW<=1'b1;
                              cache_read<=1'b1;
                              rready<=1'b1;
                              axi_read_state<=WAIT_FOR_CLK_CYCLE;
                          end
                          else if(Manager_ThreadState==EXECUTING && second_cache_access_needed==HIGH && inst_counter==1)
                          begin
                              cache_address<=cache_address+4;
                              cache_enable_RW<=1'b1;
                              cache_read<=1'b1;
                              axi_read_state<=WAIT_FOR_CLK_CYCLE;
                              rready<=1'b1;
                          end
                          else
                          begin
                              rready<=1'b1;
                              cache_enable_RW=1'b0;
                              axi_read_state<=CACHE_IDLE;
                          end
                     end
                     else
                     begin
                           if(Channel_ThreadState[next_thread_to_execute]==EXECUTING && inst_counter==0)
                           begin
                               cache_address<=CPC[next_thread_to_execute];
                               cache_enable_RW<=1'b1;
                               cache_read<=1'b1;
                               rready<=1'b1;
                               axi_read_state<=WAIT_FOR_CLK_CYCLE;
                           end
                           else if(Channel_ThreadState[next_thread_to_execute]==EXECUTING && second_cache_access_needed==HIGH && inst_counter==1)
                           begin
                               cache_address<=cache_address+4;
                               cache_enable_RW<=1'b1;
                               cache_read<=1'b1;
                               rready=1'b1;
                               axi_read_state<=WAIT_FOR_CLK_CYCLE;       
                           end
                           else
                           begin
                               rready<=1'b1;
                               axi_read_state<=CACHE_IDLE; 
                           end
                     end 
                end
                WAIT_FOR_CLK_CYCLE:
                begin
                    axi_read_state<=CHECK_FOR_CACHE_MISS;
                end
                CHECK_FOR_CACHE_MISS:
                begin
                    if(cache_miss==HIGH)
                    begin
                        axi_read_state<=WAIT_FOR_RVALID;
                        no_of_cache_misses<=no_of_cache_misses+1;
                        rready=1'b1;
                        if(next_thread_to_execute==0)
                        begin
                            if(no_of_cache_misses==0)
                            begin
                                cache_address_register[0]<=DPC;
                                channel_waiting_for_fill[0]<=next_thread_to_execute;
                            end
                            else if(no_of_cache_misses==1)
                            begin
                                cache_address_register[1]<=DPC;
                                channel_waiting_for_fill[1]<=next_thread_to_execute;
                            end
                            
                        end
                        else
                        begin
                            if(no_of_cache_misses==0)
                            begin
                                cache_address_register[0]<=CPC[next_thread_to_execute];
                                channel_waiting_for_fill[0]<=next_thread_to_execute;
                                
                            end
                            else if(no_of_cache_misses==1)
                            begin
                                cache_address_register[1]<=CPC[next_thread_to_execute];
                                channel_waiting_for_fill[1]<=next_thread_to_execute;
                            end
                            else
                            begin
                            
                            end
                            
                        end
                    end
                    else
                    begin
                        if(second_cache_access_needed==HIGH)
                        begin
                            cache_address<=cache_address+4;
                            cache_enable_RW<=1'b1;
                            cache_read<=1'b1;
                            axi_read_state<=WAIT_FOR_CLK_CYCLE;
                        end
                        else
                        begin
                            if(no_of_cache_misses>1)
                            begin
                                axi_read_state<=WAIT_FOR_RVALID;
                            end
                            else
                            begin
                                axi_read_state<=CACHE_IDLE;
                            end
                        end
                    end
                end
                WAIT_FOR_RVALID://here rready is 1
                begin
                    if(rvalid==HIGH)
                    begin
                        if(rid==channel_waiting_for_fill[0] || rid==channel_waiting_for_fill[1])
                        begin
                            if(rid==channel_waiting_for_fill[0])
                            begin
                                if(no_reads[0]==0)
                                begin
                                    cache_address<=cache_address_register[0];
                                    cache_data_in<=rdata;
                                    cache_enable_RW<=1'b1;
                                    cache_read<=1'b0;//cache write
                                    no_reads[0]<=no_reads[0]+1;
                                    axi_read_state<=WAIT_FOR_RVALID;
                                end
                                else if(no_reads[0]==7)
                                begin
                                    cache_address<=cache_address_register[0]+4;
                                    cache_data_in<=rdata;
                                    cache_enable_RW<=1'b1;
                                    cache_read<=1'b0;//cache write
                                    no_reads[0]<=0;
                                    no_of_cache_misses<=no_of_cache_misses-1;
                                    axi_read_state<=CACHE_IDLE;
                                end
                                else
                                begin
                                    cache_address<=cache_address_register[0]+4;
                                    cache_address_register[0]<=cache_address_register[0]+4;
                                    cache_data_in<=rdata;
                                    cache_enable_RW<=1'b1;
                                    cache_read<=1'b0;//cache write
                                    no_reads[0]<=no_reads[0]+1;
                                    axi_read_state<=WAIT_FOR_RVALID;
                                end
                            end
                            else
                            begin
                                if(no_reads[1]==0)
                                begin
                                    cache_address<=cache_address_register[1];
                                    cache_data_in<=rdata;
                                    cache_enable_RW<=1'b1;
                                    cache_read<=1'b0;//cache write
                                    no_reads[1]<=no_reads[1]+1;
                                    axi_read_state<=WAIT_FOR_RVALID;
                                end
                                else if(no_reads[1]==7)
                                begin
                                    cache_address<=cache_address_register[1]+4;
                                    cache_address_register[1]<=cache_address_register[1]+4;
                                    cache_data_in<=rdata;
                                    cache_enable_RW<=1'b1;
                                    cache_read<=1'b0;//cache write
                                    no_reads[1]<=0;
                                    if(no_of_cache_misses==2)
                                    begin
                                        axi_read_state<=WAIT_FOR_RVALID;
                                        no_of_cache_misses<=no_of_cache_misses-1;
                                    end
                                    else
                                    begin
                                        axi_read_state<=CACHE_IDLE;
                                        no_of_cache_misses<=no_of_cache_misses-1;
                                    end
                                    
                                end
                                else
                                begin
                                    cache_address<=cache_address_register[1]+4;
                                    cache_address_register[1]<=cache_address_register[1]+4;
                                    cache_data_in<=rdata;
                                    cache_enable_RW<=1'b1;
                                    cache_read<=1'b0;       //cache write
                                    no_reads[1]<=no_reads[1]+1;
                                    axi_read_state<=WAIT_FOR_RVALID;
                                end
                            end 
                        end
                        else            // the rid is for load request
                        begin
                            for(temp_reg=0;temp_reg<=CC[rid][3:1];temp_reg=temp_reg+1)
                            begin
                                channel_data_buffer[rid][channel_write_pointer[rid]+temp_reg]<=rdata[temp_reg];
                            end
                            SA[rid]<=SA[rid]+CC[rid][3:1];
                            channel_write_pointer[rid]<=channel_write_pointer[rid]+CC[rid][3:1];
                            if(rlast)
                            begin
                                if(no_of_cache_misses>0)
                                begin
                                    axi_read_state<=WAIT_FOR_RVALID;
                                end
                                else
                                begin
                                    axi_read_state<=CACHE_IDLE;//should be changed according to the requirement
                                end
                            end
                            else
                            begin
                                axi_read_state<=WAIT_FOR_RVALID;
                            end    
                            cache_enable_RW<=1'b0;
                            cache_read<=1'b1;
                        end
                        
                    end
                    else if((next_thread_to_execute==0 && Manager_ThreadState==EXECUTING && inst_counter==0 && no_of_cache_misses<2) ||
                        (next_thread_to_execute==0 && Manager_ThreadState==EXECUTING && second_cache_access_needed==HIGH && inst_counter==1 && no_of_cache_misses<2))
                    begin
                        cache_address<=DPC;
                        cache_enable_RW<=1'b1;
                        cache_read<=1'b1;
                        axi_read_state<=WAIT_FOR_CLK_CYCLE;
                        rready<=1'b0;
                        
                    end
                    else if((Channel_ThreadState[next_thread_to_execute]==EXECUTING && inst_counter==0 && no_of_cache_misses<2) ||
                            (Channel_ThreadState[next_thread_to_execute]==EXECUTING && second_cache_access_needed==HIGH && inst_counter==1))
                    begin
                        cache_address<=CPC[next_thread_to_execute];
                        cache_enable_RW<=1'b1;
                        cache_read<=1'b1;
                        axi_read_state<=WAIT_FOR_CLK_CYCLE;
                        rready<=1'b0;
                        
                    end 
                    else
                    begin
                        axi_read_state<=WAIT_FOR_RVALID;
                        cache_enable_RW<=1'b0;
                        cache_read<=1'b0;
                        rready<=1'b1;
                    end
                end
            endcase
        end
   end
    /*
        a process for always block
    */
    always@(*)
    begin
        if(rvalid==HIGH)
        begin
            rready=1'b1;
        end
        else
        begin
        end
        
    end
    
    
    
    /*
        controller for data load in the channel buffer
    */
    reg channel_data_load_completed;
    reg[1:0] axi_channel_buffer_state;
    parameter CHANNEL_IDLE=2'b00;
    parameter CHANNEL_FILL=2'b01;
    parameter CHANNEL_WAIT=2'b10;
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            channel_data_load_completed<=0;
        end
        else
        begin
            case(dma_ld_req)
                CHANNEL_IDLE:
                begin
                end
                
            endcase
            if(rvalid==HIGH)
            begin
                if(rid!=0)
                begin
                    for(temp_reg=0;temp_reg<=CC[rid][3:1];temp_reg=temp_reg+1)
                    begin
                        channel_data_buffer[rid][channel_write_pointer[rid]+temp_reg]<=rdata[temp_reg];
                    end
                    SA[rid]<=SA[rid]+CC[rid][3:1];
                    channel_write_pointer[rid]<=channel_write_pointer[rid]+CC[rid][3:1];
                    if(rlast)
                    begin
                        channel_data_load_completed<=1;   
                    end
                    else
                    begin
                        channel_data_load_completed<=0;
                    end
                end
            end
        end
    end
    
    /*
        process to drive fetched instruction
    */
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            fetched_instruction=0;
        end
        else
        begin
            if(next_thread_to_execute==0)
            begin
                if(Manager_ThreadState==EXECUTING && inst_counter==0 && axi_read_state==CHECK_FOR_CACHE_MISS && second_cache_access_needed==1 && cache_miss==LOW)
                begin
                    fetched_instruction<=cache_data_out;
                end
            end
            else
            begin
                if(Channel_ThreadState[next_thread_to_execute]==EXECUTING && inst_counter==0 && axi_read_state==CHECK_FOR_CACHE_MISS && second_cache_access_needed==1 && cache_miss==LOW)
                begin
                    fetched_instruction<=cache_data_out;
                end
            end
        end
    end
    
    
    
    
    
    
   /*
        Track the state in each clk cycle in EXECUTING state
   */
   
   
   reg wait_for_second_access;
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            inst_counter=0;
        end
        else
        begin
            if(next_thread_to_execute==0)
            begin
                if(Manager_ThreadState==EXECUTING && inst_counter==0 && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && second_cache_access_needed==HIGH)
                begin
                    inst_counter<=inst_counter+1;//first 32 bits accessed or a cache miss
                end
                else if((Manager_ThreadState==EXECUTING && inst_counter==0 && axi_read_state==CHECK_FOR_CACHE_MISS && second_cache_access_needed==LOW)
                        || (Manager_ThreadState==EXECUTING && inst_counter==1 && axi_read_state==CHECK_FOR_CACHE_MISS))//when the PC is getting updated reset the instruction counter
                begin
                    inst_counter<=0;
                end
                else
                begin
                    inst_counter<=inst_counter;
                end
            end
            else
            begin
                if(Channel_ThreadState[next_thread_to_execute]==EXECUTING && inst_counter==0 && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW)
                begin
                    inst_counter<=inst_counter+1;//first 32 bits or a cache miss
                end
                else if((Channel_ThreadState[next_thread_to_execute]==EXECUTING && inst_counter==0 && axi_read_state==CHECK_FOR_CACHE_MISS && second_cache_access_needed==LOW)
                        || (Channel_ThreadState[next_thread_to_execute]==EXECUTING && inst_counter==1 && axi_read_state==CHECK_FOR_CACHE_MISS))//when the PC is getting updated reset the instruction counter
                begin
                    inst_counter<=0;
                end
                else
                begin
                    inst_counter<=inst_counter;
                end
            end
            
        end
   end
   

    
    /*
        state machine for each channel thread....these states are used only for inclusion or exclusion of threads while arbitration
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
                                    if(boot_from_pc==HIGH)
                                    begin
                                        DPC<=boot_addr;
                                    end
                                    else
                                    begin
                                        DPC<={DBGINST1,DBGINST0[31:16]};
                                    end
                                    
                                    Manager_ThreadState=EXECUTING;
                                end
                                else
                                begin
                                    Manager_ThreadState=STOPPED;
                                end
                            end
                            EXECUTING:
                            begin
                                if(next_thread_to_execute==0 && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)
                                begin
                                    Manager_ThreadState<=CACHE_MISS;
                                end
                                else if(((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==0) && (second_cache_access_needed==LOW) && (invalid_instruction==LOW)) 
                                                                || ((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==1) && (invalid_instruction==LOW))
                                                                || ((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==0) && (second_cache_access_needed==HIGH) && (invalid_instruction==LOW)))
                                begin
                                    if(end_current_thread==1 && next_thread_to_execute==0)
                                    begin
                                        Manager_ThreadState<=STOPPED;
                                    end
                                    else
                                    begin 
                                        Manager_ThreadState<=UPDATING_PC;
                                    end
                                end
                                else if(((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==0) && (second_cache_access_needed==LOW) && (invalid_instruction==HIGH)) 
                                                                || ((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==1) && (invalid_instruction==HIGH)))
                                begin
                                    Manager_ThreadState<=FAULTING;
                                end
                                else
                                begin
                                    Manager_ThreadState<=EXECUTING;
                                end
                            end
                            CACHE_MISS:
                            begin
                                if(axi_read_state==WAIT_FOR_RVALID && rid==0 && rvalid==1)
                                begin
                                    if(rid==channel_waiting_for_fill[0])
                                    begin
                                        if(no_reads[0]==7)
                                        begin
                                            Manager_ThreadState<=EXECUTING;
                                        end
                                        else
                                        begin
                                            Manager_ThreadState<=CACHE_MISS;
                                        end
                                    end
                                    else if(rid==channel_waiting_for_fill[1])
                                    begin
                                        if(no_reads[1]==7)
                                        begin
                                            Manager_ThreadState<=EXECUTING;
                                        end
                                        else
                                        begin
                                            Manager_ThreadState<=CACHE_MISS;
                                        end
                                        
                                    end
                                    else
                                    begin
                                        Manager_ThreadState<=CACHE_MISS;
                                    end
                                end
                                else
                                begin
                                    Manager_ThreadState<=CACHE_MISS;
                                end
                            end
                            UPDATING_PC:
                            begin
                                //if the instruction is a branch go to the branched instruction or else PC=PC+4 or 8
                                DPC<=DPC+4;
                                Manager_ThreadState<=EXECUTING;
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
                                if(DBGCMD[0]==HIGH && DBGINST0[23:16]==8'd1)//DMAKILL instruction
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
                       Channel_ThreadState[thread_no]<=STOPPED;
                    end
                    else
                    begin
                        case(Channel_ThreadState[thread_no])
                            STOPPED:
                            begin
                                if(dma_start_channel==HIGH && dma_channel==thread_no)
                                begin
                                    CPC[thread_no]<={cache_data_out[15:0],fetched_instruction[31:16]};
                                    Channel_ThreadState[thread_no]<=EXECUTING;
                                end
                                else
                                begin
                                    Channel_ThreadState[thread_no]<=STOPPED;
                                end
                            end
                            EXECUTING:
                            begin
                               if(next_thread_to_execute==thread_no)
                               begin
                                    if(thread_stall)//I am not sure if dma_ld_S_B should be there
                                    begin
                                        if(watchdog_timer==127)
                                        begin
                                            Channel_ThreadState[thread_no]<=FAULTING;
                                        end
                                        else
                                        begin
                                            Channel_ThreadState[thread_no]<=EXECUTING;
                                        end
                                    end
                                    else
                                    begin
                                        if(axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)
                                        begin
                                            Channel_ThreadState[thread_no]<=CACHE_MISS;
                                        end
                                        else if((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==0) && (second_cache_access_needed==LOW) && (invalid_instruction==LOW))
                                        begin
                                                if(load_data_S_B)//multi cycle instruction
                                                begin
                                                    Channel_ThreadState[thread_no]<=EXECUTING;
                                                end
                                                else
                                                begin
                                                    Channel_ThreadState[thread_no]<=UPDATING_PC;
                                                end     
                                        end
                                        else if((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==1) && (invalid_instruction==LOW) && (second_cache_access_needed==LOW))
                                        begin
                                            Channel_ThreadState[thread_no]<=UPDATING_PC;
                                        end
                                        else if(((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==0) && (second_cache_access_needed==LOW) && (invalid_instruction==HIGH)) 
                                                                        || ((axi_read_state==CHECK_FOR_CACHE_MISS) && (cache_miss==LOW) && (inst_counter==1) && (invalid_instruction==HIGH))
                                                                        || (thread_stall && watchdog_timer==127))
                                        begin
                                            Channel_ThreadState[thread_no]<=FAULTING;
                                        end
                                        else
                                        begin
                                            Channel_ThreadState[thread_no]<=EXECUTING;
                                        end
                                    end
                                end
                                else
                                begin
                                    Channel_ThreadState[thread_no]<=EXECUTING;
                                end 
                            end
                            CACHE_MISS:
                            begin
                                if(axi_read_state==WAIT_FOR_RVALID && rid==thread_no && rvalid==1)
                                begin
                                    if(rid==channel_waiting_for_fill[0])
                                    begin
                                        if(no_reads[0]==7)
                                        begin
                                            Channel_ThreadState[thread_no]<=EXECUTING;
                                        end
                                        else
                                        begin
                                            Channel_ThreadState[thread_no]<=CACHE_MISS;
                                        end
                                    end
                                    else if(rid==channel_waiting_for_fill[1])
                                    begin
                                        if(no_reads[1]==7)
                                        begin
                                            Channel_ThreadState[thread_no]<=EXECUTING;
                                        end
                                        else
                                        begin
                                            Channel_ThreadState[thread_no]<=CACHE_MISS;
                                        end
                                        
                                    end
                                    else
                                    begin
                                        Channel_ThreadState[thread_no]<=CACHE_MISS;
                                    end
                                end
                                else
                                begin
                                    Channel_ThreadState[thread_no]<=CACHE_MISS;
                                end
                            end
                            UPDATING_PC:
                            begin
                                CPC[thread_no]<=CPC[thread_no]+4;
                                Channel_ThreadState[thread_no]<=EXECUTING;
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
    
    //write for source address registers
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
        Roundrobin scheme
    */
    
    
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            next_thread_to_execute=0;//manager thread but in stopped state
        end
        else
        begin
            /*
                Only when any thread completes executing an instruction it should change the thread state
            */
            case(next_thread_to_execute)
                4'd0:
                begin
                    if((Manager_ThreadState==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                ||(Manager_ThreadState==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                ||(Manager_ThreadState==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                //we need to include barriers,faulting,completing etc states for manager
                                )
                    begin
                        if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd1:
                begin
                    if((Channel_ThreadState[1]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[1]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[1]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for manager
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd2:
                begin
                    if((Channel_ThreadState[2]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW  && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[2]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[2]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for manager
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd3:
                begin
                    if((Channel_ThreadState[3]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW  && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[3]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[3]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for manager
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd4:
                begin
                    if((Channel_ThreadState[4]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW  && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[4]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[4]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for manager
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd5:
                begin
                    if((Channel_ThreadState[5]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW  && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[5]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[5]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for manager
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd6:
                begin
                    if((Channel_ThreadState[6]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW  && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[6]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[6]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for manager
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd7:
                begin
                    if((Channel_ThreadState[7]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW  && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[7]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[7]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for manager
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=8;
                        end
                        else if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd8:
                begin
                    if((Channel_ThreadState[8]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==LOW  && load_data_S_B==0 && thread_stall==0)//next state is update PC state
                                            ||(Channel_ThreadState[8]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[8]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1)//again an update PC state in case of 64 bit instructions
                                            //we need to include barriers,faulting,completing etc states for each channel thread
                                            )
                    begin
                        if(Manager_ThreadState==EXECUTING)
                        begin
                            next_thread_to_execute=0;
                        end
                        else if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=2;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=3;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=4;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=5;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=6;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=7;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                default:
                begin
                    next_thread_to_execute=0;
                end
            endcase
        end    
    end
    
   integer count;
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            for(count=0;count<7;count=count+1)
            begin
                SA[count]<=32'd0;
            end
        end
        else
        begin
            if(sr_write)
            begin
                SA[next_thread_to_execute]<=SA[next_thread_to_execute]+sr_imm;
            end
        end
   end
   
   
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            for(count=0;count<7;count=count+1)
            begin
                DA[count]<=32'd0;
            end
        end
        else
        begin
            if(dr_write)
            begin
                DA[next_thread_to_execute]<=DA[next_thread_to_execute]+dr_imm;
            end
        end
   end
   
   //whether the cache needs to access a second time
   always@(*)
   begin
        if(Manager_ThreadState==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0)
        begin
            casez(cache_data_out[15:0])
                16'b00000zzz10111100://i.e. if they are DMAGO or DMAMOV instructions
                begin
                    second_cache_access_needed=1;
                end
                16'b00000zzz101000z0:
                begin
                    second_cache_access_needed=1;
                end
                default:
                begin
                    second_cache_access_needed=0;
                end
            endcase
        end
        else
        begin
            second_cache_access_needed=0;
        end
   end
   
    
    /*
        Execution module round robin scheme initially only manager thread no arbitration
    */
    reg[2:0] thread_count;
    reg manager_switch;
    
    
  
    
    always@(*)
    begin
           // if fetched_instruction
       invalid_instruction=LOW;
       if(dma_ld_req==1 || thread_stall==1)
       begin
            //no op
            sr_write=0;
            dr_write=0;
            end_current_thread=0;
            dma_start_channel=LOW;
            dma_channel=0;
            invalid_instruction=LOW;
            load_data_S_B=LOW;
       end 
       else if((next_thread_to_execute!=0 && Channel_ThreadState[next_thread_to_execute]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0)
            || (next_thread_to_execute==0 && Manager_ThreadState==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==0 && second_cache_access_needed==0))//I doubt that the first statement is needed like Manager_ThreadState is executing
       begin
            
          casez(cache_data_out)
            32'bzzzzzzzzzzzzzzzzzzzzzzzz010101z0: // the register to add an immediate is contained in [26:24], the current channel thread source and destination are to be set
            begin
                if(cache_data_out[1]==LOW)//add the immediate to the source register
                begin
                    sr_no=next_thread_to_execute;
                    sr_write=1'b1;
                    sr_imm=cache_data_out[23:8];
                end
                else
                begin
                    dr_no=next_thread_to_execute;
                    dr_write=1'b1;
                    dr_imm=cache_data_out[23:16];
                end
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz00000000://DMAEND
            begin
                end_current_thread=1;//move the channel to a stoopped state
            end
            32'bzzzzzzzzzzzzzzzzzzzzz00000110101://DMAFLUSHP
            begin
                
            end
            32'bzzzzzzzzzzzzzzzz00000zzz101000z0://DMAGO
            begin
                $display("EXECUTING DMA GO");
                $display("The address is %d ",cache_data_out);
                invalid_instruction=LOW;
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz000001zz://DMALD[S/B]
            begin
                notify_peripheral=1'b0;
                if(cache_data_out[1]==0)//S is the option
                begin
                    if(request_flag==0)//request flag set to single,write into the read queue
                    begin
                        load_data_S_B=1'b1;
                        load_type=1'b0;//single
                        
                    end
                    else//request flag set to burst
                    begin
                        load_data_S_B=1'b0;//no operation
                        load_type=1'b0;
                    end
                end
                else
                begin
                    if(request_flag==0)//request flag set to single
                    begin
                        load_data_S_B=1'b0;//no operation
                        load_type=1'b0;
                    end
                    else//request flag set to burst
                    begin
                        load_data_S_B=1'b1;
                        load_type=1'b1;
                        if(read_inst_full[next_thread_to_execute]!=1)
                        begin
                            read_inst_read[next_thread_to_execute]=1;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                        end
                        else
                        begin
                            //thread stalled in executing state
                            read_inst_read[next_thread_to_execute]=0;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                            
                        end
                    end
                end
            end
            32'bzzzzzzzzzzzzzzzzzzzzz000001001z1://DMALDP[S/B]
            begin
                notify_peripheral=1'b1;
                peripheral_number=cache_data_out[15:11];
                if(cache_data_out[1]==0)//S is the option
                begin
                    if(request_flag==0)//request flag set to single
                    begin
                        load_data_S_B=1'b1;
                        load_type=1'b0;//single
                        if(read_inst_full[next_thread_to_execute]!=1)
                        begin
                            read_inst_read[next_thread_to_execute]=1;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                        end
                        else
                        begin
                            //thread stalled in executing state
                            read_inst_read[next_thread_to_execute]=0;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                            
                        end
                    end
                    else//request flag set to burst
                    begin
                        load_data_S_B=1'b0;//no operation
                        load_type=1'b0;
                        if(read_inst_full[next_thread_to_execute]!=1)
                        begin
                            read_inst_read[next_thread_to_execute]=1;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                        end
                        else
                        begin
                            //thread stalled in executing state
                            read_inst_read[next_thread_to_execute]=0;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                            
                        end
                    end
                end
                else
                begin
                    if(request_flag==0)//request flag set to single
                    begin
                        load_data_S_B=1'b0;//no operation
                        load_type=1'b0;
                    end
                    else//request flag set to burst
                    begin
                        load_data_S_B=1'b1;
                        load_type=1'b1;
                        if(read_inst_full[next_thread_to_execute]!=1)
                        begin
                            read_inst_read[next_thread_to_execute]=1;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                        end
                        else
                        begin
                            //thread stalled in executing state
                            read_inst_read[next_thread_to_execute]=0;
                            read_inst_data_in[next_thread_to_execute]=32'b00000000000000000000000000000100;
                            
                        end
                    end
                end
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz0010000z0://DMALP
            begin
                loop_value=cache_data_out[15:8];
                reg_to_use=0;
                write_to_loop=1'b1;
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz001z1zzz://DMALPEND
            begin
                dma_loop_end=1'b1;
                backward_jump_number=cache_data_out[15:8];
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz00000001://DMAKILL
            begin
                dma_kill=1'b1;
            end
            32'bzzzzzzzzzzzzzzzz00000zzz10111100://DMAMOV
            begin
                
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz00011000://DMANOP
            begin
                
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz00010010://DMARMB
            begin
                read_memory_barrier=1;
            end
            32'bzzzzzzzzzzzzzzzzzzzzz00000110100://DMASEV
            begin
                event_number=cache_data_out[15:11];
                signal_event=1;
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz000010zz://DMAST[S/B]
            begin
                notify_peripheral=1'b0;
                if(cache_data_out[1]==0)//S is the option
                begin
                    if(request_flag==0)//request flag set to single,write into the read queue
                    begin
                        load_data_S_B=1'b1;
                        load_type=1'b0;//single
                        //write into the read instruction queue
                        if(read_inst_full[next_thread_to_execute]!=1)
                        begin
                            read_inst_read[next_thread_to_execute]=1;
                            read_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzzzzz000010zz;
                        end
                        else
                        begin
                            //thread stalled in executing state
                            read_inst_read[next_thread_to_execute]=0;
                            read_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzzzzz000010zz;
                            
                        end
                    end
                    else//request flag set to burst
                    begin
                        load_data_S_B=1'b0;//no operation
                        load_type=1'b0;
                    end
                end
                else
                begin
                    if(request_flag==0)//request flag set to single
                    begin
                        load_data_S_B=1'b0;//no operation
                        load_type=1'b0;
                    end
                    else//request flag set to burst
                    begin
                        load_data_S_B=1'b1;
                        load_type=1'b1;
                        if(read_inst_full[next_thread_to_execute]!=1)
                        begin
                            write_inst_read[next_thread_to_execute]=1;
                            write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzzzzz000010zz;
                        end
                        else
                        begin
                            //thread stalled in executing state
                            write_inst_read[next_thread_to_execute]=0;
                            write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzzzzz000010zz;
                            
                        end
                    end
                end
                
            
            end
            32'bzzzzzzzzzzzzzzzzzzzzz000001010z1://DMASTP[S/B]
            begin
               notify_peripheral=1'b1;
               peripheral_number=cache_data_out[15:11];
               if(cache_data_out[1]==0)//S is the option
               begin
                   if(request_flag==0)//request flag set to single
                   begin
                       load_data_S_B=1'b1;
                       load_type=1'b0;//single
                       if(write_inst_full[next_thread_to_execute]!=1)
                       begin
                           write_inst_read[next_thread_to_execute]=1;
                           write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzz000001010z1;
                       end
                       else
                       begin
                           //thread stalled in executing state
                           write_inst_read[next_thread_to_execute]=0;
                           write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzz000001010z1;
                           
                       end
                   end
                   else//request flag set to burst
                   begin
                       load_data_S_B=1'b0;//no operation
                       load_type=1'b0;
                       if(read_inst_full[next_thread_to_execute]!=1)
                       begin
                           write_inst_read[next_thread_to_execute]=1;
                           write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzz000001010z1;
                       end
                       else
                       begin
                           //thread stalled in executing state
                           write_inst_read[next_thread_to_execute]=0;
                           write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzz000001010z1;
                           
                       end
                   end
               end
               else
               begin
                   if(request_flag==0)//request flag set to single
                   begin
                       load_data_S_B=1'b0;//no operation
                       load_type=1'b0;
                   end
                   else//request flag set to burst
                   begin
                       load_data_S_B=1'b1;
                       load_type=1'b1;
                       if(write_inst_full[next_thread_to_execute]!=1)
                       begin
                           write_inst_read[next_thread_to_execute]=1;
                           write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzz000001010z1;
                           
                       end
                       else
                       begin
                           //thread stalled in executing state
                           write_inst_read[next_thread_to_execute]=0;
                           write_inst_data_in[next_thread_to_execute]=32'bzzzzzzzzzzzzzzzzzzzzz000001010z1;
                           
                       end
                   end
               end
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz00001100://DMASTZ
            begin
            
            end
            32'bzzzzzzzzzzzzzzzzzzzzz0z000110110://DMAWFE
            begin
            
            end
            32'bzzzzzzzzzzzzzzzzzzzzz000001100zz://DMAWFP[S/B/P]
            begin
            
            end
            32'bzzzzzzzzzzzzzzzzzzzzzzzz00010011://DMAWMB
            begin
                
            end
            default:
            begin
                invalid_instruction=HIGH;
                $display("The opcode for the instruction is %d", cache_data_out);//if it is undefined i.e., x also then this will be printed
            end
          endcase
                  
        end
        else if((next_thread_to_execute==0 && Manager_ThreadState==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1) 
                || (next_thread_to_execute!=0 && Channel_ThreadState[next_thread_to_execute]==EXECUTING && axi_read_state==CHECK_FOR_CACHE_MISS && cache_miss==LOW && inst_counter==1))
        
        begin
            casez(fetched_instruction[15:0])//64 bit instruction the first 32 bits are stored in fetched_instruction register
                16'b00000zzz101000z0://DMAGO
                begin
                    dma_start_channel=HIGH;
                    dma_channel=fetched_instruction[10:8];
                end
                16'b00000zzz10111100://DMAMOV
                begin
                    dma_write_register=1;
                    dma_register_type=fetched_instruction[10:8];
                    dma_register_value={cache_data_out[15:0],fetched_instruction[31:16]};
                end
                default:
                begin
                    invalid_instruction=1;//an abort
                   // $display("An Abort has occurred");
                end
            endcase
        end
        else//execute a nop(no operation)
        begin
            sr_write=0;
            dr_write=0;
            end_current_thread=0;
            dma_start_channel=LOW;
            dma_channel=0;
            invalid_instruction=LOW;
            load_data_S_B=LOW;
            
        end
    end
    
    /*
            WatchDog timer
    */
    
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            watchdog_timer<=0;
        end
        else
        begin
            if(thread_stall)
            begin
                watchdog_timer<=watchdog_timer+1;
            end
            else
            begin
                watchdog_timer<=0;
            end
        end
    end
    
    /*
        Moving values into the SA,DA,CC registers
    */
    integer i;
    
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            for(i=1;i<=8;i=i+1)
            begin
                SA[i]<=0;
                CC[i]<=0;
                DA[i]<=0;
            end
        end
        else
        begin
            if(dma_write_register)
            begin
                if(dma_register_type==3'b000)// source address register
                begin
                    SA[next_thread_to_execute]<=dma_register_value;
                end
                else if(dma_register_type==3'b001)// channel control register
                begin
                    CC[next_thread_to_execute]<=dma_register_value;
                end
                else if(dma_register_type==3'b010)// destination address register
                begin
                    DA[next_thread_to_execute]<=dma_register_value;
                end
            end
           
        end
    end
    
    /*
        Process to perform DMALD ....writes one instruction into the read queue ....tries to execute an instruction from read queue and one from write queue
    */
    reg[1:0] read_state;
    parameter LOAD_IDLE=2'b00;
    parameter WAIT_FOR_ADDRESS_TRANSFER=2'B10; 
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            read_state<=LOAD_IDLE;
        end
        else
        begin
            case(read_state)
                LOAD_IDLE:
                begin
                    if(load_data_S_B)
                    begin
                        dma_ld_req<=1;
                        read_state<=WAIT_FOR_ADDRESS_TRANSFER;
                    end
                    else
                    begin
                        read_state<=LOAD_IDLE;
                    end
                end
                WAIT_FOR_ADDRESS_TRANSFER:
                begin
                    if(axi_read_address_state==AXI_ADDRESS_IDLE)
                    begin
                        dma_ld_req=0;
                        read_state<=LOAD_IDLE;
                    end
                    else
                    begin
                        dma_ld_req=1;
                        read_state<=WAIT_FOR_ADDRESS_TRANSFER;
                    end
                end
                
            endcase
        end
    end
    /*
        Process to perform DMAST ....writes one instruction into the write queue....tries to execute an instruction from read queue and one from write queue 
    */
    
    /*
        Process for Thread Stall
    */
    reg Thread_Stall_state;
    parameter THREAD_STALL_IDLE=0;
    parameter WAIT_FOR_CONDITION=1;
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            thread_stall<=0;
        end
        else
        begin
            case(Thread_Stall_state)
                THREAD_STALL_IDLE:
                begin
                    if(load_data_S_B)
                    begin
                        thread_stall<=1;
                        Thread_Stall_state<=WAIT_FOR_CONDITION;
                    end
                    else
                    begin
                        thread_stall<=0;
                        Thread_Stall_state<=THREAD_STALL_IDLE;
                    end
                end
                WAIT_FOR_CONDITION:
                begin
                    if(axi_read_state==WAIT_FOR_RVALID && rlast==1 && rvalid==1 && rid==next_thread_to_execute)
                    begin
                        thread_stall<=0;
                        Thread_Stall_state<=THREAD_STALL_IDLE;
                    end
                    else
                    begin
                        thread_stall<=1;
                        Thread_Stall_state<=WAIT_FOR_CONDITION;
                    end
                end
            endcase    
        end
    end
    
    
endmodule
