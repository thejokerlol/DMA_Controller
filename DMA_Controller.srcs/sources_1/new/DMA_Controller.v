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
    reg[31:0] cache_address;
    reg cache_reset;
    reg cache_read;
    reg cache_enable_RW;
    reg[31:0] cache_data_in;
    wire[31:0] cache_data_out;
    wire cache_hit;
    
    //inbetween signals
    reg[3:0] axi_read_count;
    reg[31:0] read_data;
    
    
    //cache state
    reg[2:0] cache_state;
    
    
        
    /*
        cache instance
    */
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
         cache control state machine
    */
    reg instruction_word_no;
    reg[3:0] cache_fill_read_count;
 
    always@(posedge clk or negedge reset)
    begin
         if(!reset)
         begin
             //invalidate all entries
             cache_state=IDLE;
             cache_fill_read_count=0;
             
         end
         else
         begin
             case(cache_state)
                 IDLE:
                 begin
                     if(next_thread_to_execute==0)
                     begin
                         if(Manager_ThreadState==EXECUTING && inst_counter==0)
                          begin
                              cache_address<=DPC;
                              cache_state<=READ;
                          end
                          else if(Manager_ThreadState==EXECUTING && second_cache_access_needed==HIGH && inst_counter==1)
                          begin
                              cache_address<=cache_address+4;
                              cache_state<=READ;
                              
                          end
                          else
                          begin
                              cache_state<=IDLE;
                              
                          end
                     end
                     else
                     begin
                           if(Channel_ThreadState[next_thread_to_execute]==EXECUTING && inst_counter==0)
                           begin
                               cache_address<=CPC[next_thread_to_execute];
                               cache_state<=READ;
                           end
                           else if(Channel_ThreadState[next_thread_to_execute]==EXECUTING && second_cache_access_needed==HIGH && inst_counter==1)
                           begin
                               cache_address<=cache_address+4;
                               cache_state<=READ;
                               
                           end
                           else
                           begin
                               cache_state<=IDLE;
                               
                           end
                     end
                     
                 end
                 INVALIDATE:
                 begin
                 end
                 READ:
                 begin
                     cache_state<=WAIT_FOR_CLK_CYCLE;
                 end
                 WAIT_FOR_CLK_CYCLE:
                 begin
                     if(cache_miss==HIGH)
                     begin
                         cache_state<=WAIT_FOR_AXI_DATA;
                     end
                     else
                     begin
                         
                         cache_state<=IDLE;
                     end
                 end
                 WRITE:
                 begin
                 end
                 
                 WAIT_FOR_AXI_DATA:
                 begin
                     if(axi_state==UPDATE_CACHE && axi_read_count==0)
                     begin
                         cache_state=IDLE;
                     end
                     else
                     begin
                         cache_state=WAIT_FOR_AXI_DATA;
                     end
                 end
             endcase
         end
    end
    
    /*
         cache control state machine decoder 
    */
    always@(*)
    begin
         case(cache_state)
             IDLE:
             begin
                 cache_enable_RW=LOW;
                 cache_read=HIGH;
             end
             INVALIDATE:
             begin
             end
             READ:
             begin
                 cache_enable_RW=HIGH;
                 cache_read=HIGH;
             end
             WRITE:
             begin
                 cache_enable_RW=HIGH;
                 cache_read=LOW;
             end
             WAIT_FOR_AXI_DATA:
             begin
             end
             WAIT_FOR_CLK_CYCLE:
             begin
                 cache_enable_RW=LOW;
                 cache_read=HIGH;
             end
         endcase
    end   

        
   /*
        Read Instruction Buffer instance
   */     
      reg read_inst_write;
      reg[31:0] read_inst_data_in;
      reg read_inst_read;
      wire[31:0] read_inst_data_out;
      wire read_inst_empty;
      wire read_inst_full;
      Instruction_Buffer Read_Instr_Buffer(
       clk,
       reset,
       read_inst_write,
       read_inst_data_in,
       read_inst_read,
       read_inst_data_out,
       read_inst_empty,
       read_inst_full
   
       );
       
    /*
        Write Instruction Buffer
    */   
      reg write_inst_write;
      reg[31:0] write_inst_data_in;
      reg write_inst_read;
      wire[31:0] write_inst_data_out;
      wire write_inst_empty;
      wire write_inst_full;
      
      Instruction_Buffer Write_Instr_Buffer(
           clk,
           reset,
           write_inst_write,
           write_inst_data_in,
           write_inst_read,
           write_inst_data_out,
           write_inst_empty,
           write_inst_full
       
           ); 
         
    
    //data buffer
    reg[2:0] read_pointer_width;
    reg[2:0] read_pointer_depth;
   
    reg[2:0] write_pointer_width;
    reg[2:0] write_pointer_depth;
   
    reg[7:0] data_mem[0:7][0:7];
   
        
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

   always@(posedge clk or negedge reset)
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
                    if(cache_miss==HIGH)
                    begin
                        if(next_thread_to_execute==0)
                        begin
                            araddr<=DPC;
                            arlen<=4'b1000;
                            arsize<=3'b101;
                            arburst<=2'b10;//wrapping burst
                            axi_read_address_state<=AXI_WAITING_FOR_READY;
                        end
                        else
                        begin
                            araddr<=CPC[next_thread_to_execute];
                            arlen<=4'b1000;
                            arsize<=3'b101;
                            arburst<=2'b10;//wrapping burst
                            axi_read_address_state<=AXI_WAITING_FOR_READY;
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
                    end
                    else
                    begin
                        axi_read_address_state<=AXI_ADDRESS_IDLE;
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
                            end
                            else
                            begin
                                araddr<=CPC[next_thread_to_execute];
                                arlen<=4'b1000;
                                arsize<=3'b101;
                                arburst<=2'b10;//wrapping burst
                                axi_read_address_state<=AXI_WAITING_FOR_READY;
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
                            
                        end
                        else
                        begin
                            axi_read_address_state<=AXI_ADDRESS_IDLE;
                        end
                    end
                end       
            endcase
        end
        
   end
   always@(*)
   begin
        case(axi_read_address_state)
            AXI_ADDRESS_IDLE:
            begin
                arvalid=LOW;
            end
            AXI_WAITING_FOR_READY:
            begin
                arvalid=HIGH;
            end
        endcase
   end
   /*
        ADDRESS WRITE AXI STATES
   */
   reg[1:0] axi_write_address_state;
   /*
        write_address control
   */
   reg dma_st_req;
   reg[3:0] dma_st_channel_no;  
   
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
                    end
                    else
                    begin
                        axi_write_address_state<=AXI_ADDRESS_IDLE;
                    end
                end
                AXI_WAITING_FOR_READY:
                begin
                    if(awready==LOW)
                    begin
                        axi_write_address_state<=AXI_WAITING_FOR_READY;
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
                            end
                            else
                            begin
                                axi_write_address_state<=AXI_ADDRESS_IDLE;
                            end
                        end
                        else
                        begin
                            axi_write_address_state<=AXI_WAITING_FOR_READY;
                        end
                    end
                end
            endcase
            
        end
   end
   always@(*)
   begin
        case(axi_write_address_state)
            AXI_ADDRESS_IDLE:
            begin
                awvalid=LOW;
            end
            AXI_WAITING_FOR_READY:
            begin
                awvalid=HIGH;
            end
        endcase
   end
   
   /*
        cache controller for read data logic(since rready is to be generated here only)
   */
   reg[2:0] axi_read_state;
   parameter IDLE=3'b000;
   parameter WAIT_FOR_RVALID=3'b001;
   parameter UPDATE_CACHE_OR_MEM=3'b010;
   reg[3:0] no_reads;//counter to check how many read transfers have occured
   reg[31:0] cache_address_register;
   reg[1:0] req_type;
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            axi_read_state<=IDLE;
            no_reads<=0;
        end
        else
        begin
            case(axi_read_state)
                IDLE:
                begin
                    if(cache_miss==HIGH)
                    begin
                        axi_read_state<=WAIT_FOR_RVALID;
                        if(next_thread_to_execute==0)
                        begin
                            cache_address_register<=DPC;
                        end
                        else
                        begin
                            cache_address_register<=CPC[next_thread_to_execute];
                        end
                    end
                    else if(dma_ld_req)
                    begin
                        axi_read_state<=WAIT_FOR_RVALID;
                    end
                    else
                    begin
                        axi_read_state<=IDLE;
                    end
                end
                WAIT_FOR_RVALID:
                begin
                    if(rvalid==HIGH)//cache access
                    begin
                        if(rid==0)
                        begin
                            if(no_reads==0)
                            begin
                                cache_address<=cache_address_register;
                            end
                            else
                            begin
                                cache_address<=cache_address_register+4;
                            end
                            cache_data_in<=rdata;
                            axi_read_state<=UPDATE_CACHE;
                        end
                        else
                        begin
                            if(Channel_ThreadState[rid]==CACHE_MISS)
                            begin
                                if(no_reads==0)
                                begin
                                    cache_address<=cache_address_register;
                                end
                                else
                                begin
                                    cache_address<=cache_address_register+4;
                                end
                                cache_data_in<=rdata;
                                axi_read_state<=UPDATE_CACHE;
                            end
                            else
                            begin
                                //memory update
                                axi_read_state<=UPDATE_MEMORY;
                            end
                        end
                        
                    end
                    else
                    begin
                        axi_read_state<=WAIT_FOR_RVALID;
                    end
                end
                UPDATE_CACHE:
                begin
                    if(no_reads!=8)
                    begin
                        axi_read_state<=WAIT_FOR_RVALID;
                        no_reads<=no_reads+1;
                    end
                    else
                    begin
                        axi_read_state<=CACHE_IDLE;
                    end
                end
                UPDATE_MEMORY:
                begin
                end
            endcase
        end
   end
   always@(*)
   begin
        case(cache_state)
            CACHE_IDLE:
            begin
                cache_enable_RW=0;
                cache_read=1;
            end
            WAIT_FOR_RVALID:
            begin
                cache_enable_RW=0;
                cache_read=1;
            end
            UPDATE_CACHE:
            begin
                cache_enable_RW=1;
                cache_read=1;
            end
        endcase
   end
   /*
        memory buffer controller vvv imp
   */
   reg[1:0] dma_req;//lets say for dmald the req type is 01 and dmast the req type is 10
   reg[1:0] buffer_controller_state;
   parameter IDLE_STATE=2'b00;
   
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            
        end
        else
        begin
            if(dma_req==2'b01)//dma load executed
            begin
                
            end
            else if(dma_req==2'b10)//dma store executed
            begin
                
            end
            else//nothing executed
            begin
                
            end
        end
   end
   
   

   /*
        Track the state in each clk cycle in EXECUTING state
   */
   
   reg[1:0] inst_counter;
   
   always@(posedge clk or negedge reset)
   begin
        if(!reset)
        begin
            inst_counter=0;
        end
        else
        begin
            if(Manager_ThreadState==EXECUTING && inst_counter==0 && cache_state==WAIT_FOR_CLK_CYCLE)
            begin
                inst_counter<=inst_counter+1;//first 32 bits accessed or a cache miss
            end
            else if(Manager_ThreadState==EXECUTING && second_cache_access_needed==HIGH && inst_counter==1)
            begin
                inst_counter<=inst_counter<=inst_counter+1;//second 32 bits accessed
            end
            else if(Manager_ThreadState==UPDATING_PC)//when the PC is getting updated reset the instruction counter
            begin
                inst_counter<=0;
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
                                    DPC<={DBGINST1,DBGINST0[31:16]};
                                    Manager_ThreadState=EXECUTING;
                                end
                                else
                                begin
                                    Manager_ThreadState=STOPPED;
                                end
                            end
                            EXECUTING:
                            begin
                                if(cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)
                                begin
                                    Manager_ThreadState=CACHE_MISS;
                                end
                                else if(((cache_state==WAIT_FOR_CLK_CYCLE) && (cache_miss==LOW) && (inst_counter==0) && (second_instruction_cache_access_needed==LOW)) 
                                || ((cache_state==WAIT_FOR_CLK_CYCLE) && (cache_miss==LOW) && (inst_counter==1)))
                                begin
                                    Manager_ThreadState=UPDATING_PC;
                                end
                                else
                                begin
                                    Manager_ThreadState=EXECUTING;
                                end
                            end
                            CACHE_MISS:
                            begin
                                if((axi_read_count==0) && (axi_state==UPDATE_CACHE))
                                begin
                                    Manager_ThreadState=EXECUTING;
                                end
                                else
                                begin
                                    Manager_ThreadState=CACHE_MISS;
                                end
                            end
                            UPDATING_PC:
                            begin
                                //if the instruction is a branch go to the branched instruction or else PC=PC+4 or 8
                                
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
    reg[3:0] next_thread_to_execute;//this is taken by the next executing thread
    reg[3:0] current_executing_thread;//this is used to determine the next thread to execute
    reg execute_manager;
    integer thread_number;
    
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
                    if((Manager_ThreadState==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                ||(Manager_ThreadState==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                ||(Manager_ThreadState==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
                                //we need to include barriers,faulting,completing etc states for manager
                                )
                    begin
                        if(Channel_ThreadState[1]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[2]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[3]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[4]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[5]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[6]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[7]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else if(Channel_ThreadState[8]==EXECUTING)
                        begin
                            next_thread_to_execute=1;
                        end
                        else
                        begin
                            next_thread_to_execute=0;
                        end
                    end
                end
                4'd1:
                begin
                    if((Channel_ThreadState[1]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[1]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[1]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
                    if((Channel_ThreadState[2]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[2]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[2]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
                    if((Channel_ThreadState[3]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[3]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[3]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
                    if((Channel_ThreadState[4]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[4]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[4]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
                    if((Channel_ThreadState[5]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[5]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[5]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
                    if((Channel_ThreadState[6]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[6]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[6]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
                    if((Channel_ThreadState[7]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[7]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[7]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
                    if((Channel_ThreadState[8]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0 && second_cache_access_needed==LOW)//next state is update PC state
                                            ||(Channel_ThreadState[8]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==HIGH)//next state is cache miss
                                            ||(Channel_ThreadState[8]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1)//again an update PC state in case of 64 bit instructions
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
   reg second_cache_access_needed;
   always@(*)
   begin
        if(Manager_ThreadState==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0)
        begin
            if(dataout[15:0]==16'b00000zzz10111100 || dataout[15:0]==16'b00000zzz101000z0)//i.e. if they are DMAGO or DMAMOV instructions
            begin
                second_cache_access_needed=1;
            end
            else
            begin
                second_cache_access_needed=0;
            end
        end
        else
        begin
            second_cache_cache_access_needed=0;
        end
   end
   
    
    /*
        Execution module round robin scheme initially only manager thread no arbitration
    */
    reg[2:0] thread_count;
    reg manager_switch;
    
    
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
    
    /*
        Memory barriers executed
    */
    reg read_memory_barrier;
    reg write_memory_barrier;
    
    
    /*
        DMASEV
    */
    reg[4:0] event_number;
    reg signal_event;
    
    reg invalid_instruction;
    always@(posedge clk or negedge reset)
    begin
        if(!reset)
        begin
            thread_count=0;
            manager_switch=0;
        end
        else
        begin
           // if fetched_instruction
           if((next_thread_to_execute!=0 && Channel_ThreadState[next_thread_to_execute]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0)
                || (next_thread_to_execute==0 && Manager_ThreadState==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==0))//I doubt that the first statement is needed like Manager_ThreadState is executing
           begin
              casez(data_out)
                32'bzzzzzzzzzzzzzzzzzzzzzzzz010101z0: // the register to add an immediate is contained in [26:24], the current channel thread source and destination are to be set
                begin
                    if(data_out[1]==LOW)//add the immediate to the source register
                    begin
                        sr_no=next_thread_to_execute;
                        sr_write=1'b1;
                        sr_imm=data_out[23:8];
                    end
                    else
                    begin
                        dr_no=next_thread_to_execute;
                        dr_write=1'b1;
                        dr_imm=data_out[23:16];
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
                
                end
                32'bzzzzzzzzzzzzzzzzzzzzzzzz000001zz://DMALD[S/B]
                begin
                    notify_peripheral=1'b0;
                    if(dataout[1]==0)//S is the option
                    begin
                        if(request_flag==0)//request flag set to single
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
                        end
                    end
                end
                32'bzzzzzzzzzzzzzzzzzzzzz000001001z1://DMALDP[S/B]
                begin
                    notify_peripheral=1'b1;
                    peripheral_number=dataout[15:11];
                    if(dataout[1]==0)//S is the option
                    begin
                        if(request_flag==0)//request flag set to single
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
                        end
                    end
                end
                32'bzzzzzzzzzzzzzzzzzzzzzzzz0010000z0://DMALP
                begin
                    loop_value=dataout[15:8];
                    reg_to_use=lc;
                    write_to_loop=1'b1;
                end
                32'bzzzzzzzzzzzzzzzzzzzzzzzz001z1zzz://DMALPEND
                begin
                    dma_loop_end=1'b1;
                    backward_jump_number=dataout[15:8];
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
                    event_number=dataout[15:11];
                    signal_event=1;
                end
                32'bzzzzzzzzzzzzzzzzzzzzzzzz000010zz://DMAST[S/B]
                begin
                
                end
                32'bzzzzzzzzzzzzzzzzzzzzz000001010z1://DMASTP[S/B]
                begin
                
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
                    invalid_instruction=1;
                end
              endcase
                      
            end
            else if((next_thread_to_execute==0 && Manager_ThreadState==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1) 
                    || (next_thread_to_execute!=0 && Channel_ThreadState[next_thread_to_execute]==EXECUTING && cache_state==WAIT_FOR_CLK_CYCLE && cache_miss==LOW && instr_counter==1))
            
            begin
                casez(fetched_instruction[15:0])//64 bit instruction the first 32 bits are stored in fetched_instruction register
                    16'b00000zzz10111100://DMAGO
                    begin
                    end
                    16'b00000zzz101000z0://DMAMOV
                    begin
                    end
                    default:
                    begin
                        invalid_instruction=1;
                    end
                endcase
            end
            else//execute a nop(no operation)
            begin
                sr_write=0;
                dr_write=0;
                end_current_thread=0;
                
            end
        end
    end
    
    
endmodule
