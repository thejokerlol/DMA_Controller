`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.03.2018 15:14:39
// Design Name: 
// Module Name: DMA_Controller_tb
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


module DMA_Controller_tb(

    );
    
    parameter ID_MSB=3;
    parameter DATA_MSB=64;
    parameter STRB_MSB=3;
    parameter n=3; 
    
    
    //Global signals
    reg aclk;
    reg areset;
    
    //Write Address channel
    reg awready;
    wire[ID_MSB:0] awid;
    wire[31:0] awaddr;
    wire[3:0] awlen;
    wire[2:0] awsize;
    wire[1:0] awburst;
    wire awvalid;
    
    //write channel
    reg wready;
    wire[ID_MSB:0] wid;
    wire[DATA_MSB:0] wdata;//better to keep 8 bit data of different length
    wire[STRB_MSB:0] wstrb;
    wire wlast;
    wire wvalid;
    
    //write response channel
    reg[ID_MSB:0] bid;
    reg[1:0] bresp;
    reg bvalid;
    wire bready;
    
    //Read address channel
    reg arready;
    wire[ID_MSB:0] arid;
    wire[31:0] araddr;
    wire[3:0] arlen;
    wire[2:0] arsize;
    wire[1:0] arburst;
    wire arvalid;
    
    //Read channel
    reg[ID_MSB:0] rid;
    reg[DATA_MSB:0] rdata;
    reg[1:0] rresp;
    reg rlast;
    reg rvalid;
    wire rready;
    
    
    
    /*
        peripheral request interface(currently only 1) May be use an array of signals to configure it correctly
    */
    
    reg daready;
    reg drlast;
    reg[1:0] drtype;
    wire[1:0] datype;
    wire davalid;
    wire drready;
    
    /*
        interrupt interface, n should be configurable
    */
    
    wire[n:0] irq;
    wire irq_abort;
    
    
    
    /*
        Reset initialization interface
    */
    
    reg[31:0] boot_addr;
    reg boot_from_pc;
    reg boot_manager_ns;
    reg[n:0] boot_irq_ns;
    reg[n:0] boot_peripheral_ns;
    
    /*
        APB interface signals
    */
    reg[31:0] paddr;
    reg penable;
    reg psel;
    reg[31:0] pwdata;
    reg pwrite;
    wire[31:0] prdata;
    wire pready;
    
    
    
    DMA_Controller D_controller(
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
        
        initial
        begin
                    //Global signals
            aclk=0;
            areset=1;
            
            //Write Address channel
            awready=0;
            
            
            //write channel
            wready=0;
            
            
            //write response channel
            bid=0;
            bresp=0;
            bvalid=0;
            
            
            //Read address channel
            arready=0;
            
            
            //Read channel
            rid=0;
            rdata=0;
            rresp=0;
            rlast=0;
            rvalid=0;
            
            /*
                peripheral request interface(currently only 1) May be use an array of signals to configure it correctly
            */
            
            daready=0;
            drlast=0;
            drtype=0;
            /*
                interrupt interface, n should be configurable
            */
            
            
            
            
            
            /*
                Reset initialization interface
            */
            
            boot_addr=0;
            boot_from_pc=0;
            boot_manager_ns=0;
            boot_irq_ns=0;
            boot_peripheral_ns=0;
            
            /*
                APB interface signals
            */
            paddr=0;
            penable=0;
            psel=0;
            pwdata=0;
            pwrite=0;
            
        end
        
        always
            #20 aclk=!aclk;
            
        
        initial
        begin
            #10000 $finish;
        end
        
        
        initial
        begin
            #4 areset=0;
            #8 areset=1;
            
            
            #60 boot_from_pc<=1;
                boot_addr<=32'd16;
                
            #40 boot_from_pc=0;
            #120 arready=1;
            #40 arready=0;
            
            /*
                cache update signals and data , give data to start 3 DMA channels and execute a DMAEND
            */
            #40 rid=0;
                rdata=32'h000001A0;//DMAGO for channel 1, decimal value 416(just for check)
                rvalid=1;
                
            #40 rvalid=0;
            #40 rid=0;
                rdata=32'h00000100;//PC for channel 1,decimal value for 256(just for check)
                rvalid=1;
            #40 rvalid=0;
            #80 rid=0;
                rdata=32'h004002A0;//DMAGO for channel 2
                rvalid=1;
                
            #40 rvalid=0;
            
            #120 rid=0;
                rdata=32'h00001000;//PC for channel 2
                rvalid=1;
                
            #40 rvalid=0;
            
            #40 rid=0;
                rdata=32'h008003A0;//DMAGO for channel 3
                rvalid=1;
                
            #40 rvalid=0;
            
            #40 rid=0;
                rdata=32'h00000010;//PC for channel 3
                rvalid=1;
                 
            #40 rid=0;
                rdata=0;//DMAEND
                rvalid=1;
                
            #40 rid=0;
                rdata=92;
                rvalid=1;
                
            #80 rvalid=0;  
            
            #480
            #40 arready=1;
            #40 arready=0;
            
            
            /*
                rvalid data for the first channel
            */
            #280    
            #40 rid<=1;
                rdata<=32'h000000BC;//DMAMOV for source address register
                rvalid<=1'b1;
            
            #40 rvalid<=1'b0;
            
            #40 rdata<=32'h00000800;
                rvalid<=1'b1;
                
            #40 rvalid<=1'b0;    
            
            #40 rdata<=32'h000001BC;//DMAMOV for destination address register
                rvalid<=1'b1;
                
            #40 rvalid<=1'b0;
            
            #40 rdata<=32'h0000000E;
                rvalid<=1'b1;
            
                
            #40 rvalid<=1'b0;
            
            #40 rdata<=32'h403902BC;//DMAMOV for channel control register
                rvalid<=1'b1;
                
            #40 rvalid<=1'b0;
            
            #40 rvalid<=1'b1;
                rdata<=32'h0000000E;
                
            #40 rvalid<=1'b0;
            
            #40 rdata<=32'h00000007;
                rvalid<=1'b1;
            
            #40 rvalid<=1'b0;
                
            #40 rdata<=32'h00000007;
                rvalid<=1'b1;
                
            #40 rvalid<=1'b0;
            
            
            //In the next cache access
                
            /*#40 rvalid<=1'b1;
                rdata<=32'h0000000B;
                
            #40 rvalid<=1'b0;
            
            #40 rdata<=32'h0000000B;
                rvalid<=1'b1;
                
            #40 rvalid<=1'b0;
            
            #40 rdata<=32'h00000000;
                rvalid<=1'b1;
                
            #40 rvalid<=1'b0;  */ 
            
            
            /*
                rvalid data for second channel
            */
            #480
            #40 arready=1;
            #40 arready=0;
            
           #280    
           #40 rid<=2;
               rdata<=32'h000000BC;//DMAMOV for source address register
               rvalid<=1'b1;
           
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h00000800;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;    
           
           #40 rdata<=32'h000001BC;//DMAMOV for destination address register
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h0000000E;
               rvalid<=1'b1;
           
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h403902BC;//DMAMOV for channel control register
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           #40 rvalid<=1'b1;
               rdata<=32'h0000000E;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h00000007;
               rvalid<=1'b1;
           
           #40 rvalid<=1'b0;
               
           #40 rdata<=32'h00000007;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           
           //In the next cache access
               
           /*#40 rvalid<=1'b1;
               rdata<=32'h0000000B;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h0000000B;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h00000000;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;  */ 
            
            /*
                rvalid data for third channel
            */
            #480
            #40 arready=1;
            #40 arready=0;
            
           #360    
           #40 rid<=3;
               rdata<=32'h000000BC;//DMAMOV for source address register
               rvalid<=1'b1;
           
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h00000800;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;    
           
           #40 rdata<=32'h000001BC;//DMAMOV for destination address register
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h0000000E;
               rvalid<=1'b1;
           
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h403902BC;//DMAMOV for channel control register
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           #40 rvalid<=1'b1;
               rdata<=32'h0000000E;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h00000007;
               rvalid<=1'b1;
           
           #40 rvalid<=1'b0;
               
           #40 rdata<=32'h00000007;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           #80 arready=1'b1;
           #40 arready=1'b0;
           
           /*
                Data for load
           */
           #40 rid=1;
               rdata=27;
               rvalid=1'b1;
               
          #40 rvalid=0;
          
          #40 rdata=36;
              rvalid=1'b1;
              
          #40 rvalid=1'b0;
          
          #40 rdata=44;
              rvalid=1'b1;
              rlast=1'b1;
              
          #40 rvalid=1'b0;
              rlast=1'b0;           
           
           
           //In the next cache access
               
           /*#40 rvalid<=1'b1;
               rdata<=32'h0000000B;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h0000000B;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;
           
           #40 rdata<=32'h00000000;
               rvalid<=1'b1;
               
           #40 rvalid<=1'b0;  */ 
           
           
          #600
          
          #40 arready=1'b1;
          
          #40 arready=1'b0;
          
         
           /*
              Data for second load for dma channel 1
         */
         #40 rid=1;
             rdata=27;
             rvalid=1'b1;
             
        #40 rvalid=0;
        
        #40 rdata=36;
            rvalid=1'b1;
            
        #40 rvalid=1'b0;
        
        #40 rdata=44;
            rvalid=1'b1;
            rlast=1'b1;
            
        #40 rvalid=1'b0;
            rlast=1'b0; 
           
        /*
            arready for channel 2
        */
        #280 arready=1'b1;
        
        #40 arready=1'b0;
        
        /*
            data for first load for dma channel 2
        */
        
          #40 rid=2;
               rdata=27;
               rvalid=1'b1;
               
          #40 rvalid=0;
          
          #40 rdata=36;
              rvalid=1'b1;
              
          #40 rvalid=1'b0;
          
          #40 rdata=44;
              rvalid=1'b1;
              rlast=1'b1;
              
          #40 rvalid=1'b0;
              rlast=1'b0;
         
         /*
                Another three instructions for the first channel
         */
         #400 arready=1'b1;
         
         #40 arready=1'b0;
         
         
         /*
              other 8 data for the cache update  
         */
         
         #40 rid<=1'b1;
             rdata<=32'h0000000B;
             rvalid<=1'b1;
             
         #40 rvalid<=1'b0;
         
         
         #40 rdata<=332'h0000000B;
             rvalid<=1'b1;
             
         #40 rvalid<=1'b0;
         
         
         #40 rdata<=32'h00000000;
             rvalid<=1'b1;
             
         #40 rvalid<=1'b0;            
         
         #40 rdata<=32'h00000000;
             rvalid<=1'b1;
          
         #40 rvalid<=1'b0;
                
         #40 rdata<=32'h00000000;
             rvalid<=1'b1;
           
         #40 rvalid<=1'b0;
          
          
         #40 rdata<=32'h00000000;
             rvalid<=1'b1;
            
         #40 rvalid<=1'b0;          
                   
         #40 rdata<=32'h00000000;
             rvalid<=1'b1;
             
         #40 rvalid<=1'b0;
         
         #40 rdata<=32'h00000000;
             rvalid<=1'b1;
              
         #40 rvalid<=1'b0;
         
         
         
         /*
                Load Data for channel 2
         */
         
         #40 arready<=1'b1;
         
         #40 arready<=1'b0;
         
         /*
             data for first load for dma channel 2
         */
         
           #40 rid=2;
                rdata=27;
                rvalid=1'b1;
                
           #40 rvalid=0;
           
           #40 rdata=36;
               rvalid=1'b1;
               
           #40 rvalid=1'b0;
           
           #40 rdata=44;
               rvalid=1'b1;
               rlast=1'b1;
               
           #40 rvalid=1'b0;
               rlast=1'b0;
                   
        end
        
        
        
        
        
endmodule
