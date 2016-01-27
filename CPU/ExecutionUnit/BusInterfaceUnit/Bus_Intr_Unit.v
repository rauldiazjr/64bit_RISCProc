`timescale 1ns / 1ps
/********************************************************************************
 *
 * Class:       CECS 440
 * Project:     Lab 6
 * Filename:    Bus_Intr_Unit.v
 * Due Date:    OCT 22, 2014
 * Description: The Bus interface unit takes 64bit input and divides the data
                among their respective registers. In this BIU, there exists
                2 32bit floating point register, 2 32bit integer write registers, 
                a Memory Address register, 2 32bit Read registers, and an IR
                register. The BIU also contatins a 32bit bi-direction data bus line
                used to communicate with a memory module.                
*********************************************************************************/
module Bus_Intr_Unit(Clk, Reset, FPBuf, FP1_ld, FP0_ld, FP1_oe, FP0_oe, 
                     RdBuf, RdBuf1_ld, RdBuf0_ld, 
                     MAR_in, MAR_ld, MAR_inc, dMem_addr,
                     WrBuf, WrBuf1_ld, WrBuf0_ld, WrBuf1_oe, WrBuf0_oe, 
                     signext, IR_ld, Bus_data,
                     IP_inc, SP_inc, SP_dec, Rdmux1, Rdmux0, dMem_addr_sel,   
                     I_addr,I_data,IP_ld,IP_isr,Wrbuf0_sel,RdBuf1_oe,RdBuf0_oe,
                     IP_sel, J_sel, B_Jsel, SP_sel, SP_ld, se64IR_sel, cpsrFlags, Wrbuf1_sel );   
                     
   
   input       Clk, Reset, FP1_ld, FP0_ld, FP1_oe, FP0_oe, RdBuf1_ld, RdBuf0_ld; 
   input       MAR_ld, MAR_inc, WrBuf1_ld, WrBuf0_ld, WrBuf1_oe, WrBuf0_oe, IR_ld;
   
   input       IP_inc, SP_inc, SP_dec, Rdmux1, Rdmux0, dMem_addr_sel; 
   input       IP_ld, IP_isr,  Wrbuf0_sel, RdBuf1_oe, RdBuf0_oe, Wrbuf1_sel; 
      
   input       IP_sel, J_sel, B_Jsel ;
   input       SP_sel, SP_ld; 
   input       se64IR_sel;
   input       [31:0] cpsrFlags;
   input       [63:0] FPBuf,  WrBuf, MAR_in;   
   
   
   inout       [31:0] Bus_data;    
   inout       [31:0] I_data;            
   output      [31:0] I_addr, dMem_addr;  
   
   output wire [63:0] RdBuf ; 
   output wire [63:0] signext;
   
   wire   [31:0] Bus_data, MAR_out; 
   wire   [31:0] IR_out;
   wire   [31:0] MuxtoRdbuf1, MuxtoRdbuf0, Stack_addr, I_data, IP_in, muxToWrbuf0;
   wire   [31:0]Branch_Jump,Jump_addr, Branch_addr, StkPtr_in, muxToWrbuf1 ;
   
   //************************************************************
   //32bit Floating point register buffers
   //************************************************************ 
   reg32    FPBuf_1(Clk, Reset, FPBuf[63:32], Bus_data, FP1_ld, FP1_oe, 1'b0); 
   reg32    FPBuf_0(Clk, Reset, FPBuf[31: 0], Bus_data, FP0_ld, FP0_oe, 1'b0);
   
   //************************************************************
   //32bit Read register buffers & Mux
   //************************************************************  
      
      //************************************************************
      //   2 32bit Multiplexers to select data from the instruction reg
      //         Or the 32bit data bus from the data memory port
      //************************************************************ 
         assign MuxtoRdbuf1 = Rdmux1 ? I_data : Bus_data ;
         assign MuxtoRdbuf0 = Rdmux0 ? I_data : Bus_data ;
   
         Rd_reg32    RdBuf_1(Clk, Reset, MuxtoRdbuf1, RdBuf[63:32], 
                        RdBuf1_ld, RdBuf1_oe, 1'b0); 
         Rd_reg32    RdBuf_0(Clk, Reset, MuxtoRdbuf0, RdBuf[31: 0], 
                        RdBuf0_ld, RdBuf0_oe, 1'b0);    
   //************************************************************
   //32bit mux to select data going into data memory address port
   //    from memory address register or a 32bit stack pointer
   //************************************************************ 
      assign   dMem_addr = dMem_addr_sel ? Stack_addr : MAR_out;
   
   //************************************************************
   //32bit Memory address register 
   //************************************************************  
      reg32    MAR(Clk, Reset, MAR_in[31:0], MAR_out, MAR_ld, 1'b1, MAR_inc); 
   //************************************************************
   //32bit Integer write register buffers
   //************************************************************   
      reg32    WrBuf_1(Clk, Reset, muxToWrbuf1, Bus_data, WrBuf1_ld, WrBuf1_oe, 1'b0); 
      reg32    WrBuf_0(Clk, Reset, muxToWrbuf0, Bus_data, WrBuf0_ld, WrBuf0_oe, 1'b0);
      //************************************************************
      //32bit Mux for WrBuf0 register for purposes of loading IP
      //************************************************************  
         assign    muxToWrbuf0 = Wrbuf0_sel ? I_addr : WrBuf[31: 0];
      //************************************************************
      //32bit Mux for WrBuf1 register for purposes of present state flags
      //************************************************************ 
         assign   muxToWrbuf1 = Wrbuf1_sel ? cpsrFlags : WrBuf[63:32];
   //************************************************************
   //32bit Instruction register 
   //************************************************************ 
      reg32         IR(Clk, Reset, I_data, IR_out, IR_ld, 1'b1, 1'b0); 
   
   //************************************************************
   //32bit Intruction Pointer register
   //************************************************************ 
      IP_reg32     IP(Clk, Reset, IP_in, I_addr, IP_ld, 1'b1, IP_inc, IP_isr); 
   
   //************************************************************
   //32bit Mux for IP_in port for purposes of loading IP from 
   //      RdBuf0 port 
   //************************************************************     
      assign Branch_addr = I_addr + {IR_out[15:0], 2'b00}; 
      assign Jump_addr = J_sel ? MAR_in[31:0] : (I_addr + {{8{IR_out[23]}}, IR_out[23:0]}); 
      assign Branch_Jump = B_Jsel ? Jump_addr : Branch_addr;   
      assign IP_in = IP_sel ?  RdBuf[31: 0] : Branch_Jump;    
   //************************************************************
   //32bit Stack Pointer register with inc/dec enables
   //************************************************************ 
      //************************************************************
      //32bit Mux for SP dataIn port for purposes of loading IR data  
      //   into SP reg
      //************************************************************    
      assign StkPtr_in = SP_sel ? WrBuf[31: 0] : {8'h00, IR_out[23:0]} ; 
      stack_reg32     SP(Clk, Reset, StkPtr_in, Stack_addr, SP_ld, 1'b1, SP_inc, SP_dec);  
   
   //************************************************************
   // Sign extention unit used to extend a 24 bit Instuction
   //************************************************************ 
   assign signext = se64IR_sel ? {{40{IR_out[23]}}, IR_out[23:0]} : {{56{IR_out[15]}}, IR_out[15:8]} ;  
   
endmodule
