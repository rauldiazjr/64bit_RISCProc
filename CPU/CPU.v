`timescale 1ns / 1ps
/********************************************************************************
 *
 * Class:       CECS 440
 * Project:     Lab 6
 * Filename:    CPU.v
 * Due Date:    OCT 22, 2014
 * Description: This CPU is used for the purposes of testing the 
                operations loaded into the instruction memory module by 
                instantiating the CPU control unit and CPU Execution unit. The
                TB will 'kickstart' the CU by asserting a reset, the desired 
                results will be displayed onto the consl window.
*********************************************************************************/ 
module CPU(
//******************************************************************
   //   CPU PortList
//******************************************************************
   input   Clk, Reset, 
   input intr,

   inout [31:0] Bus_data, I_data,
   output int_ack,
   output dcs_,
   output drd_,
   output dwr_,
   
   output ics_,
   output ird_,
   output iwr_,
   
   output IOcs_, 
   output IOrd_, 
   output IOwr_,
   
   output [31:0] Mem_addr, I_addr   
   );
   wire IP_sel, J_sel, B_Jsel, SP_sel, SP_ld, se64IR_sel, Wrbuf1_sel;
   wire [3:0]  fpBuf_Ctrl;
   wire [4:0]  wrBuf_Ctrl;
   wire [5:0]  rdBuf_Ctrl;
   wire [10:0] CUFlagStatus; 
   wire [8:0]  addr_Ctrl;
   wire [20:0] fpdp_Ctrl;
   wire [23:0] idp_Ctrl;
   wire [4:0]  Int_status;
   wire [5:0]  FP_status;
   wire [31:0] Bus_data, I_data; //inout
   
//******************************************************************
   //   CPU Control Unit 
//******************************************************************
   CU64      CU64(Clk, Reset, intr, Int_status[3], Int_status[2],Int_status[1],
                 Int_status[0],FP_status, int_ack, IP_sel, J_sel, B_Jsel,
                 SP_sel, SP_ld, se64IR_sel, Wrbuf1_sel,
                 {dcs_,drd_,dwr_},
                 {ics_,ird_,iwr_},
                 {IOcs_,IOrd_,IOwr_}, fpBuf_Ctrl, wrBuf_Ctrl, rdBuf_Ctrl,  
                 addr_Ctrl, CUFlagStatus, fpdp_Ctrl, idp_Ctrl);                 
//******************************************************************
   //   CPU Exectution Unit
//******************************************************************
   CPU_EU   CPU_EU(Clk, Reset, fpBuf_Ctrl, wrBuf_Ctrl, rdBuf_Ctrl, idp_Ctrl, 
                   fpdp_Ctrl,addr_Ctrl, Int_status, FP_status, Mem_addr,  
                   Bus_data, I_addr,I_data, IP_sel, J_sel, B_Jsel, SP_sel, SP_ld, se64IR_sel,
                   Wrbuf1_sel, CUFlagStatus  );

endmodule