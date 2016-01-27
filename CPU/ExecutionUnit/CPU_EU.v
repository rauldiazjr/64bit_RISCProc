`timescale 1ns / 1ps
/********************************************************************************
 *
 * Class:       CECS 440
 * Project:     Lab 6
 * Filename:    CPU_EU.v
 * Due Date:    OCT 22, 2014
 * Description: The CPU Exection unit module instatiates a integer datapath module
                a floating point module and bus interface unit module. By 
                interconnecting the modules, a pipeline archetecture is constructed
                and data is can be shared given the correct control signals. 
                
*********************************************************************************/
//*----------------------------------------------------------------
//*                   CPU C O N T R O L   W O R D
//*----------------------------------------------------------------
/**********************************************************************************
// fpBuf_Ctrl [3:0] {FPBuff1_ld | FPBuff0_ld | FPBuff1_oe | FPBuff0_oe }
*********************************************************************************
// wrBuf_Ctrl [4:0] {Wrbuf0_sel | WrBuf1_ld | WrBuf1_ld | WrBuf1_oe | WrBuf1_oe }
*********************************************************************************
// rdBuf_Ctrl [5:0] {RdBuf1_oe | RdBuf0_oe| RdBuf1_ld | RdBuf0_ld |Rdmux1 | Rdmux0}
*********************************************************************************
// addr_Ctrl [8:3] {IP_isr | IP_ld | IP_inc | IR_ld | SP_inc | SP_dec  
////****     [2:0]    dMem_addr_sel | MAR_ld | MAR_inc}                             
*********************************************************************************
// fpdp_Ctrl [20:5]{FW_en|FW_addr[19:15]|FR_addr[14:10]|FS_addr[9:5]
////****     [4:0]                | FP_sel | FP_op[3:0] }                           
*********************************************************************************   
// idp_Ctrl [23:7] {DS_sel | W_en | W_addr[21:17] | R_addr[16:12] |S_addr[11:7] | 
////****      [6:0]     | S_sel[6] | ALU_op[5:1] | Y_sel }                          
////******************************************************************/

////******************************************************************/
 
module CPU_EU(Clk, Reset, fpBuf_Ctrl, wrBuf_Ctrl, rdBuf_Ctrl,idp_Ctrl, fpdp_Ctrl, 
               addr_Ctrl, Int_status, FP_status, Mem_addr, Bus_data, I_addr, I_data,
               IP_sel, J_sel, B_Jsel, SP_sel, SP_ld, se64IR_sel,Wrbuf1_sel, CUFlagStatus);
               
   input        Clk, Reset;
   input        IP_sel, J_sel, B_Jsel, SP_sel, SP_ld, se64IR_sel, Wrbuf1_sel; 
   input [3:0]  fpBuf_Ctrl;
   input [4:0]  wrBuf_Ctrl;
   input [5:0]  rdBuf_Ctrl;
   input [8:0]  addr_Ctrl; 
   input [10:0] CUFlagStatus; 
   input [20:0] fpdp_Ctrl; 
   input [23:0] idp_Ctrl;  
   inout [31:0] Bus_data, I_data;
   
   output [4:0]  Int_status; // {Intr, C, N, Z, O }
   output [5:0]  FP_status;
   output [31:0] Mem_addr, I_addr; 
   
   wire [31:0]   Bus_data, I_data, cpsrFlags, se_CUFlags;
   wire [63:0]   FP_out, Data_from_Mem, IDPReg_to_MAR, IDPALU_to_WrBuf, se64_IR;
   wire [63:0]   DS; 
   
   
   //************************************************************
   //CURRENT PROGRAM STATUS REGISTER 
   //************************************************************   
      assign      se_CUFlags = {{21{CUFlagStatus[10]}}, CUFlagStatus[10:0]};
      reg32       cpsr(Clk, Reset, se_CUFlags, cpsrFlags, 1'b1, 1'b1, 1'b0); 
   
   //************************************************************
   //BUS INTERFACE UNIT MODULE
   //************************************************************   
      Bus_Intr_Unit  biu(Clk, Reset, FP_out, fpBuf_Ctrl[3], fpBuf_Ctrl[2], 
                       fpBuf_Ctrl[1],    fpBuf_Ctrl[0],                    
                       Data_from_Mem,    rdBuf_Ctrl[3], rdBuf_Ctrl[2],     
                       IDPReg_to_MAR,    addr_Ctrl[1], addr_Ctrl[0], Mem_addr, 
                       IDPALU_to_WrBuf,  wrBuf_Ctrl[3], wrBuf_Ctrl[2],   
                       wrBuf_Ctrl[1],    wrBuf_Ctrl[0],                    
                       se64_IR,          addr_Ctrl[5], Bus_data ,
                       addr_Ctrl[6],     addr_Ctrl[4], addr_Ctrl[3], rdBuf_Ctrl[1], 
                       rdBuf_Ctrl[0],    addr_Ctrl[2],
                       I_addr, I_data,   addr_Ctrl[7], addr_Ctrl[8], wrBuf_Ctrl[4],
                       rdBuf_Ctrl[5],    rdBuf_Ctrl[4], 
                       IP_sel, J_sel, B_Jsel, SP_sel, SP_ld, se64IR_sel, cpsrFlags, Wrbuf1_sel);
   //************************************************************
   //INTEGER DATA PATH MODULE 
   //************************************************************
      //************************************************************
      //64 Bit mux for DS port for purposes of selecting RdBuf0
      //************************************************************
      assign   DS = idp_Ctrl[23] ?    Data_from_Mem   : se64_IR;      

      Integer_Datapath  idp(Clk, idp_Ctrl[22], idp_Ctrl[21:17], idp_Ctrl[16:12], 
                         idp_Ctrl[11:7], DS, idp_Ctrl[6],    idp_Ctrl[5:1],
                         Data_from_Mem,  Int_status[3],      Int_status[2], Int_status[1],
                         Int_status[0],  idp_Ctrl[0],
                         IDPReg_to_MAR,  IDPALU_to_WrBuf); 
   
   //************************************************************
   //FLOATING POINT DATAPATH MODULE
   //************************************************************
      FloatDP  fdp(Clk, fpdp_Ctrl[20], fpdp_Ctrl[19:15], fpdp_Ctrl[14:10], 
                fpdp_Ctrl[9:5] ,    Data_from_Mem,    fpdp_Ctrl[4],
                fpdp_Ctrl[3:0],     FP_status,        FP_out);       

endmodule
