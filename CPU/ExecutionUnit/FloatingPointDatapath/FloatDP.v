`timescale 1ns / 1ps
/********************************************************************************
 *
 * Class:       CECS 440
 * Author:      Raul Diaz
 * Email:       raul.diaz91@live.com
 * Project:     Lab 3
 * Filename:    Integer_Datapath.v
 * Due Date:    Sept 29, 2014
 * Description: This Floating point data path module instantiates a 64 bit 
                floating point ALU module, a 64 bit floating point Register module 
                and a 64 bit multiplexers which drive the F_ALU's input to give 
                a user the option of an immediate value or a value from the reg file.
                
*********************************************************************************/
module FloatDP(Clk, FW_en, FW_addr, FR_addr, FS_addr, Float_in, FP_sel, FP_op, 
               FP_status, Float_out );
   
   input          Clk, FW_en, FP_sel;
   input   [3:0]  FP_op; 
   input   [4:0]  FW_addr, FR_addr, FS_addr;
   input  [63:0]  Float_in;
   output  [5:0]  FP_status; 
   output [63:0]  Float_out; 
   
   wire   [63:0]  R, StoFmux, FmuxtoAlu; 
   //************************************************************
   //64bit Floating point Register Module
   //************************************************************
   regfile64   freg64(Clk, FW_en, FW_addr, Float_out, FR_addr, FS_addr, R, StoFmux);
   
   //************************************************************
   //64bit FMUX Module
   //************************************************************
   assign FmuxtoAlu =  FP_sel ? Float_in : StoFmux ;
   
   //************************************************************
   //64bit Floating point ALU  Module
   //************************************************************
   FloatAlu         falu64(Float_out, R, FmuxtoAlu, FP_op, FP_status); 
   
endmodule
