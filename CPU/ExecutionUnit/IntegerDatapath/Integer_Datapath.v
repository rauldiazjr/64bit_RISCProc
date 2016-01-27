`timescale 1ns / 1ps
/********************************************************************************
 *
 * Class:       CECS 440
 * Author:      Raul Diaz
 * Email:       raul.diaz91@live.com
 * Project:     Lab 3
 * Filename:    Integer_Datapath.v
 * Due Date:    Sept 29, 2014
 * Description: This integer data path module instantiates a 64 bit ALU module, 
                 a 64 bit Register module and 2 64 bit multiplexers which drive 
                 the ALU's input and outputs. 
*********************************************************************************/
module Integer_Datapath(Clk, W_en, W_addr, R_addr, S_addr, DS, S_sel, ALU_op, DY, 
                        C, N, Z, O, Y_sel, REG_out, ALU_out); 
    input         Clk, W_en, S_sel,  Y_sel;
    input  [4:0]  W_addr, R_addr, S_addr, ALU_op;
    input  [63:0] DS, DY; 
    output        C, N, Z, O;
    output [63:0] REG_out, ALU_out; 
    wire   [63:0] S_regTOsmux, S_smuxTOalu, Y_aluTOymux; 
    
    //************************************************************
    //64bit Register Module
    //************************************************************
    //Clk, W_en, W_Addr, WR, R_Addr, S_Addr, R, S
    regfile64 reg64(Clk, W_en, W_addr, ALU_out, R_addr, S_addr, REG_out, S_regTOsmux) ;
    
    //************************************************************
    //64bit SMUX Module
    //************************************************************
    assign S_smuxTOalu = S_sel ? DS : S_regTOsmux;
    
    //************************************************************
    //64bit ALU Module
    //************************************************************
    //R, S, Alu_Op, Y, C, N, Z, O
    ALU_64    ALU64(REG_out, S_smuxTOalu, ALU_op, Y_aluTOymux, C,N,Z,O);
    
    //************************************************************
    //64bit YMUX Module
    //************************************************************
    assign ALU_out = Y_sel ? DY : Y_aluTOymux;
    
 //************************************************************
 //END of IDP module
 //************************************************************
endmodule
