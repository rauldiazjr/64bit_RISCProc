`timescale 1ns / 1ps
/********************************************************************************
 *
 * Class:       CECS 440
 * Author:      Raul Diaz
 * Email:       raul.diaz91@live.com
 * Project:     Lab 2
 * Filename:    regFile.v
 * Due Date:    Sept 17, 2014
 * Description: A 32x64 Register file. This register has 3 address inputs: W_Addr
                R_Addr, and S_Addr used for loading and storing data into their 
                respective 64bit register outputs WR, R, and S. W_en input must
                be high in order to write to the register file.
*********************************************************************************/
module regfile64(Clk, W_en, W_Addr, WR, R_Addr, S_Addr, R, S);
   input Clk, W_en; 
   input [4:0] W_Addr, R_Addr, S_Addr; 
   input [63:0] WR; 

   output reg [63:0] R, S; 
   reg [63:0] regFile[31:0]; 
   
   //************************************************************
   //Write to regFile
   //************************************************************
   always @(posedge Clk ) begin
      if(W_en)
         regFile[W_Addr] <= WR;       
   end

   //************************************************************
   //Write to R output referencing R_Addr
   //************************************************************
   always @( R_Addr, regFile[R_Addr]) begin
      R <= regFile[R_Addr]; 
      
   end
   //************************************************************
   //Write to S output referencing S_Addr 
   //************************************************************
   always @( S_Addr, regFile[S_Addr]) begin
      S <= regFile[S_Addr]; 
      
   end

endmodule
