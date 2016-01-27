`timescale 1ns / 1ps

module Rd_reg32(Clk, Reset, Din, Dout, ld, oe, inc);

   input                Clk, Reset, ld, oe, inc; 
   input         [31:0] Din; 
   
   output        [31:0] Dout; 
   reg           [31:0] data; 
   
   assign Dout = oe? data : 32'b0; 
   
   always @(posedge Clk, posedge Reset) begin
      if(Reset == 1'b1)
         data <= 32'b0; 
      else
         casex ({ld, inc})
            2'b01: data <= data + 1;         
            2'b1x: data <= Din;     
            default: data <= data; 
            
         endcase
    end
   
   

endmodule
