`timescale 1ns / 1ps

module IP_reg32(Clk, Reset, Din, Dout, ld, oe, inc, isr);

   input                Clk, Reset, ld, oe, inc, isr; 
   input         [31:0] Din; 
   
   output        [31:0] Dout; 
   reg           [31:0] data; 
   
   assign Dout = oe? data : 32'bz; 
   
   always @(posedge Clk, posedge Reset) begin
      if(Reset == 1'b1)
         data <= 32'h0; 
      else
         case ({inc, isr, ld})
            3'b100:  data <= data + 1;         
            3'b010:  data <= 32'h3FF;    //hardcoded ISR address
            3'b001:  data <= Din;
            default: data <= data;             
         endcase
    end
   
   

endmodule
