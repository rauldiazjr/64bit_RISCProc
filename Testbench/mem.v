`timescale 1ns / 1ps

module mem( Clk, CS_, RD_, WR_, Addr, Data );
// *********************************************************
// A 1024x32 memory with bi-directional data lines.
//
// Reading is done asynchronously, without regard to Clk,
// and is modeled with a conditional continuous assignment
// to implement the bi-directional outputs (with hi-z).
// Writing is done synchronously, only on the positive edge
// of Clk (iff CS_ and WR_ are asserted) and is modeled
// using a procedural block.
//
// Note: CS_, RD_, and WR_ are all active-low.
//
// The memory is to be initialized from within the testbench
// that instantiates it, using the $readmemh function.
//
// Note: when instantiating this module, only use the least
// 10-significant address lines, e.g. addr[9:0]
// *********************************************************
   input Clk, CS_, RD_, WR_;   
   input [9:0] Addr;   
   inout [31:0] Data;
   
   reg   [31:0] memarray [0:1023];   // actual "array of registers"
   wire  [31:0] Data;               // wire for tri-state I/O
   
   //---------------------------------------------------
   // conditional continuous assignment to implement
   // if (both CS_ and RD_) are asserted
   // then Data = memarray[Addr]
   // else Data = Hi-z
   //---------------------------------------------------
   assign  Data = (!CS_ & !RD_) ? memarray[Addr] : 32'bz;
   
   //---------------------------------------------------
   //procedural assignment to implement
   // if (both CS_ and WR_ are asserted) on posedge Clk
   // then memarray[Addr] = Data
   //---------------------------------------------------
   always @(posedge Clk)
     if (!CS_ & !WR_)
       memarray[Addr] = Data;

endmodule
