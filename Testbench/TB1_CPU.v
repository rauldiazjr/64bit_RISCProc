`timescale 1ns / 1ps 


module TB1_CPU;

   // Inputs
   reg Clk;
   reg Reset;
   

   // Outputs
   wire int_ack;
   wire dcs_;
   wire drd_;
   wire dwr_;
   wire ics_;
   wire ird_;
   wire iwr_;
   wire IOcs_;
   wire IOrd_;
   wire IOwr_;
   wire [31:0] Mem_addr;
   wire [31:0] I_addr;
   wire intr;
   // Bidirs
   wire [31:0] Bus_data;
   wire [31:0] I_data;

   // Instantiate the Unit Under Test (UUT)
   CPU uut (
      .Clk(Clk), 
      .Reset(Reset), 
      .intr(intr), 
      .Bus_data(Bus_data), 
      .I_data(I_data), 
      .int_ack(int_ack), 
      .dcs_(dcs_), 
      .drd_(drd_), 
      .dwr_(dwr_), 
      .ics_(ics_), 
      .ird_(ird_), 
      .iwr_(iwr_), 
      .IOcs_(IOcs_), 
      .IOrd_(IOrd_), 
      .IOwr_(IOwr_), 
      .Mem_addr(Mem_addr), 
      .I_addr(I_addr)
   );
   mem dmem(
      .Clk(Clk), 
      .CS_(dcs_), 
      .RD_(drd_), 
      .WR_(dwr_), 
      .Addr(Mem_addr[9:0]), 
      .Data(Bus_data)
   );
   
   mem imem(
      .Clk(Clk), 
      .CS_(ics_), 
      .RD_(ird_), 
      .WR_(iwr_), 
      .Addr(I_addr[9:0]), 
      .Data(I_data)
   );
   
   IOmem IO(
      .Clk(Clk), 
      .CS_(IOcs_), 
      .RD_(IOrd_), 
      .WR_(IOwr_), 
      .Addr(Mem_addr[9:0]), 
      .Data(Bus_data),
      .Int_ack(int_ack), 
      .Int_req(intr)
   );
   always
      #1 Clk = ~Clk;
      
   initial begin
      $timeformat(-12, 1, "ps", 9);
   
      // Initialize Inputs
      Clk = 0;
      Reset = 0;
      

      // Wait 100 ns for global reset to finish
      #100;
       Reset = 1'b1; 
       
      #50
      @ (negedge Clk)
          Reset = 1'b0;
      
      
      /*Test Module 14
      ###NOTE: Test Bench simulation requires extra run time to complete instructions*/       
//      $readmemh("dMem14.dat", dmem.memarray);$readmemh("iMem14.dat", imem.memarray);
            
      /*Test Module 13 
      ###NOTE: Comment out dump_Ireg task call under HALT state in CU64 module.
             and assert dumpFPR task call*/      
//      $readmemh("dMem13.dat", dmem.memarray);$readmemh("iMem13.dat", imem.memarray);
            
      /*Test Module 12*/         
//      $readmemh("dMem12.dat", dmem.memarray);$readmemh("iMem12.dat", imem.memarray);
      
      /*Test Module 11
      ###NOTE: Test Bench simulation requires extra run time to complete instructions*/      
//      $readmemh("dMem11.dat", dmem.memarray);$readmemh("iMem11.dat", imem.memarray);
      
      /*Test Module 10*/      
//      $readmemh("dMem10.dat", dmem.memarray);$readmemh("iMem10.dat", imem.memarray);
         
      /*Test Module 09*/      
//      $readmemh("dMem09.dat", dmem.memarray);$readmemh("iMem09.dat", imem.memarray);
      
      /*Test Module 08*/      
//      $readmemh("dMem08.dat", dmem.memarray);$readmemh("iMem08.dat", imem.memarray);
      
      /*Test Module 07
      ###NOTE: Test Bench simulation requires extra run time to complete instructions*/         
//      $readmemh("dMem07.dat", dmem.memarray);$readmemh("iMem07.dat", imem.memarray);
      
      /*Test Module 06*/      
//      $readmemh("dMem06.dat", dmem.memarray);$readmemh("iMem06.dat", imem.memarray);
      
      /*Test Module 05*/      
//    $readmemh("dMem05.dat", dmem.memarray);$readmemh("iMem05.dat", imem.memarray);
      
      /*Test Module 04*/      
//      $readmemh("dMem04.dat", dmem.memarray);$readmemh("iMem04.dat", imem.memarray);
      
      /*Test Module 03*/      
//      $readmemh("dMem03.dat", dmem.memarray);$readmemh("iMem03.dat", imem.memarray);
      
      /*Test Module 02*/      
//      $readmemh("dMem02.dat", dmem.memarray);$readmemh("iMem02.dat", imem.memarray);
      
      /*Test Module 01*/      
    $readmemh("dMem01.dat", dmem.memarray);$readmemh("iMem01.dat", imem.memarray);
      
      $display(" "); $display(" ");
      $display("***************************************************************");
      $display("C O R B E N I C  T e s t b e n c h    R e s u l t s   ");;
      $display("***************************************************************");
      $display(" ");
      
            
   end
      
endmodule


