`timescale 1ns / 1ps
/**********************************************************************************
* Date: January 12, 2014
* File: CU64.v
*
* A state machine that implements the Control Unit (CU) for the major cycles of 
* fetch, execute and some 440 ISA instructions from memory, including checking for 
* interrupts.
*
*-----------------------------------------------------------------------------
*******************  CU64 C O N T R O L W O R D  *****************************
*-----------------------------------------------------------------------------
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

    
module CU64(  
    //******************************************************************
    //    CU PortList
    //******************************************************************
    input    sys_clk, reset, intr, 
    input    c, n, z, o, 
    input    [5:0] FP_status, 
    
    output   int_ack,
    output   IP_sel, J_sel, B_Jsel , SP_sel, SP_ld, se64IR_sel, Wrbuf1_sel, 
    output   [2:0]  dmem_Ctrl, 
    output   [2:0]  imem_Ctrl,
    output   [2:0]  IOmem_Ctrl,
    output   [3:0]  fpBuf_Ctrl,
    output   [4:0]  wrBuf_Ctrl,
    output   [5:0]  rdBuf_Ctrl,
    output   [8:0]  addr_Ctrl,
    output   [10:0] CUFlagStatus,
    output   [20:0] fpdp_Ctrl, 
    output   [23:0] idp_Ctrl
    );
    
    reg   int_ack; 
    reg   IP_sel, J_sel, B_Jsel, SP_sel, SP_ld, se64IR_sel, Wrbuf1_sel; 
    reg   [4:0] NS_Istat, PS_Istat; 
    reg   [5:0] NS_FPstat, PS_FPstat;
    reg   [2:0]  dmem_Ctrl; 
    reg   [2:0]  imem_Ctrl;
    reg   [2:0]  IOmem_Ctrl; 
    reg   [3:0]  fpBuf_Ctrl;
    reg   [4:0]  wrBuf_Ctrl;
    reg   [5:0]  rdBuf_Ctrl;
    reg   [8:0]  addr_Ctrl; 
    reg   [10:0] CUFlagStatus; 
    reg   [20:0] fpdp_Ctrl; 
    reg   [23:0] idp_Ctrl;

    //******************************************************************
    //    Internal Data Structure
    //******************************************************************
    
    //state assignments 
    parameter 
        FETCH  = 0 ,  DECODE =  1,   INTR_1 =  2,  INTR_2 =  3,  INTR_3  = 4,
        ADD    = 10,  INC    = 11,   DEC    = 12, 
        JNZ    = 14,  JP     = 15,   JC     = 16,  JR     = 17,  J       = 19,
        LDI_1  = 20,  LDI_2  = 21,     
        ORHI_1 = 23,  ORHI_2 = 24, 
        STO_1  = 26,  STO_2  = 27,   STO_3  = 28, 
        LDR_1  = 30,  LDR_2  = 31,   LDR_3  = 32,  LDR_4  = 33,
        COPY   = 35,  ASHR   = 36, 
        NOP    = 40,  SUB    = 41,   LSL    = 42,  NEG    = 43,  MUL_1   = 44,     
        MUL_2  = 45,  CMP    = 46,   DIV_1  = 47,  DIV_2  = 48,  
        XOR    = 50,  JZ     = 51,   LdSP   = 52, 
        CALL_1 = 55,  CALL_2 = 56,   CALL_3 = 57, 
        RET_1  = 60,  RET_2  = 61,   ADDi   = 62,  MULi_1 = 63,  MULi_2  = 64, 
        RCALL_1= 65,  RCALL_2= 66,   RCALL_3= 67, 
        PUSH_1 = 70,  PUSH_2 = 71,   PUSH_3 = 72, 
        POP_1  = 75,  POP_2  = 76,   POP_3  = 77, 
        NOT    = 80,  SUBi   = 81,   ASHL   = 82,  DIVi_1 = 83,  DIVi_2  = 84, 
        ROL    = 85,  ROR    = 86,   JLT    = 87,  JGE    = 88, 
        STC    = 90,  CLC    = 91,   CMPi   = 92,  JNO    = 93,  JO      = 94, 
        JLE    = 95,  JGT    = 96,   JAE    = 97,  JA     = 98,  JBE     = 99,
        JNC    = 100, JN     = 101,  JB     = 102,
        FADD   = 105, FSUB   = 106,  FMUL   = 107, FDIV   = 108, FINC    = 109, 
        FDEC   = 110, FZERO  = 111,  FONE   = 112, 
        FLDI_1 = 115, FLDI_2 = 116,
        FLDR_1 = 120, FLDR_2 = 121,  FLDR_3 = 122, FLDR_4 = 123,
        FSTO_1 = 125, FSTO_2 = 126,  FSTO_3 = 127, FORHI_1= 128, FORHI_2 = 129,
        RETI_1 = 130, RETI_2 = 131,  RETI_3 = 132, RETI_4 = 133, 
        OUT_1  = 135, OUT_2  = 136,  OUT_3  = 137, CLRINT = 138,
        IN_1   = 140, IN_2   = 141,  IN_3   = 142, IN_4   = 143,
        STINT  = 145, EXCH_1 = 146,  EXCH_2 = 147, EXCH_3 = 148, 
        
        HALT       = 510, 
        ILLEGAL_OP = 511;
    
    //State Register (512 possible states)
    reg      [8:0] state; 
    
//********************************************************************************
/*                          440 RISC CONTROL UNIT (FINITE STATE MACHINE)           */
//********************************************************************************

    always @(posedge sys_clk)
    begin
        PS_Istat =  NS_Istat ; 
        PS_FPstat = NS_FPstat ;
        CUFlagStatus = {NS_FPstat, NS_Istat}; //update CPSR
      
    if (reset)
        begin 
        @(negedge sys_clk)
            int_ack     = 1'b0; 
            imem_Ctrl  = 3'b111; 
            dmem_Ctrl  = 3'b111; 
            IOmem_Ctrl = 3'b111;
            fpBuf_Ctrl = 4'b0; 
            wrBuf_Ctrl = 5'b0; 
            rdBuf_Ctrl = 6'b0; 
            addr_Ctrl  = 9'b0; 
            fpdp_Ctrl  = 21'b0; 
            idp_Ctrl   = 24'b0;
            PS_Istat   = 5'b0; 
            PS_FPstat  = 6'b0; 
            NS_Istat   = 5'b0;
            NS_FPstat  = 6'b0; 
            {IP_sel, J_sel, B_Jsel} = 3'b000 ;
            {SP_sel, SP_ld} = 2'b00; 
            se64IR_sel = 1'b0; 
            Wrbuf1_sel = 1'b0; 
            state = FETCH;         

        end
    
    else 
        case (state)
            FETCH:
                begin
                if (int_ack ==0 & intr ==1 & PS_Istat[4] == 1) // & intr enable
                    begin     
                    /************************************************************/
                    ////**INTR pending, ready ISR
                    //Control word for deassertion
                    /************************************************************/
                    @(negedge sys_clk)    
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        IOmem_Ctrl = 3'b111;
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0; 
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl   = 24'b0;
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        se64IR_sel = 1'b0; 
                        Wrbuf1_sel = 1'b0; 
                        NS_Istat = PS_Istat; //Flags remain same
                        NS_FPstat = PS_FPstat;
                        state =     INTR_1;
                        
                    end
                else
                    begin // ** NO INTR pending , NS = FETCH
                        if( int_ack == 1 & intr ==0) int_ack=1'b0;
                    @(negedge sys_clk)
                    // IR <- iM[IP]; IP <- IP + 1
                        imem_Ctrl  = 3'b001; 
                        dmem_Ctrl  = 3'b111; 
                        IOmem_Ctrl = 3'b111;
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b00_11_00_0_00; 
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl   = 24'b0;
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        se64IR_sel = 1'b0; 
                        Wrbuf1_sel = 1'b0; 
                        NS_Istat  = PS_Istat; //Flags remain same
                        NS_FPstat = PS_FPstat;
                        state = DECODE;                               
                            
                    end
                end    
            DECODE:
                begin
                    @(negedge sys_clk)
                    
                    /************************************************************/
                    // default control word assignments 
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;                         
                        fpdp_Ctrl     = {1'b0, 
                                          CPU.CPU_EU.biu.IR.data[4:0],   //FW_addr = IR[4:0]
                                          CPU.CPU_EU.biu.IR.data[20:16], //FR_addr = IR{20:16]
                                          CPU.CPU_EU.biu.IR.data[12:8],  //FS_addr = IR[12:8]
                                          1'b0, 
                                          4'b0}; 
                                          
                        idp_Ctrl   = {2'b00,
                                          CPU.CPU_EU.biu.IR.data[4:0],   //W_addr = IR[4:0]
                                          CPU.CPU_EU.biu.IR.data[20:16], //R_addr = IR{20:16]
                                          CPU.CPU_EU.biu.IR.data[12:8],  //S_addr = IR[12:8]
                                          1'b0,
                                          5'b0,
                                          1'b0};
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        se64IR_sel = 1'b0; 
                        Wrbuf1_sel = 1'b0; 
                        NS_Istat  = PS_Istat;      //Flags remain same
                        NS_FPstat = PS_FPstat;
                            
                    case(CPU.CPU_EU.biu.IR.data[31:24])
                    // 1. Triple Op Instr
                        8'h80 :  state = ADD;
                        8'h81 :  state = SUB;
                        8'h82 :  state = MUL_1;
                        8'h83 :  state = DIV_1;
                        //8'H84 :  state = AND; 
                        //8'H85 :  state = OR;
                        8'h86 :  state = XOR;
                  
                     // 2. Double Op Instr
                        8'h87 :  state = LDR_1;
                        8'h88 :  state = LDI_1; 
                        8'h89 :  state = STO_1; 
                        8'h8A :  state = COPY;
                        8'H8B :  state = EXCH_1;
                        8'H8C :  state = OUT_1;
                        8'H8D :  state = IN_1;                  
                        8'h8E :  state = CMP; 
                        //8'H8F :  state = TEST_1
                        8'h9F :  state = ORHI_1;
                  
                     // 3. Single Op Instr
                        8'h90 :  state = POP_1;
                        8'h91 :  state = PUSH_1;
                        8'h92 :  state = NEG;
                        8'h93 :  state = NOT;
                        8'h94 :  state = INC; 
                        8'h95 :  state = DEC;
                        //8'h98 :  state = LSR; 
                        8'h99 :  state = LSL; 
                        8'h9A :  state = ASHR;
                        8'h9B :  state = ASHL;
                        8'h9C :  state = ROR;
                        8'h9D :  state = ROL; 
                        
               // 4. Conditional Jumps
                        8'hA0 :  state = JC;
                        8'hA1 :  state = JNC;
                        8'hA2 :  state = JZ;
                        8'hA3 :  state = JNZ;
                        8'hA4 :  state = JN;
                        8'hA5 :  state = JP;                         
                        8'hA6 :  state = JO; 
                        8'hA7 :  state = JNO; 
                        
                        8'hA8 :  state = JLT;
                        8'hA9 :  state = JGE; 
                        8'hAA :  state = JGT; 
                        8'hAB :  state = JLE; 
                        8'hAC :  state = JC; //JB 
                        8'hAD :  state = JNC;//JAE 
                        8'hAE :  state = JA; 
                        8'hAF :  state = JBE; 
                  
               // 5. UnConditional Jumps
                        8'hB0 :  state = J;
                        8'hB1 :  state = JR; 
                        8'hB2 :  state = CALL_1;
                        8'hB3 :  state = RCALL_1;
                        8'hB4 :  state = RET_1;
                        8'hB5 :  state = RETI_1;  
               
               // 6. Flags/Processor Instr
                        8'hC0 :  state = CLC;
                        8'hC1 :  state = STC;
                        //8'hC2 :  state = ;
                        8'hC3 :  state = CLRINT;
                        8'hC4 :  state = STINT;
                  
                        8'hC6 :  state = NOP; 
                        8'hC7 :  state = LdSP;
                        8'hCF :  state = HALT; 
                  
               // 7. Immediate Op Instr
                        8'hD0 :  state = ADDi;
                        8'hD1 :  state = SUBi;
                        8'hD2 :  state = MULi_1; 
                        8'hD3 :  state = DIVi_1; 
                        //8'hD4 :  state = ANDi;
                        //8'hD5 :  state = ORi;
                        //8'hD6 :  state = XORi; 
                        8'hDB :  state = CMPi; 
                        ///8'hDC :  state = TESTi; 
                  
               // 8. Floating Point Instr
                        8'hE0 :  state = FADD; 
                        8'hE1 :  state = FSUB; 
                        8'hE2 :  state = FMUL; 
                        8'hE3 :  state = FDIV; 
                        8'hE4 :  state = FINC; 
                        8'hE5 :  state = FDEC; 
                        8'hE6 :  state = FZERO; 
                        8'hE7 :  state = FONE; 
                        8'hE8 :  state = FLDI_1; 
                        8'hE9 :  state = FLDR_1; 
                        8'hEA :  state = FSTO_1; 
                        8'hEF :  state = FORHI_1; 
                        
                    // DEFAULT CASE
                        default : state = ILLEGAL_OP; 
                    endcase                    
                end
                
            INTR_1:
                begin
                @(negedge sys_clk)
                /************************************************************/
                // IP gets address of interrupt vector; prepare to push IP
                //             IP <- 0x3FF;  Wrbuf0<-IP; SP<-SP-1; IE <- 0 
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111;
                    
                    fpBuf_Ctrl = 4'b0;                     
                    wrBuf_Ctrl = 5'b1_01_00;             //Wrbuf0<-IP
                    rdBuf_Ctrl = 6'b0; 
                    
                    addr_Ctrl  = 9'b100_0_01_0_00;  //IP <- 0x3FF; SP<-SP-1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                    {SP_sel, SP_ld} = 2'b00;                     
                    NS_Istat[4]  = 1'b0;           //IE FLAG = 0
                    NS_FPstat = PS_FPstat;
                    
                    state = INTR_2;               
                end

                    
            INTR_2:
                begin
                @(negedge sys_clk)
                /************************************************************/
                // push IP; Read address of ISR into RdBuf0
                // dM[sp] <- Wrbuf0; rdBuf0 < - iM[IP]
                // sp <- sp -1
                // WrBuf1 <- CPSR
                /************************************************************/
                    imem_Ctrl  = 3'b001;                         //iMem read en
                    dmem_Ctrl  = 3'b010;                         //dMem write en
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_10_01;                     //wrbuf0 oe = 1
                    Wrbuf1_sel = 1'b1; 
                    rdBuf_Ctrl = 6'b01_01;                          //rdbuf0_ld & rdmux0 = 1
                    
                    addr_Ctrl  = 9'b0000_01_1_00;             //dMem_addr_sel = 1; SP_dec = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat  = PS_Istat;                         //Flags remain same
                    NS_FPstat = PS_FPstat;
                    
                    state = INTR_3;
               
                end
                                
            INTR_3:
                begin
                @(negedge sys_clk)
                /************************************************************/
                // Reload IP with address of ISR; ack the intr; gointo FETCH
                // IP <- RdBuf0; int_ack <- 1
                // dM{sp} <- wrBuf1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_10;           //wrbuf0 oe = 1
                    Wrbuf1_sel = 1'b0; 
                    rdBuf_Ctrl = 6'b01_00_00;          //RdBuf0_oe =1
                    addr_Ctrl  = 9'b0100_00_1_00;      //IP_ld =1; dMem_addr_sel = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b100 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;             // same Flags
                    NS_FPstat  = PS_FPstat;
                    
                    int_ack = 1'b1; 
                    
                    state = FETCH;  
                end            
            ADD:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] + R[s2]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                        //W_en=1
                    idp_Ctrl[6:0] = 7'b0_00111_0;                // ALU_op = ADD
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;               
                    state = FETCH;
                    #1 

                       NS_Istat[3:0]   = {c, n, z, o};            //update flags 
                       NS_FPstat  = PS_FPstat;                                                   
            end
            
            SUB:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] - R[s2]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                      //W_en=1
                    idp_Ctrl[6:0] = 7'b0_00110_0;                 // ALU_op = SUB
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};             //update flags 
                       NS_FPstat  = PS_FPstat;                                              
            end
                
            MUL_1:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] * R[s2] (LSW of product)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                      //W_en=1
                    idp_Ctrl[6:0] = 7'b0_00010_0;              // ALU_op = MUL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;               
                    state = MUL_2;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};         //update flags 
                       NS_FPstat  = PS_FPstat;             
                end
                
            MUL_2:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d + 1] <- R[s1] * R[s2]  (MSW of product)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[21:17] = idp_Ctrl[21:17] + 1;                 //dest = dest + 1
                    idp_Ctrl[6:0] = 7'b0_00011_0;                             // ALU_op = MSW of MUL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                    
                end
            DIV_1:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] / R[s2] (Quotient)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[6:0] = 7'b0_00100_0;                             // ALU_op = DIV
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = DIV_2;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                        
                end
                
            DIV_2:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d + 1] <- R[s1] / R[s2]  (Remainder of Quotient)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[21:17] = idp_Ctrl[21:17] + 1;                 //dest = dest + 1
                    idp_Ctrl[6:0] = 7'b0_00101_0;                             // ALU_op = REM of DIV
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                    
    
                end
                
                
            XOR:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] ^ R[s2]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[6:0] = 7'b0_01010_0;                             // ALU_op = XOR
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                    
                end
                
                
                
            LDR_1:
                begin
                /************************************************************/
                //mar <-R[$r] (REGOUT)
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;     
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_00_0_10;         //MAR_ld = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = LDR_2;
                    
                end
                
            LDR_2:
                begin
                /************************************************************/
                //RdBuf0 <- dM[mar]
                //mar <- mar + 1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001;                         //dcs = 0 drd= 0
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b00_01_00;                //RdBuf0_ld = 1 
                    addr_Ctrl  = 9'b0000_00_0_01;         //MAR_inc = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = LDR_3;                    
                end
                
            LDR_3:
                begin
                /************************************************************/
                //RdBuf1 <- dM[mar]
                //
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b11_10_00;                 //RdBuf_oe = 2'b11; RdBuf1_ld = 1 
                    addr_Ctrl  = 9'b0; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                //W_en = 1; 
                    idp_Ctrl[6:0] = 7'b0_00000_1;            //Y_sel = 1
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = LDR_4;                     
                    
                    
                end
                
         /*
            LDR works with 3 states by settiig up 2 instances of #1 on rdbuf_Ctrl 
            and before the end statement        
        */        
            LDR_4:
                begin
                /************************************************************/
                // R[dest] <- RdBuf1 
                //
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b11_00_00;                 //RdBuf_oe = 2'b11; 
                    addr_Ctrl  = 9'b0; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                //W_en = 1; 
                    idp_Ctrl[6:0] = 7'b0_00000_1;            //Y_sel = 1
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;    
                  
                end
            
            
            LDI_1:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //Control Word: RdBuf0 <- iM[IP]; IP <- IP + 1
                /************************************************************/
                    imem_Ctrl  = 3'b001; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b00_01_01;                   //RdBuf_ld0 =1; Rdmux0 =1
                    addr_Ctrl  = 9'b001_0_00_0_00;           //IP_inc = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = LDI_2;
                end
            
            LDI_2:
                begin
                /************************************************************/
                //R[d] <- {32'h0, RdBuf0}
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b01_00_00;                     //RdBuf0_oe = 1
                    addr_Ctrl  = 9'b000_0_00_0_00; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;
                    idp_Ctrl[6:0] = 7'b0_00000_1;                //Ysel =1            
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                   {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;
                    
                end
            ORHI_1:
                begin
                /************************************************************/
                // RdBuf1 <-iM[IP], IP <- IP + 1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b001; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b00_10_10;                // RdBuf1_ld = 1; Rdmux1 =1
                    addr_Ctrl  = 9'b001_0_00_0_00;         //IP_inc = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b10;
                    idp_Ctrl[16:12] = idp_Ctrl[21:17];    //R_addr = W_addr
                    idp_Ctrl[6:0] = 7'b1_01001_0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = ORHI_2;
                end

            ORHI_2:
                begin
                /************************************************************/
                // R[d] <- R[d] | {RdBuf1, 32'b0}
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b10_00_00;              //RdBuf1_oe = 1
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b11;
                    idp_Ctrl[6:0] = 7'b1_01001_0;                                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;
                end
                
            STO_1:
                begin
                /************************************************************/
                // MAR <- R[d]; {Wrbuf0} <- R[s1]
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_11_00;                      //WrBuf[0/1]_ld = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b000_0_00_0_10;            //MAR_ld = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];         //S_addr = R_addr 
                    idp_Ctrl[16:12] = idp_Ctrl[21:17];          //R_addr = W_addr
                    idp_Ctrl[6:0] = 7'b0_10100_0;                //pass S        
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = STO_2;
               
                end

            STO_2:
                begin
                /************************************************************/
                // dM[mar] <- WrBuf0; {WrBuf1} <- R[s1] ;MAR <- MAR +1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_01;                     //  WrBuf0_oe=1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b000_0_00_0_01;             // MAR_inc =1;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = STO_3;                
                end
                
            STO_3:
                begin
                /************************************************************/
                //dM[mar] <-WrBuf01
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_10;                     //WrBuf1_oe = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;               
                end
         
         COPY:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     // W_en=1                    
                    idp_Ctrl[6:0] = 7'b0_10011_0;                             // ALU_op = PASS R
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;
                end    
            
         EXCH_1: 
            begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    
                    idp_Ctrl[23:22] = 2'b01;                                     // W_en=1   
                    idp_Ctrl[6:0] = 7'b0_01010_0;                             // ALU_op = R XOR S
                    idp_Ctrl[21:17] = idp_Ctrl[16:12] ;                 //W_addr = R_addr
               
               {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     
                    NS_FPstat  = PS_FPstat;
                    
                    state = EXCH_2;                   
            end
         EXCH_2: 
            begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
               
                    idp_Ctrl[23:22] = 2'b01;                                     // W_en=1                    
                    idp_Ctrl[6:0] = 7'b0_01010_0;                             // ALU_op = R XOR S
                    idp_Ctrl[21:17] = idp_Ctrl[11:7];                //W_addr = S_addr
               
               
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     
                    NS_FPstat  = PS_FPstat;
                    
                    state = EXCH_3;                   
            end   
         EXCH_3: 
            begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     // W_en=1                    
                    idp_Ctrl[6:0] = 7'b0_01010_0;                             // ALU_op = R XOR S
                    idp_Ctrl[21:17] = idp_Ctrl[16:12];                  //W_addr = R_addr
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;
                   
            end
         OUT_1:
                begin
                /************************************************************/
                // MAR <- R[d]; {Wrbuf1,Wrbuf0} <- R[s1]
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    IOmem_Ctrl = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_11_00;                      //WrBuf[0/1]_ld = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b000_0_00_0_10;            //MAR_ld = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];         //S_addr = R_addr 
                    idp_Ctrl[16:12] = idp_Ctrl[21:17];          //R_addr = W_addr
                    idp_Ctrl[6:0] = 7'b0_10100_0;                //pass S        
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = OUT_2;
               
                end

            OUT_2:
                begin
                /************************************************************/
                // dM[mar] <- WrBuf0; {WrBuf1} <- R[s1] ;MAR <- MAR +1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
               IOmem_Ctrl  = 3'b010;
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_01;                     //  WrBuf0_oe=1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b000_0_00_0_01;             // MAR_inc =1;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = OUT_3;
 
                end
                
            OUT_3:
                begin
                /************************************************************/
                //dM[mar] <-WrBuf01
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    IOmem_Ctrl  = 3'b010;
               
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_10;                     //WrBuf1_oe = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;

                end
            
         IN_1:
                begin
                /************************************************************/
                //mar <-R[$r] (REGOUT)
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    IOmem_Ctrl  = 3'b111;
               
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;     
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_00_0_10;         //MAR_ld = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = IN_2;
                    
                end
                
            IN_2:
                begin
                /************************************************************/
                //RdBuf0 <- dM[mar]
                //mar <- mar + 1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
               dmem_Ctrl  = 3'b111;
                    IOmem_Ctrl  = 3'b001;                         //IOcs = 0 IOrd= 0
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b00_01_00;                //RdBuf0_ld = 1 
                    addr_Ctrl  = 9'b0000_00_0_01;         //MAR_inc = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = IN_3;            

                end
                
            IN_3:
                begin
                /************************************************************/
                //RdBuf1 <- dM[mar]
                //
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111;
                    IOmem_Ctrl  = 3'b001; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b11_10_00;                 //RdBuf_oe = 2'b11; RdBuf1_ld = 1  
                    addr_Ctrl  = 9'b0; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                //W_en = 1; 
                    idp_Ctrl[6:0] = 7'b0_00000_1;            //Y_sel = 1
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = IN_4;                                         
                end
                        
            IN_4:
                begin
                /************************************************************/
                // R[dest] <- RdBuf1 
                //
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    IOmem_Ctrl  = 3'b111;
               
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b11_00_00;                 //RdBuf_oe = 2'b11; 
                    addr_Ctrl  = 9'b0; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                //W_en = 1; 
                    idp_Ctrl[6:0] = 7'b0_00000_1;            //Y_sel = 1
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                  
                  
                end
         
            CMP:
                begin
                @(negedge sys_clk)
                /************************************************************/
                // R[IR[s1]] - R[IR[s2]]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;                                                         
                    idp_Ctrl[6:0] = 7'b0_00110_0;                             // ALU_op = SUB
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = FETCH;
                    
                    #1 NS_Istat[3:0]   = {c, n, z, o};                        //update flags
                       NS_FPstat  = PS_FPstat;                
                        
                end
         
         POP_1:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        RdBuf0 <- M[SP]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001;    //dCS = 0; dRD = 0        
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b00_01_00; //RdBuf0_ld = 1
                    addr_Ctrl  = 9'b0000_10_1_00;   //Dmem_addr_sel = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = POP_2;                                        
                end
            
         POP_2:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        RdBuf1 <- M[SP]
            //    SP <- SP + 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001;    //dCS = 0; dRD = 0        
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b00_10_00;           //RdBuf1_ld = 1
                    addr_Ctrl  = 9'b0000_10_1_00;       //SP_inc = 1 ; Dmem_addr_sel = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = POP_3;        
                end
            
         POP_3:
                begin
                /************************************************************/
                //   R[dest] <- RdBuf 
                //   SP <- SP + 1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b11_00_00;                 //RdBuf_oe = 2'b11; 
                    addr_Ctrl  = 9'b0000_00_0_00;       //SP_inc = 1 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                //W_en = 1; 
                    idp_Ctrl[6:0] = 7'b0_00000_1;            //Y_sel = 1
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;            
                end
            
            PUSH_1:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        SP <- SP - 1
                //        WrBuf <- R[s1]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_11_00;             //    WrBuf_ld = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_01_0_00;        //SP_dec = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[11:7] = idp_Ctrl[16:12]; //S_addr = R_addr
                    idp_Ctrl[6:0] = 7'b0_10100_0;     // ALU_op = Pass S 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = PUSH_2;                                                                
                end
                
            PUSH_2:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        M[SP] <- WrBuf1 ; SP < SP - 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010;             //dCS = 0 ; dWR = 0; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_10;         //WrBuf1_oe = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_01_1_00;        //SP_dec =1 ; Dmem_addr_sel = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = PUSH_3;        
              
                end
            
         PUSH_3:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        M[SP] <- WrBuf0
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010;             //dCS = 0 ; dWR = 0;         
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_01;         //WrBuf0_oe = 1  
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_00_1_00;        //Dmem_addr_sel = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                                                          
                    
                end
         
            NEG:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- ~R[IR[s1]] + 1 ( Two's Compliment )
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];                         // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_01100_0;                             // ALU_op = NEG
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = FETCH;
                    
                    #1 NS_Istat[3:0]   = {c, n, z, o};                         //update flags
                       NS_FPstat  = PS_FPstat;                
                end
         
         NOT:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- ~R[IR[s1]]  ( Ones's Compliment )
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];                         // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_01011_0;                             // ALU_op = NOT
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = FETCH;
                    
                    #1 NS_Istat[3:0]   = {c, n, z, o};                        //update flags
                       NS_FPstat  = PS_FPstat;        
                end
                
            INC:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[IR[d]] <- R[IR[s1]] + 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];                         // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_00001_0;                             // ALU_op = INC S
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                                        
                    state = FETCH;
                    
                    #1 NS_Istat[3:0]   = {c, n, z, o};                        //update flags
                       NS_FPstat  = PS_FPstat;                
                end
                
            DEC:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s2] - 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     // W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];                         // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_00000_0;                             // ALU_op = DEC S
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                   {SP_sel, SP_ld} = 2'b00; 
                    state = FETCH;
                                        
                    #1 NS_Istat[3:0]   = {c, n, z, o};                         // 
                        NS_FPstat  = PS_FPstat;                      
                      
                end
                        
            LSL:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[IR[s1]] << 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];                         // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_01101_0;                             // ALU_op = LSL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;                 
                    {SP_sel, SP_ld} = 2'b00; 
                    state = FETCH;
                    #1 NS_Istat[3:0]   = {c, n, z, o};                         // update flags
                        NS_FPstat  = PS_FPstat;
                end
                
            ASHR:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[IR[s1]] >>> 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];                         // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_10000_0;                             // ALU_op = ASHR
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;                 
                    {SP_sel, SP_ld} = 2'b00; 
                    state = FETCH;
                    #1 NS_Istat[3:0]   = {c, n, z, o};                         // 
                        NS_FPstat  = PS_FPstat;
                end
            
         ASHL:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[IR[s1]] <<< 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];                         // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_01111_0;                             // ALU_op = ASHL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;                 
                    {SP_sel, SP_ld} = 2'b00; 
                    state = FETCH;
                    #1 NS_Istat[3:0]   = {c, n, z, o};                         // 
                        NS_FPstat  = PS_FPstat;
                
                end
            
         ROR:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[IR[s1]] ROTATE RIGHT
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                        //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];            // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_10110_0;                // ALU_op = ASHL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;      
                    {SP_sel, SP_ld} = 2'b00; 
                    state = FETCH;
                    #1 NS_Istat[3:0]   = {c, n, z, o};            // 
                        NS_FPstat  = PS_FPstat;
                
                end
        
         ROL:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[IR[s1]] ROTATE LEFT
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                         //W_en=1
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];             // S_addr = Raddr
                    idp_Ctrl[6:0] = 7'b0_10101_0;                 // ALU_op = ASHL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;       
                    {SP_sel, SP_ld} = 2'b00; 
                    state = FETCH;
                    #1 NS_Istat[3:0]   = {c, n, z, o};             // 
                        NS_FPstat  = PS_FPstat;
                
                end         
         JC:
                if(PS_Istat[3] == 1) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(C) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        state = FETCH;                        
                    end

            JNC:
                if(PS_Istat[3] != 1) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(!C) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat; 
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        state = FETCH;                        
                    end
                    
            JZ:
                if(PS_Istat[1] == 1) //if(PS_Istat[2] == 1)
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(Z) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end                    
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
            JNZ:
                if(PS_Istat[1] != 1) //if(PS_Istat[2] != 1)
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(!Z) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end                    
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
            JN:
                if(PS_Istat[2] == 1 )
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(N) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
            JP:
                if(PS_Istat[2] != 1 && PS_Istat[1] != 1) //if(n != 1 & z != 1)
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(!N) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
         JO:
                if(PS_Istat[0] == 1 ) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(OV) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
            JNO:
                if(PS_Istat[0] != 1)
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(!OV) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
                    
         JLT:
                if(PS_Istat[2] != PS_Istat[0] ) //if(n ==1)
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(!N) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
           JGE:
                if(PS_Istat[2] == PS_Istat[0]) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(N == V) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;                    
                        state = FETCH;                        
                    end
            JGT:
                if(PS_Istat[2] == PS_Istat[0] && PS_Istat[1] == 0) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(Z == 0 & N == V) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end        
            JLE:
                if(PS_Istat[2] != PS_Istat[0] || PS_Istat[1] == 1) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(Z == 1 || N != V) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
            JA:
                if(PS_Istat[3] == 0  || PS_Istat[1] == 0) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(Z == 0 || C == 0) 
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
                    
            JBE:
                if(PS_Istat[3] == 1 || PS_Istat[1] == 1) 
                   begin
                    @(negedge sys_clk)
                    /************************************************************/
                    // if(Z == 1 || C == 1)  
                    //        IP <- {IP[31:28], IR[25:0], 2'b00}
                    /************************************************************/
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0100_00_0_00;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;             
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                                            
                        
                    end
              else 
                    begin
                    @(negedge sys_clk)
                        imem_Ctrl  = 3'b111; 
                        dmem_Ctrl  = 3'b111; 
                        
                        fpBuf_Ctrl = 4'b0; 
                        wrBuf_Ctrl = 5'b0; 
                        rdBuf_Ctrl = 6'b0; 
                        addr_Ctrl  = 9'b0;
                        
                        fpdp_Ctrl  = 21'b0; 
                        idp_Ctrl[23:22] = 2'b00;     
                        idp_Ctrl[6:0] = 7'b0_00000_0; 
                        
                        {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                        {SP_sel, SP_ld} = 2'b00; 
                        NS_Istat   = PS_Istat ;        
                        NS_FPstat  = PS_FPstat;
                        
                        state = FETCH;                        
                    end
            
            J:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        IP <- {IP[31:28], IR[25:0], 2'b00}
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0100_00_0_00;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                                            
                    
                end
            JR:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        IP <- R[IR[S1]] (Raddr)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0100_00_0_00;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b011 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                                            
                    
                end
                
            CALL_1:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        SP <- SP - 1
                //        WrBuf0 <- IP
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b1_01_00;             //    WrBuf0_sel = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_01_0_00;        //SP_dec = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = CALL_2;                                                            
                end
                
            CALL_2:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        SP <- SP - 1 ** IP stored in LSW of stack block
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111;            
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;        
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_01_0_00;        //SP_dec = 1  
                                                //**32bit IP stored in LSW of 64bit in stack block. 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = CALL_3;                                         
                end
            
         CALL_3:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        IP <- IP + IR[25:0]
                //    M[SP] <- WrBuf0   
                //    ORIGINAL IP ADDRESS STORED IN LSW OF 64bit STACK BLOCK
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010;        
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_01;        //WrBuf0_oe = 1 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0100_00_1_00;   //IP_ld = 1 ; 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b001 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                                            
               
                end
         
         RCALL_1:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        SP <- SP - 1
                //        WrBuf0 <- IP
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b1_01_00;             //    WrBuf0_sel = 1
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_01_0_00;        //SP_dec = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = RCALL_2;                                            
                     
                end
                
            RCALL_2:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        SP <- SP-1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111;             
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;         
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_01_0_00;        //SP_dec = 1; //*****SP_dec MODIFED FROM MOD6
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[11:7] = idp_Ctrl[16:12]; //S_addr = R_addr
                    idp_Ctrl[6:0] = 7'b0_10100_0;     // ALU_op = Pass S
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = RCALL_3;    
                  
                end
    
         RCALL_3:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //       IP <- IP + R[s1] (ALUOUT[31:0])
                //       M[SP] <- WrBuf0
                /************************************************************/
                    imem_Ctrl  = 3'b111;            
                    dmem_Ctrl  = 3'b010;             //dCS = 0 ; dWR = 0; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_01;         //WrBuf0_oe
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0100_00_1_00;   //IP_ld = 1 dMem_sel = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b011 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                                            
                    
                end           
         
         RET_1:        //RETI_3        
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        RdBuf0 <-M[SP] 
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001;    //dCS = 0; dRD = 0        
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b00_01_00;       //RdBuf0_ld = 1
                    addr_Ctrl  = 9'b0000_10_1_00;   // Dmem_addr_sel = 1 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = RET_2;                                                           
                end
            
         RET_2:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        IP <- RdBuf0; 
                //        SP <- SP + 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111;           
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b01_00_00;           //RdBuf0_oe = 1
                    addr_Ctrl  = 9'b0100_10_0_00;       //IP_ld = 1; SP_inc = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b100 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                       
                end
         
         RETI_1:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        RdBuf0 <-M[SP] 
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001;    //dCS = 0; dRD = 0        
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b00_01_00;       //RdBuf0_ld = 1
                    addr_Ctrl  = 9'b0000_00_1_00;   // Dmem_addr_sel = 1 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = RETI_2;                     
                end
            
         RETI_2:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        ALUFLAGS <- RdBuf0(3:0)
                //    FPFLAGS <- RdBuf0(10:5) 
                //    SP <- SP + 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111;           
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b01_00_00;           //RdBuf0_oe = 1
                    addr_Ctrl  = 9'b0000_10_0_00;       // SP_inc = 1
                    
                    {fpdp_Ctrl[20],                               //FP_sel = 1
                     fpdp_Ctrl[4:0]}  = 6'b0_1_1111;       //FP_OP= LOADFLAGS
                
                    idp_Ctrl[23:22] = 2'b10;            //DS_sel = 1
                    idp_Ctrl[6:0] = 7'b1_10111_0;      //ALUOP = LOADFLAGS; S_sel = 1
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                                        
                    state = RETI_3;
               
               #1 NS_Istat[3:0]   = {c, n, z, o};      //UPDATE FLAGS        
                  NS_FPstat  = FP_status;                 
                end
         RETI_3:        //RETI_3        
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        RdBuf0 <-M[SP] 
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001;    //dCS = 0; dRD = 0        
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b00_01_00;       //RdBuf0_ld = 1
                    addr_Ctrl  = 9'b0000_00_1_00;   // Dmem_addr_sel = 1 
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = RETI_4;                                             
                end
            
         RETI_4:                
                begin
                @(negedge sys_clk)
                /************************************************************/                 
                //        IP <- RdBuf0; 
                //        SP <- SP + 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111;           
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0_00_00;  
                    rdBuf_Ctrl = 6'b01_00_00;           //RdBuf0_oe = 1
                    addr_Ctrl  = 9'b0100_10_0_00;       //IP_ld = 1; SP_inc = 1
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0_00000_0; 
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b100 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat ;             
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                      
                end
         
           CLC:
                begin
                /****************************************************************
                //         CLEAR CARRY 
                ****************************************************************/
                @(negedge sys_clk)                        
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat[3]         = 1'b0;             //CLEAR CARRY FLAG
                    NS_Istat[2:0]   = PS_Istat[2:0]; //Flags remain same                    
                    NS_FPstat  = PS_FPstat;
                    state = FETCH; 
                    
                end
            STC:
                begin
                /****************************************************************
                //         SET CARRY 
                ****************************************************************/
                @(negedge sys_clk)                        
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat[3]         = 1'b1;             //SET CARRY FLAG
                    NS_Istat[2:0]   = PS_Istat[2:0]; //Flags remain same                    
                    NS_FPstat  = PS_FPstat;
                    state = FETCH; 
                end
         CLRINT:
                begin
                /****************************************************************
                //         SET CARRY 
                ****************************************************************/
                @(negedge sys_clk)                        
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat[4]         = 1'b0;             //CLEAR INTR ENABLE FLAG
                    NS_Istat[2:0]   = PS_Istat[2:0]; //Flags remain same                    
                    NS_FPstat  = PS_FPstat;
                    state = FETCH; 
                
                end
         STINT:
                begin
                /****************************************************************
                //         SET CARRY 
                ****************************************************************/
                @(negedge sys_clk)                        
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat[4]         = 1'b1;             //SET INTR ENABLE FLAG
                    NS_Istat[2:0]   = PS_Istat[2:0]; //Flags remain same                    
                    NS_FPstat  = PS_FPstat;
                    state = FETCH; 
                end
            NOP:
                begin
                /****************************************************************
                //         NO OPERATION 
                ****************************************************************/
                @(negedge sys_clk)                        
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    state = FETCH; 
                end
                
            LdSP:
                begin
                /****************************************************************
                //         Load Stack Pointer
                ****************************************************************/
                @(negedge sys_clk)                        
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                    {SP_sel, SP_ld} = 2'b01; 
                     state = FETCH; 
                end
                
            HALT:
                begin
                @(negedge sys_clk)
                    $display("HALT INSTRUCTION FETCHED %t", $time);                        
                    // control word assignments for "deasserting" everything
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    Dump_IReg;
                    Dump_Mem;
               //Dump_FPR;
                    $finish;
                end
            
         ADDi:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] + IR[15:8] (8bit immediate value)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[6:0] = 7'b1_00111_0;                             //S_sel = 1; ALU_op = ADD
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                                                    
            
            end 
            
         SUBi:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] - IR[15:8] (8bit immediate value)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[6:0] = 7'b1_00110_0;                             //S_sel = 1; ALU_op = SUB
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                                   
            end 
            
         MULi_1:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] * IR[15:8] (LSW of product)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[6:0] = 7'b1_00010_0;                             //S_sel = 1; ALU_op = MUL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;               
                    state = MULi_2;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                    
                end
                
            MULi_2:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d + 1] <- R[s1] * IR[15:8] (MSW of product)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[21:17] = idp_Ctrl[21:17] + 1;                 //dest = dest + 1
                    idp_Ctrl[6:0] = 7'b0_00011_0;                             // ALU_op = MSW of MUL
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;
                    se64IR_sel = 1'b0;                       
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                        //update flags 
                       NS_FPstat  = PS_FPstat;                                    
                end
            DIVi_1:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d] <- R[s1] / IR[15:8] (Quotient)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[6:0] = 7'b1_00100_0;                             //S_sel = 1 ALU_op = DIV
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
                    state = DIVi_2;
               
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                        //update flags 
                       NS_FPstat  = PS_FPstat;                                                         

                end
                
            DIVi_2:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //R[d + 1] <- R[s1] / IR[15:8]  (Remainder of Quotient)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b01;                                     //W_en=1
                    idp_Ctrl[21:17] = idp_Ctrl[21:17] + 1;                 //dest = dest + 1
                    idp_Ctrl[6:0] = 7'b0_00101_0;                             // ALU_op = REM of DIV
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;                     
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                         //update flags 
                       NS_FPstat  = PS_FPstat;                                    
    
                end
            CMPi:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  R[s1] - IR[15:8] (8bit immediate value)
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    fpdp_Ctrl  = 21'b0; 
                    idp_Ctrl[23:22] = 2'b00;                                     
                    idp_Ctrl[6:0] = 7'b1_00110_0;                             //S_sel = 1; ALU_op = SUB
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat[3:0]   = {c, n, z, o};                        //update flags 
                       NS_FPstat  = PS_FPstat;                                               
            end 
                
          FADD:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = FPR[s1] + FPR[s2]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                                        //FPW_en = 1
                     fpdp_Ctrl[4:0]}  = 6'b1_0_0010;                 //FP_OP= ADD
                    
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated                
            end
               
         FSUB:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = FPR[s1] - FPR[s2]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                                     // FPW_en = 1
                     fpdp_Ctrl[4:0]}  = 6'b1_0_0011;             // FP_OP= SUB
                    
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated                
            end
            
                
         FMUL:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = FPR[s1] * FPR[s2
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                                     //FPW_en = 1;
                     fpdp_Ctrl[4:0]}  = 6'b1_0_0101;            //FP_OP= MUL
                        
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated                
            end
                
                
         FDIV:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = FPR[s1] / FPR[s2]
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                                      //FPW_en = 1;
                     fpdp_Ctrl[4:0]}  = 6'b1_0_0110;              //FP_OP= DIV
                    
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated    
                                
            end    
                
                
         FINC:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = FPR[s1] + 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                                     //FPW_en = 1 
                     fpdp_Ctrl[4:0]}  = 6'b1_0_1010;             //FP_OP= INC
                    
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated                
            end    
                
                
         FDEC:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = FPR[s1] - 1
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                                     //FPW_en = 1;
                     fpdp_Ctrl[4:0]}  = 6'b1_0_1100;              //FP_OP = DEC
                    
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated                
            end    
                
                
         FZERO:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = 0.0
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                                     //FPW_en = 1
                     fpdp_Ctrl[4:0]}  = 6'b1_0_1000;             // FP_OP = ZERO
                    
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated                
            end    
                
                
                
         FONE:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //  FPR[dest] = 1.0
                /************************************************************/
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                             //FPW_en = 1;
                     fpdp_Ctrl[4:0]}  = 6'b1_0_1001;    // FP_OP= ONE
                    
                    idp_Ctrl[23:22] = 2'b0;                                     
                    idp_Ctrl[6:0] = 7'b0;                         
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00;     
                    se64IR_sel = 1'b0;
               
                    state = FETCH;
                    #1 
                       NS_Istat   = PS_Istat;                         //INT Flags remain same
                       NS_FPstat  = FP_status;                        //FP Flags updated                
            end
                
           FLDI_1:
                begin
                @(negedge sys_clk)
                /************************************************************/
                //Control Word: RdBuf0 <- iM[IP]; IP <- IP + 1
                /************************************************************/
                    imem_Ctrl  = 3'b001; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b00_01_01;                   //RdBuf_ld0 =1; Rdmux0 =1
                    addr_Ctrl  = 9'b001_0_00_0_00;           //IP_inc = 1
                    
                    {fpdp_Ctrl[20],                         
                     fpdp_Ctrl[4:0]}  = 6'b0;
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FLDI_2;
                end
            
            FLDI_2:
                begin
                /************************************************************/
                //FPR[dest] <- {32'h0, RdBuf0}
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b01_00_00;                     //RdBuf0_oe = 1
                    addr_Ctrl  = 9'b000_0_00_0_00; 
                    
                    {fpdp_Ctrl[20],                               //FPW_en = 1
                     fpdp_Ctrl[4:0]}  = 6'b1_1_0000;       //FP_OP= PASS S
                    
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;                            
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ;
                   {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;
                    
                end    
                
         FLDR_1:
                begin
                /************************************************************/
                //mar <-R[s1] (REGOUT)
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;     
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0000_00_0_10;         //MAR_ld = 1
                    
                    {fpdp_Ctrl[20],                         
                     fpdp_Ctrl[4:0]}  = 6'b0;
                    
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FLDR_2;
                    
                end
                
            FLDR_2:
                begin
                /************************************************************/
                //RdBuf0 <- dM[mar]
                //mar <- mar + 1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001;                         //dcs = 0 drd= 0
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b00_01_00;                //RdBuf0_ld = 1 
                    addr_Ctrl  = 9'b0000_00_0_01;         //MAR_inc = 1
                    
                    {fpdp_Ctrl[20],                         
                     fpdp_Ctrl[4:0]}  = 6'b0;
                    
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FLDR_3;                    
                end
                
            FLDR_3:
                begin
                /************************************************************/
                //  RdBuf1 <- dM[mar]
                //
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b001; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b11_10_00;                    //RdBuf1_ld = 1
                    addr_Ctrl  = 9'b0; 
                    
                    {fpdp_Ctrl[20],                               //FPW_en = 1
                     fpdp_Ctrl[4:0]}  = 6'b1_1_0000;       //FP_OP= PASS S
                     
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FLDR_4;                                                        
                end
                    
            FLDR_4:
                begin
                /************************************************************/
                // FPR[dest] <- {RdBuf1, RdBuf0}
                //
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0;
                    rdBuf_Ctrl = 6'b11_00_00;                 //RdBuf_oe = 2'b11; 
                    addr_Ctrl  = 9'b0; 
                    
                    {fpdp_Ctrl[20],                               //FPW_en = 1
                     fpdp_Ctrl[4:0]}  = 6'b1_1_0000;       //FP_OP= PASS S
                     
                    idp_Ctrl[23:22] = 2'b0;                
                    idp_Ctrl[6:0] = 7'b0;            
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;                                          
                end
            
            FSTO_1:
                begin
                /************************************************************/
                //   MAR <- R[d] (REGOUT) 
                //   {FPBuf1, FPBuf0} <- FPR[s1]
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b11_00;                         //FPBuf[0/1]_ld = 1
                    wrBuf_Ctrl = 5'b0;                      
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b000_0_00_0_10;            //MAR_ld = 1
                    
                    {fpdp_Ctrl[20],                              
                     fpdp_Ctrl[4:0]}  = 6'b0_0_0001;       //FP_OP= PASS R 
                     
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[11:7] = idp_Ctrl[16:12];         //S_addr = R_addr 
                    idp_Ctrl[16:12] = idp_Ctrl[21:17];      //R_addr = W_addr
                    idp_Ctrl[6:0] = 7'b0_10100_0;                //pass S        
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FSTO_2;
               
                end

            FSTO_2:
                begin
                /************************************************************/
                // dM[mar] <- FPBuf0
                // MAR <- MAR +1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010; 
                    
                    fpBuf_Ctrl = 4'b00_01;                     //  FPBuf0_oe=1
                    wrBuf_Ctrl = 5'b0;                     
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b000_0_00_0_01;             // MAR_inc =1;
                    
                    {fpdp_Ctrl[20],                         
                     fpdp_Ctrl[4:0]}  = 6'b0;
                    
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;    
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FSTO_3;
              
                end
                
            FSTO_3:
                begin
                /************************************************************/
                //dM[mar] <-FPBuf01
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b010; 
                    
                    fpBuf_Ctrl = 4'b00_10;             //FPBuf1_oe = 1
                    wrBuf_Ctrl = 5'b0;                     
                    rdBuf_Ctrl = 6'b0; 
                    addr_Ctrl  = 9'b0; 
                    
                    {fpdp_Ctrl[20],                         
                     fpdp_Ctrl[4:0]}  = 6'b0;
                    
                    idp_Ctrl[23:22] = 2'b00;
                    idp_Ctrl[6:0] = 7'b0;
                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                         //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;               
                end
            
            FORHI_1:
                begin
                /************************************************************/
                // RdBuf1 <-iM[IP], IP <- IP + 1
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b001; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b00_10_10;                // RdBuf1_ld = 1; Rdmux1 =1
                    addr_Ctrl  = 9'b001_0_00_0_00;         //IP_inc = 1
                    
                    {fpdp_Ctrl[20],                         
                     fpdp_Ctrl[4:0]}  = 6'b0;
                    
                    idp_Ctrl[23:22] = 2'b10;
                    idp_Ctrl[16:12] = idp_Ctrl[21:17];    //R_addr = W_addr
                    idp_Ctrl[6:0] = 7'b1_01001_0;
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FORHI_2;
                end

            FORHI_2:
                begin
                /************************************************************/
                // FPR[d] <- FPR[d] | {RdBuf1, 32'b0}
                /************************************************************/
                @(negedge sys_clk)
                    imem_Ctrl  = 3'b111; 
                    dmem_Ctrl  = 3'b111; 
                    
                    fpBuf_Ctrl = 4'b0; 
                    wrBuf_Ctrl = 5'b0; 
                    rdBuf_Ctrl = 6'b10_00_00;              //RdBuf1_oe = 1
                    addr_Ctrl  = 9'b0;
                    
                    {fpdp_Ctrl[20],                             //FPW_en = 1
                     fpdp_Ctrl[4:0]}  = 6'b1_1_1110;    //FP_sel = 1; FP_OP= LOGICAL OR
                 
                    idp_Ctrl[23:22] = 2'b0;
                    idp_Ctrl[6:0] = 7'b0;                                    
                    {IP_sel, J_sel, B_Jsel} = 3'b000 ; 
                    {SP_sel, SP_ld} = 2'b00; 
                    NS_Istat   = PS_Istat;                     //Flags remain same
                    NS_FPstat  = PS_FPstat;
                    
                    state = FETCH;
                end
            
                        
            
            ILLEGAL_OP:
                begin
                @(negedge sys_clk)
                    $display("ILLEGAL OPCODE FETCHED %t", $time);                        
                    Dump_FPR;                        
                    Dump_IP_and_IR;
                    
                    $finish;
                end
                
        endcase // end of FSM logic
        end //end always block
        
        
/************************************************************/
// 
//************************************************************ /
   task Dump_Mem;
        integer i;
      begin
            
         #1 $display(" "); $display(" ");    
         //************************************************************
         //Dump Memory contents to console
         //************************************************************                     
         @(negedge sys_clk) begin 
            idp_Ctrl[16:12] = 5'b01110;             
            addr_Ctrl[1:0] = 2'b10; 
            dmem_Ctrl = 3'b001;
            CPU.CPU_EU.idp.reg64.regFile[idp_Ctrl[16:12]] = 32'hE0;//0xE0
          end 
         for (i=0; i<16; i=i+1) begin            
            @(negedge sys_clk)  
               rdBuf_Ctrl[3:2] = 2'b01; 
               addr_Ctrl[1:0] = 2'b01; 
               dmem_Ctrl = 3'b001;
             #1 $display("t=%t || MAR = [%h] || MEM[MAR] = %h", 
                        $time, CPU.Mem_addr[9:0],  CPU.Bus_data, );
            @(negedge sys_clk) 
               rdBuf_Ctrl[3:2] = 2'b10; 
               addr_Ctrl[1:0] = 2'b01;
                dmem_Ctrl = 3'b001;
             #1 $display("t=%t || MAR = [%h] || MEM[MAR] = %h", 
                        $time, CPU.Mem_addr[9:0],  CPU.Bus_data);
                              
         end //End of for-loop      
      
      end //End of begin Block
   endtask //End of Dump_Mem task  
    
/************************************************************/
// Dumps contents of the IDP register file.
//************************************************************ /
   task Dump_IReg;
        integer i;
      begin      
         #1 $display(" "); $display(" ");    
         //******************************************************
         //Dump Memory contents to console
         //******************************************************                     
         idp_Ctrl[5:1] = 5'b10100;
            for (i=0; i<10; i=i+1) begin
            @(negedge sys_clk)
               idp_Ctrl[16:12] = i; //R_addr
            @(negedge sys_clk)
               idp_Ctrl[11:7]  = i + 8; //S_addr
                                
            #1 $display("t=%t || R%d = 0x%h || R%d =0x%h", $time
                        , idp_Ctrl[16:12],  CPU.CPU_EU.IDPReg_to_MAR, 
                          idp_Ctrl[11:7] , CPU.CPU_EU.IDPALU_to_WrBuf);
        
         end //End of for-loop            
      end //End of begin Block
   endtask //End of Dump_Mem task      
    
//************************************************************
// 
// 
//************************************************************ 
    task Dump_IP_and_IR;
        begin 
            $display("******** DUMP IP & IR ***********");
            $display("IP=%h || IR=%h",CPU.CPU_EU.biu.IP.data, CPU.CPU_EU.biu.IR.data);
                    
        end
    endtask //End of Dump_IP_and_IR task    

//************************************************************
// Dump_Mem verilog task used to display the contents of 
// the 64 bit RegisterFile
//************************************************************  
   task Dump_FPR;
    real float; 
    integer i;     
        begin
            #1 $display(" "); $display(" ");    
         //**************************************************
         //Dump Memory contents to console
         //***************************************************         
         {fpdp_Ctrl[20],                             
            fpdp_Ctrl[4:0]}  = 6'b0_0_0000;
            for (i=0; i<16; i=i+1) begin
                                   
                @( negedge sys_clk)
                    fpdp_Ctrl[9:5] = i; 
                #1 float = $bitstoreal(CPU.CPU_EU.FP_out);     
                #1 $display("t=%t || FPR %h  = Float_OUT=%h // %g", 
                        $time, fpdp_Ctrl[9:5],  CPU.CPU_EU.FP_out, float);    
                
            end //end of for-loop
        
        end    // end of begin block
    endtask    //End of task 
endmodule