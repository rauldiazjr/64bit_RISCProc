`timescale 1ns / 1ps
/********************************************************************************
 *
 * Class:       CECS 440
 * Author:      Raul Diaz
 * Email:       raul.diaz91@live.com
 * Project:     Lab1
 * Filename:    ALU_64.v
 * Due Date:    Sept 8, 2014
 * Description: A 64 bit ALU verilog module with signed arithmetic operations 
                including multiplication, division, addition, subtraction and 
                several other Logic operation. The multipication implentaion
                of R and S registers can return a 128bit value using 2 scratch
                resisters in the form of high and low 64 bit words. Division is
                handled in a similar hardware implementaion however the remainder
                is found in the high 64bit word of the 'remain' register with 
                the quotient being stored in the low 64bit word. 
                The logical SHR & SHL operations are handled similar to 
                any other hardware implementation(i.e ignoring the signed MSB) 
                The Arithmetic shifts preserve the signed bit of 
                the MSB. 
*********************************************************************************/
module ALU_64( R, S, Alu_Op, Y, C, N, Z, O); 
   input [63:0] R, S; 
   input [4:0]  Alu_Op; 
   output reg [63:0] Y; 
   output reg N, Z, C, O;
   
   //Scratch registers used for storing data at runtime. 
   reg signR, signS; 
   reg[127:0] product, remain; 
   reg[63:0] multcand, divisor; 
   integer i; 
   
   always @( R or S or Alu_Op) begin
      case (Alu_Op)        
          5'b00000: begin
                        {C, Y} = S + -1;                 //Decrement S
                         O = Y[63] ^ S[63];
                    end
          5'b00001: begin
                        {C, Y} = S + 1;                //increment S 
                        O = Y[63] ^ S[63];
                         
                    end
          5'b00010:                                 //Mul; Return LSW
                     begin                           
                       //Saves sign of R & S values
                       signS = S[63];    
                       signR = R[63];
                       
                       //Get magnitudes of R and S registers. 
                       if(signR)
                           product[63:0] = ~R + 1; 
                       else
                           product[63:0] = R; 
                       if(signS)
                           multcand = ~S + 1; 
                       else
                           multcand = S; 
                           
                       //initialize MSW to 0; 
                       product[127:64] = 64'b0; 
                       
                       for(i=0; i<64; i = i + 1) begin
                           if(product[0] == 1) //add multiplicand too MSW
                              product[127:64] = product[127:64] + multcand;
                           
                           product = product >> 1; //SHR 128bit product
                       end //end forloop
                       
                       if(product[127]) //MSB is high therefore overflow
                           O = 1'b1;
                       
                       if(signS ^ signR)   //configure sign from R&S values
                           product = ~product +1; 
                       {C, Y} = {1'b0, product[63:0]}; 
                    end
                               
          5'b00011: Y =  product[127:64]; //retrun product MSW       
          5'b00100: //Div; return REM LSW
                     begin                                
                       signR = R[63];
                       signS = S[63];                          
                       //Get magnitudes from R and S registers. 
                       if(signR)
                           remain[63:0] = ~R + 1; 
                       else
                           remain[63:0] = R; 
                       if(signS)
                           divisor = ~S + 1; 
                       else
                           divisor = S; 
                           
                       //initialize MSW to 0; SHL
                       remain[127:64] = 64'b0; 
                       remain = remain << 1; 
                       
                       //Begin division algorithm loop
                       for(i = 0; i<64; i=i+1) begin
                           remain[127:64] = remain[127:64] - divisor; 
                           if(remain[127] == 1'b1) begin
                              remain[127:64] = remain[127:64] + divisor; 
                              remain = remain << 1; 
                              remain[0] = 1'b0; 
                           end
                           else   begin
                              remain = remain << 1; 
                              remain[0] = 1'b1;
                           end                                                   
                        end //end for loop
                        
                        remain[127:64] = remain[127:64] >> 1;       
                        if(signS ^ signR)   //configure sign from R&S values
                           remain[63:0] = ~remain[63:0] + 1; 
                       {C, Y} = {1'b0, remain[63:0]};       
                    end //end block scope
          5'b00101: {C, Y} = {1'b0, remain[127:64]};        // REM MSW          
          5'b00110: // SUB
                   begin 
                       signS = S[63];    
                       signR = R[63];
                       {C, Y} = R - S;                                  
                       if(signS != signR) 
                           O = ~(Y[63] ^ signS); 
                           
                       else 
                           O = 1'b0; 
                             
                   end
         5'b00111: begin              // ADD     
                       signS = S[63];    
                       signR = R[63];
                       
                       {C, Y} = R + S;               
                       if(signS == signR)
                           O = Y[63] ^ signS; 
                       else
                           O = 1'b0; 
                    end
          5'b01000: {C, Y} = {1'b0, R & S};               // logic AND
          5'b01001: {C, Y} = {1'b0, R | S};                   // logic or 
          5'b01010: {C, Y} = {1'b0, R ^ S};                 // logic xor
          5'b01011: {C, Y} = {1'b0, ~S};                    // logic not S (1's comp) 
          5'b01100: {C, Y} = {0-S};                 // Negate (2's comp) 
          5'b01101: begin                                    //Logical SHL
                        C = S[63]; 
                        Y = S << 1; 
                    end
          5'b01110: begin                                    //Logical SHR
               C = S[0]; 
               Y = S >> 1; 
               end
          5'b01111: begin                                    //Arithmetic SHL
               C = S[63]; 
               Y = S <<< 1;
               O = C ^ Y[63];                                    
               end
          5'b10000: begin                                    //Arithmetic SHR 
                        C = S[0]; 
                        Y = {S[63], S[62:0] >> 1};
                    end
          5'b10001: {C, Y} = {1'b0, 64'b0};                  //64 bit Zeros
          5'b10010: {C, Y} = {1'b0, 64'hFFFFFFFFFFFFFFFF};   //64 bitOnes
          5'b10011: {C, Y} = {1'b0, R};                     //Pass R
          5'b10100: {C, Y} = {1'b0, S};                     //Pass S
          5'b10101: Y = {S[62:0], S[63]};                    //Rotate Left on S
          5'b10110: Y = {S[0], S[63:1]};                     //Rotate Right on S
          5'b10111: {C,N,Z,O} = Y[3:0];                      //Load Flags     
          default:  {C, Y} = {1'b0, S};                      // pass S for default 
      endcase
      
      // handle last two status flags 
      if(Alu_Op == 5'h3 || Alu_Op == 5'h2) //multiplication operations
         N = (signS ^ signR) ;
         
      else
         N=  Y[63]; 
      
      if (Y == 64'b0)       
         Z = 1'b1; 
      else
         Z = 1'b0;
      
   end // end always
endmodule