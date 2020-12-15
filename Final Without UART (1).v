module Multiplier(
    input clk, 
    input RxD,
input en,
input rst,
input div0,
input [7:0] A,
input [7:0] B,
output reg [6:0] sevenSegment,
output reg [3:0] enable
);
 
 reg EA=0;
 
 reg EP=0;
 wire [15:0] P;
 reg [15:0] A_comp=0;
 reg [7:0] B_comp=0;
 reg [15:0] P_temp=0;
 
 
 UART (.clk(clk), .RxD(RxD), .RxData(), .output_level(en));
 UART #(8'b01100010)(.clk(clk), .RxD(RxD), .RxData(), .output_level(rst));
 UART #(8'b01100011)(.clk(clk), .RxD(RxD), .RxData(), .output_level(div0));

stabilizer m( .clk(clk),.rst(rst),.in(div0),.out(div));


  always@(posedge clk) begin
      if(rst) begin
           P_temp=0;
           A_comp=0;
           B_comp=0;
           EA=0;
           EP=0;
        
      end
      
      else if(en) begin
       A_comp= (A[7])? { 8'b00000000, ~A + 8'b00000001}: A;
       B_comp= (B[7])? ~B + 8'b00000001: B;
       EA=1;
      end
      else if(~div) begin
            if(B_comp != 0 && EA) begin
           if(B_comp[0] == 1) begin
                 P_temp = P_temp + A_comp;
           end
      
        B_comp=B_comp >> 1;
        A_comp= A_comp <<1;
        
        end
        else if (B_comp == 0 && EA) begin
        //EA= 1'b0;
        EP= 1'b1;
        end
      end
      
      else if(div) begin
          if((A_comp- B_comp) > 0 && EA) begin
              A_comp = A_comp - B_comp;
              P_temp= P_temp +1;
           end
          else if (EA && ~EP) begin EP= 1'b1; P_temp= P_temp+1;
          end
        end

end

assign P= (~EP | rst)? 0 : P_temp; 

       wire [6:0] onesSeg, tensSeg, hundredsSeg;
     wire [3:0] ones, tens;
     wire [1:0] hundreds;
       
       bin2bcd b2b (.indata(P), .ones(ones), .tens(tens), .hundreds(hundreds));
       
       SevenSegmentDecoder ssd1 ( .in(ones), .out(onesSeg));
       SevenSegmentDecoder ssd2 ( .in(tens), .out(tensSeg));
       SevenSegmentDecoder ssd3 ( .in({2'b00, hundreds}), .out(hundredsSeg));
       
       wire clk_out;
       ClockDivider cd (.clk(clk), .rst(1'b0), .clk_out(clk_out));
       
       wire [1:0] count;
       Counter #(2) co (.clk(clk_out), .reset(0), .count(count));
       
      
     
       
       always@ (clk_out) begin
         case (count) 
             2'b00: begin
            sevenSegment= (A[7]^B[7])? 7'b1111110: 7'b1111111;
              enable= 4'b0111;
             end
             2'b01: begin
             sevenSegment= hundredsSeg;
               enable= 4'b1011;
             end
             2'b10: begin
             sevenSegment= tensSeg;
               enable= 4'b1101;
             end
             2'b11: begin
             sevenSegment= onesSeg;
               enable= 4'b1110;
             end        
         endcase
         
       end

endmodule




set_property PACKAGE_PIN W5 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]



set_property PACKAGE_PIN V17 [get_ports A[0]]
set_property IOSTANDARD LVCMOS33 [get_ports A[0]]

set_property PACKAGE_PIN V16 [get_ports A[1]]
set_property IOSTANDARD LVCMOS33 [get_ports A[1]]

set_property PACKAGE_PIN W16 [get_ports A[2]]
set_property IOSTANDARD LVCMOS33 [get_ports A[2]]

set_property PACKAGE_PIN W17 [get_ports A[3]]
set_property IOSTANDARD LVCMOS33 [get_ports A[3]]

set_property PACKAGE_PIN W15 [get_ports A[4]]
set_property IOSTANDARD LVCMOS33 [get_ports A[4]]

set_property PACKAGE_PIN V15 [get_ports A[5]]
set_property IOSTANDARD LVCMOS33 [get_ports A[5]]

set_property PACKAGE_PIN W14 [get_ports A[6]]
set_property IOSTANDARD LVCMOS33 [get_ports A[6]]

set_property PACKAGE_PIN W13 [get_ports A[7]]
set_property IOSTANDARD LVCMOS33 [get_ports A[7]]



set_property PACKAGE_PIN V2 [get_ports B[0]]
set_property IOSTANDARD LVCMOS33 [get_ports B[0]]

set_property PACKAGE_PIN T3 [get_ports B[1]]
set_property IOSTANDARD LVCMOS33 [get_ports B[1]]

set_property PACKAGE_PIN T2 [get_ports B[2]]
set_property IOSTANDARD LVCMOS33 [get_ports B[2]]

set_property PACKAGE_PIN R3 [get_ports B[3]]
set_property IOSTANDARD LVCMOS33 [get_ports B[3]]

set_property PACKAGE_PIN W2 [get_ports B[4]]
set_property IOSTANDARD LVCMOS33 [get_ports B[4]]

set_property PACKAGE_PIN U1 [get_ports B[5]]
set_property IOSTANDARD LVCMOS33 [get_ports B[5]]

set_property PACKAGE_PIN T1 [get_ports B[6]]
set_property IOSTANDARD LVCMOS33 [get_ports B[6]]

set_property PACKAGE_PIN R2 [get_ports B[7]]
set_property IOSTANDARD LVCMOS33 [get_ports B[7]]




set_property PACKAGE_PIN W7 [get_ports sevenSegment[6]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[6]]

set_property PACKAGE_PIN W6 [get_ports sevenSegment[5]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[5]]

set_property PACKAGE_PIN U8 [get_ports sevenSegment[4]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[4]]

set_property PACKAGE_PIN V8 [get_ports sevenSegment[3]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[3]]

set_property PACKAGE_PIN U5 [get_ports sevenSegment[2]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[2]]

set_property PACKAGE_PIN V5 [get_ports sevenSegment[1]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[1]]

set_property PACKAGE_PIN U7 [get_ports sevenSegment[0]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[0]]

set_property PACKAGE_PIN W4 [get_ports enable[3]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[3]]

set_property PACKAGE_PIN V4 [get_ports enable[2]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[2]]

set_property PACKAGE_PIN U4 [get_ports enable[1]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[1]]

set_property PACKAGE_PIN U2 [get_ports enable[0]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[0]]







//Final but UART in en and rst
module Multiplier(
    input clk, 
    input RxD,
//input en,
//input rst,
//input div0,
//input [7:0] A,
//input [7:0] B,
output reg [6:0] sevenSegment,
output reg [3:0] enable
);
 wire [7:0] A = 8'b00000100 ;
 wire [7:0] B = 8'b11111110;
 reg EA=0;
 
 reg EP=0;
 wire [15:0] P;
 reg [15:0] A_comp=0;
 reg [7:0] B_comp=0;
 reg [15:0] P_temp=0;
 
 wire en, rst, div0;
 
 UART (.clk(clk), .RxD(RxD), .RxData(), .output_level(en));
 UART #(8'b01100010)(.clk(clk), .RxD(RxD), .RxData(), .output_level(rst));
 UART #(8'b01100011)(.clk(clk), .RxD(RxD), .RxData(), .output_level(div0));

stabilizer m( .clk(clk),.rst(rst),.in(div0),.out(div));


  always@(posedge clk) begin
      if(rst) begin
           P_temp=0;
           A_comp=0;
           B_comp=0;
           EA=0;
           EP=0;
        
      end
      
      else if(en) begin
       A_comp= (A[7])? { 8'b00000000, ~A + 8'b00000001}: A;
       B_comp= (B[7])? ~B + 8'b00000001: B;
       EA=1;
      end
      else if(~div) begin
            if(B_comp != 0 && EA) begin
           if(B_comp[0] == 1) begin
                 P_temp = P_temp + A_comp;
           end
      
        B_comp=B_comp >> 1;
        A_comp= A_comp <<1;
        
        end
        else if (B_comp == 0 && EA) begin
        //EA= 1'b0;
        EP= 1'b1;
        end
      end
      
      else if(div) begin
          if((A_comp- B_comp) > 0 && EA) begin
              A_comp = A_comp - B_comp;
              P_temp= P_temp +1;
           end
          else if (EA && ~EP) begin EP= 1'b1; P_temp= P_temp+1;
          end
        end

end

assign P= (~EP | rst)? 0 : P_temp; 

       wire [6:0] onesSeg, tensSeg, hundredsSeg;
     wire [3:0] ones, tens;
     wire [1:0] hundreds;
       
       bin2bcd b2b (.indata(P), .ones(ones), .tens(tens), .hundreds(hundreds));
       
       SevenSegmentDecoder ssd1 ( .in(ones), .out(onesSeg));
       SevenSegmentDecoder ssd2 ( .in(tens), .out(tensSeg));
       SevenSegmentDecoder ssd3 ( .in({2'b00, hundreds}), .out(hundredsSeg));
       
       wire clk_out;
       ClockDivider cd (.clk(clk), .rst(1'b0), .clk_out(clk_out));
       
       wire [1:0] count;
       Counter #(2) co (.clk(clk_out), .reset(0), .count(count));
       
      
     
       
       always@ (clk_out) begin
         case (count) 
             2'b00: begin
            sevenSegment= (A[7]^B[7])? 7'b1111110: 7'b1111111;
              enable= 4'b0111;
             end
             2'b01: begin
             sevenSegment= hundredsSeg;
               enable= 4'b1011;
             end
             2'b10: begin
             sevenSegment= tensSeg;
               enable= 4'b1101;
             end
             2'b11: begin
             sevenSegment= onesSeg;
               enable= 4'b1110;
             end        
         endcase
         
       end

endmodule




set_property PACKAGE_PIN W5 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]

set_property PACKAGE_PIN B18 [get_ports RxD]
set_property IOSTANDARD LVCMOS33 [get_ports RxD]


set_property PACKAGE_PIN W7 [get_ports sevenSegment[6]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[6]]

set_property PACKAGE_PIN W6 [get_ports sevenSegment[5]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[5]]

set_property PACKAGE_PIN U8 [get_ports sevenSegment[4]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[4]]

set_property PACKAGE_PIN V8 [get_ports sevenSegment[3]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[3]]

set_property PACKAGE_PIN U5 [get_ports sevenSegment[2]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[2]]

set_property PACKAGE_PIN V5 [get_ports sevenSegment[1]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[1]]

set_property PACKAGE_PIN U7 [get_ports sevenSegment[0]]
set_property IOSTANDARD LVCMOS33 [get_ports sevenSegment[0]]

set_property PACKAGE_PIN W4 [get_ports enable[3]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[3]]

set_property PACKAGE_PIN V4 [get_ports enable[2]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[2]]

set_property PACKAGE_PIN U4 [get_ports enable[1]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[1]]

set_property PACKAGE_PIN U2 [get_ports enable[0]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[0]]




//Simulation
module test(
    input clk, 
input en,
input rst,
input div,
input [7:0] A,
input [7:0] B,
output [15:0] P,
output reg [6:0] sevenSegment,
output reg [3:0] enable,
output [3:0] ones, 
output [3:0] tens,
output [1:0] hundreds
);

reg EA=0;

reg EP=0;
//wire [15:0] P;
reg [15:0] A_comp=0;
reg [7:0] B_comp=0;
reg [15:0] P_temp=0;

always@(posedge clk) begin
  if(rst) begin
       P_temp=0;
       A_comp=0;
       B_comp=0;
       EA=0;
       EP=0;
  end
  
  else if(en) begin
   A_comp= (A[7])? { 8'b00000000, ~A + 8'b00000001}: A;
   B_comp= (B[7])? ~B + 8'b00000001: B;
   EA=1;
  end
  else 
  if(~div) begin
    if(B_comp != 0 && EA) begin
       if(B_comp[0] == 1) begin
             P_temp = P_temp + A_comp;
       end
  
    B_comp=B_comp >> 1;
    A_comp= A_comp <<1;
    
    end
    else if (B_comp == 0 && EA) begin
    //EA= 1'b0;
    EP= 1'b1;
    end
  end
  
  else if(div) begin
    if((A_comp- B_comp) > 0 && EA) begin
        A_comp = A_comp - B_comp;
        P_temp= P_temp +1;
     end
    else if (EA && ~EP) begin EP= 1'b1; P_temp= P_temp+1;
    end
  end

end

assign P= (~EP | rst)? 0 : P_temp; 

   wire [6:0] onesSeg, tensSeg, hundredsSeg;
// wire [3:0] ones, tens;
// wire [1:0] hundreds;
   
   bin2bcd b2b (.indata(P), .ones(ones), .tens(tens), .hundreds(hundreds));
   
   SevenSegmentDecoder ssd1 ( .in(ones), .out(onesSeg));
   SevenSegmentDecoder ssd2 ( .in(tens), .out(tensSeg));
   SevenSegmentDecoder ssd3 ( .in({2'b00, hundreds}), .out(hundredsSeg));
   
   wire clk_out;
   ClockDivider cd (.clk(clk), .rst(1'b0), .clk_out(clk_out));
   
   wire [1:0] count;
   Counter #(2) co (.clk(clk_out), .reset(0), .count(count));
   
  
 
   
   always@ (clk_out) begin
     case (count) 
         2'b00: begin
        sevenSegment= (A[7]^B[7])? 7'b1111110: 7'b1111111;
          enable= 4'b0111;
         end
         2'b01: begin
         sevenSegment= hundredsSeg;
           enable= 4'b1011;
         end
         2'b10: begin
         sevenSegment= tensSeg;
           enable= 4'b1101;
         end
         2'b11: begin
         sevenSegment= onesSeg;
           enable= 4'b1110;
         end        
     endcase
     
   end

endmodule




//Testbench
module Multiplier_Sim();
    reg clk, en, rst, div;
    wire [15:0] P;
    reg [7:0] A;
    reg [7:0] B;
    wire [6:0] sevenSegment;
    wire [3:0] enable;
    wire [3:0] ones, tens;
    wire [1:0] hundreds;
    
    
    test m (  .clk(clk), .en(en), .rst(rst), .div(div), .A(A), .B(B), .P(P), .sevenSegment(sevenSegment), .enable(enable), .ones(ones), .tens(tens), .hundreds(hundreds));
    
    initial begin
    clk=0;
    forever #20 clk=~clk;
    end
    
    initial begin
        A= 8'b11111100; B= 8'b11111110; en=1; rst=0; div=0;
       #40
        A= 8'b00000000; B= 8'b00000000; en=0; rst=0; div=0;
        #200
        A= 8'b00000011; B= 8'b00000010; en=0; rst=1; div=0;
                #40
         A= 8'b00000100; B= 8'b00000010; en=1; rst=0; div=1;
               #60
              A= 8'b00000100; B= 8'b00000010; en=0; rst=0; div=1;  
              #200  
        A= 8'b00000011; B= 8'b00000011; en=0; rst=1; div=0;  
        #60
         A= 8'b00011001; B= 8'b11111011; en=1; rst=0; div=1;
                      #60
                     A= 8'b00011001; B= 8'b11111010; en=0; rst=0; div=1;  
                     #200  
               A= 8'b00000011; B= 8'b00000010; en=0; rst=1; div=0;      
        end
    
endmodule
