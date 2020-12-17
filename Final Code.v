module Multiplier(
    input clk, 
    input RxD,
//input en,
//input rst,
//input div0,
input [7:0] A,
input [7:0] B,
output reg [6:0] sevenSegment,
output reg [3:0] enable
);
 wire en, rst, div0;
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




module UART#(parameter reset_key=8'b01100001,clk_freq = 100_000_000,parameter baud_rate = 9_600,parameter oversamples = 4,
parameter reset_counter = clk_freq/(baud_rate*oversamples),parameter counter_mid_sample = (oversamples/2),parameter num_bit = 10,
parameter reset_high_seconds = 1,parameter reset_time_counter = clk_freq*reset_high_seconds)( // UART (Asynchronous) receiver, that gives pulse on output_level when 'a' keyboard key ascii value is received
input clk, // input clock
//input reset, // reset should be normally an input wire that is used to initialize some registers to 0, but in our remote case we don't have access to any GPIOs to use for reset, so we specify reg initial values directly to 0 instead of waiting for a reset signal to do so (for example reg state = 0;), take care that a wire doesn't have storage so an initial value to it doesn't make sense.
input RxD, // input receiving data line
output [7:0] RxData, // output for 8 bits data
output output_level); // output level
//internal variables
reg [3:0] bitcounter = 0; // 4 bits counter to count if 10 bits data transmission complete or not
reg [1:0] samplecounter = 0; // 2 bits sample counter to count up to 4 for oversampling
reg clear_bitcounter, inc_bitcounter, inc_samplecounter, clear_samplecounter; // clear or increment the counter
reg [13:0] counter = 0; // 14 bits counter to count the baud rate (symbol rate) for UART receiving
reg state = 0; // initial state variable (mealy finite state machine)
reg nextstate; // next state variable (mealy finite state machine)
reg shift; // signal to indicate shifting data is ready
reg [9:0] rxshiftreg; // 10 bits data needed to be shifted in during transmission. For storing the serial package and sending its bits one by one. The least significant bit is initialized with the binary value "0" (a start bit) A binary value "1" is introduced in the most significant bit
reg output_reset = 0; // our output reset
reg [31:0] time_counter = 0; // needed for our output reset
// Constants (a parameter infers its type from its value)
//parameter clk_freq = 100_000_000; // system clock frequency
//parameter baud_rate = 9_600; // baud rate (should be agreed upon with the transmitter)
//parameter oversamples = 4; // oversampling
//parameter reset_counter = clk_freq/(baud_rate*oversamples); // counter upper bound
// <------------ 100M clock cycles ----------->
// <------------ 9,600 bits ------------------>
// <------------ 9,600x4 samples ------------->
// reset_counter = 2604, means counter goes from 0 to 2604 (during that time I should take 1 sample)
//parameter counter_mid_sample = (oversamples/2); // this is the middle point of a bit where you want to sample it
//parameter num_bit = 10; // 1 start, 8 data, 1 stop
//parameter reset_key = 8'b01100001; // 8'b01100001; is the binary value of the ASCII character of small 'a' keyboard button
//parameter reset_high_seconds = 1;
//parameter reset_time_counter = clk_freq*reset_high_seconds;
assign RxData = rxshiftreg [8:1]; // assign the RxData from the shiftregister
assign output_level = output_reset;
// UART receiver logic
always @ (posedge clk) begin
counter <= counter +1; // start count in the counter
if (counter >= reset_counter-1) begin // if counter reach the baud rate with sampling
counter <=0; //reset the counter
state <= nextstate; // assign the state to nextstate
if (shift)rxshiftreg <= {RxD,rxshiftreg[9:1]}; //if shift asserted, load the receiving data
if (clear_samplecounter) samplecounter <=0; // if clear sampl counter asserted, reset sample counter
if (inc_samplecounter) samplecounter <= samplecounter +1; //if increment counter asserted, start sample count
if (clear_bitcounter) bitcounter <=0; // if clear bit counter asserted, reset bit counter
if (inc_bitcounter)bitcounter <= bitcounter +1; // if increment bit counter asserted, start count bit counter
end
// Handle our desired reset output
if (output_reset == 0 && rxshiftreg[8:1] == reset_key)
output_reset <= 1;
if (output_reset == 1)
if (time_counter >= reset_time_counter) begin
time_counter <= 0;
output_reset <= 0;
rxshiftreg[8:1] <= 0;
end
else time_counter <= time_counter +1;
end
// Finite state machine
always @ (posedge clk) begin //trigger by clock
shift <= 0; // set shift to 0 to avoid any shifting
clear_samplecounter <=0; // set clear sample counter to 0 to avoid reset
inc_samplecounter <=0; // set increment sample counter to 0 to avoid any increment
clear_bitcounter <=0; // set clear bit counter to 0 to avoid claring
inc_bitcounter <=0; // set increment bit counter to avoid any count
nextstate <=0; // set next state to be idle state
case (state)
0: begin // idle state
if (RxD) // if input RxD data line asserted
nextstate <=0; // back to idle state because RxD needs to be low to start transmission
else begin // if input RxD data line is not asserted
nextstate <=1; // jump to receiving state
clear_bitcounter <=1; // trigger to clear bit counter
clear_samplecounter <=1; // trigger to clear sample counter
end
end
1: begin // receiving state
nextstate <= 1; // DEFAULT
if (samplecounter == counter_mid_sample - 1) shift <= 1; // if sample counter is 1, trigger shift
if (samplecounter == oversamples - 1) begin // if sample counter is 3 as the sample rate used is 3
if (bitcounter == num_bit - 1) // check if bit counter if 9 or not
nextstate <= 0; // back to idle state if bit counter is 9 as receving is complete
inc_bitcounter <=1; // trigger the increment bit counter if bit counter is not 9
clear_samplecounter <=1; //trigger the sample counter to reset the sample counter
end
else inc_samplecounter <=1; // if sample is not equal to 3, keep counting
end
default: nextstate <=0; //default idle state
endcase
end
endmodule




module stabilizer(input clk,rst, in,output out);

reg state=0, nextstate;
parameter Szero=1'b0, Sone=1'b1;
always@(state or in ) begin
case (state)
Szero : if(in) nextstate=Sone;  else nextstate=Szero;
Sone :  nextstate=Sone;
endcase 
end

always@(posedge clk or posedge rst ) begin 
if(rst)
state<=Szero;
else 
state<=nextstate;
end

assign out=state;
endmodule





module bin2bcd(indata, ones, tens, hundreds);
input [7:0] indata;
output [3:0] ones, tens;
output [1:0] hundreds;
wire [3:0] c1, c2, c3, c4, c5, c6, c7, c8, c9;
wire [3:0] d1, d2, d3, d4, d5, d6, d7, d8, d9;

assign d1 = {1'b0,indata[7:5]};
assign d2 = {c1[2:0],indata[4]};
assign d3 = {c2[2:0],indata[3]};
assign d4 = {c3[2:0],indata[2]};
assign d5 = {c4[2:0],indata[1]};
assign d6 = {1'b0,c1[3],c2[3],c3[3]};
assign d7 = {c6[2:0],c4[3]};

lut m1(d1,c1);
lut m2(d2,c2);
lut m3(d3,c3);
lut m4(d4,c4);
lut m5(d5,c5);
lut m6(d6,c6);
lut m7(d7,c7);

assign ones = {c5[2:0],indata[0]};
assign tens = {c7[2:0],c5[3]};
assign hundreds = {c6[3],c7[3]};
endmodule

//////////////////////// module for data mapping
module lut(in,out);
input [3:0] in;
output [3:0] out;
reg [3:0] out;

always @ (in)
begin
    case (in)
        4'b0000: out <= 4'b0000;
        4'b0001: out <= 4'b0001;
        4'b0010: out <= 4'b0010; 
        4'b0011: out <= 4'b0011; 
        4'b0100: out <= 4'b0100; 
        4'b0101: out <= 4'b1000; 
        4'b0110: out <= 4'b1001; 
        4'b0111: out <= 4'b1010; 
        4'b1000: out <= 4'b1011; 
        4'b1001: out <= 4'b1100; 
        default: out <= 4'b0000; 
    endcase 
end
endmodule





module SevenSegmentDecoder(
	input [3:0] in,
	output reg [6:0] out
);

always @ (*) begin
	case(in)
		4'b0000: out= 7'b0000001;
		4'b0001: out= 7'b1001111;
		4'b0010: out= 7'b0010010;
		4'b0011: out= 7'b0000110;
		4'b0100: out= 7'b1001100;
		4'b0101: out= 7'b0100100;
		4'b0110: out= 7'b0100000;
		4'b0111: out= 7'b0001111;
		4'b1000: out= 7'b0000000;
		4'b1001: out= 7'b0000100;
	endcase
end


endmodule



module ClockDivider #(parameter n= 500000) (
    input clk, 
    input rst, 
output reg clk_out
);

reg[24:0]count;

always@(posedge(clk),posedge(rst))begin// Asynchronous Reset
    if(rst ==1'b1)
        count <=25'b0;
    else if(count ==n-1)
        count <=32'b0;
    else
        count <= count +1;
end
    
always@(posedge(clk), posedge(rst)) begin
    if(rst ==1'b1)
        clk_out <=1'b0;
    else if(count ==n-1)
        clk_out <= ~clk_out;
    else
        clk_out <= clk_out;
end
endmodule





module Counter #(parameter n=3)(input clk, reset, output reg [n:0]count);
always @(posedge clk, posedge reset) begin

 if (reset == 1)
 count <= 0; // non blocking assignment 
 else 
         count <= count + 1; 

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
