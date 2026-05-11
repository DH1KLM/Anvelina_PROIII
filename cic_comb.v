/*
--------------------------------------------------------------------------------
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Library General Public
License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Library General Public License for more details.
You should have received a copy of the GNU Library General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
Boston, MA  02110-1301, USA.
--------------------------------------------------------------------------------
*/


//------------------------------------------------------------------------------
//           Copyright (c) 2008 Alex Shovkoplyas, VE3NEA
//------------------------------------------------------------------------------



module cic_comb( clock, strobe,  in_data,  out_data );

parameter WIDTH = 64;

input clock;
input strobe;
input signed [WIDTH-1:0] in_data;
output reg signed [WIDTH-1:0] out_data;

reg signed [WIDTH-1:0] prev_data = 0;

always @(posedge clock)  
  if (strobe) 
    begin
    out_data <= in_data - prev_data;
    prev_data <= in_data;
    end



endmodule

module cic_comp(
  input clock,
  input signed [17:0] cic_in,
  output signed [17:0] cic_out
);
  // Коэффициенты для компенсации sinc^4 при R=40..640
  localparam B0 = 18'b000000000000000001; // 1
  localparam B1 = 18'b111111111111111110; // -2
  localparam B2 = 18'b000000000000000001; // 1
  
  reg signed [17:0] d1, d2;
  always @(posedge clock) begin
    d1 <= cic_in;
    d2 <= d1;
    cic_out <= (B0 * cic_in) + (B1 * d1) + (B2 * d2); // FIR 2nd order
  end
endmodule