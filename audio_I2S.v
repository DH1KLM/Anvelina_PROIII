// audio_I2S.v

//
//  HPSDR - High Performance Software Defined Radio
//
//  Angelia code. 
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

// (C) Phil Harman VK6PH - 2014 

/*


      LRCLK    ---------------+                                         +-----------------------------------------+
                              |                                         |                                         |
                              +-----------------------------------------+                                         +-----

      BCLK      --+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+
                  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
                  +--+  +--+  +--+  +--+  +--+  +  +  +--+  +--+  +  +  +--+  +--+  +--+  +  +  +--+  +--+  +  +  +--+  +--

      DIN       --------------------+-----+-----+-- --+-----+-----+ --    ----+-----+-- --+-----+-----+-----+------
      (data bit)                    | 31  | 30  |  // |  17 |  16 |  15 //    | 15  | 14  |     |  1  |  0  |
                --------------------+-----+-----+-- --+-----+-----+ --    ----+-----+-----+-- --+-----+-----+------
                                    <-----------  left channel ---------------><----------- right channel --------------> 
                                    
                        +-----+                                                                             +-----+
                        |     |                                                                             |     |
      get_data ---------+     +-----------------------------------------------------------------------------+     +-----------------            

      TLV320 is in slave, I2S mode.

      NOTE:  BCLK in PLL_IF is set at 180 degrees so that LRCLK changes on negative edge to comply with I2S format 					 
					 
*/

//==============================================================================
// Module: audio_I2S
// Project: Anvelina ProIII DX (Orion-based firmware)
// Author: eu2av
// Description: I2S audio interface for TLV320AIC23B codec
//              1. Fixed Warning 10230 via explicit 16-bit wires
//              2. Restored Original FIFO Logic (safe threshold > 767)
//              3. Hardware Output Mute extended to 60ms for complete click suppression
//==============================================================================
module audio_I2S 
	( 
		input [31:0]data_in,
		input [12:0]rdusedw,
		input run,
		input empty,
		input BCLK,
		input LRCLK,
		output get_data,
		output reg data_out	
	);
	
	
reg [31:0] data_in_tmp;
reg [4:0] state;
reg [4:0] data_count;
reg [15:0] ramp_count;
reg [1:0] ramp_dir = 0; // 0=stay_down 1=stay_up 2=ramp_up 3=ramp_down
reg holdoff;
reg shift = 0;

//==============================================================================
// [eu2av] 1. Hardware Output Mute (60ms @ 48kHz = 3000 LRCLK cycles)
// Forces data_out = 0 during startup to mask analog transients completely
//==============================================================================
reg [11:0] hw_mute_cnt;
reg hw_mute_active;

always @(posedge LRCLK) begin
	if (!run) begin
		hw_mute_cnt <= 12'd0;
		hw_mute_active <= 1'b1; // Arm mute on stop
	end
	else if (hw_mute_active) begin
		hw_mute_cnt <= hw_mute_cnt + 1'b1;
		if (hw_mute_cnt == 12'd3000) hw_mute_active <= 1'b0; // Release after ~60ms
	end
end

//==============================================================================
// [eu2av] 2. 16-bit ramp arithmetic (module-level wires -> no Warning 10230)
// Math is identical to original, only width promotion is prevented
//==============================================================================
wire [15:0] ramp_mag;            
wire signed [15:0] ramp_signed_L; 
wire signed [15:0] ramp_signed_R; 
wire signed [15:0] diff_L, diff_R; 
wire signed [15:0] out_L, out_R;   

assign ramp_mag = (ramp_count << 7) - 16'd1;
assign ramp_signed_L = data_in[31] ? $signed(ramp_mag) : -$signed(ramp_mag);
assign ramp_signed_R = data_in[15] ? $signed(ramp_mag) : -$signed(ramp_mag);
assign diff_L = $signed(data_in[31:16]) - ramp_signed_L;
assign diff_R = $signed(data_in[15:0]) - ramp_signed_R;
assign out_L = (data_in[30:16] > ramp_mag[14:0]) ? diff_L : 16'd0;
assign out_R = (data_in[14:0] > ramp_mag[14:0]) ? diff_R : 16'd0;

//==============================================================================
// 3. Main ramp control logic (EXACT ORIGINAL STATE MACHINE)
// Preserves the safe FIFO > 767 threshold that prevents crackling
//==============================================================================
always @ (posedge LRCLK)
begin 
	if (!run) begin
		ramp_dir <= 2'd0;
		ramp_count <= 15'd0;
		data_in_tmp <= 32'd0;
		holdoff <= 1'd1;
	end
	// low water reached, start ramping down
	else if (rdusedw < 13'd260 && ramp_dir == 2'd1) begin
		ramp_dir <= 2'd3;
		ramp_count <= 15'd1;
		data_in_tmp <= data_in;
	end
	// high water reached, start ramping back up (Original Safe Threshold)
	else if (rdusedw > 13'd767 && ramp_dir == 2'd0) begin
		ramp_dir <= 2'd2;
		ramp_count <= 15'd256; 
		data_in_tmp <= 32'd0;
		holdoff <= 1'd0;
	end
	// ramp up
	else if (ramp_dir == 2'd2) begin
		data_in_tmp <= {out_L, out_R};
		if (ramp_count == 15'd1) ramp_dir <= 2'd1; 
		ramp_count <= ramp_count - 15'd1;
	end
	// ramp down
	else if (ramp_dir == 2'd3) begin
		data_in_tmp <= {out_L, out_R};
		if (ramp_count == 15'd256) begin
			ramp_dir <= 2'd0; 
			holdoff <= 1'd1;
		end
		ramp_count <= ramp_count + 15'd1;
	end
	// stable states
	else if (ramp_dir == 2'd0) data_in_tmp <= 32'd0; 
	else data_in_tmp <= data_in; 
end

//==============================================================================
// 4. I2S serial output state machine (Unchanged)
//==============================================================================
always @ (posedge BCLK)
begin 
	if(!empty)
	begin
		case (state)

		0:	begin
				if (LRCLK) state <= 1;
			end 
			
		1:  begin 
				if (!LRCLK) begin
					shift <= 1;
					data_count <= 5'd31;
					state <= 2;
				end
			end
			
		2: begin
				if (data_count == 16) begin 
					shift <= 0;
					if(LRCLK) begin 
						data_count <= 15;
						shift <= 1;
						state <= 3;
					end 			
				end 
				else begin
					data_count <= data_count - 5'd1;
				end
			end
			
		3:	begin 
				if (data_count == 0) begin
					shift <= 0;
					state <= 0;
				end
				else data_count <= data_count - 5'd1;
			end 
			
		default: state <= 0;
		endcase
	end 
end 

// Serial output timing
reg [5:0] get_count;		
always @ (negedge BCLK)
begin 
	// [eu2av] HARDWARE MUTE: Force output to 0 for ~60ms after START
	if (hw_mute_active) data_out <= 1'b0;
	else if (shift) data_out <= data_in_tmp[data_count];
		else data_out <= 0;
		if (!LRCLK) get_count <= 0;
		else get_count <= get_count + 6'd1;
end 
			
// Block get_data during mute to prevent FIFO underrun
assign get_data = ((get_count == 6'd30) && !holdoff && !hw_mute_active);
			
			
endmodule
	
