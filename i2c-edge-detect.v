
/* synchronously detect a level transition of 'lin'.
 * Assert lohi for one clock cycle after a 0->1 transition.
 * Assert hilo for one clock cycle after a 1->0 transition.
 */
module i2c_edge_detect(input clk, input lin, output reg hilo, output reg lohi, input rst);

`include "i2c-timing-params.vh"

/* signal must be stable for 1/2 rise-/fall-time */
function integer perClip(integer val, integer scl);
begin
  perClip = val/scl - 1;
  if ( perClip < 0 ) perClip = 0;
end endfunction

localparam integer STABLE_RISING  = perClip(PER_TR,2);
localparam integer STABLE_FALLING = perClip(PER_TF,2);

localparam MAX_PER = STABLE_RISING > STABLE_FALLING ? STABLE_RISING : STABLE_FALLING;

/* iverilog doesn't support constant functions :-( -- use trickery... MUST use max. period */
localparam m1  =  MAX_PER & 32'hffff0000;
localparam m1a =  m1 ? m1 : MAX_PER;
localparam m2  =  m1a & 32'hff00ff00;
localparam m2a =  m2 ? m2 : m1a;
localparam m3  =  m2a & 32'hf0f0f0f0;
localparam m3a =  m3 ? m3 : m2a;
localparam m4  =  m3a & 32'hcccccccc;
localparam m4a =  m4 ? m4 : m3a;
localparam m5  =  m4a & 32'haaaaaaaa;

localparam PER_LD_SIZE = (m1?16:0) + (m2?8:0) + (m3?4:0) + (m4?2:0) + (m5?1:0) + 1;

reg [PER_LD_SIZE-1:0] div = STABLE_FALLING;
reg lin_l = 1'b1;

	always @(posedge clk) begin
		if ( rst ) begin
			hilo  <= 0;
			lohi  <= 0;
			lin_l <= lin;
			div   <= lin ? STABLE_FALLING : STABLE_RISING;
		end else begin
			hilo  <= 0;
			lohi  <= 0;
			lin_l <= lin_l;
            if ( lin != lin_l ) begin
	 			if ( div > 0 ) begin
					div   <= div - 1;
				end else begin
					lin_l <= lin;
					case ( {lin_l, lin} )
						2'b01       : begin hilo <= 0; lohi <= 1; end
						2'b10       : begin hilo <= 1; lohi <= 0; end
					endcase
				end
			end else begin 
				div <= lin_l ? STABLE_FALLING : STABLE_RISING;
			end
		end
	end
endmodule

module i2c_bby_detect(input clk, input sda, input scl, output sto, output sta, output reg bby, input rst);
`include "i2c-timing-params.vh"

wire sda_hilo, sda_lohi;

	assign sto = sda_lohi && scl;
	assign sta = sda_hilo && scl;

	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			/* Not completely accurate - we can't know if another master holds
			 * the bus - hopefully arbitration will eventually resolve this condition
			 */
			bby <= 0;
		end else begin
			if ( sta )
				bby <= 1;
			else if ( sto )
				bby <= 0;
		end
	end
	
	i2c_edge_detect #(.US(US),.I2C_MODE(I2C_MODE)) sda_det(.clk(clk), .lin(sda), .hilo(sda_hilo), .lohi(sda_lohi), .rst(rst) );
endmodule
