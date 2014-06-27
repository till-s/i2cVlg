
/* synchronously detect a level transition of 'lin'.
 * Assert lohi for one clock cycle after a 0->1 transition.
 * Assert hilo for one clock cycle after a 1->0 transition.
 */
module i2c_edge_detect(input clk, input lin, output reg hilo, output reg lohi, input rst);

parameter  US=1;
localparam PER_TR = 10*US/10; /* SDA/SCL rise time (max)  1us/.3us */
localparam PER_TF =  3*US/10; /* SDA/SCL fall time (max) .3us/.3us */
localparam MAX_PER=PER_TR;

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


reg [PER_LD_SIZE-1:0] div;
reg lin_l;

	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			hilo  <= 0;
			lohi  <= 0;
			lin_l <= lin;
			div   <= PER_TR;
		end else begin
			hilo  <= 0;
			lohi  <= 0;
			if ( div > 0 ) begin
				div <= div - 1;
			end else begin 
				div <= lin ? (PER_TF > 0 ? PER_TF - 1 : 0) :
                             (PER_TR > 0 ? PER_TR - 1 : 0) 
				       ;
				case ( {lin_l, lin} )
					2'b01       : begin hilo <= 0; lohi <= 1; end
					2'b10       : begin hilo <= 1; lohi <= 0; end
				endcase
				lin_l <= lin;
			end
		end
	end
endmodule

module i2c_bby_detect(input clk, input sda, input scl, output sto, output sta, output reg bby, input rst);
parameter US = 1;

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
	
	i2c_edge_detect #(.US(US)) sda_det(.clk(clk), .lin(sda), .hilo(sda_hilo), .lohi(sda_lohi), .rst(rst) );
endmodule