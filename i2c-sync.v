module i2c_sync(clk, rst, scl_i, sda_i, scl_o, sda_o);
parameter integer STAGES_G=3; /* slow signal */
input  wire clk;
input  wire rst;
input  wire sda_i;
output wire sda_o;
input  wire scl_i;
output wire scl_o;

localparam [STAGES_G-1:0] SYN_INIT_C = {STAGES_G { 1'b1 }};

(* ASYNC_REG = "TRUE" *) reg [STAGES_G-1:0] scl_syn = SYN_INIT_C;
(* ASYNC_REG = "TRUE" *) reg [STAGES_G-1:0] sda_syn = SYN_INIT_C;

	// registered scl/sda
	always @(posedge clk) begin
		if (rst) begin
			sda_syn <= SYN_INIT_C;
			scl_syn <= SYN_INIT_C;
		end else begin
			sda_syn <= {sda_syn[STAGES_G-2:0], sda_i};
			scl_syn <= {scl_syn[STAGES_G-2:0], scl_i};
		end
	end

	assign sda_o = sda_syn[STAGES_G-1];
	assign scl_o = scl_syn[STAGES_G-1];
	
endmodule
