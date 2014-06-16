module pin_drv(pin, o);
	inout pin;
	input  o;
	bufif0 #(1,0,1)(pin, 0, o);
endmodule

module i2c_s(sda, scl);
inout sda, scl;
reg [7:0] i;
reg rst;
reg [7:0] dat;
reg [7:0] dat_out;
reg sda_r, sda_l;
reg ack;
parameter ST_IDLE=3'b000;
parameter ST_ADDR=3'b001;
parameter ST_READ=3'b010;
parameter ST_WRTE=3'b011;
reg [2:0] state;
parameter MYADDR=(7'h3b);
integer det;

	pin_drv sda_drv(sda, sda_r);

	initial begin 
		rst = 0;
		det = 0;
		sda_r = 1;
		dat_out = 8'ha5;
		state = ST_IDLE;
	end

	// detect STOP
	always @(posedge sda) begin
		if ( scl != 0 ) begin
			rst <= 1;
		end
	end

	// detect START
	always @(negedge sda) begin
		if ( scl != 0 ) begin
			if ( state == ST_IDLE ) begin
				state <= ST_ADDR;	
				i     <= 0;
			end else begin
				$display("Unexpected state in START detector: %i\n", state);
				rst <= 1;
			end
		end
	end

	always @(scl or posedge rst) begin
		if ( rst == 1 ) begin
			state <= ST_IDLE;
			rst   <= 0;
			sda_r <= 1'b1;
		end else begin
			case ( state ) 
				ST_WRTE: if ( scl == 0 ) begin
					i <= i+1;
					if ( i < 7 ) begin
						sda_r <= dat_out[6 - i];
					end else if ( i == 7 ) begin
						sda_r <= 1'b1;
					end else begin
						if ( ack )
							sda_r = dat_out[7];
						i <= 0;
					end
				end else begin
					if ( i == 8 ) begin
						ack = !sda;	
					end
				end

				ST_ADDR,
				ST_READ: begin
					if ( scl == 1 ) begin
						if ( i < 8 ) begin
							i <= i + 1;
							dat <= {dat[6:0], sda};
						end
					end else begin
						if ( i == 8 ) begin
							if ( state != ST_ADDR || (dat & 8'hfe) == (MYADDR << 1 ) ) begin
								sda_r <= 1'b0;
								i <= i+1;
							end else begin
								/* NO ACK */
								state <= ST_IDLE;
							end
						end else if ( i == 9 ) begin
							if ( state == ST_ADDR ) begin
								if ( dat[0] == 0 ) begin
									state <= ST_READ;
								end else begin
									state <= ST_WRTE;
									sda_r = dat_out[7];	
								end
							end
							i <= 0;
						end
					end
				end

				default: state = ST_IDLE;
			endcase
		end
	end

`ifdef XXX
	always @(state or rst)  begin
		if ( rst ) begin
			state <= ST_IDLE;	
			rst   <= 0;
		end else
		case ( state )

			ST_IDLE: begin
			
				@(negedge sda or posedge rst) begin
					if ( rst != 0 ) begin
						rst <= 0;
					end else begin
						state <= ST_ADDR;
					end
				end
			end

			ST_WRTE: begin
				for ( i=0; i<8; i=i+1 ) begin
					@(negedge scl or posedge rst) begin
						if ( rst != 0 ) begin
							i = 8;
						end else begin
							sda_r <= dat[7-i];
						end
					end
				end
				if ( ! rst ) begin
					@(negedge scl or posedge rst) begin
						if ( ! rst ) begin
							sda_r <= 1'b1;
							@(negedge scl or posedge rst)
								;
						end
					end
				end
				if ( rst ) begin
					sda_r <= 1'b1;
					state <= ST_IDLE;
				end
			end

			ST_ADDR,
				// START detected
			ST_READ: begin
				for ( i=0; i<8; i=i+1 ) begin
					@(posedge scl or rst) begin
						if ( rst != 0 ) begin
							i=8;
						end else begin
							sda_l = sda;
							dat <= { dat[6:0], sda_l };
						end
					end
				end
				if ( ! rst ) begin
					if ( state != ST_ADDR || (dat & 8'h3f) == MYADDR ) begin
						@(negedge scl or rst) begin
							if ( ! rst ) begin
								sda_r <= 0;
								@(negedge scl or rst) begin
									sda_r <= 1;
									if ( ! rst && state == ST_ADDR ) begin
										if ( sda_l == 0 ) begin
											state <= ST_READ;
										end else begin
											state <= ST_WRTE;
											dat   <= 8'h5a;
										end
									end
								end
							end
						end
					end else begin
						/* state == ST_ADDR && dat != MYADDR */	
						state <= ST_IDLE;
					end	
				end
				if ( rst ) begin
					rst   <= 0;
					state <= ST_IDLE;
				end
			end

			default: state <= ST_IDLE; 
		endcase
	end
`endif
endmodule

`define C_STRT   4'b0010
`define C_STOP   4'b0001
`define C_READ   4'b0100
`define C_NACK   4'b1000
`define C_STLY   4'b0000

module i2c_m(bsy, clk, sda, scl, cmd_in, dat_in, ds, rst);
parameter PER=2;
parameter SDA_PER=1;
input clk;
inout sda;
inout scl;
input [7:0]dat_in;
input [3:0]cmd_in;
input ds;
input rst;
output bsy;

parameter W_NONE=2'b00;
parameter W_CLKH=2'b01;
parameter W_CLKL=2'b10;
reg [1:0]  wai;

parameter ST_IDLE=3'b000;
parameter ST_BITR=3'b010;
parameter ST_BITW=3'b011;
parameter ST_STRT=3'b100;
parameter ST_STOP=3'b101;
parameter ST_DONE=3'b110;
reg [2:0]  state;
reg [31:0] div;
reg [3:0]  i;
reg sda_r, scl_r;
reg [8:0]  dat;
reg        bsy;
reg        alo;

reg [3:0]  cmd;

	pin_drv sda_pin(sda, sda_r);
	pin_drv scl_pin(scl, scl_r);

	always @(posedge clk or posedge rst) begin
		if ( rst != 0 ) begin
			state <= ST_IDLE;
			wai   <= W_NONE;
			div   <= 0;
			sda_r <= 1;
			scl_r <= 1;
			alo   <= 0;

			bsy   <= 0;
		end else if ( ! bsy && ds ) begin
			/* If no start then set state to ST_BITW/ST_BITR */
			/* LSB is ACK bit and must be set when writing */
			/* all bits must be set (in dat) when reading */
			bsy    <= 1;
			div    <= PER;
			if ( (cmd_in & `C_READ) == 0 ) begin
				dat    <= {dat_in, 1'b1};
				state  <= ST_BITW;
			end else begin
				dat    <= { 8'hff, ((cmd_in & `C_NACK) ? 1'b1 : 1'b0) };
				state  <= ST_BITR;
			end
			if ( (cmd_in & `C_STRT) != 0 ) begin
				state  <= ST_STRT;
				sda_r  <= 0;
			end else if ( cmd_in == `C_STLY ) begin
				/* only STOP */
				state  <= ST_STOP;
				sda_r  <= 0;
			end
			cmd    <= cmd_in;
			i      <= 8;
		end else begin

			case ( wai )
				W_CLKH:
					if ( scl == 1 ) begin
						wai <= W_NONE;
						if ( sda_r && !sda && state != ST_BITR ) begin
							$display("Arbitration lost\n");
							alo   <= 1;
							/* arbitration lost */
							/* sda_r and scl_r are both 1 at this point */
							state <= ST_IDLE;
							bsy   <= 0;
						end else begin
							if ( state == ST_BITW || state == ST_BITR ) begin
								if ( i > 0 ) begin
									i <= i - 1;
								end else if ( i == 0 ) begin
									state <= ST_DONE;
								end
								if ( state == ST_BITR ) begin
									dat[i] <= sda;
								end
							end
						end
					end

				W_CLKL:
					if ( scl == 0 ) begin
						wai <= W_NONE;
						case ( state )
							ST_BITW: begin
								sda_r <= dat[i];
								if ( i == 0 )
									state <= ST_BITR;
							end

							ST_BITR: begin
								sda_r <= dat[i];
								if ( i == 0 )
									state <= ST_BITW;
							end

							ST_DONE: begin
								/* If we should generate stop */	
								if ( (cmd & `C_STOP) ) begin
									sda_r <= 0;
									state <= ST_STOP;
									div   <= PER;
								end else begin
									sda_r <= 1;
									state <= ST_IDLE;
									bsy   <= 0;
								end
							end

							default: /* never reached */;
						endcase
					end

				default:
					if ( div > 0 ) begin
						div <= div - 1;
						if ( div == 1 ) begin
							case ( state )
								ST_STRT:
								begin
									scl_r <= 0;
									wai   <= W_CLKL;
									div   <= PER;
									state <= ((cmd & `C_READ) == 0 ) ? ST_BITW : ST_BITR;
								end

								ST_BITR, ST_BITW:
								begin
									scl_r <= ~scl_r;
									div   <= PER;
									wai   <= scl_r ? W_CLKL : W_CLKH;
								end

								ST_DONE:
								begin
									scl_r <= 0;
									wai   <= W_CLKL;
								end

								ST_STOP:
								begin
									if ( scl_r == 0 ) begin
										scl_r <= 1;
										wai   <= W_CLKH;
										div   <= PER;
									end else begin
										sda_r <= 1;
										div   <= SDA_PER;
										wai   <= W_CLKH; /* delay a little bit and check SDA */
										state <= ST_IDLE;
										bsy   <= 0;
									end
								end

								default:
									;
							endcase
						end
					end
			endcase
		end
	end
endmodule

module test;
	tri1 sda, scl;
	reg  clk, rst, ds;
	reg  [7:0] dat;
	reg  [3:0] cmd;
	wire       bsy;

	i2c_s slv(sda, scl);
//	pin_drv clkdrv(scl, clk);
	i2c_m mst(bsy, clk, sda, scl, cmd, dat, ds, rst);

	always #1 clk=~clk;

	initial
	begin
		$dumpvars;
		clk = 0;
		rst = 1;
		dat = 8'h5a;
		cmd = `C_STRT | `C_STOP;
		#1 rst = 0;
		#2 ds  = 1;
		#2 ds  = 0;
		@(negedge bsy);
		dat = (7'h3b << 1) | 1'b1;
		cmd = `C_STRT;
		#2 ds  = 1;
		#2 ds  = 0;
		@(negedge bsy);

		cmd = `C_READ;
		#2 ds  = 1;
		#2 ds  = 0;
		@(negedge bsy);

		cmd = `C_READ | `C_NACK;
		#2 ds  = 1;
		#2 ds  = 0;
		@(negedge bsy);

		cmd = `C_STLY;
		#2 ds  = 1;
		#2 ds  = 0;
		@(negedge bsy);
				
		#200;
		$finish;
	end
endmodule
