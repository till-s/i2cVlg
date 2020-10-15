`include "i2c-master.vh"

module i2c_master(clk, sda_in, sda_out, scl_in, scl_out, cmd, stat_out, dat, dat_out, ws, rst, debug); 

`include "i2c-timing-params.vh"

/* Must be max of the parameters declared in the header */
localparam MAX_PER=PER_LO;

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

localparam S_SZ_INT    = `S_SZ - 1; /* separately maintained flags: bby */

input  clk;
input  sda_in;
output sda_out;
input  scl_in;
output scl_out;
input  [`C_SZ-1:0]cmd;
output [`S_SZ-1:0]stat_out;
input  [7:0]      dat;
output [7:0]      dat_out;
input             ws;
input             rst;
output [`D_SZ-1:0]debug;

localparam W_NONE=2'b00;
localparam W_CLKH=2'b01;
localparam W_CLKL=2'b10;
reg [1:0]  wai;

localparam ST_IDLE=3'b000;
localparam ST_INIT=3'b001;
localparam ST_BITR=3'b010;
localparam ST_BITW=3'b011;
localparam ST_STRT=3'b100;
localparam ST_STOP=3'b101;
localparam ST_DONE=3'b110;
localparam ST_SCHK=3'b111;
reg [2:0]  state;

reg [PER_LD_SIZE-1:0] div;
reg [PER_LD_SIZE-1:0] dely;
reg [3:0]             i;
reg                   sda_r, scl_r;
wire                  scl, sda;
reg [8:0]             dat_r;
reg                   started;
reg [S_SZ_INT-1:0]    status;
reg                   lra;  // last read acked
wire                  bby, sto;

localparam  GST_IDLE=2'b00;
localparam  GST_STRT=2'b01;
localparam  GST_READ=2'b10;
localparam  GST_WRTE=2'b11;
reg [1:0]             gstat;

reg [`C_SZ-1:0] cmd_r;

wire [`S_SZ-1:0]stat_out = {bby, status};
wire [7:0]      dat_out  = dat_r[8:1];
wire            sda_out  = sda_r;
wire            scl_out  = scl_r;

wire [`D_SZ-1:0]debug    = {2'b0, scl_r, sda_r, 2'b0, gstat, 2'b0, wai, 1'b0, state};

	task arbitration_lost;
	begin
		$display("Arbitration lost\n");
		/* this clears the busy flag also */
		status <= (`S_ERR | `S_ALO | `S_DON);
		/* arbitration lost */
		/* sda_r and scl_r are both 1 at this point */
		state <= ST_IDLE;
		gstat <= GST_IDLE;
		lra   <= 0;
	end
	endtask
	
	// registered scl/sda
	i2c_sync U_sync(.clk(clk), .rst(rst), .scl_i(scl_in), .sda_i(sda_in), .scl_o(scl), .sda_o(sda));
	
	i2c_bby_detect #(.US(US), .I2C_MODE(I2C_MODE)) U_bby_det(.clk(clk), .scl(scl), .sda(sda), .bby(bby), .sto(sto), .rst(rst));

`ifdef TEST_WITHOUT_DELAY_TIMER
	always @(dely) begin
		dely <= 1;
	end
`endif

	always @(posedge clk or posedge rst) begin
		if ( rst != 0 ) begin
			state <= ST_INIT;
			wai   <= W_NONE;
			div   <= PER_TBUF;
			sda_r <= 1;
			scl_r <= 1;
			status<= `S_BSY;
			gstat <= GST_IDLE;
			dely  <= 1;
			lra   <= 0;
		end else begin
		if ( sto )
			dely <= PER_TBUF;
		else if ( dely > 1 ) /* dely == 1 means no further delay */
			dely <= dely - 1;
		if ( ! status[`SB_BSY] && ws ) begin
			if ( cmd == `C_CLRS ) begin
				/* clear status */
				status <= 0;
			end else begin
				/* If no start then set state to ST_BITW/ST_BITR */
				/* LSB is ACK bit and must be set when writing */
				/* all bits must be set (in dat_r) when reading */
				/* sanity checks
					- must START if we are currently idle
					- cannot READ while WRITing and vice versa -- unless restart
					- cannot READ and WRITE simultaneously
					- cannot READ just after START (must address)
					- cannot STOP if there was a preceding ack'ed READ
				 */
				if ( ( ! cmd[`CB_STRT] && (
						gstat == GST_IDLE
					 || (gstat == GST_WRTE && cmd[`CB_READ])
					 || (gstat == GST_READ && cmd[`CB_WRTE])
					 ) )
					 || (cmd[`CB_READ] && cmd[`CB_WRTE])
					 || ((cmd[`CB_STRT] || gstat == GST_STRT) && cmd[`CB_READ])
				   ) begin
					status <= `S_ERR | `S_DON;
				end else if ( gstat == GST_IDLE && bby ) begin
					status <= `S_ERR | `S_BBL | `S_DON;
				end else if (    cmd[`CB_STOP]
				              && (    ( cmd[`CB_READ] && ~ cmd[`CB_NACK] )
                                   || ( lra && ! (cmd[`CB_READ] && cmd[`CB_NACK]) )
                                 )
				            ) begin
					status <= `S_ERR | `S_LRA | `S_DON;
				end else begin
					status <= `S_BSY;
					div    <= dely > PER_SU_DATA ? dely : PER_SU_DATA;
					lra    <= 0;
					if ( cmd[`CB_WRTE] ) begin
						dat_r  <= {dat, 1'b1};
						state  <= ST_BITW;
					end else if ( cmd[`CB_READ] ) begin
						lra    <= !cmd[`CB_NACK];
						dat_r  <= { 8'hff, cmd[`CB_NACK] };
						state  <= ST_BITR;
					end
					if ( cmd[`CB_STRT] ) begin
						gstat  <= GST_STRT; /* must send address */
						state  <= ST_STRT;
						/* must delay for remaining tbuf (new start) or clock low (restart) time */
						div    <= dely;
					end else if ( (cmd & ~`C_NACK) == `C_STOP ) begin
						/* only STOP */
						state  <= ST_STOP;
						sda_r  <= 0;
					end else begin
						/* set wai <= W_CLKL so that the first data bit
						 * is pushed onto SDA if state == ST_WRTE.
						 * In all other cases it doesn't matter:
						 *  - if ST_READ -> a 1 is shifted out since dat is filled with ones
						 *  - the only other state in case(wai) below is ST_DONE which
						 *    is not used here.
						 *  - finally, for RESTART, we set wai <= W_CLKH; below
						 * The only critical case (START) where we must not wait for CLKL
						 * is avoided in this 'else' branch.
						 */
						wai    <= W_CLKL;
					end
					if ( cmd[`CB_WRTE] && (cmd[`CB_STRT] || gstat == GST_STRT) ) begin
						gstat <= dat[0] ? GST_READ : GST_WRTE;
					end
					cmd_r  <= cmd;
					i      <= 8;
				end
			end
		end else begin

			case ( wai )
				W_CLKH:
					if ( scl == 1 ) begin
						wai <= W_NONE;
						if ( sda_r && !sda && state != ST_BITR ) begin
							arbitration_lost;
						end else begin
							if ( state == ST_BITW || state == ST_BITR ) begin
								if ( i > 0 ) begin
									i <= i - 1;
								end else if ( i == 0 ) begin
									state <= ST_DONE;
								end
								if ( state == ST_BITR ) begin
									dat_r[i] <= sda;
								end
							end
						end
					end

				W_CLKL:
					if ( scl == 0 ) begin
						wai <= W_NONE;
						case ( state )
							ST_BITW: begin
								sda_r <= dat_r[i];
								if ( i == 0 )
									state <= ST_BITR;
							end

							ST_BITR: begin
								sda_r <= dat_r[i];
								if ( i == 0 )
									state <= ST_BITW;
							end

							ST_DONE: begin
								/* If we should generate stop */	
								if ( cmd_r[`CB_STOP] ) begin
									sda_r <= 0;
									state <= ST_STOP;
									div   <= PER_LO; /* SCL already low; no need to add fall-time */
								end else begin
									sda_r           <= 1;
									state           <= ST_IDLE;
									status[`SB_BSY] <= 0;
									status[`SB_DON] <= 1;
									status[`SB_ACK] <= ~dat_r[0];
									/* Start dely timer -- next time we may have to wait until
									 * next cycle can be accepted 
									 */
									dely            <= PER_LO; /* SCL already low; no need to add fall-time */
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
									if ( scl == 0 ) begin
										/* restart */
										scl_r  <= 1;
										wai    <= W_CLKH;
										div    <= PER_SU_RSRT;
									end else begin
										if ( sda_r ) begin
											/* pull SDA low and wait for the hold period */
											sda_r <= 0;
											div   <= PER_HD_STRT;
										end else begin
											scl_r <= 0;
											wai   <= W_CLKL;
											div   <= PER_LO + PER_TF;
											if ( cmd_r[`CB_WRTE] )
												state <= ST_BITW;
											else if ( cmd_r[`CB_READ] )
												state <= ST_BITR;
											else
												state <= ST_DONE;
										end
									end
								end

								ST_BITR, ST_BITW:
								begin
									if ( scl_r ) begin
										wai   <= W_CLKL;
										div   <= PER_LO + PER_TF;
									end else begin
										wai   <= W_CLKH;
										div   <= PER_HI + PER_TR;
									end
									scl_r <= ~scl_r;
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
										div   <= PER_SU_STOP;
									end else begin
										sda_r <= 1;
										div   <= SDA_PER;
										state <= ST_SCHK;
									end
								end

								ST_SCHK:
									/* Check if stop condition persists */
									if ( sda_r && ! sda ) begin
										/* scl_r and sda_r are high at this point */
										arbitration_lost;
									end else begin
										state           <= ST_IDLE;
										gstat           <= GST_IDLE;
										status[`SB_BSY] <= 0;
										status[`SB_DON] <= 1;
										status[`SB_ACK] <= ~dat_r[0];
									end

								ST_INIT:
									begin
										/* initial settling time expired; ready to go */
										state           <= ST_IDLE;
										status[`SB_BSY] <= 0;
									end

								default:
									;
							endcase
						end
					end
			endcase
		end
		end
	end
specify
	/* hold time for start condition */
	$hold(negedge sda, negedge scl, PER_HD_STRT);  // tHD;STA
	$width(negedge scl, PER_LO);                   // tLOW
	$hold(negedge scl, sda, PER_HD_DATA);          // tHD;DAT
	$setup(sda, posedge scl, PER_SU_DATA);         // tSU;DAT
	$width(posedge scl, PER_HI);                   // tHIGH
	$setup(posedge scl, negedge sda, PER_SU_RSRT); // tSU;STA
	$setup(posedge scl, posedge sda, PER_SU_STOP); // tSU;STO
endspecify
/* seems that icarus does not implement the above checks but ignores them */
`ifdef  CHECK_TIMING
	check_EE tHD_STA(rst, !sda, !scl, PER_HD_STRT);
	check_EE tLOW   (rst, !scl,  scl, PER_LO);
	check_EL tHD_DAT(rst, !scl,  sda, PER_HD_DATA);
	check_LE tSU_DAT(rst,  scl,  scl, PER_SU_DATA);
	check_EE tHI    (rst,  scl, !scl, PER_HI);
	check_EE tSU_STA(rst,  scl, !sda, PER_SU_RSRT);
	check_EE tSU_STO(rst,  scl,  sda, PER_SU_STOP);
	check_SS tBUF   (rst,  scl,  sda, PER_TBUF);
`endif
endmodule

`ifdef  CHECK_TIMING
`define CHECK_MOD(nm,e1,e2) \
	module nm(rst, s1, s2, lim); \
		input           rst; \
		input           s1, s2;  \
		input [63:0]    lim; \
		time            a,b;  \
		initial         a = 0; \
		reg            vio;  \
		initial        vio = 0; \
		reg            ini;    \
		initial        ini = 1; \
		always @(negedge rst) ini <= 0; \
		always @(posedge rst) begin \
			ini <= 1; \
			vio <= 0; \
		end \
		always @(vio) if ( vio ) #2 vio <= 0; \
		always @(e1 s1) a <= $time; \
		always @(e2 s2) begin \
			b = $time; \
			if ( b - a < lim && !ini ) begin \
				$display("Timing violation %t @%t\n", b-a, b); \
				vio <= 1; \
			end \
		end \
	endmodule

`CHECK_MOD(check_EE, posedge, posedge)
`CHECK_MOD(check_EL, posedge,        )
`CHECK_MOD(check_LE,        , posedge)
`CHECK_MOD(check_LL,        ,        )

/* measure bus-free time */
module check_SS(rst, scl, sda, lim);
	input rst, scl, sda;
	input [63:0] lim;

	time a,b;
	
	always @(posedge rst or posedge sda) begin
		if ( scl ) begin
			// STOP
			a     <= $time;
		end
	end

	always @(negedge sda) begin
		if ( scl ) begin
			b = $time;
			if (  b - a < lim ) begin
				$display("TBUF Timing violation %t @%t\n", b-a, b); 
			end
		end
	end
endmodule
`endif
