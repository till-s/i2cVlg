`define CB_STRT 0
`define CB_STOP 1
`define CB_READ 2
`define CB_WRTE 3
`define CB_NACK 4

`define C_STRT   (1<<`CB_STRT)
`define C_STOP   (1<<`CB_STOP)
`define C_READ   (1<<`CB_READ)
`define C_WRTE   (1<<`CB_WRTE)
`define C_NACK   (1<<`CB_NACK)
`define C_CLRS   0

`define SB_BSY 0
`define SB_ERR 1
`define SB_ALO 2
`define SB_ACK 3

`define S_BSY (1<<`SB_BSY)
`define S_ERR (1<<`SB_ERR)
`define S_ALO (1<<`SB_ALO)
`define S_ACK (1<<`SB_ACK)

`define C_SZ 5
`define S_SZ 4

module i2c_master(clk, sda, sda_out, scl, scl_out, cmd, stat_out, dat, dat_out, ws, rst);
/* Timing given for standard/fast mode */
parameter PER_HI=2;      /* SCL period HI 4.0us/0.6us */
parameter PER_LO=2;      /* SCL period LO 4.7us/1.3us */
parameter PER_SU_STOP=2; /* setup time for STOP:     SCL raise -> SDA raise 4.0us/0.6us */
parameter PER_SU_DATA=2; /* setup time for SDA:      SDA valid -> SCL raise .24us/0.1us */
parameter PER_SU_RSRT=2; /* setup time for RESTART:  SCL raise -> SDA fall  4.7us/0.6us (? -- shouldn't it be 1.3) */
parameter PER_HD_STRT=2; /* hold time for (RE)START: SDA fall  -> SCL fall              */
parameter PER_TBUF=2;    /* Bus free time between STOP and new START 4.7us/1.3us        */
parameter SDA_PER=1;     /* sampling time to confirm STOP vs lost arbitration - not in spec;
                         /* SDA raise -> SDA sample. Must be < PER_TBUF! */
parameter PER_LD_SIZE=4;

input  clk;
input  sda;
output sda_out;
input  scl;
output scl_out;
input  [`C_SZ-1:0]cmd;
output [`S_SZ-1:0]stat_out;
input  [7:0]      dat;
output [7:0]      dat_out;
input             ws;
input             rst;

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
parameter ST_SCHK=3'b111;
reg [2:0]  state;
reg [PER_LD_SIZE-1:0] div;
reg [PER_LD_SIZE-1:0] dely;
reg [3:0]  i;
reg sda_r, scl_r;
reg [8:0]  dat_r;
reg        started;
reg [`S_SZ-1:0]  status;
parameter  GST_IDLE=2'b00;
parameter  GST_STRT=2'b01;
parameter  GST_READ=2'b10;
parameter  GST_WRTE=2'b11;
reg [1:0]  gstat;

reg [`C_SZ-1:0]  cmd_r;

wire [`S_SZ-1:0]stat_out = status;
wire [7:0]      dat_out  = dat_r[8:1];
wire            sda_out  = sda_r;
wire            scl_out  = scl_r;

	task arbitration_lost;
	begin
		$display("Arbitration lost\n");
		/* this clears the busy flag also */
		status <= (`S_ERR | `S_ALO);
		/* arbitration lost */
		/* sda_r and scl_r are both 1 at this point */
		state <= ST_IDLE;
		gstat <= GST_IDLE;
	end
	endtask

	always @(posedge sda or posedge rst) begin
		if ( rst ) begin
			dely <= 1;
		end else if ( scl ) begin
			// STOP condition detected; start dely timer
			dely <= PER_TBUF;
		end
	end

	always @(posedge clk) begin
		/* dely == 1 means no further delay */
		if ( dely > 1 )
			dely = dely - 1;
	end

	always @(posedge clk or posedge rst) begin
		if ( rst != 0 ) begin
			state <= ST_IDLE;
			wai   <= W_NONE;
			div   <= 0;
			sda_r <= 1;
			scl_r <= 1;
			status<= 0;
			gstat <= GST_IDLE;
		end else if ( ! status[`SB_BSY] && ws ) begin
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
				 */
				if ( ( ! cmd[`CB_STRT] && (
						gstat == GST_IDLE
					 || (gstat == GST_WRTE && cmd[`CB_READ])
					 || (gstat == GST_READ && cmd[`CB_WRTE])
					 ) )
					 || (cmd[`CB_READ] && cmd[`CB_WRTE])
					 || ((cmd[`CB_STRT] || gstat == GST_STRT) && cmd[`CB_READ])
				   ) begin
					status <= `S_ERR;
				end else begin
					status <= `S_BSY;
					div    <= dely > PER_SU_DATA ? dely : PER_SU_DATA;
					if ( cmd[`CB_WRTE] ) begin
						dat_r  <= {dat, 1'b1};
						state  <= ST_BITW;
					end else if ( cmd[`CB_READ] ) begin
						dat_r  <= { 8'hff, cmd[`CB_NACK] };
						state  <= ST_BITR;
					end
					if ( cmd[`CB_STRT] ) begin
						gstat  <= GST_STRT; /* must send address */
						state  <= ST_STRT;
						if ( gstat != GST_IDLE ) begin
							/* restart */
							scl_r  <= 1;
							wai    <= W_CLKH;
							div    <= PER_SU_RSRT;
						end else begin
							/* new START; must delay for remaining tbuf time */
							div    <= dely;
						end
					end else if ( (cmd & ~`C_NACK) == `C_STOP ) begin
						/* only STOP */
						div    <= PER_SU_DATA; /* not entirely clear... */
						state  <= ST_STOP;
						sda_r  <= 0;
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
									div   <= PER_LO;
								end else begin
									sda_r           <= 1;
									state           <= ST_IDLE;
									status[`SB_BSY] <= 0;
									status[`SB_ACK] <= ~dat_r[0];
									/* Start dely timer -- next time we may have to wait until
									 * next cycle can be accepted 
									 */
									dely            <= PER_LO;
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
									if ( sda_r ) begin
										/* pull SDA low and wait for the hold period */
										sda_r <= 0;
										div   <= PER_HD_STRT;
									end else begin
										scl_r <= 0;
										wai   <= W_CLKL;
										div   <= PER_LO;
										if ( cmd_r[`CB_WRTE] )
											state <= ST_BITW;
										else if ( cmd_r[`CB_READ] )
											state <= ST_BITR;
										else
											state <= ST_DONE;
									end
								end

								ST_BITR, ST_BITW:
								begin
									if ( scl_r ) begin
										wai   <= W_CLKL;
										div   <= PER_LO;
									end else begin
										wai   <= W_CLKH;
										div   <= PER_HI;
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
										status[`SB_ACK] <= ~dat_r[0];
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
