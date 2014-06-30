/* Command bit positions */
`define CB_STRT 0
`define CB_STOP 1
`define CB_READ 2
`define CB_WRTE 3
`define CB_NACK 4

/* Command bits                */
`define C_STRT   (1<<`CB_STRT) /* Issue START condition (prior to optional data xfer) */
`define C_STOP   (1<<`CB_STOP) /* Issue STOP  condition (after optional data xfer )   */
`define C_READ   (1<<`CB_READ) /* Read 1 byte                                         */
`define C_WRTE   (1<<`CB_WRTE) /* Write 1 byte                                        */
`define C_NACK   (1<<`CB_NACK) /* Do not send ACK after reading byte                  */
/* C_CLRS clear latched status */ 
`define C_CLRS   0

/* Status bit positions        */
`define SB_DON 0
`define SB_ERR 1
`define SB_ALO 2
`define SB_BBL 3
`define SB_ACK 4
`define SB_BSY 5
`define SB_BBY 6  

/* Status bits                 */
`define S_DON (1<<`SB_DON)     /* Command done                                        */
`define S_ERR (1<<`SB_ERR)     /* Error (latched) can be further qualified (ALO/BBL)  */
`define S_ALO (1<<`SB_ALO)     /* Arbitration lost (in addition to ERR)               */
`define S_BBL (1<<`SB_BBL)     /* Bus busy (latched); ERR if bus owned by other master*/
`define S_ACK (1<<`SB_ACK)     /* Received ACK (after WRTE)                           */
`define S_BSY (1<<`SB_BSY)     /* Controller busy (not latched)                       */
`define S_BBY (1<<`SB_BBY)     /* Bus busy (not latched)                              */

`define C_SZ 5
`define S_SZ 7

module i2c_master(clk, sda, sda_out, scl, scl_out, cmd, stat_out, dat, dat_out, ws, rst);
/* Timing given for standard/fast mode */
parameter  US=1;                 /* How many cycles per microsecond */
localparam PER_HI=40*US/10;      /* SCL period HI 4.0us/0.6us       */
localparam PER_LO=47*US/10;      /* SCL period LO 4.7us/1.3us       */
localparam PER_SU_STOP=40*US/10; /* setup time for STOP:     SCL raise -> SDA raise 4.0us/0.6us */
localparam PER_SU_DATA=3*US/10;  /* setup time for SDA:      SDA valid -> SCL raise .24us/0.1us */
localparam PER_SU_RSRT=47*US/10; /* setup time for RESTART:  SCL raise -> SDA fall  4.7us/0.6us (? -- shouldn't it be 1.3) */
localparam PER_HD_STRT=40*US/10; /* hold time for (RE)START: SDA fall  -> SCL fall   4.0us/0.6us */
localparam PER_HD_DATA=0*US/10;  /* hold time for data       SCL fall  -> SDA change 0us/0us (300ns internal)  but we wait for CLKL so should be OK */
localparam PER_TBUF=47*US/10;    /* Bus free time between STOP and new START 4.7us/1.3us         */
localparam SDA_PER=2*10*US/10;   /* sampling time to confirm STOP vs lost arbitration - not in spec;
                                 /* SDA raise -> SDA sample. Must be < PER_TBUF! Make twice the rise time
                                  * so that the start/stop detector can remove BBY before we declare victory! */

/* Must be max of the above */
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
(* mark_debug = "true" *) input  sda;
output sda_out;
(* mark_debug = "true" *) input  scl;
output scl_out;
input  [`C_SZ-1:0]cmd;
output [`S_SZ-1:0]stat_out;
input  [7:0]      dat;
output [7:0]      dat_out;
(* mark_debug = "true" *) input             ws;
input             rst;

localparam W_NONE=2'b00;
localparam W_CLKH=2'b01;
localparam W_CLKL=2'b10;
reg [1:0]  wai, new_w_wai, new_s_wai, new_i_wai;

localparam ST_IDLE=3'b000;
localparam ST_BITR=3'b010;
localparam ST_BITW=3'b011;
localparam ST_STRT=3'b100;
localparam ST_STOP=3'b101;
localparam ST_DONE=3'b110;
localparam ST_SCHK=3'b111;
(* mark_debug= "true" *) reg [2:0]  state, new_state, new_w_state, new_s_state, new_i_state;

reg [PER_LD_SIZE-1:0] div, new_w_div, new_s_div, new_i_div, new_div;
reg [PER_LD_SIZE-1:0] dely, new_dely;
(* mark_debug = "true" *) reg [3:0]             i, new_i;
reg                   sda_r, scl_r, new_s_sda, new_s_scl, new_w_sda;
reg [8:0]             dat_r, dat_in;
reg                   started;
(* mark_debug = "true" *) reg [S_SZ_INT-1:0]    status;
wire                  bby, sto;

reg                   new_w_alo, new_s_alo;

reg                   new_dat_i;

localparam  GST_IDLE=2'b00;
localparam  GST_STRT=2'b01;
localparam  GST_READ=2'b10;
localparam  GST_WRTE=2'b11;
reg [1:0]             gstat;

reg [`C_SZ-1:0] cmd_r;

wire [`S_SZ-1:0]stat_out = {bby, status};
wire [7:0]      dat_out  = dat_in[8:1];
wire            sda_out  = sda_r;
wire            scl_out  = scl_r;

	i2c_bby_detect #(.US(US)) bby_det(.clk(clk), .scl(scl), .sda(sda), .bby(bby), .sto(sto), .rst(rst));

`ifdef TEST_WITHOUT_DELAY_TIMER
	always @(dely) begin
		dely <= 1;
	end
`endif

	always @(posedge clk or posedge rst) begin
		if ( rst != 0 ) begin
			status      <= `S_BSY;
			gstat       <= GST_IDLE;
			new_i_div   <= 0;
			new_i_state <= ST_IDLE;
			new_i_wai   <= W_NONE;
		end else begin

			new_i_div   <= 0;
			new_i_state <= ST_IDLE;
			new_i_wai   <= W_NONE;

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
					end else begin
						status     <= `S_BSY;
						new_i_div  <= dely > PER_SU_DATA ? dely : PER_SU_DATA;
						if ( cmd[`CB_WRTE] ) begin
							dat_r  <= {dat, 1'b1};
							new_i_state  <= ST_BITW;
						end else if ( cmd[`CB_READ] ) begin
							dat_r  <= { 8'hff, cmd[`CB_NACK] };
							new_i_state  <= ST_BITR;
						end
						if ( cmd[`CB_STRT] ) begin
							gstat        <= GST_STRT; /* must send address */
							new_i_state  <= ST_STRT;
							/* must delay for remaining tbuf (new start) or clock low (restart) time */
							new_i_div    <= dely;
						end else if ( (cmd & ~`C_NACK) == `C_STOP ) begin
							/* only STOP */
							new_i_state  <= ST_STOP;
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
							new_i_wai   <= W_CLKL;
						end
						if ( cmd[`CB_WRTE] && (cmd[`CB_STRT] || gstat == GST_STRT) ) begin
							gstat <= dat[0] ? GST_READ : GST_WRTE;
						end
						cmd_r  <= cmd;
					end
				end
			end else begin

				if ( new_s_alo || new_w_alo ) begin
					$display("Arbitration lost\n");
					/* this clears the busy flag also */
					status <= (`S_ERR | `S_ALO | `S_DON);
					/* arbitration lost */
					/* sda_r and scl_r are both 1 at this point */
					gstat <= GST_IDLE;
				end

				if ( new_state == ST_IDLE ) begin
					if ( status[`SB_BSY] ) begin
						status[`SB_DON] <= 1;
						status[`SB_ACK] <= ~dat_in[0];
						status[`SB_BSY] <= 0;
					end
					if ( sto )
						gstat <= GST_IDLE;
				end
			end
		end
	end

	// compute new wait state

	always @(*) begin

		new_w_wai   = wai;
		new_i       = i;
		new_w_state = state;
		new_dat_i   = dat_in[i];
		new_w_sda   = sda_r;
		new_dely    = 0;
		new_w_div   = 0;
		new_w_alo   = 0;

		case ( wai )
			W_CLKH:
				if ( scl == 1 ) begin
					new_w_wai = W_NONE;
					if ( sda_r && !sda && state != ST_BITR ) begin
						new_w_alo = 1;
						new_w_state = ST_IDLE;
					end else begin
						if ( state == ST_BITW || state == ST_BITR ) begin
							if ( i > 0 ) begin
								new_i = i - 1;
							end else if ( i == 0 ) begin
								new_w_state = ST_DONE;
							end
							if ( state == ST_BITR ) begin
								new_dat_i = sda;
							end
						end
					end
				end

			W_CLKL:
				if ( scl == 0 ) begin
					new_w_wai = W_NONE;
					case ( state )
						ST_BITW: begin
							new_w_sda = dat_r[i];
							if ( i == 0 )
								new_w_state = ST_BITR;
						end

						ST_BITR: begin
							new_w_sda = dat_r[i];
							if ( i == 0 )
								new_w_state = ST_BITW;
						end

						ST_DONE: begin
							/* If we should generate stop */	
							if ( cmd_r[`CB_STOP] ) begin
								new_w_sda       = 0;
								new_w_state     = ST_STOP;
								new_w_div       = PER_LO;
							end else begin
								new_w_sda       = 1;
								new_w_state     = ST_IDLE;
								/* Start dely timer -- next time we may have to wait until
								 * next cycle can be accepted 
								 */
								new_dely         = PER_LO;
							end
						end

						default: /* never reached */;
					endcase
				end
		endcase
	end

	// compute new state
	always @(*) begin

		new_s_state = state;
		new_s_sda   = sda_r;
		new_s_scl   = scl_r;
		new_s_wai   = W_NONE;
		new_s_div   = 0;
		new_s_alo   = 0;

		if ( div == 1 ) begin

		case ( state )
			ST_STRT:
			begin
				if ( scl == 0 ) begin
					/* restart */
					new_s_scl = 1;
					new_s_wai = W_CLKH;
					new_s_div = PER_SU_RSRT;
				end else begin
					if ( sda_r ) begin
						/* pull SDA low and wait for the hold period */
						new_s_sda = 0;
						new_s_div = PER_HD_STRT;
					end else begin
						new_s_scl = 0;
						new_s_wai = W_CLKL;
						new_s_div = PER_LO;
						if ( cmd_r[`CB_WRTE] )
							new_s_state = ST_BITW;
						else if ( cmd_r[`CB_READ] )
							new_s_state = ST_BITR;
						else
							new_s_state = ST_DONE;
					end
				end
			end

			ST_BITR, ST_BITW:
			begin
				if ( scl_r ) begin
					new_s_wai = W_CLKL;
					new_s_div = PER_LO;
				end else begin
					new_s_wai = W_CLKH;
					new_s_div = PER_HI;
				end
				new_s_scl <= ~scl_r;
			end

			ST_DONE:
			begin
				new_s_scl = 0;
				new_s_wai = W_CLKL;
			end

			ST_STOP:
			begin
				if ( scl_r == 0 ) begin
					new_s_scl   = 1;
					new_s_wai   = W_CLKH;
					new_s_div   = PER_SU_STOP;
				end else begin
					new_s_sda   = 1;
					new_s_div   = SDA_PER;
					new_s_state = ST_SCHK;
				end
			end

			ST_SCHK:
				begin
					/* Check if stop condition persists */
					if ( sda_r && ! sda ) begin
						/* scl_r and sda_r are high at this point */
						new_s_alo = 1;
					end
					new_s_state     = ST_IDLE;
				end

			default:
				;
		endcase

		end
	end


	// sda and scl
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			sda_r <= 1;
			scl_r <= 1;
		end else begin
			if ( wai != W_NONE )
				sda_r <= new_w_sda;
			else
				sda_r <= new_s_sda;
			if ( new_i_state == ST_STOP )
				sda_r <= 0;
			scl_r <= new_s_scl;
		end
	end

	// wait state machine
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			wai      <= W_NONE;
		end else begin
			// only one of these can be nonzero (W_NONE)
			wai       <= new_w_wai | new_s_wai | new_i_wai;
			dat_in[i] <= new_dat_i;
			i         <= new_i;
			if ( state == ST_IDLE )
				    i <= 8; // reset bit counter
		end
	end

	always @(*) begin
		// only one of these can be set at once
		new_div = new_w_div | new_s_div | new_i_div;
	end

	// div counter
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			div   <= PER_TBUF;
		end else begin
			if ( new_div ) begin
				div <= new_div;
			end else if ( div > 0 && wai == W_NONE ) begin
				div <= div - 1;
			end
		end
	end

	// delay counter
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			dely <= 1;
		end else begin
			if ( sto )
				dely <= PER_TBUF;
			else if ( new_dely )
				dely <= new_dely;
			else if ( dely > 1 ) /* dely == 1 means no further delay */
				dely <= dely - 1;
		end
	end

	always @(*) begin
			if ( new_i_state != ST_IDLE )
				new_state <= new_i_state;
			else if ( wai != W_NONE )
				new_state <= new_w_state;
			else 
				new_state <= new_s_state;
	end

	// state machine
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			state <= ST_SCHK;
		end else begin
			state <= new_state;
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
