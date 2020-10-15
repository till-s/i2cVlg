parameter  US=1;        /* How many cycles per microsecond  */
parameter  I2C_MODE=1;  /* 0 -> STD (100kHz), 1 -> FM (400kHz), 2 -> FM+ (1MHz) */
`define PW 32
`define PRANGE 3*`PW-1:0
`define PSEL(arr) ((arr[`PW*(2-I2C_MODE+1)-1:`PW*(2-I2C_MODE)]*US)/NS)
localparam NS=1000;
/* SCL period HI 4.0us/0.6us/0.26us */
localparam [`PRANGE] PER_HI_ARR      = {`PW'd4000,  `PW'd600, `PW'd260 };
/* SCL period LO 4.7us/1.3us/0.50us */
localparam [`PRANGE] PER_LO_ARR      = {`PW'd4700, `PW'd1300, `PW'd500 };
/* setup time for SDA:      SDA valid -> SCL raise .24us/0.1us/0.05us */
localparam [`PRANGE] PER_SU_DATA_ARR = { `PW'd250,  `PW'd100,  `PW'd50 };
/* setup time for RESTART:  SCL raise -> SDA fall  4.7us/0.6us/0.26us */
localparam [`PRANGE] PER_SU_RSRT_ARR = {`PW'd4700,  `PW'd600, `PW'd260 };
localparam [`PRANGE] PER_TR_ARR      = {`PW'd1000,  `PW'd300, `PW'd120 };
localparam [`PRANGE] PER_TF_ARR      = { `PW'd300,  `PW'd300, `PW'd120 };

localparam PER_HI=`PSEL(PER_HI_ARR);
localparam PER_LO=`PSEL(PER_LO_ARR);
localparam PER_TR=`PSEL(PER_TR_ARR);
localparam PER_TF=`PSEL(PER_TF_ARR);
localparam PER_SU_STOP=PER_HI;   /* setup time for STOP:     SCL raise -> SDA raise 4.0us/0.6us/0.26us */
localparam PER_SU_DATA=`PSEL(PER_SU_DATA_ARR);
localparam PER_SU_RSRT=`PSEL(PER_SU_RSRT_ARR);
localparam PER_HD_STRT=PER_HI;   /* hold time for (RE)START: SDA fall  -> SCL fall   4.0us/0.6us/0.26us */
localparam PER_HD_DATA=0*US/NS;  /* hold time for data       SCL fall  -> SDA change 0us/0us (300ns internal)  but we wait for CLKL so should be OK */
/* Bus free time between STOP and new START 4.7us/1.3us/0.5us   */
localparam PER_TBUF=PER_LO;
localparam SDA_PER=2*PER_TR;     /* sampling time to confirm STOP vs lost arbitration - not in spec;
                                  * SDA raise -> SDA sample. Must be < PER_TBUF! Make twice the rise time
                                  * so that the start/stop detector can remove BBY before we declare victory! */

`undef PW
`undef PRANGE
`undef PSEL
