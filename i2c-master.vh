`ifndef I2C_MASTER_VH
`define I2C_MASTER_VH

`define C_SZ 5

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

`define S_SZ 8

/* Status bit positions        */
`define SB_DON 0
`define SB_ERR 1
`define SB_ALO 2
`define SB_BBL 3
`define SB_ACK 4
`define SB_LRA 5
`define SB_BSY 6
`define SB_BBY 7  

/* Status bits                 */
`define S_DON (1<<`SB_DON)     /* Command done                                        */
`define S_ERR (1<<`SB_ERR)     /* Error (latched) can be further qualified (ALO/BBL)  */
`define S_ALO (1<<`SB_ALO)     /* Arbitration lost (in addition to ERR)               */
`define S_BBL (1<<`SB_BBL)     /* Bus busy (latched); ERR if bus owned by other master*/
`define S_ACK (1<<`SB_ACK)     /* Received ACK (after WRTE)                           */
`define S_LRA (1<<`SB_LRA)     /* Sent ACK after last READ -- must not STOP           */
`define S_BSY (1<<`SB_BSY)     /* Controller busy (not latched)                       */
`define S_BBY (1<<`SB_BBY)     /* Bus busy (not latched)                              */

`define D_SZ 16

`endif
