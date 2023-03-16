`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//  INTERFACING NRF24L01 WITH FPGA...THIS IS A RECEIVER CODE WHICH IS SPI CONTROLLED.
//  HERE WE ARE RECEIVING DATA CONTINOIOUSLY FROM TRANSMITTER AS CE=1.
// 
// 
// ////////////////////////////////////////////////////////////////////////////////// 


module nrf(clk,clk2mhz,reset,spi_sck,spi_ce,spi_csn,spi_miso,spi_mosi,nrf_out,spi_ss_b,sf_ce0,amp_cs,
fpga_init_b,dac_cs,ad_conv);

input clk,reset;
output reg clk2mhz;
output reg spi_sck,spi_csn,spi_ce;
output reg spi_mosi;
input spi_miso;
output reg [7:0]nrf_out;

output  spi_ss_b,sf_ce0,fpga_init_b,dac_cs,ad_conv,amp_cs;  // DISABLING SIGNAL


reg [7:0]rx_address =  8'b11101101 ;  // TX_ADDRESS=ED

//reg [7:0]tx_address2 =  8'b11100111 ;  // TX_ADDRESS=E7
//reg [7:0]tx_address3 =  8'b11100111 ;  // TX_ADDRESS=E7
//reg [7:0]tx_address4 =  8'b11100111 ;  // TX_ADDRESS=E7
//reg [7:0]tx_address5 =  8'b11100111 ;  // TX_ADDRESS=E7



reg [7:0]CONFIG_value=8'b00011111; // 0x1E--Enable CRC of 2 Byte and PWR_UP=1 i.e. PRX=1 ,Receiver

reg [7:0]EN_AA_value     =8'b00000000;  // Disable Auto Acknowledgement
reg [7:0]EN_RXADDR_value =8'b00000001;  // Enable data pipe 0
reg [7:0]SETUP_AW_value  =8'b00000011;  // Address width is 5 Bytes

reg [7:0]RF_CH_value     =8'b00000001;  //RF channel registry 0b0000 0001 =2,401GHz (same in the TX RX)
reg [7:0]RF_SETUP_value  =8'b00000111;  //1Mbps,-0 dBm
reg [7:0]STATUS_value    =8'b01000000;  //Reset all IRQ in status register (to be able to listen again)
reg [7:0]RX_PW_P0_value  =8'b00000001;  //1 Byte Payload
reg [7:0]SETUP_RETR_value=8'b00000000;

//reg [7:0]SETUP_RETR_value=8'b00011110;
//reg [7:0]CONFIG_value=8'b00011110;
//reg [7:0]RX_ADDR_P0_value=8'b00011110;


reg [7:0]payload1=8'b01110010; // 8 bit payload data 
reg [7:0]payload2=8'b11101010;
reg [7:0]payload3=8'b00111010;





reg [14:0]count_pwr_on_off=0; //counter for power off to power on state(12 ms)

reg [9:0]count_pwr_down_pwr_up=0; // counter for power down to power up mode

reg [7:0]count_pwr_up_tx_rx=0;

reg [6:0]clk_count=0; // for dividing 50 mHZ TO 2 mHZ CLOCK
reg [3:0]reg_count=8; for shifting 8 bit SPI data 
reg [3:0]status_count=8;
reg [3:0]status_clk=8;
reg [10:0]countm=0;
reg [7:0]ce_count=0; //CE count ,which is minimum 10 micro second
reg [10:0]count_standby=0; // counter for POWER DOWN to STANDBY (min. 2 milisecond)
reg [2:0]payload_count=0; // number of payload to be sent



reg [8:0]state=000000000; // states of nrf24l01


// DISBALING OTHER PERIPHERAL COMMUNICATING WITH SPI BUS

assign spi_ss_b=0;  // Disable SPI Flash on spartan 3E 
assign sf_ce0=1;  //FLASH
assign fpga_init_b=1;
assign dac_cs=1;  //DAC
assign ad_conv=0;  //ADC
assign amp_cs=1;  ADC

// MEMORY MAP for WRITING into Nrf Register--W_REGISTER---[001A AAAA] FORMAT

parameter  	CONFIG_W      =8'b00100000,//8'h0
 		EN_AA_W        =8'b00100001,//8'h1
 		EN_RXADDR_W    =8'b00100010,//8'h2
 		SETUP_AW_W     =8'b00100011,//8'h3
 		SETUP_RETR_W   =8'b00100100,//8'h4
 		RF_CH_W        =8'b00100101,//8'h5
 		RF_SETUP_W     =8'b00100110,//8'h6
 		STATUS_W       =8'b00100111,//8'h7
 		OBSERVE_TX_W   =8'b00101000,//8'h8
 		CD_W           =8'b00101001,//8'h9
 		RX_ADDR_P0_W   =8'b00101010,//8'hA
 		RX_ADDR_P1_W   =8'b00101011,//8'hB
 		RX_ADDR_P2_W   =8'b00101100,//8'hC
 		RX_ADDR_P3_W   =8'b00101101,//8'hD
 		RX_ADDR_P4_W   =8'b00101110,//8'hE
 		RX_ADDR_P5_W   =8'b00101111,//8'hF
 		TX_ADDR_W      =8'b00110000,//8'h10
 		RX_PW_P0_W     =8'b00110001,//8'h11
 		RX_PW_P1_W     =8'b00110010,//8'h12
 		RX_PW_P2_W     =8'b00110011,//8'h13
 		RX_PW_P3_W     =8'b00110100,//8'h14
 		RX_PW_P4_W     =8'b00110101,//8'h15
 		RX_PW_P5_W     =8'b00110110,//8'h16
 		FIFO_STATUS_W  =8'b00110111;//8'h17
 		
 		
// MEMORY MAP FOR READING Nrf REGISTER----R_REGISTER---[000A AAAA] FORMAT

parameter  	CONFIG_R      =8'b00000000,//8'h0
 		EN_AA_R       =8'b00000001,//8'h1
 		EN_RXADDR_R   =8'b00000010,//8'h2
 		SETUP_AW_R    =8'b00000011,//8'h3
 		SETUP_RETR_R  =8'b00000100,//8'h4
 		RF_CH_R       =8'b00000101,//8'h5
 		RF_SETUP_R    =8'b00000110,//8'h6
 		STATUS_R      =8'b00000111,//8'h7
 		OBSERVE_TX_R  =8'b00001000,//8'h8
 		CD_R          =8'b00001001,//8'h9
 		RX_ADDR_P0_R  =8'b00001010,//8'hA
 		RX_ADDR_P1_R  =8'b00001011,//8'hB
 		RX_ADDR_P2_R  =8'b00001100,//8'hC
 		RX_ADDR_P3_R  =8'b00001101,//8'hD
 		RX_ADDR_P4_R  =8'b00001110,//8'hE
 		RX_ADDR_P5_R  =8'b00001111,//8'hF
 		TX_ADDR_R     =8'b00010000,//8'h10
 		RX_PW_P0_R    =8'b00010001,//8'h11
 		RX_PW_P1_R    =8'b00010010,//8'h12
 		RX_PW_P2_R    =8'b00010011,//8'h13
 		RX_PW_P3_R    =8'b00010100,//8'h14
 		RX_PW_P4_R    =8'b00010101,//8'h15
 		RX_PW_P5_R    =8'b00010110,//8'h16
 		FIFO_STATUS_R =8'b00010111;//8'h17


// SPI INSTRUCTION
parameter  	R_REGISTER    =8'b00000000,//8'h0
 		W_REGISTER    =8'b00100000,//8'h20
 		REGISTER_MASK =8'b00011111,//8'h1F
 		R_RX_PAYLOAD  =8'b01100001,//8'h61
 		W_TX_PAYLOAD  =8'b10100000,//8'hA0
 		FLUSH_TX      =8'b11100001,//8'hE1
 		FLUSH_RX      =8'b11100010,//8'hE2
 		REUSE_TX_PL   =8'b11100011,//8'hE3
 		NOP           =8'b11111111;//8'hFF

////////////////  50 mhz to 2 Mhz division /////////////////////////////////


always@(posedge clk )  // 2 Mhz clock
 
begin
if((clk_count>=25)&& (clk_count<50))
begin
clk2mhz<=1;
clk_count<=clk_count+1;
end
else
begin
clk2mhz<=0;
clk_count<=clk_count+1;
end
if(clk_count>=50)
  begin
    clk_count<=0;
    
    end
end


always @(posedge clk2mhz or posedge reset)
begin

if(reset)

begin
spi_mosi<=0;
spi_csn<=1;
spi_ce<=0;
spi_sck<=0;
state<=1;
nrf_out<=8'b01111110; // default nrf output (user define)
end


else 
begin
case(state)  ///////  NRF states start here

1:begin
if(count_pwr_on_off==6000)  //12 Ms //A 10.3 ms delay is required between power off and power on states (controlled by 3.3 V supply)
begin
count_pwr_on_off<=0;
state<=state+1;
end
else
count_pwr_on_off<=count_pwr_on_off+1;
spi_sck<=0;
end


2:begin
spi_csn<=0;
spi_ce<=1;
spi_sck<=0;
state<=state+1;
end

3:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=CONFIG_W[reg_count-1]; // Configuration of CONFIG REGISTER for CRC,EN_CRC,PWR_UP,PRIM_RX_TX
reg_count<=reg_count-1;
state<=state+1;
end

4:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=3;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

5:begin
if(countm==5)  //waiting for sometime between CONFIG write and CONFIG_VALUE write
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

6:begin

spi_sck<=0;
spi_mosi<=CONFIG_value[reg_count-1]; //writing data into  CONFIG REGISTER .
reg_count<=reg_count-1;
state<=state+1;
end

7:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=6;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

8:begin
if(countm==5)  //delay for next configuration
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

9:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

10:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

11:begin
spi_sck<=0;
spi_csn<=1;
state<=12;
end


12:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end




13:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=EN_AA_W[reg_count-1];   // Configuration of EN_AA REGISTER for Enable ‘Auto Acknowledgment
reg_count<=reg_count-1;
state<=state+1;
end

14:begin

spi_sck<=1;
if(reg_count>0) // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=13;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

15:begin
if(countm==5)  //DELAY BETWEEN data given into EN_AA Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

16:begin

spi_sck<=0;
spi_mosi<=EN_AA_value[reg_count-1]; // writing data into  EN_AA Register i.e  disabling auto acknowledgment
reg_count<=reg_count-1;
state<=state+1;
end

17:begin

spi_sck<=1;
if(reg_count>0) // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=16;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

18:begin
if(countm==5)   //delay for next configuration
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

19:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

20:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

21:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


22:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end




23:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=EN_RXADDR_W[reg_count-1]; // Configuration of EN_RXADDR REGISTER TO ENABLE particular PIPE
reg_count<=reg_count-1;
state<=state+1;
end

24:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=23;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

25:begin
if(countm==5)  //DELAY BETWEEN data given into EN_RXADDR Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

26:begin

spi_sck<=0;
spi_mosi<=EN_RXADDR_value[reg_count-1]; // writing data into  EN_RXADDR Register
reg_count<=reg_count-1;
state<=state+1;
end

27:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=26;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

28:begin
if(countm==5)   //DELAY for next configuration register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

29:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

30:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

31:begin
spi_sck<=0;
spi_csn<=1;
state<=33;
end


33:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end



34:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=SETUP_AW_W[reg_count-1]; // Configuration of SETUP_AW REGISTER for setting address width
reg_count<=reg_count-1;
state<=state+1;
end

35:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=34;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

36:begin
if(countm==5)   //DELAY BETWEEN data given into SETUP_AW REGISTER
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

37:begin

spi_sck<=0;
spi_mosi<=SETUP_AW_value[reg_count-1];  // writing data into  SETUP_AW Register
reg_count<=reg_count-1;
state<=state+1;
end

38:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=37;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

39:begin
if(countm==5)   //DELAY for next configuration register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

40:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

41:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

42:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


43:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end


44:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=RF_CH_W[reg_count-1]; // Configuration of RF_CH REGISTER TO set freq of channel
reg_count<=reg_count-1;
state<=state+1;
end

45:begin

spi_sck<=1;
if(reg_count>0) // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=44;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

46:begin
if(countm==5)  //DELAY BETWEEN data given into RF_CH Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

47:begin

spi_sck<=0;
spi_mosi<=RF_CH_value[reg_count-1]; // writing data into  RF_CH Register
reg_count<=reg_count-1;
state<=state+1;
end

48:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=47;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

49:begin
if(countm==5)  //DELAY for next configuration register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

50:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

51:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

52:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


53:begin
spi_csn<=0;
//spi_ce<=0;
spi_sck<=0;
state<=state+1;
end



54:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=RF_SETUP_W[reg_count-1];  // Configuration of RF_SETUP REGISTER TO set RF output power,data rate
reg_count<=reg_count-1;
state<=state+1;
end

55:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=54;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

56:begin
if(countm==5)   //DELAY BETWEEN data given into RF_SETUP Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

57:begin

spi_sck<=0;
spi_mosi<=RF_SETUP_value[reg_count-1];  // writing data into  RF_SETUP Register
reg_count<=reg_count-1;
state<=state+1;
end

58:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=57;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

59:begin
if(countm==5)  //DELAY for next configuration register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

60:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

61:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

62:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


63:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end



64:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=SETUP_RETR_W[reg_count-1]; // Configuration of SETUP_RETR REGISTER for Setting up of Automatic Retransmission
reg_count<=reg_count-1;
state<=state+1;
end

65:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=64;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

66:begin
if(countm==5) //DELAY BETWEEN data given into SETUP_RETR Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

67:begin

spi_sck<=0;
spi_mosi<=SETUP_RETR_value[reg_count-1];  // writing data into  SETUP_RETR Register
reg_count<=reg_count-1;
state<=state+1;
end

68:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=67;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

69:begin
if(countm==5)  //DELAY for next configuration register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end



70:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

71:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

72:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


73:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end





74:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=RX_PW_P0_W[reg_count-1];  // Configuration of RX_PW_P0 REGISTER TO set RX payload width
reg_count<=reg_count-1;
state<=state+1;
end

75:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=74;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

76:begin
if(countm==5)   //DELAY BETWEEN data given into RX_PW_P0 Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

77:begin

spi_sck<=0;
spi_mosi<=RX_PW_P0_value[reg_count-1];  // writing data into  RX_PW_P0 Register (1 byte)
reg_count<=reg_count-1;
state<=state+1;
end

78:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=77;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

79:begin
if(countm==5) //DELAY for next configuration register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

80:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

81:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

82:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


83:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end


84:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=RX_ADDR_P0_W[reg_count-1]; // Configuration of RX_ADDR_PO WHICH IS EQUAL TO tx_addr address
reg_count<=reg_count-1;
state<=state+1;
end

85:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=84;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

86:begin
if(countm==5)   //DELAY BETWEEN data given into TX_ADDR Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

87:begin

spi_sck<=0;
spi_mosi<=rx_address[reg_count-1];  // writing address into  RX_ADDR_P0 Register (1 byte 5 times)(ED_ED_ED_ED_ED=TX_ADDR)
reg_count<=reg_count-1;
state<=state+1;
end

88:begin

spi_sck<=1;
if(reg_count>0)   // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=87;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end




89:begin
if(countm==4)  //for 40 bit addressing ,this loop is used 5 times for RX_ADDRESS=ED
begin
countm<=0;
state<=state+1;
end
else
begin
countm<=countm+1;
spi_sck<=0;
state<=87;  //repetation till 40 bit RX Address address is completed
end
end


90:begin
if(countm==5)  //DELAY for next configuration register
begin
countm<=0;
state<=state+1;
end
else
begin
countm<=countm+1;
spi_sck<=0;
end
end



91:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

92:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

93:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


94:begin
spi_csn<=0;
//spi_ce<=0;
spi_sck<=0;
state<=state+1;
end



95:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end

96:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=FLUSH_RX[reg_count-1];  // flush RX fifo which removeS all the data from RX FIFO permanently
reg_count<=reg_count-1;
state<=state+1;
end

97:begin

spi_sck<=1;
if(reg_count>0)   // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=96;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end



98:begin
if(countm==5)   //Delay for next configuration
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end



99:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

100:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

101:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

102:begin
spi_sck<=0;
spi_csn<=0;
state<=state+1;
end


103:begin
spi_csn<=0;
spi_sck<=0;
spi_mosi<=STATUS_W[reg_count-1];  // Configuration of STATUS REGISTER TO clear all the FLAGS
reg_count<=reg_count-1;
state<=state+1;
end

104:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=103;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

105:begin
if(countm==5)  //DELAY BETWEEN data given into STATUS Register
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end

106:begin

spi_sck<=0;
spi_mosi<=STATUS_value[reg_count-1]; // writing data into  STATUS Register
reg_count<=reg_count-1;
state<=state+1;
end

107:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=106;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

108:begin
if(countm==5)  // DELAY FOR next operartion
begin
countm<=0;
state<=state+1;//////////////////////////////
//state<=121;
end
else
countm<=countm+1;
spi_sck<=0;
end



//////////////////////////  TO READ DATA FROM RX_FIFO TX FIFO  //////////////////////
109:begin
spi_sck<=0;
payload_count<=payload_count+1;  //////////////////////////
spi_csn<=1;
state<=state+1;
end

110:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

111:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end


112:begin
spi_csn<=0;
spi_sck<=0;
state<=state+1;
end

113:begin

spi_sck<=0;
spi_mosi<=R_RX_PAYLOAD[reg_count-1];  // Read RX_PAYLOAD command
reg_count<=reg_count-1;
state<=state+1;
end

114:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=113;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end


115:begin
spi_sck<=0;
spi_csn<=0;
state<=state+1;
end

116:begin
spi_sck<=0;
spi_csn<=0;
state<=state+1;
//payload_count<=payload_count+1;
end


117:begin

spi_sck<=0;
spi_mosi<=payload1[reg_count-1];  // give any 1 byte data (as WIDTH OF PAYLOAD was set to 1 Byte IN TX and RX)
reg_count<=reg_count-1;
state<=state+1;

end

118:begin

spi_sck<=1;
if(reg_count>0)  // SHIFTING 8 BIT DATA THROUGH SPI
begin
state<=117;
end

else

begin
spi_sck<=1;
spi_csn<=0;
state<=state+1;
reg_count<=8;
end
end

119:begin
spi_sck<=0;
spi_csn<=0;
state<=state+1;
end




120:begin
if(payload_count<4)  /////////////// reading 3 byte payload 1 after another
begin///////////////////////
spi_sck<=0;
spi_csn<=0;
state<=109;  // it REPEATES  R_RX_PAYLOAD COMMAND to read RX_FIFO 3 TIMES
end
else
state<=state+1;
end



///////////////OUT OF POWER DOWN MODE/////////////////////////////

121:begin
if(count_standby==1024)  //2 Ms between POWER DOWN and STANDBY MODE controlled by PWR_UP bit
begin
count_standby<=0;
state<=state+1;
end
else
count_standby<=count_standby+1;
spi_sck<=0;
end


122:begin
spi_sck<=0;
spi_ce<=1;
state<=state+1;
end

123:begin
//spi_ce<=1;
if(ce_count==100)  //min CE High=10 us 
begin
ce_count<=0;
//state<=state+1;/////////////////////////////////
state<=134;
end
else
ce_count<=ce_count+1;
spi_sck<=0;
end





134:begin
if(countm==10)  //DELAY is providded before reading out NRF returned data
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end


135:begin
spi_sck<=0;
spi_csn<=0;
state<=state+1;
end


136:begin
spi_sck<=1;
if(status_clk>0)
begin
nrf_out[status_count-1]<=spi_miso;  // NRF returns data stored in RX_FIFO
status_count<=status_count-1;
status_clk<=status_clk-1;
state<=135;
end
else
begin
spi_sck<=1;
state<=state+1;
spi_csn<=0;
end
end

137:begin
if(countm==500)  // DELAY for next operation
begin
countm<=0;
state<=state+1;
end
else
countm<=countm+1;
spi_sck<=0;
end


138:begin
spi_sck<=0;
spi_csn<=1;
state<=state+1;
end

139:begin
spi_sck<=0;
spi_csn<=1;
//state<=109; // to read data continiously from NRF
state<=92;  
end


endcase
end
end
endmodule

