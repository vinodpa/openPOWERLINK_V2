------------------------------------------------------------------------------
--Copyright (c) 2014, Kalycito Infotech Pvt Ltd
--All rights reserved.
--
--Redistribution and use in source and binary forms, with or without
--modification, are permitted provided that the following conditions are met:
--    * Redistributions of source code must retain the above copyright
--      notice, this list of conditions and the following disclaimer.
--    * Redistributions in binary form must reproduce the above copyright
--      notice, this list of conditions and the following disclaimer in the
--      documentation and/or other materials provided with the distribution.
--    * Neither the name of the copyright holders nor the
--      names of its contributors may be used to endorse or promote products
--      derived from this software without specific prior written permission.
--
--THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
--ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
--DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
--ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
--(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
--SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;

entity toplevel is
    port (
    -- FPGA peripherals ports
	fpga_dipsw_pio   : in    std_logic_vector (3 downto 0);                   
	fpga_led_pio     : out   std_logic_vector (3 downto 0);                     
	fpga_button_pio  : in    std_logic_vector (1 downto 0);  
    -- HPS memory controller ports
	hps_memory_mem_a     : out   std_logic_vector (14 downto 0);
	hps_memory_mem_ba    : out   std_logic_vector (2 downto 0);                         
	hps_memory_mem_ck    : out   std_logic;                          
	hps_memory_mem_ck_n  : out   std_logic;
	hps_memory_mem_cke   : out   std_logic;     
	hps_memory_mem_cs_n  : out   std_logic;
	hps_memory_mem_ras_n : out   std_logic;     
	hps_memory_mem_cas_n : out   std_logic;     
	hps_memory_mem_we_n  : out   std_logic;     
	hps_memory_mem_reset_n : out std_logic;     
	hps_memory_mem_dq    : inout std_logic_vector (39 downto 0);                          
	hps_memory_mem_dqs   : inout std_logic_vector (4 downto 0);
	hps_memory_mem_dqs_n : inout std_logic_vector (4 downto 0);
	hps_memory_mem_odt   : out   std_logic;
	hps_memory_mem_dm    : out   std_logic_vector (4 downto 0);
	hps_memory_oct_rzqin : in std_logic;
    -- HPS peripherals
	hps_emac1_TX_CLK     : out std_logic;   
	hps_emac1_TXD0       : out std_logic; 
	hps_emac1_TXD1       : out std_logic;
	hps_emac1_TXD2       : out std_logic;     
	hps_emac1_TXD3       : out std_logic;
	hps_emac1_RXD0       : in std_logic;     
	hps_emac1_MDIO       : inout std_logic;
	hps_emac1_MDC        : out std_logic;      
	hps_emac1_RX_CTL     : in std_logic;
	hps_emac1_TX_CTL     : out std_logic;   
	hps_emac1_RX_CLK     : in std_logic;
	hps_emac1_RXD1       : in std_logic;
	hps_emac1_RXD2       : in std_logic;
	hps_emac1_RXD3       : in std_logic;
	hps_qspi_IO0         : inout std_logic;
	hps_qspi_IO1         : inout std_logic;
	hps_qspi_IO2         : inout std_logic;
	hps_qspi_IO3         : inout std_logic;
	hps_qspi_SS0         : out std_logic;       
	hps_qspi_CLK         : out std_logic;       
	hps_sdio_CMD         : inout std_logic;
	hps_sdio_D0          : inout std_logic;
	hps_sdio_D1          : inout std_logic;
	hps_sdio_CLK         : out std_logic;       
	hps_sdio_D2          : inout std_logic;
	hps_sdio_D3          : inout std_logic;
	hps_usb1_D0          : inout std_logic;
	hps_usb1_D1          : inout std_logic;
	hps_usb1_D2          : inout std_logic;
	hps_usb1_D3          : inout std_logic;
	hps_usb1_D4          : inout std_logic;
	hps_usb1_D5          : inout std_logic;
	hps_usb1_D6          : inout std_logic;
	hps_usb1_D7          : inout std_logic;
	hps_usb1_CLK         : in std_logic;
	hps_usb1_STP         : out std_logic;       
	hps_usb1_DIR         : in std_logic;
	hps_usb1_NXT         : in std_logic;
	hps_spim0_CLK        : out std_logic;      
	hps_spim0_MOSI       : out std_logic;     
	hps_spim0_MISO       : in std_logic;
	hps_spim0_SS0        : out std_logic;      
	hps_uart0_RX         : in std_logic;
	hps_uart0_TX         : out std_logic;       
	hps_i2c0_SDA         : inout std_logic;
	hps_i2c0_SCL         : inout std_logic;
	hps_can0_RX          : in std_logic;
	hps_can0_TX          : out std_logic;        
	hps_trace_CLK        : out std_logic;
	hps_trace_D0         : out std_logic;       
	hps_trace_D1         : out std_logic;
	hps_trace_D2         : out std_logic;
	hps_trace_D3         : out std_logic;
	hps_trace_D4         : out std_logic;
	hps_trace_D5         : out std_logic;
	hps_trace_D6         : out std_logic;
	hps_trace_D7         : out std_logic;
	hps_gpio_GPIO09      : inout std_logic;    
	hps_gpio_GPIO35      : inout std_logic;
	hps_gpio_GPIO41      : inout std_logic;
	hps_gpio_GPIO42      : inout std_logic;
	hps_gpio_GPIO43      : inout std_logic;
	hps_gpio_GPIO44      : inout std_logic;
   -- FPGA SDRAM
  fpga_memory_mem_a    : out std_logic_vector (12 downto 0);
  fpga_memory_mem_ba   : out std_logic_vector (2 downto 0);
  fpga_memory_mem_ck   : out std_logic_vector (0 downto 0);
  fpga_memory_mem_ck_n : out std_logic_vector (0 downto 0);
  fpga_memory_mem_cke  : out std_logic_vector (0 downto 0);
  fpga_memory_mem_cs_n : out std_logic_vector (0 downto 0);
  fpga_memory_mem_dm   : out std_logic_vector (3 downto 0);
  -- output wire [1:0]  fpga_memory_mem_dm,   
  fpga_memory_mem_ras_n : out std_logic_vector (0 downto 0);
  fpga_memory_mem_cas_n : out std_logic_vector (0 downto 0);
  fpga_memory_mem_we_n  : out std_logic_vector (0 downto 0);
  fpga_memory_mem_reset_n : out std_logic;
  fpga_memory_mem_dq       : inout std_logic_vector (31 downto 0);     
  fpga_memory_mem_dqs      : inout std_logic_vector (3 downto 0);
  fpga_memory_mem_dqs_n    : inout std_logic_vector (3 downto 0);    
  fpga_memory_mem_odt      : out std_logic_vector (0 downto 0);
  fpga_oct_rzqin       : in std_logic;
  -- FPGA clock and reset
  fpga_clk_50         : in std_logic;
  PLNK_MII_TXEN       : out   std_logic_vector(1 downto 0);                     -- txEnable
  PLNK_MII_TXD       : out   std_logic_vector(7 downto 0);                     -- txData
  PLNK_MII_TXCLK        : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- txClk
  PLNK_MII_RXERR      : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxError
  PLNK_MII_RXDV  : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxDataValid
  PLNK_MII_RXD       : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- rxData
  PLNK_MII_RXCLK        : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxClk
  PLNK_SMI_PHYRSTN      : out   std_logic_vector(0 downto 0);                     -- nPhyRst
  PLNK_SMI_CLK          : out   std_logic_vector(0 downto 0);                     -- clk
  PLNK_SMI_DIO          : inout std_logic_vector(0 downto 0)  := (others => 'X'); -- dio
  PLNK_MAC_TIMER : out   std_logic_vector(0 downto 0);                      -- export
  
  --TEST PORTS
  test_pushbutton	     : out  std_logic;
  test_pllLocked	     : out  std_logic;
  test_hps_fpga_reset_n_src	     : out  std_logic;
  test_ddr3_afi_resetn	     : out  std_logic;
  test_hps_fpga_reset_n	     : out  std_logic;
  test_pulse_resetn_ddr	     : out  std_logic
    
  
);
end toplevel;

architecture rtl of toplevel is
-- internal wires and registers declaration
  signal fpga_debounced_buttons : std_logic_vector (1 downto 0);
  signal fpga_led_internal      : std_logic_vector (3 downto 0);
  signal hps_fpga_reset_n       : std_logic;
  signal hps_reset_req          : std_logic_vector (2 downto 0);
  signal hps_cold_reset         : std_logic;
  signal hps_warm_reset         : std_logic;
  signal hps_debug_reset        : std_logic;
  signal pulse_resetn_ddr       : std_logic;
  signal hps_fpga_reset_n_src   : std_logic;
  signal ddr3_afi_resetn        : std_logic;
  
  signal clk50						  : std_logic;
  signal clk25						  : std_logic;
  signal clk100					  : std_logic;
  signal clk100_p  				  : std_logic;
  signal pllLocked				  : std_logic;
  
  signal clock_in					  : std_logic;
  
  
  component mnDualHostifGpio is
        port (
            memory_mem_a                                      : out   std_logic_vector(14 downto 0);                    -- mem_a
            memory_mem_ba                                     : out   std_logic_vector(2 downto 0);                     -- mem_ba
            memory_mem_ck                                     : out   std_logic;                                        -- mem_ck
            memory_mem_ck_n                                   : out   std_logic;                                        -- mem_ck_n
            memory_mem_cke                                    : out   std_logic;                                        -- mem_cke
            memory_mem_cs_n                                   : out   std_logic;                                        -- mem_cs_n
            memory_mem_ras_n                                  : out   std_logic;                                        -- mem_ras_n
            memory_mem_cas_n                                  : out   std_logic;                                        -- mem_cas_n
            memory_mem_we_n                                   : out   std_logic;                                        -- mem_we_n
            memory_mem_reset_n                                : out   std_logic;                                        -- mem_reset_n
            memory_mem_dq                                     : inout std_logic_vector(39 downto 0) := (others => 'X'); -- mem_dq
            memory_mem_dqs                                    : inout std_logic_vector(4 downto 0)  := (others => 'X'); -- mem_dqs
            memory_mem_dqs_n                                  : inout std_logic_vector(4 downto 0)  := (others => 'X'); -- mem_dqs_n
            memory_mem_odt                                    : out   std_logic;                                        -- mem_odt
            memory_mem_dm                                     : out   std_logic_vector(4 downto 0);                     -- mem_dm
            memory_oct_rzqin                                  : in    std_logic                     := 'X';             -- oct_rzqin
            memory_0_mem_a                                    : out   std_logic_vector(12 downto 0);                    -- mem_a
            memory_0_mem_ba                                   : out   std_logic_vector(2 downto 0);                     -- mem_ba
            memory_0_mem_ck                                   : out   std_logic_vector(0 downto 0);                     -- mem_ck
            memory_0_mem_ck_n                                 : out   std_logic_vector(0 downto 0);                     -- mem_ck_n
            memory_0_mem_cke                                  : out   std_logic_vector(0 downto 0);                     -- mem_cke
            memory_0_mem_cs_n                                 : out   std_logic_vector(0 downto 0);                     -- mem_cs_n
            memory_0_mem_dm                                   : out   std_logic_vector(3 downto 0);                     -- mem_dm
            memory_0_mem_ras_n                                : out   std_logic_vector(0 downto 0);                     -- mem_ras_n
            memory_0_mem_cas_n                                : out   std_logic_vector(0 downto 0);                     -- mem_cas_n
            memory_0_mem_we_n                                 : out   std_logic_vector(0 downto 0);                     -- mem_we_n
            memory_0_mem_reset_n                              : out   std_logic;                                        -- mem_reset_n
            memory_0_mem_dq                                   : inout std_logic_vector(31 downto 0) := (others => 'X'); -- mem_dq
            memory_0_mem_dqs                                  : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs
            memory_0_mem_dqs_n                                : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs_n
            memory_0_mem_odt                                  : out   std_logic_vector(0 downto 0);                     -- mem_odt
            oct_rzqin                                         : in    std_logic                     := 'X';             -- rzqin
            clk_50_clk                                        : in    std_logic                     := 'X';             -- clk
	    clk_100_clk                                       : in    std_logic                     := 'X';             -- clk
            reset_reset_n                                     : in    std_logic                     := 'X';             -- reset_n
            hps_io_hps_io_emac1_inst_TX_CLK             : out   std_logic;                                        -- hps_io_emac1_inst_TX_CLK
            hps_io_hps_io_emac1_inst_TXD0               : out   std_logic;                                        -- hps_io_emac1_inst_TXD0
            hps_io_hps_io_emac1_inst_TXD1               : out   std_logic;                                        -- hps_io_emac1_inst_TXD1
            hps_io_hps_io_emac1_inst_TXD2               : out   std_logic;                                        -- hps_io_emac1_inst_TXD2
            hps_io_hps_io_emac1_inst_TXD3               : out   std_logic;                                        -- hps_io_emac1_inst_TXD3
            hps_io_hps_io_emac1_inst_RXD0               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD0
            hps_io_hps_io_emac1_inst_MDIO               : inout std_logic                     := 'X';             -- hps_io_emac1_inst_MDIO
            hps_io_hps_io_emac1_inst_MDC                : out   std_logic;                                        -- hps_io_emac1_inst_MDC
            hps_io_hps_io_emac1_inst_RX_CTL             : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CTL
            hps_io_hps_io_emac1_inst_TX_CTL             : out   std_logic;                                        -- hps_io_emac1_inst_TX_CTL
            hps_io_hps_io_emac1_inst_RX_CLK             : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CLK
            hps_io_hps_io_emac1_inst_RXD1               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD1
            hps_io_hps_io_emac1_inst_RXD2               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD2
            hps_io_hps_io_emac1_inst_RXD3               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD3
            hps_io_hps_io_qspi_inst_IO0                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO0
            hps_io_hps_io_qspi_inst_IO1                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO1
            hps_io_hps_io_qspi_inst_IO2                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO2
            hps_io_hps_io_qspi_inst_IO3                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO3
            hps_io_hps_io_qspi_inst_SS0                 : out   std_logic;                                        -- hps_io_qspi_inst_SS0
            hps_io_hps_io_qspi_inst_CLK                 : out   std_logic;                                        -- hps_io_qspi_inst_CLK
            hps_io_hps_io_sdio_inst_CMD                 : inout std_logic                     := 'X';             -- hps_io_sdio_inst_CMD
            hps_io_hps_io_sdio_inst_D0                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D0
            hps_io_hps_io_sdio_inst_D1                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D1
            hps_io_hps_io_sdio_inst_CLK                 : out   std_logic;                                        -- hps_io_sdio_inst_CLK
            hps_io_hps_io_sdio_inst_D2                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D2
            hps_io_hps_io_sdio_inst_D3                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D3
            hps_io_hps_io_usb1_inst_D0                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D0
            hps_io_hps_io_usb1_inst_D1                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D1
            hps_io_hps_io_usb1_inst_D2                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D2
            hps_io_hps_io_usb1_inst_D3                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D3
            hps_io_hps_io_usb1_inst_D4                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D4
            hps_io_hps_io_usb1_inst_D5                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D5
            hps_io_hps_io_usb1_inst_D6                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D6
            hps_io_hps_io_usb1_inst_D7                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D7
            hps_io_hps_io_usb1_inst_CLK                 : in    std_logic                     := 'X';             -- hps_io_usb1_inst_CLK
            hps_io_hps_io_usb1_inst_STP                 : out   std_logic;                                        -- hps_io_usb1_inst_STP
            hps_io_hps_io_usb1_inst_DIR                 : in    std_logic                     := 'X';             -- hps_io_usb1_inst_DIR
            hps_io_hps_io_usb1_inst_NXT                 : in    std_logic                     := 'X';             -- hps_io_usb1_inst_NXT
            hps_io_hps_io_spim0_inst_CLK                : out   std_logic;                                        -- hps_io_spim0_inst_CLK
            hps_io_hps_io_spim0_inst_MOSI               : out   std_logic;                                        -- hps_io_spim0_inst_MOSI
            hps_io_hps_io_spim0_inst_MISO               : in    std_logic                     := 'X';             -- hps_io_spim0_inst_MISO
            hps_io_hps_io_spim0_inst_SS0                : out   std_logic;                                        -- hps_io_spim0_inst_SS0
            hps_io_hps_io_uart0_inst_RX                 : in    std_logic                     := 'X';             -- hps_io_uart0_inst_RX
            hps_io_hps_io_uart0_inst_TX                 : out   std_logic;                                        -- hps_io_uart0_inst_TX
            hps_io_hps_io_i2c0_inst_SDA                 : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SDA
            hps_io_hps_io_i2c0_inst_SCL                 : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SCL
            hps_io_hps_io_can0_inst_RX                  : in    std_logic                     := 'X';             -- hps_io_can0_inst_RX
            hps_io_hps_io_can0_inst_TX                  : out   std_logic;                                        -- hps_io_can0_inst_TX
            hps_io_hps_io_trace_inst_CLK                : out   std_logic;                                        -- hps_io_trace_inst_CLK
            hps_io_hps_io_trace_inst_D0                 : out   std_logic;                                        -- hps_io_trace_inst_D0
            hps_io_hps_io_trace_inst_D1                 : out   std_logic;                                        -- hps_io_trace_inst_D1
            hps_io_hps_io_trace_inst_D2                 : out   std_logic;                                        -- hps_io_trace_inst_D2
            hps_io_hps_io_trace_inst_D3                 : out   std_logic;                                        -- hps_io_trace_inst_D3
            hps_io_hps_io_trace_inst_D4                 : out   std_logic;                                        -- hps_io_trace_inst_D4
            hps_io_hps_io_trace_inst_D5                 : out   std_logic;                                        -- hps_io_trace_inst_D5
            hps_io_hps_io_trace_inst_D6                 : out   std_logic;                                        -- hps_io_trace_inst_D6
            hps_io_hps_io_trace_inst_D7                 : out   std_logic;                                        -- hps_io_trace_inst_D7
            hps_io_hps_io_gpio_inst_GPIO09              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO09
            hps_io_hps_io_gpio_inst_GPIO35              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO35
            hps_io_hps_io_gpio_inst_GPIO41              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO41
            hps_io_hps_io_gpio_inst_GPIO42              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO42
            hps_io_hps_io_gpio_inst_GPIO43              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO43
            hps_io_hps_io_gpio_inst_GPIO44              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO44
            led_pio_external_connection_in_port               : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- in_port
            led_pio_external_connection_out_port              : out   std_logic_vector(3 downto 0);                     -- out_port
            dipsw_pio_external_connection_export              : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- export
            button_pio_external_connection_export             : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- export
            host_0_hps_0_h2f_reset_reset_n                    : out   std_logic;                                        -- reset_n
            ddr3_emif_0_status_local_init_done                : out   std_logic;                                        -- local_init_done
            ddr3_emif_0_status_local_cal_success              : out   std_logic;                                        -- local_cal_success
            ddr3_emif_0_status_local_cal_fail                 : out   std_logic;                                        -- local_cal_fail
            ddr3_emif_0_pll_sharing_pll_mem_clk               : out   std_logic;                                        -- pll_mem_clk
            ddr3_emif_0_pll_sharing_pll_write_clk             : out   std_logic;                                        -- pll_write_clk
            ddr3_emif_0_pll_sharing_pll_locked                : out   std_logic;                                        -- pll_locked
            ddr3_emif_0_pll_sharing_pll_write_clk_pre_phy_clk : out   std_logic;                                        -- pll_write_clk_pre_phy_clk
            ddr3_emif_0_pll_sharing_pll_addr_cmd_clk          : out   std_logic;                                        -- pll_addr_cmd_clk
            ddr3_emif_0_pll_sharing_pll_avl_clk               : out   std_logic;                                        -- pll_avl_clk
            ddr3_emif_0_pll_sharing_pll_config_clk            : out   std_logic;                                        -- pll_config_clk
            ddr3_emif_0_pll_sharing_pll_dr_clk                : out   std_logic;                                        -- pll_dr_clk
            ddr3_emif_0_pll_sharing_pll_dr_clk_pre_phy_clk    : out   std_logic;                                        -- pll_dr_clk_pre_phy_clk
            ddr3_emif_0_pll_sharing_pll_mem_phy_clk           : out   std_logic;                                        -- pll_mem_phy_clk
            ddr3_emif_0_pll_sharing_afi_phy_clk               : out   std_logic;                                        -- afi_phy_clk
            ddr3_emif_0_pll_sharing_pll_avl_phy_clk           : out   std_logic;                                        -- pll_avl_phy_clk
            ddr3_emif_0_global_reset_reset_n                  : in    std_logic                     := 'X';             -- reset_n
            ddr3_emif_0_afi_reset_export_reset_n              : out   std_logic;                                        -- reset_n
				ddr3_emif_0_pll_ref_clk_clk                       : in    std_logic                     := 'X';             -- clk
            ddr3_emif_0_soft_reset_reset_n                    : in    std_logic                     := 'X'             -- reset_n
--				  openmac_0_mii_txEnable                            : out   std_logic_vector(1 downto 0);                     -- txEnable
--            openmac_0_mii_txData                              : out   std_logic_vector(7 downto 0);                     -- txData
--            openmac_0_mii_txClk                               : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- txClk
--            openmac_0_mii_rxError                             : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxError
--            openmac_0_mii_rxDataValid                         : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxDataValid
--            openmac_0_mii_rxData                              : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- rxData
--            openmac_0_mii_rxClk                               : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxClk
--            openmac_0_smi_nPhyRst                             : out   std_logic_vector(0 downto 0);                     -- nPhyRst
--            openmac_0_smi_clk                                 : out   std_logic_vector(0 downto 0);                     -- clk
--            openmac_0_smi_dio                                 : inout std_logic_vector(0 downto 0)  := (others => 'X'); -- dio
--            openmac_0_mactimerout_export                      : out   std_logic_vector(0 downto 0)                      -- export
        );
    end component mnDualHostifGpio;
	 
	 component debounce 
	 generic (
	  WIDTH		: integer := 32 ;
	  POLARITY	: string := "HIGH" ;
	  TIMEOUT   : integer := 50000 ;
	  TIMEOUT_WIDTH : integer := 16
	 );
	 port (
		clk		: in std_logic ;
		reset_n  : in std_logic ;
		data_in  : in std_logic_vector (WIDTH-1 downto 0) ;
		data_out : out std_logic_vector (WIDTH-1 downto 0)
	 );
	 end component debounce; 

	component altera_edge_detector 
	generic (
		PULSE_EXT : integer := 0;
		EDGE_TYPE : integer := 0;
		IGNORE_RST_WHILE_BUSY : integer := 0
	);
   port(
	clk   : in std_logic;
	rst_n : in std_logic ;
	signal_in : in std_logic;
	pulse_out : out std_logic
	);
	end component altera_edge_detector;
	
	component altera_reset_controller 
	generic(
		RESET_SOURCE_COUNT : integer := 1;
		RESET_SYNC_LENGTH  : integer := 8;
		GENERATE_PULSE_OUT : integer := 1;
      PULSE_LENGTH       : integer := 1
	);
	port(
		clk : in std_logic ;
      reset_n_src : in std_logic_vector (RESET_SOURCE_COUNT-1 downto 0);
		combined_reset_n : out std_logic;
      pulse_reset_n : out std_logic 
    );
	end component altera_reset_controller ;
 
  component hps_reset 
  port(
	--probe		:	in std_logic;
	source_clk : in std_logic;
	source : out std_logic_vector (2 downto 0)
	);
	end component hps_reset ;
	
	component pll
	PORT
	(
		inclk0		: IN STD_LOGIC  := '0';
		c0		: OUT STD_LOGIC ;
		c1		: OUT STD_LOGIC ;
		c2		: OUT STD_LOGIC ;
		c3		: OUT STD_LOGIC ;
		locked		: OUT STD_LOGIC 
	);
   end component pll;

 begin
 
 
-- connection of internal logics
  fpga_led_pio <= fpga_led_internal; 
  clock_in <= fpga_clk_50 ;


soc_inst: component mnDualHostifGpio
   port map (
      --HPS External Memory
      memory_mem_a                          =>  hps_memory_mem_a,                               
      memory_mem_ba                         =>  hps_memory_mem_ba,
      memory_mem_ck                         =>  hps_memory_mem_ck,                         
      memory_mem_ck_n                       =>  hps_memory_mem_ck_n,                       
      memory_mem_cke                        =>  hps_memory_mem_cke,                        
      memory_mem_cs_n                       =>  hps_memory_mem_cs_n,                       
      memory_mem_ras_n                      =>  hps_memory_mem_ras_n,                      
      memory_mem_cas_n                      =>  hps_memory_mem_cas_n,                      
      memory_mem_we_n                       =>  hps_memory_mem_we_n,                       
      memory_mem_reset_n                    =>  hps_memory_mem_reset_n,                    
      memory_mem_dq                         =>  hps_memory_mem_dq,                         
      memory_mem_dqs                        =>  hps_memory_mem_dqs,                        
      memory_mem_dqs_n                      =>  hps_memory_mem_dqs_n,                      
      memory_mem_odt                        =>  hps_memory_mem_odt,                        
      memory_mem_dm                         =>  hps_memory_mem_dm,                         
      memory_oct_rzqin                      =>  hps_memory_oct_rzqin, 
      --DIP Switch FPGA       
      dipsw_pio_external_connection_export  =>  fpga_dipsw_pio,    
      led_pio_external_connection_in_port   =>  fpga_led_internal,
      led_pio_external_connection_out_port  =>  fpga_led_internal,                   
      button_pio_external_connection_export =>  "00",                  
      hps_io_hps_io_emac1_inst_TX_CLK =>  hps_emac1_TX_CLK, 
      hps_io_hps_io_emac1_inst_TXD0   =>  hps_emac1_TXD0,   
      hps_io_hps_io_emac1_inst_TXD1   =>  hps_emac1_TXD1,   
      hps_io_hps_io_emac1_inst_TXD2   =>  hps_emac1_TXD2,   
      hps_io_hps_io_emac1_inst_TXD3   =>  hps_emac1_TXD3,   
      hps_io_hps_io_emac1_inst_RXD0   =>  hps_emac1_RXD0,   
      hps_io_hps_io_emac1_inst_MDIO   =>  hps_emac1_MDIO,   
      hps_io_hps_io_emac1_inst_MDC    =>  hps_emac1_MDC,    
      hps_io_hps_io_emac1_inst_RX_CTL =>  hps_emac1_RX_CTL, 
      hps_io_hps_io_emac1_inst_TX_CTL =>  hps_emac1_TX_CTL, 
      hps_io_hps_io_emac1_inst_RX_CLK =>  hps_emac1_RX_CLK, 
      hps_io_hps_io_emac1_inst_RXD1   =>  hps_emac1_RXD1,   
      hps_io_hps_io_emac1_inst_RXD2   =>  hps_emac1_RXD2,   
      hps_io_hps_io_emac1_inst_RXD3   =>  hps_emac1_RXD3,   
      hps_io_hps_io_qspi_inst_IO0     =>  hps_qspi_IO0,     
      hps_io_hps_io_qspi_inst_IO1     =>  hps_qspi_IO1,     
      hps_io_hps_io_qspi_inst_IO2     =>  hps_qspi_IO2,     
      hps_io_hps_io_qspi_inst_IO3     =>  hps_qspi_IO3,     
      hps_io_hps_io_qspi_inst_SS0     =>  hps_qspi_SS0,     
      hps_io_hps_io_qspi_inst_CLK     =>  hps_qspi_CLK,     
      hps_io_hps_io_sdio_inst_CMD     =>  hps_sdio_CMD,     
      hps_io_hps_io_sdio_inst_D0      =>  hps_sdio_D0,      
      hps_io_hps_io_sdio_inst_D1      =>  hps_sdio_D1,      
      hps_io_hps_io_sdio_inst_CLK     =>  hps_sdio_CLK,     
      hps_io_hps_io_sdio_inst_D2      =>  hps_sdio_D2,      
      hps_io_hps_io_sdio_inst_D3      =>  hps_sdio_D3,      
      hps_io_hps_io_usb1_inst_D0      =>  hps_usb1_D0,      
      hps_io_hps_io_usb1_inst_D1      =>  hps_usb1_D1,      
      hps_io_hps_io_usb1_inst_D2      =>  hps_usb1_D2,      
      hps_io_hps_io_usb1_inst_D3      =>  hps_usb1_D3,      
      hps_io_hps_io_usb1_inst_D4      =>  hps_usb1_D4,      
      hps_io_hps_io_usb1_inst_D5      =>  hps_usb1_D5,      
      hps_io_hps_io_usb1_inst_D6      =>  hps_usb1_D6,      
      hps_io_hps_io_usb1_inst_D7      =>  hps_usb1_D7,      
      hps_io_hps_io_usb1_inst_CLK     =>  hps_usb1_CLK,     
      hps_io_hps_io_usb1_inst_STP     =>  hps_usb1_STP,     
      hps_io_hps_io_usb1_inst_DIR     =>  hps_usb1_DIR,     
      hps_io_hps_io_usb1_inst_NXT     =>  hps_usb1_NXT,     
      hps_io_hps_io_spim0_inst_CLK    =>  hps_spim0_CLK,    
      hps_io_hps_io_spim0_inst_MOSI   =>  hps_spim0_MOSI,   
      hps_io_hps_io_spim0_inst_MISO   =>  hps_spim0_MISO,   
      hps_io_hps_io_spim0_inst_SS0    =>  hps_spim0_SS0,    
      hps_io_hps_io_uart0_inst_RX     =>  hps_uart0_RX,     
      hps_io_hps_io_uart0_inst_TX     =>  hps_uart0_TX,     
      hps_io_hps_io_i2c0_inst_SDA     =>  hps_i2c0_SDA,     
      hps_io_hps_io_i2c0_inst_SCL     =>  hps_i2c0_SCL,     
      hps_io_hps_io_can0_inst_RX      =>  hps_can0_RX,      
      hps_io_hps_io_can0_inst_TX      =>  hps_can0_TX,      
      hps_io_hps_io_trace_inst_CLK    =>  hps_trace_CLK,    
      hps_io_hps_io_trace_inst_D0     =>  hps_trace_D0,     
      hps_io_hps_io_trace_inst_D1     =>  hps_trace_D1,     
      hps_io_hps_io_trace_inst_D2     =>  hps_trace_D2,     
      hps_io_hps_io_trace_inst_D3     =>  hps_trace_D3,     
      hps_io_hps_io_trace_inst_D4     =>  hps_trace_D4,     
      hps_io_hps_io_trace_inst_D5     =>  hps_trace_D5,     
      hps_io_hps_io_trace_inst_D6     =>  hps_trace_D6,     
      hps_io_hps_io_trace_inst_D7     =>  hps_trace_D7,     
      hps_io_hps_io_gpio_inst_GPIO09  =>  hps_gpio_GPIO09,  
      hps_io_hps_io_gpio_inst_GPIO35  =>  hps_gpio_GPIO35,  
      hps_io_hps_io_gpio_inst_GPIO41  =>  hps_gpio_GPIO41,  
      hps_io_hps_io_gpio_inst_GPIO42  =>  hps_gpio_GPIO42,  
      hps_io_hps_io_gpio_inst_GPIO43  =>  hps_gpio_GPIO43,  
      hps_io_hps_io_gpio_inst_GPIO44  =>  hps_gpio_GPIO44,  
      clk_50_clk                            =>  clk50,
      clk_100_clk                           =>  clk100,
      host_0_hps_0_h2f_reset_reset_n        =>  hps_fpga_reset_n_src,
      reset_reset_n                         =>  hps_fpga_reset_n,
      memory_0_mem_a                        =>  fpga_memory_mem_a,                                   
      memory_0_mem_ba                       =>  fpga_memory_mem_ba,                                    
      memory_0_mem_ck                       =>  fpga_memory_mem_ck,                                    
      memory_0_mem_ck_n                     =>  fpga_memory_mem_ck_n,                                  
      memory_0_mem_cke                      =>  fpga_memory_mem_cke,                                   
      memory_0_mem_cs_n                     =>  fpga_memory_mem_cs_n,                                  
      memory_0_mem_dm                       =>  fpga_memory_mem_dm,                                    
      memory_0_mem_ras_n                    =>  fpga_memory_mem_ras_n,                                 
      memory_0_mem_cas_n                    =>  fpga_memory_mem_cas_n,                                 
      memory_0_mem_we_n                     =>  fpga_memory_mem_we_n,                                  
      memory_0_mem_reset_n                  =>  fpga_memory_mem_reset_n,                               
      memory_0_mem_dq                       =>  fpga_memory_mem_dq,                                    
      memory_0_mem_dqs                      =>  fpga_memory_mem_dqs,                                   
      memory_0_mem_dqs_n                    =>  fpga_memory_mem_dqs_n,                                 
      memory_0_mem_odt                      =>  fpga_memory_mem_odt, 
      ddr3_emif_0_global_reset_reset_n      =>  pulse_resetn_ddr,  
      ddr3_emif_0_soft_reset_reset_n        =>  pulse_resetn_ddr, 
      ddr3_emif_0_afi_reset_export_reset_n  =>  ddr3_afi_resetn,                       
      ddr3_emif_0_pll_ref_clk_clk           =>  fpga_clk_50,
      oct_rzqin                             =>  fpga_oct_rzqin
--		  openmac_0_mii_txEnable                =>  PLNK_MII_TXEN,                       --                  openmac_0_mii.txEnable
--      openmac_0_mii_txData                  =>  PLNK_MII_TXD,                        --                               .txData
--      openmac_0_mii_txClk                   =>  PLNK_MII_TXCLK,                      --                               .txClk
--      openmac_0_mii_rxError                 =>  PLNK_MII_RXERR,                      --                               .rxError
--      openmac_0_mii_rxDataValid             =>  PLNK_MII_RXDV,                       --                               .rxDataValid
--      openmac_0_mii_rxData                  =>  PLNK_MII_RXD,                        --                               .rxData
--      openmac_0_mii_rxClk                   =>  PLNK_MII_RXCLK,                      --                               .rxClk
--      openmac_0_smi_nPhyRst                 =>  PLNK_SMI_PHYRSTN,                    --                  openmac_0_smi.nPhyRst
--      openmac_0_smi_clk                     =>  PLNK_SMI_CLK,                        --                               .clk
--      openmac_0_smi_dio                     =>  PLNK_SMI_DIO,                        --                               .dio
--      openmac_0_mactimerout_export          =>  PLNK_MAC_TIMER                       --          openmac_0_mactimerout.export
    );  
	 
	 
debounce_inst: component debounce
generic map (
    WIDTH         => 2,
    POLARITY      => "LOW",
    TIMEOUT       => 50000,     -- at 50Mhz this is a debounce time of 1ms
    TIMEOUT_WIDTH => 16  -- ceil(log2(TIMEOUT))
 )
port map (
    clk         => clk50,
    reset_n     => hps_fpga_reset_n,  
    data_in     => fpga_button_pio,
    data_out    => fpga_debounced_buttons
);
rst_ctrl_inst: component altera_reset_controller 
generic map (
  RESET_SOURCE_COUNT => 3,
  RESET_SYNC_LENGTH  => 10,
  GENERATE_PULSE_OUT => 1,
  PULSE_LENGTH       => 5
)
port map(
  clk              => clk50,
  reset_n_src      => pllLocked & fpga_debounced_buttons(0) & ddr3_afi_resetn,
  --reset_n_src      => pllLocked & hps_fpga_reset_n_src & ddr3_afi_resetn,
  combined_reset_n => hps_fpga_reset_n,
  pulse_reset_n    => pulse_resetn_ddr 
 );

  test_pushbutton	          <= fpga_debounced_buttons(0); 
  test_pllLocked	          <= pllLocked;
  test_hps_fpga_reset_n_src <= hps_fpga_reset_n_src;
  test_ddr3_afi_resetn	    <= ddr3_afi_resetn;
  test_hps_fpga_reset_n	    <= hps_fpga_reset_n;
  test_pulse_resetn_ddr	    <= pulse_resetn_ddr;

pllInst : pll 
port map
	(
		inclk0 => fpga_clk_50,
		c0		 => clk100,
		c1		 => clk50,
		c2		 => clk25,
		c3		 => clk100_p,
		locked => pllLocked		
	);
	
end rtl;
