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
			reset_reset_n                                     : in    std_logic                     := 'X';             -- reset_n
			hps_0_hps_io_hps_io_emac1_inst_TX_CLK             : out   std_logic;                                        -- hps_io_emac1_inst_TX_CLK
			hps_0_hps_io_hps_io_emac1_inst_TXD0               : out   std_logic;                                        -- hps_io_emac1_inst_TXD0
			hps_0_hps_io_hps_io_emac1_inst_TXD1               : out   std_logic;                                        -- hps_io_emac1_inst_TXD1
			hps_0_hps_io_hps_io_emac1_inst_TXD2               : out   std_logic;                                        -- hps_io_emac1_inst_TXD2
			hps_0_hps_io_hps_io_emac1_inst_TXD3               : out   std_logic;                                        -- hps_io_emac1_inst_TXD3
			hps_0_hps_io_hps_io_emac1_inst_RXD0               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD0
			hps_0_hps_io_hps_io_emac1_inst_MDIO               : inout std_logic                     := 'X';             -- hps_io_emac1_inst_MDIO
			hps_0_hps_io_hps_io_emac1_inst_MDC                : out   std_logic;                                        -- hps_io_emac1_inst_MDC
			hps_0_hps_io_hps_io_emac1_inst_RX_CTL             : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CTL
			hps_0_hps_io_hps_io_emac1_inst_TX_CTL             : out   std_logic;                                        -- hps_io_emac1_inst_TX_CTL
			hps_0_hps_io_hps_io_emac1_inst_RX_CLK             : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CLK
			hps_0_hps_io_hps_io_emac1_inst_RXD1               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD1
			hps_0_hps_io_hps_io_emac1_inst_RXD2               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD2
			hps_0_hps_io_hps_io_emac1_inst_RXD3               : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD3
			hps_0_hps_io_hps_io_qspi_inst_IO0                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO0
			hps_0_hps_io_hps_io_qspi_inst_IO1                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO1
			hps_0_hps_io_hps_io_qspi_inst_IO2                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO2
			hps_0_hps_io_hps_io_qspi_inst_IO3                 : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO3
			hps_0_hps_io_hps_io_qspi_inst_SS0                 : out   std_logic;                                        -- hps_io_qspi_inst_SS0
			hps_0_hps_io_hps_io_qspi_inst_CLK                 : out   std_logic;                                        -- hps_io_qspi_inst_CLK
			hps_0_hps_io_hps_io_sdio_inst_CMD                 : inout std_logic                     := 'X';             -- hps_io_sdio_inst_CMD
			hps_0_hps_io_hps_io_sdio_inst_D0                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D0
			hps_0_hps_io_hps_io_sdio_inst_D1                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D1
			hps_0_hps_io_hps_io_sdio_inst_CLK                 : out   std_logic;                                        -- hps_io_sdio_inst_CLK
			hps_0_hps_io_hps_io_sdio_inst_D2                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D2
			hps_0_hps_io_hps_io_sdio_inst_D3                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D3
			hps_0_hps_io_hps_io_usb1_inst_D0                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D0
			hps_0_hps_io_hps_io_usb1_inst_D1                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D1
			hps_0_hps_io_hps_io_usb1_inst_D2                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D2
			hps_0_hps_io_hps_io_usb1_inst_D3                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D3
			hps_0_hps_io_hps_io_usb1_inst_D4                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D4
			hps_0_hps_io_hps_io_usb1_inst_D5                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D5
			hps_0_hps_io_hps_io_usb1_inst_D6                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D6
			hps_0_hps_io_hps_io_usb1_inst_D7                  : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D7
			hps_0_hps_io_hps_io_usb1_inst_CLK                 : in    std_logic                     := 'X';             -- hps_io_usb1_inst_CLK
			hps_0_hps_io_hps_io_usb1_inst_STP                 : out   std_logic;                                        -- hps_io_usb1_inst_STP
			hps_0_hps_io_hps_io_usb1_inst_DIR                 : in    std_logic                     := 'X';             -- hps_io_usb1_inst_DIR
			hps_0_hps_io_hps_io_usb1_inst_NXT                 : in    std_logic                     := 'X';             -- hps_io_usb1_inst_NXT
			hps_0_hps_io_hps_io_spim0_inst_CLK                : out   std_logic;                                        -- hps_io_spim0_inst_CLK
			hps_0_hps_io_hps_io_spim0_inst_MOSI               : out   std_logic;                                        -- hps_io_spim0_inst_MOSI
			hps_0_hps_io_hps_io_spim0_inst_MISO               : in    std_logic                     := 'X';             -- hps_io_spim0_inst_MISO
			hps_0_hps_io_hps_io_spim0_inst_SS0                : out   std_logic;                                        -- hps_io_spim0_inst_SS0
			hps_0_hps_io_hps_io_uart0_inst_RX                 : in    std_logic                     := 'X';             -- hps_io_uart0_inst_RX
			hps_0_hps_io_hps_io_uart0_inst_TX                 : out   std_logic;                                        -- hps_io_uart0_inst_TX
			hps_0_hps_io_hps_io_i2c0_inst_SDA                 : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SDA
			hps_0_hps_io_hps_io_i2c0_inst_SCL                 : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SCL
			hps_0_hps_io_hps_io_can0_inst_RX                  : in    std_logic                     := 'X';             -- hps_io_can0_inst_RX
			hps_0_hps_io_hps_io_can0_inst_TX                  : out   std_logic;                                        -- hps_io_can0_inst_TX
			hps_0_hps_io_hps_io_trace_inst_CLK                : out   std_logic;                                        -- hps_io_trace_inst_CLK
			hps_0_hps_io_hps_io_trace_inst_D0                 : out   std_logic;                                        -- hps_io_trace_inst_D0
			hps_0_hps_io_hps_io_trace_inst_D1                 : out   std_logic;                                        -- hps_io_trace_inst_D1
			hps_0_hps_io_hps_io_trace_inst_D2                 : out   std_logic;                                        -- hps_io_trace_inst_D2
			hps_0_hps_io_hps_io_trace_inst_D3                 : out   std_logic;                                        -- hps_io_trace_inst_D3
			hps_0_hps_io_hps_io_trace_inst_D4                 : out   std_logic;                                        -- hps_io_trace_inst_D4
			hps_0_hps_io_hps_io_trace_inst_D5                 : out   std_logic;                                        -- hps_io_trace_inst_D5
			hps_0_hps_io_hps_io_trace_inst_D6                 : out   std_logic;                                        -- hps_io_trace_inst_D6
			hps_0_hps_io_hps_io_trace_inst_D7                 : out   std_logic;                                        -- hps_io_trace_inst_D7
			hps_0_hps_io_hps_io_gpio_inst_GPIO09              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO09
			hps_0_hps_io_hps_io_gpio_inst_GPIO35              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO35
			hps_0_hps_io_hps_io_gpio_inst_GPIO41              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO41
			hps_0_hps_io_hps_io_gpio_inst_GPIO42              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO42
			hps_0_hps_io_hps_io_gpio_inst_GPIO43              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO43
			hps_0_hps_io_hps_io_gpio_inst_GPIO44              : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO44
			led_pio_external_connection_in_port               : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- in_port
			led_pio_external_connection_out_port              : out   std_logic_vector(3 downto 0);                     -- out_port
			dipsw_pio_external_connection_export              : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- export
			button_pio_external_connection_export             : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- export
			hps_0_h2f_reset_reset_n                           : out   std_logic;                                        -- reset_n
			hps_0_f2h_cold_reset_req_reset_n                  : in    std_logic                     := 'X';             -- reset_n
			hps_0_f2h_debug_reset_req_reset_n                 : in    std_logic                     := 'X';             -- reset_n
			hps_0_f2h_warm_reset_req_reset_n                  : in    std_logic                     := 'X';             -- reset_n
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
			ddr3_emif_0_soft_reset_reset_n                    : in    std_logic                     := 'X';             -- reset_n
			clk_100_clk                                       : in    std_logic                     := 'X';             -- clk
			openmac_0_mii_txEnable                            : out   std_logic_vector(1 downto 0);                     -- txEnable
			openmac_0_mii_txData                              : out   std_logic_vector(7 downto 0);                     -- txData
			openmac_0_mii_txClk                               : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- txClk
			openmac_0_mii_rxError                             : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxError
			openmac_0_mii_rxDataValid                         : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxDataValid
			openmac_0_mii_rxData                              : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- rxData
			openmac_0_mii_rxClk                               : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- rxClk
			openmac_0_smi_nPhyRst                             : out   std_logic_vector(0 downto 0);                     -- nPhyRst
			openmac_0_smi_clk                                 : out   std_logic_vector(0 downto 0);                     -- clk
			openmac_0_smi_dio                                 : inout std_logic_vector(0 downto 0)  := (others => 'X'); -- dio
			openmac_0_mactimerout_export                      : out   std_logic_vector(0 downto 0);                     -- export
			ddr3_emif_0_pll_ref_clk_clk                       : in    std_logic                     := 'X';             -- clk
			pcp_benchmark_pio_export                          : out   std_logic_vector(7 downto 0)                      -- export
		);
	end component mnDualHostifGpio;

	u0 : component mnDualHostifGpio
		port map (
			memory_mem_a                                      => CONNECTED_TO_memory_mem_a,                                      --                         memory.mem_a
			memory_mem_ba                                     => CONNECTED_TO_memory_mem_ba,                                     --                               .mem_ba
			memory_mem_ck                                     => CONNECTED_TO_memory_mem_ck,                                     --                               .mem_ck
			memory_mem_ck_n                                   => CONNECTED_TO_memory_mem_ck_n,                                   --                               .mem_ck_n
			memory_mem_cke                                    => CONNECTED_TO_memory_mem_cke,                                    --                               .mem_cke
			memory_mem_cs_n                                   => CONNECTED_TO_memory_mem_cs_n,                                   --                               .mem_cs_n
			memory_mem_ras_n                                  => CONNECTED_TO_memory_mem_ras_n,                                  --                               .mem_ras_n
			memory_mem_cas_n                                  => CONNECTED_TO_memory_mem_cas_n,                                  --                               .mem_cas_n
			memory_mem_we_n                                   => CONNECTED_TO_memory_mem_we_n,                                   --                               .mem_we_n
			memory_mem_reset_n                                => CONNECTED_TO_memory_mem_reset_n,                                --                               .mem_reset_n
			memory_mem_dq                                     => CONNECTED_TO_memory_mem_dq,                                     --                               .mem_dq
			memory_mem_dqs                                    => CONNECTED_TO_memory_mem_dqs,                                    --                               .mem_dqs
			memory_mem_dqs_n                                  => CONNECTED_TO_memory_mem_dqs_n,                                  --                               .mem_dqs_n
			memory_mem_odt                                    => CONNECTED_TO_memory_mem_odt,                                    --                               .mem_odt
			memory_mem_dm                                     => CONNECTED_TO_memory_mem_dm,                                     --                               .mem_dm
			memory_oct_rzqin                                  => CONNECTED_TO_memory_oct_rzqin,                                  --                               .oct_rzqin
			memory_0_mem_a                                    => CONNECTED_TO_memory_0_mem_a,                                    --                       memory_0.mem_a
			memory_0_mem_ba                                   => CONNECTED_TO_memory_0_mem_ba,                                   --                               .mem_ba
			memory_0_mem_ck                                   => CONNECTED_TO_memory_0_mem_ck,                                   --                               .mem_ck
			memory_0_mem_ck_n                                 => CONNECTED_TO_memory_0_mem_ck_n,                                 --                               .mem_ck_n
			memory_0_mem_cke                                  => CONNECTED_TO_memory_0_mem_cke,                                  --                               .mem_cke
			memory_0_mem_cs_n                                 => CONNECTED_TO_memory_0_mem_cs_n,                                 --                               .mem_cs_n
			memory_0_mem_dm                                   => CONNECTED_TO_memory_0_mem_dm,                                   --                               .mem_dm
			memory_0_mem_ras_n                                => CONNECTED_TO_memory_0_mem_ras_n,                                --                               .mem_ras_n
			memory_0_mem_cas_n                                => CONNECTED_TO_memory_0_mem_cas_n,                                --                               .mem_cas_n
			memory_0_mem_we_n                                 => CONNECTED_TO_memory_0_mem_we_n,                                 --                               .mem_we_n
			memory_0_mem_reset_n                              => CONNECTED_TO_memory_0_mem_reset_n,                              --                               .mem_reset_n
			memory_0_mem_dq                                   => CONNECTED_TO_memory_0_mem_dq,                                   --                               .mem_dq
			memory_0_mem_dqs                                  => CONNECTED_TO_memory_0_mem_dqs,                                  --                               .mem_dqs
			memory_0_mem_dqs_n                                => CONNECTED_TO_memory_0_mem_dqs_n,                                --                               .mem_dqs_n
			memory_0_mem_odt                                  => CONNECTED_TO_memory_0_mem_odt,                                  --                               .mem_odt
			oct_rzqin                                         => CONNECTED_TO_oct_rzqin,                                         --                            oct.rzqin
			clk_50_clk                                        => CONNECTED_TO_clk_50_clk,                                        --                         clk_50.clk
			reset_reset_n                                     => CONNECTED_TO_reset_reset_n,                                     --                          reset.reset_n
			hps_0_hps_io_hps_io_emac1_inst_TX_CLK             => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_TX_CLK,             --                   hps_0_hps_io.hps_io_emac1_inst_TX_CLK
			hps_0_hps_io_hps_io_emac1_inst_TXD0               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_TXD0,               --                               .hps_io_emac1_inst_TXD0
			hps_0_hps_io_hps_io_emac1_inst_TXD1               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_TXD1,               --                               .hps_io_emac1_inst_TXD1
			hps_0_hps_io_hps_io_emac1_inst_TXD2               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_TXD2,               --                               .hps_io_emac1_inst_TXD2
			hps_0_hps_io_hps_io_emac1_inst_TXD3               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_TXD3,               --                               .hps_io_emac1_inst_TXD3
			hps_0_hps_io_hps_io_emac1_inst_RXD0               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_RXD0,               --                               .hps_io_emac1_inst_RXD0
			hps_0_hps_io_hps_io_emac1_inst_MDIO               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_MDIO,               --                               .hps_io_emac1_inst_MDIO
			hps_0_hps_io_hps_io_emac1_inst_MDC                => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_MDC,                --                               .hps_io_emac1_inst_MDC
			hps_0_hps_io_hps_io_emac1_inst_RX_CTL             => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_RX_CTL,             --                               .hps_io_emac1_inst_RX_CTL
			hps_0_hps_io_hps_io_emac1_inst_TX_CTL             => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_TX_CTL,             --                               .hps_io_emac1_inst_TX_CTL
			hps_0_hps_io_hps_io_emac1_inst_RX_CLK             => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_RX_CLK,             --                               .hps_io_emac1_inst_RX_CLK
			hps_0_hps_io_hps_io_emac1_inst_RXD1               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_RXD1,               --                               .hps_io_emac1_inst_RXD1
			hps_0_hps_io_hps_io_emac1_inst_RXD2               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_RXD2,               --                               .hps_io_emac1_inst_RXD2
			hps_0_hps_io_hps_io_emac1_inst_RXD3               => CONNECTED_TO_hps_0_hps_io_hps_io_emac1_inst_RXD3,               --                               .hps_io_emac1_inst_RXD3
			hps_0_hps_io_hps_io_qspi_inst_IO0                 => CONNECTED_TO_hps_0_hps_io_hps_io_qspi_inst_IO0,                 --                               .hps_io_qspi_inst_IO0
			hps_0_hps_io_hps_io_qspi_inst_IO1                 => CONNECTED_TO_hps_0_hps_io_hps_io_qspi_inst_IO1,                 --                               .hps_io_qspi_inst_IO1
			hps_0_hps_io_hps_io_qspi_inst_IO2                 => CONNECTED_TO_hps_0_hps_io_hps_io_qspi_inst_IO2,                 --                               .hps_io_qspi_inst_IO2
			hps_0_hps_io_hps_io_qspi_inst_IO3                 => CONNECTED_TO_hps_0_hps_io_hps_io_qspi_inst_IO3,                 --                               .hps_io_qspi_inst_IO3
			hps_0_hps_io_hps_io_qspi_inst_SS0                 => CONNECTED_TO_hps_0_hps_io_hps_io_qspi_inst_SS0,                 --                               .hps_io_qspi_inst_SS0
			hps_0_hps_io_hps_io_qspi_inst_CLK                 => CONNECTED_TO_hps_0_hps_io_hps_io_qspi_inst_CLK,                 --                               .hps_io_qspi_inst_CLK
			hps_0_hps_io_hps_io_sdio_inst_CMD                 => CONNECTED_TO_hps_0_hps_io_hps_io_sdio_inst_CMD,                 --                               .hps_io_sdio_inst_CMD
			hps_0_hps_io_hps_io_sdio_inst_D0                  => CONNECTED_TO_hps_0_hps_io_hps_io_sdio_inst_D0,                  --                               .hps_io_sdio_inst_D0
			hps_0_hps_io_hps_io_sdio_inst_D1                  => CONNECTED_TO_hps_0_hps_io_hps_io_sdio_inst_D1,                  --                               .hps_io_sdio_inst_D1
			hps_0_hps_io_hps_io_sdio_inst_CLK                 => CONNECTED_TO_hps_0_hps_io_hps_io_sdio_inst_CLK,                 --                               .hps_io_sdio_inst_CLK
			hps_0_hps_io_hps_io_sdio_inst_D2                  => CONNECTED_TO_hps_0_hps_io_hps_io_sdio_inst_D2,                  --                               .hps_io_sdio_inst_D2
			hps_0_hps_io_hps_io_sdio_inst_D3                  => CONNECTED_TO_hps_0_hps_io_hps_io_sdio_inst_D3,                  --                               .hps_io_sdio_inst_D3
			hps_0_hps_io_hps_io_usb1_inst_D0                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D0,                  --                               .hps_io_usb1_inst_D0
			hps_0_hps_io_hps_io_usb1_inst_D1                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D1,                  --                               .hps_io_usb1_inst_D1
			hps_0_hps_io_hps_io_usb1_inst_D2                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D2,                  --                               .hps_io_usb1_inst_D2
			hps_0_hps_io_hps_io_usb1_inst_D3                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D3,                  --                               .hps_io_usb1_inst_D3
			hps_0_hps_io_hps_io_usb1_inst_D4                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D4,                  --                               .hps_io_usb1_inst_D4
			hps_0_hps_io_hps_io_usb1_inst_D5                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D5,                  --                               .hps_io_usb1_inst_D5
			hps_0_hps_io_hps_io_usb1_inst_D6                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D6,                  --                               .hps_io_usb1_inst_D6
			hps_0_hps_io_hps_io_usb1_inst_D7                  => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_D7,                  --                               .hps_io_usb1_inst_D7
			hps_0_hps_io_hps_io_usb1_inst_CLK                 => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_CLK,                 --                               .hps_io_usb1_inst_CLK
			hps_0_hps_io_hps_io_usb1_inst_STP                 => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_STP,                 --                               .hps_io_usb1_inst_STP
			hps_0_hps_io_hps_io_usb1_inst_DIR                 => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_DIR,                 --                               .hps_io_usb1_inst_DIR
			hps_0_hps_io_hps_io_usb1_inst_NXT                 => CONNECTED_TO_hps_0_hps_io_hps_io_usb1_inst_NXT,                 --                               .hps_io_usb1_inst_NXT
			hps_0_hps_io_hps_io_spim0_inst_CLK                => CONNECTED_TO_hps_0_hps_io_hps_io_spim0_inst_CLK,                --                               .hps_io_spim0_inst_CLK
			hps_0_hps_io_hps_io_spim0_inst_MOSI               => CONNECTED_TO_hps_0_hps_io_hps_io_spim0_inst_MOSI,               --                               .hps_io_spim0_inst_MOSI
			hps_0_hps_io_hps_io_spim0_inst_MISO               => CONNECTED_TO_hps_0_hps_io_hps_io_spim0_inst_MISO,               --                               .hps_io_spim0_inst_MISO
			hps_0_hps_io_hps_io_spim0_inst_SS0                => CONNECTED_TO_hps_0_hps_io_hps_io_spim0_inst_SS0,                --                               .hps_io_spim0_inst_SS0
			hps_0_hps_io_hps_io_uart0_inst_RX                 => CONNECTED_TO_hps_0_hps_io_hps_io_uart0_inst_RX,                 --                               .hps_io_uart0_inst_RX
			hps_0_hps_io_hps_io_uart0_inst_TX                 => CONNECTED_TO_hps_0_hps_io_hps_io_uart0_inst_TX,                 --                               .hps_io_uart0_inst_TX
			hps_0_hps_io_hps_io_i2c0_inst_SDA                 => CONNECTED_TO_hps_0_hps_io_hps_io_i2c0_inst_SDA,                 --                               .hps_io_i2c0_inst_SDA
			hps_0_hps_io_hps_io_i2c0_inst_SCL                 => CONNECTED_TO_hps_0_hps_io_hps_io_i2c0_inst_SCL,                 --                               .hps_io_i2c0_inst_SCL
			hps_0_hps_io_hps_io_can0_inst_RX                  => CONNECTED_TO_hps_0_hps_io_hps_io_can0_inst_RX,                  --                               .hps_io_can0_inst_RX
			hps_0_hps_io_hps_io_can0_inst_TX                  => CONNECTED_TO_hps_0_hps_io_hps_io_can0_inst_TX,                  --                               .hps_io_can0_inst_TX
			hps_0_hps_io_hps_io_trace_inst_CLK                => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_CLK,                --                               .hps_io_trace_inst_CLK
			hps_0_hps_io_hps_io_trace_inst_D0                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D0,                 --                               .hps_io_trace_inst_D0
			hps_0_hps_io_hps_io_trace_inst_D1                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D1,                 --                               .hps_io_trace_inst_D1
			hps_0_hps_io_hps_io_trace_inst_D2                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D2,                 --                               .hps_io_trace_inst_D2
			hps_0_hps_io_hps_io_trace_inst_D3                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D3,                 --                               .hps_io_trace_inst_D3
			hps_0_hps_io_hps_io_trace_inst_D4                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D4,                 --                               .hps_io_trace_inst_D4
			hps_0_hps_io_hps_io_trace_inst_D5                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D5,                 --                               .hps_io_trace_inst_D5
			hps_0_hps_io_hps_io_trace_inst_D6                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D6,                 --                               .hps_io_trace_inst_D6
			hps_0_hps_io_hps_io_trace_inst_D7                 => CONNECTED_TO_hps_0_hps_io_hps_io_trace_inst_D7,                 --                               .hps_io_trace_inst_D7
			hps_0_hps_io_hps_io_gpio_inst_GPIO09              => CONNECTED_TO_hps_0_hps_io_hps_io_gpio_inst_GPIO09,              --                               .hps_io_gpio_inst_GPIO09
			hps_0_hps_io_hps_io_gpio_inst_GPIO35              => CONNECTED_TO_hps_0_hps_io_hps_io_gpio_inst_GPIO35,              --                               .hps_io_gpio_inst_GPIO35
			hps_0_hps_io_hps_io_gpio_inst_GPIO41              => CONNECTED_TO_hps_0_hps_io_hps_io_gpio_inst_GPIO41,              --                               .hps_io_gpio_inst_GPIO41
			hps_0_hps_io_hps_io_gpio_inst_GPIO42              => CONNECTED_TO_hps_0_hps_io_hps_io_gpio_inst_GPIO42,              --                               .hps_io_gpio_inst_GPIO42
			hps_0_hps_io_hps_io_gpio_inst_GPIO43              => CONNECTED_TO_hps_0_hps_io_hps_io_gpio_inst_GPIO43,              --                               .hps_io_gpio_inst_GPIO43
			hps_0_hps_io_hps_io_gpio_inst_GPIO44              => CONNECTED_TO_hps_0_hps_io_hps_io_gpio_inst_GPIO44,              --                               .hps_io_gpio_inst_GPIO44
			led_pio_external_connection_in_port               => CONNECTED_TO_led_pio_external_connection_in_port,               --    led_pio_external_connection.in_port
			led_pio_external_connection_out_port              => CONNECTED_TO_led_pio_external_connection_out_port,              --                               .out_port
			dipsw_pio_external_connection_export              => CONNECTED_TO_dipsw_pio_external_connection_export,              --  dipsw_pio_external_connection.export
			button_pio_external_connection_export             => CONNECTED_TO_button_pio_external_connection_export,             -- button_pio_external_connection.export
			hps_0_h2f_reset_reset_n                           => CONNECTED_TO_hps_0_h2f_reset_reset_n,                           --                hps_0_h2f_reset.reset_n
			hps_0_f2h_cold_reset_req_reset_n                  => CONNECTED_TO_hps_0_f2h_cold_reset_req_reset_n,                  --       hps_0_f2h_cold_reset_req.reset_n
			hps_0_f2h_debug_reset_req_reset_n                 => CONNECTED_TO_hps_0_f2h_debug_reset_req_reset_n,                 --      hps_0_f2h_debug_reset_req.reset_n
			hps_0_f2h_warm_reset_req_reset_n                  => CONNECTED_TO_hps_0_f2h_warm_reset_req_reset_n,                  --       hps_0_f2h_warm_reset_req.reset_n
			ddr3_emif_0_status_local_init_done                => CONNECTED_TO_ddr3_emif_0_status_local_init_done,                --             ddr3_emif_0_status.local_init_done
			ddr3_emif_0_status_local_cal_success              => CONNECTED_TO_ddr3_emif_0_status_local_cal_success,              --                               .local_cal_success
			ddr3_emif_0_status_local_cal_fail                 => CONNECTED_TO_ddr3_emif_0_status_local_cal_fail,                 --                               .local_cal_fail
			ddr3_emif_0_pll_sharing_pll_mem_clk               => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_mem_clk,               --        ddr3_emif_0_pll_sharing.pll_mem_clk
			ddr3_emif_0_pll_sharing_pll_write_clk             => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_write_clk,             --                               .pll_write_clk
			ddr3_emif_0_pll_sharing_pll_locked                => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_locked,                --                               .pll_locked
			ddr3_emif_0_pll_sharing_pll_write_clk_pre_phy_clk => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_write_clk_pre_phy_clk, --                               .pll_write_clk_pre_phy_clk
			ddr3_emif_0_pll_sharing_pll_addr_cmd_clk          => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_addr_cmd_clk,          --                               .pll_addr_cmd_clk
			ddr3_emif_0_pll_sharing_pll_avl_clk               => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_avl_clk,               --                               .pll_avl_clk
			ddr3_emif_0_pll_sharing_pll_config_clk            => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_config_clk,            --                               .pll_config_clk
			ddr3_emif_0_pll_sharing_pll_dr_clk                => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_dr_clk,                --                               .pll_dr_clk
			ddr3_emif_0_pll_sharing_pll_dr_clk_pre_phy_clk    => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_dr_clk_pre_phy_clk,    --                               .pll_dr_clk_pre_phy_clk
			ddr3_emif_0_pll_sharing_pll_mem_phy_clk           => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_mem_phy_clk,           --                               .pll_mem_phy_clk
			ddr3_emif_0_pll_sharing_afi_phy_clk               => CONNECTED_TO_ddr3_emif_0_pll_sharing_afi_phy_clk,               --                               .afi_phy_clk
			ddr3_emif_0_pll_sharing_pll_avl_phy_clk           => CONNECTED_TO_ddr3_emif_0_pll_sharing_pll_avl_phy_clk,           --                               .pll_avl_phy_clk
			ddr3_emif_0_global_reset_reset_n                  => CONNECTED_TO_ddr3_emif_0_global_reset_reset_n,                  --       ddr3_emif_0_global_reset.reset_n
			ddr3_emif_0_afi_reset_export_reset_n              => CONNECTED_TO_ddr3_emif_0_afi_reset_export_reset_n,              --   ddr3_emif_0_afi_reset_export.reset_n
			ddr3_emif_0_soft_reset_reset_n                    => CONNECTED_TO_ddr3_emif_0_soft_reset_reset_n,                    --         ddr3_emif_0_soft_reset.reset_n
			clk_100_clk                                       => CONNECTED_TO_clk_100_clk,                                       --                        clk_100.clk
			openmac_0_mii_txEnable                            => CONNECTED_TO_openmac_0_mii_txEnable,                            --                  openmac_0_mii.txEnable
			openmac_0_mii_txData                              => CONNECTED_TO_openmac_0_mii_txData,                              --                               .txData
			openmac_0_mii_txClk                               => CONNECTED_TO_openmac_0_mii_txClk,                               --                               .txClk
			openmac_0_mii_rxError                             => CONNECTED_TO_openmac_0_mii_rxError,                             --                               .rxError
			openmac_0_mii_rxDataValid                         => CONNECTED_TO_openmac_0_mii_rxDataValid,                         --                               .rxDataValid
			openmac_0_mii_rxData                              => CONNECTED_TO_openmac_0_mii_rxData,                              --                               .rxData
			openmac_0_mii_rxClk                               => CONNECTED_TO_openmac_0_mii_rxClk,                               --                               .rxClk
			openmac_0_smi_nPhyRst                             => CONNECTED_TO_openmac_0_smi_nPhyRst,                             --                  openmac_0_smi.nPhyRst
			openmac_0_smi_clk                                 => CONNECTED_TO_openmac_0_smi_clk,                                 --                               .clk
			openmac_0_smi_dio                                 => CONNECTED_TO_openmac_0_smi_dio,                                 --                               .dio
			openmac_0_mactimerout_export                      => CONNECTED_TO_openmac_0_mactimerout_export,                      --          openmac_0_mactimerout.export
			ddr3_emif_0_pll_ref_clk_clk                       => CONNECTED_TO_ddr3_emif_0_pll_ref_clk_clk,                       --        ddr3_emif_0_pll_ref_clk.clk
			pcp_benchmark_pio_export                          => CONNECTED_TO_pcp_benchmark_pio_export                           --              pcp_benchmark_pio.export
		);
