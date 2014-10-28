
module mnDualHostifGpio (
	memory_mem_a,
	memory_mem_ba,
	memory_mem_ck,
	memory_mem_ck_n,
	memory_mem_cke,
	memory_mem_cs_n,
	memory_mem_ras_n,
	memory_mem_cas_n,
	memory_mem_we_n,
	memory_mem_reset_n,
	memory_mem_dq,
	memory_mem_dqs,
	memory_mem_dqs_n,
	memory_mem_odt,
	memory_mem_dm,
	memory_oct_rzqin,
	memory_0_mem_a,
	memory_0_mem_ba,
	memory_0_mem_ck,
	memory_0_mem_ck_n,
	memory_0_mem_cke,
	memory_0_mem_cs_n,
	memory_0_mem_dm,
	memory_0_mem_ras_n,
	memory_0_mem_cas_n,
	memory_0_mem_we_n,
	memory_0_mem_reset_n,
	memory_0_mem_dq,
	memory_0_mem_dqs,
	memory_0_mem_dqs_n,
	memory_0_mem_odt,
	oct_rzqin,
	clk_50_clk,
	reset_reset_n,
	hps_0_hps_io_hps_io_emac1_inst_TX_CLK,
	hps_0_hps_io_hps_io_emac1_inst_TXD0,
	hps_0_hps_io_hps_io_emac1_inst_TXD1,
	hps_0_hps_io_hps_io_emac1_inst_TXD2,
	hps_0_hps_io_hps_io_emac1_inst_TXD3,
	hps_0_hps_io_hps_io_emac1_inst_RXD0,
	hps_0_hps_io_hps_io_emac1_inst_MDIO,
	hps_0_hps_io_hps_io_emac1_inst_MDC,
	hps_0_hps_io_hps_io_emac1_inst_RX_CTL,
	hps_0_hps_io_hps_io_emac1_inst_TX_CTL,
	hps_0_hps_io_hps_io_emac1_inst_RX_CLK,
	hps_0_hps_io_hps_io_emac1_inst_RXD1,
	hps_0_hps_io_hps_io_emac1_inst_RXD2,
	hps_0_hps_io_hps_io_emac1_inst_RXD3,
	hps_0_hps_io_hps_io_qspi_inst_IO0,
	hps_0_hps_io_hps_io_qspi_inst_IO1,
	hps_0_hps_io_hps_io_qspi_inst_IO2,
	hps_0_hps_io_hps_io_qspi_inst_IO3,
	hps_0_hps_io_hps_io_qspi_inst_SS0,
	hps_0_hps_io_hps_io_qspi_inst_CLK,
	hps_0_hps_io_hps_io_sdio_inst_CMD,
	hps_0_hps_io_hps_io_sdio_inst_D0,
	hps_0_hps_io_hps_io_sdio_inst_D1,
	hps_0_hps_io_hps_io_sdio_inst_CLK,
	hps_0_hps_io_hps_io_sdio_inst_D2,
	hps_0_hps_io_hps_io_sdio_inst_D3,
	hps_0_hps_io_hps_io_usb1_inst_D0,
	hps_0_hps_io_hps_io_usb1_inst_D1,
	hps_0_hps_io_hps_io_usb1_inst_D2,
	hps_0_hps_io_hps_io_usb1_inst_D3,
	hps_0_hps_io_hps_io_usb1_inst_D4,
	hps_0_hps_io_hps_io_usb1_inst_D5,
	hps_0_hps_io_hps_io_usb1_inst_D6,
	hps_0_hps_io_hps_io_usb1_inst_D7,
	hps_0_hps_io_hps_io_usb1_inst_CLK,
	hps_0_hps_io_hps_io_usb1_inst_STP,
	hps_0_hps_io_hps_io_usb1_inst_DIR,
	hps_0_hps_io_hps_io_usb1_inst_NXT,
	hps_0_hps_io_hps_io_spim0_inst_CLK,
	hps_0_hps_io_hps_io_spim0_inst_MOSI,
	hps_0_hps_io_hps_io_spim0_inst_MISO,
	hps_0_hps_io_hps_io_spim0_inst_SS0,
	hps_0_hps_io_hps_io_uart0_inst_RX,
	hps_0_hps_io_hps_io_uart0_inst_TX,
	hps_0_hps_io_hps_io_i2c0_inst_SDA,
	hps_0_hps_io_hps_io_i2c0_inst_SCL,
	hps_0_hps_io_hps_io_can0_inst_RX,
	hps_0_hps_io_hps_io_can0_inst_TX,
	hps_0_hps_io_hps_io_trace_inst_CLK,
	hps_0_hps_io_hps_io_trace_inst_D0,
	hps_0_hps_io_hps_io_trace_inst_D1,
	hps_0_hps_io_hps_io_trace_inst_D2,
	hps_0_hps_io_hps_io_trace_inst_D3,
	hps_0_hps_io_hps_io_trace_inst_D4,
	hps_0_hps_io_hps_io_trace_inst_D5,
	hps_0_hps_io_hps_io_trace_inst_D6,
	hps_0_hps_io_hps_io_trace_inst_D7,
	hps_0_hps_io_hps_io_gpio_inst_GPIO09,
	hps_0_hps_io_hps_io_gpio_inst_GPIO35,
	hps_0_hps_io_hps_io_gpio_inst_GPIO41,
	hps_0_hps_io_hps_io_gpio_inst_GPIO42,
	hps_0_hps_io_hps_io_gpio_inst_GPIO43,
	hps_0_hps_io_hps_io_gpio_inst_GPIO44,
	led_pio_external_connection_in_port,
	led_pio_external_connection_out_port,
	dipsw_pio_external_connection_export,
	button_pio_external_connection_export,
	hps_0_h2f_reset_reset_n,
	hps_0_f2h_cold_reset_req_reset_n,
	hps_0_f2h_debug_reset_req_reset_n,
	hps_0_f2h_warm_reset_req_reset_n,
	ddr3_emif_0_status_local_init_done,
	ddr3_emif_0_status_local_cal_success,
	ddr3_emif_0_status_local_cal_fail,
	ddr3_emif_0_pll_sharing_pll_mem_clk,
	ddr3_emif_0_pll_sharing_pll_write_clk,
	ddr3_emif_0_pll_sharing_pll_locked,
	ddr3_emif_0_pll_sharing_pll_write_clk_pre_phy_clk,
	ddr3_emif_0_pll_sharing_pll_addr_cmd_clk,
	ddr3_emif_0_pll_sharing_pll_avl_clk,
	ddr3_emif_0_pll_sharing_pll_config_clk,
	ddr3_emif_0_pll_sharing_pll_dr_clk,
	ddr3_emif_0_pll_sharing_pll_dr_clk_pre_phy_clk,
	ddr3_emif_0_pll_sharing_pll_mem_phy_clk,
	ddr3_emif_0_pll_sharing_afi_phy_clk,
	ddr3_emif_0_pll_sharing_pll_avl_phy_clk,
	ddr3_emif_0_global_reset_reset_n,
	ddr3_emif_0_afi_reset_export_reset_n,
	ddr3_emif_0_soft_reset_reset_n,
	clk_100_clk,
	openmac_0_mii_txEnable,
	openmac_0_mii_txData,
	openmac_0_mii_txClk,
	openmac_0_mii_rxError,
	openmac_0_mii_rxDataValid,
	openmac_0_mii_rxData,
	openmac_0_mii_rxClk,
	openmac_0_smi_nPhyRst,
	openmac_0_smi_clk,
	openmac_0_smi_dio,
	openmac_0_mactimerout_export,
	ddr3_emif_0_pll_ref_clk_clk,
	pcp_benchmark_pio_export);	

	output	[14:0]	memory_mem_a;
	output	[2:0]	memory_mem_ba;
	output		memory_mem_ck;
	output		memory_mem_ck_n;
	output		memory_mem_cke;
	output		memory_mem_cs_n;
	output		memory_mem_ras_n;
	output		memory_mem_cas_n;
	output		memory_mem_we_n;
	output		memory_mem_reset_n;
	inout	[39:0]	memory_mem_dq;
	inout	[4:0]	memory_mem_dqs;
	inout	[4:0]	memory_mem_dqs_n;
	output		memory_mem_odt;
	output	[4:0]	memory_mem_dm;
	input		memory_oct_rzqin;
	output	[12:0]	memory_0_mem_a;
	output	[2:0]	memory_0_mem_ba;
	output	[0:0]	memory_0_mem_ck;
	output	[0:0]	memory_0_mem_ck_n;
	output	[0:0]	memory_0_mem_cke;
	output	[0:0]	memory_0_mem_cs_n;
	output	[3:0]	memory_0_mem_dm;
	output	[0:0]	memory_0_mem_ras_n;
	output	[0:0]	memory_0_mem_cas_n;
	output	[0:0]	memory_0_mem_we_n;
	output		memory_0_mem_reset_n;
	inout	[31:0]	memory_0_mem_dq;
	inout	[3:0]	memory_0_mem_dqs;
	inout	[3:0]	memory_0_mem_dqs_n;
	output	[0:0]	memory_0_mem_odt;
	input		oct_rzqin;
	input		clk_50_clk;
	input		reset_reset_n;
	output		hps_0_hps_io_hps_io_emac1_inst_TX_CLK;
	output		hps_0_hps_io_hps_io_emac1_inst_TXD0;
	output		hps_0_hps_io_hps_io_emac1_inst_TXD1;
	output		hps_0_hps_io_hps_io_emac1_inst_TXD2;
	output		hps_0_hps_io_hps_io_emac1_inst_TXD3;
	input		hps_0_hps_io_hps_io_emac1_inst_RXD0;
	inout		hps_0_hps_io_hps_io_emac1_inst_MDIO;
	output		hps_0_hps_io_hps_io_emac1_inst_MDC;
	input		hps_0_hps_io_hps_io_emac1_inst_RX_CTL;
	output		hps_0_hps_io_hps_io_emac1_inst_TX_CTL;
	input		hps_0_hps_io_hps_io_emac1_inst_RX_CLK;
	input		hps_0_hps_io_hps_io_emac1_inst_RXD1;
	input		hps_0_hps_io_hps_io_emac1_inst_RXD2;
	input		hps_0_hps_io_hps_io_emac1_inst_RXD3;
	inout		hps_0_hps_io_hps_io_qspi_inst_IO0;
	inout		hps_0_hps_io_hps_io_qspi_inst_IO1;
	inout		hps_0_hps_io_hps_io_qspi_inst_IO2;
	inout		hps_0_hps_io_hps_io_qspi_inst_IO3;
	output		hps_0_hps_io_hps_io_qspi_inst_SS0;
	output		hps_0_hps_io_hps_io_qspi_inst_CLK;
	inout		hps_0_hps_io_hps_io_sdio_inst_CMD;
	inout		hps_0_hps_io_hps_io_sdio_inst_D0;
	inout		hps_0_hps_io_hps_io_sdio_inst_D1;
	output		hps_0_hps_io_hps_io_sdio_inst_CLK;
	inout		hps_0_hps_io_hps_io_sdio_inst_D2;
	inout		hps_0_hps_io_hps_io_sdio_inst_D3;
	inout		hps_0_hps_io_hps_io_usb1_inst_D0;
	inout		hps_0_hps_io_hps_io_usb1_inst_D1;
	inout		hps_0_hps_io_hps_io_usb1_inst_D2;
	inout		hps_0_hps_io_hps_io_usb1_inst_D3;
	inout		hps_0_hps_io_hps_io_usb1_inst_D4;
	inout		hps_0_hps_io_hps_io_usb1_inst_D5;
	inout		hps_0_hps_io_hps_io_usb1_inst_D6;
	inout		hps_0_hps_io_hps_io_usb1_inst_D7;
	input		hps_0_hps_io_hps_io_usb1_inst_CLK;
	output		hps_0_hps_io_hps_io_usb1_inst_STP;
	input		hps_0_hps_io_hps_io_usb1_inst_DIR;
	input		hps_0_hps_io_hps_io_usb1_inst_NXT;
	output		hps_0_hps_io_hps_io_spim0_inst_CLK;
	output		hps_0_hps_io_hps_io_spim0_inst_MOSI;
	input		hps_0_hps_io_hps_io_spim0_inst_MISO;
	output		hps_0_hps_io_hps_io_spim0_inst_SS0;
	input		hps_0_hps_io_hps_io_uart0_inst_RX;
	output		hps_0_hps_io_hps_io_uart0_inst_TX;
	inout		hps_0_hps_io_hps_io_i2c0_inst_SDA;
	inout		hps_0_hps_io_hps_io_i2c0_inst_SCL;
	input		hps_0_hps_io_hps_io_can0_inst_RX;
	output		hps_0_hps_io_hps_io_can0_inst_TX;
	output		hps_0_hps_io_hps_io_trace_inst_CLK;
	output		hps_0_hps_io_hps_io_trace_inst_D0;
	output		hps_0_hps_io_hps_io_trace_inst_D1;
	output		hps_0_hps_io_hps_io_trace_inst_D2;
	output		hps_0_hps_io_hps_io_trace_inst_D3;
	output		hps_0_hps_io_hps_io_trace_inst_D4;
	output		hps_0_hps_io_hps_io_trace_inst_D5;
	output		hps_0_hps_io_hps_io_trace_inst_D6;
	output		hps_0_hps_io_hps_io_trace_inst_D7;
	inout		hps_0_hps_io_hps_io_gpio_inst_GPIO09;
	inout		hps_0_hps_io_hps_io_gpio_inst_GPIO35;
	inout		hps_0_hps_io_hps_io_gpio_inst_GPIO41;
	inout		hps_0_hps_io_hps_io_gpio_inst_GPIO42;
	inout		hps_0_hps_io_hps_io_gpio_inst_GPIO43;
	inout		hps_0_hps_io_hps_io_gpio_inst_GPIO44;
	input	[3:0]	led_pio_external_connection_in_port;
	output	[3:0]	led_pio_external_connection_out_port;
	input	[3:0]	dipsw_pio_external_connection_export;
	input	[1:0]	button_pio_external_connection_export;
	output		hps_0_h2f_reset_reset_n;
	input		hps_0_f2h_cold_reset_req_reset_n;
	input		hps_0_f2h_debug_reset_req_reset_n;
	input		hps_0_f2h_warm_reset_req_reset_n;
	output		ddr3_emif_0_status_local_init_done;
	output		ddr3_emif_0_status_local_cal_success;
	output		ddr3_emif_0_status_local_cal_fail;
	output		ddr3_emif_0_pll_sharing_pll_mem_clk;
	output		ddr3_emif_0_pll_sharing_pll_write_clk;
	output		ddr3_emif_0_pll_sharing_pll_locked;
	output		ddr3_emif_0_pll_sharing_pll_write_clk_pre_phy_clk;
	output		ddr3_emif_0_pll_sharing_pll_addr_cmd_clk;
	output		ddr3_emif_0_pll_sharing_pll_avl_clk;
	output		ddr3_emif_0_pll_sharing_pll_config_clk;
	output		ddr3_emif_0_pll_sharing_pll_dr_clk;
	output		ddr3_emif_0_pll_sharing_pll_dr_clk_pre_phy_clk;
	output		ddr3_emif_0_pll_sharing_pll_mem_phy_clk;
	output		ddr3_emif_0_pll_sharing_afi_phy_clk;
	output		ddr3_emif_0_pll_sharing_pll_avl_phy_clk;
	input		ddr3_emif_0_global_reset_reset_n;
	output		ddr3_emif_0_afi_reset_export_reset_n;
	input		ddr3_emif_0_soft_reset_reset_n;
	input		clk_100_clk;
	output	[1:0]	openmac_0_mii_txEnable;
	output	[7:0]	openmac_0_mii_txData;
	input	[1:0]	openmac_0_mii_txClk;
	input	[1:0]	openmac_0_mii_rxError;
	input	[1:0]	openmac_0_mii_rxDataValid;
	input	[7:0]	openmac_0_mii_rxData;
	input	[1:0]	openmac_0_mii_rxClk;
	output	[0:0]	openmac_0_smi_nPhyRst;
	output	[0:0]	openmac_0_smi_clk;
	inout	[0:0]	openmac_0_smi_dio;
	output	[0:0]	openmac_0_mactimerout_export;
	input		ddr3_emif_0_pll_ref_clk_clk;
	output	[7:0]	pcp_benchmark_pio_export;
endmodule
