module altera_reset_controller #(
 parameter RESET_SOURCE_COUNT = 1,
 parameter RESET_SYNC_LENGTH  = 8,
 parameter GENERATE_PULSE_OUT = 1,
 parameter PULSE_LENGTH       = 1
) (
input                           clk,
input  [RESET_SOURCE_COUNT-1:0] reset_n_src,
output                          combined_reset_n,
output                          pulse_reset_n
);

wire   [RESET_SOURCE_COUNT-1:0] sync_reset_n;
wire   [RESET_SOURCE_COUNT-1:0] rst_pulse;
genvar i;
generate
  for (i=0; i<RESET_SOURCE_COUNT; i=i+1) begin: reset_bus_inst
  // multiple instantiation of reset bit sync
    rst_ctrl_bit_sync #(
    .SYNC_LENGTH (RESET_SYNC_LENGTH)
    ) rst_sync_inst (
    .clk         (clk),
    .rst_n       (reset_n_src[i]),
    .sync_rst_n  (sync_reset_n[i])
    );
  end
endgenerate

genvar k;
generate if (GENERATE_PULSE_OUT) begin: has_valid_pulse
  for (k=0; k<RESET_SOURCE_COUNT; k=k+1) begin: pulse_rst_bus_inst
    altera_edge_detector #(
    .PULSE_EXT (PULSE_LENGTH),
    .EDGE_TYPE (0),
    .IGNORE_RST_WHILE_BUSY (0)
    ) edge_detect_inst (
    .clk       (clk),
    .rst_n     (1'b1),
    .signal_in (sync_reset_n[k]),
    .pulse_out (rst_pulse[k])
    );
  end
//  assign pulse_reset_n = &sync_reset_n_2cyc;
  assign pulse_reset_n = ~(|rst_pulse);
end else begin: no_pulse
  assign pulse_reset_n = 1'b1;
end
endgenerate

assign combined_reset_n = &sync_reset_n;

endmodule
