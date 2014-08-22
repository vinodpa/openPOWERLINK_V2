module rst_ctrl_bit_sync #(
 parameter SYNC_LENGTH = 2
) (
 input  clk,
 input  rst_n,
 output sync_rst_n
);

reg [SYNC_LENGTH-1:0] internal_sync_path;
reg                   first_stage_rst_path;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) first_stage_rst_path <= 1'b0;
  else        first_stage_rst_path <= 1'b1;
end

genvar j;
generate if(SYNC_LENGTH>1) begin: has_sync_flop
  for (j=1; j<SYNC_LENGTH; j=j+1) begin: sync_flop_inst
    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) internal_sync_path[j] <= 1'b0;
      else        internal_sync_path[j] <= internal_sync_path[j-1];
    end
  end
end
endgenerate

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) internal_sync_path[0] <= 1'b0;
  else        internal_sync_path[0] <= first_stage_rst_path;
end

assign sync_rst_n = internal_sync_path[SYNC_LENGTH-1];

endmodule
