// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
// Date: 19.03.2017
// Description: Ariane Top-level wrapper to break out SV structs to logic vectors.


module ariane_verilog_wrap
    import ariane_pkg::*;
#(
  parameter int unsigned               RASDepth              = 2,
  parameter int unsigned               BTBEntries            = 32,
  parameter int unsigned               BHTEntries            = 128,
  // debug module base address
  parameter logic [63:0]               DmBaseAddress         = 64'h0,
  // swap endianess in l15 adapter
  parameter bit                        SwapEndianess         = 1,
  // PMA configuration
  // idempotent region
  parameter int unsigned               NrNonIdempotentRules  =  1,
  parameter logic [NrMaxRules*64-1:0]  NonIdempotentAddrBase = 64'h00C0000000,
  parameter logic [NrMaxRules*64-1:0]  NonIdempotentLength   = 64'hFFFFFFFFFF,
  // executable regions
  parameter int unsigned               NrExecuteRegionRules  =  0,
  parameter logic [NrMaxRules*64-1:0]  ExecuteRegionAddrBase = '0,
  parameter logic [NrMaxRules*64-1:0]  ExecuteRegionLength   = '0,
  // cacheable regions
  parameter int unsigned               NrCachedRegionRules   =  0,
  parameter logic [NrMaxRules*64-1:0]  CachedRegionAddrBase  = '0,
  parameter logic [NrMaxRules*64-1:0]  CachedRegionLength    = '0,
  // PMP
  parameter int unsigned               NrPMPEntries          =  8
) (
  input                       clk_i,
  input                       reset_l,      // this is an openpiton-specific name, do not change (hier. paths in TB use this)
  output                      spc_grst_l,   // this is an openpiton-specific name, do not change (hier. paths in TB use this)
  // Core ID, Cluster ID and boot address are considered more or less static
  input  [riscv::VLEN-1:0]               boot_addr_i,  // reset boot address
  input  [riscv::XLEN-1:0]               hart_id_i,    // hart id in a multicore environment (reflected in a CSR)
  // Interrupt inputs
  input  [1:0]                irq_i,        // level sensitive IR lines, mip & sip (async)
  input                       ipi_i,        // inter-processor interrupts (async)
  // Timer facilities
  input                       time_irq_i,   // timer interrupt in (async)
  input                       debug_req_i,  // debug request (async)

`ifdef PITON_ARIANE
  // L15 (memory side)
  output [$size(wt_cache_pkg::l15_req_t)-1:0]  l15_req_o,
  input  [$size(wt_cache_pkg::l15_rtrn_t)-1:0] l15_rtrn_i
`else
  // AXI (memory side)
  output [$size(ariane_axi::req_t)-1:0]             axi_req_o,
  input  [$size(ariane_axi::resp_t)-1:0]            axi_resp_i
`endif
 );

// assign bitvector to packed struct and vice versa
`ifdef PITON_ARIANE
  // L15 (memory side)
  wt_cache_pkg::l15_req_t  l15_req;
  wt_cache_pkg::l15_rtrn_t l15_rtrn;

  assign l15_rtrn  = l15_rtrn_i;

`ifdef NO_PMESH_NOC_WAKEUP
  assign l15_req_o = l15_req;
`else
  wt_cache_pkg::l15_req_t  l15_req_gated;
  assign l15_req_o = l15_req_gated;
`endif
`else
  ariane_axi::req_t             axi_req;
  ariane_axi::resp_t            axi_resp;

  assign axi_req_o = axi_req;
  assign axi_resp  = axi_resp_i;
`endif

`ifndef NO_PMESH_NOC_WAKEUP
  /////////////////////////////
  // Core wakeup mechanism
  /////////////////////////////
  // Ariane in future could support an idle mode
  // For now, we just gate the transducer_l15_val signal
  // until the wakeup interrupt is received.
  // Off-chip logic will send one of these to core 0
  // and that core can then use stores to INT_VEC_DIS
  // to send wakeup interrupts to the other cores

  // // the logic below catches the initial wake up interrupt that enables the cores.
  logic wake_up_d, wake_up_q;
  logic rst_n;
  logic wakeup_override_q;

  // See OpenPiton micro_arch.pdf and OST1 microarchitecture spec for format
  // [17:16] == 01 and [5:0] == 000001 means power on reset
  assign wake_up_d = wake_up_q || ((l15_rtrn.l15_returntype == wt_cache_pkg::L15_INT_RET) && l15_rtrn.l15_val && (l15_rtrn.l15_data_0[17:16] == 2'b01) && (l15_rtrn.l15_data_0[5:0] == 6'b000001));

  always_ff @(posedge clk_i) begin : p_regs
    if(~reset_l) begin
      wake_up_q <= 0;
    end else begin
      wake_up_q <= wake_up_d;
    end
  end

  // // reset gate this
  // assign rst_n = wake_up_q & reset_l;
  assign rst_n = reset_l;

  always_comb begin
    l15_req_gated = l15_req;
    l15_req_gated.l15_val = l15_req.l15_val && (wake_up_q || wakeup_override_q);
  end

`else // ifndef NO_PMESH_NOC_WAKEUP

  // this is a workaround,
  // we basically wait for 32k cycles such that the SRAMs in openpiton can initialize
  // 128KB..8K cycles
  // 256KB..16K cycles
  // etc, so this should be enough for 512k per tile

  logic [15:0] wake_up_cnt_d, wake_up_cnt_q;
  logic rst_n;

  assign wake_up_cnt_d = (wake_up_cnt_q[$high(wake_up_cnt_q)]) ? wake_up_cnt_q : wake_up_cnt_q + 1;

  always_ff @(posedge clk_i) begin : p_regs
    if(~reset_l) begin
      wake_up_cnt_q <= 0;
    end else begin
      wake_up_cnt_q <= wake_up_cnt_d;
    end
  end

  // reset gate this
  assign rst_n = wake_up_cnt_q[$high(wake_up_cnt_q)] & reset_l;

`endif // ifndef NO_PMESH_NOC_WAKEUP


  /////////////////////////////
  // synchronizers
  /////////////////////////////

  logic [1:0] irq;
  logic ipi, time_irq, debug_req;

  // reset synchronization
  /*
   * Warning: `initial` assignments are strongly discouraged in ASIC-oriented RTL
   * designs. The following initialization is used solely to handle SV assertions
   * embedded inside the Ariane core.
  */
  logic spc_grst_l = 1'b0;
  logic rst_n_f;

  always_ff @(posedge clk_i)
      rst_n_f <= rst_n;

  always_ff @(posedge clk_i)
    /*
     * Warning: force conversion from 'x' to '0' is strongly discouraged in
      * ASIC-oriented RTL designs. The following initilization is used solely to
      * handle SV assertions embedded inside the Ariane core.
    */
    // synopsys translate_off
    if (rst_n_f === 1'bx)
      spc_grst_l <= 1'b0;
    else
    // synopsys translate_on
      spc_grst_l <= rst_n_f;

  // interrupts
  always_ff @(posedge clk_i) begin
      if (~spc_grst_l) begin
          irq <= 'b0;
          ipi <= 'b0;
          time_irq <= 'b0;
          debug_req <= 'b0;
      end else begin
          irq <= irq_i;
          ipi <= ipi_i;
          time_irq <= time_irq_i;
          debug_req <= debug_req_i;
      end
  end

  /////////////////////////////
  // ariane instance
  /////////////////////////////

  localparam ariane_pkg::ariane_cfg_t ArianeOpenPitonCfg = '{
    RASDepth:              RASDepth,
    BTBEntries:            BTBEntries,
    BHTEntries:            BHTEntries,
    // idempotent region
    NrNonIdempotentRules:  NrNonIdempotentRules,
    NonIdempotentAddrBase: NonIdempotentAddrBase,
    NonIdempotentLength:   NonIdempotentLength,
    NrExecuteRegionRules:  NrExecuteRegionRules,
    ExecuteRegionAddrBase: ExecuteRegionAddrBase,
    ExecuteRegionLength:   ExecuteRegionLength,
    // cached region
    NrCachedRegionRules:   NrCachedRegionRules,
    CachedRegionAddrBase:  CachedRegionAddrBase,
    CachedRegionLength:    CachedRegionLength,
    // cache config
    Axi64BitCompliant:     1'b0,
    SwapEndianess:         SwapEndianess,
    // debug
    DmBaseAddress:         DmBaseAddress,
    NrPMPEntries:          NrPMPEntries
  };

  ariane #(
    .ArianeCfg ( ArianeOpenPitonCfg )
  ) ariane (
    .clk_i       ( clk_i      ),
    .rst_ni      ( spc_grst_l ),
    .boot_addr_i              ,// constant
    .hart_id_i                ,// constant
    .irq_i       ( irq        ),
    .ipi_i       ( ipi        ),
    .time_irq_i  ( time_irq   ),
    .debug_req_i ( debug_req  ),
`ifdef PITON_ARIANE
    .l15_req_o   ( l15_req   ),
    .l15_rtrn_i  ( l15_rtrn  )
`else
    .axi_req_o   ( axi_req   ),
    .axi_resp_i  ( axi_resp  )
`endif
  );

endmodule // ariane_verilog_wrap
