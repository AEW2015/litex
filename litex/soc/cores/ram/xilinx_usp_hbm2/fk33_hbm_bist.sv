// One ordered AXI burst unit per native HBM port. No clock/reset/HBM control.
`timescale 1ns/1ps
module fk33_hbm_bist #(parameter integer CHANNEL=0)(
 input wire clk,rst,start,enable,stop,
 input wire [27:0] cfg_base,
 input wire [4:0] cfg_beats,
 input wire [31:0] cfg_count,cfg_seed,cfg_strobe,
 input wire [2:0] cfg_stall,
 input wire cfg_masked,
 output wire busy,
 output reg done,stopped,faulted,
 output reg [63:0] write_beats,read_beats,cycles,errors,
 output reg [32:0] first_bad_addr,
 output reg [31:0] first_bad_expected,first_bad_actual,
 output reg [7:0] first_bad_code,
 output wire transfer,
 output wire [32:0] m_awaddr,
 output wire [5:0] m_awid,
 output wire [7:0] m_awlen,
 output wire [2:0] m_awsize,
 output wire [1:0] m_awburst,
 output wire m_awvalid,input wire m_awready,
 output wire [255:0] m_wdata,
 output wire [31:0] m_wstrb,
 output wire m_wlast,m_wvalid,input wire m_wready,
 input wire [5:0] m_bid,input wire [1:0] m_bresp,
 input wire m_bvalid,output wire m_bready,
 output wire [32:0] m_araddr,
 output wire [5:0] m_arid,
 output wire [7:0] m_arlen,
 output wire [2:0] m_arsize,
 output wire [1:0] m_arburst,
 output wire m_arvalid,input wire m_arready,
 input wire [5:0] m_rid,input wire [255:0] m_rdata,
 input wire [1:0] m_rresp,input wire m_rlast,m_rvalid,
 output wire m_rready
);
 localparam [2:0] IDLE=0,AW=1,W=2,B=3,AR=4,R=5;
 localparam [4:0] ADDRESS_TAG=5'd4+(CHANNEL%4);
 reg [2:0] state;
 reg [18:0] unit_hi,next_unit_hi;
 reg [4:0] beats,beat;
 reg [31:0] count,strobe,unit_index;
 // Keep these captured configuration FFs local to each engine. Otherwise
 // equivalent registers merge across ports and route into remote checkers.
 (* DONT_TOUCH = "TRUE" *) reg [31:0] seed;
 (* DONT_TOUCH = "TRUE" *) reg masked;
 reg [2:0] stall;
 reg mask_phase,stop_pending,w_held,aw_done,w_done;
 // Accepted bases are 512-byte aligned. Register the unit address so the
 // per-beat checker has no unit-index/address carry chain. next_unit_hi also
 // preserves the old address arithmetic for malformed late beats 16..31.
 wire [27:0] unit_addr={unit_hi,9'b0};
 wire [27:0] beat_addr={beat[4] ? next_unit_hi : unit_hi,beat[3:0],5'b0};
 wire [32:0] full_beat_addr={ADDRESS_TAG,beat_addr};
 assign busy=state!=IDLE;
 assign m_awaddr={ADDRESS_TAG,unit_addr};
 assign m_araddr={ADDRESS_TAG,unit_addr};
 assign m_awid=6'd1;
 assign m_arid=6'd1;
 assign m_awlen={3'b000,beats}-1'b1;
 assign m_arlen={3'b000,beats}-1'b1;
 assign m_awsize=3'd5;
 assign m_arsize=3'd5;
 assign m_awburst=2'b01;
 assign m_arburst=2'b01;
 assign m_awvalid=state==AW && !aw_done;
 assign m_arvalid=state==AR;
 assign m_wvalid=state==AW && !w_done && (!stall[0] || cycles[1:0]!=0 || w_held);
 assign m_wlast=beat==beats-1'b1;
 assign m_wstrb=mask_phase ? strobe : 32'hffffffff;
 assign m_bready=state==B && (!stall[2] || cycles[2:0]>=3);
 assign m_rready=state==R && (!stall[1] || cycles[2:0]>=3);
 wire aw_fire=m_awvalid&&m_awready;
 wire w_fire=m_wvalid&&m_wready;
 wire b_fire=m_bvalid&&m_bready;
 wire ar_fire=m_arvalid&&m_arready;
 wire r_fire=m_rvalid&&m_rready;
 assign transfer=w_fire|r_fire;
 function automatic [31:0] pattern(input [27:0] address,input integer lane,input [31:0] value_seed);
  pattern=value_seed ^ 32'h94660000 ^ (CHANNEL*32'h01010101) ^
          {4'b0,address} ^ (lane*32'h01020304);
 endfunction
 wire [31:0] lane_expected[0:7];
 wire [7:0] lane_bad;
 wire data_bad=|lane_bad;
 genvar lane,byte_index;
 generate for(lane=0;lane<8;lane=lane+1) begin: lanes
  localparam [2:0] LANE=lane;
  wire [27:0] lane_addr={beat_addr[27:5],LANE,2'b00};
  wire [31:0] lane_pattern=pattern(lane_addr,lane,seed);
  assign m_wdata[lane*32+:32]=mask_phase ? ~lane_pattern : lane_pattern;
  for(byte_index=0;byte_index<4;byte_index=byte_index+1) begin: bytes
   assign lane_expected[lane][byte_index*8+:8]=
    lane_pattern[byte_index*8+:8] ^ {8{masked&&strobe[lane*4+byte_index]}};
  end
  assign lane_bad[lane]=m_rdata[lane*32+:32]!=lane_expected[lane];
 end endgenerate
 // Balanced lowest-index selection. The error flag uses the parallel OR,
 // independently of the metadata priority mux and first-error register data.
 wire [2:0] pair0=lane_bad[0] ? 3'd0 : 3'd1;
 wire [2:0] pair1=lane_bad[2] ? 3'd2 : 3'd3;
 wire [2:0] pair2=lane_bad[4] ? 3'd4 : 3'd5;
 wire [2:0] pair3=lane_bad[6] ? 3'd6 : 3'd7;
 wire [2:0] half0=(|lane_bad[1:0]) ? pair0 : pair1;
 wire [2:0] half1=(|lane_bad[5:4]) ? pair2 : pair3;
 wire [2:0] bad_lane=data_bad ? ((|lane_bad[3:0]) ? half0 : half1) : 3'd0;
 wire [31:0] bad_expected=data_bad ? lane_expected[bad_lane] : 32'b0;
 wire [31:0] bad_actual=data_bad ? m_rdata[bad_lane*32+:32] : 32'b0;
 wire response_error=(b_fire&&((m_bid!=6'd1)||(m_bresp!=0))) ||
  (r_fire&&((m_rid!=6'd1)||(m_rresp!=0)||
             (m_rlast!=(beat==beats-1'b1))||data_bad));
 reg [7:0] event_code;
 reg [32:0] event_addr;
 reg [31:0] event_expected,event_actual;
 always @* begin
  event_code=0;event_addr=full_beat_addr;event_expected=0;event_actual=0;
  if(b_fire) begin
   if(m_bid!=6'd1) begin event_code=1;event_expected=1;event_actual={26'b0,m_bid};end
   else if(m_bresp!=0) begin event_code=2;event_actual={30'b0,m_bresp};end
  end
  if(r_fire) begin
   if(m_rid!=6'd1) begin event_code=3;event_expected=1;event_actual={26'b0,m_rid};end
   else if(m_rresp!=0) begin event_code=4;event_actual={30'b0,m_rresp};end
   else if(m_rlast!=(beat==beats-1'b1)) begin
    event_code=5;event_expected=beat==beats-1'b1;event_actual=m_rlast;
   end else if(data_bad) begin
    event_code=6;event_addr={ADDRESS_TAG,beat_addr[27:5],bad_lane,2'b00};
    event_expected=bad_expected;event_actual=bad_actual;
   end
  end
 end
 always @(posedge clk) begin
  if(rst) begin
   state<=IDLE;done<=0;stopped<=0;write_beats<=0;read_beats<=0;cycles<=0;errors<=0;faulted<=0;
   first_bad_addr<=0;first_bad_expected<=0;first_bad_actual<=0;first_bad_code<=0;
   unit_hi<=0;next_unit_hi<=0;beats<=0;count<=0;seed<=0;strobe<=0;stall<=0;masked<=0;
   beat<=0;unit_index<=0;mask_phase<=0;stop_pending<=0;w_held<=0;aw_done<=0;w_done<=0;
  end else if(start) begin
   // Hub sends START to all ports: unselected counters clear without traffic.
   state<=enable ? AW : IDLE;done<=0;stopped<=0;
   write_beats<=0;read_beats<=0;cycles<=0;errors<=0;faulted<=0;
   first_bad_addr<=0;first_bad_expected<=0;first_bad_actual<=0;first_bad_code<=0;
   unit_hi<=cfg_base[27:9];next_unit_hi<=cfg_base[27:9]+19'd1;beats<=cfg_beats;count<=cfg_count;seed<=cfg_seed;
   strobe<=cfg_strobe;stall<=cfg_stall;masked<=cfg_masked;
   beat<=0;unit_index<=0;mask_phase<=0;stop_pending<=0;w_held<=0;aw_done<=0;w_done<=0;
  end else begin
   if(busy) cycles<=cycles+1'b1;
   if(busy&&(stop||faulted)) stop_pending<=1;
   if(m_wvalid&&!m_wready) w_held<=1;
   else if(w_fire) w_held<=0;
   if(w_fire) write_beats<=write_beats+1'b1;
   if(r_fire) read_beats<=read_beats+1'b1;
   if(response_error) begin
    // Fault control is independent of telemetry counter width and overflow.
    faulted<=1;errors<=errors+1'b1;
    if(!faulted) begin
     first_bad_addr<=event_addr;first_bad_expected<=event_expected;
     first_bad_actual<=event_actual;first_bad_code<=event_code;
    end
   end
   case(state)
    AW:begin
     // AW and W are independent; neither VALID waits for the other's READY.
     if(aw_fire) aw_done<=1;
     if(w_fire) begin
      if(m_wlast) w_done<=1;
      else beat<=beat+1'b1;
     end
     if((aw_done||aw_fire)&&(w_done||(w_fire&&m_wlast))) begin state<=B;beat<=0;end
    end
    B:if(b_fire) begin
     if(masked&&!mask_phase) begin mask_phase<=1;state<=AW;aw_done<=0;w_done<=0;beat<=0;end
     else begin state<=AR;beat<=0;end
    end
    AR:if(ar_fire) begin state<=R;beat<=0;end
    R:if(r_fire) begin
     // Consume an actual terminal response even if LAST is early/late.
     // A missing LAST keeps the engine busy: no fabricated drain result.
     if(m_rlast) begin
      if(stop_pending||stop||faulted||response_error||unit_index==count-1'b1) begin
       state<=IDLE;done<=1;
       stopped<=stop_pending||stop||faulted||response_error;
      end else begin
       unit_index<=unit_index+1'b1;
       unit_hi<=next_unit_hi;next_unit_hi<=next_unit_hi+19'd1;
       mask_phase<=0;beat<=0;state<=AW;aw_done<=0;w_done<=0;
      end
     end else if(beat<31) beat<=beat+1'b1;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
