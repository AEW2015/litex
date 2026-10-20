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
 localparam [2:0] IDLE=0,AW=1,CHECK=2,B=3,AR=4,R=5;
 localparam [4:0] ADDRESS_TAG=5'd4+(CHANNEL%4);
 reg [2:0] state;
 reg [18:0] unit_hi,next_unit_hi;
 reg [4:0] beats,beat;
 reg [31:0] count,unit_index;
 // Keep these captured configuration FFs local to each engine. Otherwise
 // equivalent registers merge across ports and route into remote checkers.
 (* DONT_TOUCH = "TRUE" *) reg [31:0] seed;
 (* DONT_TOUCH = "TRUE" *) reg [31:0] strobe;
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
 // One result per clock: comparison flags and response context advance
 // together, decoupling the wide checker from telemetry register enables.
 reg checker_valid,checker_read,checker_last,checker_expected_last;
 reg [5:0] checker_id;
 reg [1:0] checker_resp;
 reg [7:0] checker_bad;
 reg [32:0] checker_addr;
 reg [31:0] checker_prefix;
 reg [255:0] checker_data;
 wire data_bad=|checker_bad;
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
 // Carry full candidate tuples through a balanced priority tree, avoiding
 // a lane-index decoder followed by another eight-way metadata mux.
 wire [66:0] candidate[0:7];
 genvar checked_lane,checked_byte;
 generate for(checked_lane=0;checked_lane<8;checked_lane=checked_lane+1) begin: checked_lanes
  localparam [2:0] LANE=checked_lane;
  wire [31:0] raw_expected=checker_prefix ^ (checked_lane*32'd4) ^
                           (checked_lane*32'h01020304);
  wire [31:0] expected;
  for(checked_byte=0;checked_byte<4;checked_byte=checked_byte+1) begin: bytes
   assign expected[checked_byte*8+:8]=raw_expected[checked_byte*8+:8] ^
          {8{masked&&strobe[checked_lane*4+checked_byte]}};
  end
  assign candidate[checked_lane]={LANE,expected,checker_data[checked_lane*32+:32]};
 end endgenerate
 wire [66:0] pair0=checker_bad[0] ? candidate[0] : candidate[1];
 wire [66:0] pair1=checker_bad[2] ? candidate[2] : candidate[3];
 wire [66:0] pair2=checker_bad[4] ? candidate[4] : candidate[5];
 wire [66:0] pair3=checker_bad[6] ? candidate[6] : candidate[7];
 wire [66:0] half0=(|checker_bad[1:0]) ? pair0 : pair1;
 wire [66:0] half1=(|checker_bad[5:4]) ? pair2 : pair3;
 wire [66:0] first_bad=(|checker_bad[3:0]) ? half0 : half1;
 wire [2:0] bad_lane=first_bad[66:64];
 wire [31:0] bad_expected=first_bad[63:32];
 wire [31:0] bad_actual=first_bad[31:0];
 wire response_error=checker_valid&&((checker_id!=6'd1)||(checker_resp!=0)||
  (checker_read&&((checker_last!=checker_expected_last)||data_bad)));
 reg [7:0] event_code;
 reg [32:0] event_addr;
 reg [31:0] event_expected,event_actual;
 always @* begin
  event_code=0;event_addr=checker_addr;event_expected=0;event_actual=0;
  if(checker_valid) begin
   if(!checker_read) begin
    if(checker_id!=6'd1) begin event_code=1;event_expected=1;event_actual={26'b0,checker_id};end
    else if(checker_resp!=0) begin event_code=2;event_actual={30'b0,checker_resp};end
   end else begin
    if(checker_id!=6'd1) begin event_code=3;event_expected=1;event_actual={26'b0,checker_id};end
    else if(checker_resp!=0) begin event_code=4;event_actual={30'b0,checker_resp};end
    else if(checker_last!=checker_expected_last) begin
     event_code=5;event_expected=checker_expected_last;event_actual=checker_last;
    end else if(data_bad) begin
     event_code=6;event_addr={checker_addr[32:5],bad_lane,2'b00};
     event_expected=bad_expected;event_actual=bad_actual;
    end
   end
  end
 end
 always @(posedge clk) begin
  if(rst) begin
   state<=IDLE;done<=0;stopped<=0;write_beats<=0;read_beats<=0;cycles<=0;errors<=0;faulted<=0;
   first_bad_addr<=0;first_bad_expected<=0;first_bad_actual<=0;first_bad_code<=0;
   unit_hi<=0;next_unit_hi<=0;beats<=0;count<=0;seed<=0;strobe<=0;stall<=0;masked<=0;
   beat<=0;unit_index<=0;mask_phase<=0;stop_pending<=0;w_held<=0;aw_done<=0;w_done<=0;
   checker_valid<=0;checker_read<=0;checker_last<=0;checker_expected_last<=0;
   checker_id<=0;checker_resp<=0;checker_bad<=0;checker_addr<=0;checker_prefix<=0;checker_data<=0;
  end else if(start) begin
   // Hub sends START to all ports: unselected counters clear without traffic.
   state<=enable ? AW : IDLE;done<=0;stopped<=0;
   write_beats<=0;read_beats<=0;cycles<=0;errors<=0;faulted<=0;
   first_bad_addr<=0;first_bad_expected<=0;first_bad_actual<=0;first_bad_code<=0;
   unit_hi<=cfg_base[27:9];next_unit_hi<=cfg_base[27:9]+19'd1;beats<=cfg_beats;count<=cfg_count;seed<=cfg_seed;
   strobe<=cfg_strobe;stall<=cfg_stall;masked<=cfg_masked;
   beat<=0;unit_index<=0;mask_phase<=0;stop_pending<=0;w_held<=0;aw_done<=0;w_done<=0;
   checker_valid<=0;checker_read<=0;checker_last<=0;checker_expected_last<=0;
   checker_id<=0;checker_resp<=0;checker_bad<=0;checker_addr<=0;checker_prefix<=0;checker_data<=0;
  end else begin
   checker_valid<=b_fire||r_fire;
   if(b_fire) begin
    checker_read<=0;checker_id<=m_bid;checker_resp<=m_bresp;
    checker_last<=0;checker_expected_last<=0;checker_bad<=0;checker_addr<=full_beat_addr;
   end
   if(r_fire) begin
    checker_read<=1;checker_id<=m_rid;checker_resp<=m_rresp;
    checker_last<=m_rlast;checker_expected_last<=beat==beats-1'b1;
    checker_bad<=lane_bad;checker_addr<=full_beat_addr;
    checker_prefix<=pattern(beat_addr,0,seed);checker_data<=m_rdata;
   end
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
     // Missing LAST never fabricates drain. Remain busy until the actual
     // terminal response's registered checker result commits.
     if(m_rlast) state<=CHECK;
     else if(beat<31) beat<=beat+1'b1;
    end
    CHECK:if(checker_valid&&checker_read&&checker_last) begin
     // This terminal result commits on the same edge as completion. Include
     // its error directly: faulted still holds the preceding beat's value.
     if(stop_pending||stop||faulted||response_error||unit_index==count-1'b1) begin
      state<=IDLE;done<=1;
      stopped<=stop_pending||stop||faulted||response_error;
     end else begin
      unit_index<=unit_index+1'b1;
      unit_hi<=next_unit_hi;next_unit_hi<=next_unit_hi+19'd1;
      mask_phase<=0;beat<=0;state<=AW;aw_done<=0;w_done<=0;
     end
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
