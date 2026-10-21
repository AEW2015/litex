// Admission/drain tracking for the FK33 AXI3 subset: ordered write data,
// no WID interleaving, up to 16 beats per burst. Payloads bypass this module.
// Never drives AXI reset, HBM READY, APB or clocks. A missing response/last
// beat prevents drain; software must retain quiesce and report a deadline.
`timescale 1ns/1ps
module fk33_hbm_quiesce #(
    parameter integer MAX_OUTSTANDING = 8,
    parameter integer REGISTERED_COMPLETIONS = 0
)(
    input wire clk, rst, request,
    input wire s_arvalid, input wire [7:0] s_arlen,
    output wire s_arready, output wire m_arvalid, input wire m_arready,
    input wire s_awvalid, input wire [7:0] s_awlen,
    output wire s_awready, output wire m_awvalid, input wire m_awready,
    input wire s_wvalid, s_wlast,
    output wire s_wready, output wire m_wvalid, input wire m_wready,
    input wire m_rvalid, m_rlast, s_rready,
    input wire m_bvalid, s_bready,
    output wire drained,
    output reg error
);
    localparam integer CW = $clog2(MAX_OUTSTANDING+1);
    reg [CW-1:0] reads, addresses, writes_started, writes_completed;
    reg ar_held, aw_held, w_held, w_active;
    wire stop = request | error;

    // An already presented/stalled VALID must survive a new request. For
    // writes, either half admitted/offered before STOP owns the matching
    // other half. Continuing W beats finish the owned burst through WLAST.
    wire ar_allow = ar_held | (!stop && reads < MAX_OUTSTANDING);
    wire aw_debt = (writes_started > addresses) |
        (w_held && !w_active && writes_started == addresses);
    wire w_debt = (addresses > writes_started) |
        (aw_held && addresses == writes_started);
    wire aw_allow = aw_held | ((addresses < MAX_OUTSTANDING) && (!stop || aw_debt));
    wire w_allow = w_active | w_held |
        ((writes_started < MAX_OUTSTANDING) && (!stop || w_debt));
    assign m_arvalid = s_arvalid & ar_allow;
    assign s_arready = m_arready & ar_allow;
    assign m_awvalid = s_awvalid & aw_allow;
    assign s_awready = m_awready & aw_allow;
    assign m_wvalid = s_wvalid & w_allow;
    assign s_wready = m_wready & w_allow;

    wire ar_fire = m_arvalid & m_arready;
    wire aw_fire = m_awvalid & m_awready;
    wire w_fire = m_wvalid & m_wready;
    wire w_start = w_fire & ~w_active;
    wire w_finish = w_fire & s_wlast;
    wire r_fire = m_rvalid & s_rready;
    wire r_finish = r_fire & m_rlast;
    wire b_fire = m_bvalid & s_bready;
    // Optional timing cut on accepted response completion to counter enables.
    // Payload/READY bypass is unchanged. Counters include any pending retire;
    // protocol checks subtract that already accepted completion immediately.
    reg r_pending, b_pending;
    wire r_retire = REGISTERED_COMPLETIONS ? r_pending : r_finish;
    wire b_retire = REGISTERED_COMPLETIONS ? b_pending : b_fire;
    wire no_read_available = reads == 0 ||
        (REGISTERED_COMPLETIONS && r_pending && reads == 1);
    wire no_address_available = addresses == 0 ||
        (REGISTERED_COMPLETIONS && b_pending && addresses == 1);
    wire no_write_available = writes_completed == 0 ||
        (REGISTERED_COMPLETIONS && b_pending && writes_completed == 1);
    assign drained = !error && reads == 0 && addresses == 0 &&
        writes_started == 0 && writes_completed == 0 &&
        !(REGISTERED_COMPLETIONS && (r_pending || b_pending)) &&
        !w_active && !ar_held && !aw_held && !w_held;

    always @(posedge clk) begin
        if (rst) begin
            reads<=0; addresses<=0; writes_started<=0; writes_completed<=0;
            ar_held<=0; aw_held<=0; w_held<=0; w_active<=0; error<=0;
            r_pending<=0; b_pending<=0;
        end else begin
            r_pending<=r_finish; b_pending<=b_fire;
            if (m_arvalid && !m_arready) ar_held<=1;
            else if (ar_fire) ar_held<=0;
            if (m_awvalid && !m_awready) aw_held<=1;
            else if (aw_fire) aw_held<=0;
            if (m_wvalid && !m_wready) w_held<=1;
            else if (w_fire) w_held<=0;
            if (w_fire) w_active<=!s_wlast;

            // Counters count transactions, not beats. Completion requires
            // the host to accept the final R beat or B response.
            case ({ar_fire,r_retire})
                2'b10: reads<=reads+1'b1;
                2'b01: if (reads!=0) reads<=reads-1'b1;
            endcase
            case ({aw_fire,b_retire})
                2'b10: addresses<=addresses+1'b1;
                2'b01: if (addresses!=0) addresses<=addresses-1'b1;
            endcase
            case ({w_start,b_retire})
                2'b10: writes_started<=writes_started+1'b1;
                2'b01: if (writes_started!=0) writes_started<=writes_started-1'b1;
            endcase
            case ({w_finish,b_retire})
                2'b10: writes_completed<=writes_completed+1'b1;
                2'b01: if (writes_completed!=0) writes_completed<=writes_completed-1'b1;
            endcase

            // Sticky accounting/protocol fault: no invented drain result.
            // It requires a destructive system reset, not request release.
            if ((ar_held && !s_arvalid) || (aw_held && !s_awvalid) ||
                (w_held && !s_wvalid) ||
                (ar_fire && |s_arlen[7:4]) || (aw_fire && |s_awlen[7:4]) ||
                (r_fire && no_read_available && !ar_fire) ||
                (b_fire && ((no_address_available && !aw_fire) ||
                            (no_write_available && !w_finish)))) error<=1;
        end
    end
endmodule
