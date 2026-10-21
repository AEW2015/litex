// CSR bank 6 (byte base 0x3000), SYS250 to APB100 bundled-data mailbox.
// One outstanding request. Request/response bundles remain stable until handshake.
`timescale 1ns/1ps
module fk33_hbm_control #(
    parameter integer APB_TIMEOUT = 100000,
    parameter [31:0] TEMPERATURE_MAX_AGE = 32'd3750000000,
    parameter integer SYS_TIMEOUT = APB_TIMEOUT * 8 + 64
)(
    input wire sys_clk, sys_rst, apb_clk, apb_rst,
    input wire [13:0] csr_adr,
    input wire [31:0] csr_dat_w,
    input wire csr_we,
    output reg [31:0] csr_dat_r,
    input wire [1:0] trip,
    input wire [2:0] raw_temp_0, raw_temp_1,
    output reg [1:0] sw_ready,
    output reg [6:0] sw_temperature_0, sw_temperature_1,
    output reg [1:0] sw_temperature_valid,
    output wire [21:0] paddr,
    output wire [31:0] pwdata,
    output wire pwrite,
    output wire [1:0] psel,
    output wire penable,
    input wire [31:0] prdata_0, prdata_1,
    input wire [1:0] pready, pslverr
);
    wire selected = csr_adr[13:9] == 5'd6;
    wire [8:0] index = csr_adr[8:0];
    // Per-bit diagnostic sampling only: these codes are not Celsius and do
    // not constitute an atomic multi-bit temperature snapshot. Full valid
    // Celsius values must come from the software APB temperature sequence.
    (* ASYNC_REG="TRUE" *) reg [2:0] raw_temperature_0_sync_0, raw_temperature_0_sync_1;
    (* ASYNC_REG="TRUE" *) reg [2:0] raw_temperature_1_sync_0, raw_temperature_1_sync_1;
    // HBM status outputs launch on PCLK. Capture beside the SYS sampling
    // chain so the fixed hard-macro location is not a short APB-to-SYS path.
    // The primitive's direct asynchronous CATTRIP traffic gate is unchanged.
    reg [1:0] trip_apb;
    reg [2:0] raw_temperature_0_apb, raw_temperature_1_apb;
    reg [21:0] address;
    reg [31:0] write_data;
    // Either reset aborts the entire mailbox; independently synchronized release
    // prevents a one-domain reset from replaying a side-effecting APB request.
    reg mailbox_fault;
    wire reset_all = sys_rst | apb_rst;
    wire apb_abort_reset = reset_all | mailbox_fault;
    // Match Migen's existing asynchronous-reset synchronizer constraints:
    // cut only these six entry PRE pins and bound each stage0-to-stage1 path.
    // Functional reset destinations remain subject to recovery/removal timing.
    (* ASYNC_REG="TRUE", ars_ff1="TRUE" *) reg sys_reset_sync_stage0;
    (* ASYNC_REG="TRUE", ars_ff2="TRUE" *) reg sys_reset_sync_stage1;
    (* ASYNC_REG="TRUE", ars_ff1="TRUE" *) reg apb_reset_sync_stage0;
    (* ASYNC_REG="TRUE", ars_ff2="TRUE" *) reg apb_reset_sync_stage1;
    // Diagnostic sampling is reset by either domain reset, never mailbox_fault.
    (* ASYNC_REG="TRUE", ars_ff1="TRUE" *) reg apb_monitor_reset_sync_stage0;
    (* ASYNC_REG="TRUE", ars_ff2="TRUE" *) reg apb_monitor_reset_sync_stage1;
    wire [1:0] sys_reset_sync = {sys_reset_sync_stage1,sys_reset_sync_stage0};
    wire [1:0] apb_reset_sync = {apb_reset_sync_stage1,apb_reset_sync_stage0};
    wire [1:0] apb_monitor_reset_sync = {apb_monitor_reset_sync_stage1,apb_monitor_reset_sync_stage0};
    always @(posedge sys_clk or posedge reset_all)
        if (reset_all) begin
            sys_reset_sync_stage0 <= 1'b1;
            sys_reset_sync_stage1 <= 1'b1;
        end else begin
            sys_reset_sync_stage0 <= 1'b0;
            sys_reset_sync_stage1 <= sys_reset_sync_stage0;
        end
    always @(posedge apb_clk or posedge apb_abort_reset)
        if (apb_abort_reset) begin
            apb_reset_sync_stage0 <= 1'b1;
            apb_reset_sync_stage1 <= 1'b1;
        end else begin
            apb_reset_sync_stage0 <= 1'b0;
            apb_reset_sync_stage1 <= apb_reset_sync_stage0;
        end
    always @(posedge apb_clk or posedge reset_all)
        if (reset_all) begin
            apb_monitor_reset_sync_stage0 <= 1'b1;
            apb_monitor_reset_sync_stage1 <= 1'b1;
        end else begin
            apb_monitor_reset_sync_stage0 <= 1'b0;
            apb_monitor_reset_sync_stage1 <= apb_monitor_reset_sync_stage0;
        end
    reg request_toggle, busy, done, error_flag, timeout_flag;
    reg inject_armed, request_inject, transfer_inject;
    reg [31:0] sys_timeout_count;
    reg [31:0] temp_age_0, temp_age_1;
    wire temperature_expired_0 = sw_temperature_valid[0] && temp_age_0 >= TEMPERATURE_MAX_AGE;
    wire temperature_expired_1 = sw_temperature_valid[1] && temp_age_1 >= TEMPERATURE_MAX_AGE;
    wire command_conflict = |csr_dat_w[31:6] ||
        (({2'b00,csr_dat_w[0]} + {2'b00,csr_dat_w[3]} +
          {2'b00,csr_dat_w[4]} + {2'b00,csr_dat_w[5]}) > 1) ||
        (!csr_dat_w[0] && |csr_dat_w[2:1]);
    reg [21:0] request_address;
    reg [31:0] request_data;
    reg request_write, request_stack;
    reg [31:0] read_data;
    (* ASYNC_REG="TRUE" *) reg [1:0] trip_sync_0, trip_sync_1;
    (* ASYNC_REG="TRUE" *) reg response_sync_0, response_sync_1;
    reg response_seen;
    reg [31:0] response_data;
    reg response_error, response_timeout, response_toggle;
    (* ASYNC_REG="TRUE" *) reg request_sync_0, request_sync_1;
    reg request_seen;
    reg [1:0] state;
    reg [21:0] transfer_address;
    reg [31:0] transfer_data;
    reg transfer_write, transfer_stack;
    reg [31:0] timeout_count;

    always @(posedge apb_clk or posedge apb_monitor_reset_sync[1]) begin
        if (apb_monitor_reset_sync[1]) begin
            trip_apb <= 0;
            raw_temperature_0_apb <= 0;
            raw_temperature_1_apb <= 0;
        end else begin
            trip_apb <= trip;
            raw_temperature_0_apb <= raw_temp_0;
            raw_temperature_1_apb <= raw_temp_1;
        end
    end

    assign paddr = transfer_address;
    assign pwdata = transfer_data;
    assign pwrite = transfer_write;
    assign psel = state == 0 || transfer_inject ? 2'b00 : (transfer_stack ? 2'b10 : 2'b01);
    assign penable = state == 2 && !transfer_inject;
    wire selected_ready = transfer_stack ? pready[1] : pready[0];
    wire selected_error = transfer_stack ? pslverr[1] : pslverr[0];
    wire [31:0] selected_data = transfer_stack ? prdata_1 : prdata_0;

    reg [31:0] csr_read_mux;
    always @* begin
        csr_read_mux = 0;
        if (selected) case (index)
            0: csr_read_mux = 32'h4c48424d;
            1: csr_read_mux = 1;
            2: csr_read_mux = {10'b0,address};
            3: csr_read_mux = write_data;
            5: csr_read_mux = {14'b0,trip_sync_1,6'b0,sw_ready,2'b0,inject_armed,mailbox_fault,timeout_flag,error_flag,done,busy};
            6: csr_read_mux = read_data;
            7: csr_read_mux = {10'b0,request_address};
            8: csr_read_mux = {23'b0,sw_temperature_valid[0],1'b0,sw_temperature_0};
            9: csr_read_mux = {23'b0,sw_temperature_valid[1],1'b0,sw_temperature_1};
            10: csr_read_mux = {21'b0,raw_temperature_1_sync_1,5'b0,raw_temperature_0_sync_1};
            default: csr_read_mux = 0;
        endcase
    end

    // LiteX CSR bank data is registered in the request cycle. The bridge
    // drops the address before the following Wishbone response cycle.
    always @(posedge sys_clk) csr_dat_r <= csr_read_mux;

    always @(posedge sys_clk or posedge sys_reset_sync[1]) begin
        if (sys_reset_sync[1]) begin
            raw_temperature_0_sync_0<=0; raw_temperature_0_sync_1<=0;
            raw_temperature_1_sync_0<=0; raw_temperature_1_sync_1<=0;
            address<=0; write_data<=0; request_toggle<=0; busy<=0;
            done<=0; error_flag<=0; timeout_flag<=0; read_data<=0;
            mailbox_fault<=0; inject_armed<=0; request_inject<=0; sys_timeout_count<=0;
            request_address<=0; request_data<=0; request_write<=0; request_stack<=0;
            response_sync_0<=0; response_sync_1<=0; response_seen<=0;
            trip_sync_0<=0; trip_sync_1<=0;
            sw_ready<=0; sw_temperature_0<=0; sw_temperature_1<=0;
            sw_temperature_valid<=0; temp_age_0<=0; temp_age_1<=0;
        end else begin
            raw_temperature_0_sync_0<=raw_temperature_0_apb; raw_temperature_0_sync_1<=raw_temperature_0_sync_0;
            raw_temperature_1_sync_0<=raw_temperature_1_apb; raw_temperature_1_sync_1<=raw_temperature_1_sync_0;
            response_sync_0<=response_toggle; response_sync_1<=response_sync_0;
            trip_sync_0<=trip_apb; trip_sync_1<=trip_sync_0;
            if (sw_temperature_valid[0]) begin
                if (temp_age_0 >= TEMPERATURE_MAX_AGE) begin
                    sw_temperature_valid[0]<=0; sw_ready<=0;
                end else temp_age_0<=temp_age_0+1;
            end
            if (sw_temperature_valid[1]) begin
                if (temp_age_1 >= TEMPERATURE_MAX_AGE) begin
                    sw_temperature_valid[1]<=0; sw_ready<=0;
                end else temp_age_1<=temp_age_1+1;
            end
            if (response_sync_1 != response_seen) begin
                response_seen<=response_sync_1;
                if (busy && !mailbox_fault) begin
                    read_data<=response_data; busy<=0; done<=1;
                    error_flag<=error_flag | response_error;
                    timeout_flag<=timeout_flag | response_timeout;
                    if (response_error || response_timeout) sw_ready<=0;
                end
            end
            if (busy) begin
                if (sys_timeout_count >= SYS_TIMEOUT-1) begin
                    // Lost APB clock/response: retire the host operation but do
                    // not permit another request until reset re-baselines CDC.
                    mailbox_fault<=1; busy<=0; done<=1;
                    error_flag<=1; timeout_flag<=1; sw_ready<=0;
                end else sys_timeout_count<=sys_timeout_count+1;
            end
            if (selected && csr_we) case (index)
                2: address<=csr_dat_w[21:0];
                3: write_data<=csr_dat_w;
                4: if (command_conflict) begin error_flag<=1; sw_ready<=0; end
                else begin
                    if (csr_dat_w[3] && !busy && !mailbox_fault) begin done<=0; error_flag<=0; timeout_flag<=0; end
                    if (csr_dat_w[4]) begin sw_ready<=0; sw_temperature_valid<=0; end
                    if (csr_dat_w[5]) begin
                        if (!busy && !mailbox_fault && !error_flag && !timeout_flag && (&sw_temperature_valid) && !(|trip_sync_1))
                            sw_ready<=3;
                        else begin error_flag<=1; sw_ready<=0; end
                    end
                    if (csr_dat_w[0]) begin
                        if (busy || mailbox_fault) begin error_flag<=1; sw_ready<=0; end
                        else begin
                            request_address<=address; request_data<=write_data;
                            request_write<=csr_dat_w[1]; request_stack<=csr_dat_w[2];
                            request_toggle<=~request_toggle;
                            request_inject<=inject_armed; inject_armed<=0;
                            sys_timeout_count<=0;
                            busy<=1; done<=0; error_flag<=0; timeout_flag<=0;
                        end
                    end
                end
                8: begin
                    sw_temperature_0<=csr_dat_w[6:0]; temp_age_0<=0;
                    sw_temperature_valid[0]<=csr_dat_w[8] && csr_dat_w[6:0]>0 && csr_dat_w[6:0]<60;
                    if (!csr_dat_w[8] || csr_dat_w[6:0]==0 || csr_dat_w[6:0]>=60) sw_ready<=0;
                end
                9: begin
                    sw_temperature_1<=csr_dat_w[6:0]; temp_age_1<=0;
                    sw_temperature_valid[1]<=csr_dat_w[8] && csr_dat_w[6:0]>0 && csr_dat_w[6:0]<60;
                    if (!csr_dat_w[8] || csr_dat_w[6:0]==0 || csr_dat_w[6:0]>=60) sw_ready<=0;
                end
                11: begin
                    if (csr_dat_w == 32'h54494d45 && !busy && !mailbox_fault) inject_armed<=1;
                    else begin error_flag<=1; sw_ready<=0; end
                end
            endcase
            // Expiration wins over a same-cycle COMMIT of an old shadow.
            if (temperature_expired_0) begin sw_temperature_valid[0]<=0; sw_ready<=0; end
            if (temperature_expired_1) begin sw_temperature_valid[1]<=0; sw_ready<=0; end
            // Highest priority: a physical trip always withdraws READY.
            if (|trip_sync_1) begin sw_ready<=0; error_flag<=1; end
        end
    end

    always @(posedge apb_clk or posedge apb_reset_sync[1]) begin
        if (apb_reset_sync[1]) begin
            request_sync_0<=0; request_sync_1<=0; request_seen<=0;
            state<=0; transfer_address<=0; transfer_data<=0;
            transfer_write<=0; transfer_stack<=0; transfer_inject<=0; timeout_count<=0;
            response_data<=0; response_error<=0; response_timeout<=0; response_toggle<=0;
        end else begin
            request_sync_0<=request_toggle; request_sync_1<=request_sync_0;
            case (state)
                0: if (request_sync_1 != request_seen) begin
                    request_seen<=request_sync_1;
                    transfer_address<=request_address; transfer_data<=request_data;
                    transfer_write<=request_write; transfer_stack<=request_stack;
                    transfer_inject<=request_inject;
                    timeout_count<=0; response_error<=0; response_timeout<=0;
                    state<=1;
                end
                1: state<=2;
                2: begin
                    if (selected_ready && !transfer_inject) begin
                        response_data<=selected_data; response_error<=selected_error;
                        response_toggle<=~response_toggle; state<=0;
                    end else if (timeout_count >= APB_TIMEOUT-1) begin
                        response_data<=32'hffffffff; response_error<=1; response_timeout<=1;
                        response_toggle<=~response_toggle; state<=0;
                    end else timeout_count<=timeout_count+1;
                end
                default: state<=0;
            endcase
        end
    end
endmodule
