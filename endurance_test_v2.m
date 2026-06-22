function endurance_test_v2()
% =========================================================================
%  TEST 3 — ENDURANCE / FATIGUE TEST
%  Capstan Drive Reliability Project
%
%  Protocol (from methodology doc):
%    - Motion: continuous back-and-forth (0 to 70 deg output) in position mode
%    - Load:   50–60% of rated output torque (dead load, applied by operator)
%    - Log:    one CSV row per cycle (mean TE, mean current, temperature, failure tier)
%    - Inspect every 500 cycles: auto-pause, operator scores rope, then resume
%    - Stop:   Critical failure (tier 3) OR 10,000 cycles completed
%    - E-STOP: click green figure window, press SPACE
%
%  Failure tiers (auto-computed each cycle):
%    0 = Nominal
%    1 = Minimal  — mean TE drift > ±1 deg above baseline, OR current 5–10% above baseline
%    2 = Intermediate — mean TE > ±3 deg across last 10 cycles, OR current > 20% above baseline,
%                       OR slip on >3 of last 10 cycles
%    3 = Critical — any single TE > ±15 deg (catastrophic slip), OR current > rated limit
%
%  CSV columns (one row per cycle):
%    CYCLE, TIMESTAMP_S, CMD_POS_DEG, CMD_NEG_DEG,
%    MEAN_TRANS_ERR_DEG, STD_TRANS_ERR_DEG,
%    SLIP_FLAG, MEAN_CURRENT_mA, FAILURE_TIER, TEMPERATURE_C
% =========================================================================

clc;
clear;
warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% ================================================================
%% TUNING PARAMETERS
%% ================================================================

MAX_CYCLES          = 6000;     % Stop condition: censored observation
INSPECTION_INTERVAL = 2000;     % Cycles between mandatory operator inspections
LOAD_PCT            = 55;       % Nominal load level (50–60% rated torque, for labelling)

TEST3_RANGE_DEG     = 70;       % Total output swing (±35 deg from midpoint)
PROFILE_VELOCITY    = 2600;     % Dynamixel profile velocity (0.01 rev/min\   units)
POSITION_TOLERANCE  = 50;       % Counts — how close Dxl must be before logging
SETTLE_AT_TARGET    = 0.05;    % Seconds to wait stationary at each endpoint before logging
SAMPLE_PERIOD       = 0.05;     % Polling period during motion (seconds)

% Failure tier thresholds (methodology doc §2)
SLIP_THRESHOLD_DEG        = 15.0;   % Tier 3: catastrophic slip — single TE exceeding this
CURRENT_RATED_LIMIT_mA    = 1750;   % Tier 3: Dynamixel PM42 rated current limit (mA)
TIER2_TE_DEG              = 3.0;    % Tier 2: mean TE over last 10 cycles
TIER2_CURRENT_PCT         = 0.20;   % Tier 2: current > 20% above baseline
TIER2_SLIP_IN_10          = 3;      % Tier 2: slip events in last 10 cycles
TIER1_TE_DEG              = 1.0;    % Tier 1: mean TE drift above baseline
TIER1_CURRENT_PCT_LOW     = 0.05;   % Tier 1: current 5% above baseline (lower edge)
TIER1_CURRENT_PCT_HIGH    = 0.10;   % Tier 1: current 10% above baseline (upper edge)

%% ================================================================
%% E-STOP FIGURE
%% ================================================================

fig = figure('Name','>>> ENDURANCE TEST v1 | Click here then press SPACE to E-STOP <<<', ...
             'Color',[0.1 0.7 0.1], ...
             'NumberTitle','off', ...
             'MenuBar','none', ...
             'ToolBar','none', ...
             'KeyPressFcn',@(src,evt) setappdata(src,'stop_key',evt.Key));
setappdata(fig,'stop_key','');
drawnow;
fprintf('=== E-STOP READY — click the green figure window, then press SPACE to stop ===\n\n');

%% ================================================================
%% ENCODER SETUP
%% ================================================================

encoder = serialport("COM7", 115200);
flush(encoder);
fprintf('Encoder serial port opened (COM7)\n\n');

%% ================================================================
%% DYNAMIXEL SETUP
%% ================================================================

lib_name = '';
if strcmp(computer,'PCWIN'),       lib_name = 'dxl_x86_c';
elseif strcmp(computer,'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer,'GLNX86'),  lib_name = 'libdxl_x86_c';
elseif strcmp(computer,'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer,'MACI64'),  lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    loadlibrary(lib_name,'dynamixel_sdk.h', ...
        'addheader','port_handler.h', ...
        'addheader','packet_handler.h');
end

ADDR_TORQUE_ENABLE       = 512;
ADDR_OPERATING_MODE      = 11;
ADDR_GOAL_VELOCITY       = 552;
ADDR_PROFILE_VELOCITY    = 560;
ADDR_PRESENT_POSITION    = 580;
ADDR_PRESENT_CURRENT     = 574;
ADDR_GOAL_POSITION       = 564;
ADDR_PRESENT_TEMPERATURE = 594;   % 1-byte, degrees Celsius

PROTOCOL_VERSION = 2.0;
DXL_ID           = 1;
DEVICENAME       = 'COM8';
BAUDRATE         = 57600;
TORQUE_ENABLE    = 1;
TORQUE_DISABLE   = 0;

port_num = portHandler(DEVICENAME);
packetHandler();

if openPort(port_num)
    fprintf('Port opened (COM8)\n');
else
    emergency_shutdown(fig, port_num, lib_name, 0, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    error('Failed to open port');
end

if setBaudRate(port_num, BAUDRATE)
    fprintf('Baudrate set\n\n');
else
    emergency_shutdown(fig, port_num, lib_name, 0, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    error('Failed to set baudrate');
end

%% ================================================================
%% PHASE 1 — FIND LIMITS (1 TIME ONLY)
%% ================================================================

fprintf('==========================================\n');
fprintf('  PHASE 1: Limit Search (1 run)\n');
fprintf('==========================================\n\n');

[limit_CCW, limit_CW, limit_CCW_enc, limit_CW_enc] = find_capstan_limits(fig, encoder, port_num, ...
    PROTOCOL_VERSION, DXL_ID, lib_name, ...
    ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE, ADDR_GOAL_VELOCITY, ...
    ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ADDR_GOAL_POSITION, ...
    TORQUE_ENABLE, TORQUE_DISABLE);

if isnan(limit_CCW) || isnan(limit_CW)
    fprintf('[ERROR] Limit search failed — aborting.\n');
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end

%% ================================================================
%% COMPUTE GEAR RATIO AND MIDPOINT
%% ================================================================

range_dxl  = abs(limit_CW     - limit_CCW);
range_enc  = abs(limit_CW_enc - limit_CCW_enc);
gear_ratio = range_dxl / range_enc;
mid_dxl    = (limit_CCW     + limit_CW)     / 2;
mid_enc    = (limit_CCW_enc + limit_CW_enc) / 2;

fprintf('==========================================\n');
fprintf('  LIMIT SEARCH RESULT\n');
fprintf('  Dynamixel | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW, limit_CW, range_dxl);
fprintf('  Encoder   | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW_enc, limit_CW_enc, range_enc);
fprintf('  Gear ratio (Dxl / Enc): %.4f\n', gear_ratio);
fprintf('  Midpoint  | Dxl: %+.3f  Enc: %+.3f\n', mid_dxl, mid_enc);
fprintf('==========================================\n\n');

%% Verify ±half_range on encoder fits within physical limits
half_range = TEST3_RANGE_DEG / 2;
if (mid_enc - half_range) < min(limit_CCW_enc, limit_CW_enc) || ...
   (mid_enc + half_range) > max(limit_CCW_enc, limit_CW_enc)
    fprintf('[WARNING] Requested ±%.0f deg output range exceeds physical limits. Aborting.\n', half_range);
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end
fprintf('  Range check passed: ±%.0f deg output fits within physical limits.\n\n', half_range);

%% ================================================================
%% MOVE TO MIDPOINT AND ZERO SENSORS
%% ================================================================

fprintf('Entering extended position mode and moving to midpoint...\n');

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, ...
               typecast(int32(PROFILE_VELOCITY), 'uint32'));

mid_counts = int32(mid_dxl / 0.00068392);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
               typecast(mid_counts, 'uint32'));

ok = wait_for_position(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_PRESENT_POSITION, mid_counts, POSITION_TOLERANCE, 20);
if ~ok
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end
pause(0.3);
fprintf('At midpoint.\n\n');

%% Zero both sensors
raw_pos_zero    = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
dxl_zero_counts = typecast(uint32(raw_pos_zero), 'int32');
dxl_zero_deg    = double(dxl_zero_counts) * 0.00068392;

flush(encoder);
pause(0.2);
enc_zero_deg = read_encoder(encoder);

fprintf('Zeroing complete.\n');
fprintf('  Dxl zero reference: %.4f deg\n', dxl_zero_deg);
fprintf('  Enc zero reference: %.4f deg\n\n', enc_zero_deg);

%% Pre-compute Dynamixel target counts for +half_range and -half_range output
target_dxl_pos_deg = +half_range * gear_ratio;
target_dxl_neg_deg = -half_range * gear_ratio;
target_pos_counts  = int32((dxl_zero_deg + target_dxl_pos_deg) / 0.00068392);
target_neg_counts  = int32((dxl_zero_deg + target_dxl_neg_deg) / 0.00068392);

fprintf('  Dynamixel endpoint counts:\n');
fprintf('    +%.0f deg enc -> %+.2f deg Dxl -> counts %d\n', half_range, target_dxl_pos_deg, target_pos_counts);
fprintf('    -%.0f deg enc -> %+.2f deg Dxl -> counts %d\n', half_range, target_dxl_neg_deg, target_neg_counts);
fprintf('\n');

%% ================================================================
%% OPERATOR: APPLY LOAD AND CONFIRM PRETENSION
%% ================================================================

fprintf('==========================================\n');
fprintf('  PRE-TEST CHECKLIST\n');
fprintf('  1. Verify rope pretension (transverse deflection method)\n');
fprintf('  2. Record pretension value in physical logbook\n');
fprintf('  3. Apply %d%% rated torque dead load weight\n', LOAD_PCT);
fprintf('  4. Photograph rope contact zone -> save as endurance_baseline.jpg\n');
fprintf('==========================================\n');
fprintf('  >>> Press Enter when load is applied and checklist is complete...\n');
input('', 's');

if estop_pressed(fig)
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end

%% ================================================================
%% BASELINE CHARACTERIZATION — 20 WARM-UP CYCLES TO ESTABLISH REFERENCE
%% ================================================================

fprintf('\n==========================================\n');
fprintf('  BASELINE: 20 warm-up cycles (no failure tier active)\n');
fprintf('==========================================\n\n');

baseline_te      = [];
baseline_current = [];

for cyc = 1:20
    [te_pos, te_neg, curr_pos, curr_neg, slip_pos, slip_neg, ~, ok] = run_one_cycle( ...
        fig, port_num, encoder, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ADDR_GOAL_POSITION, ...
        ADDR_PRESENT_TEMPERATURE, ...
        target_pos_counts, target_neg_counts, mid_counts, ...
        POSITION_TOLERANCE, SETTLE_AT_TARGET, SAMPLE_PERIOD, ...
        dxl_zero_deg, enc_zero_deg, gear_ratio, half_range);

    if ~ok
        emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ...
            ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
        return;
    end

    baseline_te(end+1)      = mean([te_pos, te_neg]); %#ok<AGROW>
    baseline_current(end+1) = mean([abs(curr_pos), abs(curr_neg)]); %#ok<AGROW>
    fprintf('  Baseline cy %2d | TE pos:%+6.3f neg:%+6.3f  I pos:%4.0f neg:%4.0f mA\n', ...
            cyc, te_pos, te_neg, curr_pos, curr_neg);
end

baseline_mean_te      = mean(baseline_te);
baseline_mean_current = mean(baseline_current);

fprintf('\n  Baseline established:\n');
fprintf('    Mean TE      = %+.4f deg\n', baseline_mean_te);
fprintf('    Mean current = %.1f mA\n\n', baseline_mean_current);

%% ================================================================
%% PRE-ALLOCATE LOG ARRAYS
%% ================================================================

log_cycle         = zeros(1, MAX_CYCLES);
log_timestamp     = zeros(1, MAX_CYCLES);
log_cmd_pos       = zeros(1, MAX_CYCLES);
log_cmd_neg       = zeros(1, MAX_CYCLES);
log_mean_dxl      = zeros(1, MAX_CYCLES);
log_mean_enc      = zeros(1, MAX_CYCLES);
log_mean_te       = zeros(1, MAX_CYCLES);
log_std_te        = zeros(1, MAX_CYCLES);
log_slip_flag     = zeros(1, MAX_CYCLES);
log_mean_current  = zeros(1, MAX_CYCLES);
log_failure_tier  = zeros(1, MAX_CYCLES);
log_temperature_C = zeros(1, MAX_CYCLES);   % <-- temperature log

%% Rolling window for tier assessment
recent_te         = zeros(1, 10);
recent_slip       = zeros(1, 10);
recent_idx        = 0;

stop_reason = 'completed';
t0 = tic;

%% ================================================================
%% PHASE 2 — ENDURANCE RUN
%% ================================================================

fprintf('==========================================\n');
fprintf('  PHASE 2: Endurance Run — up to %d cycles\n', MAX_CYCLES);
fprintf('  Load: %d%% rated torque | Motion: ±%.0f deg output\n', LOAD_PCT, half_range);
fprintf('  Inspection every %d cycles | E-STOP: SPACE\n', INSPECTION_INTERVAL);
fprintf('==========================================\n\n');

k = 0;

for cycle = 1:MAX_CYCLES

    %% ---- Run one back-and-forth cycle ----
    [te_pos, te_neg, curr_pos, curr_neg, slip_pos, slip_neg, cycle_temp_C, ok] = run_one_cycle( ...
        fig, port_num, encoder, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ADDR_GOAL_POSITION, ...
        ADDR_PRESENT_TEMPERATURE, ...
        target_pos_counts, target_neg_counts, mid_counts, ...
        POSITION_TOLERANCE, SETTLE_AT_TARGET, SAMPLE_PERIOD, ...
        dxl_zero_deg, enc_zero_deg, gear_ratio, half_range);

    if ~ok
        stop_reason = 'estop';
        break;
    end

    k = k + 1;

    cycle_mean_te      = mean([te_pos, te_neg]);
    cycle_std_te       = std([te_pos, te_neg]);
    cycle_slip         = double(slip_pos || slip_neg);
    cycle_mean_current = mean([abs(curr_pos), abs(curr_neg)]);

    %% Update rolling window
    recent_idx = mod(recent_idx, 10) + 1;
    recent_te(recent_idx)   = cycle_mean_te;
    recent_slip(recent_idx) = cycle_slip;

    %% ---- Failure tier computation ----
    tier = 0;

    %% Tier 3 — Critical: catastrophic slip or overcurrent
    if abs(te_pos) > SLIP_THRESHOLD_DEG || abs(te_neg) > SLIP_THRESHOLD_DEG
        tier = 3;
    elseif cycle_mean_current > CURRENT_RATED_LIMIT_mA
        tier = 3;
    end

    %% Tier 2 — Intermediate
    if tier < 3 && cycle >= 10
        window_mean_te    = mean(recent_te);
        window_slip_count = sum(recent_slip);
        current_rise_pct  = (cycle_mean_current - baseline_mean_current) / baseline_mean_current;

        if abs(window_mean_te) > TIER2_TE_DEG
            tier = max(tier, 2);
        end
        if window_slip_count > TIER2_SLIP_IN_10
            tier = max(tier, 2);
        end
        if current_rise_pct > TIER2_CURRENT_PCT
            tier = max(tier, 2);
        end
    end

    %% Tier 1 — Minimal
    if tier < 2
        te_drift         = abs(cycle_mean_te - baseline_mean_te);
        current_rise_pct = (cycle_mean_current - baseline_mean_current) / baseline_mean_current;

        if te_drift > TIER1_TE_DEG
            tier = max(tier, 1);
        end
        if current_rise_pct >= TIER1_CURRENT_PCT_LOW && current_rise_pct < TIER2_CURRENT_PCT
            tier = max(tier, 1);
        end
        if cycle_slip
            tier = max(tier, 1);
        end
    end

    %% ---- Log row ----
    log_cycle(k)         = cycle;
    log_timestamp(k)     = toc(t0);
    log_cmd_pos(k)       = +half_range;
    log_cmd_neg(k)       = -half_range;
    log_mean_dxl(k)      = mean([abs(te_pos + half_range), abs(te_neg - (-half_range))]);
    log_mean_enc(k)      = cycle_mean_te + half_range;
    log_mean_te(k)       = cycle_mean_te;
    log_std_te(k)        = cycle_std_te;
    log_slip_flag(k)     = cycle_slip;
    log_mean_current(k)  = cycle_mean_current;
    log_failure_tier(k)  = tier;
    log_temperature_C(k) = cycle_temp_C;   % <-- log temperature

    %% Console output
    if cycle <= 20 || mod(cycle, 10) == 0
        tier_str = {'OK','T1','T2','T3'};
        fprintf('  Cy%5d | TE:%+6.3f deg  I:%5.0f mA  T:%3.0f°C  Slip:%d  [%s]\n', ...
                cycle, cycle_mean_te, cycle_mean_current, cycle_temp_C, cycle_slip, tier_str{tier+1});
    end

    %% ---- Critical failure: stop immediately ----
    if tier == 3
        fprintf('\n!!! CRITICAL FAILURE (Tier 3) at cycle %d !!!\n', cycle);
        fprintf('    TE pos: %+.3f deg  TE neg: %+.3f deg  Current: %.0f mA  Temp: %.0f°C\n', ...
                te_pos, te_neg, cycle_mean_current, cycle_temp_C);
        stop_reason = 'critical_failure';
        break;
    end

    %% ---- Inspection pause every INSPECTION_INTERVAL cycles ----
    if mod(cycle, INSPECTION_INTERVAL) == 0
        %% ---- Mid-run CSV save ----
        T_mid = table( ...
            log_cycle(1:k)', log_timestamp(1:k)', ...
            log_cmd_pos(1:k)', log_cmd_neg(1:k)', ...
            log_mean_te(1:k)', log_std_te(1:k)', ...
            log_slip_flag(1:k)', log_mean_current(1:k)', ...
            log_failure_tier(1:k)', log_temperature_C(1:k)', ...
            'VariableNames', { ...
                'CYCLE','TIMESTAMP_S', ...
                'CMD_POS_DEG','CMD_NEG_DEG', ...
                'MEAN_TRANS_ERR_DEG','STD_TRANS_ERR_DEG', ...
                'SLIP_FLAG','MEAN_CURRENT_mA','FAILURE_TIER','TEMPERATURE_C'});
        mid_filename = sprintf('test3_endurance_mid_cy%05d_%s.csv', ...
                               cycle, datestr(now,'yyyymmdd_HHMMSS'));
        writetable(T_mid, mid_filename);
        fprintf('  [SAVED] Mid-run CSV: %s\n', mid_filename);
        
        fprintf('\n==========================================\n');
        fprintf('  INSPECTION #%d — Cycle %d\n', cycle/INSPECTION_INTERVAL, cycle);
        fprintf('==========================================\n');
        fprintf('  Drive is PAUSED. Motor holding position.\n\n');
        fprintf('  Please complete the following and record in logbook:\n');
        fprintf('    1. Rope condition score  (0=pristine / 1=scuffing / 2=fretting / 3=wire break)\n');
        fprintf('    2. Photograph rope contact zone\n');
        fprintf('    3. Note condition of 3D printed parts\n');
        fprintf('    4. Note any unusual observations\n');
        fprintf('    5. Human-assessed failure tier: %d (auto)  — confirm or override in logbook\n', tier);
        fprintf('\n  Current run statistics:\n');
        fprintf('    Cycles completed  : %d\n', cycle);
        fprintf('    Motor temperature : %.0f °C\n', cycle_temp_C);
        fprintf('    Mean TE (last 10) : %+.4f deg\n', mean(recent_te));
        fprintf('    Mean current      : %.1f mA  (baseline: %.1f mA, %.1f%% rise)\n', ...
                cycle_mean_current, baseline_mean_current, ...
                100*(cycle_mean_current - baseline_mean_current)/baseline_mean_current);
        fprintf('    Slip events (last 10 cycles): %d\n', sum(recent_slip));
        fprintf('\n  >>> Press Enter when inspection is complete to resume...\n');
        input('', 's');

        if estop_pressed(fig)
            stop_reason = 'estop';
            break;
        end

        fprintf('  Resuming endurance run...\n\n');
    end

end  % cycle loop

%% ================================================================
%% SAVE DATA AND GENERATE PLOTS
%% ================================================================

save_and_plot(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
    log_cycle, log_timestamp, log_cmd_pos, log_cmd_neg, ...
    log_mean_dxl, log_mean_enc, log_mean_te, log_std_te, ...
    log_slip_flag, log_mean_current, log_failure_tier, log_temperature_C, ...
    k, gear_ratio, range_dxl, range_enc, ...
    baseline_mean_te, baseline_mean_current, ...
    LOAD_PCT, half_range, stop_reason);

end  %% ---- END MAIN ----


%% ================================================================
%% HELPER: run_one_cycle
%% Moves to +half_range, logs TE + temp, moves to -half_range, logs TE + temp,
%% returns averaged temp_C. Returns per-endpoint TE, current, slip flag.
%% ================================================================
function [te_pos, te_neg, curr_pos, curr_neg, slip_pos, slip_neg, temp_C, ok] = run_one_cycle( ...
        fig, port_num, encoder, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ADDR_GOAL_POSITION, ...
        ADDR_PRESENT_TEMPERATURE, ...
        target_pos_counts, target_neg_counts, mid_counts, ...
        POSITION_TOLERANCE, SETTLE_AT_TARGET, SAMPLE_PERIOD, ...
        dxl_zero_deg, enc_zero_deg, gear_ratio, half_range)  %#ok<INUSL>

    SLIP_THRESHOLD = 15.0;  % deg
    ok = true;

    te_pos   = 0;  te_neg   = 0;
    curr_pos = 0;  curr_neg = 0;
    slip_pos = false; slip_neg = false;
    temp_C   = 0;   % default until read

    %% --- Leg 1: positive endpoint ---
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                   typecast(target_pos_counts, 'uint32'));
    ok = wait_for_position(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
                           ADDR_PRESENT_POSITION, target_pos_counts, POSITION_TOLERANCE, 15);
    if ~ok, return; end
    pause(SETTLE_AT_TARGET);

    raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    pos_deg  = double(typecast(uint32(raw_pos),'int32')) * 0.00068392 - dxl_zero_deg;
    enc_deg  = read_encoder(encoder) - enc_zero_deg;
    raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
    curr_pos = double(typecast(uint16(raw_curr),'int16'));
    raw_temp = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_TEMPERATURE);
    temp_pos = double(raw_temp);

    expected = pos_deg / gear_ratio;
    te_pos   = expected - enc_deg;
    slip_pos = abs(te_pos) > SLIP_THRESHOLD;

    %% --- Leg 2: negative endpoint ---
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                   typecast(target_neg_counts, 'uint32'));
    ok = wait_for_position(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
                           ADDR_PRESENT_POSITION, target_neg_counts, POSITION_TOLERANCE, 15);
    if ~ok, return; end
    pause(SETTLE_AT_TARGET);

    raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    pos_deg  = double(typecast(uint32(raw_pos),'int32')) * 0.00068392 - dxl_zero_deg;
    enc_deg  = read_encoder(encoder) - enc_zero_deg;
    raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
    curr_neg = double(typecast(uint16(raw_curr),'int16'));
    raw_temp = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_TEMPERATURE);
    temp_neg = double(raw_temp);

    expected = pos_deg / gear_ratio;
    te_neg   = expected - enc_deg;
    slip_neg = abs(te_neg) > SLIP_THRESHOLD;

    %% Average temperature across both endpoints
    temp_C = mean([temp_pos, temp_neg]);
end


%% ================================================================
%% HELPER: wait_for_position
%% ================================================================
function ok = wait_for_position(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_PRESENT_POSITION, goal_counts, tolerance, timeout_sec)
    ok = true;
    t_start = tic;
    while true
        if estop_pressed(fig)
            ok = false; return;
        end
        if toc(t_start) > timeout_sec
            fprintf('[WARNING] Position move timeout — continuing.\n');
            break;
        end
        raw_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        pos_now = typecast(uint32(raw_pos), 'int32');
        if abs(double(pos_now) - double(goal_counts)) < tolerance
            break;
        end
        pause(0.05);
    end
end


%% ================================================================
%% HELPER: find_capstan_limits
%% ================================================================
function [limit_CCW, limit_CW, limit_CCW_enc, limit_CW_enc] = find_capstan_limits(fig, encoder, port_num, ...
        PROTOCOL_VERSION, DXL_ID, lib_name, ...
        ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE, ADDR_GOAL_VELOCITY, ...
        ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ADDR_GOAL_POSITION, ...
        TORQUE_ENABLE, TORQUE_DISABLE)  %#ok<INUSL>

    SEARCH_VELOCITY      = 400;
    CURRENT_ABS_LIMIT    = 350;
    CURRENT_SPIKE_DELTA  = 150;
    SPIKE_WINDOW         = 20;
    TIMEOUT_SECONDS      = 120;
    RETRACT_COUNTS       = 800000;

    limit_CCW = NaN; limit_CW = NaN;
    limit_CCW_enc = NaN; limit_CW_enc = NaN;

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

    fprintf('  Searching CCW limit...\n');
    [limit_CCW, limit_CCW_enc, found] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
        -SEARCH_VELOCITY, CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, TIMEOUT_SECONDS, 0.02);
    if ~found, return; end
    fprintf('  CCW found: Dxl %+.3f | Enc %+.3f\n', limit_CCW, limit_CCW_enc);

    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
    pause(0.5);
    retract(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ...
            ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, TORQUE_ENABLE, RETRACT_COUNTS, +1);

    fprintf('  Searching CW limit...\n');
    [limit_CW, limit_CW_enc, found] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
        +SEARCH_VELOCITY, CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, TIMEOUT_SECONDS, 0.02);
    if ~found, return; end
    fprintf('  CW  found: Dxl %+.3f | Enc %+.3f\n', limit_CW, limit_CW_enc);

    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
    pause(0.5);
    retract(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ...
            ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, TORQUE_ENABLE, RETRACT_COUNTS, -1);
end


%% ================================================================
%% HELPER: search_limit
%% ================================================================
function [limit_deg, limit_enc, found] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
        velocity_cmd, ABS_LIMIT, SPIKE_DELTA, SPIKE_WINDOW, TIMEOUT, SAMPLE_PERIOD)

    found = false; limit_deg = 0; limit_enc = 0; history = [];
    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, velocity_cmd);
    t_start = tic;

    while toc(t_start) < TIMEOUT
        if estop_pressed(fig)
            fprintf('\n>>> E-STOP during limit search <<<\n');
            set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
            return;
        end
        raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
        curr     = double(typecast(uint16(raw_curr), 'int16'));
        raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        pos_deg  = double(typecast(uint32(raw_pos), 'int32')) * 0.00068392;
        encoder_angle = read_encoder(encoder);

        fprintf('    Pos: %+8.3f deg | Current: %5.0f mA | Encoder: %+8.3f deg\n', pos_deg, curr, encoder_angle);

        history(end+1) = abs(curr); %#ok<AGROW>
        if length(history) > SPIKE_WINDOW
            history = history(end-SPIKE_WINDOW+1:end);
        end

        if abs(curr) > ABS_LIMIT
            fprintf('  [LIMIT] Absolute threshold: %d mA\n', curr);
            limit_deg = pos_deg; limit_enc = encoder_angle; found = true; break;
        end
        if length(history) == SPIKE_WINDOW
            baseline = mean(history(1:end-1));
            if (abs(curr) - baseline) > SPIKE_DELTA
                fprintf('  [LIMIT] Spike: %.0f mA above baseline %.0f mA\n', abs(curr)-baseline, baseline);
                limit_deg = pos_deg; limit_enc = encoder_angle; found = true; break;
            end
        end
        pause(SAMPLE_PERIOD);
    end
    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
end


%% ================================================================
%% HELPER: retract
%% ================================================================
function retract(port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ...
        ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, ...
        TORQUE_ENABLE, RETRACT_COUNTS, direction)

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 4);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

    raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    pos_now  = typecast(uint32(raw_pos), 'int32');
    goal_pos = int32(pos_now + direction * int32(RETRACT_COUNTS));

    ADDR_PROFILE_VELOCITY = 560;
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, ...
                   typecast(int32(2600), 'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                   typecast(goal_pos, 'uint32'));
    pause(4.0);

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);
end


%% ================================================================
%% HELPER: set_velocity
%% ================================================================
function set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, vel)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, ...
                   typecast(int32(vel), 'uint32'));
end


%% ================================================================
%% HELPER: estop_pressed
%% ================================================================
function fired = estop_pressed(fig)
    drawnow;
    fired = strcmpi(getappdata(fig, 'stop_key'), 'space');
end


%% ================================================================
%% HELPER: read_encoder
%% ================================================================
function angle = read_encoder(encoder)
    persistent last_angle;
    if isempty(last_angle), last_angle = 0; end
    while encoder.NumBytesAvailable > 0
        data = readline(encoder);
        val  = str2double(strtrim(data));
        if ~isnan(val), last_angle = val; end
    end
    angle = last_angle;
end


%% ================================================================
%% HELPER: emergency_shutdown
%% ================================================================
function emergency_shutdown(fig, port_num, lib_name, torque_was_on, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder)
    fprintf('\n>>> EMERGENCY SHUTDOWN <<<\n');
    if torque_was_on
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    end
    closePort(port_num);
    unloadlibrary(lib_name);
    clear encoder;
    set(fig,'Color',[0.8 0.1 0.1],'Name','>>> E-STOP / ERROR <<<');
end


%% ================================================================
%% HELPER: save_and_plot
%% ================================================================
function save_and_plot(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
        log_cycle, log_timestamp, log_cmd_pos, log_cmd_neg, ...
        log_mean_dxl, log_mean_enc, log_mean_te, log_std_te, ...
        log_slip_flag, log_mean_current, log_failure_tier, log_temperature_C, ...
        k, gear_ratio, range_dxl, range_enc, ...
        baseline_mean_te, baseline_mean_current, ...
        LOAD_PCT, half_range, stop_reason)

    k_end = max(k, 1);

    T = table( ...
        log_cycle(1:k_end)', ...
        log_timestamp(1:k_end)', ...
        log_cmd_pos(1:k_end)', ...
        log_cmd_neg(1:k_end)', ...
        log_mean_te(1:k_end)', ...
        log_std_te(1:k_end)', ...
        log_slip_flag(1:k_end)', ...
        log_mean_current(1:k_end)', ...
        log_failure_tier(1:k_end)', ...
        log_temperature_C(1:k_end)', ...
        'VariableNames', { ...
            'CYCLE', 'TIMESTAMP_S', ...
            'CMD_POS_DEG', 'CMD_NEG_DEG', ...
            'MEAN_TRANS_ERR_DEG', 'STD_TRANS_ERR_DEG', ...
            'SLIP_FLAG', 'MEAN_CURRENT_mA', 'FAILURE_TIER', 'TEMPERATURE_C'});

    filename = sprintf('test3_endurance_%s.csv', datestr(now,'yyyymmdd_HHMMSS'));
    writetable(T, filename);

    %% Append metadata
    fid = fopen(filename, 'a');
    fprintf(fid, '\nMETADATA\n');
    fprintf(fid, 'STOP_REASON,%s\n',               stop_reason);
    fprintf(fid, 'TOTAL_CYCLES,%d\n',              k_end);
    fprintf(fid, 'LOAD_PCT,%d\n',                  LOAD_PCT);
    fprintf(fid, 'OUTPUT_HALF_RANGE_DEG,%.2f\n',   half_range);
    fprintf(fid, 'GEAR_RATIO,%.6f\n',              gear_ratio);
    fprintf(fid, 'INPUT_RANGE_DEG,%.4f\n',         range_dxl);
    fprintf(fid, 'OUTPUT_RANGE_DEG,%.4f\n',        range_enc);
    fprintf(fid, 'BASELINE_MEAN_TE_DEG,%.6f\n',    baseline_mean_te);
    fprintf(fid, 'BASELINE_MEAN_CURRENT_mA,%.2f\n',baseline_mean_current);
    fprintf(fid, 'CENSORED,%d\n',                  strcmp(stop_reason,'completed'));
    fclose(fid);

    fprintf('\nData saved to: %s  (%d cycles logged)\n', filename, k_end);
    fprintf('  Stop reason     : %s\n', stop_reason);
    fprintf('  Gear ratio      : %.4f\n', gear_ratio);
    fprintf('  Baseline TE     : %+.4f deg\n', baseline_mean_te);
    fprintf('  Baseline current: %.1f mA\n', baseline_mean_current);

    %% Shutdown hardware
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    fprintf('Torque disabled.\n');
    closePort(port_num);
    unloadlibrary(lib_name);
    clear encoder;
    set(fig,'Color',[0.2 0.2 0.8],'Name','>>> DONE <<<');

    %% ---- Plots ----
    cyc  = log_cycle(1:k_end);
    te   = log_mean_te(1:k_end);
    sd   = log_std_te(1:k_end);
    cur  = log_mean_current(1:k_end);
    slp  = log_slip_flag(1:k_end);
    tir  = log_failure_tier(1:k_end);
    temp = log_temperature_C(1:k_end);

    tier_colors = [0.2 0.7 0.2;   % 0 = green
                   1.0 0.8 0.0;   % 1 = yellow
                   1.0 0.5 0.0;   % 2 = orange
                   0.8 0.1 0.1];  % 3 = red

    %% Plot 1: Transmission Error over cycles with ±1 SD band
    figure('Name','Test 3 — TE vs Cycles');
    hold on; grid on;
    fill([cyc fliplr(cyc)], [te+sd fliplr(te-sd)], [0.7 0.7 1.0], ...
         'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', '±1 SD');
    plot(cyc, te, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Mean TE');
    yline(baseline_mean_te + 1.0, 'y--', 'Tier 1 upper', 'LineWidth', 1);
    yline(baseline_mean_te - 1.0, 'y--', 'Tier 1 lower', 'LineWidth', 1);
    yline( 3.0, 'r--', 'Tier 2 upper', 'LineWidth', 1);
    yline(-3.0, 'r--', 'Tier 2 lower', 'LineWidth', 1);
    xlabel('Cycle'); ylabel('Mean Transmission Error [deg]');
    title('Test 3 — Transmission Error vs Cycles');
    legend('Location','best');

    %% Plot 2: Motor current over cycles
    figure('Name','Test 3 — Current vs Cycles');
    hold on; grid on;
    plot(cyc, cur, 'k-', 'LineWidth', 1.0, 'DisplayName', 'Mean |Current|');
    yline(baseline_mean_current * 1.05, 'y--', '+5% (T1)',  'LineWidth', 1);
    yline(baseline_mean_current * 1.20, 'r--', '+20% (T2)', 'LineWidth', 1);
    xlabel('Cycle'); ylabel('Mean Current [mA]');
    title('Test 3 — Motor Current vs Cycles');
    legend('Location','best');

    %% Plot 3: Failure tier over cycles (colour-coded scatter)
    figure('Name','Test 3 — Failure Tier Timeline');
    hold on; grid on;
    for t_val = 0:3
        mask = tir == t_val;
        if any(mask)
            scatter(cyc(mask), tir(mask), 8, tier_colors(t_val+1,:), 'filled', ...
                    'DisplayName', sprintf('Tier %d', t_val));
        end
    end
    slip_mask = slp == 1;
    if any(slip_mask)
        scatter(cyc(slip_mask), tir(slip_mask), 40, 'k', 'x', ...
                'LineWidth', 1.5, 'DisplayName', 'Slip event');
    end
    yticks(0:3); yticklabels({'0-Nominal','1-Minimal','2-Intermediate','3-Critical'});
    xlabel('Cycle'); ylabel('Failure Tier');
    title('Test 3 — Failure Tier Timeline');
    legend('Location','best');

    %% Plot 4: Motor temperature over cycles
    figure('Name','Test 3 — Temperature vs Cycles');
    plot(cyc, temp, 'r-', 'LineWidth', 1.0);
    xlabel('Cycle'); ylabel('Temperature [°C]');
    title('Test 3 — Motor Temperature vs Cycles');
    grid on;

    fprintf('Plots generated. Done.\n');
end