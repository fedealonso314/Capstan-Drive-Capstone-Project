function creep_static_load_test_v1()

clc;
clear;

warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% ================================================================
%% TUNING PARAMETERS
%% ================================================================

NUM_REPETITIONS         = 3;        % Repeat full hold 3 times (fresh setup each)
HOLD_DURATION_SEC       = 60 * 60;  % 60 minutes
LOG_INTERVAL_SEC        = 1.0;      % One row logged per second during hold
POSITION_TOLERANCE      = 50;       % Counts — how close Dynamixel must be before hold begins
PROFILE_VELOCITY        = 2600;     % Dynamixel profile velocity (0.01 rev/min units)
REST_BETWEEN_REPS       = 60;      % Seconds rest between repetitions (5 min)

% Snapshot timestamps (seconds from load application) — for reporting
SNAPSHOT_TIMES_SEC      = [300, 900, 1800, 3600];   % seconds

% Failure tier drift thresholds (degrees from hold position)
CRIT_DRIFT_DEG          = 15.0;
INTER_DRIFT_DEG         = 3.0;
MINIMAL_DRIFT_DEG       = 1.0;

%% ================================================================
%% E-STOP FIGURE
%% ================================================================

fig = figure('Name','>>> CREEP / STATIC LOAD TEST v1 | Click here then press SPACE to E-STOP <<<', ...
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
fprintf('Encoder serial port opened\n\n');

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

ADDR_TORQUE_ENABLE    = 512;
ADDR_OPERATING_MODE   = 11;
ADDR_GOAL_VELOCITY    = 552;
ADDR_PROFILE_VELOCITY = 560;
ADDR_PRESENT_POSITION = 580;
ADDR_PRESENT_CURRENT  = 574;
ADDR_GOAL_POSITION    = 564;

PROTOCOL_VERSION      = 2.0;
DXL_ID                = 1;
DEVICENAME            = 'COM8';
BAUDRATE              = 57600;
TORQUE_ENABLE         = 1;
TORQUE_DISABLE        = 0;

port_num = portHandler(DEVICENAME);
packetHandler();

if openPort(port_num)
    fprintf('Port opened\n');
else
    emergency_shutdown(fig, port_num, lib_name, 0, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    error('Failed to open port');
end

if setBaudRate(port_num, BAUDRATE)
    fprintf('Baudrate set\n\n');
else
    emergency_shutdown(fig, port_num, lib_name, 0, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
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
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end

%% ================================================================
%% COMPUTE GEAR RATIO AND MIDPOINT
%% ================================================================

range_dxl  = abs(limit_CW     - limit_CCW);
range_enc  = abs(limit_CW_enc - limit_CCW_enc);
gear_ratio = range_dxl / range_enc;
mid_dxl    = (limit_CCW     + limit_CW)     / 2;

fprintf('==========================================\n');
fprintf('  LIMIT SEARCH RESULT\n');
fprintf('  Dynamixel | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW, limit_CW, range_dxl);
fprintf('  Encoder   | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW_enc, limit_CW_enc, range_enc);
fprintf('  Gear ratio (Dxl range / Enc range): %.4f\n', gear_ratio);
fprintf('  Midpoint  | Dxl: %+.3f\n', mid_dxl);
fprintf('==========================================\n\n');

%% ================================================================
%% MOVE TO MIDPOINT AND ZERO SENSORS
%% ================================================================

fprintf('Moving to midpoint (extended position mode)...\n');

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, ...
               typecast(int32(PROFILE_VELOCITY), 'uint32'));

mid_counts = int32(mid_dxl / 0.00068392);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
               typecast(mid_counts, 'uint32'));

ok = wait_for_position(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
                       ADDR_PRESENT_POSITION, mid_counts, POSITION_TOLERANCE, 15);
if ~ok
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end
pause(0.3);
fprintf('At midpoint.\n\n');

%% Zero both sensors at midpoint — 0 deg on both axes from here on
fprintf('Zeroing both sensors at midpoint...\n');
raw_pos_zero    = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
dxl_zero_counts = typecast(uint32(raw_pos_zero), 'int32');
dxl_zero_deg    = double(dxl_zero_counts) * 0.00068392;

flush(encoder);
pause(0.2);
enc_zero_deg = read_encoder(encoder);

fprintf('  Dxl zero reference : %.4f deg\n', dxl_zero_deg);
fprintf('  Enc zero reference : %.4f deg\n', enc_zero_deg);
fprintf('Zeroing complete. Hold position is 0 deg (midpoint) on both sensors.\n\n');

%% ================================================================
%% PRE-ALLOCATE LOG
%% 3 reps x 1800 s x 1 sample/s + margin
%% ================================================================

maxSamples    = NUM_REPETITIONS * (ceil(HOLD_DURATION_SEC / LOG_INTERVAL_SEC) + 60);
log_rep       = zeros(1, maxSamples);
log_time      = zeros(1, maxSamples);   % Elapsed seconds from load application
log_dxl_pos   = zeros(1, maxSamples);  % Dynamixel position zeroed (deg)
log_enc_pos   = zeros(1, maxSamples);  % Encoder position zeroed (deg)
log_exp_out   = zeros(1, maxSamples);  % Expected output = dxl_pos / gear_ratio
log_trans_err = zeros(1, maxSamples);  % TE = expected_output - enc_pos
log_drift     = zeros(1, maxSamples);  % Drift from hold position (enc_pos, since hold = 0)
log_current   = zeros(1, maxSamples);
log_tier      = zeros(1, maxSamples);  % 0=none, 1=minimal, 2=intermediate, 3=critical
k = 1;

%% ================================================================
%% MAIN LOOP — REPETITIONS
%% ================================================================

for rep = 1:NUM_REPETITIONS

    fprintf('==========================================\n');
    fprintf('  REPETITION %d of %d\n', rep, NUM_REPETITIONS);
    fprintf('==========================================\n\n');

    %% ---- Return to midpoint (no load) before each rep ----
    fprintf('  Returning to midpoint (0 deg) before load application...\n');
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                   typecast(mid_counts, 'uint32'));
    ok = wait_for_position(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
                           ADDR_PRESENT_POSITION, mid_counts, POSITION_TOLERANCE, 15);
    if ~ok
        save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
            ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
            log_rep, log_time, log_dxl_pos, log_enc_pos, log_exp_out, ...
            log_trans_err, log_drift, log_current, log_tier, k, ...
            gear_ratio, range_dxl, range_enc, SNAPSHOT_TIMES_SEC);
        return;
    end
    pause(1.0);

    %% ---- Prompt operator to apply dead load ----
    fprintf('\n  >>> Drive is at midpoint (0 deg). Verify pretension, then apply the dead load now.\n');
    fprintf('  >>> Press Enter when load is applied to start the 30-minute hold.\n');
    input('', 's');

    if estop_pressed(fig)
        save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
            ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
            log_rep, log_time, log_dxl_pos, log_enc_pos, log_exp_out, ...
            log_trans_err, log_drift, log_current, log_tier, k, ...
            gear_ratio, range_dxl, range_enc, SNAPSHOT_TIMES_SEC);
        return;
    end

    %% t=0 snapshot immediately after load application
    flush(encoder);
    pause(0.1);
    raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    dxl_pos0 = double(typecast(uint32(raw_pos), 'int32')) * 0.00068392 - dxl_zero_deg;
    enc_pos0 = read_encoder(encoder) - enc_zero_deg;
    fprintf('  Load applied. t=0 | Dxl: %+.4f deg | Enc: %+.4f deg\n\n', dxl_pos0, enc_pos0);

    snap_recorded = false(1, numel(SNAPSHOT_TIMES_SEC));

    %% ---- 30-minute continuous hold loop ----
    t_hold        = tic;
    last_log_time = -LOG_INTERVAL_SEC;  % Force immediate first log entry

    while toc(t_hold) < HOLD_DURATION_SEC

        %% E-STOP check
        if estop_pressed(fig)
            save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                log_rep, log_time, log_dxl_pos, log_enc_pos, log_exp_out, ...
                log_trans_err, log_drift, log_current, log_tier, k, ...
                gear_ratio, range_dxl, range_enc, SNAPSHOT_TIMES_SEC);
            return;
        end

        elapsed = toc(t_hold);

        %% Only log at LOG_INTERVAL_SEC cadence
        if (elapsed - last_log_time) < LOG_INTERVAL_SEC
            pause(0.02);
            continue;
        end
        last_log_time = elapsed;

        %% Read sensors
        raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        dxl_pos  = double(typecast(uint32(raw_pos), 'int32')) * 0.00068392 - dxl_zero_deg;
        enc_pos  = read_encoder(encoder) - enc_zero_deg;
        raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
        curr     = double(typecast(uint16(raw_curr), 'int16'));

        exp_out   = dxl_pos / gear_ratio;  % Expected output based on input shaft
        trans_err = exp_out - enc_pos;      % TE = expected_output - actual_output
        drift     = enc_pos;               % Drift from hold position (hold = 0 deg)

        %% Assign failure tier based on absolute drift
        if abs(drift) >= CRIT_DRIFT_DEG
            tier = 3;
        elseif abs(drift) >= INTER_DRIFT_DEG
            tier = 2;
        elseif abs(drift) >= MINIMAL_DRIFT_DEG
            tier = 1;
        else
            tier = 0;
        end

        %% Store in log
        if k <= maxSamples
            log_rep(k)       = rep;
            log_time(k)      = elapsed;
            log_dxl_pos(k)   = dxl_pos;
            log_enc_pos(k)   = enc_pos;
            log_exp_out(k)   = exp_out;
            log_trans_err(k) = trans_err;
            log_drift(k)     = drift;
            log_current(k)   = curr;
            log_tier(k)      = tier;
            k = k + 1;
        end

        %% Console — print every 30 s to avoid flooding
        if mod(round(elapsed), 30) == 0 || elapsed < 2
            fprintf('  Rep%d | t=%5.0fs | Dxl:%+7.3f Exp.Out:%+7.3f Enc:%+7.3f TE:%+6.3f Drift:%+6.3f deg | I:%5.0fmA | Tier:%d\n', ...
                    rep, elapsed, dxl_pos, exp_out, enc_pos, trans_err, drift, curr, tier);
        end

        %% Snapshot printouts at 1, 5, 15, 30 min
        for si = 1:numel(SNAPSHOT_TIMES_SEC)
            if ~snap_recorded(si) && elapsed >= SNAPSHOT_TIMES_SEC(si)
                snap_recorded(si) = true;
                fprintf('\n  *** SNAPSHOT at t=%.0f s (%.0f min) | Drift: %+.4f deg | TE: %+.4f deg | Tier: %d ***\n\n', ...
                        elapsed, elapsed/60, drift, trans_err, tier);
            end
        end

        %% Critical failure: stop this rep immediately
        if tier == 3
            fprintf('\n[CRITICAL FAILURE] Drift exceeded %.1f deg at t=%.0f s. Stopping rep %d.\n\n', ...
                    CRIT_DRIFT_DEG, elapsed, rep);
            break;
        end

    end % hold loop

    fprintf('  Repetition %d complete. Elapsed: %.0f s\n\n', rep, toc(t_hold));

    %% Prompt operator to remove load before rest/next rep
    fprintf('  >>> Remove the dead load now.\n');
    fprintf('  >>> Press Enter to continue.\n');
    input('', 's');

    if estop_pressed(fig)
        save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
            ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
            log_rep, log_time, log_dxl_pos, log_enc_pos, log_exp_out, ...
            log_trans_err, log_drift, log_current, log_tier, k, ...
            gear_ratio, range_dxl, range_enc, SNAPSHOT_TIMES_SEC);
        return;
    end

    %% 5-minute rest between repetitions
    if rep < NUM_REPETITIONS
        fprintf('\n  Resting %d seconds before next repetition...\n\n', REST_BETWEEN_REPS);
        t_rest = tic;
        while toc(t_rest) < REST_BETWEEN_REPS
            if estop_pressed(fig)
                save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                    log_rep, log_time, log_dxl_pos, log_enc_pos, log_exp_out, ...
                    log_trans_err, log_drift, log_current, log_tier, k, ...
                    gear_ratio, range_dxl, range_enc, SNAPSHOT_TIMES_SEC);
                return;
            end
            pause(10.0);
            fprintf('  Rest: %.0f / %d s\n', toc(t_rest), REST_BETWEEN_REPS);
        end
        fprintf('  Rest complete. Starting repetition %d.\n\n', rep+1);
    end

end % repetitions

%% ================================================================
%% SAVE AND PLOT
%% ================================================================

save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
    log_rep, log_time, log_dxl_pos, log_enc_pos, log_exp_out, ...
    log_trans_err, log_drift, log_current, log_tier, k, ...
    gear_ratio, range_dxl, range_enc, SNAPSHOT_TIMES_SEC);

end % ---- END MAIN ----


%% ================================================================
%% HELPER: wait_for_position
%% ================================================================
function ok = wait_for_position(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_PRESENT_POSITION, goal_counts, tolerance, timeout_sec)

    ok = true;
    t_start = tic;
    while true
        if estop_pressed(fig)
            ok = false;
            return;
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
        TORQUE_ENABLE, TORQUE_DISABLE)

    SEARCH_VELOCITY      = 400;
    CURRENT_ABS_LIMIT    = 350;
    CURRENT_SPIKE_DELTA  = 150;
    SPIKE_WINDOW         = 20;
    TIMEOUT_SECONDS      = 120;
    RETRACT_COUNTS       = 900000;
    SAMPLE_PERIOD        = 0.02;

    limit_CCW     = NaN;
    limit_CW      = NaN;
    limit_CCW_enc = NaN;
    limit_CW_enc  = NaN;

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

    fprintf('  Searching CCW limit...\n');
    [limit_CCW, limit_CCW_enc, found] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
        -SEARCH_VELOCITY, CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, TIMEOUT_SECONDS, SAMPLE_PERIOD);

    if ~found, return; end
    fprintf('  CCW found: Dxl %+.3f | Enc %+.3f\n', limit_CCW, limit_CCW_enc);

    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
    pause(0.5);
    retract(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ...
            ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, TORQUE_ENABLE, RETRACT_COUNTS, +1);

    fprintf('  Searching CW limit...\n');
    [limit_CW, limit_CW_enc, found] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
        +SEARCH_VELOCITY, CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, TIMEOUT_SECONDS, SAMPLE_PERIOD);

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

    found     = false;
    limit_deg = 0;
    limit_enc = 0;
    history   = [];

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
            limit_deg = pos_deg;
            limit_enc = encoder_angle;
            found     = true;
            break;
        end

        if length(history) == SPIKE_WINDOW
            baseline = mean(history(1:end-1));
            if (abs(curr) - baseline) > SPIKE_DELTA
                fprintf('  [LIMIT] Spike: %.0f mA above baseline %.0f mA\n', abs(curr)-baseline, baseline);
                limit_deg = pos_deg;
                limit_enc = encoder_angle;
                found     = true;
                break;
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
    RETRACT_VELOCITY      = 2600;

    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, ...
                   typecast(int32(RETRACT_VELOCITY), 'uint32'));

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
    if isempty(last_angle)
        last_angle = 0;
    end
    while encoder.NumBytesAvailable > 0
        data = readline(encoder);
        val  = str2double(strtrim(data));
        if ~isnan(val)
            last_angle = val;
        end
    end
    angle = last_angle;
end


%% ================================================================
%% HELPER: emergency_shutdown
%% ================================================================
function emergency_shutdown(fig, port_num, lib_name, torque_was_on, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder)
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
%% HELPER: save_and_shutdown
%% ================================================================
function save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
        log_rep, log_time, log_dxl_pos, log_enc_pos, log_exp_out, ...
        log_trans_err, log_drift, log_current, log_tier, k, ...
        gear_ratio, range_dxl, range_enc, SNAPSHOT_TIMES_SEC)

    k_end = max(k-1, 1);

    T = table(log_rep(1:k_end)', log_time(1:k_end)', ...
              log_dxl_pos(1:k_end)', log_enc_pos(1:k_end)', ...
              log_exp_out(1:k_end)', log_trans_err(1:k_end)', ...
              log_drift(1:k_end)', log_current(1:k_end)', log_tier(1:k_end)', ...
        'VariableNames', {'REP','TIME_SEC', ...
                          'DXL_POS_DEG','ENC_POS_DEG', ...
                          'EXP_OUTPUT_DEG','TRANS_ERR_DEG', ...
                          'DRIFT_DEG','CURRENT_mA','FAILURE_TIER'});

    filename = sprintf('test2_creep_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
    writetable(T, filename);

    %% Append metadata
    fid = fopen(filename, 'a');
    fprintf(fid, '\n');
    fprintf(fid, 'METADATA\n');
    fprintf(fid, 'GEAR_RATIO,%.6f\n',       gear_ratio);
    fprintf(fid, 'INPUT_RANGE_DEG,%.4f\n',  range_dxl);
    fprintf(fid, 'OUTPUT_RANGE_DEG,%.4f\n', range_enc);
    fprintf(fid, 'HOLD_POSITION_DEG,0.0\n');
    fprintf(fid, 'SNAPSHOT_TIMES_SEC,%s\n', num2str(SNAPSHOT_TIMES_SEC));
    fclose(fid);

    fprintf('\nData saved to: %s  (%d samples)\n', filename, k_end);
    fprintf('  Gear ratio:    %.4f\n', gear_ratio);
    fprintf('  Input range:   %.4f deg (Dynamixel)\n', range_dxl);
    fprintf('  Output range:  %.4f deg (Encoder)\n',   range_enc);

    %% Shutdown hardware
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    fprintf('Torque disabled.\n');
    closePort(port_num);
    unloadlibrary(lib_name);
    clear encoder;
    set(fig,'Color',[0.2 0.2 0.8],'Name','>>> DONE <<<');

    %% ================================================================
    %% PLOTS
    %% ================================================================

    reps   = unique(log_rep(1:k_end));
    colors = lines(numel(reps));

    %% Plot 1: Drift over time — all reps overlaid
    figure('Name','Test 2 — Drift vs Time');
    hold on; grid on;
    for ri = 1:numel(reps)
        mask = log_rep(1:k_end) == reps(ri);
        plot(log_time(mask) / 60, log_drift(mask), ...
             'Color', colors(ri,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Rep %d', reps(ri)));
    end
    yline( 3.0, '--r', 'Intermediate (3 deg)', 'LabelHorizontalAlignment','left');
    yline(-3.0, '--r', '',                      'HandleVisibility','off');
    yline( 1.0, '--y', 'Minimal (1 deg)',       'LabelHorizontalAlignment','left');
    yline(-1.0, '--y', '',                      'HandleVisibility','off');
    xlabel('Time [min]');
    ylabel('Drift from Hold Position [deg]');
    title('Test 2 — Output Drift vs Time (Hold at 0 deg / Midpoint, Dead Load)');
    legend('Location','best');

    %% Plot 2: Transmission error over time
    figure('Name','Test 2 — Transmission Error vs Time');
    hold on; grid on;
    for ri = 1:numel(reps)
        mask = log_rep(1:k_end) == reps(ri);
        plot(log_time(mask) / 60, log_trans_err(mask), ...
             'Color', colors(ri,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Rep %d', reps(ri)));
    end
    xlabel('Time [min]');
    ylabel('Transmission Error [deg]');
    title('Test 2 — Transmission Error vs Time');
    legend('Location','best');

    %% Plot 3: Motor current over time
    figure('Name','Test 2 — Motor Current vs Time');
    hold on; grid on;
    for ri = 1:numel(reps)
        mask = log_rep(1:k_end) == reps(ri);
        plot(log_time(mask) / 60, abs(log_current(mask)), ...
             'Color', colors(ri,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Rep %d', reps(ri)));
    end
    xlabel('Time [min]');
    ylabel('|Current| [mA]');
    title('Test 2 — Motor Current vs Time');
    legend('Location','best');

    %% Print snapshot summary table to console
    fprintf('\n--- Drift Snapshots (mean ± SD across reps) ---\n');
    fprintf('%-12s  %-14s  %-12s  %-12s\n', 'Time', 'Mean Drift', 'SD Drift', 'Mean TE');
    for si = 1:numel(SNAPSHOT_TIMES_SEC)
        t_snap     = SNAPSHOT_TIMES_SEC(si);
        drift_vals = [];
        te_vals    = [];
        for ri = 1:numel(reps)
            mask  = log_rep(1:k_end) == reps(ri);
            t_rep = log_time(mask);
            d_rep = log_drift(mask);
            e_rep = log_trans_err(mask);
            [~, idx]          = min(abs(t_rep - t_snap));
            drift_vals(end+1) = d_rep(idx); %#ok<AGROW>
            te_vals(end+1)    = e_rep(idx); %#ok<AGROW>
        end
        fprintf('t=%-7.0fs     %+.4f deg    %.4f deg    %+.4f deg\n', ...
                t_snap, mean(drift_vals), std(drift_vals), mean(te_vals));
    end

    fprintf('\nPlots generated. Done.\n');
end