function load_response_test_v1()

clc;
clear;

warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% ================================================================
%% TUNING PARAMETERS
%% ================================================================

NUM_LIMIT_SEARCHES  = 1;        % Only 1 limit search for Test 1
TEST1_CYCLES        = 2;       % Cycles per load level
TEST1_RANGE_DEG     = 50;       % Total output range: -30 to +30 deg
TEST1_VELOCITY      = 2600;     % Velocity command for motion legs
SETTLE_TIME         = 5;       % Seconds to settle before each load level
REST_BETWEEN_LOADS  = 5;       % Seconds rest between load levels
REST_BETWEEN_PASSES = 300;      % Seconds rest between Pass 1 and Pass 2
NUM_PASSES          = 2;        % Run the full load sequence twice
SAMPLE_PERIOD       = 0.02;

% Load levels as fraction of rated torque (for labelling only)
% Operator applies the physical weight before each level
LOAD_LEVELS_PCT     = [0, 20, 40, 60, 80];

%% ================================================================
%% E-STOP FIGURE
%% ================================================================

fig = figure('Name','>>> LOAD RESPONSE TEST | Click here then press space to E-STOP <<<', ...
             'Color',[0.1 0.7 0.1], ...
             'NumberTitle','off', ...
             'MenuBar','none', ...
             'ToolBar','none', ...
             'KeyPressFcn',@(src,evt) setappdata(src,'stop_key',evt.Key));
setappdata(fig,'stop_key','');
drawnow;
fprintf('=== E-STOP READY — click the green figure window, then press Space to stop ===\n\n');

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
mid_enc    = (limit_CCW_enc + limit_CW_enc) / 2;

fprintf('==========================================\n');
fprintf('  LIMIT SEARCH RESULT\n');
fprintf('  Dynamixel | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW, limit_CW, range_dxl);
fprintf('  Encoder   | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW_enc, limit_CW_enc, range_enc);
fprintf('  Gear ratio (Dxl range / Enc range): %.4f\n', gear_ratio);
fprintf('  Midpoint  | Dxl: %+.3f  Enc: %+.3f\n', mid_dxl, mid_enc);
fprintf('==========================================\n\n');

%% Verify that +-30 deg on the encoder fits within the physical limits
half_range = TEST1_RANGE_DEG / 2;  % 30 deg
if (mid_enc - half_range) < min(limit_CCW_enc, limit_CW_enc) || ...
   (mid_enc + half_range) > max(limit_CCW_enc, limit_CW_enc)
    fprintf('[WARNING] Requested +-%.0f deg range exceeds physical limits. Aborting.\n', half_range);
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end
fprintf('  Range check passed: +-%.0f deg fits within physical limits.\n\n', half_range);

%% ================================================================
%% MOVE TO MIDPOINT AND ZERO SENSORS
%% ================================================================

fprintf('Moving to midpoint...\n');

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

mid_counts = int32(mid_dxl / 0.00068392);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
               typecast(mid_counts, 'uint32'));

while true
    if estop_pressed(fig)
        emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
        return;
    end
    raw_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    pos_now = typecast(uint32(raw_pos), 'int32');
    if abs(pos_now - mid_counts) < 30
        break;
    end
    pause(0.05);
end
fprintf('At midpoint.\n\n');

%% Zero both sensors at midpoint
fprintf('Zeroing both sensors at midpoint...\n');
raw_pos_zero    = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
dxl_zero_counts = typecast(uint32(raw_pos_zero), 'int32');
dxl_zero_deg    = double(dxl_zero_counts) * 0.00068392;

flush(encoder);
pause(0.2);
enc_zero_deg = read_encoder(encoder);

fprintf('  Dxl zero reference : %.4f deg\n', dxl_zero_deg);
fprintf('  Enc zero reference : %.4f deg\n', enc_zero_deg);
fprintf('Zeroing complete. Both sensors now read 0 at midpoint.\n\n');
input('  >> Press Enter to start Test 1... ', 's');
%% ================================================================
%% PHASE 2 — TEST 1: LOAD RESPONSE (2 PASSES)
%% ================================================================

fprintf('==========================================\n');
fprintf('  PHASE 2: Test 1 — Load Response\n');
fprintf('  %d load levels x %d cycles x %d passes\n', ...
        numel(LOAD_LEVELS_PCT), TEST1_CYCLES, NUM_PASSES);
fprintf('==========================================\n\n');

%% Pre-allocate log (generous upper bound)
maxSamples    = NUM_PASSES * numel(LOAD_LEVELS_PCT) * TEST1_CYCLES * 500;
log_pass      = zeros(1, maxSamples);
log_load_pct  = zeros(1, maxSamples);
log_cycle     = zeros(1, maxSamples);
log_time      = zeros(1, maxSamples);
log_cmd_pos   = zeros(1, maxSamples);
log_dxl_pos   = zeros(1, maxSamples);
log_enc_pos   = zeros(1, maxSamples);
log_trans_err = zeros(1, maxSamples);
log_current   = zeros(1, maxSamples);
k  = 1;
t0 = tic;

%% Switch to velocity mode for all cyclic motion
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

for pass = 1:NUM_PASSES

    fprintf('============================\n');
    fprintf('  PASS %d of %d\n', pass, NUM_PASSES);
    fprintf('============================\n\n');

    for lvl_idx = 1:numel(LOAD_LEVELS_PCT)

        load_pct = LOAD_LEVELS_PCT(lvl_idx);

        %% --------------------------------------------------------
        %% Operator prompt: apply physical load weight
        %% --------------------------------------------------------
        fprintf('--------------------------------------------------\n');
        fprintf('  LOAD LEVEL: %d%% of rated torque\n', load_pct);
        fprintf('  >>> Please apply the %d%% weight now.\n', load_pct);
        fprintf('  >>> Then press Enter to begin %d-second settling time...\n', SETTLE_TIME);
        input('', 's');

        if estop_pressed(fig)
            save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                log_enc_pos, log_trans_err, log_current, k, gear_ratio, range_dxl, range_enc);
            return;
        end

        %% Return to midpoint before settling
        fprintf('  Returning to midpoint...\n');
        ok = move_to_midpoint(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
            ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE, ADDR_GOAL_POSITION, ...
            ADDR_PRESENT_POSITION, TORQUE_ENABLE, mid_counts);
        if ~ok
            save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                log_enc_pos, log_trans_err, log_current, k, gear_ratio, range_dxl, range_enc);
            return;
        end

        %% Re-enter velocity mode after midpoint move
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

        %% Settling period (stationary, logging current and position)
        fprintf('  Settling for %d seconds...\n', SETTLE_TIME);
        t_settle = tic;
        while toc(t_settle) < SETTLE_TIME
            if estop_pressed(fig)
                save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                    log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                    log_enc_pos, log_trans_err, log_current, k, gear_ratio, range_dxl, range_enc);
                return;
            end
            pause(1.0);
            fprintf('    Settling: %.0f / %d s\n', toc(t_settle), SETTLE_TIME);
        end
        fprintf('  Settling complete.\n\n');

        %% --------------------------------------------------------
        %% Run TEST1_CYCLES back-and-forth cycles
        %% --------------------------------------------------------
        fprintf('  Running %d cycles at %d%% load...\n', TEST1_CYCLES, load_pct);

        for cycle = 1:TEST1_CYCLES

            fprintf('  Cycle %d / %d\n', cycle, TEST1_CYCLES);

            % Two legs: go to +30 deg then back to -30 deg (encoder coordinates)
            targets_enc = [+half_range, -half_range];
            directions  = [+1, -1];

            for leg = 1:2

                target_enc = targets_enc(leg);
                vel_cmd    = directions(leg) * TEST1_VELOCITY;

                set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, vel_cmd);

                while true

                    if estop_pressed(fig)
                        set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
                        save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                            ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                            log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                            log_enc_pos, log_trans_err, log_current, k, gear_ratio, range_dxl, range_enc);
                        return;
                    end

                    %% Read Dynamixel position (zeroed)
                    raw_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
                    pos_deg = double(typecast(uint32(raw_pos), 'int32')) * 0.00068392 - dxl_zero_deg;

                    %% Read encoder (zeroed)
                    enc_deg = read_encoder(encoder) - enc_zero_deg;

                    %% Read current
                    raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
                    curr     = double(typecast(uint16(raw_curr), 'int16'));

                    %% Transmission error: expected output - actual output
                    expected_output = pos_deg / gear_ratio;
                    TE = expected_output - enc_deg;

                    %% Log
                    if k <= maxSamples
                        log_pass(k)      = pass;
                        log_load_pct(k)  = load_pct;
                        log_cycle(k)     = cycle;
                        log_time(k)      = toc(t0);
                        log_cmd_pos(k)   = expected_output;
                        log_dxl_pos(k)   = pos_deg;
                        log_enc_pos(k)   = enc_deg;
                        log_trans_err(k) = TE;
                        log_current(k)   = curr;
                        k = k + 1;
                    end

                    fprintf('    P%d L%d%% Cy%2d Lg%d | Dxl:%+7.2f Exp.Out:%+7.2f Enc:%+7.2f TE:%+6.3f deg I:%4.0f mA\n', ...
                            pass, load_pct, cycle, leg, pos_deg, expected_output, enc_deg, TE, curr);

                    %% Stop when encoder reaches target within 1 deg
                    if abs(enc_deg - target_enc) < 1.0
                        set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
                        pause(0.2);
                        break;
                    end

                    pause(SAMPLE_PERIOD);
                end
            end
        end

        fprintf('  Load level %d%% complete.\n\n', load_pct);

        %% 60-second rest between load levels (except after last level of each pass)
        is_last_level = (lvl_idx == numel(LOAD_LEVELS_PCT));
        is_last_pass  = (pass == NUM_PASSES);

        if ~is_last_level
            fprintf('  Resting %d seconds before next load level...\n', REST_BETWEEN_LOADS);
            fprintf('  >>> Remove current weight during this rest.\n\n');
            t_rest = tic;
            while toc(t_rest) < REST_BETWEEN_LOADS
                if estop_pressed(fig)
                    save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                        log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                        log_enc_pos, log_trans_err, log_current, k, gear_ratio, range_dxl, range_enc);
                    return;
                end
                pause(5.0);
                fprintf('    Rest: %.0f / %d s\n', toc(t_rest), REST_BETWEEN_LOADS);
            end
        end

    end % load levels

    %% 5-minute rest between Pass 1 and Pass 2
    if pass < NUM_PASSES
        fprintf('\n============================\n');
        fprintf('  Pass %d complete.\n', pass);
        fprintf('  5-minute rest before Pass %d.\n', pass+1);
        fprintf('  >>> Remove all weights during this rest.\n');
        fprintf('============================\n\n');
        t_rest = tic;
        while toc(t_rest) < REST_BETWEEN_PASSES
            if estop_pressed(fig)
                save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                    log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                    log_enc_pos, log_trans_err, log_current, k, gear_ratio, range_dxl, range_enc);
                return;
            end
            pause(10.0);
            fprintf('  Inter-pass rest: %.0f / %d s\n', toc(t_rest), REST_BETWEEN_PASSES);
        end
        fprintf('  Rest complete. Starting Pass %d.\n\n', pass+1);
    end

end % passes

%% ================================================================
%% SAVE AND PLOT
%% ================================================================

set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);

save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
    log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
    log_enc_pos, log_trans_err, log_current, k, gear_ratio, range_dxl, range_enc);

end % ---- END MAIN ----


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
    RETRACT_COUNTS       = 600000;
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

    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                   typecast(goal_pos, 'uint32'));
    pause(4.0);

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);
end


%% ================================================================
%% HELPER: move_to_midpoint  (position mode, then returns to vel mode)
%% ================================================================
function ok = move_to_midpoint(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE, ADDR_GOAL_POSITION, ...
        ADDR_PRESENT_POSITION, TORQUE_ENABLE, mid_counts)

    ok = true;
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 4);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                   typecast(mid_counts, 'uint32'));

    t_start = tic;
    while true
        if estop_pressed(fig)
            ok = false;
            return;
        end
        if toc(t_start) > 10
            fprintf('[WARNING] Midpoint move timeout — continuing anyway.\n');
            break;
        end
        raw_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        pos_now = typecast(uint32(raw_pos), 'int32');
        if abs(pos_now - mid_counts) < 30
            break;
        end
        pause(0.05);
    end
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
        log_pass, log_load_pct, log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
        log_enc_pos, log_trans_err, log_current, k, ...
        gear_ratio, range_dxl, range_enc)

    k_end = max(k-1, 1);

    %% Main data table — includes pass and load_pct columns
    T = table(log_pass(1:k_end)', log_load_pct(1:k_end)', log_cycle(1:k_end)', ...
              log_time(1:k_end)', log_cmd_pos(1:k_end)', log_dxl_pos(1:k_end)', ...
              log_enc_pos(1:k_end)', log_trans_err(1:k_end)', log_current(1:k_end)', ...
        'VariableNames', {'PASS','LOAD_PCT','CYCLE','TIME', ...
                          'EXPECTED_OUTPUT_DEG','DXL_POS_DEG','ENC_POS_DEG', ...
                          'TRANS_ERR_DEG','CURRENT_mA'});

    filename = sprintf('test1_load_response_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
    writetable(T, filename);

    %% Append metadata
    fid = fopen(filename, 'a');
    fprintf(fid, '\n');
    fprintf(fid, 'METADATA\n');
    fprintf(fid, 'GEAR_RATIO,%.6f\n',    gear_ratio);
    fprintf(fid, 'INPUT_RANGE_DEG,%.4f\n',  range_dxl);
    fprintf(fid, 'OUTPUT_RANGE_DEG,%.4f\n', range_enc);
    fclose(fid);

    fprintf('\nData saved to: %s  (%d samples)\n', filename, k_end);
    fprintf('  Gear ratio:    %.4f\n', gear_ratio);
    fprintf('  Input range:   %.4f deg (Dynamixel)\n', range_dxl);
    fprintf('  Output range:  %.4f deg (Encoder)\n',   range_enc);

    %% Shutdown
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    fprintf('Torque disabled.\n');
    closePort(port_num);
    unloadlibrary(lib_name);
    clear encoder;
    set(fig,'Color',[0.2 0.2 0.8],'Name','>>> DONE <<<');

    %% ---- Plots ----
    passes    = unique(log_pass(1:k_end));
    load_lvls = unique(log_load_pct(1:k_end));
    colors    = lines(numel(load_lvls));

    %% Plot 1: Mean TE vs Load (both passes overlaid)
    figure('Name','Test 1 — Mean Transmission Error vs Load');
    hold on; grid on;
    legend_entries = {};
    for pi = 1:numel(passes)
        for li = 1:numel(load_lvls)
            mask = (log_pass(1:k_end) == passes(pi)) & (log_load_pct(1:k_end) == load_lvls(li));
            if any(mask)
                te_vals = log_trans_err(mask);
                errorbar(load_lvls(li), mean(te_vals), std(te_vals), 'o', ...
                         'Color', colors(li,:), 'MarkerFaceColor', colors(li,:), ...
                         'DisplayName', sprintf('Pass %d, %d%%', passes(pi), load_lvls(li)));
                legend_entries{end+1} = sprintf('Pass %d, %d%%', passes(pi), load_lvls(li)); %#ok<AGROW>
            end
        end
    end
    xlabel('Load [% rated torque]');
    ylabel('Mean Transmission Error [deg]');
    title('Test 1 — Mean TE ± 1 SD vs Load Level');
    legend('Location','best');

    %% Plot 2: Mean Current vs Load
    figure('Name','Test 1 — Mean Current vs Load');
    hold on; grid on;
    for pi = 1:numel(passes)
        mean_curr = zeros(1, numel(load_lvls));
        for li = 1:numel(load_lvls)
            mask = (log_pass(1:k_end) == passes(pi)) & (log_load_pct(1:k_end) == load_lvls(li));
            if any(mask)
                mean_curr(li) = mean(abs(log_current(mask)));
            end
        end
        plot(load_lvls, mean_curr, '-o', 'DisplayName', sprintf('Pass %d', passes(pi)));
    end
    xlabel('Load [% rated torque]');
    ylabel('Mean |Current| [mA]');
    title('Test 1 — Mean Current vs Load Level');
    legend('Location','best');

    %% Plot 3: TE time series coloured by load level
    figure('Name','Test 1 — TE Time Series');
    hold on; grid on;
    for li = 1:numel(load_lvls)
        mask = log_load_pct(1:k_end) == load_lvls(li);
        if any(mask)
            plot(log_time(mask), log_trans_err(mask), '.', ...
                 'Color', colors(li,:), ...
                 'DisplayName', sprintf('%d%% load', load_lvls(li)));
        end
    end
    xlabel('Time [s]');
    ylabel('Transmission Error [deg]');
    title('Test 1 — TE vs Time (coloured by load level)');
    legend('Location','best');

    fprintf('Plots generated. Done.\n');
end