function baseline_test_v2()

clc;
clear;

warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% ================================================================
%% TUNING PARAMETERS
%% ================================================================

NUM_LIMIT_SEARCHES  = 3;
TEST0_CYCLES        = 20;
TEST0_RANGE_DEG     = 100;    % total range on output drum (+-50 deg from centre)
SEARCH_VELOCITY     = 400;    % velocity for Test 0 motion
SAMPLE_PERIOD       = 0.02;

%% ================================================================
%% E-STOP FIGURE
%% ================================================================

fig = figure('Name','>>> BASELINE TEST | Click here then press Space to E-STOP <<<', ...
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
%% PHASE 1 — FIND LIMITS 3 TIMES
%% ================================================================

fprintf('==========================================\n');
fprintf('  PHASE 1: Limit Search (%d runs)\n', NUM_LIMIT_SEARCHES);
fprintf('==========================================\n\n');

all_CCW     = zeros(1, NUM_LIMIT_SEARCHES);
all_CW      = zeros(1, NUM_LIMIT_SEARCHES);
all_CCW_enc = zeros(1, NUM_LIMIT_SEARCHES);
all_CW_enc  = zeros(1, NUM_LIMIT_SEARCHES);

for run = 1:NUM_LIMIT_SEARCHES

    fprintf('--- Limit Search Run %d of %d ---\n', run, NUM_LIMIT_SEARCHES);

    [ccw, cw, ccw_enc, cw_enc] = find_capstan_limits(fig, encoder, port_num, ...
        PROTOCOL_VERSION, DXL_ID, lib_name, ...
        ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE, ADDR_GOAL_VELOCITY, ...
        ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ADDR_GOAL_POSITION, ...
        TORQUE_ENABLE, TORQUE_DISABLE);

    if isnan(ccw) || isnan(cw)
        fprintf('[ERROR] Run %d failed — aborting.\n', run);
        emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
        return;
    end

    all_CCW(run)     = ccw;
    all_CW(run)      = cw;
    all_CCW_enc(run) = ccw_enc;
    all_CW_enc(run)  = cw_enc;

    fprintf('  Run %d | Dxl CCW: %+.3f  CW: %+.3f | Enc CCW: %+.3f  CW: %+.3f\n\n', ...
            run, ccw, cw, ccw_enc, cw_enc);

    if run < NUM_LIMIT_SEARCHES
        fprintf('  >> Run %d complete. Press Enter to start Run %d...\n', run, run+1);
        input('', 's');
    end

    if estop_pressed(fig)
        emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
        return;
    end
end

%% ================================================================
%% COMPUTE AVERAGED LIMITS AND GEAR RATIO
%% ================================================================

limit_CCW     = mean(all_CCW);
limit_CW      = mean(all_CW);
limit_CCW_enc = mean(all_CCW_enc);
limit_CW_enc  = mean(all_CW_enc);
range_dxl     = abs(limit_CW     - limit_CCW);
range_enc     = abs(limit_CW_enc - limit_CCW_enc);
gear_ratio    = range_dxl / range_enc;
mid_dxl       = (limit_CCW     + limit_CW)     / 2;
mid_enc       = (limit_CCW_enc + limit_CW_enc) / 2;

%% Test 0 output drum targets (+-50 deg from midpoint on encoder)
test0_start_enc = mid_enc - TEST0_RANGE_DEG / 2;
test0_end_enc   = mid_enc + TEST0_RANGE_DEG / 2;

fprintf('==========================================\n');
fprintf('  AVERAGED LIMITS (over %d runs)\n', NUM_LIMIT_SEARCHES);
fprintf('  Dynamixel | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW, limit_CW, range_dxl);
fprintf('  Encoder   | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW_enc, limit_CW_enc, range_enc);
fprintf('  Gear ratio (Dxl range / Enc range): %.4f\n', gear_ratio);
fprintf('  Midpoint  | Dxl: %+.3f  Enc: %+.3f\n', mid_dxl, mid_enc);
fprintf('  Test 0 targets (encoder) | Start: %.3f  End: %.3f deg\n', test0_start_enc, test0_end_enc);
fprintf('==========================================\n\n');

%% Pause for user confirmation
input('  >> Press Enter to move to midpoint and start Test 0... ', 's');

%% ================================================================
%% MOVE TO MIDPOINT BEFORE TEST 0
%% ================================================================

fprintf('\nMoving to midpoint...\n');

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
fprintf('At midpoint. Ready.\n\n');

%% ================================================================
%% ZERO BOTH SENSORS AT MIDPOINT
%% ================================================================

fprintf('Zeroing both sensors at midpoint...\n');

% Read current Dynamixel position as the zero reference
raw_pos_zero = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
dxl_zero_counts = typecast(uint32(raw_pos_zero), 'int32');
dxl_zero_deg    = double(dxl_zero_counts) * 0.00068392;

% Read current encoder position as the zero reference
% Flush buffer first to get a fresh reading
flush(encoder);
pause(0.2);
enc_zero_deg = read_encoder(encoder);

fprintf('  Dxl zero reference : %.4f deg\n', dxl_zero_deg);
fprintf('  Enc zero reference : %.4f deg\n', enc_zero_deg);
fprintf('Zeroing complete. Both sensors now read 0 at midpoint.\n\n');

input('  >> Press Enter to start Test 0... ', 's');

%% ================================================================
%% PHASE 2 — TEST 0: BASELINE CHARACTERIZATION
%% ================================================================

fprintf('==========================================\n');
fprintf('  PHASE 2: Test 0 — Baseline (%d cycles)\n', TEST0_CYCLES);
fprintf('==========================================\n\n');

%% Switch to velocity mode for encoder-based motion
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

%% Pre-allocate log
maxSamples    = TEST0_CYCLES * 500;
log_cycle     = zeros(1, maxSamples);
log_time      = zeros(1, maxSamples);
log_cmd_pos   = zeros(1, maxSamples);
log_dxl_pos   = zeros(1, maxSamples);
log_enc_pos   = zeros(1, maxSamples);
log_trans_err = zeros(1, maxSamples);
log_current   = zeros(1, maxSamples);
k  = 1;
t0 = tic;

for cycle = 1:TEST0_CYCLES

    fprintf('Cycle %d of %d\n', cycle, TEST0_CYCLES);

    % Two legs: go to END (+50 deg) then back to START (-50 deg)
    targets_enc = [+TEST0_RANGE_DEG/2, -TEST0_RANGE_DEG/2];
    directions  = [+1, -1];   % CW to reach end, CCW to reach start

    for leg = 1:2

        target_enc = targets_enc(leg);
        vel_cmd    = directions(leg) * 2600; %set test velocity

        set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, vel_cmd);

        while true

            if estop_pressed(fig)
                set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
                save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                    log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                    log_enc_pos, log_trans_err, log_current, k, ...
                    gear_ratio, range_dxl, range_enc);
                return;
            end

            %% Read Dynamixel position (zeroed)
            raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
            pos_deg  = double(typecast(uint32(raw_pos), 'int32')) * 0.00068392 - dxl_zero_deg;
            
            %% Read encoder (zeroed)
            enc_deg  = read_encoder(encoder) - enc_zero_deg;
            
            %% Read current
            raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
            curr     = double(typecast(uint16(raw_curr), 'int16'));
            
            %% Transmission error: expected output - actual output
            expected_output = pos_deg / gear_ratio;
            TE = expected_output - enc_deg;

            %% Log
            if k <= maxSamples
                log_cycle(k)     = cycle;
                log_time(k)      = toc(t0);
                log_cmd_pos(k)   = expected_output;
                log_dxl_pos(k)   = pos_deg;
                log_enc_pos(k)   = enc_deg;
                log_trans_err(k) = TE;
                log_current(k)   = curr;
                k = k + 1;
            end

            fprintf('  Cyc%2d Leg%d | Dxl: %+7.2f | Exp.Out: %+7.2f | Enc: %+7.2f | TE: %+6.3f deg | I: %4.0f mA\n', ...
                    cycle, leg, pos_deg, expected_output, enc_deg, TE, curr);

            %% Stop when encoder reaches target within 1 deg tolerance
            if abs(enc_deg - target_enc) < 1.0
                set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
                pause(0.2);
                break;
            end

            pause(SAMPLE_PERIOD);
        end
    end
end

%% ================================================================
%% SAVE AND PLOT RESULTS
%% ================================================================

set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);

save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
    log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
    log_enc_pos, log_trans_err, log_current, k, ...
    gear_ratio, range_dxl, range_enc);

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
    RETRACT_COUNTS       = 400000;
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
%% HELPER: save_and_shutdown
%% ================================================================
function save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
        log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
        log_enc_pos, log_trans_err, log_current, k, ...
        gear_ratio, range_dxl, range_enc)

    k_end = max(k-1, 1);

    %% Main data table
    T = table(log_cycle(1:k_end)', log_time(1:k_end)', ...
              log_cmd_pos(1:k_end)', log_dxl_pos(1:k_end)', ...
              log_enc_pos(1:k_end)', log_trans_err(1:k_end)', ...
              log_current(1:k_end)', ...
        'VariableNames', {'CYCLE','TIME','EXPECTED_OUTPUT_DEG', ...
                          'DXL_POS_DEG','ENC_POS_DEG','TRANS_ERR_DEG','CURRENT_mA'});

    filename = sprintf('test0_baseline_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));

    %% Write main data
    writetable(T, filename);

    %% Append metadata rows at bottom of CSV
    fid = fopen(filename, 'a');
    fprintf(fid, '\n');
    fprintf(fid, 'METADATA\n');
    fprintf(fid, 'GEAR_RATIO,%.6f\n',  gear_ratio);
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

    %% Plots
    t = log_time(1:k_end);

    figure;
    plot(t, log_trans_err(1:k_end));
    xlabel('Time [s]'); ylabel('Transmission Error [deg]');
    title('Test 0 — Transmission Error vs Time'); grid on;

    figure;
    plot(t, log_current(1:k_end));
    xlabel('Time [s]'); ylabel('Current [mA]');
    title('Test 0 — Motor Current vs Time'); grid on;

    figure;
    plot(t, log_cmd_pos(1:k_end)); hold on;
    plot(t, log_enc_pos(1:k_end));
    legend('Expected Output (Dxl/ratio)','Encoder (Actual Output)');
    xlabel('Time [s]'); ylabel('Position [deg]');
    title('Test 0 — Expected vs Actual Output Position'); grid on;

    fprintf('Done.\n');
end