function baseline_test()

clc;
clear;

warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% ================================================================
%% TUNING PARAMETERS
%% ================================================================

NUM_LIMIT_SEARCHES  = 3;      % number of times to find limits
TEST0_CYCLES        = 20;     % back-and-forth cycles for baseline
TEST0_RANGE_DEG     = 100;    % degrees on big drum (encoder)
SAMPLE_PERIOD       = 0.02;   % seconds between samples

%% ================================================================
%% E-STOP FIGURE — created here, passed into all functions
%% ================================================================

fig = figure('Name','>>> BASELINE TEST | Click here then press Q to E-STOP <<<', ...
             'Color',[0.1 0.7 0.1], ...
             'NumberTitle','off', ...
             'MenuBar','none', ...
             'ToolBar','none', ...
             'KeyPressFcn',@(src,evt) setappdata(src,'stop_key',evt.Key));
setappdata(fig,'stop_key','');
drawnow;
fprintf('=== E-STOP READY — click the green figure window, then press Q to stop ===\n\n');

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
        input('', 's');   % waits for Enter, prints nothing
    end


    % E-stop check between runs
    if estop_pressed(fig)
        emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
        return;
    end
end

%% Average limits across 3 runs
limit_CCW     = mean(all_CCW);
limit_CW      = mean(all_CW);
limit_CCW_enc = mean(all_CCW_enc);
limit_CW_enc  = mean(all_CW_enc);
range_dxl     = abs(limit_CW     - limit_CCW);
range_enc     = abs(limit_CW_enc - limit_CCW_enc);
mid_dxl       = (limit_CCW     + limit_CW)     / 2;
mid_enc       = (limit_CCW_enc + limit_CW_enc) / 2;

fprintf('==========================================\n');
fprintf('  AVERAGED LIMITS (over %d runs)\n', NUM_LIMIT_SEARCHES);
fprintf('  Dynamixel | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW, limit_CW, range_dxl);
fprintf('  Encoder   | CCW: %+.3f  CW: %+.3f  Range: %.3f deg\n', limit_CCW_enc, limit_CW_enc, range_enc);
fprintf('  Midpoint  | Dxl: %+.3f  Enc: %+.3f\n', mid_dxl, mid_enc);
fprintf('==========================================\n\n');

%% Compute Test 0 target positions on encoder (centred on midpoint)
test0_start_enc = mid_enc - TEST0_RANGE_DEG / 2;
test0_end_enc   = mid_enc + TEST0_RANGE_DEG / 2;

fprintf('  Test 0 encoder targets:\n');
fprintf('  Start: %.3f deg | End: %.3f deg\n\n', test0_start_enc, test0_end_enc);

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
input('  >> Press Enter to  start Test 0... ', 's');

%% ================================================================
%% PHASE 2 — TEST 0: BASELINE CHARACTERIZATION
%% ================================================================

fprintf('==========================================\n');
fprintf('  PHASE 2: Test 0 — Baseline (%d cycles)\n', TEST0_CYCLES);
fprintf('==========================================\n\n');

%% Pre-allocate log
maxSamples     = TEST0_CYCLES * 500;
log_cycle      = zeros(1, maxSamples);
log_time       = zeros(1, maxSamples);
log_cmd_pos    = zeros(1, maxSamples);
log_dxl_pos    = zeros(1, maxSamples);
log_enc_pos    = zeros(1, maxSamples);
log_trans_err  = zeros(1, maxSamples);
log_current    = zeros(1, maxSamples);
k = 1;
t0 = tic;

%% Convert encoder targets to Dynamixel counts
start_counts = int32(test0_start_enc / 0.00068392 * (range_dxl / range_enc));
end_counts   = int32(test0_end_enc   / 0.00068392 * (range_dxl / range_enc));

for cycle = 1:TEST0_CYCLES

    fprintf('Cycle %d of %d\n', cycle, TEST0_CYCLES);

    % Two legs per cycle: go to END then back to START
    targets = [end_counts, start_counts];

    for leg = 1:2

        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                       typecast(targets(leg), 'uint32'));

        while true

            if estop_pressed(fig)
                save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
                    log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
                    log_enc_pos, log_trans_err, log_current, k);
                return;
            end

            %% Read Dynamixel position
            raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
            pos_now  = typecast(uint32(raw_pos), 'int32');
            pos_deg  = double(pos_now) * 0.00068392;

            %% Read current
            raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
            curr     = double(typecast(uint16(raw_curr), 'int16'));

            %% Read encoder
            enc_deg  = read_encoder(encoder);

            %% Transmission error (encoder = actual output, Dynamixel = input)
            cmd_deg  = double(targets(leg)) * 0.00068392;
            TE       = enc_deg - cmd_deg;

            %% Log
            if k <= maxSamples
                log_cycle(k)     = cycle;
                log_time(k)      = toc(t0);
                log_cmd_pos(k)   = cmd_deg;
                log_dxl_pos(k)   = pos_deg;
                log_enc_pos(k)   = enc_deg;
                log_trans_err(k) = TE;
                log_current(k)   = curr;
                k = k + 1;
            end

            fprintf('  Cyc%2d | Dxl: %+7.2f deg | Enc: %+7.2f deg | TE: %+6.2f deg | I: %4.0f mA\n', ...
                    cycle, pos_deg, enc_deg, TE, curr);

            %% Check if target reached
            if abs(pos_now - targets(leg)) < 30
                break;
            end

            pause(SAMPLE_PERIOD);
        end
    end
end

%% ================================================================
%% SAVE AND PLOT RESULTS
%% ================================================================

save_and_shutdown(fig, port_num, lib_name, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder, ...
    log_cycle, log_time, log_cmd_pos, log_dxl_pos, ...
    log_enc_pos, log_trans_err, log_current, k);

end % ---- END MAIN ----


%% ================================================================
%% HELPER: find_capstan_limits
%% ================================================================
function [limit_CCW, limit_CW, limit_CCW_enc, limit_CW_enc] = find_capstan_limits(fig, encoder, port_num, ...
        PROTOCOL_VERSION, DXL_ID, lib_name, ...
        ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE, ADDR_GOAL_VELOCITY, ...
        ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ADDR_GOAL_POSITION, ...
        TORQUE_ENABLE, TORQUE_DISABLE)

    SEARCH_VELOCITY      = 300;
    CURRENT_ABS_LIMIT    = 350;
    CURRENT_SPIKE_DELTA  = 150;
    SPIKE_WINDOW         = 20;
    TIMEOUT_SECONDS      = 120;
    RETRACT_COUNTS       = 100000;
    SAMPLE_PERIOD        = 0.02;

    limit_CCW     = NaN;
    limit_CW      = NaN;
    limit_CCW_enc = NaN;
    limit_CW_enc  = NaN;

    %% Set velocity mode
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

    %% CCW limit
    fprintf('  Searching CCW limit...\n');
    [limit_CCW, limit_CCW_enc, found] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
        -SEARCH_VELOCITY, CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, TIMEOUT_SECONDS, SAMPLE_PERIOD);

    if ~found, return; end
    fprintf('  CCW found: Dxl %+.3f | Enc %+.3f\n', limit_CCW, limit_CCW_enc);

    %% Retract CW
    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
    pause(0.5);
    retract(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ...
            ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, TORQUE_ENABLE, 5000, +1);

    %% CW limit
    fprintf('  Searching CW limit...\n');
    [limit_CW, limit_CW_enc, found] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
        +SEARCH_VELOCITY, CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, TIMEOUT_SECONDS, SAMPLE_PERIOD);

    if ~found, return; end
    fprintf('  CW  found: Dxl %+.3f | Enc %+.3f\n', limit_CW, limit_CW_enc);

    %% Retract CCW back to middle
    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
    pause(0.5);
    retract(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ...
            ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, TORQUE_ENABLE, 5000, -1);
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
    pause(2.0);

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
    fired = strcmpi(getappdata(fig, 'stop_key'), 'q');
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
        log_enc_pos, log_trans_err, log_current, k)

    k_end = max(k-1, 1);

    T = table(log_cycle(1:k_end)', log_time(1:k_end)', ...
              log_cmd_pos(1:k_end)', log_dxl_pos(1:k_end)', ...
              log_enc_pos(1:k_end)', log_trans_err(1:k_end)', ...
              log_current(1:k_end)', ...
        'VariableNames', {'CYCLE','TIME','CMD_POS','DXL_POS','ENC_POS','TRANS_ERR','CURRENT'});

    filename = sprintf('test0_baseline_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
    writetable(T, filename);
    fprintf('\nData saved to: %s  (%d samples)\n', filename, k_end);

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    fprintf('Torque disabled.\n');
    closePort(port_num);
    unloadlibrary(lib_name);
    clear encoder;

    set(fig,'Color',[0.2 0.2 0.8],'Name','>>> DONE <<<');

    %% Plots
    t   = log_time(1:k_end);
    cyc = log_cycle(1:k_end);

    figure;
    plot(t, log_trans_err(1:k_end));
    xlabel('Time [s]'); ylabel('Transmission Error [deg]');
    title('Test 0 — Transmission Error vs Time'); grid on;

    figure;
    plot(t, log_current(1:k_end));
    xlabel('Time [s]'); ylabel('Current [mA]');
    title('Test 0 — Motor Current vs Time'); grid on;

    figure;
    plot(t, log_dxl_pos(1:k_end)); hold on;
    plot(t, log_enc_pos(1:k_end));
    legend('Dynamixel','Encoder');
    xlabel('Time [s]'); ylabel('Position [deg]');
    title('Test 0 — Position vs Time'); grid on;

    fprintf('Done.\n');
end