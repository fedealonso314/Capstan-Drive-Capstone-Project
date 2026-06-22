function [limit_CCW, limit_CW, limit_CCW_enc, limit_CW_enc] = find_capstan_limits()

clc;
clear;

warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% Encoder setup
encoder = serialport("COM7", 115200);
flush(encoder);
fprintf('Encoder serial port opened\n');

%% ---- E-STOP FIGURE ----
fig = figure('Name','>>> LIMIT SEARCH | Click here then press Q to E-STOP <<<', ...
             'Color',[0.1 0.7 0.1], ...
             'NumberTitle','off', ...
             'MenuBar','none', ...
             'ToolBar','none', ...
             'KeyPressFcn',@(src,evt) setappdata(src,'stop_key',evt.Key));

setappdata(fig,'stop_key','');
drawnow;
fprintf('=== E-STOP READY — click the green figure window, then press Q to stop ===\n');

%% ================================================================
%% TUNING PARAMETERS
%% ================================================================

SEARCH_VELOCITY      = 300;
CURRENT_ABS_LIMIT    = 350;
CURRENT_SPIKE_DELTA  = 150;
SPIKE_WINDOW         = 20;
TIMEOUT_SECONDS      = 120;
RETRACT_COUNTS       = 50000;
SAMPLE_PERIOD        = 0.02;

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

%% P-series addresses
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
OPERATING_MODE_VEL    = 1;
OPERATING_MODE_POS    = 4;

%% Open port
port_num = portHandler(DEVICENAME);
packetHandler();

if openPort(port_num)
    fprintf('Port opened\n');
else
    emergency_shutdown(fig, port_num, lib_name, 0, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    error('Failed to open port');
end

if setBaudRate(port_num, BAUDRATE)
    fprintf('Baudrate set\n');
else
    emergency_shutdown(fig, port_num, lib_name, 0, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    error('Failed to set baudrate');
end

%% Set velocity mode and enable torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE_VEL);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);
fprintf('Velocity mode enabled\n\n');

%% ================================================================
%% STEP 1 — FIND COUNTERCLOCKWISE LIMIT
%% ================================================================

fprintf('--- Searching CCW limit (negative velocity)... ---\n');

[limit_CCW, limit_CCW_enc, found_CCW] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
    -SEARCH_VELOCITY, ...
    CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, ...
    TIMEOUT_SECONDS, SAMPLE_PERIOD);

if ~found_CCW
    fprintf('[ERROR] CCW limit not found within timeout.\n');
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end

fprintf('[OK] CCW limit found: %.4f deg (Dynamixel) | %.4f deg (Encoder)\n\n', limit_CCW, limit_CCW_enc);

%% Stop and retract away from CCW limit (move CW)
set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
pause(0.5);
retract(port_num, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, ...
    TORQUE_ENABLE, RETRACT_COUNTS, +1);

%% ================================================================
%% STEP 2 — FIND CLOCKWISE LIMIT
%% ================================================================

fprintf('--- Searching CW limit (positive velocity)... ---\n');

[limit_CW, limit_CW_enc, found_CW] = search_limit(fig, encoder, port_num, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_GOAL_VELOCITY, ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT, ...
    +SEARCH_VELOCITY, ...
    CURRENT_ABS_LIMIT, CURRENT_SPIKE_DELTA, SPIKE_WINDOW, ...
    TIMEOUT_SECONDS, SAMPLE_PERIOD);

if ~found_CW
    fprintf('[ERROR] CW limit not found within timeout.\n');
    emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
    return;
end

fprintf('[OK] CW limit found: %.4f deg (Dynamixel) | %.4f deg (Encoder)\n\n', limit_CW, limit_CW_enc);

%% Stop and retract away from CW limit (move CCW)
set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
pause(0.5);
retract(port_num, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, ...
    TORQUE_ENABLE, RETRACT_COUNTS, -1);

%% ================================================================
%% RESULTS
%% ================================================================

range_dxl = abs(limit_CW     - limit_CCW);
range_enc = abs(limit_CW_enc - limit_CCW_enc);

fprintf('==========================================\n');
fprintf('  --- Dynamixel ---\n');
fprintf('  CCW limit  : %+.4f deg\n', limit_CCW);
fprintf('  CW  limit  : %+.4f deg\n', limit_CW);
fprintf('  Total range: %.4f deg\n',  range_dxl);
fprintf('  --- Encoder ---\n');
fprintf('  CCW limit  : %+.4f deg\n', limit_CCW_enc);
fprintf('  CW  limit  : %+.4f deg\n', limit_CW_enc);
fprintf('  Total range: %.4f deg\n',  range_enc);
fprintf('==========================================\n');

%% ================================================================
%% GO TO MIDDLE POSITION
%% ================================================================

mid_dxl = (limit_CCW + limit_CW) / 2;
mid_enc = (limit_CCW_enc + limit_CW_enc) / 2;

fprintf('Moving to middle position: %.4f deg (Dynamixel) | %.4f deg (Encoder)\n', mid_dxl, mid_enc);

%% Switch to position mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE_POS);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);

%% Convert mid angle to counts and send
mid_counts = int32(mid_dxl / 0.00068392);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
               typecast(mid_counts, 'uint32'));

%% Wait until position is reached
while true
    if estop_pressed(fig)
        fprintf('\n>>> E-STOP pressed during centering move <<<\n');
        emergency_shutdown(fig, port_num, lib_name, 1, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder);
        return;
    end

    raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    pos_now  = typecast(uint32(raw_pos), 'int32');
    enc_now  = read_encoder(encoder);

    fprintf('  Moving to centre | Dxl: %+.3f deg | Encoder: %+.3f deg\n', ...
            double(pos_now) * 0.00068392, enc_now);

    if abs(pos_now - mid_counts) < 30   % 30 counts threshold ~ same as DXL_MOVING_STATUS_THRESHOLD
        break;
    end

    pause(0.02);
end

fprintf('[OK] Centred at %.4f deg (Dynamixel) | %.4f deg (Encoder)\n', ...
        double(pos_now) * 0.00068392, enc_now);



%% ---- NORMAL SHUTDOWN ----
set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
pause(0.3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
fprintf('Torque disabled. Done.\n');
closePort(port_num);
unloadlibrary(lib_name);
clear encoder;
set(fig,'Color',[0.2 0.2 0.8],'Name','>>> DONE <<<');

end % ---- END MAIN ----


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

        %% E-stop check
        if estop_pressed(fig)
            fprintf('\n>>> E-STOP pressed during limit search <<<\n');
            set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
            found = false;
            return;
        end

        %% Read current
        raw_curr = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
        curr     = double(typecast(uint16(raw_curr), 'int16'));

        %% Read Dynamixel position
        raw_pos  = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        pos_deg  = double(typecast(uint32(raw_pos), 'int32')) * 0.00068392;

        %% Read encoder
        encoder_angle = read_encoder(encoder);

        %% Print
        fprintf('  Pos: %+8.3f deg | Current: %5.0f mA | Encoder: %+8.3f deg\n', ...
                pos_deg, curr, encoder_angle);

        %% Update rolling baseline
        history(end+1) = abs(curr); %#ok<AGROW>
        if length(history) > SPIKE_WINDOW
            history = history(end-SPIKE_WINDOW+1:end);
        end

        %% Detection — absolute threshold
        if abs(curr) > ABS_LIMIT
            fprintf('[LIMIT] Absolute current threshold hit: %d mA\n', curr);
            limit_deg = pos_deg;
            limit_enc = encoder_angle;
            found     = true;
            break;
        end

        %% Detection — relative spike
        if length(history) == SPIKE_WINDOW
            baseline = mean(history(1:end-1));
            if (abs(curr) - baseline) > SPIKE_DELTA
                fprintf('[LIMIT] Current spike detected: %.0f mA above baseline %.0f mA\n', ...
                        abs(curr) - baseline, baseline);
                limit_deg = pos_deg;
                limit_enc = encoder_angle;
                found     = true;
                break;
            end
        end

        pause(SAMPLE_PERIOD);
    end

    %% Stop velocity
    set_velocity(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_VELOCITY, 0);
end


%% ================================================================
%% HELPER: retract
%% ================================================================
function retract(port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_OPERATING_MODE, ADDR_TORQUE_ENABLE, ...
        ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, ...
        TORQUE_ENABLE, RETRACT_COUNTS, direction)

    fprintf('  Retracting from limit...\n');

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
    fprintf('  Retract complete.\n');
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


