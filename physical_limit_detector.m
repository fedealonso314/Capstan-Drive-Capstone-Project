function  physical_limit_detector()

clc;
clear;

%% Suppress SDK load warnings
warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% ---- E-STOP FIGURE ----
fig = figure('Name', '>>> RUNNING | Click here then press Q to E-STOP <<<', ...
             'Color', [0.1 0.7 0.1], ...
             'NumberTitle', 'off', ...
             'KeyPressFcn', @(src,evt) setappdata(src, 'stop_key', evt.Key));

setappdata(fig, 'stop_key', '');
drawnow;
fprintf('=== E-STOP READY — click the green figure window, then press Q to stop ===\n');

%% Detect OS and library name
lib_name = '';
if strcmp(computer, 'PCWIN'),       lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86'),  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64'),  lib_name = 'libdxl_mac_c';
end

%% Load Dynamixel library
if ~libisloaded(lib_name)
    loadlibrary(lib_name,'dynamixel_sdk.h', ...
        'addheader','port_handler.h', ...
        'addheader','packet_handler.h');
end

%% Dynamixel Model Setup
My_DXL = 'P_SERIES';

switch (My_DXL)
    case {'P_SERIES','PRO_A_SERIES'}
        ADDR_TORQUE_ENABLE          = 512;
        ADDR_GOAL_POSITION          = 564;
        ADDR_PRESENT_POSITION       = 580;
        ADDR_PRESENT_CURRENT        = 574;
        DXL_MINIMUM_POSITION_VALUE  = -90/0.00068392;
        DXL_MAXIMUM_POSITION_VALUE  =  90/0.00068392;
        BAUDRATE = 57600;
end

%% Protocol and device setup
PROTOCOL_VERSION            = 2.0;
DXL_ID                      = 2;
DEVICENAME                  = 'COM8';
ADDR_OPERATING_MODE         = 11;
OPERATING_MODE              = 1; %velocity mode
TORQUE_ENABLE               = 1;
TORQUE_DISABLE              = 0;
DXL_MOVING_STATUS_THRESHOLD = 30;
ESC_CHARACTER               = 'e';
limit_current               = 450;

%% Initialize Port
port_num = portHandler(DEVICENAME);
packetHandler();

if openPort(port_num)
    fprintf('Port opened successfully\n');
else
    error('Failed to open port');
end

if setBaudRate(port_num, BAUDRATE)
    fprintf('Baudrate set successfully\n');
else
    error('Failed to set baudrate');
end

%% Enable torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE);
fprintf('Dynamixel connected\n');

%% Data Logging Setup
maxSamples     = 10000;
currentVector  = zeros(1, maxSamples);
positionVector = zeros(1, maxSamples);
timeVector     = zeros(1, maxSamples);
torqueVector   = zeros(1, maxSamples);
k  = 1;
t0 = tic;

%% ---- MAIN LOOP ----
fprintf('Press e to exit normally, c to read current, Q in green window to E-STOP\n');

while true

    if estop_pressed(fig)
        emergency_shutdown(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
            ADDR_TORQUE_ENABLE, TORQUE_DISABLE, lib_name, ...
            currentVector, positionVector, timeVector, torqueVector, k);
        return;
    end

    try
        value = input('Enter desired angle (d): ', 's');

        if strcmpi(value, ESC_CHARACTER)
            break;
        end

        if strcmpi(value, 'c')
            dxl_present_current = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
            curr = typecast(uint16(dxl_present_current), 'int16');
            fprintf('Current: %d mA\n', curr);
            continue;
        end

        numValue = str2double(value);
        if isnan(numValue), error('Input is not a valid number.'); end
        position_input = int32(numValue / 0.00068392);

    catch ME
        fprintf('Error: %s\n', ME.message);
        continue;
    end

    %% Send goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, ...
                   typecast(int32(position_input), 'uint32'));

    %% Monitor motion
    while true

        if estop_pressed(fig)
            emergency_shutdown(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
                ADDR_TORQUE_ENABLE, TORQUE_DISABLE, lib_name, ...
                currentVector, positionVector, timeVector, torqueVector, k);
            return;
        end

        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        pos = typecast(uint32(dxl_present_position), 'int32') * 0.00068392;

        dxl_present_current = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
        curr = typecast(uint16(dxl_present_current), 'int16');

        if k <= maxSamples
            currentVector(k)  = curr;
            positionVector(k) = pos;
            timeVector(k)     = toc(t0);
            torqueVector(k)   = curr * 0.00269;
            k = k + 1;
        end

        fprintf('Current: %d mA | Position: %.2f deg\n', curr, pos);

        if ~(abs(position_input - typecast(uint32(dxl_present_position), 'int32')) ...
                > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end

        pause(0.01);
    end
end

%% ---- NORMAL EXIT ----
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
fprintf('Torque disabled (normal exit)\n');
closePort(port_num);
unloadlibrary(lib_name);

k_end          = max(k-1, 1);
currentVector  = currentVector(1:k_end);
positionVector = positionVector(1:k_end);
timeVector     = timeVector(1:k_end);
torqueVector   = torqueVector(1:k_end);

T = table(currentVector', positionVector', timeVector', torqueVector', ...
    'VariableNames', {'CURRENT','POSITION','TIME','TORQUE'});
filename = sprintf('myData_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
writetable(T, filename);
fprintf('Data saved to: %s\n', filename);

figure; plot(timeVector, currentVector);
xlabel('Time [s]'); ylabel('Current [mA]'); grid on; title('Motor Current vs Time');

figure; plot(timeVector, positionVector);
xlabel('Time [s]'); ylabel('Position [deg]'); grid on; title('Position vs Time');

figure; plot(positionVector, torqueVector);
xlabel('Position [deg]'); ylabel('Torque [Nm]'); grid on; title('Estimated Torque vs Position');

end % ---- END MAIN FUNCTION ----


%% ================================================================
%% STANDALONE HELPER FUNCTIONS (outside main, receive fig as input)
%% ================================================================

function fired = estop_pressed(fig)
    drawnow;
    fired = strcmpi(getappdata(fig, 'stop_key'), 'q');
end

function emergency_shutdown(fig, port_num, PROTOCOL_VERSION, DXL_ID, ...
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, lib_name, ...
        currentVector, positionVector, timeVector, torqueVector, k)

    fprintf('\n>>> E-STOP FIRED <<<\n');

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    fprintf('Torque DISABLED\n');

    set(fig, 'Color', [0.8 0.1 0.1], 'Name', '>>> E-STOP FIRED — data saved <<<');
    drawnow;

    k_end          = max(k-1, 1);
    currentVector  = currentVector(1:k_end);
    positionVector = positionVector(1:k_end);
    timeVector     = timeVector(1:k_end);
    torqueVector   = torqueVector(1:k_end);

    T = table(currentVector', positionVector', timeVector', torqueVector', ...
        'VariableNames', {'CURRENT','POSITION','TIME','TORQUE'});
    filename = sprintf('ESTOP_myData_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
    writetable(T, filename);
    fprintf('Data saved to: %s  (%d samples)\n', filename, k_end);

    closePort(port_num);
    unloadlibrary(lib_name);
end