clc;
clear;

%% Suppress SDK load warnings
warning('off','MATLAB:loadlibrary:FunctionNotFound');

%% Detect OS and library name
lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
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
        ADDR_TORQUE_ENABLE = 512;
        ADDR_GOAL_POSITION = 564;
        ADDR_PRESENT_POSITION = 580;

        ADDR_PRESENT_CURRENT = 574;

        DXL_MINIMUM_POSITION_VALUE = -90/0.00068392;
        DXL_MAXIMUM_POSITION_VALUE = 90/0.00068392;

        BAUDRATE = 57600;
end

%% Protocol and device setup

PROTOCOL_VERSION = 2.0;
DXL_ID = 1;

DEVICENAME = 'COM8';

ADDR_OPERATING_MODE = 11;
OPERATING_MODE = 4;

TORQUE_ENABLE = 1;
TORQUE_DISABLE = 0;

DXL_MOVING_STATUS_THRESHOLD = 30;

COMM_SUCCESS = 0;

ESC_CHARACTER = 'e';

%% Initialize Port

port_num = portHandler(DEVICENAME);
packetHandler();

dxl_comm_result = COMM_SUCCESS;
dxl_error = 0;

%% Open port

if openPort(port_num)
    fprintf('Port opened successfully\n');
else
    error('Failed to open port');
end

%% Set baudrate

if setBaudRate(port_num,BAUDRATE)
    fprintf('Baudrate set successfully\n');
else
    error('Failed to set baudrate');
end

%% Enable torque and set mode

write1ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID,ADDR_OPERATING_MODE,OPERATING_MODE);
write1ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID,ADDR_TORQUE_ENABLE,TORQUE_ENABLE);

fprintf('Dynamixel connected\n');

%% Data Logging Setup

maxSamples = 10000;

currentVector = zeros(1,maxSamples);
positionVector = zeros(1,maxSamples);
timeVector = zeros(1,maxSamples);
torqueVector = zeros(1,maxSamples);

k = 1;

t0 = tic;

%% Main loop

fprintf('Press e to terminate program and c to read current\n');

% Limit current declaration

limit_current = 350; 


while true
    
    %% User input position

    try
        value = input('Enter desired angle (d): ', 's'); 

        if value == ESC_CHARACTER
            break
        end

        if value == 'c'
            dxl_present_current = read2ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID,ADDR_PRESENT_CURRENT);
            curr = typecast(uint16(dxl_present_current),'int16');
            fprintf('Current: %d mAh\n', curr); 
        end

        numValue = str2double(value);
        position_input = int32(numValue);
        position_input = position_input/0.00068392;
        
        if isnan(numValue) 
            error('Input is not a valid number.');
        end
    
    catch ME
        fprintf('Error: %s\n', ME.message);
    end

    

    %% Safety clamp

    %position_input = max(DXL_MINIMUM_POSITION_VALUE, min(position_input,DXL_MAXIMUM_POSITION_VALUE));

    %% Send goal position

    write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID, ADDR_GOAL_POSITION,typecast(int32(position_input),'uint32'));

    %% Monitor motion

    while true


        %% Read position

        dxl_present_position = read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID,ADDR_PRESENT_POSITION);
        pos = typecast(uint32(dxl_present_position),'int32')*0.00068392;

        %% Read current

        dxl_present_current = read2ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID,ADDR_PRESENT_CURRENT);
        curr = typecast(uint16(dxl_present_current),'int16');
        
        
        % CURRENT LIMITER
        % 
        % if (abs(curr) > abs(limit_current))
        %     goal_current = 0; 
        %     write1ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID,ADDR_OPERATING_MODE,0);
        %     write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, 550, typecast(int16(goal_current), 'uint16'));
        %     break
        % end



        %% Log data

        if k <= maxSamples

            currentVector(k) = curr;
            positionVector(k) = pos;
            timeVector(k) = toc(t0);

            %% Torque estimation (P-series approx)

            torqueVector(k) = curr * 0.00269;

            k = k + 1;

        end

        %% Print info

        fprintf('Current: %d mA | Position: %.2f deg\n',curr,pos);

        %% Check if target reached

        if ~(abs(position_input - typecast(uint32(dxl_present_position),'int32')) ...
                > DXL_MOVING_STATUS_THRESHOLD)
            break
        end

        pause(0.01)

    end

end

%% Disable torque

write1ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID,ADDR_TORQUE_ENABLE,TORQUE_DISABLE);

fprintf('Torque disabled\n')

%% Close port

closePort(port_num);

unloadlibrary(lib_name);

%% Trim vectors

currentVector = currentVector(1:k-1);
positionVector = positionVector(1:k-1);
timeVector = timeVector(1:k-1);
torqueVector = torqueVector(1:k-1);



%% Store data 

T = table(currentVector', positionVector', timeVector', ...
    'VariableNames', {'CURRENT', 'POSITION', 'TIME'}); 

filename = sprintf('myData_%s.csv', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));

writetable(T, filename);


%% Plot results

figure
plot(timeVector,currentVector)
xlabel('Time [s]')
ylabel('Current [mA]')
grid on
title('Motor Current vs Time')

figure
plot(timeVector,positionVector)
xlabel('Time [s]')
ylabel('Position [deg]')
grid on
title('Position vs Time')

figure
plot(positionVector,torqueVector)
xlabel('Position [deg]')
ylabel('Torque [Nm]')
grid on
title('Estimated Torque vs Position')