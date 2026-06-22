% Replace COM3 with your actual port
s = serialport("COM7", 115200);

% Optional: clear buffer
flush(s);

while true
    if s.NumBytesAvailable > 0
        data = readline(s);              % Read line from ESP32
        angle = str2double(data);       % Convert to number
        
        if ~isnan(angle)
            fprintf("Encoder Angle: %.2f degrees\n", angle);
        else
            fprintf("Invalid data received\n");
        end
    end
    
    pause(1); % Wait 1 second

end
