%% ================================================================
%% HELPER: emergency_shutdown
%% ================================================================
function emergency_shutdown(fig, port_num, lib_name, port_open, ...
        PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, encoder)

    fprintf('\n>>> EMERGENCY SHUTDOWN <<<\n');
    set(fig,'Color',[0.8 0.1 0.1],'Name','>>> E-STOP FIRED <<<');
    drawnow;

    if port_open
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
        fprintf('Torque disabled\n');
        closePort(port_num);
    end

    if libisloaded(lib_name)
        unloadlibrary(lib_name);
    end

    clear encoder;
end