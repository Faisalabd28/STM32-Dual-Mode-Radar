%% STM32 Dual-Mode Radar Visualization
% This script establishes a non-blocking, dual-port serial connection to an STM32.
% It listens for incoming UART data (USB or Bluetooth) and dynamically visualizes 
% the ultrasonic radar sweep on a live polar plot.

clear; clc; close all;

% --- 1. System Configuration ---
% Serial Port Settings
portName1 = 'COM5';      % Primary port (USB-to-TTL)
baudRate1 = 115200;      
portName2 = 'COM12';     % Fallback port (HC-06 Bluetooth)
baudRate2 = 9600;        

% Radar & Actuator Parameters
maxDistance = 80;        % Maximum plot radius in cm
servoMin = 500;          % PWM pulse width for 0 degrees
servoMax = 2500;         % PWM pulse width for 180 degrees

% Suppress readline timeout warnings for clean terminal execution
warning('off', 'serialport:serialport:ReadlineWarning');

% --- 2. Initialize Serial Ports ---
% Opens both ports simultaneously. Timeouts are set to 0.1s to ensure the 
% main loop remains non-blocking during port polling.
fprintf('Initializing hardware ports...\n');

try 
    dev1 = serialport(portName1, baudRate1, "Timeout", 0.1); 
    configureTerminator(dev1, "CR/LF"); flush(dev1);
    fprintf('Successfully connected to Primary Port: %s\n', portName1);
catch
    dev1 = []; fprintf('Warning: Could not open %s\n', portName1);
end

try 
    dev2 = serialport(portName2, baudRate2, "Timeout", 0.1); 
    configureTerminator(dev2, "CR/LF"); flush(dev2);
    fprintf('Successfully connected to Fallback Port: %s\n', portName2);
catch
    dev2 = []; fprintf('Warning: Could not open %s\n', portName2);
end

% --- 3. Setup Polar Plot UI ---
figure('Name', 'STM32 Ultrasonic Radar', 'Color', 'k');
ax = polaraxes('Color', 'k', 'ThetaColor', 'w', 'RColor', 'w');
grid on; hold on;
thetalim([0 180]);
rlim([0 maxDistance]);

% Initialize graphical objects for the current beam and historical data points
hBeam = polarplot([0 0], [0 maxDistance], 'g-', 'LineWidth', 2);
hDots = polarplot(0, 0, 'r.', 'MarkerSize', 15);
title('STM32 Ultrasonic Radar', 'Color', 'w', 'FontSize', 14);

% Data State Variables
allAngles = [];
allDistances = [];
prevAngleDeg = -1;
sweepDirection = 0;

fprintf('\nRadar initialized. Awaiting data stream...\n');

% --- 4. Main Visualization Loop ---
while ishandle(hBeam)
    
    % Poll active serial ports for incoming packets
    dataLine = getAvailableData(dev1, dev2);
    
    % Process valid string data
    if strlength(dataLine) > 0
        if contains(dataLine, 'error')
            continue; % Skip corrupt packets
        end
        
        % Parse CSV packet: [PulseWidth, Distance, EchoTime]
        values = str2double(split(dataLine, ','));
        
        if length(values) >= 2 && ~isnan(values(1)) && ~isnan(values(2))
            pulseWidth = values(1);
            distance = values(2);
            
            % Map PWM pulse width to geometric angle (Radians)
            angleDeg = (pulseWidth - servoMin) / (servoMax - servoMin) * 180;
            angleRad = deg2rad(angleDeg);
            
            % Sweep Reversal Detection
            % Clears historical data points when the servo changes direction
            if prevAngleDeg ~= -1
                currentDirection = sign(angleDeg - prevAngleDeg);
                
                if currentDirection ~= 0 && sweepDirection ~= 0 && currentDirection ~= sweepDirection
                    allAngles = [];
                    allDistances = [];
                end
                
                if currentDirection ~= 0
                    sweepDirection = currentDirection;
                end
            end
            prevAngleDeg = angleDeg;
            
            % Update real-time beam position
            set(hBeam, 'ThetaData', [angleRad angleRad], 'RData', [0 maxDistance]);
            
            % Append and plot valid distance measurements within range
            if distance > 0 && distance <= maxDistance
                allAngles = [allAngles, angleRad];
                allDistances = [allDistances, distance];
                set(hDots, 'ThetaData', allAngles, 'RData', allDistances);
            end
            
            drawnow limitrate;
        end
    else
        % Throttle loop to prevent CPU saturation when no data is available
        pause(0.01); 
    end
end

% --- 5. Clean Disconnect ---
if ~isempty(dev1), clear dev1; end
if ~isempty(dev2), clear dev2; end
fprintf('Visualization terminated. Ports released.\n');

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function dataLine = getAvailableData(dev1, dev2)
    % GETAVAILABLEDATA Polls multiple serial objects for available bytes.
    % Returns the raw string line if data exists, handling incomplete transmissions
    % by flushing the active buffer to prevent read locks.
    
    dataLine = ""; 
    
    try
        % Priority 1: USB Connection
        if ~isempty(dev1) && dev1.NumBytesAvailable > 0
            dataLine = string(readline(dev1));
            
            % Clear buffer of fragmented data if line termination was not found
            if strlength(dataLine) == 0
                flush(dev1);
            end
            
        % Priority 2: Bluetooth Connection
        elseif ~isempty(dev2) && dev2.NumBytesAvailable > 0
            dataLine = string(readline(dev2));
            
            % Clear buffer of fragmented data if line termination was not found
            if strlength(dataLine) == 0
                flush(dev2);
            end
        end
    catch
        % Suppress hardware disconnect faults during active reads
        dataLine = "";
    end
end