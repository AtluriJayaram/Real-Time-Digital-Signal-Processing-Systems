close all; clear all;

% define COM port #, change to your own serial port number
s1 = serialport('/dev/cu.usbserial-14610', 115200);
s1.Timeout = 2000;
s1.Parity='none';
s1.DataBits = 8;
s1.StopBits = 1;

fs = 16000; % sampling frequency of the board
SIZE = 512; % number of FFT points received = board FFT_SIZE/2, only plot half size FFT
step_size = fs/2/SIZE; % spectrum from 0 to fs/2 at step_size
x = 0:step_size:(fs/2-1); % label x axis


% FFT plot only has positive frequencies (negative frequencies are conjugate symmetric)
% plot a bar chart
b = bar(x, zeros(1, SIZE));
ylim([0 100]); % limit the y axis range


while (ishandle(b))
    data = read(s1,SIZE+1,"uint8") % read serial data
    if data(1) ~= 255 % frame start: 0xFF
        % to avoid data error:
        % 1) put IDE in debugging mode, do not run
        % 2) start this matlab script
        % 3) run the program on board from IDE

        disp('Data alignment error!');
    end
    
    if (ishandle(b))
        set(b,'YData',data(2:SIZE+1));
    end
   
end

clear s1;
disp('Session Terminated...');


