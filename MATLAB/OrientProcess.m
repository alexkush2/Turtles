function [data, Orientation] = OrientProcess(FILE_NAME)
data = csv2struct(FILE_NAME);
varlen = size(data.xAccel);
Orientation = zeros(varlen(1),1);
Time1 = data.Time(1);
for i=1:length(data.Time)
    data.Time(i) = data.Time(i) - Time1;
    if(i==1)
        BNO055_SAMPLERATE_DELAY_MS = data.Time(i);
    else
        BNO055_SAMPLERATE_DELAY_MS = data.Time(i) - data.Time(i-1); % how often to read data from the board
    end    
    Orient_TRANSITION(i) = BNO055_SAMPLERATE_DELAY_MS/1000; % gives us value in per milisecond
    Orientation(i,1) = data.xOrient(i);
end
end
