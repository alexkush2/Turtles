function [data, headingVelocity] = AccelProcess(FILE_NAME)
data = csv2struct(FILE_NAME);
varlen = size(data.xAccel);
headingVelocity = zeros(varlen(1),1);
Time1 = data.Time(1);
xAverage = mean(data.xAccel);
for i=1:length(data.Time)
    data.Time(i) = data.Time(i) - Time1;
    data.xAccel(i) = data.xAccel(i) - xAverage;
    if(i==1)
        BNO055_SAMPLERATE_DELAY_MS = data.Time(i);
    else
        BNO055_SAMPLERATE_DELAY_MS = data.Time(i) - data.Time(i-1); % how often to read data from the board
    end    
    ACCEL_VEL_TRANSITION(i) = BNO055_SAMPLERATE_DELAY_MS/1000; % gives us value in per milisecond
%     headingVelocity(i,1)=ACCEL_VEL_TRANSITION.*data.xAccel(i)./cos((data.xOrient(i)).*(pi/180));
    if(i==1)
        headingVelocity(i,1)=ACCEL_VEL_TRANSITION(i).*data.xAccel(i) + headingVelocity(i,1);
    else
        headingVelocity(i,1)=ACCEL_VEL_TRANSITION(i).*data.xAccel(i) + headingVelocity(i-1,1);
    end
end
end
