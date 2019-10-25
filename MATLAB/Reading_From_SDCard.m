close all
clc

%data = csvread('exampleACCEL.CSV', 1, 0); % csvread(Filename,row offset,column offset)
data = csv2struct('exampleACCEL.CSV');
for i=1:length(data)
    data(i).Time= data(i).Time - data(i).Time(1);
end

varlen = size(data.xAccel);
xVelocity = zeros(varlen(1),1);

BNO055_SAMPLERATE_DELAY_MS = 136; % how often to read data from the board
ACCEL_VEL_TRANSITION = BNO055_SAMPLERATE_DELAY_MS/1000; % gives us value in per second
% velocity = accel*dt (dt in seconds)

for i=1:length(data)
   xVelocity(:,i)=ACCEL_VEL_TRANSITION.*data(i).xAccel./cos((data(i).xOrient).*(pi/180));
end
plot(data.Time, xVelocity);
% subplot(3,1,1)
% plot(data.Time, data.xAccel)
% ylabel('m/s^2')
% subplot(3,1,2)
% plot(data.Time, data.yAccel)
% ylabel('m/s^2')
% subplot(3,1,3)
% plot(data.Time, data.zAccel)
% xlabel('Time (ms)')
% ylabel('m/s^2')
% 
