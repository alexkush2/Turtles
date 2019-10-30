close all
clear all
clc

data1 = csv2struct('ACCELinTech.CSV');
varlen = size(data1.xAccel);
headingVelocity1 = zeros(varlen(1),1);
Time1 = data1.Time(1);
xAverage = mean(data1.xAccel);
for i=1:length(data1.Time)
    data1.Time(i) = data1.Time(i) - Time1;
    data1.xAccel(i) = data1.xAccel(i) - xAverage;
    if(i==1)
        BNO055_SAMPLERATE_DELAY_MS = data1.Time(i);
    else
        BNO055_SAMPLERATE_DELAY_MS = data1.Time(i) - data1.Time(i-1); % how often to read data from the board
    end    
    ACCEL_VEL_TRANSITION(i) = BNO055_SAMPLERATE_DELAY_MS/1000; % gives us value in per milisecond
%     headingVelocity(i,1)=ACCEL_VEL_TRANSITION.*data.xAccel(i)./cos((data.xOrient(i)).*(pi/180));
    if(i==1)
        headingVelocity1(i,1)=ACCEL_VEL_TRANSITION(i).*data1.xAccel(i) + headingVelocity1(i,1);
    else
        headingVelocity1(i,1)=ACCEL_VEL_TRANSITION(i).*data1.xAccel(i) + headingVelocity1(i-1,1);
    end
end

data2 = csv2struct('ACCEL_10282019car.CSV');
varlen = size(data2.xAccel);
headingVelocity2 = zeros(varlen(1),1);
Time1 = data2.Time(1);
xAverage = mean(data2.xAccel);
for i=1:length(data2.Time)
    data2.Time(i) = data2.Time(i) - Time1;
    data2.xAccel(i) = data2.xAccel(i) - xAverage;
    if(i==1)
        BNO055_SAMPLERATE_DELAY_MS = data2.Time(i);
    else
        BNO055_SAMPLERATE_DELAY_MS = data2.Time(i) - data2.Time(i-1); % how often to read data from the board
    end    
    ACCEL_VEL_TRANSITION(i) = BNO055_SAMPLERATE_DELAY_MS/1000; % gives us value in per milisecond
%     headingVelocity(i,1)=ACCEL_VEL_TRANSITION.*data.xAccel(i)./cos((data.xOrient(i)).*(pi/180));
    if(i==1)
        headingVelocity2(i,1)=ACCEL_VEL_TRANSITION(i).*data2.xAccel(i) + headingVelocity2(i,1);
    else
        headingVelocity2(i,1)=ACCEL_VEL_TRANSITION(i).*data2.xAccel(i) + headingVelocity2(i-1,1);
    end
end


data3 = csv2struct('ACCEL_10292019car.CSV');
varlen = size(data3.xAccel);
headingVelocity3 = zeros(varlen(1),1);
Time1 = data3.Time(1);
xAverage = mean(data3.xAccel);
for i=1:length(data3.Time)
    data3.Time(i) = data3.Time(i) - Time1;
    data3.xAccel(i) = data3.xAccel(i) - xAverage;
    if(i==1)
        BNO055_SAMPLERATE_DELAY_MS = data3.Time(i);
    else
        BNO055_SAMPLERATE_DELAY_MS = data3.Time(i) - data3.Time(i-1); % how often to read data from the board
    end    
    ACCEL_VEL_TRANSITION(i) = BNO055_SAMPLERATE_DELAY_MS/1000; % gives us value in per milisecond
%     headingVelocity(i,1)=ACCEL_VEL_TRANSITION.*data.xAccel(i)./cos((data.xOrient(i)).*(pi/180));
    if(i==1)
        headingVelocity3(i,1)=ACCEL_VEL_TRANSITION(i).*data3.xAccel(i) + headingVelocity3(i,1);
    else
        headingVelocity3(i,1)=ACCEL_VEL_TRANSITION(i).*data3.xAccel(i) + headingVelocity3(i-1,1);
    end
end

subplot(2,1,1)
% plot(data1.Time, headingVelocity1, 'Linewidth', 2);
% hold on
% plot(data2.Time, headingVelocity2, 'Linewidth', 2);
plot(data3.Time, headingVelocity3, 'Linewidth', 2);
ylabel('velocity (m/s)')
% legend('AccelinTech', 'ACCEL10282019car.CSV', 'ACCEL10292019car.CSV')
subplot(2,1,2)
% plot(data1.Time, data1.xAccel)
% hold on
% plot(data2.Time, data2.xAccel)
plot(data3.Time, data3.xAccel)
ylabel('acceleration (m/s^2)')
xlabel('Time (ms)')
% legend('AccelinTech', 'ACCEL10282019car.CSV', 'ACCEL10292019car.CSV')




