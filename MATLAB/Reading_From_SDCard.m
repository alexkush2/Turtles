close all
clc

%data = csvread('exampleACCEL.CSV', 1, 0); % csvread(Filename, row offset, column offset
data = csv2struct('exampleACCEL.CSV');
for i=1:length(data)
    data(i).Time= data(i).Time - data(i).Time(1);
end

subplot(3,1,1)
plot(data.Time, data.xAccel)
ylabel('m/s^2')
subplot(3,1,2)
plot(data.Time, data.yAccel)
ylabel('m/s^2')
subplot(3,1,3)
plot(data.Time, data.zAccel)
xlabel('Time (ms)')
ylabel('m/s^2')

