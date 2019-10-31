close all
clear all
clc


[data1, headingVelocity1] = AccelProcess('ACCEL_10302019carT3.CSV');
[data2, headingVelocity2] = AccelProcess('ACCEL_10302019carT4.CSV');
[data3, headingVelocity3] = AccelProcess('ACCEL_10302019carT5.CSV');
figure(1)
subplot(2,1,1)
plot(data1.Time*1e-3, headingVelocity1, 'Linewidth', 2);
hold on
plot(data2.Time*1e-3, headingVelocity2, 'Linewidth', 2);
plot(data3.Time*1e-3, headingVelocity3, 'Linewidth', 2);
ylabel('velocity (m/s)')
legend('T1', 'T2', 'T3')
subplot(2,1,2)
plot(data1.Time*1e-3, data1.xAccel)
hold on
plot(data2.Time*1e-3, data2.xAccel)
plot(data3.Time*1e-3, data3.xAccel)
ylabel('acceleration (m/s^2)')
xlabel('Time (s)')
legend('T1', 'T2', 'T3')

export_fig(figure (1))
% figure(2)
% plot(data1.Time*1e-3, data1.zAccel)
% hold on
% plot(data2.Time*1e-3, data2.zAccel)
% plot(data3.Time*1e-3, data3.zAccel)
% xlabel('Time (s)')
% ylabel('(m/s^2)')
% 
