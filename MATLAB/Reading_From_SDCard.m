close all
clear all
clc


[data1V, headingVelocity1] = AccelProcess('ACCEL_10302019carT3.CSV');
[data2V, headingVelocity2] = AccelProcess('ACCEL_10302019carT4.CSV');
[data3V, headingVelocity3] = AccelProcess('ACCEL_10302019carT5.CSV');
[data1, Orient1] = OrientProcess('ACCEL_10302019carT3.CSV');
[data2, Orient2] = OrientProcess('ACCEL_10302019carT4.CSV');
[data3, Orient3] = OrientProcess('ACCEL_10302019carT5.CSV');

t1=1:length(headingVelocity1);
t2=1:length(headingVelocity2);
t3=1:length(headingVelocity3);
y1=headingVelocity1;
y2=headingVelocity2;
y3=headingVelocity3;
%%

figure(1)
subplot(2,1,1)
h1=animatedline('Color','b','Linewidth',1);
hold on 
plot(t1,y1)
h2=animatedline('Color','g','Linewidth',1);
plot(t2,y2)
hold on
h3=animatedline('Color','r','Linewidth',1);
hold on
plot(t3,y3)
axis([0,1,-4,10])
x = linspace(0,length(data3V),length(headingVelocity1));
ylabel('velocity (m/s)')
xlabel('Time (minutes)')
% legend('T1', 'T2', 'T3')

for i=1:30:length(headingVelocity3)
    
    addpoints(h1,x(i),y1(i));%data1.Time*1e-3, headingVelocity1, 'Linewidth', 2);
    addpoints(h2,x(i),y2(i));
    addpoints(h3,x(i),y3(i));
    drawnow
    [a1,b1]=pol2cart((pi/180)*Orient1(i),1);
    [a2,b2]=pol2cart((pi/180)*Orient2(i),1);
    [a3,b3]=pol2cart((pi/180)*Orient3(i),1);
    subplot(2,1,2)
    compass(a1,b1)
    
end

%% alex's mods

figure(1)
subplot(2,1,1)
h1=animatedline('Color','b','Linewidth',1);
hold on 
plot(t1,y1)
h2=animatedline('Color','g','Linewidth',1);
plot(t2,y2)
hold on
h3=animatedline('Color','r','Linewidth',1);
hold on
grid on
plot(t3,y3)
axis([0,1,-4,10])
x = linspace(0,length(data3V),length(headingVelocity1));
ylabel('velocity (m/s)')
xlabel('Time (minutes)')
% legend('T1', 'T2', 'T3')

subplot(2,1,2);
compass(0,0)


for i=1:30:length(headingVelocity3)
    
    addpoints(h1,x(i),y1(i));%data1.Time*1e-3, headingVelocity1, 'Linewidth', 2);
    addpoints(h2,x(i),y2(i));
    addpoints(h3,x(i),y3(i));
    drawnow
    [a1,b1]=pol2cart((pi/180)*Orient1(i),1);
    [a2,b2]=pol2cart((pi/180)*Orient2(i),1);
    [a3,b3]=pol2cart((pi/180)*Orient3(i),1);
    subplot(2,1,2)
    compass(a1,b1)
    
end
hold on;
[a1,b1]=pol2cart((pi/180)*Orient1(1),1);
compass(a1,b1,'r')
%%
%     plot(data2.Time*1e-3, headingVelocity2, 'Linewidth', 2);
%     plot(data3.Time*1e-3, headingVelocity3, 'Linewidth', 2);
%     ylabel('velocity (m/s)')
%     legend('T1', 'T2', 'T3')
%     subplot(2,1,2)
%     plot(data1.Time*1e-3, data1.xAccel)
%     hold on
%     plot(data2.Time*1e-3, data2.xAccel)
%     plot(data3.Time*1e-3, data3.xAccel)
%     ylabel('acceleration (m/s^2)')
%     xlabel('Time (s)')
%     legend('T1', 'T2', 'T3')
%%
% [data3, Orient3] = OrientProcess('ACCEL_10302019carT5.CSV');
% for i=1:35:length(Orient3)
%     %theta(i) = Orient3(i,1);
%     %rho = 1;%sin(2*theta).*cos(2*theta);
%     [x,y]=pol2cart((2*pi/360)*Orient3(i),1);
%     figure(2)
%     compass(x,y)  % Linewidth?
%     hold off;
% end
%%
% [x,y]=pol2cart((2*pi/360)*Orient3,1);
% for i=1:1:length(Orient3)
%     %theta(i) = Orient3(i,1);
%     %rho = 1;%sin(2*theta).*cos(2*theta);
%     figure(2)
%     compass(x(i),y(i))
%     hold off;
% end

