clear all
close all
clc

data = load("HK_data.txt");
% data = load("240424_HK_data(0to0.8to0m per s).txt");


HA(1) = subplot(2, 1, 1);
plot(data(:,1), data(:,4))
hold on
plot(data(:,1), data(:,5))
grid on
xticklabels(''); 
axis([0 50 0 1])
legend("Target Velocity-x", "Pelv Velocity-x",'Location', 'northeast')
ylabel("Velocity[m/s]")

HA(2) = subplot(2, 1, 2);
plot(data(:,1), data(:,2))
hold on
plot(data(:,1), data(:,3))
plot(data(:,1), data(:,2)+data(:,3))
grid on
axis([0 50 -1500 1500])
legend("RF-FT-z", "LF-FT-z", "SUM", 'Location', 'northeast')
ylabel("Force[N]")
ylabel("Time[s]")

% 
% gapscale = 0; % zero gap
% P = vertcat(HA.Position);
% gap = P(2,1)-(P(1,1)+P(1,3));
% P(1,1) = P(1,1)+gap*(1-gapscale)/2;
% P(2,1) = P(2,1)-gap*(1-gapscale)/2;
% HA(1).Position = P(1,:);
% HA(2).Position = P(2,:);
% HA(2).YTick = [];
gap = -0.1; % zero gap
P = vertcat(HA.Position);
P(2,2) = P(2,2)-gap;
HA(2).Position = P(2,:);