clear;

% dt, motorER, dER, motorIntegratorR, uR
data = dlmread('data1.txt', ' ');
t = cumsum(data(:,1));


figure(1); clf(1); hold on;

plot(t, data(:,2), 'DisplayName', 'e');
plot(t, data(:,4), 'DisplayName', 'int');
plot(t, data(:,5), 'DisplayName', 'u');
legend()