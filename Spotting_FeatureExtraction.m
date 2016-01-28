%%10 total features: average angular velocity, average mean shifted
%%acceleration, distribution of power per frequency from 0-8Hz in 8 bins

%%Sample: vectors of ax,ay,az,gx,gy,gz (Mx6)

ax = Sample(:,1);
ay = Sample(:,2);
az = Sample(:,3);
gx = Sample(:,4);
gy = Sample(:,5);
gz = Sample(:,6);

%Sampling frequency - remember to set this based on Arduino sampling
%frequency
Fs = 1000; %Hz
%% Calculate average angular velocity
avg_ang_vel = 0;

[M N] = size(Sample);
for i = 1:M
    avg_ang_vel = avg_ang_vel + sqrt(gx(i)^2 + gy(i)^2 + gz(i)^2);
end
avg_ang_vel = 1/M*avg_ang_vel;

%% Calculate average mean shifted acceleration

avg_accel = 0;
for i = 1:M
    avg_accel = avg_accel + sqrt(ax(i)^2 + ay(i)^2 + az(i)^2)
end
avg_accel = 1/M*avg_accel;

avg_mean_shift_accel = 0;
for i = 1:M
    diff = abs(sqrt(ax(i)^2 + ay(i)^2 + az(i)^2) - avg_accel);
    avg_mean_shift_accel = avg_mean_shift_accel + diff;
end
avg_mean_shift_accel = 1/M*avg_mean_shift_accel;

%% Calculation of power spectral decomposition
% Choose one of the six?

nfft = 1000; %Length of FFT
% Take FFT, padding with zeroes so that length(X) is 1000
X = fft(ax,nfft);

% FFT is symmetric, so throw away second half
X = X(1:nfft/2);

% Take the magnitude of FFT of x
mag_X = abs(X);

% Frequency vector - should extend to at least 8
f = (0:nfft/2-1)*Fs/nfft;

% Find maximum power in each bin
len = length(f);
for j = 1:8
    power_max(j) = 0;
    for k = 1:len
        while ((f(k) > j-1) && (f(k) <= j)) 
            if (X(k) > power_max(j))
                power_max(j) = X(k);
            end
        end
    end
end

% 10 features: avg_ang_vel, avg_mean_shift_accel, power_max (1x8)