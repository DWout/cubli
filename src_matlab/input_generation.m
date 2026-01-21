clear all
T = 1;
Ts = 1/50;      % Sample time
Tsinput = 0.2;  % 
Period = T/Ts;  % Period = T/Ts*Tsinput
%Generate a Sum-of-Sinusoids Signal.
% Specify that the signal has N samples in each period and P periods.
NumPeriod = 10;
Range = [-0.1 0.1];
%Specify the frequency range of the signal between 0 and Nyquist [0 1]
Band = [1 50];
%Now modify the number, frequency, and phase of the sinusoids 
% 2*pi*[1:GridSkip:fix(Period/2)]/Period and the passband pi*Band.
NumSinusoids = 10;
NumTrials = 10;
GridSkip = 2;
SineData = [NumSinusoids,NumTrials,GridSkip];
%Generate the signal using default characteristics for the sine waves.
[u,freq] = idinput([Period 1 NumPeriod],'sine',Band,Range,SineData);

%% Verification
ufft = fft(u);
Fs = 1/Ts;
L = length(u);
w = (0:L-1)*Fs/L;

figure;
subplot(121); stem(w(1:L/2),abs(ufft(1:L/2))) % Plot until Nyquist frequency
subplot(122); stem(w(1:L/2),abs(ufft(1:L/2))) % Just a little zoom
xlim([0 50]);
title('Single-Sided Amplitude Spectrum of u(t)')
xlabel('Frequency (rad/s)')
ylabel('Amplitude')

%Find Peak-Magnitude-to-RMS Ratio of Sinusoid
disp(['Peak-Magnitude-to-RMS Ratio: ' num2str(peak2rms(u))])

%Returns the sum-of-sinusoids signal in u and the frequencies of the sinusoids in freq. 
freq = freq/Ts;

%% Final Signal - For hardware
u = iddata([],u,Tsinput,'TimeUnit','seconds');
figure;
plot(u)