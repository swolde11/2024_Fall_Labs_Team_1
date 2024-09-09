%% 3.1

[y, Fs] = audioread('..\..\2024_Fall_Lab_2\Sound_Files\human_voice.wav');
t = linspace(0, length(y)/Fs, length(y));
t = t';
figure(1)
plot(t, y)

y2 = y(1 : 6 : end);
t2 = linspace(0, length(y2)/8000, length(y2));
t2 = t2'
figure(2)
plot(t2, y2)

size(y2)
size(y)

%% 3.2

m1 = audioread('..\..\2024_Fall_Lab_2\Sound_Files\M1.wav');
m2 = audioread('..\..\2024_Fall_Lab_2\Sound_Files\M2.wav');
m3 = audioread('..\..\2024_Fall_Lab_2\Sound_Files\M3.wav');

length(m1)
length(m2)

rms1 = rms(m1);
rms2 = rms(m2);
rms3 = rms(m3);

% very slow, but works

% m1pad = [m1, zeros(length(m2))];
% m2pad = [m2, zeros(length(m1))];
% 
% for i = 1:(length(m1)+length(m2)-1)
%     Rxy(i) = 0;
%     for j=1:length(m2)
%         if ((i-j+1) > 0)
%             Rxy(i) = Rxy(i) + (m1pad(j)*m2pad(i-j+1));
%         else
%         end
%     end
% end


Rxy = xcorr(m1, m2);

figure(1);
p = plot(Rxy);
p.XData = p.XData / 8000;
p

%% 3.3

[y, Fs] = audioread('..\..\2024_Fall_Lab_2\Sound_Files\Cafe_with_noise.wav');
t = linspace(0, length(y)/Fs, length(y));
t = t';
figure(1)
plot(t, y)

freq_y = fft(y);

figure(2);
plot(Fs/length(freq_y)*(0:(length(freq_y)-1)),abs(freq_y))

y_lowpass = lowpass(y, 800, Fs, 'ImpulseResponse','iir');
sound(y_lowpass, Fs)

ylpfft = fft(y_lowpass);

figure(3);
plot(Fs/length(ylpfft)*(0:(length(ylpfft)-1)),abs(ylpfft))
