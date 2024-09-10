from pathlib import Path
import soundfile as sf
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfilt

file_path = Path('C:\\Users\\jesto\\Desktop\\enee408i\\2024_Fall_Lab_2\\Sound_Files\\Cafe_with_noise.wav')

y, Fs = sf.read(file_path)
t = np.linspace(0, len(y) / Fs, len(y))

plt.figure(1)
plt.plot(t,y)
plt.title('Original Audio Signal')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.grid(True)
plt.show()

freq_y = np.fft.fft(y)
plt.figure(2)
plt.plot(Fs/len(freq_y) * np.arange(len(freq_y)), np.abs(freq_y))
plt.title('FFT of original signal')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.grid(True)
plt.show()

sos = butter(10, 800, 'low', fs=Fs, output='sos')
y_lowpass = sosfilt(sos,y)
sf.write('filtered_audio.wav',y_lowpass, Fs)

ylpfft = np.fft.fft(y_lowpass)
plt.figure(3)
plt.plot(Fs/ len(ylpfft) * np.arange(len(ylpfft)), np.abs(ylpfft))
plt.title('FFT of filtered signal')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.grid(True)
plt.show()

sf.write('filtered.wav', y_lowpass, Fs)