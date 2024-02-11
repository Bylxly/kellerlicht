import numpy as np
import pyaudio
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter

# PyAudio-Einstellungen
SAMPLE_RATE = 44100
CHUNK_SIZE = 2056
FORMAT = pyaudio.paInt16
CHANNELS = 1

# PyAudio-Instanz
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=SAMPLE_RATE, input=True, frames_per_buffer=CHUNK_SIZE)

plt.ion()
fig, ax = plt.subplots(1, 1)

# Definiere die Frequenzbänder des MSGEQ7
bands = [(63, 160), (160, 400), (400, 1000), (1000, 2500), (2500, 6250), (6250, 16000), (16000, 20000)]
band_names = ['63Hz', '160Hz', '400Hz', '1kHz', '2.5kHz', '6.25kHz', '16kHz']


def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a


def highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


def fft(audio_data):
    # Fensterfunktion anwenden
    window = np.hamming(len(audio_data))
    audio_data = audio_data * window
    freqs = np.fft.rfftfreq(len(audio_data), 1 / SAMPLE_RATE)
    amps = np.abs(np.fft.rfft(audio_data))
    return freqs, amps


# def fft_self(audio_data):


try:
    while True:
        data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
        stereo_data = np.frombuffer(data, dtype=np.int16)

        # Aufteilen der Kanäle
        # left_channel = stereo_data[0::2]
        # right_channel = stereo_data[1::2]

        normalized_tone = stereo_data / (np.max(np.abs(stereo_data)) + 1e-5)

        # Definieren Sie die Cutoff-Frequenz für den Hochpassfilter (63 Hz)
        cutoff_frequency = 63

        # Filtern Sie das normalisierte Signal
        filtered_signal = highpass_filter(normalized_tone, cutoff_frequency, SAMPLE_RATE)

        # Hochpassfilter auf beide Kanäle anwenden
        freqs, amps = fft(filtered_signal)

        # Initialisiere eine Liste für die Bandamplituden
        band_amplitudes = []

        for low, high in bands:
            # Finde die Indizes der Frequenzen im gewünschten Band
            idx = np.where((freqs >= low) & (freqs <= high))
            # Berechne die durchschnittliche Amplitude im Band
            amplitude = np.mean(np.abs(amps[idx]))
            band_amplitudes.append(amplitude)

        ax.clear()
        ax.bar(band_names, band_amplitudes, color="skyblue")
        ax.set(xlabel="Frequency Band", ylabel="Amplitude")
        ax.set_ylim(0, 100)

        plt.pause(0.01)

except KeyboardInterrupt:
    print("Analyse beendet")

# Aufräumen
stream.stop_stream()
stream.close()
p.terminate()
plt.close()
