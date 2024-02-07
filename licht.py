import numpy as np
import matplotlib.pyplot as plt
import pyaudio
from scipy.signal import lfilter, butter
import serial
import time

ser = serial.Serial('COM5', 9600, timeout=1)  # timeout=1s für das Lesen der Daten


# Initialisierung der dynamischen Grenzwerte für die Skalierung
dynamic_min_power = np.inf  # Startwert für das Minimum
dynamic_max_power = -np.inf  # Startwert für das Maximum
window_size = 50  # Größe des gleitenden Fensters für die dynamische Anpassung
recent_powers = np.zeros(window_size)  # Gleitendes Fenster für die Band-Power-Werte

# PyAudio-Einstellungen
SAMPLE_RATE = 44100
CHUNK_SIZE = 2056
FORMAT = pyaudio.paInt16
CHANNELS = 1

# Frequenzbereiche
SUB_BASS = (16, 60)
BASS = (60, 250)
LOWER_MIDRANGE = (250, 500)
MID_RANGE = (500, 2000)
HIGHER_MIDRANGE = (2000, 4000)
HIGH_RANGE = (4000, 6000)
BRILLIANCE = (6000, 20000)


DMX_FIXTURES = [(1, 255), (7, 255), (13, 255), (19, 255)]  # (StartChannel, BrightnessCap)
FIXTURE_PROFILES = [
    (0xFF0000, 0x00000FF),  # Rotes Licht für niedrige Frequenzen
    (0x0000FF, 0x0039000),  # Blaues Licht für etwas höhere Frequenzen
    (0xFF0000, 0x00000FF),  # Rotes Licht wieder für noch höhere Frequenzen
    (0x00FF00, 0xFF00000)   # Grünes Licht für die höchsten Frequenzen
]

# PyAudio-Instanz
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=SAMPLE_RATE, input=True, frames_per_buffer=CHUNK_SIZE)


# FFT-Funktion
def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a


def highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


# Filterparameter
cutoff = SUB_BASS[0]  # Alles unterhalb dieses Wertes abschneiden


# FFT-Funktion mit Fensterfunktion
def fft(audio_data):
    # Fensterfunktion anwenden
    window = np.hamming(len(audio_data))
    audio_data = audio_data * window
    freqs = np.fft.rfftfreq(len(audio_data), 1 / SAMPLE_RATE)
    spectrum = np.abs(np.fft.rfft(audio_data))
    return freqs, spectrum


# Frequenzband-Energie-Funktion
def get_band_energy(freqs, spectrum, band_range):
    band_freqs = (freqs >= band_range[0]) & (freqs <= band_range[1])
    band_energy = np.sum(spectrum[band_freqs])
    return band_energy


def scale_to_dmx(band_powers, dmx_min=0, dmx_max=255):
    global dynamic_min_power, dynamic_max_power, recent_powers
    # Aktualisiere die dynamischen Grenzwerte basierend auf den aktuellen Band-Power-Werten
    current_max_power = np.max(band_powers)
    recent_powers = np.roll(recent_powers, -1)
    recent_powers[-1] = current_max_power

    # Vermeidung der Verwendung von np.min und np.max auf recent_powers direkt,
    # wenn recent_powers nur Nullen enthält, was zu Beginn der Fall sein könnte.
    if np.any(recent_powers > 0):
        dynamic_min_power = min(dynamic_min_power, np.min(recent_powers[recent_powers > 0]))
        dynamic_max_power = max(dynamic_max_power, current_max_power)
    else:
        dynamic_min_power = min(dynamic_min_power, current_max_power)
        dynamic_max_power = max(dynamic_max_power, current_max_power)

    # Stelle sicher, dass dynamic_min_power und dynamic_max_power sinnvolle Werte haben,
    # um Division durch Null oder negative Skalierungen zu vermeiden.
    if dynamic_max_power <= dynamic_min_power:
        dynamic_max_power = dynamic_min_power + 1

    # Skalierung der Band-Power-Werte
    normalized_powers = (band_powers - dynamic_min_power) / (dynamic_max_power - dynamic_min_power)
    dmx_values = np.clip((normalized_powers * (dmx_max - dmx_min)) + dmx_min, dmx_min, dmx_max)

    return dmx_values.astype(int)


def convert_to_dmx_values(band_powers):
    """
    Wandelt Band-Power-Werte in DMX-Werte um, basierend auf Fixture-Profilen.
    """
    dmx_data = {}
    # Durchlaufe jedes Fixture und weise basierend auf dem Profil und der Band-Power Farben zu
    for i, (start_channel, brightness_cap) in enumerate(DMX_FIXTURES):
        # Ermittle die zugehörige Farbe und Frequenz aus dem Profil
        color, _ = FIXTURE_PROFILES[i]

        # Einfache Umwandlung der Farbe in DMX-Werte (vereinfacht)
        red = (color >> 16) & 0xFF
        green = (color >> 8) & 0xFF
        blue = color & 0xFF

        # Skalierung der Farbintensität basierend auf der Band-Power und BrightnessCap
        # Hier nehmen wir an, dass die Band-Power direkt die Helligkeit beeinflusst (vereinfacht)
        brightness_factor = min(max(band_powers[i], 0), brightness_cap) / brightness_cap
        dmx_data[start_channel] = int(red * brightness_factor)
        dmx_data[start_channel + 1] = int(green * brightness_factor)
        dmx_data[start_channel + 2] = int(blue * brightness_factor)

    return dmx_data

def send_dmx_command(channel, value):
    """
    Sendet einen DMX-Befehl an den Arduino und liest die Antwort.
    :param channel: DMX-Kanal (1-512)
    :param value: Wert für den Kanal (0-255)
    """
    command = f'{channel}:{value}\n'.encode()
    ser.write(command)
    print(f"DMX Befehl gesendet: Kanal {channel}, Wert {value}")

    # Warten auf Antwort vom Arduino
    response = ser.readline().decode().strip()  # Lese die Antwort und entferne Whitespaces/Newline
    if response:
        print(f"Antwort vom Arduino: {response}")
    else:
        print("Keine Antwort vom Arduino erhalten.")

try:
    while True:
        data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
        stereo_data = np.frombuffer(data, dtype=np.int16)
        stereo_data = stereo_data / 1e5

        # Aufteilen der Kanäle
        left_channel = stereo_data[0::2]
        right_channel = stereo_data[1::2]

        # Hochpassfilter auf beide Kanäle anwenden
        left_data_filtered = highpass_filter(left_channel, cutoff, SAMPLE_RATE)
        right_data_filtered = highpass_filter(right_channel, cutoff, SAMPLE_RATE)

        # FFT für beide Kanäle durchführen
        l_freqs, l_spectrum = fft(left_data_filtered)
        r_freqs, r_spectrum = fft(right_data_filtered)

        # Energie im linken Band berechnen
        l_band_powers = []
        l_sub_bass_energy = get_band_energy(l_freqs, l_spectrum, SUB_BASS)
        l_band_powers.append(l_sub_bass_energy)
        l_bass_energy = get_band_energy(l_freqs, l_spectrum, BASS)
        l_band_powers.append(l_bass_energy)
        l_lower_midrange_engery = get_band_energy(l_freqs, l_spectrum, LOWER_MIDRANGE)
        l_band_powers.append(l_lower_midrange_engery)
        l_mid_energy = get_band_energy(l_freqs, l_spectrum, MID_RANGE)
        l_band_powers.append(l_mid_energy)
        l_higher_midrange_engery = get_band_energy(l_freqs, l_spectrum, HIGHER_MIDRANGE)
        l_band_powers.append(l_higher_midrange_engery)
        l_high_energy = get_band_energy(l_freqs, l_spectrum, HIGH_RANGE)
        l_band_powers.append(l_high_energy)
        l_brilliance_energy = get_band_energy(l_freqs, l_spectrum, BRILLIANCE)
        l_band_powers.append(l_brilliance_energy)

        print(l_band_powers)
        dmx_values = scale_to_dmx(l_band_powers)
        dmx_values = np.clip((dmx_values * 1.5), a_min=None, a_max=255)
        #command_dmx_values = convert_to_dmx_values(dmx_values)
        #for channel, value in command_dmx_values.items():
        send_dmx_command(2, dmx_values[1])
        send_dmx_command(9, dmx_values[3])
        print("DMX-Werte:", dmx_values)

        # Energie im rechten Band berechnen
        r_sub_bass_energy = get_band_energy(r_freqs, r_spectrum, SUB_BASS)
        r_bass_energy = get_band_energy(r_freqs, r_spectrum, BASS)
        r_lower_midrange_engery = get_band_energy(r_freqs, r_spectrum, LOWER_MIDRANGE)
        r_mid_energy = get_band_energy(r_freqs, r_spectrum, MID_RANGE)
        r_higher_mid = range_engery = get_band_energy(r_freqs, r_spectrum, HIGHER_MIDRANGE)
        r_high_energy = get_band_energy(r_freqs, r_spectrum, HIGH_RANGE)
        r_brilliance_energy = get_band_energy(r_freqs, r_spectrum, BRILLIANCE)

except KeyboardInterrupt:
    print("Analyse beendet")

# Aufräumen
stream.stop_stream()
stream.close()
p.terminate()

