import numpy as np
import pyaudio
import serial
from scipy.signal import butter, sosfilt, bessel
import matplotlib.pyplot as plt

#############################################################################
#                               CONSTANTS                                   #
#############################################################################

FREQUENCIES = [
    (16, 60),  # SUB_BASS
    (60, 250),  # BASS
    (250, 500),  # LOWER_MIDRANGE
    (500, 2000),  # MID_RANGE
    (2000, 4000),  # HIGHER_MIDRANGE
    (4000, 6000),  # HIGH_RANGE
    (6000, 20000)  # BRILLIANCE
]


# PyAudio-Einstellungen
SAMPLE_RATE = 44100
CHUNK_SIZE = 2056
FORMAT = pyaudio.paInt16
CHANNELS = 1

#############################################################################
#                               CONFIGURATION                               #
#############################################################################

DMX_FIXTURES = [(1, 255), (7, 255), (13, 255), (
    19, 255)]  # configured fixtures and their start channels. The maximum amount of supported fixtures is 16.
RGB_COLOR_SET = [
    # profiles that fixtures can assume. Each profile consists of a hex code for color and a hex code for frequencies the fixture should respond to.
    (0x0000FF, 0x00000FF),
    (0xFF0000, 0x0039000),
    (0xFF0000, 0x00000FF),
    (0x00FF00, 0xFF00000)
]

#############################################################################
#                               SUBSYSTEMS                                  #
#############################################################################

ser = serial.Serial('COM5', 9600, timeout=1)  # start serial port to arduino

# PyAudio-Instanz
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=SAMPLE_RATE, input=True, frames_per_buffer=CHUNK_SIZE)

# Matplotlib-Fenster konfigurieren
plt.ion()
fig, axs = plt.subplots(len(FREQUENCIES), 1, figsize=(10, 10))

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    print(low, high)
    sos = bessel(order, [low, high], analog=False, btype='band', output='sos')
    return sos


def butter_bandpass_filter(data, fs, order=5):
    y = []

    for band in FREQUENCIES:
        sos = butter_bandpass(band[0], band[1], fs, order=order)
        y.append(sosfilt(sos, data))
    return y

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
        left_data_filtered = butter_bandpass_filter(left_channel, SAMPLE_RATE)
        right_data_filtered = butter_bandpass_filter(right_channel, SAMPLE_RATE)

        for i, (low, high) in enumerate(FREQUENCIES):
            axs[i].clear()
            axs[i].plot(left_data_filtered[i])
            axs[i].set_title(f'Band {i + 1}: {low}-{high} Hz')

        plt.pause(0.05)  # Kurze Pause, um das Plotting nicht zu überlasten



except KeyboardInterrupt:
    print("Analyse beendet")

# Aufräumen
stream.stop_stream()
stream.close()
p.terminate()
