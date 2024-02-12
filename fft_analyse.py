import numpy as np
import pyaudio
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
import helper.NumericHistory as numericHistory
from helper.DMXFixture import DMXFixture, FixtureProfile
import time

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

start_time = time.time()

##################################################
#                    CONSTANTS                   #
##################################################

# Definiere die Frequenzbänder des MSGEQ7
bands = [(63, 160), (160, 400), (400, 1000), (1000, 2500), (2500, 6250), (6250, 16000), (16000, 20000)]
band_names = ['63Hz', '160Hz', '400Hz', '1kHz', '2.5kHz', '6.25kHz', '16kHz']

AUDIO_BAND_MAX = 1023

AMP_FACTOR_MAX = 64.0  # maximum allowed amplifaction factor
AMP_FACTOR_MIN = 0.0078125  # minimal allowed amplification factor (1/128)

DMX_CHANNEL_MAX = 255

TARGET_CLIPPING = 196.0  # target value for fixture cross-frequency duty cycle (time-clipped/time-not-clipped in parts of 1023, e.g. 196=19.2%)

BRIGHTNESS_CAP = 217

PROFILE_CYCLE_PERIOD_MS = 5000  # amount of milliseconds until the profile assignments between lamps is rotated.

##################################################
#                     CONFIG                     #
##################################################

gainModeSetting = 0

fixtures = [
    DMXFixture(1, BRIGHTNESS_CAP),
    DMXFixture(7, BRIGHTNESS_CAP),
    DMXFixture(13, BRIGHTNESS_CAP),
    DMXFixture(19, BRIGHTNESS_CAP)
]

# Definiere die Farbprofilsets
rgb_color_set = [
    FixtureProfile(0xFF0000, 0x00000FF),
    FixtureProfile(0x0000FF, 0x0039000),
    FixtureProfile(0xFF0000, 0x00000FF),
    FixtureProfile(0x00FF00, 0xFF00000)
]

cmy_color_set = [
    FixtureProfile(0x800080, 0x00000FF),
    FixtureProfile(0xA06000, 0xFF00000),
    FixtureProfile(0x800080, 0x00000FF),
    FixtureProfile(0x008080, 0x0039000)
]

cold_color_set = [
    FixtureProfile(0x4B00B4, 0x00000FF),
    FixtureProfile(0x0000FF, 0xFF00000),
    FixtureProfile(0x4B00B4, 0x00000FF),
    FixtureProfile(0x464673, 0x0039000)
]

uwu_color_set = [
    FixtureProfile(0xFF0000, 0x00000FF),
    FixtureProfile(0x71008E, 0xFF00000),
    FixtureProfile(0xFF0000, 0x00000FF),
    FixtureProfile(0xAA0055, 0x0039000)
]

# Berechne die Anzahl der Fixtures und Profile
FIXTURE_AMOUNT = len(fixtures)
PROFILE_AMOUNT = len(rgb_color_set)  # Annahme: Alle Sets haben die gleiche Länge

# Gruppiere die Profilsets in einer Liste von Listen
profile_groups = [rgb_color_set, cmy_color_set, cold_color_set, uwu_color_set]

colorSetSetting = 0

whiteSetting = 0
strobeFrequencySetting = 100
strobeEnabled = False

##################################################
#                      VARS                      #
##################################################

amplificationFactor = 12.0
noiseLevel = 0


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
    # not like MSGEQ7, window is being applied to whole data
    window = np.hamming(len(audio_data))
    audio_data = audio_data * window
    freqs = np.fft.rfftfreq(len(audio_data), 1 / SAMPLE_RATE)
    amps = np.abs(np.fft.rfft(audio_data))
    return freqs, amps


def calculate_signal_mean(band_amplitudes, noiceLevel):
    amp_hist = numericHistory.NumericHistory(32)

    crossBandSignal = getAverage(band_amplitudes, 0) - noiceLevel
    amp_hist.update(np.max(crossBandSignal, 0))

    return getAverage(amp_hist.get(), 0)


def mapAudioAmplitudeToLightLevel(band_amplitudes, band_average):
    bandClippings = np.zeros(len(bands))

    for band in range(0, len(bands)):
        halfSignalWidth = band_amplitudes[band] - band_average
        signalAmplified = max(halfSignalWidth, 0) * amplificationFactor

        if (signalAmplified >= AUDIO_BAND_MAX):
            bandClippings[band] = AUDIO_BAND_MAX

        bandClippings[band] = scale_value(signalAmplified, AUDIO_BAND_MAX, DMX_CHANNEL_MAX)

    return getAverage(bandClippings, 0)


def updateAmplificationFactor(crossBandClipping):
    global amplificationFactor

    clippingHistory = numericHistory.NumericHistory(32)

    clippingHistory.update(crossBandClipping)
    if gainModeSetting == 0:
        clippingDeviation = TARGET_CLIPPING / getAverage(clippingHistory.get(), 0)
        amplificationFactor = max(AMP_FACTOR_MIN, min(clippingDeviation, AMP_FACTOR_MAX))
    elif gainModeSetting == 1:
        amplificationFactor = 0.5
    elif gainModeSetting == 2:
        amplificationFactor = 48.0


def getAverage(array, buffer):
    average = np.mean(array)

    average += buffer
    return np.min((int(average), AUDIO_BAND_MAX))


def scale_value(input_value, input_max, output_max):
    # Skalieren des Eingabewerts vom Eingabebereich [0, input_max] auf den Ausgabebereich [0, output_max]
    scaled_value = (input_value / input_max) * output_max
    # Stellen Sie sicher, dass der Wert im Ausgabebereich bleibt
    scaled_value = min(max(int(scaled_value), 0), output_max)
    return scaled_value


def generate_permutation_code():
    """
    Generates a new permutation code based on the current time and cycle period.
    """
    # Static variables to maintain state across function calls
    global permutation_timestamp, permutation_code
    try:
        permutation_timestamp
        permutation_code
    except NameError:
        # Initialize if not already done
        permutation_timestamp = 0
        permutation_code = 0xFEDCBA9876543210 & ((0x1 << (4 * PROFILE_AMOUNT)) - 0x1)

    # Check if it's time to update the permutation
    current_time_ms = millis()
    if current_time_ms - permutation_timestamp < PROFILE_CYCLE_PERIOD_MS:
        return permutation_code

    # Cycle the permutation code
    permutation_code = (permutation_code >> 4) | ((permutation_code & ((0x1 << 4) - 0x1)) << ((4 * PROFILE_AMOUNT) - 4))
    permutation_timestamp = current_time_ms
    return permutation_code


def permutate_profiles(permutation, permuted_profiles):
    """
    Applies a permutation to a list of profiles based on a permutation code.
    """
    for profile_slot in range(len(profile_groups)):
        profile_source = permutation & 0xF
        permutation >>= 4
        permuted_profiles[profile_slot] = (profile_groups[colorSetSetting][profile_source])
    return permuted_profiles


def millis():
    return int((time.time() - start_time) * 1000)


def setFixtureColor(targetFixture, audioAmplitudes, colorResponse):
    targetFixture.setRGB(colorResponse >> 16, (colorResponse & 0x00FF00) >> 8, colorResponse & 0x0000FF)


def setFixtureBrightness(targetFixture, audioAmplitudes, audioResponse):
    brightness = 0
    observedBands = 0

    for band in range(0, len(bands)):
        bandResponse = ((audioResponse & (0xF << (band * 4))) >> (
                    band * 4))  # get response coefficient (is between 0.0 .. 15.0)
        if bandResponse > 0:
            brightness += (bandResponse / 15.0) * audioAmplitudes[band]
            observedBands += 1

    targetFixture.setRGBDimmer(int(brightness / observedBands))


def setFixtureWhite(targetFixture, fixtureID):
    global strobeEnabled, strobeFrequency, whiteSetting

    targetFixture.setWhite(0)
    targetFixture.setStrobe(0)

    if strobeEnabled:
        targetFixture.setWhite(DMX_CHANNEL_MAX)
        targetFixture.setStrobe(strobeFrequency * (DMX_CHANNEL_MAX / 100))
    elif whiteSetting:
        if (whiteSetting == 1 and fixtureID == 1) or (whiteSetting == 2 and fixtureID == 3):
            targetFixture.setWhite(32)
        elif whiteSetting == 3:
            targetFixture.setWhite(DMX_CHANNEL_MAX)


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
            amplitude = np.max(amps[idx])
            band_amplitudes.append(amplitude)

        # Transform audio singal levels to light signal levels and apply amlification
        signal_mean = calculate_signal_mean(band_amplitudes, noiseLevel)
        cross_band_clipping = mapAudioAmplitudeToLightLevel(band_amplitudes, signal_mean + noiseLevel)
        updateAmplificationFactor(cross_band_clipping)

        # select and cycle fixture profiles
        permutatedProfiles = [FixtureProfile() for _ in range(FIXTURE_AMOUNT)]
        permutation_code = generate_permutation_code()
        permuted_profiles = permutate_profiles(permutation_code, permutatedProfiles)

        for fixtureID in range(0, FIXTURE_AMOUNT):
            setFixtureColor(fixtures[fixtureID], band_amplitudes, permutatedProfiles[fixtureID].getHexColor())
            setFixtureBrightness(fixtures[fixtureID], band_amplitudes, permutatedProfiles[fixtureID].getHexFrequency())
            setFixtureWhite(fixtures[fixtureID], fixtureID)

        ax.clear()
        ax.bar(band_names, band_amplitudes, color="skyblue")
        ax.set(xlabel="Frequency Band", ylabel="Amplitude")
        ax.set_ylim(0, 200)

        plt.pause(0.01)

except KeyboardInterrupt:
    print("Analyse beendet")

# Aufräumen
stream.stop_stream()
stream.close()
p.terminate()
plt.close()
