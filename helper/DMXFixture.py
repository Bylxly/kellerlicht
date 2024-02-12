class DMXFixture:
    localDimmerChannel = 0
    localRedChannel = 1
    localGreenChannel = 2
    localBlueChannel = 3
    localWhiteChannel = 4
    localStrobeChannel = 5
    channelAmount = 6

    def __init__(self, startChannel, dimmerDefaultValue):
        self._startChannel = startChannel
        self._dimmerDefaultValue = dimmerDefaultValue
        # Initialisiere alle Attribute hier
        self._dimmerValue = dimmerDefaultValue
        self._rgbDimmerValue = 255
        self._redValue = 0
        self._greenValue = 0
        self._blueValue = 0
        self._whiteValue = 0
        self._strobeValue = 0

    def setRGB(self, redValue, greenValue, blueValue):
        self._redValue = redValue
        self._greenValue = greenValue
        self._blueValue = blueValue

    def setWhite(self, whiteValue):
        self._whiteValue = whiteValue

    def setDimmer(self, dimmerValue):
        self._dimmerValue = dimmerValue

    def setRGBDimmer(self, dimmerValue):
        self._rgbDimmerValue = dimmerValue

    def setStrobe(self, strobeValue):
        self._strobeValue = strobeValue

    def reset(self):
        self._dimmerValue = self._dimmerDefaultValue
        self._rgbDimmerValue = 255
        self._redValue = 0
        self._greenValue = 0
        self._blueValue = 0
        self._whiteValue = 0
        self._strobeValue = 0

    def display(self, dmxController):
        # Die Implementierung dieser Methode hängt von der spezifischen Implementierung
        # deines DMX_Master in Python ab. Dies ist ein Platzhalter für die Funktionalität.
        pass
        # Zum Beispiel:
        # dmxController.setChannelValue(self._startChannel + DMXFixture.localRedChannel, self._redValue)

class FixtureProfile:
    def __init__(self, color=0x0, frequency=0x0):
        self._color = color
        self._frequency = frequency

    def getHexColor(self):
        return self._color

    def getHexFrequency(self):
        return self._frequency
