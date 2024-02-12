import serial

ser = serial.Serial('/dev/tty.usbserial-A50285BI', 57600)  # Ersetzen Sie '/dev/...' durch den richtigen Port

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

    def display(self):
        # Senden Sie die Werte für jeden Kanal
        commands = [
            (self._startChannel + DMXFixture.localDimmerChannel, self._dimmerValue),
            (self._startChannel + DMXFixture.localRedChannel, int(self._redValue * (self._rgbDimmerValue / 255.0))),
            (self._startChannel + DMXFixture.localGreenChannel, int(self._greenValue * (self._rgbDimmerValue / 255.0))),
            (self._startChannel + DMXFixture.localBlueChannel, int(self._blueValue * (self._rgbDimmerValue / 255.0))),
            (self._startChannel + DMXFixture.localWhiteChannel, self._whiteValue),
            (self._startChannel + DMXFixture.localStrobeChannel, self._strobeValue),
        ]

        for channel, value in commands:
            # Formatieren Sie den Befehl als 'channel,value'
            if value > 255:
                value = 255
            print(channel, value)
            command = f"{channel},{value}\n"
            # Senden Sie den Befehl über die serielle Verbindung
            ser.write(command.encode())

    def getStartChannel(self):
        """Gibt den Startkanal der DMX-Leuchte zurück."""
        return self._startChannel

    def getDimmerValue(self):
        """Gibt den aktuellen Dimmerwert zurück."""
        return self._dimmerValue

    def getRGBDimmerValue(self):
        """Gibt den aktuellen RGB-Dimmerwert zurück."""
        return self._rgbDimmerValue

    def getRedValue(self):
        """Gibt den aktuellen Wert für Rot zurück."""
        return self._redValue

    def getGreenValue(self):
        """Gibt den aktuellen Wert für Grün zurück."""
        return self._greenValue

    def getBlueValue(self):
        """Gibt den aktuellen Wert für Blau zurück."""
        return self._blueValue

    def getWhiteValue(self):
        """Gibt den aktuellen Wert für Weiß zurück."""
        return self._whiteValue

    def getStrobeValue(self):
        """Gibt den aktuellen Strobe-Wert zurück."""
        return self._strobeValue

class FixtureProfile:
    def __init__(self, color=0x0, frequency=0x0):
        self._color = color
        self._frequency = frequency

    def getHexColor(self):
        return self._color

    def getHexFrequency(self):
        return self._frequency
