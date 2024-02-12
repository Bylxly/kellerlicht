import serial
import time

# Ersetzen Sie 'COMx' oder '/dev/ttyACM0' mit dem korrekten Port Ihres Arduino
ser = serial.Serial('/dev/tty.usbserial-A50285BI', 57600, timeout=1)  # timeout=1s für das Lesen der Daten


def send_dmx_command(channel, value):
    """
    Sendet einen DMX-Befehl an den Arduino und liest die Antwort.
    :param channel: DMX-Kanal (1-512)
    :param value: Wert für den Kanal (0-255)
    """
    command = f"{channel},{value}\n"
    print(command)
    ser.write(command.encode())
    print(f"DMX Befehl gesendet: Kanal {channel}, Wert {value}")




print("Drücken Sie Enter, um einen DMX-Befehl zu senden. Zum Beenden 'exit' eingeben.")

try:
    for i in range(1, 50):
        if i in (1, 7, 13, 19, 25):
            send_dmx_command(i, 255)
        else:
            send_dmx_command(i, 0)

    while True:

        # Warten auf Benutzereingabe
        user_input = input("Warten auf Eingabe (Format 'Kanal:Wert'): ")

        # Überprüfen, ob der Benutzer das Programm beenden möchte
        if user_input.lower() == 'exit':
            print("Programm wird beendet.")
            break

        if ':' in user_input:
            try:
                channel, value = map(int, user_input.split(':'))
                if 1 <= channel <= 512 and 0 <= value <= 255:
                    send_dmx_command(channel, value)
                else:
                    print("Ungültige Eingabe. Kanal muss zwischen 1 und 512 liegen, Wert zwischen 0 und 255.")
            except ValueError:
                print("Fehler beim Parsen der Eingabe. Bitte im Format 'Kanal:Wert' eingeben.")
        else:
            print("Ungültige Eingabe. Bitte im Format 'Kanal:Wert' eingeben.")

except KeyboardInterrupt:
    print("Programm vom Benutzer unterbrochen.")

finally:
    ser.close()  # Schließen der seriellen Verbindung
