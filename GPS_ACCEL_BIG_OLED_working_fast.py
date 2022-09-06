# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
This test will initialize the display using displayio and draw a solid white
background, a smaller black rectangle, and some white text.
"""
# Import der Bibliotheken
import busio
import busio
import digitalio
import math
import adafruit_bmp280
import time
import board
import displayio
import terminalio
import adafruit_lsm6ds.lsm6ds33
from adafruit_display_text import label
import adafruit_displayio_ssd1306

# Initialisierung der BoardLED (rote kleine LED)
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# check ob gps im wartezustand ist
Waiting = True
Lastwaiting = True

# Initialisierung der UART schnittstelle. GPS nutzt 9600br (sollte ggf. getuned werden)
uart = busio.UART(board.TX, board.RX, baudrate=9600)
byte_read = uart.read(1)  # Read one byte over UART lines

# Display vorbereiten
displayio.release_displays()
oled_reset = board.D9

# alles I2C empfänger vorbereiten
i2c = board.I2C()
display_bus = displayio.I2CDisplay(i2c, device_address=0x3C, reset=oled_reset)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)  # Druckmessung
lsm6ds33 = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c)  # Beschleunigungsmessung
bmp280.sea_level_pressure = (
    1013.25  # sealevel (sollte durch Druckwerte mit GPS genullt werden)
)

# Größen für das Display
WIDTH = 128
HEIGHT = 32  # Change to 64 if needed
BORDER = 2
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=WIDTH, height=HEIGHT)

# Make the display context
splash = displayio.Group()
display.show(splash)

color_bitmap = displayio.Bitmap(WIDTH, HEIGHT, 1)
color_palette = displayio.Palette(1)
color_palette[0] = 0xFFFFFF  # White

# Draw a label
text = "Fly like you stole it!"
text_area = label.Label(
    terminalio.FONT, text=text, color=0xFFFFFF, x=0, y=HEIGHT // 2 - 1
)
splash.append(text_area)
text_area.x = 3
text_area.y = 7
# Kooridnaten definieren
# [[Lon_GPS,Lat_GPS,alt_GPS],[Lon_GPS_GH1,Lat_GPS_GH1,alt_GPS_GH1],[Lon_GPS_GH2,Lat_GPS_GH2,alt_GPS_GH2]]
koordinates = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
dkoordinates = [0, 0, 0, 0]
time.sleep(1)
timeing = [0, 0, 0]
v=0
timestr="00:00:00"
while True:
    # Auslesen des UART Strings und Zerteilen in eine Liste
    ser_bytes = str(uart.readline())
    ser_bytes_a1 = ser_bytes.split(",")
    # Ifschleife checkt ob die Zeile die relevanten Daten beinhaltet
    if ser_bytes_a1[0] == "b'$GNGGA":     
        #Probieren ob mit den GNSS Daten was angefangen werden kann
        try:  
            # Falls die Zeile zwar die relevanten Daten beinhaltet, allerdings noch keine Sateliten gefunden sind und somit keine Position bestimmt wurde soll nichts passieren
            if ser_bytes_a1[2] == "" and ser_bytes_a1[4] == "":
                print("Waiting for Satelites")
                # Waiting und Lastwaiting sollen überprüfen ob in der letzten iteration Sateliten verfügbar waren
                Waiting = True
                Lastwaiting = True
            else:
                # Falls Sateliten verfügbar sind wird nicht mehr gewartet
                Waiting = False
                # Zeit einlesen und in korrektes Format wandeln
                timeing[0] = ser_bytes_a1[1][0:2]
                timeing[1] = ser_bytes_a1[1][2:4]
                timeing[2] = ser_bytes_a1[1][4:6]
                timestr = str(timeing[0]) + ":" + str(timeing[1]) + ":" + str(timeing[2])
                # Koorinaten einlesen und in korrektes Format umwandeln
                print(ser_bytes_a1[2][0:2],ser_bytes_a1[2][2:],ser_bytes_a1[4][0:3],ser_bytes_a1[4][3:])
                koordinates[0] = [
                    float(ser_bytes_a1[2][0:2]) + float(ser_bytes_a1[2][2:]) / 60,
                    float(ser_bytes_a1[4][0:3]) + float(ser_bytes_a1[4][3:]) / 60,
                    float(ser_bytes_a1[9]),
                    time.monotonic(),
                ]
                # Falls keine Sateliten gefunden werden werden alle Daten dennoch weiter geführt um nicht bei der Zeitdifferenz durch 0 zu teilen oder die Koordinaten bei 0 anfangen zu lassen
                if Waiting == False and Lastwaiting == True:
                    koordinates[1][0] = koordinates[0][0]
                    koordinates[1][1] = koordinates[0][1]
                    koordinates[1][2] = koordinates[0][2]
                    koordinates[1][3] = koordinates[0][3] + 0.5
                    koordinates[2][0] = koordinates[0][0]
                    koordinates[2][1] = koordinates[0][1]
                    koordinates[2][2] = koordinates[0][2]
                    koordinates[2][3] = koordinates[0][3] + 1
                    text_area.text = "Waiting for GNSS Signal"
                else:
                    pass

                # Position mit einem GH-Filter beaufschlagen
                koordinates[1][0] = (
                    koordinates[1][0] + (koordinates[0][0] - koordinates[1][0]) * 0.8
                )
                koordinates[1][1] = (
                    koordinates[1][1] + (koordinates[0][1] - koordinates[1][1]) * 0.8
                )
                koordinates[1][2] = (
                    koordinates[1][2] + (koordinates[0][2] - koordinates[1][2]) * 0.8
                )
                koordinates[1][3] = (koordinates[1][3] + koordinates[0][3]) * 0.5

                # Ableitung der kooridnaten
                dkoordinates[0] = koordinates[1][0] - koordinates[2][0]
                dkoordinates[1] = koordinates[1][1] - koordinates[2][1]
                dkoordinates[2] = koordinates[1][2] - koordinates[2][2]
                dkoordinates[3] = koordinates[1][3] - koordinates[2][3]
                dx = 111.307 * math.cos(dkoordinates[0]) * (dkoordinates[1])
                dy = 111.307 * (dkoordinates[0])

                # berechnung von Distanz und Geschwindigkeit sowie absoluter beschleunigung
                distance = math.sqrt(dx * dx + dy * dy)
                v = distance * 1000 * 3.6 / dkoordinates[3] * 0.8


                # Koordinatenspeicher für die nächste Iteration beschreiben
                koordinates[2][0] = koordinates[1][0]
                koordinates[2][1] = koordinates[1][1]
                koordinates[2][2] = koordinates[1][2]
                koordinates[2][3] = koordinates[1][3]
                # Lastwaiting beschreiben um zu klären ob es die erste Iteration ist
                Lastwaiting = False
            #Falls die GNSS Daten beschädigt waren eine kleine Fehlermeldung in der Konsole ausgeben.
        except:
            print('bad GNSS data')
    absolute_g = math.sqrt(
        (lsm6ds33.acceleration[0] / 9.8) ** 2
        + (lsm6ds33.acceleration[1] / 9.8) ** 2
        + (lsm6ds33.acceleration[2] / 10.2) ** 2
    )

    # Füllklötze basteln um die Position im OLED sauber auszurichten
    if len("{:.0f}".format(v)) > 3:
        space1 = 0
    else:
        space1 = 3 - len("{:.0f}".format(v))
    if len(timestr) > 8:
        space2 = 0
    else:
        space2 = 8 - len(timestr)
    space1str = space1 * " "
    space2str = space2 * " "
    # Anpassung des Textes im OLED Display
    text = (
        str(space1str)
        + "{:.2f}km/h  ".format(v)
        + str(space2str)
        + str(timestr)
        + "\n"
        + "{:.2f}g {:.2f}g| {:.2f}g-".format(
            absolute_g,
            lsm6ds33.acceleration[0] / 9.8,
            lsm6ds33.acceleration[1] / 9.8,
        )
    )
    text_area.text = text
