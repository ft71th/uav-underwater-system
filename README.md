# UAV Undervattensystem — Arduino-kod

## Översikt

Systemet består av två Arduino Nano:

### Boj (boj_arduino.ino)
- Läser 5 PWM-kanaler från Skyfly 2.4GHz mottagare
- Läser GPS NMEA-data via SoftwareSerial (MAX3232)
- Paketerar och skickar allt via RS485 till UAV

### UAV (uav_arduino.ino)
- Tar emot RS485-data från bojen
- Parsar PWM-värden och GPS-data
- Styr servon och ESC via PCA9685 (I²C)
- Failsafe efter 500ms utan data

## Bibliotek som behövs

Installera via Arduino IDE → Verktyg → Hantera bibliotek:

1. **Adafruit PWM Servo Driver Library** (av Adafruit)
2. **Wire** (inbyggt)
3. **SoftwareSerial** (inbyggt)

## Protokoll

```
PWM-paket:  <P:1500,1500,1500,1500,1500,CS>
GPS-paket:  <G:$GPGGA,123456.00,...>
            <G:$GPRMC,123456.00,...>
```

- `<` och `>` = start/slut-markörer
- `P:` = PWM-data, `G:` = GPS-data
- CS = XOR-checksumma (hex) över payload
- PWM skickas med ~50 Hz, GPS med ~5 Hz

## Uppladdning

**VIKTIGT:** Koppla bort MAX485 från D0/D1 innan du laddar upp!
USB-serial och MAX485 delar samma pinnar.

### Steg:
1. Koppla loss MAX485-modulens DI och RO
2. Anslut Arduino Nano via USB
3. Välj rätt kort (Arduino Nano) och port i Arduino IDE
4. Välj rätt processor (ATmega328P eller ATmega328P Old Bootloader)
5. Ladda upp
6. Koppla tillbaka MAX485

## Kanalmappning

| Skyfly CH | Funktion         | PCA9685 | Aktuator           |
|-----------|------------------|---------|--------------------|
| CH1 (D6)  | Speed +/−        | CH0     | ESC → Motor        |
| CH2 (D7)  | Yaw port/stb     | CH1     | Servo Rudder       |
| CH3 (D8)  | Pitch port       | CH2     | Servo Pitch Port   |
| CH4 (D9)  | Pitch starbord   | CH3     | Servo Pitch Stb    |
| CH5 (D10) | Ballast +/−      | CH4     | Ballast-mekanism   |

## Failsafe

Om UAV-sidan inte får data på 500ms:
- Alla kanaler sätts till 1500µs (neutral)
- Motor stannar
- Roder centreras

## Felsökning

**Inget svar från UAV:**
- Kontrollera att A→A och B→B på RS485 (inte korsade)
- Verifiera gemensam GND mellan boj och UAV
- Kontrollera 120Ω terminering vid lång kabel

**Servon rycker:**
- Lägg till 100µF + 100nF kondensator vid PCA9685 V+
- Kontrollera att buck-converter ger stabil 5.0V

**GPS ger ingen data:**
- Kontrollera TX→RX korskoppling via MAX3232
- GPS behöver klar sikt till himlen för fix
- Kolla baudrate (default 9600)
