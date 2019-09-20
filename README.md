## Air Quality Indicator

### Parts list
- ESP32 dev kit
- Plantower PMS7003
- SSD1306 based display with i2c
- MQ7 (or any other MQ*), resistor, logic level mosfet, capacitor ([schematic](docs/schematic-MQ7.png))

### Description
Device gather data from two sensors, shows it on OLED and sends to narodmon.ru.
To work properly there is necessary only one of sensors.

### Installation
- setup networking in mos.yml (docs: https://mongoose-os.com/docs/mongoose-os/quickstart/setup.md)
- second configuration part is here fs/init.js
- mos build
- mos flash
- after 2.5 minutes, you can watch first data on narodmon site (add device to your profile by MAC address).
