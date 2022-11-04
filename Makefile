.PHONY: clean all dist arduino_trinkey_qt

DIST_DIR=dist

ARDUINO_ADD_URL= \
  --additional-urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

all: dist

arduino_trinkey_qt:
	arduino-cli ${ARDUINO_ADD_URL} core update-index
	arduino-cli ${ARDUINO_ADD_URL} core install rp2040:rp2040
	arduino-cli lib install "SparkFun ADS122C04 ADC Arduino Library"
	cd mlx-i2c-stick-arduino && arduino-cli ${ARDUINO_ADD_URL} compile --fqbn rp2040:rp2040:adafruit_trinkeyrp2040qt mlx-i2c-stick-arduino.ino -e --clean
	echo "- [adafruit_trinkeyrp2040qt](firmware/rp2040.rp2040.adafruit_trinkeyrp2040qt/mlx-i2c-stick-arduino.ino.uf2)" >> firmware_list.md

dist: arduino_trinkey_qt
	@-cp -fv firmware_list.md web_interface
	make -C web_interface all
	@mkdir -p ${DIST_DIR}/firmware
	@cp -rfv mlx-i2c-stick-arduino/build/* ${DIST_DIR}/firmware

clean:
	@rm -rfv ${DIST_DIR}
	@rm -fv firmware.csv
