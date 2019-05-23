arduino-cli = ~/go/bin/arduino-cli
board = adafruit:samd:adafruit_itsybitsy_m0
port = /dev/ttyACM0

compileTool  = $(arduino-cli) compile --fqbn $(board)
uploadTool  = $(arduino-cli) upload -p$(port) --fqbn $(board)

all: compile autoUpload

compile:
	$(compileTool) ./

autoUpload:
ifneq (,$(wildcard ./enableAutoUpload))
	$(uploadTool) ./
endif

upload:
	$(uploadTool) ./

enableAutoUpload:
	touch ./enableAutoUpload

disableAutoUpload:
ifneq (,$(wildcard ./enableAutoUpload))
	rm ./enableAutoUpload 
endif
