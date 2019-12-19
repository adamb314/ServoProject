arduino-cli = ~/go/bin/arduino-cli
board = adafruit:samd:adafruit_itsybitsy_m0
port = /dev/ttyACM0

compileTool  = $(arduino-cli) compile --fqbn $(board)
uploadTool  = $(arduino-cli) upload -p$(port) --fqbn $(board)

all: compile autoUpload

compile:
	./configSelector.py
	@echo "------------- Build Start -------------"
	$(compileTool) ./
	@echo "------------- Build Done -------------"

autoUpload:
ifneq (,$(wildcard ./enableAutoUpload))
	@echo "------------- Upload Start -------------"
	$(uploadTool) ./
	@echo "------------- Upload Done -------------"
endif

upload:
	@echo "------------- Upload Start -------------"
	$(uploadTool) ./
	@echo "------------- Upload Done -------------"

enableAutoUpload:
	touch ./enableAutoUpload

disableAutoUpload:
ifneq (,$(wildcard ./enableAutoUpload))
	rm ./enableAutoUpload 
endif
