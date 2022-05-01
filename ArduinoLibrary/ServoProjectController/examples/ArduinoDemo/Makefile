arduino-cli = arduino-cli
board = adafruit:samd:adafruit_itsybitsy_m0
#board = arduino:avr:uno
port = /dev/ttyACM1

compileTool  = $(arduino-cli) compile --fqbn $(board)
uploadTool  = $(arduino-cli) upload -p$(port) --fqbn $(board)

all: compile autoUpload

compile:
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
