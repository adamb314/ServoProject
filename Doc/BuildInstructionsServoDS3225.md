
## Part list

* DS3225 MG servo
* 2 x ITR8307
* Custom PCB
* Flat cables
    * 2 leads wide
    * 3 x 3 leads wide
    * 4 leads wide
* pin header 1x4
* 3D printed parts
    * encDisc
    * encDiscTurningMount
    * gearLidBottom
    * gearLidTop
    * optSensorHolder
    * pcbMount

## Build instructions

### Prepare parts

1. Order PCB using gerber, BOM and CPL files:
    - `PcbDesignes/combined/manufacture/gerber.zip`
    - `PcbDesignes/combined/manufacture/jlcpcbPcbaBOM.csv`
    - `PcbDesignes/combined/manufacture/jlcpcbPcbaCPL.csv`

2. Burn bootloader to PCB.
    The following instructions are for burning the bootloader using a Raspberry Pi.
    
    (For burning the bootloader with a `stlinkv2` on windows, see [stlinkv2 on windows](BurnBootloaderUsingStlinkv2OnWindows.md).
    Also see [bootloader programmer](../CadFiles/BootloaderProgrammer/readme.md) for pogo-pin connector for PCB programmer)

    Power PCB via USB from pi and connect:
    ```
        Prog1:D (swdio) -> GPIO 25
        Prog1:C (swclk) -> GPIO 11
        Prog1:Reset     -> 3.3v

        (GPIO nr are for a Raspberry Pi 3 B)
    ```

     1. Install OpenOCD on the pi with this guide:
           https://learn.adafruit.com/programming-microcontrollers-using-openocd-on-raspberry-pi/compiling-openocd
        
        or using apt-get:
        ```
           sudo apt-get install openocd
        ```
    
     2. Download bootloader:
           https://github.com/adafruit/uf2-samdx1/releases/download/v3.13.0/bootloader-itsybitsy_m0-v3.13.0.bin
    
        or latest from:
           https://github.com/adafruit/uf2-samdx1/releases
          
     3. Put \*.bin file in new folder on pi together with the file [burnBootloaderWithOpenOCD.txt](../PcbDesignes/burnBootloaderWithOpenOCD.txt) and rename the script file to `openocd.cfg`

     4. run:
        ```
           cd \\|Insert name of folder on pi with the script
           sudo openocd
        ```

3. Print objects in PrusaSlicer project file `CadFiles/ServoDS3225/prusaSlicer.3mf`
    
    Print in black PLA:
    - `gearLidBottom.stl`
    - `gearLidTop.stl`
    - `optSensorHolder.stl`
    - `pcbMount.stl`

    Print in white PLA:
    - `encDisc.stl`
    - `encDiscTurningMount.stl`

### Modify the servo

Modify the servo according to the [Build Video](https://youtu.be/Ctb4s6fqnqo?t=21)

[![Build Video](https://img.youtube.com/vi/Ctb4s6fqnqo/0.jpg)](https://youtu.be/Ctb4s6fqnqo?t=21)

Build steps in video:

1. Disassemble the servo

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-13h57m31s735.png">

2. The `encDisc` part has to  be completely smooth and free from defects. Getting the required resolution out of a normal FDM 3D printer is basically impossible. So to get a smooth `encDisc` part we first need to file it down while spinning it:

    1. Remove the gear from the motor and mount `encDiscTurningMount` on motor axis. Then mount `encDisc` on `encDiscTurningMount`

        <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-13h59m28s296.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h02m07s356.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h03m29s228.png">

    2. Apply 3-5 volts to the motor and file down 3D printing seams and printing lines until the surface of `encDisc` is completely smooth

        <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h04m26s735.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h07m40s843.png">

3. Mount the smooth `encDisc` and gear on the motor axis. Then glue `optSensorHolder` to the motor

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h10m10s171.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h14m38s165.png">

4. Solder on a 3 lead wide flat cable to each of the two ITR8307 sensors. Make sure that the orientation of the sensors are mirrored

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h16m00s406.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h16m10s437.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h16m48s955.png">

5. Mount the sensors in `optSensorHolder` so that the LED segment (pin 1) of the sensor are oriented towards each other

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h17m42s497.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h21m31s828.png">

6. Solder the ITR8307 sensors and potentiometer to the PCB

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h26m21s087.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h30m35s971.png">

7. Mount `gearLidTop` to the gear box

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h35m39s072.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h37m03s713.png">

8. Mount `gearLidBottom`, `optSensorHolder` and the potentiometer together with the aluminum heat sink

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h39m37s266.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h41m09s515.png">

90. Mount `pcbMount` and screw everything together

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h42m46s404.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h43m30s998.png">

10. Glue the PCB to the `pcbMount` and solder the motor connections

    <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h49m06s877.png"> <img width="300px" src="readmeResources/ServoDS3225/vlcsnap-2024-03-17-14h44m02s886.png">

### Configure

1. Power servo (5 volt) via pin header and connect to computer via micro USB

2. Run `python3 ./ArduinoSketch/configurationWizard.py`

3. Create a new configuration file by clicking `Create new`, select `defaultDS3225.h` as template, and choose communication node nr

4. Transfer the initial configuration to the servo by clicking `Transfer to target`

5. Select `Optical Encoder` to calibrate encoder

    Calibration example:
    1. Move servo to 110 deg and Lock position by clicking `Lock`
    2. Set `Motor pwm value` to 610
    3. Set `Start motor pwm value` to 650
    4. Choose position resolution `Fine (~4 min)` or `Ultra (~8 min)`
    5. Click `Start calibration`

6. Identify system parameters with `Pwm and system identification`

    Calibration example:
    1. Move servo to 110 deg and Lock position by clicking `Lock`
    2. Set `Motor settle time` to 0.1 s
    3. Set `Min motor pwm value` to 590
    4. Set `Max motor pwm value` to 1023
    5. Set `System model cycle time` to 0.6 ms
    6. Click `Start calibration`

7. Calibrate motor position dependent disturbances with `Motor cogging torque`

    Calibration example:
    1. Click `Set advanced parameters`
    2. Set `Control speed` to 32
    3. Set `Inertia margin` to 2.0
    4. Click `OK`
    5. Set position resolution to `Standard (~3 min)`
    6. Move servo to 110 deg
    7. Click `Start calibration`

8. Calibrate nonlinearities in output potentiometer with `Output encoder calibration` (optional)

    Calibration example:
    1. Click `Set advanced parameters`
    2. Set `Control speed` to 28
    3. Set `Inertia margin` to 1.8
    4. Click `OK`
    5. Set position resolution to `Fine (~4 min)`
    6. Move servo to 110 deg
    7. Click `Start calibration`

9. To test the final configuration select `Test control loop`
