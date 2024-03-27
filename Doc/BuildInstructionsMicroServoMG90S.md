
## Part list

* WS-MG90S micro servo
* 2 x ITR8307
* Custom PCB
* Flat cables
    * 2 leads wide
    * 3 x 3 leads wide
    * 4 leads wide
* pin header 1x4
* 3D printed parts
    * baseBox
    * baseBushing
    * encDisc
    * encDiscTurningMount

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

3. Print objects in PrusaSlicer project file `CadFiles/MicroServoMG90S/prusaSlicer.3mf`
    
    Print in black PLA:
    - `baseBox.stl`
    - `baseBushings.stl`

    Print in white PLA:
    - `encDisc.stl`
    - `encDiscTurningMount.stl`

### Modify the servo

Modify the servo according to the [Build Video](https://youtu.be/VcU6IY8n05g?t=0)

[![Build Video](https://img.youtube.com/vi/VcU6IY8n05g/0.jpg)](https://youtu.be/VcU6IY8n05g?t=0)

Build steps in video:

1. Disassemble the servo

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-20h24m58s866.png">

2. The `encDisc` part has to  be completely smooth and free from defects. Getting the required resolution out of a normal FDM 3D printer is basically impossible. So to get a smooth `encDisc` part we first need to file it down while spinning it:

    1. Remove the gear from the motor and mount `encDiscTurningMount` on motor axis. Then mount `encDisc` on `encDiscTurningMount`

        <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-20h26m22s682.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-20h27m45s147.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h21m35s137.png">

    2. Apply 3-5 volts to the motor and file down 3D printing seams and printing lines until the surface of `encDisc` is completely smooth

        <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h22m09s687.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h23m30s953.png">

3. Mount the smooth `encDisc` and gear on the motor axis

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h26m01s175.png">

4. Solder on a 3 lead wide flat cable to each of the two ITR8307 sensors. Make sure that the orientation of the sensors are mirrored

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h27m25s869.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h27m32s682.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h27m50s789.png">

5. Mount the sensors in `baseBox` so that the LED segment (pin 1) of the sensors are oriented towards each other

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h29m07s946.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h32m08s966.png">

6. Solder the ITR8307 sensors and potentiometer to the PCB

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h33m13s877.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h33m52s353.png">
    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h37m47s119.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h44m28s268.png">

7. Put `baseBushing` on the center axis of the gear box

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h40m17s274.png">

8. Insert motor into `baseBox`, solder motor connections to PCB and screw everything together

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h46m21s953.png"> <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h47m58s058.png">

9. Done

    <img width="300px" src="readmeResources/MicroServoMG90S/vlcsnap-2024-03-17-21h49m10s475.png">

### Configure

1. Power servo (5 volt) via pin header and connect to computer via micro USB

2. Run `python3 ./ArduinoSketch/configurationWizard.py`

3. Create a new configuration file by clicking `Create new`, select `defaultMG90S.h` as template, and choose communication node nr

4. Transfer the initial configuration to the servo by clicking `Transfer to target`

5. Select `Optical Encoder` to calibrate encoder

    Calibration example:
    1. Move servo to 100 deg and Lock position by clicking `Lock`
    2. Set `Motor pwm value` to 320
    3. Set `Start motor pwm value` to 450
    4. Choose position resolution `Fine (~4 min)` or `Ultra (~8 min)`
    5. Click `Start calibration`

6. Identify system parameters with `Pwm and system identification`

    Calibration example:
    1. Move servo to 100 deg and Lock position by clicking `Lock`
    2. Set `Motor settle time` to 0.1 s
    3. Set `Min motor pwm value` to 320
    4. Set `Max motor pwm value` to 1023
    5. Set `System model cycle time` to 0.6 ms
    6. Click `Start calibration`

7. Calibrate motor position dependent disturbances with `Motor cogging torque` (optional)

    Calibration example:
    1. Click `Set advanced parameters`
    2. Set `Control speed` to 32
    3. Set `Inertia margin` to 2.0
    4. Click `OK`
    5. Set position resolution to `Standard (~3 min)`
    6. Move servo to 100 deg
    7. Click `Start calibration`

8. Calibrate nonlinearities in output potentiometer with `Output encoder calibration` (optional)

    Calibration example:
    1. Click `Set advanced parameters`
    2. Set `Control speed` to 28
    3. Set `Inertia margin` to 1.8
    4. Click `OK`
    5. Set position resolution to `Fine (~4 min)`
    6. Move servo to 100 deg
    7. Click `Start calibration`

9. To test the final configuration select `Test control loop`
