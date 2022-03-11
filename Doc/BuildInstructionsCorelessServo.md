Coreless servo modification (Turnigy MG959)
--------

### Part list

* 1 x Turnigy MG959 servo
* 1 x Adafruit ItsyBitsy M0 Express
  * https://learn.adafruit.com/introducing-itsy-bitsy-m0
* 1 x SparkFun Motor Driver - Dual TB6612FNG (1A)
  * https://www.sparkfun.com/products/14451
* 1 x as5048A + 6mm x 2mm comopatible magnet
  * https://ams.com/as5048a
* 2 x ITR8307
* 2 x 8.2 kohm resistor
* 2 x 120 ohm resistor

Cad and stl files for 3D printing:
```
CadFiles/ServoCoreless
```

Electrical schematic:
```
PcbDesignes/ServoCoreless
```
### Build instructions

#### Build MainPcb and AS5048aEncoderBoard

A video on how to build the MainPcb and AS5048aEncoderBoard can be found at https://youtu.be/YQpAAr5RPSE?t=60.
This video is for an older version but the building instructions are the same.

#### Servo gearbox with 3D printed parts

| <img width="500px" src="readmeResources/fullSizeServo/parts2.png"> | 
| --- |

#### PCB Assemblies

| <img width="500px" src="readmeResources/fullSizeServo/parts1.png"> | 
| --- |

#### How to mount the PCBs to servo

| <img width="500px" src="readmeResources/fullSizeServo/parts3.jpg"> | <img width="500px" src="readmeResources/fullSizeServo/parts4.jpg"> |
| --- | --- |

| <img width="500px" src="readmeResources/fullSizeServo/parts5.jpg"> | <img width="500px" src="readmeResources/fullSizeServo/parts6.jpg"> |
| --- | --- |

#### Build the Optical Encoder

| <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step1.jpg"> | <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step2.jpg"> | <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step3.jpg"> |
| --- | --- | --- |

1) Take the motor, use pliers to bend out the four indents in the metal housing and bend up the motor lid.

| <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step4.jpg"> | <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step5_1.jpg"> | <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step5_2.jpg"> |
| --- | --- | --- |

2) Remove the two steel brushes from the lid carefully by cutting the lid into pieces. Take the 3D printed motor lid and insert the two steel brushes and two ITR8307 into the new lid. Solder the ITR8307 according to:
```
PcbDesignes/Servo/OpticalEncoder/OpticalEncoder.sch
```

| <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step6_1.jpg"> | <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step6_2.jpg"> | <img width="300px" src="readmeResources/fullSizeServo/OptEncBuild/step7.jpg"> |
| --- | --- | --- |

3) Take the motor with the lid removed. Paint the rotor with black nail polish and glue in the optical encoder wheel on top as shown in the middle image. The dimensions of the encoder wheel can be found in:
```
CadFiles/Servo/OpticalEncoderWheelDimensions.png

```
Then carefully put the new lid on the motor without bending the steel brushes.

| <img width="500px" src="readmeResources/fullSizeServo/OptEncBuild/step8.jpg"> | 
| --- |

4) Solder the four leads from the optical encoder on top of ItsyBitsy board as shown in this image.
