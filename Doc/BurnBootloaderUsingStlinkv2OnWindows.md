#### Alternate way to install bootloader using openocd on windows using stlinkv2 clone.

1. Install openocd on windows using https://xpack.github.io/dev-tools/openocd/

2. Modify the openocd config file [openocd.cfg](../PcbDesignes/burnBootloaderWithOpenOCD.txt) to have:
   ```
              source [find interface/stlink.cfg]
              transport select hla_swd
   ```

3. Modify this file: `C:\Users\YOURUSERNAME\AppData\Roaming\xPacks\@xpack-dev-tools\openocd\0.12.0-2.1\.content\openocd\scripts\target\at91samdXX.cfg`

   By change this line 
   ```
      set _CPUTAPID 0x4ba00477
   ```
   To this
   ```
      set _CPUTAPID 0x0bc11477
   ```

Another note:
- install zadig and replace the stlink driver with the one named libusb-win32. The default winusb driver may not work. Also windows likes to reinstall the old driver at times.
