# ZMK Framework 16 Laptop Keyboard module

This module provides support for the 16 Laptop Keyboard in the ZMK firmware. It
includes a driver, device tree bindings, a `fw16` board, and a `fw16_ansi`
keymap.

## Features
- Runs ZMK!

That's about it.

## Missing Features
- Backlight support
- RGB support
- "Screen" key (on F9)
- "Airplane" key (on F10)
- "Settings" key (on F12, which in QMK is bound to the "Media Select" key)
- Scroll lock on FN+K combo (on the screen share key at the moment)
- Shields other than "ansi"
- FN-lock layer
- Probably other things I haven't ported yet.

## Usage
To use this module, include it in your ZMK `config/west.yml` configuration. You
can build using the `fw16` board and `fw16_ansi` shield, or supply your own
custom shields. Check [hunner/zmk-config](https://github.com/hunner/zmk-config)
for an example.

Flash your keyboard by sliding down the touchpad module a little, hold both alt
keys (left and right), and slide the touchpad module back in place. You should
see a new storage device appear in `dmesg` output. Then I `sudo mount /dev/sda1
/mnt && sudo cp firmware/fw16_ansi-fw16.uf2 /mnt && sudo umount /mnt` and the
keyboard reboots on its own. Make sure to back up your old firmware first, and
maybe have a spare keyboard nearby to use in case it doesn't work properly.

## Credits
Liberally created with [Ampcode](https://ampcode.com) since I know very little
about hardware. But I learned a lot in the process!
