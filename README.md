# hyperpixel2r-touch

Userspace touch input daemon for Pimoroni HyperPixel 2r round display on Raspberry Pi.

## Overview

This daemon reads touch data from the FT5x06 capacitive touch controller via GPIO bitbanging I2C and creates a virtual Linux input device using uinput. It supports up to 2 simultaneous touch points using the multi-touch protocol B.

## Features

- Direct GPIO bitbanging I2C - no kernel driver required
- Multi-touch protocol B support (2 touch points)
- Single-touch fallback for compatibility
- Minimal dependencies (libc + nix)
- Small binary size (~100KB stripped)

## Requirements

### Hardware

- Raspberry Pi (tested on Pi Zero 2 W)
- Pimoroni HyperPixel 2r round display (480x480)

### Software

- Linux kernel with `vc4-kms-dpi-hyperpixel2r` overlay
- `/dev/gpiomem` access (gpio group or root)
- `/dev/uinput` access (input group or root)

## Installation

### Pre-built Binary

Download from [Releases](https://github.com/task-jp/hyperpixel2r-touch/releases).

### From Source

```bash
# Native build (on Raspberry Pi)
cargo build --release

# Cross-compilation (from x86_64 host)
cargo build --release --target aarch64-unknown-linux-gnu
```

## Configuration

### Device Tree Overlay

The HyperPixel 2r display overlay is required:

```
# /boot/firmware/config.txt
dtoverlay=vc4-kms-v3d
dtoverlay=vc4-kms-dpi-hyperpixel2r
```

### Systemd Service

```bash
sudo cp hyperpixel2r-touch /usr/local/bin/
sudo cp hyperpixel2r-touch.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable hyperpixel2r-touch
sudo systemctl start hyperpixel2r-touch
```

### Manual Execution

```bash
sudo ./hyperpixel2r-touch
```

Verify the input device:

```bash
cat /proc/bus/input/devices | grep -A5 HyperPixel
```

## Technical Details

| Parameter | Value |
|-----------|-------|
| Touch Controller | FT5x06 (EDT-FT5x06 compatible) |
| I2C Address | 0x15 |
| GPIO SDA | GPIO10 |
| GPIO SCL | GPIO11 |
| GPIO INT | GPIO27 |
| Screen Resolution | 480x480 |
| Max Touch Points | 2 |
| Polling Rate | ~30Hz |

## How It Works

1. **GPIO Bitbanging**: Uses `/dev/gpiomem` to directly control GPIO pins for I2C communication, bypassing the need for kernel I2C drivers.

2. **Touch Data Reading**: Polls the FT5x06 touch controller every ~32ms to read touch status and coordinates.

3. **UInput Device**: Creates a virtual input device that applications can use like any standard touchscreen.

## Troubleshooting

### "Could not open /dev/gpiomem"

Add your user to the gpio group:
```bash
sudo usermod -a -G gpio $USER
```

### "Could not open /dev/uinput"

Add your user to the input group:
```bash
sudo usermod -a -G input $USER
```

Or run with sudo.

### "I2C read error"

- Check that the HyperPixel 2r is properly connected
- Verify the display overlay is loaded: `dmesg | grep -i hyperpixel`

## License

MIT License - see [LICENSE](LICENSE) file.

## Related Projects

- [Pimoroni HyperPixel 2r](https://shop.pimoroni.com/products/hyperpixel-2-1-round)
- [vc4-kms-dpi-hyperpixel2r overlay](https://github.com/raspberrypi/linux)
