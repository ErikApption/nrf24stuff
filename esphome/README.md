# ESP Home

## Python

```bash
python3 -m venv venv
source venv/bin/activate
pip3 install esphome
```

## Enter Flash Mode

- Hold the BOOT button (this pulls GPIO0 to GND).
- Press and release the EN/RST button (to reset the ESP32).
- Release the BOOT button after resetting.

The ESP32 is now in flash mode and ready for firmware uploads.

## Generate image

```bash
esphome run furnace.yaml
```