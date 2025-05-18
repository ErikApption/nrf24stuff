# ESP Home

## Python

```bash
python3 -m venv venv
source venv/bin/activate
pip3 install esphome
```

```powershell
python -m venv venv
. .\venv\Scripts\activat
```

## Enter Flash Mode

- Hold the BOOT button (this pulls GPIO0 to GND).
- Press and release the EN/RST button (to reset the ESP32).
- Release the BOOT button after resetting.

The ESP32 is now in flash mode and ready for firmware uploads.

## Flash image

```bash
esphome run furnace.yaml
```

## Generate Binary Image

To generate a binary image for manual upload:

```bash
esphome compile washer.yaml
```

The binary file will be saved in the `.esphome/build/<washer>/.pioenvs/<washer>` directory.

