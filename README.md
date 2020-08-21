# Autofocus code for a camera using liquid lens


## Usage

### Checking camera
```
python preview.py
```
> q: quit 

> w: increase the liquid lens'voltage

> e: drop the liquid lens'voltage

---
### Focusing camera
```
python auto_cam.py --cam_id 0 --port /dev/tty.usbserial-1410 --wait 0.5
```

---
### Focusing camera (arducam)
```
python auto_arducam.py --cfg OV2311_MIPI_2Lane_RAW8_8b_1600x1300_60fps.cfg --port /dev/tty.usbserial-1410 --wait 0.5
```
