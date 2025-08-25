# PS4 Camera Controller

## Load firmware camera

```
python ./firmware_loader/ps4eye_init.py
```

## Run viewer
```
python ./python/view_rgb.py
```

## Other experiments

Trying to use modified version of ps4_eye (library for controlling ps4 eye camera)
```
make ps4eye && sudo ./ps4eye_standalone
```

Trying to use libuvc to communicate directly
```
make && sudo ./main
```

## Misc
Sometimes the camera is too dark. I saw that the frame on the preview is much brighter that the one I get from OpenCV.

## Intersting resources

Best resource for Python. Has some examples under OpenCV_viewer
https://github.com/sieuwe1/PS4-eye-camera-for-linux-with-python-and-OpenCV

Driver
https://github.com/ps4eye/ps4eye

Similar
https://github.com/bigboss-ps3dev/PS4EYECam

USB Library to connect directly
https://github.com/libuvc/libuvc

## Frame specification

Interface 1 alt setting 0 describes all video modes.
Modes and resolutions:
- mode 0: fps 60(default), 30, 15, 8. Left and right frames: 1280x800
- mode 1: fps 120(default), 60, 30, 15, 8. Left and right frames: 640x400
- mode 2: fps 240(default), 120, 60, 30. Left and right frames: 320x192

Device frame format:

- mode 0 3448x2x808 bytes frame each row
  - header 32 bytes
  - audio 64 bytes
  - video left 1280x2 bytes
  - video right 1280x2 bytes
  - video interleave 840x2 bytes

- mode 1 1748x2x408 bytes frame each row
  - header 32 bytes
  - audio 64 bytes
  - video left 640x2 bytes
  - video right 640x2 bytes
  - video interleave 420x2 bytes

- mode 2 898x2x200 bytes frame each row
  - header 32 bytes
  - audio 64 bytes
  - video left 320x2 bytes
  - video right 320x2 bytes
  - video interleave 210x2 bytes
