# Changelog

## [0.2.0] - 2025-04-03
- Add compatibility with kernels up to 6.12
- devicetree: change label names in cam1 overlays to allow dual cam
- Improve exposure settings
- Proper implementation of GS and RS timing

## [0.1.0] - 2024-10-11
- Alpha release - basic support of the following features
- Support both variants : RGBiR (vd1941) / Mono (vd5941)
- Basic support of Rolling / Global Shutter
- 6 Frame sizes (2560x1984, 2560x1600, 2560x1440, 1920x1080, 1280x720, 640x480)
- Variable Framerate up to 54FPS (max in RS mode)
- Raw8, Raw10, Raw12
- Hflip/Vflip
- Single Exposure, Analog & Digital Gains Controls
- Test pattern : Dgrey
- Temperature
- GPIOs - frame synchronization (Follower mode: in-sync)
- GPIOs - external LED signal (synchronized with sensor integration periods)
- CSI Lane configuration : lanes number (2 or 4), lanes ordering, lanes polarity
