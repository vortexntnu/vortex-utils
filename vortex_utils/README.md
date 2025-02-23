# H264Decoder
## Prerequisites
### GStreamer
```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly gstreamer1.0-libav python3-gi \
python3-gst-1.0
```
If you experience display-related issues when launching the GUI (e.g., qt.qpa.wayland: eglSwapBuffers failed), you may need to force X11 instead of Wayland.
```bash
export QT_QPA_PLATFORM=xcb
```
### PyGObject
Step 1: Install System Dependencies
```bash
sudo apt update
sudo apt install -y libglib2.0-dev libcairo2-dev libgirepository1.0-dev \
gir1.2-gtk-3.0 python3-dev
```
Step 2: Install PyGObject
```bash
pip install pygobject
```
