"""
CameraController - wraps picamera2 for taking a single still image right
before each raster begins, NOT continuous video streaming (you don't need
a live feed, just "what did the particles look like at the moment this
raster started").

`picamera2` is imported LAZILY (inside _ensure_open(), not at module load
time) - same pattern as daqhats in daq_controller.py - so importing this
module elsewhere (e.g. testing the GUI on a non-Pi machine) doesn't blow up
just from importing it.
"""


class CameraController:
    def __init__(self, size=(640, 480)):
        self.size = size
        self.picam2 = None

    def _ensure_open(self):
        if self.picam2 is not None:
            return
        from picamera2 import Picamera2
        self.picam2 = Picamera2()
        # still_configuration, not video - we only ever want single frames,
        # not a continuous stream, so this is the more appropriate mode
        # (also typically gives a higher-quality single capture than the
        # video pipeline would).
        self.picam2.configure(self.picam2.create_still_configuration(main={"size": self.size}))
        self.picam2.start()

    def capture(self):
        """Captures ONE frame and returns it as a numpy array (RGB)."""
        self._ensure_open()
        return self.picam2.capture_array()

    def close(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception as e:
                print(f"[camera] Error stopping camera (usually harmless): {e}")
            self.picam2 = None
