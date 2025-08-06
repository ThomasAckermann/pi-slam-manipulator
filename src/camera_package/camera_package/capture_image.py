import subprocess
import logging
from pathlib import Path
from typing import Optional

class CameraCapture:
    def __init__(self, base_path: str = "/tmp"):
        self.base_path = Path(base_path)
        self.base_path.mkdir(exist_ok=True)

    def capture_image(self, filename: str, **kwargs) -> bool:
        """Capture image using rpicam-still"""
        output_path = self.base_path / filename

        cmd = ["rpicam-still", "-o", str(output_path)]

        # Add optional parameters
        if kwargs.get("immediate"):
            cmd.append("--immediate")
        if "width" in kwargs:
            cmd.extend(["--width", str(kwargs["width"])])
        if "height" in kwargs:
            cmd.extend(["--height", str(kwargs["height"])])

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if result.returncode == 0:
                logging.info(f"Image captured: {output_path}")
                return True
            else:
                logging.error(f"Camera error: {result.stderr}")
                return False
        except subprocess.TimeoutExpired:
            logging.error("Camera capture timed out")
            return False
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
            return False

# Usage
camera = CameraCapture()
success = camera.capture_image("slam_frame.jpg", immediate=True, width=1920, height=1080)
