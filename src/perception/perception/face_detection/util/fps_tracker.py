import time

class FPSTracker:
    def __init__(self, averaging_window=30):
        self.timestamps = []
        self.averaging_window = averaging_window

    def update(self):
        now = time.time()
        self.timestamps.append(now)
        if len(self.timestamps) > self.averaging_window:
            self.timestamps.pop(0)

    def get_fps(self):
        if len(self.timestamps) < 2:
            return 0.0
        time_span = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / time_span if time_span > 0 else 0.0
