import time

class LeaderSignalHandler:
    def __init__(self, hold_time=0.5):
        self.signals = {"red":False, "green":False, "blue":False}
        self.timestamps = {"red": 0.0, "green": 0.0, "blue": 0.0}
        self.hold_time = hold_time  # seconds

    def on_red(self, msg, dist):
        if dist < 4.0:
            self.signals["red"] = True
            self.timestamps["red"] = time.time()

    def on_green(self, msg, dist):
        if dist < 4.0:
            self.signals["green"] = True
            self.timestamps["green"] = time.time()

    def on_blue(self, msg, dist):
        if dist < 4.0:
            self.signals["blue"] = True
            self.timestamps["blue"] = time.time()

    def get_active_signals(self):
        now = time.time()
        active = []
        for c in self.signals:
            if now - self.timestamps[c] <= self.hold_time:
                active.append(c)
        return active
    
