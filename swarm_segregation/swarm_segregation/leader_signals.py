class LeaderSignalHandler:
    def __init__(self):
        self.signals = {"red":False, "green":False, "blue":False}

    def on_red(self, msg, dist):
        if dist < 4.0:
            self.signals["red"] = True

    def on_green(self, msg, dist):
        if dist < 4.0:
            self.signals["green"] = True

    def on_blue(self, msg, dist):
        if dist < 4.0:
            self.signals["blue"] = True

    def reset(self):
        for k in self.signals:
            self.signals[k] = False
    
