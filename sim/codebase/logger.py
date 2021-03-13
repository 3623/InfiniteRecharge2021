# Class which robustly handles logging from multiple sources and
# is time "synchronous"


class Logger():

    def __init__(self):
        self.dict = {}
        self.time = DataList("time")
        self.len = 0
        self.loggers = {}

    def registerLoggerDict(self, logDict, category):
        if category not in self.loggers:
            self.loggers[category] = logDict
            for key in logDict:
                masterKey = category + "." + key
                self.dict[masterKey] = DataList(masterKey, self.len)

    def log(self, time):
        assert self.len == 0 or time > self.time.data[-1], "Time value in past"
        self.len += 1
        self.time.append(time)

        for genre in self.loggers:
            logDict = self.loggers[genre]
            for key in logDict:
                masterKey = genre + "." + key
                self.dict[masterKey].append(logDict[key])

        for listKey in self.dict:
            assert self.dict[listKey].len == self.len, "Logger dict lists not equal length, category not updated"

    def clear(self):
        self.len = 0
        self.time.reset()
        for listKey in self.dict:
            self.dict[listKey].reset()


class DataList():

    def __init__(self, label, noneLength=0):
        self.data = [None for i in range(noneLength)]
        self.label = label
        self.min = 0
        self.max = 0
        self.len = noneLength

    def append(self, data):
        self.data.append(data)
        self.len += 1
        if data is None:
            return
        if data < self.min:
            self.min = data
        elif data > self.max:
            self.max = data

    def reset(self):
        self.data = []
        self.min = 0
        self.max = 0
        self.len = 0

    def __repr__(self):
        return self.label + ": " + str(self.data)


if __name__ == "__main__":
    l = DataList("test")
