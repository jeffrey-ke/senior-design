from uuid import uuid1
class Logger:
    file_handle = None
    count = 0
    def GiveLine(self, line: str):
        if line == "NEW":
            self._SetNewFile()
        else:
            self._WriteToFile(line)

    def _WriteToFile(self, msg: str):
        if self.file_handle is None:
            self._SetNewFile()
        self.file_handle.write("{}\n".format(msg))

    def _SetNewFile(self):
        self.count = self.count + 1
        if self.file_handle is not None:
            self.file_handle.close()
        self.file_handle = open("file{}_{}.csv".format(self.count, uuid1()), "w")
        pass

