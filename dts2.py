import ctypes
import time

class Temperature(object):
    def __init__(self, libPath):
        self.lib = ctypes.CDLL(libPath)
        self.obj = self.lib.Temperature_new()

    def check(self):
        self.lib.Temperature_check(self.obj)

    def get_result(self):
        self.lib.Temperature_get_result.restype = ctypes.c_char_p
        result = self.lib.Temperature_get_result(self.obj)
        return result.decode()

if __name__ == "__main__":
    while True:
        temperature_lib = "./temperature.so"
        temperature = Temperature(temperature_lib)
        temperature.check()
        result = temperature.get_result()
        temperature_value = result.replace("Object : ", "")
        print(result)
        time.sleep(1)
