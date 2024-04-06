#!/usr/bin/env python3

from serial import Serial
import time
import sys

class TokenBag:
    def __init__(self, tokens_arr: list[str]) -> None:
        self.buf_ = tokens_arr
        self.i_ = 0
        
    def Lookahead(self):
        if self.i_ >= len(self.buf_):
            return None
        tok = self.buf_[self.i_]
        return tok
    
    def NextToken(self):
        self.i_ = self.i_ + 1

def WaitForSer():
    global ser
    while not ser.is_open:
        print("Waiting for serial connection")
        time.sleep(1)

def WaitForHandshake():
    global ser
    while (ser.readline().decode().rstrip() != "Ready"):
        print("Waiting for handshake")
        ser.write(b"Pi Ready\n")

def GetArgTokens():
    return (" ".join(sys.argv[1:])).split()

def Match(tok):
    global tokens
    if tok != tokens.Lookahead():
            raise Exception("Unexpected token for expression in parse. See Hardwarebridge.py:Match")
    tokens.NextToken()

def Integer():
    try:
        int_val = int(tokens.Lookahead())
        tokens.NextToken()
        return int_val
    except ValueError:
        raise Exception("Tried to match an integer value, but an integer wasn\'t there.")

def Float():
    try:
        float_val = float(tokens.Lookahead())
        tokens.NextToken()
        return float_val
    except ValueError:
        raise Exception("Tried to match a float value, but a float wasn\'t there.")
    
ser = Serial('/dev/ttyACM0', 115200, timeout=1)
WaitForSer()
ser.reset_input_buffer()
WaitForHandshake()
tokens = TokenBag(GetArgTokens())

