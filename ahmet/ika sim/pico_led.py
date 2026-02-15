import time
from machine import Pin

led = Pin("LED", Pin.OUT)

print("Blink başladı")

while True:
    led.toggle()
    print("toggle")
    time.sleep(1)
