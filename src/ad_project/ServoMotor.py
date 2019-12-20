#RPI ServoMotor


import RPi.GPIO as GPIO
import time

pin = 18 # PWM pin num 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
p = GPIO.PWM(pin, 50)
p.start(0)
cnt = 0
try:
    while True:
        a = int(input())
        i = 5.7 if a == 1 else 10.7
        print "-------"
        while 5.7<=i<=10.7:
            p.ChangeDutyCycle(i)
            print "angle :" + str(i)
            time.sleep(0.05)
            i += 0.1 if a == 1 else -0.1
except KeyboardInterrupt:
    p.stop()
