'''
import Jetson.GPIO as GPIO
import time

# Pin definition (replace with your chosen GPIO pin)
PWM_PIN = 15  # Example: using pin 33

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use BOARD pin numbering
GPIO.setup(PWM_PIN, GPIO.IN)

# Function to measure PWM pulse width
def measure_pulse_width(pin):
    while GPIO.input(pin) == 0:  # Wait for the pulse to go high
        #print("0")
        pass
    start_time = time.time()
    while GPIO.input(pin) == 1:  # Wait for the pulse to go low
        print("1.............")
        pass
    end_time = time.time()
    return (end_time - start_time) * 1000000  # Return pulse width in microseconds


try:
    while True:
        pulse_width = measure_pulse_width(PWM_PIN)
        print(f"PWM Pulse Width: {pulse_width:.2f} microseconds", pulse_width)
        time.sleep(0.018)  # Adjust for desired reading frequency

except KeyboardInterrupt:
    GPIO.cleanup()
    print("GPIO Cleanup")
'''
#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time
import threading

# ==== CONFIGURATION ====
FORWARD_PIN = 13   # Board pin number (change to match your wiring)
SIDEWAYS_PIN = 16   # Board pin number (change to match your wiring)
# =======================

GPIO.setmode(GPIO.BOARD)
#GPIO.setup(FORWARD_PIN, GPIO.IN)
GPIO.setup(SIDEWAYS_PIN, GPIO.IN)


#print("Press Ctrl+C to stop")
def SidewaysPinDetect():    
    last_rising = None
    period = None
    try:
        while True:
            # Wait for rising edge
            GPIO.wait_for_edge(SIDEWAYS_PIN, GPIO.RISING)
            now = time.time()
            # Calculate period (time between rising edges)
            if last_rising is not None:
                period = now - last_rising
            last_rising = now

            # Wait for falling edge to measure high time
            GPIO.wait_for_edge(SIDEWAYS_PIN, GPIO.FALLING)
            fall_time = time.time()

            if period:
                high_time = fall_time - last_rising
                freq = 1.0 / period
                duty = (high_time / period) * 100.0
                print("Sideways pin")
                print(f"Freq: {freq:8.2f} Hz | Duty: {duty:5.2f}%")
            else:
                print("Waiting for stable signal...")

    except KeyboardInterrupt:
        print("\nExiting...")
        GPIO.cleanup()
    
def ForwardPinDetect():
    last_rising = None
    period = None
    try:
        while True:
            # Wait for rising edge
            GPIO.wait_for_edge(FORWARD_PIN, GPIO.RISING)
            now = time.time()
            # Calculate period (time between rising edges)
            if last_rising is not None:
                period = now - last_rising
            last_rising = now

            # Wait for falling edge to measure high time
            GPIO.wait_for_edge(FORWARD_PIN, GPIO.FALLING)
            fall_time = time.time()

            if period:
                high_time = fall_time - last_rising
                freq = 1.0 / period
                duty = (high_time / period) * 100.0
                print("Forward pin")
                print(f"Freq: {freq:8.2f} Hz | Duty: {duty:5.2f}%")
            else:
                print("Waiting for stable signal...")

    except KeyboardInterrupt:
        print("\nExiting...")
        GPIO.cleanup()
    

if __name__ == "__main__":
    try:
        SidewaysPinDetect()
        #t1 = threading.Thread(name="Forward Pin", target=ForwardPinDetect)
        #t1.start()
        #t2 = threading.Thread(name="Sideways Pin", target=SidewaysPinDetect)
        #t2.start()
        #print(threading.enumerate())
        #t1.join()
        #t2.join()
        exit(0)
    except:
        GPIO.cleanup()
        exit(1)
