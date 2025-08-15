import Jetson.GPIO as GPIO
import time
import threading

FORWARD_PIN = 13   # Board pin number (change to match your wiring)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(FORWARD_PIN, GPIO.IN)

def ForwardPinDetect():
    last_rising = None
    period = None
    max_duty_cycle = None
    max_frequency = None
    min_duty_cycle = None
    min_frequency = None
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
                if max_duty_cycle is None or duty > max_duty_cycle:
                    max_duty_cycle = duty
                if min_duty_cycle is None or min_duty_cycle > duty:
                    min_duty_cycle = duty
                
                if max_frequency is None or max_frequency < freq:
                    max_frequency = freq
                if min_frequency is None or min_frequency > freq:
                    min_frequency = freq
                    
                print("Forward pin")
                print(f"Freq: {freq:8.2f} Hz | Duty: {duty:5.2f}%")
            else:
                print("Waiting for stable signal...")

    except KeyboardInterrupt:
        print("\nExiting...")
        GPIO.cleanup()
    
    print(f"Max Freq: {max_frequency:8.2f}, Min Freq: {min_frequency:8.2f}")
    print(f"Max Duty Cycle: {max_duty_cycle:8.2f}, Min Duty Cycle: {min_duty_cycle:8.2f}")
        
if __name__ == "__main__":
    try:
        ForwardPinDetect()
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
