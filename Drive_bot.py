import smbus
import RPi.GPIO as GPIO
import time
 

GPIO.setmode (GPIO.BCM) #setting the pin in BCM mode
GPIO.setwarnings(False) #to prevent warnings


cfx=0
cfy=0
in1 = 05 ##setting up the pins for motor driver i.e L298
in2 = 06
in3 = 20
in4 = 21
en1 = 12
en2 = 13

GPIO.setup (in1,GPIO.OUT) ## initialising input/output of RPi
GPIO.setup (in2,GPIO.OUT)
GPIO.setup (en1,GPIO.OUT)
GPIO.setup (in3,GPIO.OUT)
GPIO.setup (in4,GPIO.OUT)
GPIO.setup (en2,GPIO.OUT)

pwm1 = GPIO.PWM (en1 , 100) #setting pwm for motor1 with frequency 100Hz
pwm2 = GPIO.PWM (en2 , 100) #setting pwm for motor2 with frequency 100Hz

GPIO.output (in1,GPIO.HIGH) ## cofiguring both motors to move forward
GPIO.output (in2,GPIO.LOW)
GPIO.output (in3,GPIO.HIGH)
GPIO.output (in4,GPIO.LOW)

pow_1 = 0x6b #register to power up module

def read_byte(adr):
    return bus.read_byte_data (address, adr)

def read_word(adr): #the values of gyroscope and accelerometer in hexadecimal
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high<<8) + low ## linking lower and higher values
    return val

def read_word_2c(adr): ## function ot get signed values from the module
    val = read_word(adr)
    if(val >= 0x8000):
        return -((65535-val) + 1)
    else :
        return val
    pwm1.start(50) ## starting pwm signals for both the motors with 50% duty cycle
    pwm2.start(50)

while True:
    bus = smbus.SMBus(1) ##starting the system management bus of the i2c
    address = 0x68 ## adress where the module is connected to 
    bus.write_byte_data(address,pow_1,1)

    gyro_x = read_word_2c(0x43)/131  ##reading the raw data of gyroscope
    gyro_y = read_word_2c(0x45)/131
    gyro_z = read_word_2c(0x47)/131

    acc_x = read_word_2c(0x3b)/16384.0  ## reading the raw data of accelerometer
    acc_y = read_word_2c(0x3d)/16384.0
    acc_z = read_word_2c(0x3f)/16384.0

    cfx = round(0.98*(cfx + gyro_x) + 0.02*acc_x)  ##combining data of both gyro and accelerometer by using complimentary filter
    print ("angle of x-axis",cfx)
        
    if cfx!=00: ## checking the ideal condition
        while cfx>00: ## if the angle is greater the to be speeding the left (completly based on assumption)
            for x in range(50,100,5): 
                print ("speeding left motor")
                pwm2.ChangeDutyCycle(x)
                time.sleep(1)
                print (cfx)
                if cfx == 00: ## after every increment in value of duty cycle checking in the ideal condition is achived or not
                    print ("now it will move straight")
                break
            break   

        
        while cfx<00:
            for y in range(50,100,5):
                print ("speeding right motor")
                pwm1.ChangeDutyCycle(y)
                time.sleep(1)
                print (cfx)
                if cfx == 00:
                    print ("now it will move straight")
                break
            break
