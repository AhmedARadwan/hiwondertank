import smbus
import struct
import time
import threading

LEFTMOTOR = 0
RIGHTMOTOR = 1

MAXVALUE = 50
MINVALUE = -50


class DifferentialDriveMotor:
    def __init__(self, i2c_bus=1, motor_addr=0x34, motor_type=3, encoder_polarity=0):
        self.bus = smbus.SMBus(i2c_bus)
        self.motor_addr = motor_addr
        self.motor_type = motor_type
        self.encoder_polarity = encoder_polarity
        
        # Register addresses
        self.MOTOR_TYPE_ADDR = 0x14
        self.MOTOR_ENCODER_POLARITY_ADDR = 0x15
        self.MOTOR_FIXED_SPEED_ADDR = 0x33
        self.MOTOR_ENCODER_TOTAL_ADDR = 0x3C
        
        self.left_speed = 0
        self.right_speed = 0
        self.stop_thread = False
        self.speed_thread = None
        
        self.init_motor()
 
    def init_motor(self):
        self.bus.write_byte_data(self.motor_addr, self.MOTOR_TYPE_ADDR, self.motor_type)
        time.sleep(0.05)
        self.bus.write_byte_data(self.motor_addr, self.MOTOR_ENCODER_POLARITY_ADDR, self.encoder_polarity)
    
    def cap_value(self, value):
        if value < MINVALUE:
            return int(MINVALUE)
        elif value > MAXVALUE:
            return int(MAXVALUE)
        else:
            return int(value)

    def set_wheel_speeds(self, left_speed, right_speed):
        speeds = [self.cap_value(left_speed), self.cap_value(right_speed), 0, 0]
        self.bus.write_i2c_block_data(self.motor_addr, self.MOTOR_FIXED_SPEED_ADDR, speeds)
    
    def read_encoder_ticks(self):
        encoder_ticks = struct.unpack('iiii', bytes(self.bus.read_i2c_block_data(self.motor_addr, self.MOTOR_ENCODER_TOTAL_ADDR, 16)))
        left_ticks = encoder_ticks[LEFTMOTOR]
        right_ticks = encoder_ticks[RIGHTMOTOR]
        return left_ticks, right_ticks
    
    def measure_speed_thread(self, interval):
        while not self.stop_thread:
            left_ticks_start, right_ticks_start = self.read_encoder_ticks()
            time.sleep(interval)
            left_ticks_end, right_ticks_end = self.read_encoder_ticks()
            
            self.left_speed = (left_ticks_end - left_ticks_start) / interval
            self.right_speed = (right_ticks_end - right_ticks_start) / interval
 
    def start_speed_measurement(self, interval=1.0):
        self.stop_thread = False
        self.speed_thread = threading.Thread(target=self.measure_speed_thread, args=(interval,))
        self.speed_thread.start()
 
    def stop_speed_measurement(self):
        self.stop_thread = True
        if self.speed_thread:
            self.speed_thread.join()
    
    def get_speeds(self):
        return self.left_speed, self.right_speed
    
    def __del__(self):
        self.stop_speed_measurement()