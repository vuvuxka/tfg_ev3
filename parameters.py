class Gains:
    # For every radian (57 degrees) we lean forward,            apply this amount of duty cycle.
    GyroAngle                  = 1700
    # For every radian/s we fall forward,                       apply this amount of duty cycle.
    GyroRate                   = 120
    # For every radian we are ahead of the reference,           apply this amount of duty cycle
    MotorAngle                 = 7
    # For every radian/s drive faster than the reference value, apply this amount of duty cycle
    MotorAngularSpeed          = 9
    # For every radian x s of accumulated motor angle,          apply this amount of duty cycle
    MotorAngleErrorAccumulated = 3

class Gyro:
    def __init__(self, gyroType):
        if(gyroType == 'LEGO-EV3-Gyro'):
            # Amount of deg/s per sensor unit For the LEGO EV3 Gyro
            self.degPerSecondPerRawGyroUnit     =  1
        elif(gyroType == 'HITECHNIC-NXT-Gyro'):
            # Amount of deg/s per sensor unit for the HiTechnic NXT Gyro
            self.degPerSecondPerRawGyroUnit     =  0.2084

class Motor:
    # For the LEGO EV3 Large Motor 1 unit = 1 deg
    degPerRawMotorUnit             = 1

    # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled)
    RPMperPerPercentSpeed          = 1.7

class Power:
    # Voltage with respect to which we tune the parameters
    voltageNominal = 8.0

    # Add this amount to any positive duty cycle; subtract this amount from any negative duty cycle
    frictionOffsetNominal = 3

class Timing:
    #Timing settings for the program
    loopTimeMiliSec             = 30                    # Time of each loop, measured in miliseconds.
    motorAngleHistoryLength     = 5                     # Number of previous motor angle samples we keep track of.
    gyroDriftCompensationFactor = 0.05

