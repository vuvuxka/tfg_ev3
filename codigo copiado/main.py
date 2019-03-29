#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import parameters
import importlib
import json

########################################################################
## File I/O functions
########################################################################

# Function for fast reading from sensor files
def FastRead(infile):
    infile.seek(0)
    return(int(infile.read().decode().strip()))

# Function for fast writing to motor files
def FastWrite(outfile,value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()

# Debug print
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# Function to set the duty cycle of the motors
def SetDuty(motorDutyFileHandle, duty):
    # Compansate for nominal voltage and round the input
    dutyInt = int(round(duty*voltageCompensation))

    # Add or subtract offset and clamp the value between -100 and 100
    if dutyInt > 0:
        dutyInt = min(100, dutyInt + frictionOffset)
    elif dutyInt < 0:
        dutyInt = max(-100, dutyInt - frictionOffset)

    # Apply the signal to the motor
    FastWrite(motorDutyFileHandle, dutyInt)

########################################################################
## One-time Hardware setup
########################################################################

# EV3 Brick
powerSupply = ev3.PowerSupply()
buttons = ev3.Button()

# Gyro Sensor setup. Possibly make this more generic to better support more sensors.
try: # Set LEGO gyro to Gyro Rate mode, if attached
    gyroSensor          = ev3.GyroSensor()
    gyroSensor.mode     = gyroSensor.MODE_GYRO_RATE
    gyroType            = 'LEGO-EV3-Gyro'
except: # Assume HiTechnic Gyro is attached if LEGO Gyro not found
    gyroSensor          = ev3.Sensor(address="ev3-ports:in2")
    gyroType            = 'HITECHNIC-NXT-Gyro'

# Open gyro rate sensor value file
gyroSensorValueRaw  = open(gyroSensor._path + "/value0", "rb")

# Touch Sensor setup
touchSensor         = ev3.TouchSensor()
touchSensorValueRaw = open(touchSensor._path + "/value0", "rb")

# IR Buttons setup

# Configure the motors
motorLeft  = ev3.LargeMotor('outD')
motorRight = ev3.LargeMotor('outA')

#############################################################################
## Outer Loop (Uses Touch Sensor to easily start and stop "Balancing Loop")
#############################################################################

while True:

    ########################################################################
    ## Read/reload Parameters
    ########################################################################

    # Reload parameters class
    importlib.reload(parameters)
    powerParameters = parameters.Power()
    gyroParameters  = parameters.Gyro(gyroType)
    motorParameters = parameters.Motor()
    gains           = parameters.Gains()
    timing          = parameters.Timing()

    # Define Math constants and conversions
    radiansPerDegree               = 3.14159/180                                                # The number of radians in a degree.
    radiansPerSecondPerRawGyroUnit = gyroParameters.degPerSecondPerRawGyroUnit*radiansPerDegree # Rate in radians/sec per gyro output unit
    radiansPerRawMotorUnit         = motorParameters.degPerRawMotorUnit*radiansPerDegree        # Angle in radians per motor encoder unit
    radPerSecPerPercentSpeed       = motorParameters.RPMperPerPercentSpeed*6*radiansPerDegree   # Actual speed in radians/sec per unit of motor speed

    # Read battery voltage
    voltageIdle = powerSupply.measured_volts
    voltageCompensation = powerParameters.voltageNominal/voltageIdle

    # Offset to limit friction deadlock
    frictionOffset = int(round(powerParameters.frictionOffsetNominal*voltageCompensation))

    #Timing settings for the program
    loopTimeSec             = timing.loopTimeMiliSec/1000  # Time of each loop, measured in seconds.
    loopCount               = 0                            # Loop counter, starting at 0

    # A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
    motorAngleHistory = deque([0],timing.motorAngleHistoryLength)

    # The rate at which we'll update the gyro offset (precise definition given in docs)
    gyroDriftCompensationRate      = timing.gyroDriftCompensationFactor*loopTimeSec*radiansPerSecondPerRawGyroUnit

    ########################################################################
    ## Hardware (Re-)Config
    ########################################################################

    # Reset the motors
    motorLeft.reset()                   # Reset the encoder
    motorRight.reset()
    motorLeft.run_direct()              # Set to run direct mode
    motorRight.run_direct()

    # Open sensor files for (fast) reading
    motorEncoderLeft    = open(motorLeft._path + "/position", "rb")
    motorEncoderRight   = open(motorRight._path + "/position", "rb")

    # Open motor files for (fast) writing
    motorDutyCycleLeft = open(motorLeft._path + "/duty_cycle_sp", "w")
    motorDutyCycleRight= open(motorRight._path + "/duty_cycle_sp", "w")

    ########################################################################
    ## Definitions and Initialization variables
    ########################################################################

    # Reset variables representing physical signals
    motorAngleRaw              = 0 # The angle of "the motor", measured in raw units (degrees for the EV3). We will take the average of both motor positions as "the motor" angle, wich is essentially how far the middle of the robot has traveled.
    motorAngle                 = 0 # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
    motorAngleReference        = 0 # The reference angle of the motor. The robot will attempt to drive forward or backward, such that its measured position equals this reference (or close enough).
    motorAngleError            = 0 # The error: the deviation of the measured motor angle from the reference. The robot attempts to make this zero, by driving toward the reference.
    motorAngleErrorAccumulated = 0 # We add up all of the motor angle error in time. If this value gets out of hand, we can use it to drive the robot back to the reference position a bit quicker.
    motorAngularSpeed          = 0 # The motor speed, estimated by how far the motor has turned in a given amount of time
    motorAngularSpeedReference = 0 # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
    motorAngularSpeedError     = 0 # The error: the deviation of the motor speed from the reference speed.
    motorDutyCycle             = 0 # The 'voltage' signal we send to the motor. We calulate a new value each time, just right to keep the robot upright.
    gyroRateRaw                = 0 # The raw value from the gyro sensor in rate mode.
    gyroRate                   = 0 # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
    gyroEstimatedAngle         = 0 # The gyro doesn't measure the angle of the robot, but we can estimate this angle by keeping track of the gyroRate value in time
    gyroOffset                 = 0 # Over time, the gyro rate value can drift. This causes the sensor to think it is moving even when it is perfectly still. We keep track of this offset.

    # Print start and stop instructions
    eprint("Hold robot upright. Press Touch Sensor to start. Or any button to exit.")

    # Wait for Touch Sensor or any Button Press
    while not touchSensor.is_pressed and not buttons.any():
        time.sleep(0.01)

    # If any of the buttons was pressed, exit the program by breaking the outer loop
    if buttons.any():
        break

    # Otherwise, if it was the Touch Sensor, wait for release and proceed to calibration and balancing
    while touchSensor.is_pressed:
        time.sleep(0.01)

    ########################################################################
    ## Create empty datalogs
    ########################################################################

    datalog = {
       'timeStart' : time.strftime("UTC: %Y-%m-%d-%H:%M:%S"),
       'tLoopStart' : [],
       'gyroRate' : [],
       'gyroEstimatedAngle' : [],
       'motorAngle' : [],
       'motorAngleError' : [],
       'motorAngularSpeed' : [],
       'motorAngleErrorAccumulated' : [],
       'motorDutyCycle' : [],
       'gyroOffset': []
    }

    ########################################################################
    ## Calibrate Gyro
    ########################################################################

    eprint("-------\nCalibrating\n-------")

    #As you hold the robot still, determine the average sensor value of 100 samples
    gyroRateCalibrateCount = 100
    for i in range(gyroRateCalibrateCount):
        gyroOffset = gyroOffset + FastRead(gyroSensorValueRaw)
        time.sleep(0.01)
    gyroOffset = gyroOffset/gyroRateCalibrateCount

    # Print calibration result
    eprint("GyroOffset: ",gyroOffset)
    eprint("-----------\nGo!\n-----------")

    ########################################################################
    ## Balancing Loop
    ########################################################################

    # Remember start time
    tProgramStart = time.time()

    # Initial fast read touch sensor value
    touchSensorPressed = False

    # Keep looping until Touch Sensor is pressed again
    while not touchSensorPressed:

        ###############################################################
        ##  Loop info
        ###############################################################
        loopCount = loopCount + 1
        tLoopStart = time.time() - tProgramStart

        ###############################################################
        ##
        ##  Driving and Steering. Modify this section as you like to
        ##  make your segway go anywhere!
        ##
        ##  To begin, uncomment one of the examples below, and modify
        ##  from there
        ##
        ###############################################################

        # Example 1: Doing nothing: just balance in place:
        speed    = 0
        steering = 0


        ###############################################################
        ##  Reading the Gyro.
        ###############################################################
        gyroRateRaw = FastRead( gyroSensorValueRaw)
        gyroRate = (gyroRateRaw - gyroOffset)*radiansPerSecondPerRawGyroUnit
        eprint("Angulo (radianes): ",gyroEstimatedAngle)
        eprint("Velocidad Angular (radianes): ",gyroRate)

        ###############################################################
        ##  Reading the Motor Position
        ###############################################################

        motorAngleRaw = (FastRead(motorEncoderLeft) + FastRead(motorEncoderRight))/2
        motorAngle = motorAngleRaw*radiansPerRawMotorUnit

        motorAngularSpeedReference = speed*radPerSecPerPercentSpeed
        motorAngleReference = motorAngleReference + motorAngularSpeedReference*loopTimeSec

        motorAngleError = motorAngle - motorAngleReference
        eprint("Rotacion del motor (radianes): ",motorAngleError)

        ###############################################################
        ##  Computing Motor Speed
        ###############################################################

        motorAngularSpeed = (motorAngle - motorAngleHistory[0])/(timing.motorAngleHistoryLength*loopTimeSec)
        motorAngularSpeedError = motorAngularSpeed# - motorAngularSpeedReference # Uncommenting this leads to a raher abrubt change in speed when using the remote. So I'll leave it commented until I add some code that gradually increases ths reference when a button is pressed or depressed
        motorAngleHistory.append(motorAngle)
        eprint("Velocidad de rotacion (radianes): ",motorAngularSpeed)

        ###############################################################
        ##  Computing the motor duty cycle value
        ###############################################################

        motorDutyCycle =( gains.GyroAngle  * gyroEstimatedAngle
                        + gains.GyroRate   * gyroRate
                        + gains.MotorAngle * motorAngleError
                        + gains.MotorAngularSpeed * motorAngularSpeedError
                        + gains.MotorAngleErrorAccumulated * motorAngleErrorAccumulated)

        eprint("Potencia: ",motorDutyCycle)
        ###############################################################
        ##  Apply the signal to the motor, and add steering
        ###############################################################

        SetDuty(motorDutyCycleRight, motorDutyCycle + steering)
        SetDuty(motorDutyCycleLeft , motorDutyCycle - steering)

        ###############################################################
        ##  Update angle estimate and Gyro Offset Estimate
        ###############################################################

        gyroEstimatedAngle = gyroEstimatedAngle + gyroRate*loopTimeSec

        gyroOffset = (1-gyroDriftCompensationRate)*gyroOffset+gyroDriftCompensationRate*gyroRateRaw

        ###############################################################
        ##  Update Accumulated Motor Error
        ###############################################################

        motorAngleErrorAccumulated = motorAngleErrorAccumulated + motorAngleError*loopTimeSec

        ###############################################################
        ##  Read the touch sensor (the kill switch)
        ###############################################################

        touchSensorPressed = FastRead(touchSensorValueRaw)

        ###############################################################
        ##  Append data datalog
        ###############################################################

        datalog['tLoopStart']                .append(tLoopStart)
        datalog['gyroRate']                  .append(gyroRate)
        datalog['gyroEstimatedAngle']        .append(gyroEstimatedAngle)
        datalog['motorAngle']                .append(motorAngle)
        datalog['motorAngleError']           .append(motorAngleError)
        datalog['motorAngularSpeed']         .append(motorAngularSpeed)
        datalog['motorAngleErrorAccumulated'].append(motorAngleErrorAccumulated)
        datalog['motorDutyCycle']            .append(motorDutyCycle)
        datalog['gyroOffset']                .append(gyroOffset)

        ###############################################################
        ##  Busy wait for the loop to complete
        ###############################################################

        while(time.time()-tProgramStart - tLoopStart <  loopTimeSec):
            time.sleep(0.0001)
    ########################################################################
    ##
    ## Closing down & Cleaning up
    ##
    ########################################################################

    # Loop end time, for stats
    tProgramEnd = time.time()

    # Turn off the motors
    FastWrite(motorDutyCycleLeft ,0)
    FastWrite(motorDutyCycleRight,0)

    # Wait for the Touch Sensor to be released
    while touchSensor.is_pressed:
        time.sleep(0.01)

    # Calculate loop time
    tLoop = (tProgramEnd - tProgramStart)/loopCount
    eprint("Loop time:", tLoop*1000,"ms")

    # Write datalog file
    with open('datalog.txt', 'w') as f:
        f.write(json.dumps(datalog))

    # Print a stop message
    eprint("----------\nStopped\n--------")
