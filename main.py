#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# import morse code dictionary object
from morse_code import MORSE_CODE_DICT

import time as time_lib
import threading

# Creating ev3 object
ev3 = EV3Brick()

# Initialise sensors and motor objects and connect to port
motor_right = Motor(Port.C) 
motor_left = Motor(Port.B) 
sensor_touch = TouchSensor(Port.S1)
sensor_color = ColorSensor(Port.S2)
sensor_infrared = InfraredSensor(Port.S3)

# Initialse global variables to be used throughout program
moving = False
times = []
timer = StopWatch()
boundary_timer = StopWatch()
obstructed_timer = StopWatch()
boundary_time = 0
obstructed_time = 0
color_now = None
obstructed = False
hit_boundary = False

def handle_sensor_pressed():
    global moving 

    if sensor_touch.pressed(): 
        moving = not moving
        if moving: 
            timer.reset()
            color_now = sensor_color.color()

def is_obstructed():
    while True: 
        global obstructed, moving
        
        if sensor_infrared.distance() < 10: 
            obstructed = True
        else: 
            obstructed = False

def has_hit_boundary(): 
    while True: 
        global hit_boundary
        
        boundary_color = sensor_color.color()
        if boundary_color != Color.RED and boundary_color != Color.WHITE and boundary_color != Color.BROWN and boundary_color != None:
            hit_boundary = True
        else: 
            hit_boundary = False

def realign_robot():
    print("In realign robot!")
    time = 1000
    
    while True: 
        if sensor_color.color() == Color.YELLOW: 
            motor_right.run_time(50, time, then=Stop.HOLD, wait=True)       
            motor_left.run_time(-50, time, then=Stop.HOLD, wait=True)                           
            if sensor_color.color() == Color.RED or sensor_color.color() == Color.WHITE:
                break
        elif sensor_color.color() == Color.BLACK: 
            motor_left.run_time(50, time, then=Stop.HOLD, wait=True)
            motor_right.run_time(-50, time, then=Stop.HOLD, wait=True)                           
            if sensor_color.color() == Color.RED or sensor_color.color() == Color.WHITE:
                break

        time += 1000

def remove_boundary_time_intervals(times): 
    """
    This function takes list of time dictionary objects and removes all 'boundary'
    time intervals, the combines the times either side as if the boundary hit had 
    not occurred.  

    Parameters:
    - times (list): A list of dictionary objects in the format {"time": 1.2, colour: "red"}

    Returns:
    - times (list): A list of dictionary objects in the format {"time": 1.2, colour: "red"}
    """
    x = 0
    while x < len(times):
        if times[x]['color'] == 'boundary':
            # Check if we can merge the previous and next entries
            if x > 0 and x < len(times) - 1 and times[x-1]['color'] == times[x+1]['color']:
                # Update the time of the previous entry by adding the time of the next entry
                times[x-1]['time'] += times[x+1]['time']
                # Remove the boundary entry and the next entry
                del times[x:x+2]
            else:
                # Just remove the boundary entry if no merging is possible
                del times[x]
        else: 
            x += 1

    return times

def decode_message(morse_code): 
    """
    This function takes in a string of morse_code, decodes it into an alphanumeric
    message and returns the results. 

    Parameters:
    - morse_code (string): A string containing morse code in a format like '-.. . ..-.   --. ....'

    Returns:
    - string: A message containing the decoded morse code such as 'DEF GH'

    Raises:
    - ExceptionType: Explanation of under what condition this exception is raised, if any.

    Example:
    message = decode_message('-.. . ..-.   --. ....')
    print(message) # DEF GH
    """
    decoded_message = ""

    words = morse_code.split('   ')
    print(words)
    
    for word in words: 
        characters = word.split(' ')
        print(characters)
        decoded_word = ''
        for character in characters:
            letter = MORSE_CODE_DICT.get(character, '')
            decoded_word += letter

        decoded_message += decoded_word + ' '

    return decoded_message

def remove_time_outliers(times):
    """
    This function takes in a list/array of times, removes any outliers by 
    calculating the interquartile range (IQR) and then removing out any numbers 
    outside of this range and returns the new filtered list. 

    Parameters:
    - times (list): A list of times in seconds such as [1.2, 3.2, 5.2, 1.2, 45.5]

    Returns:
    - list: A list of times in seconds such as [1.2, 3.2, 5.2, 1.2]

    Raises:
    - ExceptionType: Explanation of under what condition this exception is raised, if any.

    Example:
    new_times = remove_time_outliers([1.2, 3.2, 5.2, 1.2, 45.5])
    print(new_times) # [1.2, 3.2, 5.2, 1.2]
    """

    # Outliers = below Q1 - 1.5 x IQR or above Q3 + 1.5 x IQR
    times_sorted = sorted(times)

    # Calculate 25th and 75th percentile
    q1_index_location = len(times_sorted) // 4 # 25th % index
    q3_index_location = 3 * len(times_sorted) // 4 # 75th % index
    q1 = times_sorted[q1_index_location]
    q3 = times_sorted[q3_index_location]
    
    # Calculate the inter quartile range and thresholds
    IQR = q3 - q1
    below_threshold = q1 - 1.5 * IQR
    above_threshold = q3 + 1.5 * IQR

    print(below_threshold)
    print(above_threshold)

    updated_times = []
    for time in times: 
        if time >= below_threshold and time <= above_threshold: 
            updated_times.append(time)

    return updated_times

def decode_morse_code_data(times):
    red_times = []
    white_times = []

    for time in times: 
        if time["color"] == "red":
            red_times.append(time["time"])
        elif time["color"] == "white":
            white_times.append(time["time"])

    # Deconstructing times and getting thresholds
    min_dot_time = min(red_times)  
    max_dash_time = max(red_times)
    dash_threshold = (min_dot_time + max_dash_time) / 2
    
    white_times = remove_time_outliers(white_times)
    min_interchar_time = min(white_times)  
    max_char_time = max(white_times)
    char_threshold = (min_interchar_time + max_char_time) / 2

    # Decode to morse code
    morse_code = ""

    for time in times: 
        if time["color"] == "red":
            if time["time"] < dash_threshold:
                morse_code += '.'
            else: 
                morse_code += '-'
        elif time["color"] == "white":
            if time["time"] in white_times: 
                if time["time"] < char_threshold:
                    morse_code += '' 
                else:
                    morse_code += ' ' # Character space
            else: 
                morse_code += '   '
    
    return morse_code

def capture_morse_code_data(): 

    global moving, times, timer, boundary_timer, obstructed_timer, boundary_time, obstructed_time, color_now, obstructed, hit_boundary

    while True:
        if sensor_touch.pressed(): 
            moving = not moving
            if moving: 
                timer.reset()
                color_now = sensor_color.color()

        if moving:
            motor_left.run(200)
            motor_right.run(200)

            if hit_boundary:
                print("In hit boundary!")
                motor_left.stop()
                motor_right.stop()
                boundary_timer.reset()
                realign_robot()
                boundary_time_interval = (boundary_timer.time() / 1000)
                motor_left.run(200)
                motor_right.run(200)

                boundary_time = {
                    "color": "boundary", 
                    "time": boundary_time_interval
                }
                times.append(boundary_time)
                boundary_timer.reset()
                boundary_time = 0
            
            if sensor_color.color() == Color.RED or sensor_color.color() == Color.WHITE: 
                # Wait until the color changes.
                color_now = sensor_color.color()
                while sensor_color.color() == color_now:
                    if obstructed: 
                        motor_left.stop()
                        motor_right.stop()
                        obstructed_timer.reset() 
                        while obstructed: 
                            print("Obstructed!")
                        obstructed_time = (obstructed_timer.time() / 1000)
                        motor_left.run(200)
                        motor_right.run(200)

                    if not moving: 
                        return

                    wait(5)

                time_interval = ((timer.time() - obstructed_time - boundary_time) / 1000) # Seconds

                time = {
                    "color": "red" if color_now == Color.RED else "white",
                    "time": time_interval
                }
                times.append(time)
                timer.reset()
                obstructed_time = 0

        else: 
            if len(times) != 0: 
                motor_left.stop()
                motor_right.stop()
                print(times)
                return times 

# Start program
threading.Thread(target=is_obstructed).start()
threading.Thread(target=has_hit_boundary).start()
threading.Thread(target=handle_sensor_pressed).start()
ev3.speaker.beep()
ev3.speaker.say("go")
times = capture_morse_code_data()
print(times)
times = remove_boundary_time_intervals(times)
print(times)
morse_code = decode_morse_code_data(times)
print(morse_code)
message = decode_message(morse_code)
print(message)
ev3.screen.draw_text(10, 50, message)
wait(10000)






