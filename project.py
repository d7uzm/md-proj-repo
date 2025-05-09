import time
import board
import adafruit_ads7830.ads7830 as ADC
from adafruit_ads7830.analog_in import AnalogIn

import adafruit_midi
import usb_midi
import neopixel
from adafruit_midi.note_on import NoteOn
from adafruit_midi.note_off import NoteOff

print("\n" * 20)
print("Loading Complete.")

# MIDI setup
midi_out_channel = 1
midi = adafruit_midi.MIDI(
    midi_in=usb_midi.ports[0],
    midi_out=usb_midi.ports[1],
    out_channel=midi_out_channel - 1,
    debug=False
)

# ADC setup
i2c = board.I2C()
adc = ADC.ADS7830(i2c)
photoresistors = [AnalogIn(adc, i) for i in range(7)]

# NeoPixel setup
pixel_pin = board.D6
num_pixels = 22
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.3, auto_write=False)
pixels.fill((255, 255, 255))
pixels.show()

# Constants
DEBOUNCE_DELAY = 0.05
notes = [48, 50, 52, 53, 55, 57, 59] # distinct note value per trigger

# Sensor config
pl_map = [1, 0, 3, 2, 4, 5, 6]  # PHYSICAL INDEXING
light_threshold = [22000, 28000, 20000, 0, 15000, 23000, 20500]
sensor_states = [False] * len(photoresistors)  # False = not triggered

def is_trigger_condition(level, threshold, note_active, is_last):
    if note_active:
        return False
    return (level < threshold) if not is_last else (level >= threshold)

def process_sensor(i, sensor):
    sensor_index = pl_map[i]
    level = sensor.value
    note = notes[sensor_index]
    threshold = light_threshold[sensor_index]
    is_last = sensor_index == pl_map[-1]
    active = sensor_states[sensor_index]

    if is_trigger_condition(level, threshold, active, is_last):
        midi.send(NoteOn(note, 100))

        sensor_states[sensor_index] = True
        return True

    return False

def reset_sensor_states():
    for i in range(len(sensor_states)):
        sensor_states[i] = False

def all_conditions_false():
    for i, sensor in enumerate(photoresistors):
        sensor_index = pl_map[i]
        level = sensor.value
        threshold = light_threshold[sensor_index]
        is_last = sensor_index == pl_map[-1]

        if (level < threshold and not is_last) or (level >= threshold and is_last):
            return False

    return True  # No sensors are in triggering state

# Main loop
while True:
    for i, sensor in enumerate(photoresistors):
        process_sensor(i, sensor)

    if all_conditions_false():
        reset_sensor_states()

    print("sensor_states:", sensor_states)
    time.sleep(DEBOUNCE_DELAY)
