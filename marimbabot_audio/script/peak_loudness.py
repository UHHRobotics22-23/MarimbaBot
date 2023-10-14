import sounddevice as sd
import numpy as np
import time

# Constants
duration = 5  # Duration for measuring loudness in seconds
sample_rate = 44100  # Sample rate (you can adjust this as needed)
input_device_id = 18  # Change this to the ID of your desired input device

# Print a list of all available input devices
print(sd.query_devices())

# Find Scarlett 2i2 USB input device ID
for i in range(len(sd.query_devices())):
    if 'Scarlett 2i2 USB' in sd.query_devices()[i]['name']:
        print(f"Found Scarlett 2i2 USB at index {i}")
        input_device_id = i

def measure_peak_loudness(indata, frames, _, status):
    # Calculate the peak loudness from the audio data
    peak_loudness = np.abs(indata).mean()
    peak_dB = 20 * np.log10(peak_loudness + 1e-6)
    peak_dB = max(-96, peak_dB)  # Limit the minimum to -96 dB to prevent extreme values
    peak_loudness_values.append((peak_dB, time.time()))
    # Remove the oldest peak loudness value if the list is longer than the specified duration
    if peak_loudness_values[-1][1] - peak_loudness_values[0][1] > duration:
        peak_loudness_values.pop(0)

def strided_app(a, L, S ):  # Window len = L, Stride len/stepsize = S
    nrows = ((a.size-L)//S)+1
    n = a.strides[0]
    return np.lib.stride_tricks.as_strided(a, shape=(nrows,L), strides=(S*n,n))


# Get the input device info
input_info = sd.query_devices(input_device_id)

# Initialize an empty list to store peak loudness values
peak_loudness_values = []

# Start recording audio using the specified input device
with sd.InputStream(callback=measure_peak_loudness, channels=1, device=input_device_id, samplerate=sample_rate):
    while True:
        sd.sleep(500)  # Sleep for the specified duration in milliseconds

        # Calculate the average peak loudness over the 5-second span
        average_peak_dB = np.max(np.asarray(peak_loudness_values)[:, 0])
        print(f"Average Peak Loudness: {average_peak_dB:.2f} dB")
