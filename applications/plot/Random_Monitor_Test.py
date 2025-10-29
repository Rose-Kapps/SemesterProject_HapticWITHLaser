import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Sample data with added noise
np.random.seed(42)
z_positions = np.linspace(0, 1, 100)
voltages = np.sin(10 * np.pi * z_positions) + np.random.normal(0, 0.2, 100)

# Function to smooth data using a moving average
def smooth_data(data, window_size=5):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# Function to detect peaks with a threshold
def detect_peak(window, threshold=0.8):
    if len(window) < 3:
        return False
    return window[1] > window[0] and window[1] > window[2] and window[1] > threshold

# Smooth the voltage data
smoothed_voltages = smooth_data(voltages, window_size=5)
smoothed_z_positions = z_positions[:len(smoothed_voltages)]

# Initialize the figure and axis
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.2)  # Leave space for the slider

# Initial plot elements
line1, = ax.plot([], [], label='Noisy Voltage', alpha=0.5)
line2, = ax.plot([], [], label='Smoothed Voltage', linewidth=2)
scat = ax.scatter([], [], color='red', label='Peak')

# Formatting the plot
ax.set_xlim(0, 1)
ax.set_ylim(min(voltages) - 0.2, max(voltages) + 0.2)
ax.set_xlabel('Position in z [a.u]')
ax.set_ylabel('Voltage [V]')
ax.set_title('Voltage vs. Position in z')
ax.legend()
ax.grid(True)

# Add slider axis
ax_slider = plt.axes([0.2, 0.05, 0.65, 0.03])  # [left, bottom, width, height]
slider = Slider(ax_slider, 'Z Index', 3, len(z_positions) - 4, valinit=3, valstep=1)

# Function to update the plot when slider moves
def update(z_index):
    z_index = int(slider.val)  # Ensure it's an integer
    
    # Update the plots
    line1.set_data(z_positions[:z_index], voltages[:z_index])
    line2.set_data(smoothed_z_positions[:z_index], smoothed_voltages[:z_index])
    
    # Detect peaks and update scatter plot
    peaks_x, peaks_y = [], []
    state = "outside"
    
    for i in range(2, z_index):
        window = smoothed_voltages[i-2:i+1]
        if detect_peak(window):
            state = "inside" if state == "outside" else "outside"
            peaks_x.append(smoothed_z_positions[i-1])
            peaks_y.append(smoothed_voltages[i-1])
    
    scat.set_offsets(np.c_[peaks_x, peaks_y])  # Update peak markers
    ax.set_title(f'Voltage vs. Position in z (Current state: {state})')
    fig.canvas.draw_idle()  # Redraw the figure

# Connect the slider to the update function
slider.on_changed(update)

# Show the plot
plt.show()
