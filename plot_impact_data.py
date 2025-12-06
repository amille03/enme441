import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- 1. Configuration ---
# !!! IMPORTANT: Replace with your microcontroller's serial port !!!
SERIAL_PORT = "COM3" 
BAUD_RATE = 115200

# Lists to store the data for plotting
ratios = []
labels = []
hit_count = 0

# --- 2. Matplotlib Setup ---
fig, ax = plt.subplots()
ax.set_title("Bat Impact Ratio Visualization (M1/M2)")
ax.set_xlabel("Hit Number")
ax.set_ylabel("M1/M2 Ratio")
ax.grid(True)
plt.style.use('ggplot')

# Color map for the visualization
color_map = {
    "SWEET_SPOT": 'green',
    "OFF_CENTER": 'red'
}

# --- 3. Serial Communication ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Could not open serial port {SERIAL_PORT}: {e}")
    exit()

# --- 4. Animation Function (Called repeatedly by Matplotlib) ---
def animate(i):
    global hit_count
    
    # Read all lines currently in the serial buffer
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        
        if ',' in line:
            try:
                # Parse the "RATIO,LABEL" format
                ratio_str, label = line.split(',')
                ratio = float(ratio_str)
                
                # Store data
                ratios.append(ratio)
                labels.append(label)
                hit_count += 1
                
                print(f"Hit {hit_count}: Ratio={ratio:.4f}, Label={label}")
            except ValueError:
                # Ignore lines that don't match the expected format (e.g., status/error messages)
                pass
        
    # Clear the previous plot and redraw with new data
    ax.clear()
    
    if ratios:
        # Plot each hit as a scatter point, colored by the label
        colors = [color_map.get(lbl, 'gray') for lbl in labels]
        
        ax.scatter(range(1, hit_count + 1), ratios, c=colors, s=100)
        
        # Draw a line connecting the points
        ax.plot(range(1, hit_count + 1), ratios, color='darkgray', linestyle='--')
        
        # Set the Y-limits based on the data (or manually set for consistency)
        ax.set_ylim(0, max(max(ratios) + 0.5, 2.0))
        
        # Add a horizontal line to visualize the Sweet Spot threshold (1.2)
        ax.axhline(y=1.2, color='blue', linestyle='-', linewidth=1, label='Sweet Spot Threshold (1.2)')

        # Re-apply labels/title after clear()
        ax.set_title("Bat Impact Ratio Visualization (M1/M2)")
        ax.set_xlabel("Hit Number")
        ax.set_ylabel("M1/M2 Ratio")
        ax.legend(loc='upper right')

# --- 5. Run the Live Plot ---
# Set up the animation to call the 'animate' function every 100 milliseconds
ani = animation.FuncAnimation(fig, animate, interval=100)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nPlotting stopped by user.")
finally:
    ser.close()
    print("Serial connection closed.")
