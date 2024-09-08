import tkinter as tk
from tkinter import ttk
import serial
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image
import numpy as np
import serial.tools.list_ports

# Function to close all open serial ports
def clear_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        try:
            ser = serial.Serial(port.device)
            ser.close()
            print(f"Closed port: {port.device}")
        except (OSError, serial.SerialException):
            pass

# Initialize serial communication
clear_serial_ports()  # Clear all open serial ports first
ser = serial.Serial('/dev/tty.usbserial-AQ01LY3S', 115200, timeout=1)

class USVGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("USV Tracking GUI")

        self.frame = tk.Frame(self.root, padx=10, pady=10)
        self.frame.pack(fill=tk.BOTH, expand=True)

        self.text_area = tk.Text(self.frame, height=10, padx=5, pady=5, bg='#f0f0f0')
        self.text_area.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))

        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.usv_positions = {}
        self.estimated_position = None

        self.update_thread = threading.Thread(target=self.update_data)
        self.update_thread.daemon = True
        self.update_thread.start()

    def update_data(self):
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                self.text_area.insert(tk.END, line + '\n')
                self.text_area.see(tk.END)
                self.process_data(line)

    def process_data(self, data):
        parts = data.split(',')
        if len(parts) == 5 and parts[0] == 'DATA':
            try:
                usv_id = parts[1].strip()
                lat = float(parts[2])
                lon = float(parts[3])
                signal = int(parts[4])
                self.usv_positions[usv_id] = (lat, lon, signal)
            except ValueError:
                print(f"Error: Could not parse data: {data}")
        elif len(parts) == 4 and parts[0] == 'ESTIMATED':
            try:
                est_lat = float(parts[1])
                est_lon = float(parts[2])
                self.estimated_position = (est_lat, est_lon)
            except ValueError:
                print(f"Error: Could not parse estimated position: {data}")
        else:
            print(f"Warning: Unrecognized data format: {data}")

        self.plot_data()

    def plot_data(self):
        self.ax.clear()

        # Add maritime map background
        img = Image.open("background.jpg")
        self.ax.imshow(img, extent=[-90.1, -89.8, 30.0, 30.2], aspect='auto')

        for usv_id, (lat, lon, signal) in self.usv_positions.items():
            self.ax.plot(lon, lat, 'ro', markersize=10, label=f'USV {usv_id} (Signal: {signal} dB)')
            self.ax.text(lon, lat, f'USV {usv_id}\nLat: {lat:.4f}\nLon: {lon:.4f}\nSignal: {signal} dB', fontsize=9, ha='right')

            if self.estimated_position:
                est_lat, est_lon = self.estimated_position
                self.draw_bearing_line(lat, lon, est_lat, est_lon)

        if self.estimated_position:
            self.ax.plot(self.estimated_position[1], self.estimated_position[0], 'go', markersize=10, label='Estimated SOS')
            self.ax.text(self.estimated_position[1], self.estimated_position[0], f'Estimated SOS\nLat: {self.estimated_position[0]:.4f}\nLon: {self.estimated_position[1]:.4f}', fontsize=9, ha='right')

        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        self.ax.set_title('USV and SOS Tracking')
        self.canvas.draw()

    def draw_bearing_line(self, usv_lat, usv_lon, target_lat, target_lon):
        """Draw a line of bearing from the USV to the target (SOS location)."""
        # Calculate the direction vector
        direction_vector = np.array([target_lon - usv_lon, target_lat - usv_lat])
        direction_vector /= np.linalg.norm(direction_vector)  # Normalize to unit vector
        
        # Scale the line for visualization
        scale_factor = 0.01  # Adjust this scale for line length
        end_point = (usv_lon + scale_factor * direction_vector[0], usv_lat + scale_factor * direction_vector[1])

        # Plot the line
        self.ax.plot([usv_lon, end_point[0]], [usv_lat, end_point[1]], 'b-', linewidth=2)

if __name__ == "__main__":
    root = tk.Tk()
    gui = USVGUI(root)
    root.mainloop()
