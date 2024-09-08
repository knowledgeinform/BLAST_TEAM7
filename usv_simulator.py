import tkinter as tk
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image
import numpy as np
import time

class USVSimulator:
    def __init__(self, num_usvs=3):
        self.usv_positions = {f'U{i+1}': self.random_position() for i in range(num_usvs)}
        self.estimated_position = (30.1, -90.0)  # Fixed estimated SOS location for simulation

    def random_position(self):
        # Generate a random starting position within a specified range
        lat = np.random.uniform(30.05, 30.15)
        lon = np.random.uniform(-90.05, -89.95)
        signal = np.random.randint(50, 100)  # Simulate signal strength
        return lat, lon, signal

    def update_positions(self):
        # Simulate USV movement towards the estimated SOS position
        for usv_id, (lat, lon, signal) in self.usv_positions.items():
            new_lat = lat + np.random.uniform(-0.001, 0.001)
            new_lon = lon + np.random.uniform(-0.001, 0.001)
            new_signal = signal + np.random.randint(-1, 2)  # Random signal fluctuation
            self.usv_positions[usv_id] = (new_lat, new_lon, new_signal)

    def get_data(self):
        # Simulate the data format as if it was read from serial
        data_lines = []
        for usv_id, (lat, lon, signal) in self.usv_positions.items():
            data_lines.append(f"DATA,{usv_id},{lat},{lon},{signal}")
        return data_lines

class USVGUI:
    def __init__(self, root, simulator):
        self.root = root
        self.simulator = simulator
        self.root.title("USV Tracking GUI (Simulation)")

        self.frame = tk.Frame(self.root, padx=10, pady=10)
        self.frame.pack(fill=tk.BOTH, expand=True)

        self.text_area = tk.Text(self.frame, height=10, padx=5, pady=5, bg='#f0f0f0')
        self.text_area.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))

        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.update_thread = threading.Thread(target=self.update_data)
        self.update_thread.daemon = True
        self.update_thread.start()

    def update_data(self):
        while True:
            self.simulator.update_positions()
            data_lines = self.simulator.get_data()
            for line in data_lines:
                self.text_area.insert(tk.END, line + '\n')
                self.text_area.see(tk.END)
                self.process_data(line)
            time.sleep(1)  # Update every second

    def process_data(self, data):
        parts = data.split(',')
        if parts[0] == 'DATA':
            usv_id = parts[1].strip()
            lat = float(parts[2])
            lon = float(parts[3])
            signal = int(parts[4])
            self.simulator.usv_positions[usv_id] = (lat, lon, signal)

        self.plot_data()

    def plot_data(self):
        self.ax.clear()

        # Add maritime map background
        img = Image.open("background.jpg")
        self.ax.imshow(img, extent=[-90.1, -89.8, 30.0, 30.2], aspect='auto')

        for usv_id, (lat, lon, signal) in self.simulator.usv_positions.items():
            self.ax.plot(lon, lat, 'ro', markersize=10, label=f'USV {usv_id} (Signal: {signal} dB)')
            self.ax.text(lon, lat, f'USV {usv_id}\nLat: {lat:.4f}\nLon: {lon:.4f}\nSignal: {signal} dB', fontsize=9, ha='right')

            est_lat, est_lon = self.simulator.estimated_position
            self.draw_bearing_line(lat, lon, est_lat, est_lon)

        est_lat, est_lon = self.simulator.estimated_position
        self.ax.plot(est_lon, est_lat, 'go', markersize=10, label='Estimated SOS')
        self.ax.text(est_lon, est_lat, f'Estimated SOS\nLat: {est_lat:.4f}\nLon: {est_lon:.4f}', fontsize=9, ha='right')

        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        self.ax.set_title('USV and SOS Tracking (Simulation)')
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
    simulator = USVSimulator(num_usvs=3)  # Simulate 3 USVs
    root = tk.Tk()
    gui = USVGUI(root, simulator)
    root.mainloop()
