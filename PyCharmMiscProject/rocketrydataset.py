"""
Altimeter Telemetry Desktop Application
Install required packages:
pip install pyserial matplotlib numpy
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

class AltimeterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Altimeter Telemetry Monitor")
        self.root.geometry("1200x800")
        self.root.configure(bg='#f0f0f0')

        self.serial_port = None
        self.is_running = False
        self.read_thread = None

        # Data storage
        self.baro_alt = 0
        self.gps_alt = 0
        self.gps_lat = 0
        self.gps_lng = 0
        self.accel_z = 0
        self.speed = 0
        self.temperature = 0

        # Barometric altitude offset (in feet)
        self.baro_offset = 0

        # Data history for graphs (last 100 points)
        self.time_data = deque(maxlen=100)
        self.baro_alt_history = deque(maxlen=100)
        self.gps_alt_history = deque(maxlen=100)
        self.accel_z_history = deque(maxlen=100)
        self.speed_history = deque(maxlen=100)
        self.start_time = time.time()

        self.setup_ui()

    def setup_ui(self):
        # Connection Frame
        conn_frame = tk.Frame(self.root, bg='#f0f0f0')
        conn_frame.pack(pady=10, padx=10, fill='x')

        tk.Label(conn_frame, text="Serial Port:", bg='#f0f0f0', font=('Arial', 10)).pack(side='left', padx=5)

        self.port_combo = ttk.Combobox(conn_frame, width=30, state='readonly')
        self.port_combo.pack(side='left', padx=5)
        self.refresh_ports()

        self.refresh_btn = tk.Button(conn_frame, text="Refresh", command=self.refresh_ports,
                                     bg='#4CAF50', fg='white', padx=10)
        self.refresh_btn.pack(side='left', padx=5)

        self.connect_btn = tk.Button(conn_frame, text="Connect", command=self.toggle_connection,
                                     bg='#2196F3', fg='white', padx=20, font=('Arial', 10, 'bold'))
        self.connect_btn.pack(side='left', padx=5)

        self.status_label = tk.Label(conn_frame, text="● Disconnected",
                                     bg='#f0f0f0', fg='red', font=('Arial', 10, 'bold'))
        self.status_label.pack(side='left', padx=10)

        # Barometric Offset Control
        tk.Label(conn_frame, text="Baro Offset:", bg='#f0f0f0', font=('Arial', 10)).pack(side='left', padx=(20, 5))

        self.offset_entry = tk.Entry(conn_frame, width=8, font=('Arial', 10))
        self.offset_entry.insert(0, "0")
        self.offset_entry.pack(side='left', padx=5)

        tk.Label(conn_frame, text="ft", bg='#f0f0f0', font=('Arial', 10)).pack(side='left', padx=(0, 5))

        self.set_offset_btn = tk.Button(conn_frame, text="Set Offset", command=self.set_offset,
                                       bg='#FF9800', fg='white', padx=10)
        self.set_offset_btn.pack(side='left', padx=5)

        self.zero_btn = tk.Button(conn_frame, text="Zero Here", command=self.zero_altitude,
                                 bg='#9C27B0', fg='white', padx=10)
        self.zero_btn.pack(side='left', padx=5)

        # Main content frame
        main_frame = tk.Frame(self.root, bg='#f0f0f0')
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)

        # Left side - Data cards
        left_frame = tk.Frame(main_frame, bg='#f0f0f0')
        left_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))

        self.create_altitude_card(left_frame)
        self.create_gps_card(left_frame)
        self.create_motion_card(left_frame)
        self.create_temperature_card(left_frame)

        # Right side - Graphs
        right_frame = tk.Frame(main_frame, bg='white', relief='raised', bd=1)
        right_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))

        self.setup_graphs(right_frame)

        self.animate()

    def set_offset(self):
        """Set the barometric altitude offset from entry field"""
        try:
            self.baro_offset = float(self.offset_entry.get())
            messagebox.showinfo("Offset Set", f"Barometric altitude offset set to {self.baro_offset:.2f} ft")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number")

    def zero_altitude(self):
        """Set the current barometric altitude as zero (ground level)"""
        if self.baro_alt != 0:
            # Set offset so current altitude reads as 0
            self.baro_offset = -self.baro_alt
            self.offset_entry.delete(0, tk.END)
            self.offset_entry.insert(0, f"{self.baro_offset:.2f}")
            messagebox.showinfo("Zeroed", f"Barometric altitude zeroed at current position\nOffset: {self.baro_offset:.2f} ft")
        else:
            messagebox.showwarning("Warning", "Waiting for altitude data...")

    def create_altitude_card(self, parent):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text="Altitude", bg='white', fg='#2196F3', font=('Arial', 14, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        data_frame = tk.Frame(card, bg='white')
        data_frame.pack(fill='x', padx=10, pady=(0, 8))

        # Barometric Altitude
        col1 = tk.Frame(data_frame, bg='white')
        col1.pack(side='left', expand=True, fill='both')
        tk.Label(col1, text="Barometric", bg='white', fg='#666', font=('Arial', 10, 'bold')).pack()
        self.baro_alt_label = tk.Label(col1, text="0.00", bg='white', fg='#333', font=('Arial', 24, 'bold'))
        self.baro_alt_label.pack()
        tk.Label(col1, text="feet", bg='white', fg='#999', font=('Arial', 9)).pack()

        # GPS Altitude
        col2 = tk.Frame(data_frame, bg='white')
        col2.pack(side='left', expand=True, fill='both')
        tk.Label(col2, text="GPS", bg='white', fg='#666', font=('Arial', 10, 'bold')).pack()
        self.gps_alt_label = tk.Label(col2, text="0.00", bg='white', fg='#333', font=('Arial', 24, 'bold'))
        self.gps_alt_label.pack()
        tk.Label(col2, text="feet", bg='white', fg='#999', font=('Arial', 9)).pack()

    def create_gps_card(self, parent):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text="GPS Position", bg='white', fg='#2196F3', font=('Arial', 14, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        data_frame = tk.Frame(card, bg='white')
        data_frame.pack(fill='x', padx=10, pady=(0, 8))

        # Latitude
        col1 = tk.Frame(data_frame, bg='white')
        col1.pack(side='left', expand=True, fill='both')
        tk.Label(col1, text="Latitude", bg='white', fg='#666', font=('Arial', 10, 'bold')).pack()
        self.gps_lat_label = tk.Label(col1, text="0.000000", bg='white', fg='#333', font=('Arial', 16, 'bold'))
        self.gps_lat_label.pack()
        tk.Label(col1, text="°", bg='white', fg='#999', font=('Arial', 9)).pack()

        # Longitude
        col2 = tk.Frame(data_frame, bg='white')
        col2.pack(side='left', expand=True, fill='both')
        tk.Label(col2, text="Longitude", bg='white', fg='#666', font=('Arial', 10, 'bold')).pack()
        self.gps_lng_label = tk.Label(col2, text="0.000000", bg='white', fg='#333', font=('Arial', 16, 'bold'))
        self.gps_lng_label.pack()
        tk.Label(col2, text="°", bg='white', fg='#999', font=('Arial', 9)).pack()

    def create_motion_card(self, parent):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text="Motion", bg='white', fg='#2196F3', font=('Arial', 14, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        data_frame = tk.Frame(card, bg='white')
        data_frame.pack(fill='x', padx=10, pady=(0, 8))

        # Speed
        col1 = tk.Frame(data_frame, bg='white')
        col1.pack(side='left', expand=True, fill='both')
        tk.Label(col1, text="Speed", bg='white', fg='#666', font=('Arial', 10, 'bold')).pack()
        self.speed_label = tk.Label(col1, text="0.00", bg='white', fg='#333', font=('Arial', 24, 'bold'))
        self.speed_label.pack()
        tk.Label(col1, text="mph", bg='white', fg='#999', font=('Arial', 9)).pack()

        # Vertical Acceleration
        col2 = tk.Frame(data_frame, bg='white')
        col2.pack(side='left', expand=True, fill='both')
        tk.Label(col2, text="Vertical Accel", bg='white', fg='#666', font=('Arial', 10, 'bold')).pack()
        self.accel_z_label = tk.Label(col2, text="0.00", bg='white', fg='#333', font=('Arial', 24, 'bold'))
        self.accel_z_label.pack()
        tk.Label(col2, text="m/s²", bg='white', fg='#999', font=('Arial', 9)).pack()

    def create_temperature_card(self, parent):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text="Temperature", bg='white', fg='#2196F3', font=('Arial', 14, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        data_frame = tk.Frame(card, bg='white')
        data_frame.pack(fill='x', padx=10, pady=(0, 8))

        col1 = tk.Frame(data_frame, bg='white')
        col1.pack(expand=True)
        self.temp_label = tk.Label(col1, text="0.00", bg='white', fg='#333', font=('Arial', 32, 'bold'))
        self.temp_label.pack()
        tk.Label(col1, text="°C", bg='white', fg='#999', font=('Arial', 12)).pack()

    def setup_graphs(self, parent):
        tk.Label(parent, text="Real-Time Data", bg='white', fg='#2196F3',
                font=('Arial', 14, 'bold')).pack(pady=10)

        # Create figure with 2x2 subplots
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(8, 7))
        self.fig.tight_layout(pad=3.0)

        # Altitude comparison
        self.ax1.set_title('Altitude Comparison', fontsize=10, fontweight='bold')
        self.ax1.set_ylabel('Altitude (ft)')
        self.ax1.grid(True, alpha=0.3)
        self.line_baro, = self.ax1.plot([], [], 'b-', label='Barometric', linewidth=2)
        self.line_gps, = self.ax1.plot([], [], 'r-', label='GPS', linewidth=2)
        self.ax1.legend(loc='upper left', fontsize=8)

        # Speed
        self.ax2.set_title('Speed', fontsize=10, fontweight='bold')
        self.ax2.set_ylabel('Speed (mph)')
        self.ax2.grid(True, alpha=0.3)
        self.line_speed, = self.ax2.plot([], [], 'g-', linewidth=2)

        # Vertical Acceleration
        self.ax3.set_title('Vertical Acceleration', fontsize=10, fontweight='bold')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Accel (m/s²)')
        self.ax3.grid(True, alpha=0.3)
        self.line_accel, = self.ax3.plot([], [], 'm-', linewidth=2)

        # Altitude vs Speed
        self.ax4.set_title('Altitude vs Speed', fontsize=10, fontweight='bold')
        self.ax4.set_xlabel('Speed (mph)')
        self.ax4.set_ylabel('Altitude (ft)')
        self.ax4.grid(True, alpha=0.3)
        self.scatter = self.ax4.scatter([], [], c='blue', alpha=0.5, s=20)

        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=10)

    def update_graphs(self):
        if len(self.time_data) > 0:
            time_list = list(self.time_data)

            # Update altitude comparison
            self.line_baro.set_data(time_list, list(self.baro_alt_history))
            self.line_gps.set_data(time_list, list(self.gps_alt_history))
            self.ax1.relim()
            self.ax1.autoscale_view()

            # Update speed
            self.line_speed.set_data(time_list, list(self.speed_history))
            self.ax2.relim()
            self.ax2.autoscale_view()

            # Update acceleration
            self.line_accel.set_data(time_list, list(self.accel_z_history))
            self.ax3.relim()
            self.ax3.autoscale_view()

            # Update scatter plot
            self.scatter.set_offsets(list(zip(self.speed_history, self.baro_alt_history)))
            self.ax4.relim()
            self.ax4.autoscale_view()

            self.canvas.draw()

    def animate(self):
        if self.is_running:
            self.update_graphs()
        self.root.after(100, self.animate)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_running:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Error", "Please select a serial port")
            return

        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)

            self.is_running = True
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

            self.connect_btn.config(text="Disconnect", bg='#F44336')
            self.status_label.config(text="● Connected", fg='#4CAF50')
            self.port_combo.config(state='disabled')

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        self.is_running = False
        if self.read_thread:
            self.read_thread.join(timeout=2)

        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None

        self.connect_btn.config(text="Connect", bg='#2196F3')
        self.status_label.config(text="● Disconnected", fg='red')
        self.port_combo.config(state='readonly')

    def read_serial(self):
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line.startswith("ERROR") or line == "READY":
                        continue
                    self.parse_data(line)
            except Exception as e:
                print(f"Error reading serial: {e}")
                self.root.after(0, self.disconnect)
                break
            time.sleep(0.01)

    def parse_data(self, line):
        try:
            values = [float(x) for x in line.split(',')]

            if len(values) >= 35:
                # Extract only the data we care about from the CSV
                # accel_z is at index 8, pressure at 27, temp at 28,
                # altitude at 29, gps_lat at 30, gps_lng at 31,
                # gps_alt at 32, speed at 33

                self.accel_z = values[8]  # Z acceleration
                self.temperature = values[28]  # BMP temperature

                # Convert meters to feet and apply offset
                baro_alt_meters = values[29]  # Barometric altitude in meters
                self.baro_alt = (baro_alt_meters * 3.28084) + self.baro_offset  # Convert to feet and add offset

                self.gps_lat = values[30]
                self.gps_lng = values[31]

                # Convert GPS altitude from meters to feet
                gps_alt_meters = values[32]
                self.gps_alt = gps_alt_meters * 3.28084  # Convert to feet

                # Convert speed from km/h to mph
                speed_kmh = values[33]
                self.speed = speed_kmh * 0.621371  # Convert to mph

                # Update labels
                self.update_label(self.baro_alt_label, self.baro_alt)
                self.update_label(self.gps_alt_label, self.gps_alt)
                self.root.after(0, lambda: self.gps_lat_label.config(text=f"{self.gps_lat:.6f}"))
                self.root.after(0, lambda: self.gps_lng_label.config(text=f"{self.gps_lng:.6f}"))
                self.update_label(self.speed_label, self.speed)
                self.update_label(self.accel_z_label, self.accel_z)
                self.update_label(self.temp_label, self.temperature)

                # Store data for graphs
                current_time = time.time() - self.start_time
                self.time_data.append(current_time)
                self.baro_alt_history.append(self.baro_alt)
                self.gps_alt_history.append(self.gps_alt)
                self.accel_z_history.append(self.accel_z)
                self.speed_history.append(self.speed)

        except Exception as e:
            print(f"Error parsing data: {e}")

    def update_label(self, label, value):
        self.root.after(0, lambda: label.config(text=f"{value:.2f}"))

if __name__ == "__main__":
    root = tk.Tk()
    app = AltimeterApp(root)
    root.mainloop()