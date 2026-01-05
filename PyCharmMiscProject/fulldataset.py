"""
IMU Telemetry Desktop Application with 3D Arrow Visualization
Install required packages:
pip install pyserial matplotlib numpy
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform

class Arrow3D(FancyArrowPatch):
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)
        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)
        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)

class IMUTelemetryApp:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Telemetry Monitor with 3D Visualization")
        self.root.geometry("1400x800")
        self.root.configure(bg='#f0f0f0')

        self.serial_port = None
        self.is_running = False
        self.read_thread = None

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0

        self.offset_qw = 1.0
        self.offset_qx = 0.0
        self.offset_qy = 0.0
        self.offset_qz = 0.0

        self.mapping_mode = 0

        self.setup_ui()

    def next_mapping(self):
        self.mapping_mode = (self.mapping_mode + 1) % 4
        self.mapping_label.config(text=f"Map: {self.mapping_mode}")
        print(f"Switched to mapping mode {self.mapping_mode}")

    def prev_mapping(self):
        self.mapping_mode = (self.mapping_mode - 1) % 4
        self.mapping_label.config(text=f"Map: {self.mapping_mode}")
        print(f"Switched to mapping mode {self.mapping_mode}")

    def setup_ui(self):
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

        self.reset_btn = tk.Button(conn_frame, text="Reset Position", command=self.reset_orientation,
                                   bg='#FF9800', fg='white', padx=15, font=('Arial', 10))
        self.reset_btn.pack(side='left', padx=5)

        tk.Button(conn_frame, text="◀ Prev Map", command=self.prev_mapping,
                 bg='#9C27B0', fg='white', padx=10).pack(side='left', padx=2)
        self.mapping_label = tk.Label(conn_frame, text="Map: 0", bg='#f0f0f0', font=('Arial', 10, 'bold'))
        self.mapping_label.pack(side='left', padx=2)
        tk.Button(conn_frame, text="Next Map ▶", command=self.next_mapping,
                 bg='#9C27B0', fg='white', padx=10).pack(side='left', padx=2)

        main_frame = tk.Frame(self.root, bg='#f0f0f0')
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)

        left_frame = tk.Frame(main_frame, bg='white', relief='raised', bd=1)
        left_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))

        tk.Label(left_frame, text="3D Orientation", bg='white', fg='#2196F3',
                font=('Arial', 14, 'bold')).pack(pady=10)

        self.fig = plt.Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_3d_plot()

        self.canvas = FigureCanvasTkAgg(self.fig, left_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=10)

        right_frame = tk.Frame(main_frame, bg='#f0f0f0')
        right_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))

        canvas = tk.Canvas(right_frame, bg='#f0f0f0', highlightthickness=0, width=550)
        scrollbar = ttk.Scrollbar(right_frame, orient='vertical', command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg='#f0f0f0')

        scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        self.create_sensor_card(scrollable_frame, "Orientation", "orient", "deg")
        self.create_sensor_card(scrollable_frame, "Gyroscope", "gyro", "rad/s")
        self.create_sensor_card(scrollable_frame, "Accelerometer", "accel", "m/s²")
        self.create_sensor_card(scrollable_frame, "Magnetometer", "mag", "µT")
        self.create_sensor_card(scrollable_frame, "Linear Acceleration", "lin_accel", "m/s²")
        self.create_sensor_card(scrollable_frame, "Gravity", "grav", "m/s²")
        self.create_pressure_card(scrollable_frame)
        self.create_gps_card(scrollable_frame)
        self.create_status_card(scrollable_frame)

        self.animate()

    def setup_3d_plot(self):
        self.ax.clear()
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_xlabel('X', fontsize=10)
        self.ax.set_ylabel('Y', fontsize=10)
        self.ax.set_zlabel('Z', fontsize=10)
        self.ax.set_title('Arrow shows IMU orientation', fontsize=12)

        self.ax.plot([0, 1], [0, 0], [0, 0], 'r-', alpha=0.3, linewidth=1)
        self.ax.plot([0, 0], [0, 1], [0, 0], 'g-', alpha=0.3, linewidth=1)
        self.ax.plot([0, 0], [0, 0], [0, 1], 'b-', alpha=0.3, linewidth=1)

    def update_3d_arrow(self):
        self.setup_3d_plot()

        qw_s = self.qw
        qx_s = self.qx
        qy_s = self.qy
        qz_s = self.qz

        # Map 0: Map 17 from previous (yaw correct, pitch/roll reversed directions)
        # Map 1: Invert pitch (first component)
        # Map 2: Invert roll (second component)
        # Map 3: Invert both pitch and roll - SHOULD FIX MAP 17!
        mappings = [
            (-qy_s, -qx_s, -qz_s),  # Map 0: Original map 17
            (qy_s, -qx_s, -qz_s),   # Map 1: Pitch inverted
            (-qy_s, qx_s, -qz_s),   # Map 2: Roll inverted
            (qy_s, qx_s, -qz_s),    # Map 3: Both pitch and roll inverted
        ]

        qx_r, qy_r, qz_r = mappings[self.mapping_mode]
        qw_r = qw_s

        offset_conj_w = self.offset_qw
        offset_conj_x = -self.offset_qx
        offset_conj_y = -self.offset_qy
        offset_conj_z = -self.offset_qz

        qw = offset_conj_w * qw_r - offset_conj_x * qx_r - offset_conj_y * qy_r - offset_conj_z * qz_r
        qx = offset_conj_w * qx_r + offset_conj_x * qw_r + offset_conj_y * qz_r - offset_conj_z * qy_r
        qy = offset_conj_w * qy_r - offset_conj_x * qz_r + offset_conj_y * qw_r + offset_conj_z * qx_r
        qz = offset_conj_w * qz_r + offset_conj_x * qy_r - offset_conj_y * qx_r + offset_conj_z * qw_r

        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])

        arrow_dir = R @ np.array([1.5, 0, 0])
        arrow = Arrow3D(0, 0, 0, arrow_dir[0], arrow_dir[1], arrow_dir[2],
                       mutation_scale=20, lw=3, arrowstyle='->', color='red')
        self.ax.add_artist(arrow)

        side_vec = R @ np.array([0, 0.5, 0])
        top_vec = R @ np.array([0, 0, 0.5])

        self.ax.plot([0, side_vec[0]], [0, side_vec[1]], [0, side_vec[2]], 'b-', alpha=0.5, linewidth=2)
        self.ax.plot([0, -side_vec[0]], [0, -side_vec[1]], [0, -side_vec[2]], 'b-', alpha=0.5, linewidth=2)
        self.ax.plot([0, top_vec[0]], [0, top_vec[1]], [0, top_vec[2]], 'g-', alpha=0.5, linewidth=2)

        self.canvas.draw()

    def animate(self):
        if self.is_running:
            self.update_3d_arrow()
        self.root.after(50, self.animate)

    def create_sensor_card(self, parent, title, key, unit):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text=title, bg='white', fg='#2196F3', font=('Arial', 12, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        data_frame = tk.Frame(card, bg='white')
        data_frame.pack(fill='x', padx=10, pady=(0, 8))

        for axis in ['X', 'Y', 'Z']:
            col = tk.Frame(data_frame, bg='white')
            col.pack(side='left', expand=True, fill='both')

            tk.Label(col, text=axis, bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
            value_label = tk.Label(col, text="0.00", bg='white', fg='#333', font=('Arial', 16, 'bold'))
            value_label.pack()
            setattr(self, f"{key}_{axis.lower()}_label", value_label)
            tk.Label(col, text=unit, bg='white', fg='#999', font=('Arial', 8)).pack()

    def create_pressure_card(self, parent):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text="BMP390 Pressure Sensor", bg='white', fg='#2196F3', font=('Arial', 12, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        data_frame = tk.Frame(card, bg='white')
        data_frame.pack(fill='x', padx=10, pady=(0, 8))

        # Pressure
        col1 = tk.Frame(data_frame, bg='white')
        col1.pack(side='left', expand=True, fill='both')
        tk.Label(col1, text="Pressure", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.pressure_label = tk.Label(col1, text="0.00", bg='white', fg='#333', font=('Arial', 16, 'bold'))
        self.pressure_label.pack()
        tk.Label(col1, text="hPa", bg='white', fg='#999', font=('Arial', 8)).pack()

        # Temperature
        col2 = tk.Frame(data_frame, bg='white')
        col2.pack(side='left', expand=True, fill='both')
        tk.Label(col2, text="Temperature", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.bmp_temp_label = tk.Label(col2, text="0.00", bg='white', fg='#333', font=('Arial', 16, 'bold'))
        self.bmp_temp_label.pack()
        tk.Label(col2, text="°C", bg='white', fg='#999', font=('Arial', 8)).pack()

        # Altitude
        col3 = tk.Frame(data_frame, bg='white')
        col3.pack(side='left', expand=True, fill='both')
        tk.Label(col3, text="Altitude", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.altitude_label = tk.Label(col3, text="0.00", bg='white', fg='#333', font=('Arial', 16, 'bold'))
        self.altitude_label.pack()
        tk.Label(col3, text="m", bg='white', fg='#999', font=('Arial', 8)).pack()

    def create_gps_card(self, parent):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text="GPS Data", bg='white', fg='#2196F3', font=('Arial', 12, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        # Row 1: Lat/Lng
        row1 = tk.Frame(card, bg='white')
        row1.pack(fill='x', padx=10, pady=(0, 4))

        col1 = tk.Frame(row1, bg='white')
        col1.pack(side='left', expand=True, fill='both')
        tk.Label(col1, text="Latitude", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.gps_lat_label = tk.Label(col1, text="0.000000", bg='white', fg='#333', font=('Arial', 14, 'bold'))
        self.gps_lat_label.pack()
        tk.Label(col1, text="°", bg='white', fg='#999', font=('Arial', 8)).pack()

        col2 = tk.Frame(row1, bg='white')
        col2.pack(side='left', expand=True, fill='both')
        tk.Label(col2, text="Longitude", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.gps_lng_label = tk.Label(col2, text="0.000000", bg='white', fg='#333', font=('Arial', 14, 'bold'))
        self.gps_lng_label.pack()
        tk.Label(col2, text="°", bg='white', fg='#999', font=('Arial', 8)).pack()

        # Row 2: Alt/Speed/Sats
        row2 = tk.Frame(card, bg='white')
        row2.pack(fill='x', padx=10, pady=(0, 8))

        col3 = tk.Frame(row2, bg='white')
        col3.pack(side='left', expand=True, fill='both')
        tk.Label(col3, text="Altitude", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.gps_alt_label = tk.Label(col3, text="0.00", bg='white', fg='#333', font=('Arial', 14, 'bold'))
        self.gps_alt_label.pack()
        tk.Label(col3, text="m", bg='white', fg='#999', font=('Arial', 8)).pack()

        col4 = tk.Frame(row2, bg='white')
        col4.pack(side='left', expand=True, fill='both')
        tk.Label(col4, text="Speed", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.gps_speed_label = tk.Label(col4, text="0.00", bg='white', fg='#333', font=('Arial', 14, 'bold'))
        self.gps_speed_label.pack()
        tk.Label(col4, text="km/h", bg='white', fg='#999', font=('Arial', 8)).pack()

        col5 = tk.Frame(row2, bg='white')
        col5.pack(side='left', expand=True, fill='both')
        tk.Label(col5, text="Satellites", bg='white', fg='#666', font=('Arial', 9, 'bold')).pack()
        self.gps_sats_label = tk.Label(col5, text="0", bg='white', fg='#333', font=('Arial', 14, 'bold'))
        self.gps_sats_label.pack()
        tk.Label(col5, text="", bg='white', fg='#999', font=('Arial', 8)).pack()

    def create_status_card(self, parent):
        card = tk.Frame(parent, bg='white', relief='raised', bd=1)
        card.pack(fill='x', pady=5)

        title_label = tk.Label(card, text="System Status", bg='white', fg='#2196F3', font=('Arial', 12, 'bold'))
        title_label.pack(anchor='w', padx=10, pady=(8, 5))

        cal_frame = tk.Frame(card, bg='white')
        cal_frame.pack(fill='x', padx=10, pady=(0, 8))

        for name, key in [('System', 'sys'), ('Gyro', 'gyro'), ('Accel', 'accel'), ('Mag', 'mag')]:
            col = tk.Frame(cal_frame, bg='white')
            col.pack(side='left', expand=True, padx=3)
            tk.Label(col, text=name, bg='white', fg='#666', font=('Arial', 8)).pack()
            cal_box = tk.Label(col, text="0", bg='#FF3B30', fg='white', font=('Arial', 14, 'bold'), width=3, height=1)
            cal_box.pack(pady=3)
            setattr(self, f"cal_{key}_label", cal_box)

        temp_label = tk.Label(card, text="Temperature: 0 °C", bg='white', font=('Arial', 10))
        temp_label.pack(pady=(8, 10))
        self.temp_label = temp_label

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def reset_orientation(self):
        qw_s = self.qw
        qx_s = self.qx
        qy_s = self.qy
        qz_s = self.qz

        mappings = [
            (-qy_s, -qx_s, -qz_s),
            (qy_s, -qx_s, -qz_s),
            (-qy_s, qx_s, -qz_s),
            (qy_s, qx_s, -qz_s),
        ]

        qx_r, qy_r, qz_r = mappings[self.mapping_mode]

        self.offset_qw = qw_s
        self.offset_qx = qx_r
        self.offset_qy = qy_r
        self.offset_qz = qz_r

        print(f"Position reset with mapping mode {self.mapping_mode}")

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

            # Now expecting 35 values: 27 IMU + 3 BMP390 + 5 GPS
            if len(values) >= 35:
                self.yaw = values[0]
                self.roll = values[1]
                self.pitch = values[2]
                self.qw = values[23]
                self.qx = values[24]
                self.qy = values[25]
                self.qz = values[26]

                self.update_label(self.orient_x_label, values[0])
                self.update_label(self.orient_y_label, values[1])
                self.update_label(self.orient_z_label, values[2])
                self.update_label(self.gyro_x_label, values[3])
                self.update_label(self.gyro_y_label, values[4])
                self.update_label(self.gyro_z_label, values[5])
                self.update_label(self.accel_x_label, values[6])
                self.update_label(self.accel_y_label, values[7])
                self.update_label(self.accel_z_label, values[8])
                self.update_label(self.mag_x_label, values[9])
                self.update_label(self.mag_y_label, values[10])
                self.update_label(self.mag_z_label, values[11])
                self.update_label(self.lin_accel_x_label, values[12])
                self.update_label(self.lin_accel_y_label, values[13])
                self.update_label(self.lin_accel_z_label, values[14])
                self.update_label(self.grav_x_label, values[15])
                self.update_label(self.grav_y_label, values[16])
                self.update_label(self.grav_z_label, values[17])

                self.update_calibration(self.cal_sys_label, int(values[18]))
                self.update_calibration(self.cal_gyro_label, int(values[19]))
                self.update_calibration(self.cal_accel_label, int(values[20]))
                self.update_calibration(self.cal_mag_label, int(values[21]))

                self.root.after(0, lambda: self.temp_label.config(text=f"Temperature: {int(values[22])} °C"))

                # Update BMP390 data
                self.update_label(self.pressure_label, values[27])
                self.update_label(self.bmp_temp_label, values[28])
                self.update_label(self.altitude_label, values[29])

                # Update GPS data (indices 30-34)
                self.root.after(0, lambda: self.gps_lat_label.config(text=f"{values[30]:.6f}"))
                self.root.after(0, lambda: self.gps_lng_label.config(text=f"{values[31]:.6f}"))
                self.update_label(self.gps_alt_label, values[32])
                self.update_label(self.gps_speed_label, values[33])
                self.root.after(0, lambda: self.gps_sats_label.config(text=f"{int(values[34])}"))

        except Exception as e:
            print(f"Error parsing data: {e}")

    def update_label(self, label, value):
        self.root.after(0, lambda: label.config(text=f"{value:.2f}"))

    def update_calibration(self, label, value):
        colors = {0: '#FF3B30', 1: '#FF9500', 2: '#FFCC00', 3: '#34C759'}
        self.root.after(0, lambda: label.config(text=str(value), bg=colors[value]))

if __name__ == "__main__":
    root = tk.Tk()
    app = IMUTelemetryApp(root)
    root.mainloop()