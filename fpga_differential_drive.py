# ==========================================
# LIBRARY: fpga_differential_drive.py
# DESCRIPTION: Python Interface (API) for the FPGA-based 
# Hardware Accelerator for Differential Drive Kinematics.
# 
# PROTOCOL (UART 115200 Baud, 8-N-1):
# -> PC to FPGA (8 Bytes): [V (4 Bytes)] + [Omega (4 Bytes)]
# <- FPGA to PC (24 Bytes): [phi_r (4)], [phi_l (4)], [x (4)], [y (4)], [theta (4)], [cycles (4)]
# All kinematic values are signed 32-bit Q16.16 Fixed-Point format.
# ==========================================

import serial
import time

class FPGAKinematicsCore:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200, clock_freq=27_000_000):
        """
        Initializes the UART connection to the FPGA hardware.
        
        Args:
            port (str): The serial port FPGA is connected to (e.g., 'COM4' or '/dev/ttyUSB0').
            baud_rate (int): Communication speed. Must match FPGA UART TX/RX parameters.
            clock_freq (int): FPGA system clock frequency in Hz (Default: 27 MHz for Tang Nano 20K).
        """
        self.port = port
        self.baud_rate = baud_rate
        self.multiplier = 65536.0 # Multiplier for Q16.16 fixed-point format
        self.clock_freq = clock_freq
        self.ser = None
        
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=2)
            time.sleep(2) # Wait for the connection to stabilize
            print(f"[INFO] FPGA connected successfully on {self.port}")
        except Exception as e:
            print(f"[ERROR] Could not open port {self.port}. Details: {e}")

    def _float_to_bytes(self, val):
        """Converts a float value to a 32-bit Q16.16 byte array (Big Endian)."""
        return int(val * self.multiplier).to_bytes(4, byteorder='big', signed=True)

    def _bytes_to_float(self, byte_arr):
        """Converts a 32-bit Q16.16 byte array back to a float value."""
        return int.from_bytes(byte_arr, byteorder='big', signed=True) / self.multiplier

    def compute(self, target_v, target_omega):
        """
        Sends target velocities to the FPGA and returns the computed kinematics/odometry.
        
        Args:
            target_v (float): Target linear velocity in m/s.
            target_omega (float): Target angular velocity in rad/s.
            
        Returns:
            dict: A dictionary containing:
                - 'phi_r' (float): Right wheel angular velocity command (rad/s)
                - 'phi_l' (float): Left wheel angular velocity command (rad/s)
                - 'x_pos' (float): Estimated X position (m)
                - 'y_pos' (float): Estimated Y position (m)
                - 'theta' (float): Estimated orientation angle (rad)
                - 'fpga_cycles' (int): Hardware clock cycles consumed
                - 'fpga_time_ns' (float): Hardware execution time in nanoseconds
        """
        if self.ser is None:
            return {"error": "Serial connection is not active."}

        # 1. Pack Data: 4 bytes for V + 4 bytes for Omega = 8 bytes total
        tx_data = self._float_to_bytes(target_v) + self._float_to_bytes(target_omega)
        
        # 2. Trigger FPGA Hardware Calculation
        self.ser.write(tx_data)

        # 3. Read Hardware Response (24 bytes total expected)
        rx_data = self.ser.read(24)

        if len(rx_data) == 24:
            cycles = int.from_bytes(rx_data[20:24], byteorder='big', signed=False)
            fpga_time_ns = cycles * (1 / self.clock_freq) * 1_000_000_000
            
            return {
                "phi_r": self._bytes_to_float(rx_data[0:4]),
                "phi_l": self._bytes_to_float(rx_data[4:8]),
                "x_pos": self._bytes_to_float(rx_data[8:12]),
                "y_pos": self._bytes_to_float(rx_data[12:16]),
                "theta": self._bytes_to_float(rx_data[16:20]),
                "fpga_cycles": cycles,
                "fpga_time_ns": round(fpga_time_ns, 2)
            }
        else:
            return {"error": f"Timeout! Received {len(rx_data)} bytes instead of 24."}

    def close(self):
        if self.ser:
            self.ser.close()

# ==========================================
# EXAMPLE USAGE (For users testing the library)
# ==========================================
if __name__ == '__main__':
    # Initialize the core (Change COM4 to your specific port)
    robot_core = FPGAKinematicsCore(port='COM4')
    
    # Calculate for V=1.5 m/s, Omega=0.5 rad/s
    result = robot_core.compute(target_v=1.5, target_omega=0.5)
    
    # Print results beautifully
    for key, value in result.items():
        print(f"{key}: {value}")
        
    robot_core.close()
