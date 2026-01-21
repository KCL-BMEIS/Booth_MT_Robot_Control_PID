# Not functional for 4 motors, PID not tuned yet
import serial
import time
import sys
import msvcrt
import matplotlib.pyplot as plt
from simple_pid import PID

#from T249 and stepmotor
STEPS_PER_REV = 200
DEG_PER_STEP = 360 / STEPS_PER_REV
LEAD_PITCH = 8.0 

# feel free to modify the color scheme :)
G_WHITE  = (240, 240, 245)
G_BLUE   = (15, 45, 110)
G_RED    = (210, 30, 45)
G_YELLOW = (255, 215, 0)
G_DARK   = (10, 15, 25)
G_GRAY   = (80, 90, 110)

class MotorControlSystem:
    def __init__(self, port="COM10"):
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.01)
            time.sleep(2)
        except Exception as e:
            print(f"Connection Failed: {e}"); sys.exit()

        self.selected = []
        self.jog_speed = 1
        
        self.fine_deg = int(1.0 / DEG_PER_STEP) 
        self.fine_mm = int((0.1 / LEAD_PITCH) * STEPS_PER_REV)
        self.current_pos_steps = [0, 0, 0, 0]
        self.current_vel_Usteps_per_sec = [0, 0, 0, 0] # in microsteps per 10000 seconds
        self.current_vel_steps_per_sec = [0, 0, 0, 0]  # in steps per second
        # Storage for plotting
        self.plot_time = []
        self.plot_current_pos_steps = [[], [], [], []]
        self.plot_target_steps = [[], [], [], []]
        self.plot_start_time = None
        # Velocity plotting buffers
        self.plot_current_vel_steps = [[], [], [], []]
        self.plot_target_vel_steps = [[], [], [], []]
        # Holds latest target velocities for plotting
        self.target_vel_steps_per_sec = [0, 0, 0, 0]

        # Create PID controllers for each motor
        self.pid_controllers = {}
        for m in range(1, 5):
            self.pid_controllers[m] = PID(
                Kp=10000.0, # 
                Ki=0.0,    # Add integral term 0.01
                Kd=0.00,   # Add derivative term 0.05
                setpoint=0,
                output_limits=(-2500000, 2500000) # must be adjusted!
            )
        self.target_pos_steps = [0, 0, 0, 0]  # Target positions for each motor
        # Create two PIDs per motor
        self.position_pids = {}
        self.vel_pids = {}
        for m in range(1, 5):
            self.position_pids[m] = PID(Kp=50.0, Ki=0.1, Kd=5.0, 
                                         output_limits=(-10000, 10000))  # outputs velocity
            self.vel_pids[m] = PID(Kp=2000.0, Ki=0.5, Kd=0.1, 
                                         output_limits=(-2500000, 2500000))  # outputs motor command
    
    def init_plotting(self):
        """Initialize live plotting for 4 motors."""
        plt.ion()  # Turn on interactive mode
        # Compact stacked layout to fit roughly a quarter screen
        self.fig, self.axes = plt.subplots(4, 1, figsize=(8, 5), sharex=True)
        self.fig.suptitle('Motor Position Control - Cascade PID', fontsize=10)
        self.plot_start_time = time.time()
        self.plot_time = []
        self.plot_current_pos_steps = [[], [], [], []]
        self.plot_target_steps = [[], [], [], []]
        
        # Label each subplot
        for i in range(4):
            ax = self.axes[i]
            ax.set_title(f'Motor {i + 1}', fontsize=9)
            # Only bottom plot shows the X label to save space
            if i == 3:
                ax.set_xlabel('Time (s)', fontsize=8)
            else:
                ax.set_xlabel('')
            ax.set_ylabel('Position (steps)', fontsize=8)
            ax.grid(True, linewidth=0.5)
            ax.tick_params(labelsize=8)
            ax.legend(['Current', 'Target'], fontsize=7, loc='upper right')

        # Second figure: velocities
        self.fig_vel, self.axes_vel = plt.subplots(4, 1, figsize=(8, 5), sharex=True)
        self.fig_vel.suptitle('Motor Velocity - Cascade PID', fontsize=10)
        for i in range(4):
            axv = self.axes_vel[i]
            axv.set_title(f'Motor {i + 1} Velocity', fontsize=9)
            if i == 3:
                axv.set_xlabel('Time (s)', fontsize=8)
            else:
                axv.set_xlabel('')
            axv.set_ylabel('Velocity (steps/s)', fontsize=8)
            axv.grid(True, linewidth=0.5)
            axv.tick_params(labelsize=8)
            axv.legend(['Current vel', 'Target vel'], fontsize=7, loc='upper right')
    
    def update_plot(self):
        """Update live plot with current and target positions."""
        elapsed = time.time() - self.plot_start_time
        self.plot_time.append(elapsed)
        #print("---------> target_vel_steps_per_sec "+ str(self.target_vel_steps_per_sec))
        for i in range(4):
            self.plot_current_pos_steps[i].append(self.current_pos_steps[i])
            self.plot_target_steps[i].append(self.target_pos_steps[i])
            # Velocity traces
            self.plot_current_vel_steps[i].append(self.current_vel_steps_per_sec[i])
            self.plot_target_vel_steps[i].append(self.target_vel_steps_per_sec[i])


        #print("---------> plot_current_vel_steps "+ str(self.plot_current_vel_steps))
        #print("---------> plot_target_vel_steps "+ str(self.plot_target_vel_steps))
        
        # Update each subplot
        for i in range(4):
            ax = self.axes[i]
            ax.clear()
            ax.plot(self.plot_time, self.plot_current_pos_steps[i], label='Current', linewidth=1.2)
            ax.plot(self.plot_time, self.plot_target_steps[i], label='Target', linewidth=1.2, linestyle='--')
            ax.set_title(f'Motor {i + 1}', fontsize=9)
            if i == 3:
                ax.set_xlabel('Time (s)', fontsize=8)
            else:
                ax.set_xlabel('')
            ax.set_ylabel('Position (steps)', fontsize=8)
            ax.grid(True, linewidth=0.5)
            ax.tick_params(labelsize=8)
            ax.legend(fontsize=7, loc='upper right')
        # Draw velocity subplots
        for i in range(4):
            axv = self.axes_vel[i]
            axv.clear()
            axv.plot(self.plot_time, self.plot_current_vel_steps[i], label='Current vel', linewidth=1.2)
            axv.plot(self.plot_time, self.plot_target_vel_steps[i], label='Target vel', linewidth=1.2, linestyle='--')
            axv.set_title(f'Motor {i + 1} Velocity', fontsize=9)
            if i == 3:
                axv.set_xlabel('Time (s)', fontsize=8)
            else:
                axv.set_xlabel('')
            axv.set_ylabel('Velocity (steps/s)', fontsize=8)
            axv.grid(True, linewidth=0.5)
            axv.tick_params(labelsize=8)
            axv.legend(fontsize=7, loc='upper right')
        
        plt.tight_layout(rect=[0, 0, 1, 0.95])
        self.fig_vel.tight_layout(rect=[0, 0, 1, 0.95])
        plt.pause(0.01)
    def __init__(self, port="COM10"):
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.01)
            time.sleep(2)
        except Exception as e:
            print(f"Connection Failed: {e}"); sys.exit()

        self.selected = []
        self.jog_speed = 1
        
        self.current_steps = {1: 0, 2: 0, 3: 0, 4: 0}
        self.target_steps = {1: 0, 2: 0, 3: 0, 4: 0}
        
        self.fine_deg = int(1.0 / DEG_PER_STEP) 
        self.fine_mm = int((0.1 / LEAD_PITCH) * STEPS_PER_REV)

        pygame.init()
        self.screen = pygame.display.set_mode((1050, 650))
        pygame.display.set_caption("MOTOR OS - UC8626")
        
        self.f_head = pygame.font.SysFont("Impact", 38)
        self.f_main = pygame.font.SysFont("Consolas", 24, bold=True)
        self.f_data = pygame.font.SysFont("Consolas", 32, bold=True)
        self.f_small = pygame.font.SysFont("Consolas", 16)
        self.clock = pygame.time.Clock()

# Create PID controllers for each motor
        self.pid_controllers = {}
        for m in range(1, 5):
            self.pid_controllers[m] = PID(
                Kp=20000.0, 
                Ki=0.01,    
                Kd=0.05,   
                setpoint=0,
                output_limits=(-2500000, 2500000) 
            )

    def steps_to_deg(self, steps):
        steps_in_deg = [0,0,0,0]
        for m in range(1, 5):
            steps_in_deg[m] = steps[m] * DEG_PER_STEP
        return steps_in_deg

    def deg_to_steps(self, steps_in_deg):
        steps = [0,0,0,0]
        for m in range (1, 5):
            steps = int(steps_in_deg / DEG_PER_STEP)
        return steps
    
    def steps_to_mm(self, steps):
        steps_in_mm = [0,0,0,0]
        for m in range(1, 5):
            steps_in_mm[m] = (steps[m]/STEPS_PER_REV) * LEAD_PITCH
        return steps_in_mm

    def mm_to_steps(self, steps_in_mm):
        steps = [0,0,0,0]
        for m in range(1, 5):
            steps = int((steps_in_mm[m] / LEAD_PITCH) * STEPS_PER_REV)
        return steps

    def update_feedback(self):
        # Position updates
        self.ser.reset_input_buffer() 
        self.ser.write(b"GET_ALL_POS\n")
        line = self.ser.readline().decode().strip()
        if line.startswith("ALL_POS"):
            p = line.split()
            if len(p) == 5:
                vals = [int(w) for w in p[1:5]]
                self.current_pos_steps[:] = vals
                print(f"Feedback Updated (pos): {self.current_pos_steps}")
    
    def update_position_feedback(self):
        # Position updates
        self.ser.reset_input_buffer() 
        self.ser.write(b"GET_ALL_POS\n")
        line = self.ser.readline().decode().strip()
        if line.startswith("ALL_POS"):
            p = line.split()
            if len(p) == 5:
                vals = [int(w) for w in p[1:5]]
                self.current_pos_steps[:] = vals
            print(f"Feedback Updated (pos): {self.current_pos_steps}")
        #self.update_plot()
    
    def update_velocity_feedback(self):
        # Velocity updates
        self.ser.reset_input_buffer() 
        self.ser.write(b"GET_ALL_VEL\n")
        line = self.ser.readline().decode().strip()
        #print(f"Velocity Line: {line}")
        if line.startswith("ALL_VEL"):
            v = line.split()
            if len(v) == 5:
                vals = [int(t) for t in v[1:5]]
                self.current_vel_Usteps_per_sec[:] = vals
                self.current_vel_steps_per_sec = self.current_vel_Usteps_per_sec#[v / 10000.0 for v in vals] maybe all in Usteps- control is more reasonable
            #print(f"Feedback Updated (velocity ---): {self.current_vel_Usteps_per_sec}")
            #print(f"Feedback Updated (velocity steps/s): {self.current_vel_steps_per_sec}")
        #self.update_plot()

    def run_PID_vel_control_in_steps(self, target_steps):
        for m in range(1, 5):
            # Update setpoint (this happens every frame, totally fine!)
            self.pid_controllers[m].setpoint = target_steps[m-1]
            
            # PID computes control output based on current position
            vel = int(self.pid_controllers[m](self.current_pos_steps[m-1]))
            
            if abs(vel) >= 1:
                self.ser.write(f"SET_VEL {m} {vel}\n".encode())
            else:
                self.ser.write(f"SET_VEL {m} 0\n".encode())
    
    def run_direct_pos_control(self, target_steps):
        for m in range(1, 5):
            # Direct position control: compute velocity towards target
            self.ser.write(f"SET_POS {m} {target_steps[m-1]}\n".encode())

    def run_PID_cascade_control(self):
        self.update_position_feedback()
        self.update_velocity_feedback()
        for m in range(1, 5):
            idx = m - 1
            # Outer loop: position → desired velocity
            self.position_pids[m].setpoint = self.target_pos_steps[idx]
            target_vel_steps_per_sec = self.position_pids[m](self.current_pos_steps[idx])
            #print(f"Motor {m} Target Velocity (steps/s): {target_vel_steps_per_sec}")
            target_vel_steps_per_sec = 20000# # Just for tuning inner loop PID
            
            # Store target velocity for plotting
            self.target_vel_steps_per_sec[idx] = float(target_vel_steps_per_sec)
            
            # Inner loop: velocity → motor command
            self.vel_pids[m].setpoint = self.target_vel_steps_per_sec[idx]
            #motor_cmd = int(self.vel_pids[m](self.current_vel_steps_per_sec[idx])) 
            motor_cmd = int(self.vel_pids[m](self.current_vel_steps_per_sec[idx]))
       
            if abs(motor_cmd) >= 1:
                self.ser.write(f"SET_VEL {m} {motor_cmd}\n".encode())
            else:
                self.ser.write(f"SET_VEL {m} 0\n".encode())
        self.update_plot()

    def reset_motors(self):
        for m in range(1, 5):
            self.ser.write(f"RESET_POS {m} 0\n".encode())
        self.current_pos_steps = [0, 0, 0, 0]
    
    def run_PID_vel_control_in_degs_mm_via_Tic(self, dist_current_targets): # dist [X_mm, Y_mm, Z_mm, A_deg, B_deg_ C_deg], X = Catheter, Z = GW, Y not in use atm
        self.update_feedback()
        target_steps = [0, 0, 0, 0]  # X, Z, A, C
        target_steps[0] = self.mm_to_steps(dist_current_targets[0])  # X axis in mm (front motor for cath)
        target_steps[1] = self.mm_to_steps(dist_current_targets[2])  # Z axis in mm (back motor for GW)
        target_steps[2] = self.deg_to_steps(dist_current_targets[3]) # A axis in deg (left rot motor)
        target_steps[3] = self.deg_to_steps(dist_current_targets[5]) # C axis in deg (not used)
        
        for m in range(1, 4):

            # Update setpoint (this happens every frame, totally fine!)
            self.pid_controllers[m].setpoint = target_steps[m]
            
            # PID computes control output based on current position
            vel = int(self.pid_controllers[m](self.current_pos_steps[m]))
            
            if abs(vel) >= 1:
                self.ser.write(f"SET_VEL {m} {vel}\n".encode())
            else:
                self.ser.write(f"SET_VEL {m} 0\n".encode())

    def sel(self, m):
        if m in self.selected: self.selected.remove(m)
        else: self.selected.append(m)

    def main_pos_control_with_pid_test(self):
        print("Starting Position Control Test...")
        print("Press 'q' to exit.")
        self.reset_motors()
        print("Motors Reset to 0.")
        self.init_plotting()  # Initialize live plotting

        while True:
            # Check for 'q' key press
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == 'q':
                    print("\nExiting...")
                    self.reset_motors()
                    time.sleep(1)
                    break
            
            print("Moving to positions: 100, 100, 100, 100")
            self.target_pos_steps = [100, 100, 100, 100]
            self.run_PID_cascade_control()
            time.sleep(.2)

         
            '''
            self.target_pos_steps = [0, 0, 0, 0]
            self.run_PID_cascade_control()
            print("Moving to positions: 0, 0, 0, 0")
            time.sleep(1)
            '''

    def main_pos_control(self):
        print("Starting Position Control Test...")
        self.reset_motors()
        print("Motors Reset to 0.")
        self.update_feedback()
        while True: # Problem, if time sleep is not long enough, target is not reached and the final target is off! Thats why pid is rquired
                print("Moving to positions: 100, 100, 100, 100")
                self.run_direct_pos_control([100, 100, 100, 100])
                time.sleep(2)
                self.update_feedback()

                self.run_direct_pos_control([0, 0, 0, 0])
                print("Moving to positions: 0, 0, 0, 0")
                time.sleep(2)
                self.update_feedback()

                self.run_direct_pos_control([200, 200, 200, 200])
                print("Moving to positions: 200, 200, 200, 200")
                time.sleep(2)
                self.update_feedback()

                self.reset_motors()
                print("Motors Reset to 0.")
                self.update_feedback()

if __name__ == "__main__":
    app = MotorControlSystem()
    app.main_pos_control_with_pid_test()

