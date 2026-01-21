import serial
import time
import pygame
import sys
import random
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
        self.current_steps = [0, 0, 0, 0]
        '''        
        pygame.init()
        self.screen = pygame.display.set_mode((1050, 650))
        pygame.display.set_caption("MOTOR OS - UC8626")
        
        self.f_head = pygame.font.SysFont("Impact", 38)
        self.f_main = pygame.font.SysFont("Consolas", 24, bold=True)
        self.f_data = pygame.font.SysFont("Consolas", 32, bold=True)
        self.f_small = pygame.font.SysFont("Consolas", 16)
        self.clock = pygame.time.Clock()
        '''
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
    
    def steps_to_deg(self, steps):
        return steps * DEG_PER_STEP
    def deg_to_steps(self, deg):
        return int(deg / DEG_PER_STEP)

    def steps_to_mm(self, steps):
        return (steps / STEPS_PER_REV) * LEAD_PITCH
    def mm_to_steps(self, mm):
        return int((mm / LEAD_PITCH) * STEPS_PER_REV)

    def update_feedback(self):
        self.ser.reset_input_buffer() 
        self.ser.write(b"GET_ALL_POS\n")
        line = self.ser.readline().decode().strip()
        if line.startswith("ALL_POS"):
            p = line.split()
            if len(p) == 5:
                vals = [int(v) for v in p[1:5]]
                self.current_steps[:] = vals
                print(f"Feedback Updated: {self.current_steps}")

    def run_PID_vel_control_in_steps(self, target_steps):
        for m in range(1, 5):
            # Update setpoint (this happens every frame, totally fine!)
            self.pid_controllers[m].setpoint = target_steps[m-1]
            
            # PID computes control output based on current position
            vel = int(self.pid_controllers[m](self.current_steps[m-1]))
            
            if abs(vel) >= 1:
                self.ser.write(f"SET_VEL {m} {vel}\n".encode())
            else:
                self.ser.write(f"SET_VEL {m} 0\n".encode())


    def run_direct_pos_control(self, target_steps):
        for m in range(1, 5):
            # Direct position control: compute velocity towards target
            self.ser.write(f"SET_POS {m} {target_steps[m-1]}\n".encode())


    def reset_motors(self):
        for m in range(1, 5):
            self.ser.write(f"RESET_POS {m} 0\n".encode())
        self.current_steps = [0, 0, 0, 0]

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
            vel = int(self.pid_controllers[m](self.current_steps[m]))
            
            if abs(vel) >= 1:
                self.ser.write(f"SET_VEL {m} {vel}\n".encode())
            else:
                self.ser.write(f"SET_VEL {m} 0\n".encode())

    def draw_ui(self, mode, text):# nothing important below, just UI stuff
        self.screen.fill(G_DARK)
        
        pygame.draw.rect(self.screen, G_RED, (0, 0, 1050, 60))
        head_t = self.f_head.render("MOTOR COMMAND", True, G_WHITE)
        self.screen.blit(head_t, (20, 5))

        for i in range(1, 5):
            x = 50 if i % 2 != 0 else 550
            y = 100 if i <= 2 else 260
            rect = pygame.Rect(x, y, 450, 140)
            
            bg_col = G_BLUE if i in self.selected else (30, 35, 50)
            pygame.draw.rect(self.screen, bg_col, rect, border_radius=5)
            pygame.draw.rect(self.screen, G_YELLOW if i in self.selected else G_GRAY, rect, 3, border_radius=5)
            unit = "DEG" if i <= 2 else "MM"
            lbl = self.f_main.render(f"AXIS-0{i} [{unit}]", True, G_YELLOW if i in self.selected else G_WHITE)
            self.screen.blit(lbl, (x + 20, y + 15))

            # Data
            cur = (self.current_steps[i]*DEG_PER_STEP) if i <= 2 else (self.current_steps[i]/200*8)
            tar = (self.target_steps[i]*DEG_PER_STEP) if i <= 2 else (self.target_steps[i]/200*8)
            cur_t = self.f_data.render(f"POS: {cur:>8.2f}", True, G_WHITE)
            tar_t = self.f_small.render(f"TARGET: {tar:.2f}", True, G_GRAY if i not in self.selected else G_WHITE)
            self.screen.blit(cur_t, (x + 20, y + 55))
            self.screen.blit(tar_t, (x + 20, y + 100))


        cons_rect = pygame.Rect(50, 450, 950, 150)
        pygame.draw.rect(self.screen, (20, 25, 40), cons_rect)
        pygame.draw.rect(self.screen, G_BLUE, cons_rect, 2)

        stat_col = G_RED if mode else G_GRAY
        mode_t = self.f_main.render(f"SYSTEM MODE: {mode if mode else 'STANDBY'}", True, stat_col)
        val_t = self.f_data.render(f"INPUT > {text}_", True, G_WHITE)
        self.screen.blit(mode_t, (70, 470))
        self.screen.blit(val_t, (70, 510))
        help_t = self.f_small.render("L/I/O/P:SELECT | A/D:JOG | T:ABS | R:REL | Z:ZERO | Q:EXIT", True, G_GRAY)
        self.screen.blit(help_t, (70, 570))

        pygame.display.flip()
    
    def run(self):
        in_mode, in_txt = None, ""
        for _ in range(5): self.update_feedback(); time.sleep(0.1)
        test_targets_in_steps = {1: 100, 2: 200, 3: 50, 4: 75}

        while True:
            
            # Test 1
            self.update_feedback()
            self.run_PID_vel_control_in_steps(test_targets_in_steps)

            # Test 2 (please comment out linees 161 and 162 to use this)
            #self.run_PID_vel_control_in_degs_mm_via_Tic([self.steps_to_mm(test_targets_in_steps[1]), 0, self.steps_to_mm(test_targets_in_steps[2]), self.steps_to_deg(test_targets_in_steps[3]), 0, self.steps_to_deg(test_targets_in_steps[4])])
            
            keys = pygame.key.get_pressed()
            if not in_mode:
                if keys[pygame.K_a]:
                    for m in self.selected: self.target_steps[m] -= self.jog_speed
                if keys[pygame.K_d]:
                    for m in self.selected: self.target_steps[m] += self.jog_speed

            self.draw_ui(in_mode, in_txt)

            for event in pygame.event.get():
                if event.type == pygame.QUIT: return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_l: self.sel(1)
                    if event.key == pygame.K_i: self.sel(2)
                    if event.key == pygame.K_o: self.sel(3)
                    if event.key == pygame.K_p: self.sel(4)
                    if event.key == pygame.K_q: return

                    if not in_mode:
                        if event.key == pygame.K_a:
                            for m in self.selected: self.target_steps[m] -= self.fine_deg if m <= 2 else self.fine_mm
                        if event.key == pygame.K_d:
                            for m in self.selected: self.target_steps[m] += self.fine_deg if m <= 2 else self.fine_mm
                        if event.key == pygame.K_t: in_mode = "ABS"
                        if event.key == pygame.K_r: in_mode = "REL"
                        if event.key == pygame.K_z:
                            for m in self.selected:
                                self.ser.write(f"RESET_POS {m} 0\n".encode())
                                test_targets_in_steps[m] = 0
                    else:
                        if event.key == pygame.K_RETURN:
                            try:
                                v = float(in_txt)
                                for m in self.selected:
                                    s = int(v/DEG_PER_STEP) if m <= 2 else int(v/LEAD_PITCH*200)
                                    if in_mode == "ABS": self.target_steps[m] = s
                                    else: self.target_steps[m] += s
                            except: pass
                            in_mode, in_txt = None, ""
                        elif event.key == pygame.K_BACKSPACE: in_txt = in_txt[:-1]
                        elif event.key == pygame.K_ESCAPE: in_mode, in_txt = None, ""
                        elif event.unicode in "0123456789.-": in_txt += event.unicode
            self.clock.tick(30)

    def sel(self, m):
        if m in self.selected: self.selected.remove(m)
        else: self.selected.append(m)

    def main_pos_control_with_pid_test(self):
        print("Starting Position Control Test...")
        self.reset_motors()
        print("Motors Reset to 0.")

        while True: 
            print("Moving to positions: 100, 100, 100, 100")
            self.run_PID_vel_control_in_steps([100, 100, 100, 100])
            #time.sleep(2)
            self.update_feedback()

            self.run_PID_vel_control_in_steps([0, 0, 0, 0])
            print("Moving to positions: 0, 0, 0, 0")
            time.sleep(5)
            self.update_feedback()

            self.run_PID_vel_control_in_steps([200, 200, 200, 200])
            print("Moving to positions: 200, 200, 200, 200")
            time.sleep(2)
            self.update_feedback()

            self.reset_motors()
            print("Motors Reset to 0.")
            self.update_feedback()

            

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
    #app.run()
    #app.main_pos_control()
