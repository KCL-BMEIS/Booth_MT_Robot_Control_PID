import serial
import time
import pygame
import sys
import random
from simple_pid import PID

# from T249 and stepmotor
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
        self.ser.reset_input_buffer() 
        self.ser.write(b"GET_ALL_POS\n")
        line = self.ser.readline().decode().strip()
        if line.startswith("ALL_POS"):
            p = line.split()
            if len(p) == 5:
                for i in range(1, 5): self.current_steps[i] = int(p[i])
    
    def run_PID_vel_control_in_steps(self, target_steps):
        for m in range(1, 5):
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
            unit = "MM" if i <= 2 else "DEG"
            lbl = self.f_main.render(f"AXIS-0{i} [{unit}]", True, G_YELLOW if i in self.selected else G_WHITE)
            self.screen.blit(lbl, (x + 20, y + 15))

            # Data
            cur = (self.current_steps[i]*DEG_PER_STEP) if i <= 2 else (self.current_steps[i]/200*8)
            tar = (self.target_steps[i]*DEG_PER_STEP) if i <= 2 else (self.target_steps[i]/200*8)
            if i <= 2:
               
                cur = (self.current_steps[i] / STEPS_PER_REV) * LEAD_PITCH
                tar = (self.target_steps[i] / STEPS_PER_REV) * LEAD_PITCH
                u_str = "mm"
            else:
             
                cur = self.current_steps[i] * DEG_PER_STEP
                tar = self.target_steps[i] * DEG_PER_STEP
                u_str = "deg"
            cur_t = self.f_data.render(f"POS: {cur:>7.2f} {u_str}", True, G_WHITE)
            tar_t = self.f_small.render(f"TARGET: {tar:.2f} {u_str}", True, G_GRAY if i not in self.selected else G_WHITE)
            self.screen.blit(cur_t, (x + 20, y + 55))
            self.screen.blit(tar_t, (x + 20, y + 100))
            
        cons_rect = pygame.Rect(50, 450, 950, 150)
        pygame.draw.rect(self.screen, (20, 25, 40), cons_rect)
        pygame.draw.rect(self.screen, G_BLUE, cons_rect, 2)
        mode_t = self.f_main.render(f"SYSTEM MODE: {mode if mode else 'STANDBY'}", True, G_RED if mode else G_GRAY)
        val_t = self.f_data.render(f"INPUT > {text}_", True, G_WHITE)
        self.screen.blit(mode_t, (70, 470))
        self.screen.blit(val_t, (70, 510))
        help_t = self.f_small.render("L/I/O/P:SELECT | A/D:JOG | T:ABS | R:REL | Z:ZERO | Q:EXIT", True, G_GRAY)
        self.screen.blit(help_t, (70, 570))
        pygame.display.flip()
    
    def run(self):

        in_mode, in_txt = None, ""
        self.target_steps = {1: 100, 2: 200, 3: 50, 4: 75}
        for _ in range(5): self.update_feedback(); time.sleep(0.1)

        while True:
            self.update_feedback()
            keys = pygame.key.get_pressed()
            if not in_mode:
                if keys[pygame.K_a]:
                    for m in self.selected: self.target_steps[m] -= self.jog_speed
                if keys[pygame.K_d]:
                    for m in self.selected: self.target_steps[m] += self.jog_speed

            self.run_PID_vel_control_in_steps(self.target_steps)

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
                            for m in self.selected: self.target_steps[m] -= self.fine_mm if m <= 2 else self.fine_deg
                        if event.key == pygame.K_d:
                            for m in self.selected: self.target_steps[m] += self.fine_mm if m <= 2 else self.fine_deg
                        if event.key == pygame.K_t: in_mode = "ABS"
                        if event.key == pygame.K_r: in_mode = "REL"
                        if event.key == pygame.K_z:
                            for m in self.selected:
                                self.ser.write(f"RESET_POS {m} 0\n".encode())
                                self.target_steps[m] = 0
                    else:
                        if event.key == pygame.K_RETURN:
                            try:
                                v = float(in_txt)
                                for m in self.selected:
                                    s = self.mm_to_steps(v) if m <= 2 else self.deg_to_steps(v)
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

if __name__ == "__main__":
    app = MotorControlSystem()
    app.run()