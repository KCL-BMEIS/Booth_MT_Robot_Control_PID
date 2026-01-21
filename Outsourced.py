Outsourced

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