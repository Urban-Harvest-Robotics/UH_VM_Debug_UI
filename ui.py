import pygame
import os
import glob
import threading

class AgitatorControlUI:
    def __init__(self, screen, ros_interface):
        self.screen = screen
        self.ros = ros_interface
        self.font = pygame.font.Font(None, 30)

        # Dropdown options for 12 tumblers
        self.tumblers = [f"Tumbler {i+1}" for i in range(12)]
        self.selected_tumbler = 0  # Default Tumbler 1
        self.dropdown_open = False
        self.toggle_on = False

        # UI positions
        self.dropdown_rect = pygame.Rect(50, 50, 200, 40)
        self.toggle_rect = pygame.Rect(300, 50, 80, 40)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = event.pos

            # Handle dropdown click
            if self.dropdown_rect.collidepoint(mouse_pos):
                self.dropdown_open = not self.dropdown_open

            # Handle dropdown item selection
            elif self.dropdown_open:
                for i, _ in enumerate(self.tumblers):
                    item_rect = pygame.Rect(50, 90 + i * 35, 200, 30)
                    if item_rect.collidepoint(mouse_pos):
                        self.selected_tumbler = i
                        self.dropdown_open = False
                        break

            # Handle toggle button click
            if self.toggle_rect.collidepoint(mouse_pos):
                self.toggle_on = not self.toggle_on
                if self.toggle_on:
                    # 2^index = selected tumbler ON
                    value = 2 ** self.selected_tumbler
                else:
                    # 0 = all OFF
                    value = 0
                self.ros.publish_agitator_control(value)

    def draw(self):
        # Draw dropdown
        pygame.draw.rect(self.screen, (200, 200, 200), self.dropdown_rect)
        selected_text = self.font.render(self.tumblers[self.selected_tumbler], True, (0, 0, 0))
        self.screen.blit(selected_text, (self.dropdown_rect.x + 10, self.dropdown_rect.y + 8))

        # Draw dropdown items if open
        if self.dropdown_open:
            for i, tumbler in enumerate(self.tumblers):
                item_rect = pygame.Rect(50, 90 + i * 35, 200, 30)
                pygame.draw.rect(self.screen, (230, 230, 230), item_rect)
                text = self.font.render(tumbler, True, (0, 0, 0))
                self.screen.blit(text, (item_rect.x + 10, item_rect.y + 5))

        # Draw toggle
        color = (0, 200, 0) if self.toggle_on else (200, 0, 0)
        pygame.draw.rect(self.screen, color, self.toggle_rect)
        toggle_text = self.font.render("ON" if self.toggle_on else "OFF", True, (255, 255, 255))
        self.screen.blit(toggle_text, (self.toggle_rect.x + 20, self.toggle_rect.y + 8))

class DistanceDisplayUI:
    def __init__(self, screen, ros_interface, start_y=150):
        self.screen = screen
        self.ros = ros_interface
        self.font = pygame.font.Font(None, 26)
        self.start_y = start_y

    def draw(self):
        x_offset = 250
        y = self.start_y

        # Draw header
        header = self.font.render("Tumbler Distance (Ings)", True, (0, 0, 0))
        self.screen.blit(header, (x_offset, y))
        y += 30

        # Draw 12 tumbler distances
        for i, val in enumerate(self.ros.distance_data):
            text = self.font.render(f"Tumbler {i+1}: {val}", True, (50, 50, 50))
            self.screen.blit(text, (x_offset, y))
            y += 22

        # Leave space and draw solids
        y += 15
        header2 = self.font.render("Tumbler Distance (Solids)", True, (0, 0, 0))
        self.screen.blit(header2, (x_offset, y))
        y += 30

        for i, val in enumerate(self.ros.distance_solid_data):
            text = self.font.render(f"Solid {i+1}: {val}", True, (50, 50, 50))
            self.screen.blit(text, (x_offset, y))
            y += 22

class LinearGuideControlUI:
    def __init__(self, screen, ros_interface, start_y=600):
        self.screen = screen
        self.ros = ros_interface
        self.font = pygame.font.Font(None, 28)
        self.start_y = start_y

        # Swapped: X is now vertical, Y is now horizontal

        # X (vertical) range: 0 → 500
        self.x_min, self.x_max = 0, 500
        self.x_val = 0
        self.x_slider_rect = pygame.Rect(520, self.start_y - 250, 10, 250)  # longer vertical
        self.x_handle_rect = pygame.Rect(515, self.start_y - 5 - 125, 20, 10)  # start at middle

        # Y (horizontal) range: 0 → -1135
        self.y_min, self.y_max = 0, -1135
        self.y_val = 0
        self.y_slider_rect = pygame.Rect(80, self.start_y, 500, 10)  # longer horizontal
        self.y_handle_rect = pygame.Rect(80, self.start_y - 5, 10, 20)

        self.dragging_x = False
        self.dragging_y = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos
            if self.x_handle_rect.collidepoint(pos):
                self.dragging_x = True
            elif self.y_handle_rect.collidepoint(pos):
                self.dragging_y = True

        elif event.type == pygame.MOUSEBUTTONUP:
            if self.dragging_x or self.dragging_y:
                # Publish when released
                self.ros.publish_linear_position(self.x_val, self.y_val)
            self.dragging_x = False
            self.dragging_y = False

        elif event.type == pygame.MOUSEMOTION:
            pos = event.pos
            # Vertical drag (X)
            if self.dragging_x:
                new_y = min(max(pos[1], self.x_slider_rect.y), self.x_slider_rect.y + self.x_slider_rect.height)
                self.x_handle_rect.y = new_y - 5
                ratio = (new_y - self.x_slider_rect.y) / self.x_slider_rect.height
                self.x_val = int(self.x_min + ratio * (self.x_max - self.x_min))

            # Horizontal drag (Y)
            elif self.dragging_y:
                new_x = min(max(pos[0], self.y_slider_rect.x), self.y_slider_rect.x + self.y_slider_rect.width)
                self.y_handle_rect.x = new_x
                ratio = (new_x - self.y_slider_rect.x) / self.y_slider_rect.width
                self.y_val = int(self.y_min + ratio * (self.y_max - self.y_min))

    def draw(self):
        # Title
        label = self.font.render("Linear Guide Control (X/Y in mm)", True, (0, 0, 0))
        self.screen.blit(label, (80, self.start_y - 50))

        # Draw X (vertical) slider
        pygame.draw.rect(self.screen, (180, 180, 180), self.x_slider_rect)
        pygame.draw.rect(self.screen, (100, 100, 255), self.x_handle_rect)
        txt_x = self.font.render(f"X: {self.x_val} mm", True, (0, 0, 0))
        self.screen.blit(txt_x, (self.x_slider_rect.x - 50, self.x_slider_rect.y + self.x_slider_rect.height + 10))

        # Draw Y (horizontal) slider
        pygame.draw.rect(self.screen, (180, 180, 180), self.y_slider_rect)
        pygame.draw.rect(self.screen, (100, 255, 100), self.y_handle_rect)
        txt_y = self.font.render(f"Y: {self.y_val} mm", True, (0, 0, 0))
        self.screen.blit(txt_y, (self.y_slider_rect.x, self.y_slider_rect.y + 20))


class FruitSolidDispenserUI:
    def __init__(self, screen, ros_interface, x=50, y=950):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.width = 550
        self.height = 60

        self.font = pygame.font.Font(None, 28)

        self.dispensers = [f"{i+1}" for i in range(18)]  # 1–18
        self.selected_index = 0  # 0-based
        self.toggle_on = False

        self.dropdown_rect = pygame.Rect(self.x + 250, self.y, 120, 30)
        self.toggle_rect = pygame.Rect(self.x + 400, self.y, 60, 30)
        self.dropdown_open = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos

            # Toggle dropdown open/close
            if self.dropdown_rect.collidepoint(pos):
                self.dropdown_open = not self.dropdown_open

            # Click inside dropdown list
            elif self.dropdown_open:
                for i in range(len(self.dispensers)):
                    rect = pygame.Rect(
                        self.dropdown_rect.x,
                        self.dropdown_rect.y + 30 * (i + 1),
                        self.dropdown_rect.width,
                        30
                    )
                    if rect.collidepoint(pos):
                        self.selected_index = i
                        self.dropdown_open = False
                        break

            # Toggle button
            if self.toggle_rect.collidepoint(pos):
                self.toggle_on = not self.toggle_on
                speed = 1 if self.toggle_on else 0  # ✅ publish 1 for ON, 0 for OFF
                self.ros.publish_motor_command(self.selected_index + 1, speed)

    def draw(self, screen):
        # Label
        label = self.font.render("Fruit/Solid Dispenser", True, (0, 0, 0))
        screen.blit(label, (self.x, self.y + 5))

        # Dropdown box
        pygame.draw.rect(screen, (180, 180, 180), self.dropdown_rect)
        txt = self.font.render(self.dispensers[self.selected_index], True, (0, 0, 0))
        screen.blit(txt, (self.dropdown_rect.x + 10, self.dropdown_rect.y + 5))

        # Draw dropdown list if open
        if self.dropdown_open:
            for i, item in enumerate(self.dispensers):
                rect = pygame.Rect(
                    self.dropdown_rect.x,
                    self.dropdown_rect.y + 30 * (i + 1),
                    self.dropdown_rect.width,
                    30
                )
                pygame.draw.rect(screen, (220, 220, 220), rect)
                text = self.font.render(item, True, (0, 0, 0))
                screen.blit(text, (rect.x + 5, rect.y + 5))

        # Toggle button
        color = (0, 200, 0) if self.toggle_on else (200, 0, 0)
        pygame.draw.rect(screen, color, self.toggle_rect)
        toggle_text = self.font.render(
            "ON" if self.toggle_on else "OFF", True, (255, 255, 255)
        )
        screen.blit(toggle_text, (self.toggle_rect.x + 10, self.toggle_rect.y + 5))

class BlenderHeaterUI:
    def __init__(self, screen, ros_interface, x=50, y=1050):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 28)

        # Track individual toggle states
        self.blender_on = False
        self.heater_on = False

        # Define button positions
        self.blender_rect = pygame.Rect(self.x + 250, self.y, 80, 35)
        self.heater_rect = pygame.Rect(self.x + 400, self.y, 80, 35)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos

            # Blender toggle
            if self.blender_rect.collidepoint(pos):
                self.blender_on = not self.blender_on
                # Publish: [1,0] if blender ON else [0,0]
                data = [1, 0] if self.blender_on else [0, 0]
                self.ros.publish_ac_command(data)

            # Heater toggle
            if self.heater_rect.collidepoint(pos):
                self.heater_on = not self.heater_on
                # Publish: [0,1] if heater ON else [0,0]
                data = [0, 1] if self.heater_on else [0, 0]
                self.ros.publish_ac_command(data)

    def draw(self, screen):
        # Label
        label = self.font.render("Blender & Heater", True, (0, 0, 0))
        screen.blit(label, (self.x, self.y + 5))

        # Blender toggle
        color_b = (0, 200, 0) if self.blender_on else (200, 0, 0)
        pygame.draw.rect(screen, color_b, self.blender_rect)
        txt_b = self.font.render("Blender", True, (255, 255, 255))
        screen.blit(txt_b, (self.blender_rect.x + 5, self.blender_rect.y + 5))

        # Heater toggle
        color_h = (0, 200, 0) if self.heater_on else (200, 0, 0)
        pygame.draw.rect(screen, color_h, self.heater_rect)
        txt_h = self.font.render("Heater", True, (255, 255, 255))
        screen.blit(txt_h, (self.heater_rect.x + 10, self.heater_rect.y + 5))

class DoorControlUI:
    def __init__(self, screen, ros_interface, x=50, y=1150):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 28)

        # Toggle states
        self.inside_closed = False
        self.outside_closed = False

        # Button areas
        self.inside_rect = pygame.Rect(self.x + 250, self.y, 100, 35)
        self.outside_rect = pygame.Rect(self.x + 400, self.y, 120, 35)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos

            # Inside door toggle
            if self.inside_rect.collidepoint(pos):
                self.inside_closed = not self.inside_closed
                inside_val = 4200 if self.inside_closed else 0
                outside_val = 4200 if self.outside_closed else 1000
                self.ros.publish_door_command([inside_val, outside_val])

            # Outside door toggle
            if self.outside_rect.collidepoint(pos):
                self.outside_closed = not self.outside_closed
                inside_val = 4200 if self.inside_closed else 0
                outside_val = 4200 if self.outside_closed else 1000
                self.ros.publish_door_command([inside_val, outside_val])

    def draw(self, screen):
        # Label
        label = self.font.render("Door Control", True, (0, 0, 0))
        screen.blit(label, (self.x, self.y + 5))

        # Inside door toggle
        color_in = (200, 0, 0) if self.inside_closed else (0, 200, 0)
        pygame.draw.rect(screen, color_in, self.inside_rect)
        txt_in = self.font.render(
            "Inside CLOSE" if self.inside_closed else "Inside OPEN", True, (255, 255, 255)
        )
        screen.blit(txt_in, (self.inside_rect.x + 5, self.inside_rect.y + 5))

        # Outside door toggle
        color_out = (200, 0, 0) if self.outside_closed else (0, 200, 0)
        pygame.draw.rect(screen, color_out, self.outside_rect)
        txt_out = self.font.render(
            "Outside CLOSE" if self.outside_closed else "Outside OPEN", True, (255, 255, 255)
        )
        screen.blit(txt_out, (self.outside_rect.x + 5, self.outside_rect.y + 5))

class LiquidSystemUI:
    def __init__(self, screen, ros_interface, x=50, y=1250):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 28)

        # Each element represents a pump [ingredient water, x1, x2, cleaning]
        self.pumps = [False, False, False, False]
        self.labels = ["Ingredient Water", "X1", "Heater", "Cleaning Input"]

        # Button rectangles for 4 pumps
        self.buttons = [
            pygame.Rect(self.x + 200 + (i * 150), self.y, 140, 35) for i in range(4)
        ]

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos

            for i, rect in enumerate(self.buttons):
                if rect.collidepoint(pos):
                    # Toggle pump state
                    self.pumps[i] = not self.pumps[i]

                    # Build data array: 1 where pump is ON
                    data = [1 if p else 0 for p in self.pumps]
                    self.ros.publish_ac_pump_command(data)

    def draw(self, screen):
        # Label
        label = self.font.render("Liquid System", True, (0, 0, 0))
        screen.blit(label, (self.x, self.y + 5))

        # Draw all pump toggle buttons
        for i, rect in enumerate(self.buttons):
            color = (0, 200, 0) if self.pumps[i] else (200, 0, 0)
            pygame.draw.rect(screen, color, rect)

            text = self.font.render(
                self.labels[i], True, (255, 255, 255)
            )
            screen.blit(text, (rect.x + 5, rect.y + 5))

class DCPumpUI:
    def __init__(self, screen, ros_interface, x=50, y=1550):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 28)

        # Pump order: [Drainage, 2nd Ingredient, 1st Ingredient, 3rd Ingredient]
        self.pumps = [False, False, False, False]
        self.labels = ["Drainage", "2nd Ingredient", "1st Ingredient", "3rd Ingredient"]

        # Button rectangles for 4 pumps
        self.buttons = [
            pygame.Rect(self.x + 200 + i * 150, self.y, 140, 35) for i in range(4)
        ]

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos
            for i, rect in enumerate(self.buttons):
                if rect.collidepoint(pos):
                    # Toggle this pump ON/OFF independently
                    self.pumps[i] = not self.pumps[i]

                    # Build ROS2 data array and publish
                    data = [1 if p else 0 for p in self.pumps]
                    self.ros.publish_dc_pump(data)

    def draw(self, screen):
        # Label
        label = self.font.render("Liquid System (dc_pump)", True, (0, 0, 0))
        screen.blit(label, (self.x, self.y + 5))

        # Draw all pump toggle buttons
        for i, rect in enumerate(self.buttons):
            color = (0, 200, 0) if self.pumps[i] else (200, 0, 0)
            pygame.draw.rect(screen, color, rect)
            text = self.font.render(self.labels[i], True, (255, 255, 255))
            screen.blit(text, (rect.x + 5, rect.y + 5))

class ConveyorSliderUI:
    def __init__(self, screen, ros_interface, x=50, y=1700, width=400):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.width = width
        self.font = pygame.font.Font(None, 28)

        # Slider parameters
        self.min_val = 100
        self.max_val = 450
        self.value = 100  # Initial value
        self.slider_rect = pygame.Rect(self.x, self.y + 30, self.width, 6)
        self.handle_rect = pygame.Rect(self.x, self.y + 22, 12, 20)
        self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.handle_rect.collidepoint(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            if self.dragging:
                self.dragging = False
                # Publish the value when released
                self.ros.publish_conveyor_distance(self.value)
        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                # Move handle along the slider
                mouse_x = max(self.x, min(event.pos[0], self.x + self.width))
                self.handle_rect.x = mouse_x
                # Map handle position to value
                relative_pos = (mouse_x - self.x) / self.width
                self.value = int(self.min_val + relative_pos * (self.max_val - self.min_val))

    def draw(self):
        # Label
        label = self.font.render(f"Conveyor Distance: {self.value}", True, (0, 0, 0))
        self.screen.blit(label, (self.x, self.y))

        # Draw slider bar
        pygame.draw.rect(self.screen, (180, 180, 180), self.slider_rect)

        # Draw handle
        pygame.draw.rect(self.screen, (0, 180, 0), self.handle_rect)

class CupBowlUI:
    def __init__(self, screen, ros_interface, x=50, y=1850):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 28)

        # Toggle states for cup and bowl
        self.states = [False, False]  # [Cup, Bowl]
        self.labels = ["BOWL", "CUP"]

        # Button rectangles
        self.buttons = [
            pygame.Rect(self.x + 200 + i * 150, self.y, 120, 35) for i in range(2)
        ]

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos
            for i, rect in enumerate(self.buttons):
                if rect.collidepoint(pos):
                    # Toggle this element
                    self.states[i] = not self.states[i]

                    # Prevent both from being 4095 at the same time
                    if self.states[0] and self.states[1]:
                        # Turn the other one off
                        self.states[1 - i] = False

                    # Prepare ROS2 data
                    data = [4095 if self.states[0] else 0, 4095 if self.states[1] else 0]
                    self.ros.publish_target_position_array(data)

    def draw(self, screen):
        # Label
        label = self.font.render("Cup & Bowl Position", True, (0, 0, 0))
        screen.blit(label, (self.x, self.y + 5))

        # Draw buttons
        for i, rect in enumerate(self.buttons):
            color = (0, 180, 0) if self.states[i] else (200, 0, 0)
            pygame.draw.rect(screen, color, rect)
            text = self.font.render(self.labels[i], True, (255, 255, 255))
            screen.blit(text, (rect.x + 5, rect.y + 5))

class RecordPlayUI:
    def __init__(self, screen, ros_interface, x=50, y=1400):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 28)

        # Buttons
        self.record_button = pygame.Rect(self.x, self.y, 120, 35)
        self.save_button   = pygame.Rect(self.x + 130, self.y, 120, 35)
        self.play_button   = pygame.Rect(self.x + 260, self.y, 120, 35)

        # Dropdown
        self.dropdown_rect = pygame.Rect(self.x + 390, self.y, 250, 35)
        self.dropdown_open = False
        self.csv_files = self.get_csv_files()
        self.selected_file_index = 0
        self.recording = False

    def get_csv_files(self):
        files = [f for f in os.listdir() if f.startswith("ros_sequence_") and f.endswith(".csv")]
        return sorted(files)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos

            # Toggle recording
            if self.record_button.collidepoint(pos):
                if not self.recording:
                    self.ros.start_recording()
                    self.recording = True
                else:
                    self.ros.stop_recording()
                    self.recording = False
                    self.csv_files = self.get_csv_files()
                    self.selected_file_index = len(self.csv_files)-1 if self.csv_files else 0

            # Save last action
            if self.save_button.collidepoint(pos) and self.recording:
                self.ros.save_last_action()

            # Play selected
            if self.play_button.collidepoint(pos):
                if self.csv_files:
                    filename = self.csv_files[self.selected_file_index]
                    import threading
                    threading.Thread(target=self.ros.playback_sequence, args=(filename,), daemon=True).start()

            # Dropdown toggle / select
            if self.dropdown_rect.collidepoint(pos):
                self.dropdown_open = not self.dropdown_open
            if self.dropdown_open:
                for i, file in enumerate(self.csv_files):
                    file_rect = pygame.Rect(self.dropdown_rect.x,
                                            self.dropdown_rect.y + (i+1)*35,
                                            self.dropdown_rect.width, 35)
                    if file_rect.collidepoint(pos):
                        self.selected_file_index = i
                        self.dropdown_open = False

    def draw(self, screen):
        # Record button
        color = (200,0,0) if not self.recording else (0,200,0)
        pygame.draw.rect(screen, color, self.record_button)
        text = self.font.render("Record", True, (255,255,255))
        screen.blit(text, (self.record_button.x+5, self.record_button.y+5))

        # Save button
        pygame.draw.rect(screen, (255,165,0), self.save_button)
        text = self.font.render("Save Last", True, (255,255,255))
        screen.blit(text, (self.save_button.x+5, self.save_button.y+5))

        # Play button
        pygame.draw.rect(screen, (0,0,200), self.play_button)
        text = self.font.render("Play", True, (255,255,255))
        screen.blit(text, (self.play_button.x+5, self.play_button.y+5))

        # Dropdown
        pygame.draw.rect(screen, (150,150,150), self.dropdown_rect)
        selected_file = self.csv_files[self.selected_file_index] if self.csv_files else "No file"
        text = self.font.render(selected_file, True, (0,0,0))
        screen.blit(text, (self.dropdown_rect.x+5, self.dropdown_rect.y+5))

        if self.dropdown_open:
            for i, file in enumerate(self.csv_files):
                file_rect = pygame.Rect(self.dropdown_rect.x,
                                        self.dropdown_rect.y + (i+1)*35,
                                        self.dropdown_rect.width, 35)
                pygame.draw.rect(screen, (200,200,200), file_rect)
                text = self.font.render(file, True, (0,0,0))
                screen.blit(text, (file_rect.x+5, file_rect.y+5))

class BlenderPoseUI:
    def __init__(self, screen, ros_node, x=50, y=50):
        """
        screen   : pygame display surface
        ros_node : ROSInterface object
        x, y     : top-left position of the UI panel
        """
        self.screen = screen
        self.ros = ros_node
        self.x = x
        self.y = y

        # Slider values
        self.back_front = 1500  # default back
        self.up_down = 2000     # default up

        # Slider limits
        self.back_min = 1500
        self.back_max = 4000
        self.up_min = 2000
        self.up_max = 4000

        # Slider rects for UI
        self.linear_rect = pygame.Rect(self.x, self.y, 300, 20)       # horizontal track
        self.perp_rect = pygame.Rect(self.x + 350, self.y, 20, 200)   # vertical track

        # Slider knobs
        self.linear_knob = pygame.Rect(self.x, self.y - 5, 10, 30)
        self.perp_knob = pygame.Rect(self.x + 345, self.y + 200, 30, 10)

        # Drag flags
        self.drag_linear = False
        self.drag_perp = False

    def draw(self):
        # Draw sliders
        pygame.draw.rect(self.screen, (180, 180, 180), self.linear_rect)  # horizontal track
        pygame.draw.rect(self.screen, (0, 0, 255), self.linear_knob)      # horizontal knob

        pygame.draw.rect(self.screen, (180, 180, 180), self.perp_rect)    # vertical track
        pygame.draw.rect(self.screen, (0, 255, 0), self.perp_knob)        # vertical knob

        # Draw text labels
        font = pygame.font.SysFont(None, 20)
        text1 = font.render(f"Back-Front: {self.back_front}", True, (0, 0, 0))
        text2 = font.render(f"Up-Down: {self.up_down}", True, (0, 0, 0))
        self.screen.blit(text1, (self.x, self.y + 30))
        self.screen.blit(text2, (self.x + 350, self.y + 210))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.linear_knob.collidepoint(event.pos):
                self.drag_linear = True
            if self.perp_knob.collidepoint(event.pos):
                self.drag_perp = True

        elif event.type == pygame.MOUSEBUTTONUP:
            if self.drag_linear or self.drag_perp:
                # Publish current values to ROS on release
                self.ros.publish_blender_pose(self.back_front, self.up_down)
            self.drag_linear = False
            self.drag_perp = False

        elif event.type == pygame.MOUSEMOTION:
            if self.drag_linear:
                # Move horizontal knob
                new_x = max(self.linear_rect.left, min(event.pos[0], self.linear_rect.right))
                self.linear_knob.x = new_x
                # Map knob x -> back_front value
                self.back_front = int(self.back_min + (self.back_max - self.back_min) *
                                      (new_x - self.linear_rect.left) / self.linear_rect.width)

            if self.drag_perp:
                # Move vertical knob
                new_y = max(self.perp_rect.top, min(event.pos[1], self.perp_rect.bottom))
                self.perp_knob.y = new_y
                # Map knob y -> up_down value (invert for correct direction)
                self.up_down = int(self.up_max - (self.up_max - self.up_min) *
                                   (new_y - self.perp_rect.top) / self.perp_rect.height)


class LinearGuidePointsUI:
    def __init__(self, screen, ros_interface, x=50, y=1250):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 28)

        # Define all preset points
        self.points_first_row = [
            ("Cup", (365, 0)),
            ("Bowl", (236, -21)),
            ("T1", (155, -160)),
            ("T2", (155, -240)),
            ("T3", (155, -320)),
            ("T4", (155, -400)),
            ("T5", (155, -480)),
            ("T6", (155, -560)),
            ("T7", (155, -640)),
            ("T8", (155, -720)),
            ("T9", (155, -800)),
            ("T10", (155, -880)),
            ("T11", (155, -960)),
            ("T12", (155, -1040)),
        ]
        self.points_second_row = [
            ("RST", (0, 0)),
            ("Arm Pckp", (160,-1135)),
            ("S1", (35.5, -1135)),
            ("S2", (115.5, -1135)),
            ("S3", (195.5, -1135)),
            ("S4", (275.5, -1135)),
            ("S5", (355.5, -1135)),
            ("S6", (435.5, -1135)),
        ]

        # Button settings
        self.button_width = 120
        self.button_height = 40
        self.button_spacing = 10

        # Generate buttons
        self.buttons = []

        # First row
        start_x = self.x
        start_y = self.y
        for name, pos in self.points_first_row:
            rect = pygame.Rect(start_x, start_y, self.button_width, self.button_height)
            self.buttons.append((name, rect, pos))
            start_x += self.button_width + self.button_spacing

        # Second row
        start_x = self.x
        start_y += self.button_height + 10  # 10px vertical spacing
        for name, pos in self.points_second_row:
            rect = pygame.Rect(start_x, start_y, self.button_width, self.button_height)
            self.buttons.append((name, rect, pos))
            start_x += self.button_width + self.button_spacing

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos
            for name, rect, point in self.buttons:
                if rect.collidepoint(pos):
                    x_val, y_val = point
                    # Publish the selected linear guide position
                    self.ros.publish_linear_position(x_val, y_val)

    def draw(self):
        # Draw all buttons
        for name, rect, _ in self.buttons:
            pygame.draw.rect(self.screen, (180, 180, 180), rect)
            pygame.draw.rect(self.screen, (100, 200, 255), rect, 2)
            label = self.font.render(name, True, (0, 0, 0))
            # Center text inside button
            text_rect = label.get_rect(center=rect.center)
            self.screen.blit(label, text_rect)

class PlayCommandUI:
    def __init__(self, screen, ros_interface, x=1400, y=1250):
        self.screen = screen
        self.ros = ros_interface
        self.x = x
        self.y = y
        self.font = pygame.font.Font(None, 30)

        # Button dimensions
        self.button_rect = pygame.Rect(self.x, self.y, 350, 60)
        self.button_color = (0, 180, 0)
        self.hover_color = (0, 220, 0)
        self.text_color = (255, 255, 255)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.button_rect.collidepoint(event.pos):
                # Publish bowl pick & place command
                value = 1  # could be configurable later if needed
                self.ros.publish_play_command(value)
                print("▶️ Published /play_command with value:", value)

    def draw(self):
        mouse_pos = pygame.mouse.get_pos()
        color = self.hover_color if self.button_rect.collidepoint(mouse_pos) else self.button_color
        pygame.draw.rect(self.screen, color, self.button_rect, border_radius=10)

        text = self.font.render("Bowl Pick & Place → Next Chamber", True, self.text_color)
        text_rect = text.get_rect(center=self.button_rect.center)
        self.screen.blit(text, text_rect)
