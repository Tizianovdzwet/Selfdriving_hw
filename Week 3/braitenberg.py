"""
Braitenberg Vehicle Simulation (pygame)

Controls
- Left-drag light (grab the light): move light source
- Left-drag vehicle (grab the robot): reposition vehicle manually
- Click sensor then motor (in the right panel): toggle connection on/off
- Click a gain textbox: type a number (supports -, ., backspace), Enter or click away to commit
- R: reset vehicle position/heading
- ESC / close window: quit
"""

import math
import pygame
from dataclasses import dataclass

# ---------------------------- Config ----------------------------

WIDTH, HEIGHT = 1100, 700
PANEL_W = 360
ARENA_W = WIDTH - PANEL_W
FPS = 60

BG = (18, 18, 22)
ARENA_BG = (12, 12, 16)
PANEL_BG = (22, 22, 28)
WHITE = (235, 235, 240)
GRAY = (140, 140, 150)
DARK = (40, 40, 50)
ACCENT = (90, 180, 255)
GOOD = (110, 220, 160)
BAD = (255, 120, 120)
YELLOW = (255, 215, 80)

# Vehicle physics
WHEELBASE = 44.0
MAX_WHEEL_SPEED = 220.0
BASE_SPEED = 40.0

# Sensors
SENSOR_FORWARD_OFFSET = 22.0
SENSOR_LATERAL_OFFSET = 16.0
SENSOR_FOV = math.radians(75)
SENSOR_MAX_RANGE = 420.0

# Light model (intensity)
LIGHT_INTENSITY = 90000.0
LIGHT_MIN_DIST = 18.0

# UI
FONT_NAME = None
FONT_SIZE = 18
SMALL_FONT_SIZE = 15

# Panel layout (tuned so: text at top, nodes below, gains further below)
NODES_TOP_Y = 220          # moved down so instruction text is fully clear
GAINS_TOP_Y = 450          # moved down so it sits below nodes + connector area

# ---------------------------- Helpers ----------------------------

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def vec_from_angle(theta):
    return pygame.Vector2(math.cos(theta), math.sin(theta))

# ---------------------------- UI Widgets ----------------------------

class TextBox:
    def __init__(self, rect, text="0.0", label=""):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.label = label
        self.active = False
        self._committed_value = self.parse_value()

    def parse_value(self):
        try:
            if self.text.strip() in ["", "-", "+", ".", "-.", "+."]:
                return 0.0
            return float(self.text)
        except ValueError:
            return self._committed_value

    @property
    def value(self):
        return self._committed_value

    def commit(self):
        self._committed_value = self.parse_value()

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.active = True
            else:
                if self.active:
                    self.commit()
                self.active = False

        if event.type == pygame.KEYDOWN and self.active:
            if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.commit()
                self.active = False
                return
            if event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
                return

            ch = event.unicode
            if ch and (ch.isdigit() or ch in "+-."):
                if ch == "." and "." in self.text:
                    return
                if ch in "+-" and len(self.text) > 0:
                    return
                self.text += ch

    def draw(self, surf, font, small_font):
        if self.label:
            label_surf = small_font.render(self.label, True, GRAY)
            surf.blit(label_surf, (self.rect.x, self.rect.y - 18))

        border = ACCENT if self.active else GRAY
        pygame.draw.rect(surf, DARK, self.rect, border_radius=6)
        pygame.draw.rect(surf, border, self.rect, width=2, border_radius=6)

        txt = self.text if (self.active or self.text != "") else "0.0"
        txt_surf = font.render(txt, True, WHITE)
        surf.blit(txt_surf, (self.rect.x + 10, self.rect.y + (self.rect.height - txt_surf.get_height()) // 2))

class Node:
    def __init__(self, name, center, kind):
        self.name = name
        self.center = pygame.Vector2(center)
        self.kind = kind  # "sensor" or "motor"
        self.r = 16

    def hit(self, pos):
        return pygame.Vector2(pos).distance_to(self.center) <= self.r

    def draw(self, surf, font, selected=False):
        col = GOOD if self.kind == "sensor" else YELLOW
        outline = ACCENT if selected else GRAY

        # draw circle
        pygame.draw.circle(surf, col, self.center, self.r)
        pygame.draw.circle(surf, outline, self.center, self.r, width=2)

        label = font.render(self.name, True, WHITE)

        if self.kind == "sensor":
            # draw text to the LEFT of the sensor
            surf.blit(
                label,
                (
                    self.center.x - self.r - 10 - label.get_width(),
                    self.center.y - label.get_height() // 2,
                ),
            )
        else:
            # motors: keep text to the RIGHT
            surf.blit(
                label,
                (
                    self.center.x + self.r + 10,
                    self.center.y - label.get_height() // 2,
                ),
            )

# ---------------------------- Simulation Entities ----------------------------

@dataclass
class Vehicle:
    pos: pygame.Vector2
    heading: float
    radius: float = 18.0

    def sensor_world_positions(self):
        fwd = vec_from_angle(self.heading)
        left = vec_from_angle(self.heading + math.pi / 2)
        front_center = self.pos + fwd * SENSOR_FORWARD_OFFSET
        sL = front_center - left * SENSOR_LATERAL_OFFSET
        sR = front_center + left * SENSOR_LATERAL_OFFSET
        return sL, sR

    def update(self, vL, vR, dt):
        vL = clamp(vL, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        vR = clamp(vR, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

        v = 0.5 * (vR + vL)
        w = (vR - vL) / WHEELBASE

        self.heading = wrap_angle(self.heading + w * dt)
        self.pos += vec_from_angle(self.heading) * v * dt

# ---------------------------- Sensor Model ----------------------------

def sensor_reading(sensor_pos, sensor_dir, light_pos):
    v = pygame.Vector2(light_pos) - pygame.Vector2(sensor_pos)
    dist = v.length()
    if dist > SENSOR_MAX_RANGE:
        return 0.0

    dist = max(dist, LIGHT_MIN_DIST)
    v_norm = v / dist

    ang = math.acos(clamp(sensor_dir.dot(v_norm), -1.0, 1.0))
    if ang > SENSOR_FOV:
        return 0.0

    ang_gain = math.cos(ang) ** 2
    intensity = LIGHT_INTENSITY / (dist * dist)
    return intensity * ang_gain

# ---------------------------- Main ----------------------------

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Braitenberg Vehicle Simulator (pygame)")
    clock = pygame.time.Clock()

    font = pygame.font.Font(FONT_NAME, FONT_SIZE)
    small_font = pygame.font.Font(FONT_NAME, SMALL_FONT_SIZE)

    vehicle = Vehicle(pos=pygame.Vector2(ARENA_W * 0.35, HEIGHT * 0.55), heading=-0.3)
    light_pos = pygame.Vector2(ARENA_W * 0.72, HEIGHT * 0.45)

    dragging_light = False
    dragging_vehicle = False

    panel_x = ARENA_W

    # Nodes moved down so connectors won't overlap instruction text.
    nodes = {
        "S_L": Node("S_L", (panel_x + 70,  NODES_TOP_Y + 40),  "sensor"),
        "S_R": Node("S_R", (panel_x + 70,  NODES_TOP_Y + 110), "sensor"),
        "M_L": Node("M_L", (panel_x + 220, NODES_TOP_Y + 40),  "motor"),
        "M_R": Node("M_R", (panel_x + 220, NODES_TOP_Y + 110), "motor"),
    }

    pairs = [("M_L", "S_L"), ("M_L", "S_R"), ("M_R", "S_L"), ("M_R", "S_R")]
    enabled = {p: True for p in pairs}

    # Gains textboxes moved lower.
    tb = {}
    grid_origin = (panel_x + 30, GAINS_TOP_Y)
    col_w = 150
    row_h = 60

    labels = {
        ("M_L", "S_L"): "Gain M_L <- S_L",
        ("M_L", "S_R"): "Gain M_L <- S_R",
        ("M_R", "S_L"): "Gain M_R <- S_L",
        ("M_R", "S_R"): "Gain M_R <- S_R",
    }
    defaults = {
        ("M_L", "S_L"): "120.0",
        ("M_L", "S_R"): "-160.0",
        ("M_R", "S_L"): "-160.0",
        ("M_R", "S_R"): "120.0",
    }

    for i, p in enumerate(pairs):
        r = i // 2
        c = i % 2
        x = grid_origin[0] + c * col_w
        y = grid_origin[1] + r * row_h
        tb[p] = TextBox((x, y, 120, 32), text=defaults[p], label=labels[p])
        tb[p].commit()

    selected_sensor = None
    info_lines = [
        "Connect tool:",
        "1) click a SENSOR,",
        "2) then click a MOTOR",
        "to toggle that connection.",
        "",
        "Arena:",
        "Drag LIGHT or ROBOT to move.",
        "R = reset vehicle",
    ]

    def reset_vehicle():
        vehicle.pos = pygame.Vector2(ARENA_W * 0.35, HEIGHT * 0.55)
        vehicle.heading = -0.3

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        # ---------------- Events ----------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_r:
                    reset_vehicle()

            # textboxes first
            for p in pairs:
                tb[p].handle_event(event)

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos

                # Arena dragging: prefer grabbing robot if both are close
                if mx < ARENA_W:
                    m = pygame.Vector2(event.pos)
                    if m.distance_to(vehicle.pos) <= vehicle.radius + 6:
                        dragging_vehicle = True
                    elif m.distance_to(light_pos) <= 22:
                        dragging_light = True

                # Panel connect tool
                clicked = None
                for name, node in nodes.items():
                    if node.hit(event.pos):
                        clicked = name
                        break

                if clicked:
                    if nodes[clicked].kind == "sensor":
                        selected_sensor = clicked
                    elif nodes[clicked].kind == "motor" and selected_sensor is not None:
                        p = (clicked, selected_sensor)
                        if p in enabled:
                            enabled[p] = not enabled[p]
                        selected_sensor = None

            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging_light = False
                dragging_vehicle = False

            if event.type == pygame.MOUSEMOTION:
                mx, my = event.pos
                if dragging_light and mx < ARENA_W:
                    light_pos.update(mx, my)
                if dragging_vehicle and mx < ARENA_W:
                    vehicle.pos.update(mx, my)

        # ---------------- Simulation ----------------
        fwd = vec_from_angle(vehicle.heading)
        sL_pos, sR_pos = vehicle.sensor_world_positions()

        sL = sensor_reading(sL_pos, fwd, light_pos)
        sR = sensor_reading(sR_pos, fwd, light_pos)

        vL = BASE_SPEED
        vR = BASE_SPEED

        for (motor, sensor) in pairs:
            if not enabled[(motor, sensor)]:
                continue
            g = tb[(motor, sensor)].value
            s = sL if sensor == "S_L" else sR
            if motor == "M_L":
                vL += g * s
            else:
                vR += g * s

        if not dragging_vehicle:
            vehicle.update(vL, vR, dt)

        vehicle.pos.x = clamp(vehicle.pos.x, vehicle.radius, ARENA_W - vehicle.radius)
        vehicle.pos.y = clamp(vehicle.pos.y, vehicle.radius, HEIGHT - vehicle.radius)

        # ---------------- Draw ----------------
        screen.fill(BG)

        # Arena
        pygame.draw.rect(screen, ARENA_BG, pygame.Rect(0, 0, ARENA_W, HEIGHT))

        # Light
        pygame.draw.circle(screen, YELLOW, light_pos, 12)
        pygame.draw.circle(screen, WHITE, light_pos, 12, 2)
        pygame.draw.circle(screen, WHITE, light_pos, 28, 1)

        # Vehicle
        pygame.draw.circle(screen, (170, 170, 190), vehicle.pos, vehicle.radius)
        pygame.draw.circle(screen, WHITE, vehicle.pos, vehicle.radius, 2)
        head_tip = vehicle.pos + vec_from_angle(vehicle.heading) * (vehicle.radius + 12)
        pygame.draw.line(screen, ACCENT, vehicle.pos, head_tip, 3)

        # Sensors in arena
        pygame.draw.circle(screen, GOOD, sL_pos, 6)
        pygame.draw.circle(screen, GOOD, sR_pos, 6)
        pygame.draw.line(screen, (80, 140, 120), sL_pos, sL_pos + fwd * 60, 2)
        pygame.draw.line(screen, (80, 140, 120), sR_pos, sR_pos + fwd * 60, 2)

        sr = small_font.render(f"S_L={sL:.3f}  S_R={sR:.3f}", True, WHITE)
        screen.blit(sr, (12, 12))
        mr = small_font.render(f"vL={vL:.1f}  vR={vR:.1f}", True, WHITE)
        screen.blit(mr, (12, 34))

        # Panel
        pygame.draw.rect(screen, PANEL_BG, pygame.Rect(ARENA_W, 0, PANEL_W, HEIGHT))
        pygame.draw.line(screen, (60, 60, 70), (ARENA_W, 0), (ARENA_W, HEIGHT), 2)

        title = font.render("Braitenberg Connect Panel", True, WHITE)
        screen.blit(title, (panel_x + 20, 20))

        y = 60
        for line in info_lines:
            surf = small_font.render(line, True, GRAY)
            screen.blit(surf, (panel_x + 20, y))
            y += 18

        # Nodes
        nodes["S_L"].draw(screen, small_font, selected=(selected_sensor == "S_L"))
        nodes["S_R"].draw(screen, small_font, selected=(selected_sensor == "S_R"))
        nodes["M_L"].draw(screen, small_font, selected=False)
        nodes["M_R"].draw(screen, small_font, selected=False)

        # Connector lines: now directly between nodes (they're low enough to not cover text)
        for (motor, sensor) in pairs:
            a = nodes[sensor].center
            b = nodes[motor].center
            on = enabled[(motor, sensor)]
            col = ACCENT if on else (90, 90, 100)
            w = 3 if on else 1
            pygame.draw.line(screen, col, a, b, w)
            mid = (a + b) * 0.5
            pygame.draw.circle(screen, col, mid, 5)

        # Gains title moved down accordingly
        gains_title = font.render("Gains (per connection)", True, WHITE)
        screen.blit(gains_title, (panel_x + 20, GAINS_TOP_Y - 40))

        for p in pairs:
            tb[p].draw(screen, font, small_font)
            mark_x = tb[p].rect.right + 10
            mark_y = tb[p].rect.y + 6
            on = enabled[p]
            pygame.draw.rect(screen, GOOD if on else BAD, (mark_x, mark_y, 18, 18), border_radius=4)
            mtxt = small_font.render("ON" if on else "OFF", True, WHITE)
            screen.blit(mtxt, (mark_x + 26, tb[p].rect.y + 7))

        tip = small_font.render("Tip: Cross-coupling (S_L->M_R, S_R->M_L) often yields attraction.", True, GRAY)
        screen.blit(tip, (panel_x + 20, HEIGHT - 28))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
