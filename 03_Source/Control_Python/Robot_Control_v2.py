import pygame
import socket
import math

# ================= NETWORK CONFIGURATION =================
UDP_IP = "192.168.4.1" # ESP32 address
UDP_PORT = 1234 # ESP32 port
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False) 

# ================= WORKSPACE CONFIGURATION =================
R_MAX = 135.0      
Z_MIN = -335.0     
Z_MAX = -268.0     

# ================= INTERFACE CONFIGURATION =================
WIDTH, HEIGHT = 700, 700  
CENTER = (WIDTH // 2, HEIGHT // 2)
DRAW_RADIUS = int((min(WIDTH, HEIGHT) // 2) * 0.8)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Delta Robot Control Panel!")
clock = pygame.time.Clock()

try:
    font = pygame.font.SysFont("tahoma, segoeui", 20)
    font_large = pygame.font.SysFont("tahoma, segoeui", 24, bold=True)
except:
    font = pygame.font.Font(pygame.font.get_default_font(), 20) 
    font_large = pygame.font.Font(pygame.font.get_default_font(), 24)

# ================= INITIAL STATE =================
mode = 0           
z_step = 5.0       
robot_x, robot_y, robot_z = 0.0, 0.0, -300.0   

# Variables for typing coordinate input (Used for Mode 3)
is_typing = False
input_text = ""
error_msg = ""

def map_mouse_to_robot(mx, my):
    dx = mx - CENTER[0]
    dy = my - CENTER[1]
    rx = (dx / DRAW_RADIUS) * R_MAX
    ry = -(dy / DRAW_RADIUS) * R_MAX
    
    distance = math.sqrt(rx**2 + ry**2)
    if distance > R_MAX:
        rx = (rx / distance) * R_MAX
        ry = (ry / distance) * R_MAX
    return rx, ry

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
        elif event.type == pygame.KEYDOWN:
            if mode == 3 and is_typing:
                # Currently in Mode 3 text input mode
                if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                    # Parse the X Y Z string when Enter is pressed
                    try:
                        parts = input_text.strip().split()
                        if len(parts) >= 2:
                            new_x = float(parts[0])
                            new_y = float(parts[1])
                            new_z = float(parts[2]) if len(parts) == 3 else robot_z
                            
                            # Check safety limits
                            distance = math.sqrt(new_x**2 + new_y**2)
                            if distance <= R_MAX and Z_MIN <= new_z <= Z_MAX:
                                robot_x, robot_y, robot_z = new_x, new_y, new_z
                                error_msg = "Coordinates sent successfully!"
                                is_typing = False
                                input_text = "" # Clear after input is completed
                            else:
                                error_msg = f"Error: Out of range! (R<={R_MAX}, {Z_MIN}<=Z<={Z_MAX})"
                        else:
                            error_msg = "Error: Please enter at least X and Y"
                    except ValueError:
                        error_msg = "Error: Only numbers are allowed!"
                    
                elif event.key == pygame.K_BACKSPACE:
                    input_text = input_text[:-1]
                    error_msg = ""
                elif event.key == pygame.K_ESCAPE:
                    is_typing = False
                    input_text = ""
                    error_msg = "# Input cancelled."
                else:
                    # Only allow numbers, minus sign, dot, and spaces
                    if event.unicode in "0123456789- .":
                        input_text += event.unicode
                        error_msg = "" # Clear the error message when typing again
            else:
                # Normal mode (select Mode)
                if event.key == pygame.K_0 or event.key == pygame.K_KP0:
                    mode = 0
                    is_typing = False
                elif event.key == pygame.K_1 or event.key == pygame.K_KP1:
                    mode = 1
                    is_typing = False
                elif event.key == pygame.K_2 or event.key == pygame.K_KP2:
                    mode = 2
                    is_typing = False
                elif event.key == pygame.K_3 or event.key == pygame.K_KP3:
                    mode = 3
                    is_typing = False
                elif event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                    if mode == 3:
                        is_typing = True
                        input_text = ""
                        error_msg = ""
                        
        elif event.type == pygame.MOUSEWHEEL:
            # Keep the mouse wheel feature for adjusting Z in both Mode 2 and 3
            if mode in [2, 3]:
                robot_z += event.y * z_step
                if robot_z < Z_MIN: robot_z = Z_MIN
                if robot_z > Z_MAX: robot_z = Z_MAX

    # Get mouse coordinates if currently in Mode 2
    if mode == 2:
        mx, my = pygame.mouse.get_pos()
        robot_x, robot_y = map_mouse_to_robot(mx, my)

    # ================= SEND DATA TO ESP32 =================
    if mode in [0, 1]:
        msg_send = f"{mode}"
    else:
        msg_send = f"{mode},{robot_x:.7f},{robot_y:.7f},{robot_z:.7f}"
        
    try:
        sock.sendto(msg_send.encode('utf-8'), (UDP_IP, UDP_PORT))
    except BlockingIOError:
        pass 

    # ================= DRAW THE INTERFACE ON SCREEN =================
    screen.fill((30, 30, 30)) 
    
    # Draw the boundary circle of the working area
    pygame.draw.circle(screen, (100, 100, 100), CENTER, DRAW_RADIUS, 2)
    pygame.draw.line(screen, (80, 80, 80), (CENTER[0] - DRAW_RADIUS, CENTER[1]), (CENTER[0] + DRAW_RADIUS, CENTER[1]), 1)
    pygame.draw.line(screen, (80, 80, 80), (CENTER[0], CENTER[1] - DRAW_RADIUS), (CENTER[0], CENTER[1] + DRAW_RADIUS), 1)
    
    # Draw the current target point
    target_color = (0, 255, 255) if mode in [2, 3] else (150, 150, 150)
    draw_target_x = int(CENTER[0] + (robot_x / R_MAX) * DRAW_RADIUS)
    draw_target_y = int(CENTER[1] - (robot_y / R_MAX) * DRAW_RADIUS)
    
    pygame.draw.circle(screen, target_color, (draw_target_x, draw_target_y), 8)
    pygame.draw.circle(screen, (255, 255, 255), (draw_target_x, draw_target_y), 15, 1)

    # === DISPLAY TEXT INFORMATION ===
    if mode == 3:
        mode_text = "KEYBOARD (Keyboard Input) - "
        info_color = (255, 165, 0)
        target_display = f"X: {robot_x:.2f}  |  Y: {robot_y:.2f}  |  Z: {robot_z:.2f}"
    elif mode == 2:
        mode_text = "MANUAL (Mouse Control) - "
        info_color = (0, 255, 0)
        target_display = f"X: {robot_x:.2f}  |  Y: {robot_y:.2f}  |  Z: {robot_z:.2f}"
    elif mode == 1:
        mode_text = f"AUTO MODE - "
        info_color = (255, 100, 100)
        target_display = "Stop sending x,y,z" 
    else:
        mode_text = f"BACK HOME - "
        info_color = (255, 100, 100)
        target_display = "Stop sending x,y,z" 

    txt_mode = font.render(f"MODE: {mode_text} (Press 0,1,2,3 to switch)", True, info_color)
    txt_target = font.render(f"[CURRENT TARGET BEING SENT] {target_display}", True, (0, 255, 255))
    
    screen.blit(txt_mode, (20, 20))
    screen.blit(txt_target, (20, 50))
    
    # === INPUT BOX BELOW (Only appears in Mode 3) ===
    if mode == 3:
        if is_typing:
            # Draw a highlighted dialog box
            pygame.draw.rect(screen, (50, 50, 50), (20, HEIGHT - 100, WIDTH - 40, 40))
            pygame.draw.rect(screen, (0, 255, 0), (20, HEIGHT - 100, WIDTH - 40, 40), 2)
            
            # Blinking cursor
            cursor = "|" if pygame.time.get_ticks() % 1000 < 500 else ""
            txt_input = font_large.render(f"INPUT X Y Z: {input_text}{cursor}", True, (255, 255, 0))
            screen.blit(txt_input, (30, HEIGHT - 95))
            
            txt_guide = font.render("Press ENTER to confirm, ESC to cancel.", True, (200, 200, 200))
            screen.blit(txt_guide, (20, HEIGHT - 50))
        else:
            txt_guide = font.render("ENTER to input X Y Z ((scroll the mouse wheel to change Z))", True, (200, 200, 200))
            screen.blit(txt_guide, (20, HEIGHT - 50))
            
        if error_msg:
            color_err = (255, 0, 0) if "Error" in error_msg else (0, 255, 0)
            txt_err = font.render(error_msg, True, color_err)
            screen.blit(txt_err, (20, HEIGHT - 130))
            
    elif mode == 2:
        # Mouse wheel reminder for Mode 2
        txt_guide = font.render("Move mouse for X, Y. Scroll for Z.", True, (200, 200, 200))
        screen.blit(txt_guide, (20, HEIGHT - 50))

    pygame.display.flip()
    clock.tick(50)

pygame.quit()