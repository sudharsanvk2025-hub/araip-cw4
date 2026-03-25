from controller import Robot
import random

# ================= CONSTANTS =================
# We narrowed the tolerance to 20 to be more selective.
DEER_R, DEER_G, DEER_B = 77, 71, 69  
STRICT_TOLERANCE = 20                
MAX_SPEED = 6.28

# Initialize the Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 1. Setup Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 2. Setup Camera
cam = robot.getDevice('camera')
cam.enable(timestep)
width, height = cam.getWidth(), cam.getHeight()

# 3. Setup Distance Sensors
ps = [robot.getDevice(f'ps{i}') for i in range(8)]
for s in ps: 
    s.enable(timestep)

# --- TRACKING VARIABLES ---
base_speed = 4.0
steering_bias = 0.0
change_timer = 0
colors_found = [] 
image_counter = 0
deer_in_view = False  # Latch to prevent multiple photos of one deer

print("--- Robot Search Started (Strict Deer Detection) ---")

# ================= MAIN LOOP =================
while robot.step(timestep) != -1:
    
    # --- STEP 1: PERCEPTION (Averaged Center Sampling) ---
    img = cam.getImage()
    r_total, g_total, b_total = 0, 0, 0
    sample_count = 0
    
    for x in range(width // 2 - 1, width // 2 + 2):
        for y in range(height // 2 - 1, height // 2 + 2):
            r_total += cam.imageGetRed(img, width, x, y)
            g_total += cam.imageGetGreen(img, width, x, y)
            b_total += cam.imageGetBlue(img, width, x, y)
            sample_count += 1
            
    r, g, b = r_total/sample_count, g_total/sample_count, b_total/sample_count

    # --- STEP 2: LOGGING & DEER DETECTION ---
    margin = 45 
    current_color = None

    # Standard Color Detection (Red, Green, Blue)
    if r > (g + margin) and r > (b + margin):
        current_color = "Red"
    elif g > (r + margin) and g > (b + margin):
        current_color = "Green"
    elif b > (r + 30) and b > (g + 30):
        current_color = "Blue"

    # Strict Deer Detection Logic
    # 1. Must be within the RGB tolerance
    # 2. Must NOT be perfectly gray (Deer is slightly more Red/Green than Blue)
    is_near_deer_color = (abs(r - DEER_R) < STRICT_TOLERANCE and 
                          abs(g - DEER_G) < STRICT_TOLERANCE and 
                          abs(b - DEER_B) < STRICT_TOLERANCE)
    
    # This check ensures we aren't looking at a neutral gray wall (where R, G, and B are equal)
    is_not_gray = (r > b + 2) 

    if is_near_deer_color and is_not_gray:
        if not deer_in_view:
            filename = f"deer_found_{image_counter}.png"
            cam.saveImage(filename, 100)
            print(f"\n!!! DEER DETECTED !!! -> Saved as {filename}")
            
            if "Deer" not in colors_found:
                colors_found.append("Deer")
                print(f"Summary of findings: {', '.join(colors_found)}")
            
            image_counter += 1
            deer_in_view = True
    else:
        # We only reset the 'latch' when the robot is clearly NOT looking at a deer anymore
        if not is_near_deer_color:
            deer_in_view = False

    # Standard Color Logging (Only if not already logged)
    if current_color and current_color not in colors_found:
        colors_found.append(current_color)
        print(f"\nI see {current_color}!")
        print(f"Summary of findings: {', '.join(colors_found)}")

    # --- STEP 3: MOVEMENT (Avoidance & Wander) ---
    threshold = 180.0
    val_left = max(ps[7].getValue(), ps[6].getValue())
    val_right = max(ps[0].getValue(), ps[1].getValue())

    if val_left > threshold and val_right > threshold:
        left_speed, right_speed = -base_speed, -base_speed
    elif val_left > threshold:
        left_speed, right_speed = base_speed, -base_speed 
    elif val_right > threshold:
        left_speed, right_speed = -base_speed, base_speed
    else:
        change_timer += 1
        if change_timer > (1000 / timestep):
            steering_bias = random.uniform(-1.5, 1.5)
            change_timer = 0
        left_speed = base_speed + steering_bias
        right_speed = base_speed - steering_bias

    # Set motor speeds
    left_motor.setVelocity(max(min(left_speed, MAX_SPEED), -MAX_SPEED))
    right_motor.setVelocity(max(min(right_speed, MAX_SPEED), -MAX_SPEED))