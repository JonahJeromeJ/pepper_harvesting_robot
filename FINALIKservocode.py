
import math
import time
import numpy as np
import lgpio

# ==================================================
# GPIO PINS (BCM)
# ==================================================
PIN_BASE = 17
PIN_SHOULDER = 27
PIN_ELBOW = 22
PIN_WRIST = 23

# ==================================================
# INIT GPIO
# ==================================================
chip = lgpio.gpiochip_open(0)
for p in [PIN_BASE, PIN_SHOULDER, PIN_ELBOW, PIN_WRIST]:
    lgpio.gpio_claim_output(chip, p)

# ==================================================
# SERVO HELPERS
# ==================================================
SERVO_MIN = 0
SERVO_MAX = 180
PULSE_MIN = 700
PULSE_MAX = 2000

def servo_set(pin, angle):
    angle = max(SERVO_MIN, min(SERVO_MAX, angle))
    pulse = PULSE_MIN + (PULSE_MAX - PULSE_MIN) * (angle / 180.0)
    lgpio.tx_servo(chip, pin, int(pulse))

# ==================================================
# BASE WARM-UP SWEEP
# ==================================================
def warmup_base():
    for _ in range(3):
        servo_set(PIN_BASE, 45)
        time.sleep(0.5)
        servo_set(PIN_BASE, 135)
        time.sleep(0.5)

# ==================================================
# MOVE ALL SERVOS TO 90° SEQUENTIALLY
# ==================================================
def move_all_to_90():
    print("Base warm-up sweep")
    warmup_base()

    print("Base -> 90")
    servo_set(PIN_BASE, 90)
    time.sleep(2)

    print("Shoulder -> 90")
    servo_set(PIN_SHOULDER, 90)
    time.sleep(2)

    print("Elbow -> 90")
    servo_set(PIN_ELBOW, 90)
    time.sleep(2)

    print("Wrist -> 90")
    servo_set(PIN_WRIST, 90)
    time.sleep(2)

# ==================================================
# MOVE TO IK ANGLES SEQUENTIALLY
# ==================================================
def move_to_ik_angles(base, shoulder, elbow, wrist):
    print("Base warm-up sweep before IK move")
    warmup_base()

    print(f"Base -> {base}")
    servo_set(PIN_BASE, base)
    time.sleep(2)

    print(f"Shoulder -> {shoulder}")
    servo_set(PIN_SHOULDER, shoulder)
    time.sleep(2)

    print(f"Elbow -> {elbow}")
    servo_set(PIN_ELBOW, elbow)
    time.sleep(2)

    print(f"Wrist -> {wrist}")
    servo_set(PIN_WRIST, wrist)
    time.sleep(2)

# ==================================================
# LINK LENGTHS (mm)
# ==================================================
L1 = 68.93
L2 = 120.0
L3 = 99.32
L4 = 121.25

# ==================================================
# IK SETTINGS
# ==================================================
STEP = 3
BASE_RANGE = range(0, 181, STEP)
SHOULDER_RANGE = range(60, 161, STEP)
ELBOW_RANGE = range(45, 181, STEP)
WRIST_RANGE = range(80, 181, STEP)

# ==================================================
# FORWARD KINEMATICS
# ==================================================
def forward_kinematics(base, shoulder, elbow, wrist):
    b = math.radians(base - 90)
    s = math.radians(shoulder)
    e = math.radians(180 - elbow)
    w = math.radians(180 - wrist)

    r = (
        L2 * math.cos(s) +
        L3 * math.cos(s + e) +
        L4 * math.cos(s + e + w)
    )

    z = (
        L1 +
        L2 * math.sin(s) +
        L3 * math.sin(s + e) +
        L4 * math.sin(s + e + w)
    )

    x = -r * math.cos(b)
    y = -r * math.sin(b)

    return x, y, z

# ==================================================
# GENERATE FK TABLE
# ==================================================
def generate_fk_table():
    print("Generating FK table (STEP = 3°)... Please wait")
    data = []

    for base in BASE_RANGE:
        for shoulder in SHOULDER_RANGE:
            for elbow in ELBOW_RANGE:
                for wrist in WRIST_RANGE:
                    x, y, z = forward_kinematics(
                        base, shoulder, elbow, wrist
                    )
                    data.append([x, y, z, base, shoulder, elbow, wrist])

    table = np.array(data, dtype=np.float32)
    print(f"FK table generated: {table.shape[0]} points\n")
    return table

# ==================================================
# NUMERICAL IK SEARCH
# ==================================================
def find_closest_solution(table, x_t, y_t, z_t):
    dx = table[:, 0] - x_t
    dy = table[:, 1] - y_t
    dz = table[:, 2] - z_t
    dist = np.sqrt(dx*dx + dy*dy + dz*dz)
    idx = np.argmin(dist)
    return table[idx], dist[idx]

# ==================================================
# MAIN
# ==================================================
if __name__ == "__main__":

    fk_table = generate_fk_table()

    print("Initializing arm to safe home")
    move_all_to_90()

    try:
        while True:
            print("\nEnter target position (mm)")
            x_t = float(input("x: "))
            y_t = float(input("y: "))
            z_t = float(input("z: "))

            best, error = find_closest_solution(
                fk_table, x_t, y_t, z_t
            )

            base     = int(best[3])
            shoulder = int(best[4])
            elbow    = int(best[5])
            wrist    = int(best[6])

            print("\n--- SERVO ANGLES ---")
            print(f"Base     : {base} deg")
            print(f"Shoulder : {shoulder} deg")
            print(f"Elbow    : {elbow} deg")
            print(f"Wrist    : {wrist} deg")
            print(f"Error    : {error:.2f} mm")

            move_to_ik_angles(base, shoulder, elbow, wrist)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        lgpio.gpiochip_close(chip)
        print("GPIO released")
