import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import threading
import queue

# =========================
# CONFIG
# =========================
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

MIN_ACTIVE_PIXELS = 3

BASELINE_ALPHA = 0.001     # ~1 minute adaptation
NOISE_ALPHA = 0.01         # noise learning speed
NOISE_MULTIPLIER = 4.0     # detection sensitivity

# =========================
# UDP DATA
# =========================
data_queue = queue.Queue()

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP listening on {UDP_IP}:{UDP_PORT}")

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            payload = json.loads(data.decode())

            if data_queue.qsize() > 5:
                data_queue.get_nowait()

            data_queue.put(payload)

        except Exception as e:
            print("UDP error:", e)

threading.Thread(target=udp_listener, daemon=True).start()

# =========================
# PLOT SETUP
# =========================
fig, (ax_grid, ax_info) = plt.subplots(
    1, 2, figsize=(12, 5),
    gridspec_kw={'width_ratios': [3, 1]}
)

img = ax_grid.imshow(
    np.zeros((8,8)),
    vmin=0,
    vmax=2500,
    cmap='viridis'
)

plt.colorbar(img, ax=ax_grid)
ax_grid.set_title("VL53L5CX Grid")

ax_info.axis('off')

info_text = ax_info.text(
    0.5, 0.5,
    "",
    ha='center',
    va='center',
    fontsize=16
)

# =========================
# TRACKING VARIABLES
# =========================
baseline = None
noise = None

trajectory = []
tracking_active = False

total_in = 0
total_out = 0

# =========================
# MAIN LOOP
# =========================
def update(frame):
    global baseline, noise
    global trajectory, tracking_active
    global total_in, total_out

    updated = False

    while not data_queue.empty():
        data = data_queue.get()
        updated = True

        if "zones" not in data:
            continue

        zones = np.array(data["zones"])
        if len(zones) != 64:
            continue

        grid = zones.reshape((8,8))
        img.set_data(grid)

        # -------------------------
        # INIT BASELINE + NOISE
        # -------------------------
        if baseline is None:
            baseline = grid.astype(float)
            noise = np.ones((8,8)) * 50
            print("Baseline captured")
            continue

        # -------------------------
        # ADAPTIVE THRESHOLD
        # -------------------------
        diff = baseline - grid
        abs_err = np.abs(diff)

        threshold = noise * NOISE_MULTIPLIER
        occupied = diff > threshold

        occupied[0:2, :] = False

        active_pixels = np.sum(occupied)

        # -------------------------
        # UPDATE BASELINE + NOISE
        # only where NOT occupied
        # -------------------------
        stable_mask = ~occupied

        baseline[stable_mask] = (
            (1 - BASELINE_ALPHA) * baseline[stable_mask] +
            BASELINE_ALPHA * grid[stable_mask]
        )

        noise[stable_mask] = (
            (1 - NOISE_ALPHA) * noise[stable_mask] +
            NOISE_ALPHA * abs_err[stable_mask]
        )

        # -------------------------
        # TRACK TRAJECTORY
        # -------------------------
        if active_pixels >= MIN_ACTIVE_PIXELS:

            ys, xs = np.where(occupied)
            centroid_x = np.mean(xs)

            if not tracking_active:
                trajectory = []
                tracking_active = True

            trajectory.append(centroid_x)

        else:
            if tracking_active and len(trajectory) > 5:

                start = trajectory[0]
                end = trajectory[-1]
                min_x = min(trajectory)
                max_x = max(trajectory)

                if start < 2 and max_x > 5 and end > 5:
                    total_in += 1
                    print("IN")

                elif start > 5 and min_x < 2 and end < 2:
                    total_out += 1
                    print("OUT")

            tracking_active = False
            trajectory = []

        # -------------------------
        # DISPLAY
        # -------------------------
        info_text.set_text(
            f"IN : {total_in}\n"
            f"OUT: {total_out}\n\n"
            f"Tracking: {tracking_active}"
        )

    if updated:
        return [img, info_text]

    return []

ani = FuncAnimation(fig, update, interval=33, blit=False)

plt.tight_layout()
plt.show()
