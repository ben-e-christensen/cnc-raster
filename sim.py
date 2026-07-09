import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# --- Physics Parameters ---
RPM = 15
# Convert RPM to the time required to complete one full loop of the square (4 seconds at 15 RPM).
circuit_time = 60.0 / RPM  # seconds

# Square "radius" from center to face (meters)
# Total side length L = 2.0 m
R = 1.0  
L = 2 * R

# Mass of the particle (kg)
mass = 1.0  

# Visual scale factors for the impulse vector
impulse_scale = 100.0

# --- Plot Setup ---
fig, ax = plt.subplots(figsize=(6, 6))
limit = R + 0.5
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_aspect('equal')
ax.set_title(f"Particle on a Spinning Square Disk ({RPM} RPM)\nImpulsive Force from Ridge (at Corners Only)")
ax.grid(True, linestyle='--', alpha=0.6)

# Draw the Square Disk / Ridge
# Define vertices for the square patch
# patches.Rectangle works, but Polygon is clearer for vertex definitions.
vertices = np.array([
    [ R, -R], # Bottom Right (Start)
    [ R,  R], # Top Right
    [-R,  R], # Top Left
    [-R, -R]  # Bottom Left
])

# Use a Polygon patch to draw the boundary
square_ridge = patches.Polygon(vertices, closed=True, color='black', fill=False, linewidth=3)
ax.add_patch(square_ridge)
# Fill the square grey area inside
square_disk = patches.Polygon(vertices, closed=True, color='lightgray', fill=True, alpha=0.5)
ax.add_patch(square_disk)

# Initialize the particle and the force vector (quiver)
particle, = ax.plot([], [], 'ro', markersize=10, label='Particle')
# Quiver will be activated only at corners.
force_arrow = ax.quiver(0, 0, 0, 0, color='blue', scale=1000, width=0.01, label='Impulsive Force (Ridge)')

ax.legend(loc="upper right")

# --- Function to calculate piecewise particle position (Path) ---
def get_square_path(t):
    """Calculates position (x,y) piecewise for a square perimeter path."""
    t_mod = t % circuit_time
    
    # total perimeter L=8.0, circuit_time=4.0, so speed v = 2.0 m/s
    v = 8.0 / circuit_time 
    
    # Segment 1 (Right Face): Up from (1, -1) to (1, 1)
    if t_mod <= 1.0:
        return R, -R + v * t_mod
    # Segment 2 (Top Face): Left from (1, 1) to (-1, 1)
    elif t_mod <= 2.0:
        t_seg = t_mod - 1.0
        return R - v * t_seg, R
    # Segment 3 (Left Face): Down from (-1, 1) to (-1, -1)
    elif t_mod <= 3.0:
        t_seg = t_mod - 2.0
        return -R, R - v * t_seg
    # Segment 4 (Bottom Face): Right from (-1, -1) back to (1, -1)
    else:
        t_seg = t_mod - 3.0
        return -R + v * t_seg, -R

# --- Animation Functions ---
def init():
    """Initialize the background of the animation."""
    particle.set_data([], [])
    force_arrow.set_UVC(0, 0)
    return particle, force_arrow

def animate(frame):
    """Update the particle position and force vector for each frame."""
    # Time based on the frame number (assuming 60 fps)
    t = frame / 60.0 
    t_mod = t % circuit_time
    
    # Get current position along the square boundary
    x, y = get_square_path(t)
    
    # Update particle position
    particle.set_data([x], [y])
    
    # --- Calculate Impulsive Force at Corners ---
    fx, fy = 0.0, 0.0
    corner_buffer = 0.03 # seconds around corner time to make impulse visible
    
    # Corners are hit at exactly t_mod = 1.0, 2.0, 3.0, and 4.0(0.0)
    is_at_corner = False
    
    # Corner 1 (Top Right): from right -> top (Impulse diagonally in-left/in-down)
    if abs(t_mod - 1.0) < corner_buffer:
        fx, fy = -1.0, -1.0 # Diagonally Inward
        is_at_corner = True
    # Corner 2 (Top Left): from top -> left (Impulse diagonally in-right/in-down)
    elif abs(t_mod - 2.0) < corner_buffer:
        fx, fy = 1.0, -1.0 
        is_at_corner = True
    # Corner 3 (Bottom Left): from left -> bottom (Impulse diagonally in-right/in-up)
    elif abs(t_mod - 3.0) < corner_buffer:
        fx, fy = 1.0, 1.0 
        is_at_corner = True
    # Corner 4 (Bottom Right): from bottom -> right (Impulse diagonally in-left/in-up)
    elif abs(t_mod - 0.0) < corner_buffer or abs(t_mod - circuit_time) < corner_buffer:
        fx, fy = -1.0, 1.0 
        is_at_corner = True
        
    # Activate/Deactivate the arrow based on corner status
    if is_at_corner:
        force_arrow.set_offsets(np.column_stack([x, y]))
        # Scale magnitude for visualization
        force_arrow.set_UVC(fx * impulse_scale, fy * impulse_scale)
    else:
        # Make force vector zero (invisible) when on a straight section
        force_arrow.set_UVC(0, 0)
    
    return particle, force_arrow

# --- Run Animation ---
# 60 frames per second, run for 800 frames (2 full laps)
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=800, interval=1000/60, blit=False)

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.show()