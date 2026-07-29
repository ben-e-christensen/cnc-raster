import numpy as np
import matplotlib.pyplot as plt

def simulate_faraday_rotation():
    # Simulation Parameters
    z = np.linspace(0, 10, 1000)  # Propagation distance (Z-axis)
    wavelength = 1.0              # Wavelength of the radio wave
    k = 2 * np.pi / wavelength    # Wave number
    
    # Horn Antenna Beam Profile (Gaussian beam envelope for laser-like focus)
    # The beam stays relatively tight along the z-axis
    beam_envelope = np.exp(-0.02 * z) 
    
    # Magnetic Field / Plasma Effect (Faraday Rotation)
    # The rate of rotation depends on the magnetic field strength and plasma density
    rotation_rate = 0.5  # Radians per unit distance (simulating strong B-field)
    theta = rotation_rate * z # Rotation angle over distance
    
    # Generating the Electric Field components of the radio wave
    # E_0 * cos(kz) represents the oscillating wave
    E_amplitude = beam_envelope * np.cos(k * z)
    
    # Apply the rotation matrix to twist the wave
    E_x = E_amplitude * np.cos(theta)
    E_y = E_amplitude * np.sin(theta)

    # Plotting
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the 3D wave path
    ax.plot(z, E_x, E_y, color='b', label='Twisted Electric Field (with B-field in plasma)')
    
    # Plot the original wave without magnetic field (for comparison)
    E_x_orig = E_amplitude * np.cos(0)
    E_y_orig = E_amplitude * np.sin(0)
    ax.plot(z, E_x_orig, E_y_orig, color='r', alpha=0.3, linestyle='--', 
            label='Original Beam (Vacuum/No B-field)')

    # Formatting the plot to look like a directional beam
    ax.set_title('Simulation of Radio Wave Faraday Rotation in a Magnetic Field (Plasma)', fontsize=14)
    ax.set_xlabel('Propagation Direction (Z)', fontsize=12)
    ax.set_ylabel('Electric Field (X)', fontsize=12)
    ax.set_zlabel('Electric Field (Y)', fontsize=12)
    ax.legend(loc='upper right')
    
    # Set view angle for best visibility
    ax.view_init(elev=20., azim=-45)
    
    plt.show()

if __name__ == "__main__":
    simulate_faraday_rotation()