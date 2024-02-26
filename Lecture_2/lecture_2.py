import numpy as np
import matplotlib.pyplot as plt
from itertools import product

# Parameters
beta = np.array([0.1, 1, 10, 100])  # Transmission rate
delta = np.array([0.1, 1, 10, 100])  # Elimination rate of zombies by humans
gamma = np.array([0.1, 1, 10, 100])  # Recovery rate of zombies back to humans

# Extended system of differential equations
def extended_zombie_apocalypse(S, Z, beta, delta, gamma):
    dSdt = -beta * S * Z
    dZdt = beta * S * Z - delta * Z - gamma * Z
    return dSdt, dZdt

# Grid resolution
S_values, Z_values = np.meshgrid(np.linspace(0, 500, 30), np.linspace(0, 500, 30))  # Manually set grid resolution

# Iterate over all combinations of beta, delta, and gamma
for beta_val, delta_val, gamma_val in product(beta, delta, gamma):
    # Compute vector field
    dS, dZ = extended_zombie_apocalypse(S_values, Z_values, beta_val, delta_val, gamma_val)

    # Normalize vectors (for better visualization)
    N = np.sqrt(dS**2 + dZ**2)
    N[N == 0] = 1  # Avoid division by zero
    dS /= N
    dZ /= N

    # Plotting the vector field
    plt.figure(figsize=(10, 6))
    plt.quiver(S_values, Z_values, dS, dZ, N)
    plt.xlabel('Number of Susceptible Humans')
    plt.ylabel('Number of Zombies')
    plt.title(f'Extended Vector Field of Zombie Apocalypse with Recovery: {(beta_val, delta_val, gamma_val)}')
    plt.grid(True)
    plt.xlim([0, 500])
    plt.ylim([0, 500])
plt.show()
