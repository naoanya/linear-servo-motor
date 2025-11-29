import numpy as np
import matplotlib.pyplot as plt

# Generate synthetic sensor signals
P = 5*np.pi
theta = np.linspace(0, P, 500)

# A/B/C as 120° offset sine waves
A = np.sin(theta)
B = np.sin(theta - 2*np.pi/3)
C = np.sin(theta - 4*np.pi/3)

# Compute alpha, beta, angle
offset = (A + B + C) / 3
A2 = A - offset
B2 = B - offset
C2 = C - offset

alpha = A2
beta = (B2 - C2) / np.sqrt(3)
angle = np.arctan2(beta, alpha)

# Create one figure with 3 subplots
fig, axes = plt.subplots(3, 1, figsize=(10, 8))  # 3 rows, 1 column

# Plot A/B/C signals
axes[0].plot(theta, A, label='A')
axes[0].plot(theta, B, label='B')
axes[0].plot(theta, C, label='C')
axes[0].set_title("A/B/C signals")
axes[0].set_xlabel("theta")
axes[0].set_ylabel("value")
axes[0].legend()

# Plot alpha/beta
axes[1].plot(theta, alpha, label='alpha')
axes[1].plot(theta, beta, label='beta')
axes[1].set_title("alpha / beta")
axes[1].set_xlabel("theta")
axes[1].set_ylabel("value")
axes[1].legend()

# Plot angle
axes[2].plot(theta, angle, label='angle')
axes[2].set_title("computed angle")
axes[2].set_xlabel("theta")
axes[2].set_ylabel("radians")
axes[2].legend()

plt.tight_layout()
plt.show()
