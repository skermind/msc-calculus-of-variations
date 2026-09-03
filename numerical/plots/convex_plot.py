import numpy as np
import matplotlib.pyplot as plt

# Domain
x = np.linspace(-2, 2, 500)

# Convex function
y = np.abs(x)**2

# Two points
x1, x2 = -1.5, 1.2
y1, y2 = np.abs(x1)**2, np.abs(x2)**2

# Chord joining the two points
chord_x = np.linspace(x1, x2, 100)
chord_y = y1 + (y2 - y1) * (chord_x - x1) / (x2 - x1)

# Create figure
fig, ax = plt.subplots(figsize=(7, 5))

# Plot function
ax.plot(
    x, y,
    linewidth=2,
    label=r"$f(x)=|x|^2=x^2$"
)

# Plot chord
ax.plot(
    chord_x, chord_y,
    linestyle="--",
    linewidth=1.5,
    label="Chord between two points"
)

# Mark points
ax.scatter([x1, x2], [y1, y2], zorder=3)

# Labels
ax.set_xlabel(r"$x$", fontsize=12)
ax.set_ylabel(r"$f(x)$", fontsize=12)

# Title
ax.set_title(
    r"Convex Function: $f(x)=|x|^2$",
    fontsize=13,
    pad=12
)

# Equal scaling
ax.set_aspect("equal", adjustable="box")

# Grid
ax.grid(True, alpha=0.3)

# Legend
ax.legend()

# Save
plt.savefig(
    "numerical/outputs/convex_function.pdf",
    bbox_inches="tight"
)

plt.show()