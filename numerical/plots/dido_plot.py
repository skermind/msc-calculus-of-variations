import numpy as np
import matplotlib.pyplot as plt

# Parameter
l = 1

# Domain
s = np.linspace(0, l, 500)

# Parametric curve
x = -(l / np.pi) * np.cos(np.pi * s / l) - l / np.pi
y = (l / np.pi) * np.sin(np.pi * s / l)

# Create figure
fig, ax = plt.subplots(figsize=(7, 5))

ax.plot(x, y, linewidth=2)

# Labels
ax.set_xlabel(r"$x$", fontsize=12)
ax.set_ylabel(r"$y$", fontsize=12)

# Title
ax.set_title(
    r"Dido's Problem: Parametric Solution"
    "\n"
    r"$x(s)=-\frac{\ell}{\pi}\cos\left(\frac{\pi s}{\ell}\right)"
    r"-\frac{\ell}{\pi},\quad"
    r"y(s)=\frac{\ell}{\pi}\sin\left(\frac{\pi s}{\ell}\right)$"
    "\n"
    rf"$\ell={l},\quad s\in[0,{l}]$",
    fontsize=12,
    pad=12
)

# Equal scaling
ax.set_aspect("equal", adjustable="box")

# Grid
ax.grid(True, alpha=0.3)

# Mark endpoints
ax.scatter([x[0], x[-1]], [y[0], y[-1]], zorder=3)

ax.annotate(
    r"$A$",
    (x[0], y[0]),
    xytext=(-10, -15),
    textcoords="offset points"
)

ax.annotate(
    r"$B$",
    (x[-1], y[-1]),
    xytext=(5, -15),
    textcoords="offset points"
)

# Save
plt.savefig(
    "numerical/outputs/dido.pdf",
    bbox_inches="tight"
)

plt.show()