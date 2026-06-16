"""Render a pendulum_rs CSV position dump into an animated GIF.

The Rust binary (`pendulum_rs --csv frames.csv`) writes one row per frame:
    x0,y0,x1,y1,...,xn,yn   (joint positions including the anchor)
This script just visualizes those points — the physics is entirely Rust's, so
the resulting GIF is proof the Rust simulator behaves like the Python one.

    python render_from_csv.py frames.csv out.gif [fps]
"""

import sys
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

csv_path = sys.argv[1]
out_path = sys.argv[2] if len(sys.argv) > 2 else "pendulum_rs.gif"
fps = int(sys.argv[3]) if len(sys.argv) > 3 else 60

rows = np.loadtxt(csv_path, delimiter=",")
frames = rows.reshape(rows.shape[0], -1, 2)  # (T, n+1, 2)
reach = float(np.abs(frames).max()) * 1.1

fig, ax = plt.subplots(figsize=(5, 5))
ax.set_xlim(-reach, reach)
ax.set_ylim(-reach, reach)
ax.set_aspect("equal")
ax.set_title("pendulum_rs (Rust sim)")
ax.grid(alpha=0.2)

(arm,) = ax.plot([], [], "o-", lw=2, color="#1e90ff", markersize=6)
(trail,) = ax.plot([], [], "-", lw=1, color="#ff5050", alpha=0.6)
tx: list[float] = []
ty: list[float] = []
trail_len = fps * 2

# Subsample to ~30 fps GIF if the sim ran faster, to keep file size sane.
stride = max(1, fps // 30)


def update(i: int):
    pts = frames[i]
    arm.set_data(pts[:, 0], pts[:, 1])
    tx.append(pts[-1, 0])
    ty.append(pts[-1, 1])
    if len(tx) > trail_len:
        del tx[0], ty[0]
    trail.set_data(tx, ty)
    return arm, trail


idx = range(0, len(frames), stride)
anim = FuncAnimation(fig, update, frames=idx, interval=1000 / 30, blit=True)
anim.save(out_path, writer=PillowWriter(fps=30))
print(f"wrote {out_path} from {len(frames)} Rust frames")
