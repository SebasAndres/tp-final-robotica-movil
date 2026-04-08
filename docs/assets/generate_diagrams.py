"""
Generates PNG diagrams for the Informe.md:
  1. mecanum_platform.png   — top-view of the Mecanum wheel platform
  2. error_frame_rotation.png — map-frame vs robot-frame error geometry
  3. ekf_correccion.png     — EKF correction geometry (replaces the old low-res export)
"""

import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.transforms as transforms
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Arc
from matplotlib.lines import Line2D

OUT = "/home/sebastian/uba/Robotica-Movil-UBA/tp_final/docs/assets/"
DPI = 200

# ── colour palette ──────────────────────────────────────────────────────────
C_ROBOT  = "#4C72B0"
C_WHEEL  = "#2c2c2c"
C_ROLLER = "#e07b39"
C_ARROW  = "#1a1a1a"
C_RED    = "#c0392b"
C_GREEN  = "#27ae60"
C_BLUE   = "#2980b9"
C_GREY   = "#7f8c8d"
C_BG     = "white"

# ── helpers ─────────────────────────────────────────────────────────────────

def arrow(ax, x0, y0, dx, dy, color=C_ARROW, lw=1.5, hs=0.08, hw=0.06, **kw):
    ax.annotate("", xy=(x0+dx, y0+dy), xytext=(x0, y0),
                arrowprops=dict(arrowstyle=f"->,head_length={hs},head_width={hw}",
                                color=color, lw=lw), **kw)

def rot(v, theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([c*v[0]-s*v[1], s*v[0]+c*v[1]])

# ═══════════════════════════════════════════════════════════════════════════
# 1.  Mecanum platform — top view
# ═══════════════════════════════════════════════════════════════════════════

def draw_roller_lines(ax, cx, cy, angle_deg, n=5, span=0.08, lw=1.2):
    """Draw n parallel roller lines across a wheel centred at (cx,cy)."""
    angle = math.radians(angle_deg)
    perp  = math.radians(angle_deg + 90)
    step  = span / (n - 1)
    for i in range(n):
        t  = -span/2 + i * step
        sx = cx + math.cos(perp) * t
        sy = cy + math.sin(perp) * t
        ex = sx + math.cos(angle) * 0.045
        ey = sy + math.sin(angle) * 0.045
        ax.plot([sx - math.cos(angle)*0.045, ex],
                [sy - math.sin(angle)*0.045, ey],
                color=C_ROLLER, lw=lw, solid_capstyle="round", zorder=4)


def mecanum_platform():
    fig, ax = plt.subplots(figsize=(7, 6), facecolor=C_BG)
    ax.set_facecolor(C_BG)
    ax.set_aspect("equal")
    ax.axis("off")

    lx, ly = 0.175, 0.175       # half-lengths
    rw, rh = 0.08, 0.04         # wheel half-length, half-width

    # Robot body
    body = FancyBboxPatch((-lx-0.01, -ly-0.01), 2*(lx+0.01), 2*(ly+0.01),
                          boxstyle="round,pad=0.02",
                          linewidth=2, edgecolor=C_ROBOT,
                          facecolor="#d6e4f7", zorder=1)
    ax.add_patch(body)

    # Robot axes
    arrow(ax, 0, 0, 0.18, 0, color=C_RED,   lw=2, hs=0.12, hw=0.07)
    arrow(ax, 0, 0, 0,    0.18, color=C_GREEN, lw=2, hs=0.12, hw=0.07)
    ax.text(0.20, 0.01, "$x_R$",  color=C_RED,   fontsize=13, fontweight="bold")
    ax.text(0.01, 0.21, "$y_R$",  color=C_GREEN, fontsize=13, fontweight="bold")

    # Wheel positions and roller angles
    # fl=top-left, fr=top-right, rl=bottom-left, rr=bottom-right
    wheels = [
        ("fl", -lx, +ly,  45, r"$\omega_1$ (fl)"),
        ("fr", +lx, +ly, -45, r"$\omega_2$ (fr)"),
        ("rl", -lx, -ly, -45, r"$\omega_3$ (rl)"),
        ("rr", +lx, -ly,  45, r"$\omega_4$ (rr)"),
    ]
    label_off = {
        "fl": (-0.30,  0.06),
        "fr": ( 0.12,  0.06),
        "rl": (-0.30, -0.08),
        "rr": ( 0.12, -0.08),
    }

    for name, wx, wy, roller_angle, label in wheels:
        # Wheel rectangle (axis along Y of wheel = robot's X direction)
        rect = plt.Rectangle((wx - rh, wy - rw), 2*rh, 2*rw,
                              linewidth=1.5, edgecolor=C_WHEEL,
                              facecolor="#555555", zorder=2)
        ax.add_patch(rect)
        draw_roller_lines(ax, wx, wy, roller_angle, n=6, span=0.13)
        lox, loy = label_off[name]
        ax.text(wx + lox, wy + loy, label, fontsize=10, zorder=5,
                ha="left", va="center",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#aaaaaa", lw=0.7))

    # lx / ly annotations
    ax.annotate("", xy=(lx, -ly - 0.08), xytext=(0, -ly - 0.08),
                arrowprops=dict(arrowstyle="<->", color=C_GREY, lw=1.2))
    ax.text(lx/2, -ly - 0.13, r"$l_x = 0.175\ \rm{m}$",
            ha="center", fontsize=9, color=C_GREY)

    ax.annotate("", xy=(-lx - 0.10, ly), xytext=(-lx - 0.10, 0),
                arrowprops=dict(arrowstyle="<->", color=C_GREY, lw=1.2))
    ax.text(-lx - 0.20, ly/2, r"$l_y = 0.175\ \rm{m}$",
            ha="center", fontsize=9, color=C_GREY, rotation=90)

    # roller legend
    legend_elements = [
        Line2D([0], [0], color=C_ROLLER, lw=2, label="Rodillo pasivo (45°)"),
        mpatches.Patch(facecolor="#555555", edgecolor=C_WHEEL, label="Rueda Mecanum"),
        mpatches.Patch(facecolor="#d6e4f7", edgecolor=C_ROBOT, label="Chasis del robot"),
    ]
    ax.legend(handles=legend_elements, loc="lower right",
              fontsize=9, framealpha=0.95)

    ax.set_xlim(-0.55, 0.55)
    ax.set_ylim(-0.55, 0.55)
    ax.set_title("Plataforma Mecanum — vista superior", fontsize=13, fontweight="bold", pad=10)
    fig.tight_layout()
    fig.savefig(OUT + "mecanum_platform.png", dpi=DPI, bbox_inches="tight", facecolor=C_BG)
    plt.close(fig)
    print("✓ mecanum_platform.png")


# ═══════════════════════════════════════════════════════════════════════════
# 2.  Error frame rotation  (map → robot frame)
# ═══════════════════════════════════════════════════════════════════════════

def error_frame_rotation():
    theta = math.radians(40)          # robot heading

    fig, ax = plt.subplots(figsize=(8, 6), facecolor=C_BG)
    ax.set_facecolor(C_BG)
    ax.set_aspect("equal")
    ax.axis("off")

    # ── world frame axes ─────────────────────────────────────────────
    ax.annotate("", xy=(1.0, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->,head_length=0.15,head_width=0.08",
                                color=C_GREY, lw=1.5))
    ax.annotate("", xy=(0, 1.0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->,head_length=0.15,head_width=0.08",
                                color=C_GREY, lw=1.5))
    ax.text(1.05, -0.04, r"$X_{\rm map}$", color=C_GREY, fontsize=12)
    ax.text(-0.12, 1.05, r"$Y_{\rm map}$", color=C_GREY, fontsize=12)

    # ── robot body ────────────────────────────────────────────────────
    robot_pos = np.array([0.0, 0.0])
    body_pts  = np.array([[-0.15, -0.12], [0.15, -0.12],
                           [0.15,  0.12], [-0.15, 0.12]])
    c, s = math.cos(theta), math.sin(theta)
    R    = np.array([[c, -s], [s, c]])
    body_rot = (R @ body_pts.T).T + robot_pos
    robot_patch = plt.Polygon(body_rot, closed=True,
                              facecolor="#d6e4f7", edgecolor=C_ROBOT, lw=2, zorder=3)
    ax.add_patch(robot_patch)

    # heading arrow
    fwd = R @ np.array([0.28, 0])
    ax.annotate("", xy=robot_pos + fwd, xytext=robot_pos,
                arrowprops=dict(arrowstyle="->,head_length=0.12,head_width=0.07",
                                color=C_ROBOT, lw=2.0))

    # robot frame axes (small)
    xr = R @ np.array([0.22, 0])
    yr = R @ np.array([0, 0.22])
    ax.annotate("", xy=robot_pos+xr, xytext=robot_pos,
                arrowprops=dict(arrowstyle="->,head_length=0.10,head_width=0.06",
                                color=C_RED, lw=1.8))
    ax.annotate("", xy=robot_pos+yr, xytext=robot_pos,
                arrowprops=dict(arrowstyle="->,head_length=0.10,head_width=0.06",
                                color=C_GREEN, lw=1.8))
    ax.text(*(robot_pos + xr + [0.03, -0.03]), "$x_R$",
            color=C_RED, fontsize=11, fontweight="bold")
    ax.text(*(robot_pos + yr + [-0.10, 0.03]), "$y_R$",
            color=C_GREEN, fontsize=11, fontweight="bold")

    # theta arc
    arc = Arc((0,0), 0.45, 0.45, angle=0, theta1=0, theta2=math.degrees(theta),
              color=C_GREY, lw=1.2, linestyle="--")
    ax.add_patch(arc)
    mid_theta = theta / 2
    ax.text(0.26*math.cos(mid_theta), 0.26*math.sin(mid_theta),
            r"$\theta$", fontsize=12, color=C_GREY)

    # ── goal point ───────────────────────────────────────────────────
    goal = np.array([1.2, 0.85])
    ax.plot(*goal, "D", color=C_ARROW, ms=10, zorder=5)
    ax.text(goal[0]+0.05, goal[1]+0.05,
            r"Goal $(x_g, y_g)$", fontsize=11, color=C_ARROW)

    # ── error in map frame ────────────────────────────────────────────
    e_map = goal - robot_pos
    ax.annotate("", xy=goal, xytext=robot_pos,
                arrowprops=dict(arrowstyle="->,head_length=0.13,head_width=0.07",
                                color="#7f8c8d", lw=1.8, linestyle="dashed"))
    mid = robot_pos + e_map * 0.5
    ax.text(mid[0]+0.05, mid[1]-0.15,
            r"$e^{\rm map} = (e_x^{\rm map},\ e_y^{\rm map})$",
            fontsize=10, color=C_GREY, style="italic")

    # ── error in robot frame ─────────────────────────────────────────
    e_robot = R.T @ e_map          # rotate back to robot frame
    # draw in robot-frame coordinates then rotate back
    e_r_world = R @ e_robot        # should equal e_map — shown for illustration

    # project onto robot's xR axis (eˣᴿ component)
    ex_r_world = R @ np.array([e_robot[0], 0])
    ey_r_world = R @ np.array([0, e_robot[1]])

    ax.annotate("", xy=robot_pos + ex_r_world, xytext=robot_pos,
                arrowprops=dict(arrowstyle="->,head_length=0.13,head_width=0.07",
                                color=C_RED, lw=2.0))
    ax.annotate("", xy=robot_pos + ex_r_world + ey_r_world,
                xytext=robot_pos + ex_r_world,
                arrowprops=dict(arrowstyle="->,head_length=0.13,head_width=0.07",
                                color=C_GREEN, lw=2.0))

    ax.text(*(robot_pos + ex_r_world*0.5 + R @ np.array([0, -0.12])),
            r"$e_x^R$", color=C_RED, fontsize=11, fontweight="bold", ha="center")
    ax.text(*(robot_pos + ex_r_world + ey_r_world*0.5 + R @ np.array([0.10, 0])),
            r"$e_y^R$", color=C_GREEN, fontsize=11, fontweight="bold", ha="center")

    # rotation label box
    ax.text(0.62, -0.28,
            r"$e^R = R(\theta)^\top\, e^{\rm map}$",
            fontsize=12, ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.4", fc="#fffde7", ec="#f0c040", lw=1.5))

    ax.set_xlim(-0.45, 1.55)
    ax.set_ylim(-0.55, 1.25)
    ax.set_title("Rotación del error al frame del robot", fontsize=13, fontweight="bold", pad=10)
    fig.tight_layout()
    fig.savefig(OUT + "error_frame_rotation.png", dpi=DPI, bbox_inches="tight", facecolor=C_BG)
    plt.close(fig)
    print("✓ error_frame_rotation.png")


# ═══════════════════════════════════════════════════════════════════════════
# 3.  EKF correction geometry  (replaces low-res ekf_correccion.png)
# ═══════════════════════════════════════════════════════════════════════════

def ekf_correction():
    fig, ax = plt.subplots(figsize=(8.5, 6.5), facecolor=C_BG)
    ax.set_facecolor(C_BG)
    ax.set_aspect("equal")
    ax.axis("off")

    robot  = np.array([0.0, 0.0])          # μ⁻ predicted pose position
    post   = np.array([2.8, 2.0])          # known landmark P_i in map
    meas_r = 0.85                          # simulated measured range (shorter → robot moved)
    meas_b = math.atan2(post[1]-robot[1], post[0]-robot[0]) - 0.18  # bearing offset

    # expected measurement from predicted state
    delta  = post - robot
    rho    = np.linalg.norm(delta)
    beta   = math.atan2(delta[1], delta[0])

    # ── world axes ────────────────────────────────────────────────────
    for dx, dy, lbl in [(0.55, 0, r"$X_{\rm map}$"), (0, 0.55, r"$Y_{\rm map}$")]:
        ax.annotate("", xy=(robot[0]+dx, robot[1]+dy), xytext=robot,
                    arrowprops=dict(arrowstyle="->,head_length=0.12,head_width=0.06",
                                    color=C_GREY, lw=1.2))
    ax.text(robot[0]+0.58, robot[1]-0.04, r"$X_{\rm map}$", color=C_GREY, fontsize=11)
    ax.text(robot[0]-0.14, robot[1]+0.58, r"$Y_{\rm map}$", color=C_GREY, fontsize=11)

    # ── robot (predicted μ⁻) ──────────────────────────────────────────
    robot_ang = 0.2   # heading θ
    rc, rs = math.cos(robot_ang), math.sin(robot_ang)
    Rr = np.array([[rc,-rs],[rs,rc]])
    body_pts = np.array([[-0.12,-0.09],[0.12,-0.09],[0.12,0.09],[-0.12,0.09]])
    body_rot = (Rr @ body_pts.T).T + robot
    ax.add_patch(plt.Polygon(body_rot, facecolor="#d6e4f7", edgecolor=C_ROBOT, lw=2, zorder=4))
    fwd_r = Rr @ np.array([0.20, 0])
    ax.annotate("", xy=robot+fwd_r, xytext=robot,
                arrowprops=dict(arrowstyle="->,head_length=0.10,head_width=0.06",
                                color=C_ROBOT, lw=1.8))
    ax.text(robot[0]-0.30, robot[1]-0.18, r"$\boldsymbol{\mu}_k^-$",
            fontsize=13, color=C_ROBOT, fontweight="bold", zorder=5)

    # ── post (known landmark P_i) ─────────────────────────────────────
    ax.plot(*post, "^", color="#8e44ad", ms=14, zorder=5, markeredgewidth=1.5,
            markeredgecolor="white")
    ax.text(post[0]+0.10, post[1]+0.08,
            r"$P_i\ (x_i, y_i)$", fontsize=11, color="#8e44ad", fontweight="bold")

    # ── expected range ρᵢ ─────────────────────────────────────────────
    ax.annotate("", xy=post, xytext=robot,
                arrowprops=dict(arrowstyle="->,head_length=0.14,head_width=0.07",
                                color=C_BLUE, lw=2.0, linestyle="-"))
    mid_rho = robot + delta * 0.48
    ax.text(mid_rho[0]+0.10, mid_rho[1]-0.28,
            r"$\rho_i = \sqrt{\delta_x^2+\delta_y^2}$  (rango esperado)",
            fontsize=10, color=C_BLUE)

    # ── h(μ⁻) point at tip of expected measurement ───────────────────
    ax.plot(*post, "o", color=C_BLUE, ms=9, zorder=6, markeredgecolor="white",
            markeredgewidth=1.2)
    ax.text(post[0]-0.55, post[1]-0.22,
            r"$h(\boldsymbol{\mu}^-)$", fontsize=11, color=C_BLUE, fontweight="bold")

    # ── bearing arc β ─────────────────────────────────────────────────
    arc_r = 0.85
    arc = Arc(robot, 2*arc_r, 2*arc_r, angle=0,
              theta1=0, theta2=math.degrees(beta),
              color=C_BLUE, lw=1.4, linestyle="--")
    ax.add_patch(arc)
    mid_b = robot_ang + beta/2 - 0.1
    ax.text(robot[0] + arc_r*0.75*math.cos(beta/2+0.05),
            robot[1] + arc_r*0.75*math.sin(beta/2+0.05),
            r"$\beta_i$", fontsize=12, color=C_BLUE)

    # ── measured ray (z) ─────────────────────────────────────────────
    meas_end = robot + np.array([math.cos(beta+meas_b), math.sin(beta+meas_b)]) * (rho - 0.35)
    ax.annotate("", xy=meas_end, xytext=robot,
                arrowprops=dict(arrowstyle="->,head_length=0.14,head_width=0.07",
                                color=C_RED, lw=2.0, linestyle="-"))
    mid_z = robot + np.array([math.cos(beta+meas_b), math.sin(beta+meas_b)]) * (rho-0.35)*0.5
    ax.text(mid_z[0]-0.50, mid_z[1]+0.10,
            r"$\mathbf{z} = ({\rm range}_z,\ {\rm bearing}_z)$  (medido)",
            fontsize=10, color=C_RED)

    # ── innovation Δz ────────────────────────────────────────────────
    ax.annotate("", xy=meas_end, xytext=post,
                arrowprops=dict(arrowstyle="->,head_length=0.12,head_width=0.06",
                                color="#e67e22", lw=1.8, linestyle=":"))
    ax.text((meas_end[0]+post[0])/2 + 0.10, (meas_end[1]+post[1])/2 - 0.10,
            r"$\tilde{\mathbf{z}} = \mathbf{z} - h(\boldsymbol{\mu}^-)$",
            fontsize=10, color="#e67e22",
            bbox=dict(boxstyle="round,pad=0.25", fc="#fef9e7", ec="#e67e22", lw=1))

    # ── δx, δy annotations ───────────────────────────────────────────
    ax.plot([robot[0], post[0]], [robot[1], robot[1]],
            linestyle=":", color="#aaaaaa", lw=1.2)
    ax.plot([post[0], post[0]], [robot[1], post[1]],
            linestyle=":", color="#aaaaaa", lw=1.2)
    ax.text((robot[0]+post[0])/2, robot[1]-0.15,
            r"$\delta_x = x_i - \mu_x$", fontsize=9.5, color="#aaaaaa", ha="center")
    ax.text(post[0]+0.08, (robot[1]+post[1])/2,
            r"$\delta_y = y_i - \mu_y$", fontsize=9.5, color="#aaaaaa")

    # ── legend ────────────────────────────────────────────────────────
    legend_elements = [
        Line2D([0],[0], color=C_BLUE,     lw=2,   label=r"Medición esperada $h(\mu^-)$: $(\rho_i, \beta_i)$"),
        Line2D([0],[0], color=C_RED,      lw=2,   label=r"Medición real del LiDAR: $\mathbf{z}$"),
        Line2D([0],[0], color="#e67e22",  lw=2,   label=r"Innovación $\tilde{\mathbf{z}}$",
               linestyle=":"),
        Line2D([0],[0], marker="^", color="#8e44ad", ms=10, lw=0,
               label=r"Poste conocido $P_i$"),
    ]
    ax.legend(handles=legend_elements, loc="lower right", fontsize=9.5, framealpha=0.95)

    ax.set_xlim(-0.8, 3.7)
    ax.set_ylim(-0.75, 2.9)
    ax.set_title(r"EKF — Geometría de la corrección: $h(\boldsymbol{\mu}_k^-)$",
                 fontsize=13, fontweight="bold", pad=10)
    fig.tight_layout()
    fig.savefig(OUT + "ekf_correccion.png", dpi=DPI, bbox_inches="tight", facecolor=C_BG)
    plt.close(fig)
    print("✓ ekf_correccion.png")


if __name__ == "__main__":
    mecanum_platform()
    error_frame_rotation()
    ekf_correction()
    print("All done.")
