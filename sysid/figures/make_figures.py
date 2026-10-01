"""Figures for sysid/README.md.

Writes figures.drawio (one editable page per figure, robot_example.xml style) and one SVG
per figure next to it (embedded by the README).

    python sysid/figures/make_figures.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drawio_lib import (BLUE_F, BLUE_S, DKBLUE, GREEN, GREY_F, GREY_S, ORNG_F, ORNG_S, RED,  # noqa: E402
                        Fig, write)

HERE = os.path.dirname(os.path.abspath(__file__))


def panel_title(f, x, y, s):
    f.text(x, y, s, size=12, align="left", bold=True)


# ------------------------------------------------------------------ 1. testbed and frames
def fig_frames():
    f = Fig("fig2_testbed_frames", "2 testbed and frames", 760, 440)
    # ground + world frame + gravity
    f.line((20, 402), (740, 402), color=GREY_S)
    for x in range(30, 740, 18):
        f.line((x, 402), (x - 8, 412), color="#aaaaaa")
    f.triad((70, 360), "{W}", ("x", 0), ("z", 90), out=("y", -1))
    f.line((150, 250), (150, 318), end=True, label="g", t=0.45)
    # hexapod
    f.rect(380, 391, 340, 16, fill=GREY_F, stroke=GREY_S)
    for (xb, xt) in [(240, 300), (300, 260), (360, 430), (400, 330), (460, 500), (520, 460)]:
        f.line((xb, 383), (xt, 302), color=GREY_S, w=2)
    f.text(380, 425, "hexapod (Stewart platform): carries the plate physically", size=11, color=GREY_S)
    f.rect(380, 296, 380, 12, fill=BLUE_F, stroke=BLUE_S)
    # simulated base inertia
    f.rect(385, 238, 108, 96, fill=ORNG_F, stroke=ORNG_S, dashed=True, opacity=60,
           label="simulated\nbase inertia\n(3000–5000 kg)", size=11)
    # one arm on its F/T sensor
    sx = 500
    f.rect(sx, 284, 38, 12, fill=GREY_F, stroke=GREY_S)
    f.rect(sx, 252, 30, 52)
    f.text(sx - 30, 250, "B_{0}", size=11, color=DKBLUE)
    f.cylinder(sx, 214, 0)
    j2, j3, j4, j5, j6 = (sx, 180), (600, 110), (690, 160), (690, 192), (690, 224)
    f.line((sx, 226), (sx, 204))
    f.line((sx, 204), j2)
    f.line(j2, j3)
    f.line(j3, j4)
    f.line(j4, (690, 182))
    f.line((690, 202), j6)
    f.zjoint(j2, +1, start_deg=180)
    f.zjoint(j3, +1, start_deg=40)
    f.zjoint(j4, +1, start_deg=-60)
    f.cylinder(*j5, 0)
    f.circle(*j6, 7)
    f.dot(*j6, 2)
    f.line((690, 231), (690, 238))
    f.gripper((690, 238))
    for lab, (x, y) in [("B_{1}", (452, 196)), ("B_{2}", (532, 128)), ("B_{3}", (660, 118)),
                        ("B_{4}", (718, 176)), ("B_{5}", (718, 208)), ("B_{6}", (728, 254))]:
        f.text(x, y, lab, size=11, color=DKBLUE)
    # measured wrench
    f.line((560, 296), (560, 256), color=RED, w=1.5, end=True)
    f.text(606, 272, "w_{S} (F/T)", size=11, color=RED)
    # frames
    f.triad((255, 290), "{P}", ("x", 0), ("z", 90), out=("y", -1), L=30)
    f.triad((sx + 3, 284), "", ("x", 0), ("z", 90), out=("y", -1), L=26, size=10)
    f.text(sx - 24, 283, "{S}", size=12, bold=True, align="right")
    f.triad(j3, "", ("x", -29), ("y", 61), L=26, size=10)
    f.text(625, 88, "{k}: frame of B_{k},\non joint axis k", size=10, align="left")
    return f


# ------------------------------------------------------------------ 2. spatial vectors
def fig_spatial():
    f = Fig("fig3_spatial_vectors", "3 spatial vectors", 800, 480)
    # (a) twist
    panel_title(f, 20, 20, "(a) twist  v = [ν; ω]")
    f.ellipse(170, 125, 105, 62, opacity=70)
    o = (130, 140)
    f.triad(o, "{B}", ("x", 0), ("y", 90), out=("z", 1), L=30)
    f.line(o, (215, 105), color=DKBLUE, w=1.5, end=True)
    f.text(228, 98, "ν", size=14, color=DKBLUE)
    f.line((85, 78), (258, 82), end=True, curved_pts=[(130, 52), (175, 44), (220, 52)])
    f.text(170, 32, "ω", size=14)
    f.text(190, 212, "ν: velocity of the point at the origin of {B}\nω: angular velocity   (both in {B} coordinates)",
           size=11, color=GREY_S)
    # (b) wrench
    panel_title(f, 400, 20, "(b) wrench  f = [F; τ]")
    f.ellipse(550, 125, 105, 62, opacity=70)
    o = (510, 140)
    f.triad(o, "{B}", ("x", 0), ("y", 90), out=("z", 1), L=30)
    f.line(o, (600, 95), color=RED, w=1.5, end=True)
    f.text(612, 88, "F", size=14, color=RED)
    arc = [(o[0] + 42 * math.cos(math.radians(a)), o[1] - 42 * math.sin(math.radians(a))) for a in range(160, 350, 30)]
    f.line(arc[0], arc[-1], end=True, curved_pts=arc[1:-1])
    f.text(o[0] - 52, o[1] - 24, "τ", size=14)
    f.text(570, 212, "F: force;  τ: moment about the origin of {B}", size=11, color=GREY_S)
    # (c) transform
    panel_title(f, 20, 262, "(c) change of frame")
    a, b = (70, 420), (270, 330)
    f.triad(a, "{A}", ("x", 0), ("y", 90), out=("z", 1), L=34)
    f.triad(b, "{B}", ("x", 30), ("y", 120), out=("z", 1), L=34, name_off=(10, 18))
    f.line(a, b, dash=True, end=True, label="p = ^{A}p_{B}", t=0.45)
    f.text(210, 455, "^{A}T_{B} = (R, p);   ^{A}X_{B} maps twists, ^{A}X^{*}_{B} maps wrenches, {B} → {A}",
           size=11, color=GREY_S)
    # (d) spatial vs classical acceleration
    panel_title(f, 400, 262, "(d) spatial vs classical acceleration")
    c = (575, 370)
    f.ellipse(c[0], c[1], 70, 70, fill="#ffffff", stroke=GREY_S, dashed=True)
    f.zjoint(c, +1, start_deg=200)
    f.text(c[0] + 22, c[1] + 26, "ω", size=13)
    p = (c[0], c[1] - 70)
    f.dot(*p, 4)
    f.line(p, (p[0] - 70, p[1]), color=DKBLUE, w=1.5, end=True)
    f.text(p[0] - 84, p[1] - 12, "ν", size=14, color=DKBLUE)
    f.line(p, (p[0], p[1] + 44), color=RED, w=1.5, end=True)
    f.text(p[0] + 10, p[1] + 30, "p̈ = ω × ν", size=12, color=RED, align="left", bg=True)
    f.text(662, 330, "uniform rotation:\na_{lin} = ν̇ = 0,\nyet p̈ = ω × ν ≠ 0",
           size=11, color=GREY_S, align="left")
    return f


# ------------------------------------------------------------------ 3. one rigid body
def fig_body():
    f = Fig("fig4_rigid_body", "4 one rigid body", 560, 320)
    f.ellipse(290, 150, 190, 105, opacity=70)
    o, cm = (160, 195), (350, 115)
    u = np.subtract(cm, o) / np.linalg.norm(np.subtract(cm, o))
    f.line(o, tuple(np.subtract(cm, 8 * u)), end=True, label="c", t=0.5, lsize=13)
    f.circle(*cm, 8, fill="#ffffff", stroke="#000000")
    f.line((cm[0] - 8, cm[1]), (cm[0] + 8, cm[1]))
    f.line((cm[0], cm[1] - 8), (cm[0], cm[1] + 8))
    f.text(cm[0] + 16, cm[1] - 26, "COM:  m,  I_{C}", size=12, align="left")
    f.triad(o, "{k}", ("x", 0), ("y", 90), out=("z", 1), L=34)
    f.text(o[0] + 12, o[1] + 40, "about the origin:  Ī = I_{C} − m[c]_{×}^{2}", size=12, align="left")
    f.text(290, 290, "π = [ m,  h = m c,  Ī_{xx}, Ī_{xy}, Ī_{yy}, Ī_{xz}, Ī_{yz}, Ī_{zz} ]   (10 numbers; ℐ is linear in π)",
           size=12)
    return f


# ------------------------------------------------------------------ 4. RNEA
def fig_rnea():
    f = Fig("fig5_rnea", "5 RNEA", 760, 300)
    y = 150
    f.rect(48, y, 14, 60, fill=GREY_F, stroke=GREY_S)
    f.text(48, y + 46, "{S}", size=12, bold=True)
    slots = [("B_{0}", 125), ("B_{1}", 205), ("B_{2}", 285), ("…", 355), ("B_{λ(k)}", 435), ("B_{k}", 530),
             ("…", 610), ("B_{6}", 690)]
    prev = 55
    for lab, x in slots:
        if lab == "…":
            f.text(x, y, "…", size=16)
            prev = x + 10
            continue
        f.line((prev, y), (x - 30, y))
        f.rect(x, y, 60, 40, label=lab, size=12)
        prev = x + 30
    for x in (165, 245, 482):
        f.zjoint((x, y), +1, arrow=False)
    f.text(482, y + 32, "joint k: s_{k}, q_{k}", size=11, color=DKBLUE)
    f.line((95, 88), (715, 88), end=True, label="forward pass (root → tip):  v_{k},  a_{k}", t=0.5)
    f.line((715, 212), (95, 212), end=True, label="backward pass (tip → root):  f_{k};   τ_{k} = s_{k}^{T} f_{k}", t=0.5)
    f.text(70, 45, "a_{0} = a_{base} − g", size=12, align="left", color=RED)
    f.line((85, 55), (100, 82), color=RED)
    f.text(70, 262, "f_{root} = w_{S}   (what the F/T sensor measures)", size=12, align="left", color=RED)
    f.line((85, 250), (100, 218), color=RED)
    return f


# ------------------------------------------------------------------ 5. HIL loop
def fig_hil():
    f = Fig("fig1_hil_loop", "1 HIL loop", 780, 320)
    A, B, S, C = (115, 80), (320, 80), (455, 80), (650, 80)
    D, E = (650, 215), (320, 215)
    f.rect(*A, 150, 54, label="arms + platform\n(physical hardware)")
    f.rect(*B, 120, 54, label="F/T sensors\nw̃")
    f.circle(*S, 15, fill="#ffffff", stroke="#000000", label="Σ", size=13)
    f.text(S[0] - 22, S[1] - 20, "+", size=13)
    f.text(S[0] - 20, S[1] + 26, "−", size=14)
    f.rect(*C, 190, 54, label="simulated free-floating base\n(3000–5000 kg)")
    f.rect(*D, 190, 54, label="hexapod motion\n(with washout)")
    f.rect(*E, 190, 54, label="KDL RNE, per-link YAML\npredicted arm wrench")
    f.line((A[0] + 75, A[1]), (B[0] - 60, B[1]), end=True)
    f.line((B[0] + 60, B[1]), (S[0] - 15, S[1]), end=True)
    f.line((S[0] + 15, S[1]), (C[0] - 95, C[1]), end=True, label="ŵ_{ext}", t=0.5)
    f.line((C[0], C[1] + 27), (D[0], D[1] - 27), end=True, label="base motion", t=0.5)
    f.line((A[0], A[1] + 27), (A[0], E[1]), end=False)
    f.line((A[0], E[1]), (E[0] - 95, E[1]), end=True, label="q, q̇, q̈", t=0.5)
    f.line((E[0] + 95, E[1]), (S[0], E[1]), end=False)
    f.line((S[0], E[1]), (S[0], S[1] + 15), end=True)
    f.line((D[0] + 95, D[1]), (760, D[1]))
    f.line((760, D[1]), (760, 300))
    f.line((760, 300), (22, 300))
    f.line((22, 300), (22, A[1]))
    f.line((22, A[1]), (A[0] - 75, A[1]), end=True)
    f.text(510, 30, "model error enters here: Y (π_{true} − π_{model})", size=11, color=RED)
    f.line((510, 40), (510, 74), color=RED, end=True)
    f.text(320, 268, "this project: identify π̂ → per-link YAML", size=11, color=RED, bold=True)
    f.line((320, 258), (320, 243), color=RED, end=True)
    return f


# ------------------------------------------------------------------ 6. identifiability
def fig_ident():
    f = Fig("fig6_identifiability", "6 identifiability", 560, 340)
    o = (60, 300)
    f.line(o, (520, 300), end=True)
    f.line(o, (60, 30), end=True)
    f.text(530, 312, "π_{1}", size=12)
    f.text(48, 30, "π_{2}", size=12, align="right")
    p0, p1 = np.array([110.0, 60.0]), np.array([500.0, 250.0])
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    f.line(tuple(p0), tuple(p1), color=DKBLUE, w=2)
    f.text(255, 40, "{π : Yπ = w}\n(null-space direction: no effect on w)", size=11, color=DKBLUE)
    prior = np.array([200.0, 245.0])
    foot = p0 + ((prior - p0) @ d) * d
    truth = p0 + 0.82 * (p1 - p0)
    f.line(tuple(prior), tuple(foot), dash=True)
    f.dot(*prior, 4, GREY_S)
    f.text(prior[0] + 8, prior[1] + 18, "π_{URDF} (prior)", size=12, align="left", color=GREY_S)
    f.dot(*foot, 5, DKBLUE)
    f.text(foot[0] + 14, foot[1] - 22, "estimate π̂", size=12, align="left", color=DKBLUE)
    f.dot(*truth, 5, RED)
    f.text(truth[0] + 8, truth[1] + 20, "π_{true}", size=12, align="left", color=RED)
    n = np.array([d[1], -d[0]])
    a0 = p0 + 0.35 * (p1 - p0)
    f.line(tuple(a0 - 45 * n), tuple(a0 + 45 * n), start=True, end=True, color=RED)
    f.text(a0[0] - 45 * n[0] - 10, a0[1] - 45 * n[1] - 12, "identifiable: β = V_{r}^{T}π", size=11, color=RED, align="right")
    f.text(290, 330, "both π̂ and π_{true} predict the same wrench; the prior picks π̂", size=11, color=GREY_S)
    return f


# ------------------------------------------------------------------ 7. pseudo-inertia
def fig_pseudo():
    f = Fig("fig7_pseudo_inertia", "7 pseudo-inertia", 620, 300)
    panel_title(f, 20, 20, "(a) any real body")
    f.ellipse(130, 130, 95, 62, opacity=70)
    rng = np.random.default_rng(3)
    for _ in range(40):
        a, r = rng.uniform(0, 2 * math.pi), math.sqrt(rng.uniform()) * 0.85
        f.dot(130 + 95 * r * math.cos(a), 130 + 62 * r * math.sin(a), 1.6, DKBLUE)
    f.triad((70, 170), "", ("x", 0), ("y", 90), L=24, size=10)
    f.text(130, 222, "J = ∫ [x; 1][x; 1]^{T} dρ = [Σ, h; h^{T}, m] ⪰ 0", size=12)
    f.text(130, 250, "m = ∫dρ,  h = ∫x dρ,  Σ = ∫xx^{T} dρ", size=11, color=GREY_S)
    panel_title(f, 320, 20, "(b) constrained fit")
    f.ellipse(450, 140, 115, 80, fill=BLUE_F, stroke=BLUE_S, opacity=70)
    f.text(430, 130, "physically realizable\nJ(π_{k}) ⪰ 0", size=12)
    bad, ok = (590, 60), (522, 88)
    f.dot(*bad, 5, RED)
    f.text(590, 36, "unconstrained LS", size=11, color=RED, align="right")
    f.line(bad, ok, dash=True, end=True)
    f.dot(*ok, 5, DKBLUE)
    f.text(520, 112, "SDP fit", size=11, color=DKBLUE)
    f.text(450, 250, "outside: e.g. m < 0, or principal moments\nviolating the triangle inequality", size=11, color=GREY_S)
    return f


# ------------------------------------------------------------------ 8. data pipeline
def fig_pipeline():
    f = Fig("fig9_pipeline", "9 data pipeline", 780, 330)
    f.rect(70, 150, 100, 44, label=".mcap log", fill=GREY_F, stroke=GREY_S)
    f.rect(225, 150, 140, 44, label="extract_mcap.py\n→ .npz")
    rows = [("joint states, 500 Hz\nSavitzky–Golay → q, q̇, q̈", 60),
            ("platform pose, 250 Hz\n→ R_{WS}, ω, α, a_{S}", 150),
            ("F/T wrench, 250 Hz\nsign σ, zero-phase low-pass", 240)]
    for lab, y in rows:
        f.rect(430, y, 200, 48, label=lab, size=11)
    f.line((120, 150), (155, 150), end=True)
    f.line((295, 150), (330, 150), end=True)
    f.line((312, 150), (312, 60))
    f.line((312, 60), (330, 60), end=True)
    f.line((312, 150), (312, 240))
    f.line((312, 240), (330, 240), end=True)
    f.rect(665, 105, 170, 48, label="regressor Y\n(arm_model, Pinocchio)", size=11)
    f.rect(665, 230, 170, 48, label="physically-consistent\nSDP fit (identify)", size=11)
    f.line((530, 60), (665, 60))
    f.line((665, 60), (665, 81), end=True)
    f.line((530, 150), (560, 150))
    f.line((560, 150), (560, 105))
    f.line((560, 105), (580, 105), end=True)
    f.line((665, 129), (665, 206), end=True, label="Y", t=0.5)
    f.line((530, 240), (580, 240), end=True, label="w̃", t=0.5)
    f.line((665, 254), (665, 290), end=True)
    f.text(665, 305, "π̂, b̂ → validate → YAML → KDL", size=11, bold=True)
    return f


# ------------------------------------------------------------------ 9. excitation
def fig_excitation():
    f = Fig("fig8_excitation", "8 excitation", 620, 270)
    x0, y0, W, H = 60, 135, 520, 100            # axes: t in [0, 2T], q scaled to +-H
    f.line((x0, y0), (x0 + W + 20, y0), end=True)
    f.line((x0, y0 + H + 10), (x0, y0 - H - 10), end=True)
    f.text(x0 + W + 30, y0 + 14, "t", size=12)
    f.text(x0 + 12, y0 - H - 6, "q_{j}(t) − q_{j,0}", size=12, align="left")
    f0, L, T = 0.1, 5, 10.0
    rng = np.random.default_rng(7)
    t = np.linspace(0, 2 * T, 400)
    for col in (DKBLUE, RED, GREEN):
        a, b = rng.normal(size=(2, L)) / np.arange(1, L + 1)
        wl = 2 * math.pi * f0 * np.arange(1, L + 1)
        q = (np.sin(np.outer(t, wl)) * a / wl - np.cos(np.outer(t, wl)) * b / wl).sum(axis=1)
        q = q - q.mean()
        q = q / np.abs(q).max() * H * 0.9
        f.poly([(x0 + ti / (2 * T) * W, y0 - qi) for ti, qi in zip(t, q)], color=col, w=1.5)
    xT = x0 + W / 2
    f.line((xT, y0 - H - 5), (xT, y0 + H + 5), dash=True, color=GREY_S)
    f.text(xT + 6, y0 + H + 16, "T = 1/f_{0} = 10 s   (L = 5 harmonics)", size=11, color=GREY_S, align="left")
    f.text(x0 + W - 4, y0 - H - 4, "3 of the 6 joints", size=11, color=GREY_S, align="right")
    return f


if __name__ == "__main__":
    figs = [fig_hil(), fig_frames(), fig_spatial(), fig_body(), fig_rnea(), fig_ident(), fig_pseudo(),
            fig_excitation(), fig_pipeline()]      # order of appearance in the README
    write(figs, os.path.join(HERE, "figures.drawio"), HERE)
    print("wrote", os.path.join(HERE, "figures.drawio"), "and", len(figs), "SVGs")
