"""Tiny drawing library that writes the same figure twice:

- as a draw.io page (editable; styles copied from rss_platform/robot_example.xml:
  blue #dae8fc/#6c8ebf shapes, default-width black lines, uTurnArrow rotation arrows,
  cylinder3 joints, dimension labels attached to their arrows as edgeLabels), and
- as an SVG approximation, which is what the markdown README embeds.

Labels accept a tiny markup: `x_{sub}` and `x^{sup}` become subscripts / superscripts
(<sub>/<sup> in draw.io, baseline-shifted tspans in SVG). Lines split on "\\n".
"""
import html
import math
import re

BLUE_F, BLUE_S = "#dae8fc", "#6c8ebf"
GREY_F, GREY_S = "#f5f5f5", "#666666"
ORNG_F, ORNG_S = "#ffe6cc", "#d79b00"
RED, GREEN, DKBLUE = "#b85450", "#82b366", "#1a4f8b"
FONT = "Helvetica"
DASH = ' stroke-dasharray="6 4"'
RX = 'rx="8" '
_MARK = re.compile(r"([_^])\{([^}]*)\}")


def _rich_html(s):
    out, i = [], 0
    for m in _MARK.finditer(s):
        out.append(html.escape(s[i:m.start()]))
        tag = "sub" if m.group(1) == "_" else "sup"
        out.append(f"<{tag}>{html.escape(m.group(2))}</{tag}>")
        i = m.end()
    out.append(html.escape(s[i:]))
    return "".join(out)


def _rich_svg(s):
    out, i = [], 0
    for m in _MARK.finditer(s):
        if s[i:m.start()]:
            out.append(f"<tspan>{html.escape(s[i:m.start()])}</tspan>")
        shift = "sub" if m.group(1) == "_" else "super"
        out.append(f'<tspan baseline-shift="{shift}" font-size="72%">{html.escape(m.group(2))}</tspan>')
        i = m.end()
    if s[i:]:
        out.append(f"<tspan>{html.escape(s[i:])}</tspan>")
    return "".join(out)


def _plain_len(s):
    n = 0.0
    i = 0
    for m in _MARK.finditer(s):
        n += len(s[i:m.start()]) + 0.72 * len(m.group(2))
        i = m.end()
    return n + len(s[i:])


def text_size(s, size):
    lines = s.split("\n")
    return max(_plain_len(l) for l in lines) * size * 0.56 + 4, len(lines) * size * 1.25 + 2


def _smooth(pts):
    """SVG path through pts (Catmull-Rom spline as cubic Beziers)."""
    p = [pts[0]] + list(pts) + [pts[-1]]
    d = f"M {pts[0][0]:.1f} {pts[0][1]:.1f}"
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        c1 = (p1[0] + (p2[0] - p0[0]) / 6, p1[1] + (p2[1] - p0[1]) / 6)
        c2 = (p2[0] - (p3[0] - p1[0]) / 6, p2[1] - (p3[1] - p1[1]) / 6)
        d += f" C {c1[0]:.1f} {c1[1]:.1f} {c2[0]:.1f} {c2[1]:.1f} {p2[0]:.1f} {p2[1]:.1f}"
    return d


def rot_px(v, deg):
    """Rotate a screen vector clockwise on screen by deg (draw.io's rotation sense)."""
    t = math.radians(deg)
    return (v[0] * math.cos(t) - v[1] * math.sin(t), v[0] * math.sin(t) + v[1] * math.cos(t))


class Fig:
    def __init__(self, name, title, w, h):
        self.name, self.title, self.w, self.h = name, title, w, h
        self.cells, self.svg, self.colors = [], [], {"#000000"}
        self._id = 1

    def nid(self):
        self._id += 1
        return f"{self.name}-{self._id}"

    # ------------------------------------------------------------ primitives
    def vertex(self, x, y, w, h, style, value="", svg_el=None):
        cid = self.nid()
        self.cells.append(f'<mxCell id="{cid}" value="{html.escape(value, quote=True)}" style="{style}" vertex="1" parent="1">'
                          f'<mxGeometry x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" as="geometry"/></mxCell>')
        if svg_el:
            self.svg.append(svg_el)
        return cid

    def edge(self, pts, style, svg_el=None):
        cid = self.nid()
        (x0, y0), (x1, y1) = pts[0], pts[-1]
        mid = "".join(f'<mxPoint x="{x:.1f}" y="{y:.1f}"/>' for x, y in pts[1:-1])
        arr = f'<Array as="points">{mid}</Array>' if mid else ""
        self.cells.append(f'<mxCell id="{cid}" value="" style="{style}" edge="1" parent="1">'
                          f'<mxGeometry relative="1" as="geometry"><mxPoint x="{x0:.1f}" y="{y0:.1f}" as="sourcePoint"/>'
                          f'<mxPoint x="{x1:.1f}" y="{y1:.1f}" as="targetPoint"/>{arr}</mxGeometry></mxCell>')
        if svg_el:
            self.svg.append(svg_el)
        return cid

    def _mk(self, color, start=False):
        self.colors.add(color)
        return f'url(#{"s" if start else "e"}{color[1:]})'

    # ------------------------------------------------------------ drawing API
    def line(self, p, q, color="#000000", w=1, dash=False, start=False, end=False, label=None, t=0.5,
             lsize=11, lcolor="#000000", curved_pts=None):
        pts = [p] + (curved_pts or []) + [q]
        st = ((f"curved=1;" if curved_pts else "") + f"endArrow={'classic' if end else 'none'};"
              + ("startArrow=classic;" if start else "") + "html=1;rounded=0;"
              + (f"strokeWidth={w};" if w != 1 else "") + (f"strokeColor={color};" if color != "#000000" else "")
              + ("dashed=1;" if dash else ""))
        d = _smooth(pts) if curved_pts else "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in pts)
        mk = (f' marker-end="{self._mk(color)}"' if end else "") + (f' marker-start="{self._mk(color, True)}"' if start else "")
        da = ' stroke-dasharray="6 4"' if dash else ""
        eid = self.edge(pts, st, f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{max(w, 1.2)}"{da}{mk}/>')
        if label is not None:
            val = "&nbsp;" + _rich_html(label).replace("\n", "&nbsp;<br>&nbsp;") + "&nbsp;"
            self.cells.append(f'<mxCell id="{self.nid()}" value="{html.escape(val, quote=True)}" '
                              f'style="edgeLabel;html=1;align=center;verticalAlign=middle;resizable=0;points=[];'
                              f'{"" if lsize == 11 else f"fontSize={lsize};"}{"" if lcolor == "#000000" else f"fontColor={lcolor};"}" '
                              f'vertex="1" connectable="0" parent="{eid}"><mxGeometry x="{2 * t - 1:.3f}" relative="1" as="geometry">'
                              f'<mxPoint as="offset"/></mxGeometry></mxCell>')
            # label position along the (straight) edge for the SVG
            m = (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t) if not curved_pts else pts[len(pts) // 2]
            tw, th = text_size(label, lsize)
            self.svg.append(f'<rect x="{m[0] - tw / 2 - 3:.1f}" y="{m[1] - th / 2:.1f}" width="{tw + 6:.1f}" '
                            f'height="{th:.1f}" fill="#ffffff"/>')
            self._svg_text(m[0], m[1], label, lsize, lcolor, "center", False)
        return eid

    def poly(self, pts, color="#000000", w=1, dash=False):
        """Open polyline (plots)."""
        st = "endArrow=none;html=1;rounded=0;" + (f"strokeWidth={w};" if w != 1 else "") + \
             (f"strokeColor={color};" if color != "#000000" else "") + ("dashed=1;" if dash else "")
        d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in pts)
        self.edge(pts, st, f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{max(w, 1.2)}"'
                           f'{DASH if dash else ""}/>')

    def _svg_text(self, cx, cy, s, size, color, align, bold, italic=False):
        lines = s.split("\n")
        anchor = {"center": "middle", "left": "start", "right": "end"}[align]
        y0 = cy - (len(lines) - 1) * size * 1.25 / 2 + size * 0.35
        body = "".join(f'<tspan x="{cx:.1f}" dy="{0 if i == 0 else size * 1.25:.1f}">{_rich_svg(l)}</tspan>'
                       for i, l in enumerate(lines))
        self.svg.append(f'<text x="{cx:.1f}" y="{y0:.1f}" font-family="{FONT}" font-size="{size}" fill="{color}" '
                        f'text-anchor="{anchor}" font-weight="{"bold" if bold else "normal"}" '
                        f'font-style="{"italic" if italic else "normal"}">{body}</text>')

    def text(self, cx, cy, s, size=12, color="#000000", align="center", bold=False, italic=False, bg=False):
        tw, th = text_size(s, size)
        w, h = tw + 6, len(s.split("\n")) * size * 1.25 + 6
        x = cx - w / 2 if align == "center" else (cx if align == "left" else cx - w)
        fs = (1 if bold else 0) + (2 if italic else 0)
        style = (f"text;html=1;align={align};verticalAlign=middle;whiteSpace=wrap;"
                 + (f"fontSize={size};" if size != 12 else "") + (f"fontColor={color};" if color != "#000000" else "")
                 + (f"fontStyle={fs};" if fs else "") + ("labelBackgroundColor=#ffffff;" if bg else ""))
        if bg:
            bx = cx - tw / 2 if align == "center" else (cx if align == "left" else cx - tw)
            self.svg.append(f'<rect x="{bx:.1f}" y="{cy - th / 2:.1f}" width="{tw:.1f}" height="{th:.1f}" fill="#ffffff"/>')
        self.vertex(x, cy - h / 2, w, h, style, "<br>".join(_rich_html(l) for l in s.split("\n")))
        self._svg_text(cx, cy, s, size, color, align, bold, italic)

    def rect(self, cx, cy, w, h, fill=BLUE_F, stroke=BLUE_S, label="", size=12, rot=0, dashed=False,
             rounded=False, opacity=100, lcolor="#000000", bold=False):
        style = (f"{'rounded=1;' if rounded else ''}whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};"
                 + (f"rotation={rot:.1f};" if rot else "") + ("dashed=1;" if dashed else "")
                 + (f"opacity={opacity};" if opacity != 100 else "") + (f"fontSize={size};" if size != 12 else "")
                 + (f"fontColor={lcolor};" if lcolor != "#000000" else "") + ("fontStyle=1;" if bold else ""))
        op = opacity / 100
        self.vertex(cx - w / 2, cy - h / 2, w, h, style, "<br>".join(_rich_html(l) for l in label.split("\n")),
                    f'<rect x="{cx - w / 2:.1f}" y="{cy - h / 2:.1f}" width="{w:.1f}" height="{h:.1f}" '
                    f'{RX if rounded else ""}fill="{fill}" fill-opacity="{op}" stroke="{stroke}" stroke-width="1.2"'
                    f'{DASH if dashed else ""} transform="rotate({rot:.1f} {cx:.1f} {cy:.1f})"/>')
        if label:
            self._svg_text(cx, cy, label, size, lcolor, "center", bold)

    def ellipse(self, cx, cy, rx, ry, fill=BLUE_F, stroke=BLUE_S, label="", size=12, dashed=False, opacity=100):
        style = (f"ellipse;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};"
                 + ("dashed=1;" if dashed else "") + (f"opacity={opacity};" if opacity != 100 else "")
                 + (f"fontSize={size};" if size != 12 else ""))
        op = opacity / 100
        self.vertex(cx - rx, cy - ry, 2 * rx, 2 * ry, style, _rich_html(label),
                    f'<ellipse cx="{cx:.1f}" cy="{cy:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="{fill}" fill-opacity="{op}" '
                    f'stroke="{stroke}" stroke-width="1.2"{DASH if dashed else ""}/>')
        if label:
            self._svg_text(cx, cy, label, size, "#000000", "center", False)

    def circle(self, cx, cy, r, fill=BLUE_F, stroke=BLUE_S, label="", size=12):
        style = f"ellipse;whiteSpace=wrap;html=1;aspect=fixed;fillColor={fill};strokeColor={stroke};" + \
                (f"fontSize={size};" if size != 12 else "")
        self.vertex(cx - r, cy - r, 2 * r, 2 * r, style, _rich_html(label),
                    f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="1.2"/>')
        if label:
            self._svg_text(cx, cy, label, size, "#000000", "center", False)

    def dot(self, cx, cy, r=3, color="#000000"):
        self.circle(cx, cy, r, fill=color, stroke=color)

    # ------------------------------------------------------------ robot_example symbols
    def cylinder(self, cx, cy, rot=0):
        """Revolute joint whose axis lies in the page (axis = screen-up at rot=0)."""
        style = (f"shape=cylinder3;whiteSpace=wrap;html=1;boundedLbl=1;backgroundOutline=1;size=1.9523809523808495;"
                 f"rotation={rot:.1f};fillColor={BLUE_F};strokeColor={BLUE_S};")
        x, y, e = cx - 10, cy - 10, 2
        self.vertex(x, y, 20, 20, style, "",
                    f'<g transform="rotate({rot:.1f} {cx:.1f} {cy:.1f})" stroke="{BLUE_S}" stroke-width="1" fill="{BLUE_F}">'
                    f'<path d="M {x:.1f} {y + e:.1f} L {x:.1f} {y + 20 - e:.1f} A 10 {e} 0 0 0 {x + 20:.1f} {y + 20 - e:.1f} '
                    f'L {x + 20:.1f} {y + e:.1f}"/><ellipse cx="{cx:.1f}" cy="{y + e:.1f}" rx="10" ry="{e}"/></g>')
        loc = [(-15, -13.5), (-25, -13.5), (-25, -3.5), (25, -3.5), (25, -13.5), (5, -13.5)]
        pts = [(cx + rot_px(p, rot)[0], cy + rot_px(p, rot)[1]) for p in loc]
        sv = [(25 * math.cos(math.radians(220 - 280 * k / 12)), -8.5 + 5 * math.sin(math.radians(220 - 280 * k / 12)))
              for k in range(13)]
        svp = [(cx + rot_px(p, rot)[0], cy + rot_px(p, rot)[1]) for p in sv]
        d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in svp)
        self.edge(pts, "curved=1;endArrow=classic;html=1;rounded=0;",
                  f'<path d="{d}" fill="none" stroke="#000000" stroke-width="1" marker-end="{self._mk("#000000")}"/>')

    def uturn(self, c, sign=1, start_deg=200, radius=17):
        """robot_example's mxgraph.arrows2.uTurnArrow: ~200 deg rotation arrow around c.
        sign>0 counter-clockwise on screen, sign<0 clockwise."""
        cx, cy = c
        base = start_deg if sign > 0 else 180 - start_deg
        r = -base - 95
        off = rot_px((5, -8.5), r + 45)
        if sign < 0:
            r, off = -r, (-off[0], off[1])
        style = ("html=1;shadow=0;dashed=0;align=center;verticalAlign=middle;shape=mxgraph.arrows2.uTurnArrow;dy=0;"
                 f"arrowHead=8.14;dx2=5.84;flipV=0;legacyAnchorPoints=0;flipH={1 if sign > 0 else 0};"
                 f"rotation={r:.1f};fillColor=#000000;")
        pts = [(cx + radius * math.cos(math.radians(start_deg + sign * 200 * i / 12)),
                cy - radius * math.sin(math.radians(start_deg + sign * 200 * i / 12))) for i in range(13)]
        d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in pts)
        self.vertex(cx + off[0] - 10, cy + off[1] - 22.5, 20, 45, style, "",
                    f'<path d="{d}" fill="none" stroke="#000000" stroke-width="1" marker-end="{self._mk("#000000")}"/>')

    def zjoint(self, c, sign=1, start_deg=200, arrow=True):
        """Revolute joint, axis out of (dot) / into (cross) the page, as in the sim_chaser diagram."""
        cx, cy = c
        self.circle(cx, cy, 10)
        if sign > 0:
            self.dot(cx, cy, 2.5)
        else:
            self.line((cx - 5, cy - 5), (cx + 5, cy + 5))
            self.line((cx - 5, cy + 5), (cx + 5, cy - 5))
        if arrow:
            self.uturn(c, sign, start_deg)

    def gripper(self, base, opens_up=False):
        bx, by = base
        if opens_up:
            pieces = [(bx - 10, by - 30, "rotation=-180;"), (bx - 20, by - 30, "flipV=0;legacyAnchorPoints=0;flipH=1;rotation=-180;")]
            poly = [(bx - 20, by), (bx + 20, by), (bx + 20, by - 30), (bx + 10, by - 30), (bx + 10, by - 10),
                    (bx - 10, by - 10), (bx - 10, by - 30), (bx - 20, by - 30)]
        else:
            pieces = [(bx - 20, by, ""), (bx - 10, by, "flipV=0;legacyAnchorPoints=0;flipH=1;")]
            poly = [(bx - 20, by), (bx + 20, by), (bx + 20, by + 30), (bx + 10, by + 30), (bx + 10, by + 10),
                    (bx - 10, by + 10), (bx - 10, by + 30), (bx - 20, by + 30)]
        for i, (x, y, extra) in enumerate(pieces):
            self.vertex(x, y, 30, 30, f"shape=corner;whiteSpace=wrap;html=1;dx=10;dy=10;{extra}fillColor={BLUE_F};strokeColor={BLUE_S};",
                        svg_el=None if i else f'<polygon points="{" ".join(f"{px:.1f},{py:.1f}" for px, py in poly)}" '
                                              f'fill="{BLUE_F}" stroke="{BLUE_S}" stroke-width="1"/>')

    def triad(self, o, name, ax1=("x", 0), ax2=("z", 90), out=None, L=34, size=11, name_off=(-12, 13)):
        """Frame axes: two in-page arrows at screen angles (deg, CCW from right) and an optional
        third axis out of (+1) / into (-1) the page."""
        cols = {"x": RED, "y": GREEN, "z": "#3d6fb6"}
        for lab, ang in (ax1, ax2):
            e = (o[0] + L * math.cos(math.radians(ang)), o[1] - L * math.sin(math.radians(ang)))
            self.line(o, e, color=cols[lab[0]], w=1.5, end=True)
            self.text(o[0] + (L + 8) * math.cos(math.radians(ang)), o[1] - (L + 8) * math.sin(math.radians(ang)),
                      lab, size=size, color=cols[lab[0]])
        if out:
            lab = out[0]
            self.circle(o[0], o[1], 4.5, fill="#ffffff", stroke=cols[lab[0]])
            if out[1] > 0:
                self.dot(o[0], o[1], 1.6, cols[lab[0]])
            else:
                k = 2.8
                self.line((o[0] - k, o[1] - k), (o[0] + k, o[1] + k), color=cols[lab[0]])
                self.line((o[0] - k, o[1] + k), (o[0] + k, o[1] - k), color=cols[lab[0]])
        else:
            self.dot(o[0], o[1], 2.2)
        if name:
            self.text(o[0] + name_off[0], o[1] + name_off[1], name, size=size + 1,
                      align="right" if name_off[0] < 0 else "left", bold=True)

    # ------------------------------------------------------------ output
    def drawio_page(self):
        return (f'<diagram id="{self.name}" name="{html.escape(self.title)}"><mxGraphModel dx="1000" dy="800" grid="1" '
                f'gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" '
                f'pageWidth="{self.w}" pageHeight="{self.h}" math="0" shadow="0"><root><mxCell id="0"/>'
                f'<mxCell id="1" parent="0"/>' + "".join(self.cells) + "</root></mxGraphModel></diagram>")

    def svg_doc(self):
        defs = "".join(
            f'<marker id="e{c[1:]}" markerWidth="10" markerHeight="8" refX="9" refY="4" orient="auto" markerUnits="userSpaceOnUse">'
            f'<path d="M0,0 L10,4 L0,8 z" fill="{c}"/></marker>'
            f'<marker id="s{c[1:]}" markerWidth="10" markerHeight="8" refX="1" refY="4" orient="auto" markerUnits="userSpaceOnUse">'
            f'<path d="M10,0 L0,4 L10,8 z" fill="{c}"/></marker>' for c in sorted(self.colors))
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" viewBox="0 0 {self.w} {self.h}">'
                f'<defs>{defs}</defs><rect width="{self.w}" height="{self.h}" fill="#ffffff"/>' + "".join(self.svg) + "</svg>")


def write(figs, drawio_path, svg_dir):
    import os
    with open(drawio_path, "w") as f:
        f.write('<mxfile host="app.diagrams.net">' + "".join(fg.drawio_page() for fg in figs) + "</mxfile>")
    for fg in figs:
        with open(os.path.join(svg_dir, f"{fg.name}.svg"), "w") as f:
            f.write(fg.svg_doc())
