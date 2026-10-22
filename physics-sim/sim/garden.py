"""A "mystical garden" scene for watch.py --garden: looks only, no physics.

garden_xml(xml) takes a built bunny model and adds a twilight sky, mossy ground, a stone
path with an archway of glowing blossoms, mushrooms, flowers, hedges and floating
fireflies. Every decoration has contype=0 / conaffinity=0, so nothing collides with the
robot, and the floor keeps its geometry and friction (only its material changes), so a
run in the garden is the same run as on the plain grid. The bunny can walk through a
flower if it wanders off the path.

The path runs along +x from the start, where the bunny faces; the arch is ARCH_X ahead.
"""

import math
import re

import numpy as np

ARCH_X = 0.9  # m ahead of the start
PATH_HALF_WIDTH = 0.35
CLEAR_HALF_WIDTH = 0.55  # no decorations inside this, either side of the path
SEED = 7

ASSETS = """
    <texture name="garden_sky" type="skybox" builtin="gradient" rgb1="0.22 0.12 0.38" rgb2="0.02 0.03 0.08"
             mark="random" random="0.004" markrgb="0.95 0.90 1.00" width="1024" height="1024"/>
    <texture name="garden_moss" type="2d" builtin="flat" rgb1="0.11 0.22 0.13"
             mark="random" random="0.12" markrgb="0.26 0.42 0.22" width="512" height="512"/>
    <material name="garden_grass" texture="garden_moss" texrepeat="40 40" specular="0.05" shininess="0.05"/>
    <material name="garden_stone" rgba="0.55 0.52 0.58 1" specular="0.1"/>
    <material name="garden_path" rgba="0.46 0.38 0.30 1" specular="0.05"/>
    <material name="garden_stem" rgba="0.20 0.42 0.22 1"/>
    <material name="garden_hedge" rgba="0.09 0.24 0.13 1" specular="0.08"/>
    <material name="garden_vine" rgba="0.16 0.38 0.20 1" specular="0.1" emission="0.9"/>
    <material name="garden_bark" rgba="0.25 0.17 0.12 1"/>
    <material name="garden_cream" rgba="0.92 0.88 0.78 1"/>
    <material name="glow_violet" rgba="0.70 0.45 1.00 1" emission="0.8"/>
    <material name="glow_pink" rgba="1.00 0.55 0.80 1" emission="0.7"/>
    <material name="glow_teal" rgba="0.35 1.00 0.85 1" emission="0.8"/>
    <material name="glow_gold" rgba="1.00 0.85 0.40 1" emission="1.0"/>
    <material name="cap_red" rgba="0.80 0.18 0.25 1" emission="0.15"/>
    <material name="cap_violet" rgba="0.50 0.25 0.75 1" emission="0.25"/>"""

VISUAL = """
    <headlight ambient="0.30 0.27 0.38" diffuse="0.40 0.38 0.45" specular="0.05 0.05 0.05"/>
    <quality shadowsize="4096"/>
"""

GLOWS = ("glow_violet", "glow_pink", "glow_teal", "glow_gold")


def deco(geom):
    """A geom string with collisions off."""
    return geom.replace("<geom ", '<geom contype="0" conaffinity="0" group="1" ', 1)


def fmt(*v):
    return " ".join(f"{x:.4g}" for x in v)


def mushroom(x, y, s, cap):
    return [deco(f'<geom type="cylinder" pos="{fmt(x, y, 0.03 * s)}" size="{fmt(0.008 * s, 0.03 * s)}" material="garden_cream"/>'),
            deco(f'<geom type="ellipsoid" pos="{fmt(x, y, 0.062 * s)}" size="{fmt(0.03 * s, 0.03 * s, 0.016 * s)}" material="{cap}"/>')]


def flower(x, y, h, glow):
    return [deco(f'<geom type="capsule" fromto="{fmt(x, y, 0, x, y, h)}" size="0.003" material="garden_stem"/>'),
            deco(f'<geom type="sphere" pos="{fmt(x, y, h)}" size="0.014" material="{glow}"/>')]


def scatter(rng, n, x_range, y_range):
    """Points beside the path (|y| >= CLEAR_HALF_WIDTH), either side."""
    xs = rng.uniform(*x_range, n)
    ys = rng.uniform(*y_range, n) * rng.choice([-1, 1], n)
    return zip(xs, ys)


def decorations():
    rng = np.random.default_rng(SEED)
    g = []
    # The path: thin, just above the floor (the feet still stand on the real floor below it)
    g.append(deco(f'<geom type="box" pos="{fmt(4.0, 0, 0.0008)}" size="{fmt(5.0, PATH_HALF_WIDTH, 0.0008)}" material="garden_path"/>'))
    # Border stones
    for x in np.arange(-0.8, 9.0, 0.22):
        for side in (-1, 1):
            y = side * (PATH_HALF_WIDTH + 0.04 + rng.uniform(0, 0.03))
            r = rng.uniform(0.025, 0.04)
            g.append(deco(f'<geom type="ellipsoid" pos="{fmt(x + rng.uniform(-0.04, 0.04), y, 0.008)}" '
                          f'size="{fmt(r, r * 0.8, 0.014)}" material="garden_stone"/>'))
    # Archway of glowing blossoms, with two vine posts
    radius, top = PATH_HALF_WIDTH + 0.15, 0.75
    arc = [(side * radius, z) for side in (-1, 1) for z in np.linspace(0, top - radius, 10)]
    arc += [(radius * math.cos(a), top - radius + radius * math.sin(a)) for a in np.linspace(0, math.pi, 22)]
    for side in (-1, 1):
        g.append(deco(f'<geom type="capsule" fromto="{fmt(ARCH_X, side * radius, 0, ARCH_X, side * radius, top - radius)}" '
                      f'size="0.01" material="garden_bark"/>'))
    for i, (y, z) in enumerate(arc):
        g.append(deco(f'<geom type="sphere" pos="{fmt(ARCH_X + rng.uniform(-0.015, 0.015), y, z)}" '
                      f'size="{rng.uniform(0.016, 0.024):.3g}" material="garden_vine"/>'))
        g.append(deco(f'<geom type="sphere" pos="{fmt(ARCH_X + rng.uniform(-0.03, 0.03), y + rng.uniform(-0.02, 0.02), z + 0.015)}" '
                      f'size="{rng.uniform(0.011, 0.017):.3g}" material="{GLOWS[i % len(GLOWS)]}"/>'))
    # Flowers and mushrooms beside the path
    for x, y in scatter(rng, 140, (-1.5, 9.0), (CLEAR_HALF_WIDTH, 2.2)):
        g += flower(x, y, rng.uniform(0.05, 0.16), GLOWS[rng.integers(len(GLOWS))])
    for x, y in scatter(rng, 45, (-1.5, 9.0), (CLEAR_HALF_WIDTH + 0.05, 2.5)):
        g += mushroom(x, y, rng.uniform(0.7, 2.2), "cap_red" if rng.random() < 0.5 else "cap_violet")
    # Hedges and round trees further out
    for x, y in scatter(rng, 26, (-2.0, 10.0), (1.4, 3.5)):
        r = rng.uniform(0.18, 0.4)
        g.append(deco(f'<geom type="ellipsoid" pos="{fmt(x, y, r * 0.7)}" size="{fmt(r * 1.3, r, r * 0.9)}" material="garden_hedge"/>'))
    for x, y in scatter(rng, 10, (-2.0, 10.0), (2.2, 4.5)):
        h = rng.uniform(0.7, 1.3)
        g.append(deco(f'<geom type="capsule" fromto="{fmt(x, y, 0, x, y, h)}" size="0.04" material="garden_bark"/>'))
        g.append(deco(f'<geom type="sphere" pos="{fmt(x, y, h + 0.25)}" size="{rng.uniform(0.3, 0.45):.3g}" material="garden_hedge"/>'))
    # Fireflies floating over the garden
    for x, y in scatter(rng, 70, (-1.0, 9.0), (0.2, 2.5)):
        g.append(deco(f'<geom type="sphere" pos="{fmt(x, y, rng.uniform(0.15, 0.8))}" size="0.005" material="glow_gold"/>'))
    return "\n    ".join(g)


LIGHTS = f"""
    <light name="garden_moon" directional="true" pos="0 0 4" dir="0 0.25 -1" diffuse="0.45 0.45 0.60" specular="0.1 0.1 0.1" castshadow="false"/>
    <light name="garden_teal_fill" pos="3 -1.5 1.2" dir="-0.3 0.5 -0.6" diffuse="0.15 0.35 0.32" castshadow="false"/>"""


# A soft spotlight that follows the bunny: its shadow stays crisp (the shadow map only covers
# the spot, not the whole garden) and it reads like a stage light on the showcase floor.
SPOTLIGHT = """
      <light name="garden_spot" mode="trackcom" pos="0.3 0.4 1.6" dir="-0.15 -0.25 -1" cutoff="28" exponent="4"
             diffuse="0.85 0.80 0.95" specular="0.2 0.2 0.2" castshadow="true"/>"""


def garden_xml(xml):
    """The same model, dressed as a garden."""
    xml = xml.replace("</asset>", ASSETS + "\n  </asset>", 1)
    xml = re.sub(r"(<visual>.*?)(\s*</visual>)", lambda m: m.group(1) + VISUAL + m.group(2), xml, count=1, flags=re.S)
    xml = re.sub(r'<light pos="0 0 3"[^>]*/>', LIGHTS, xml, count=1)
    xml = re.sub(r'(<geom name="floor"[^>]*?)material="grid"', r'\1material="garden_grass"', xml, count=1)
    xml = xml.replace("<worldbody>", "<worldbody>\n    " + decorations(), 1)
    xml = re.sub(r'(<body name="torso"[^>]*>)', lambda m: m.group(1) + SPOTLIGHT, xml, count=1)
    return xml
