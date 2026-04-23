"""
Tokyo Night matplotlib palette, matching docs/index.html.

Exports two apply_* functions — one for dark-mode charts (default),
one for light-mode charts — that mutate rcParams in place. Call one
before creating any figure, then reset afterwards if needed by
calling the other.
"""

import matplotlib as mpl


# Palette snapshot from docs/index.html :root[data-theme="dark"]
DARK = {
    'bg':             '#1a1b26',
    'bg_elevated':    '#1f2030',
    'fg':             '#c0caf5',
    'fg_muted':       '#a9b1d6',
    'fg_dim':         '#7a83a8',
    'fg_subtle':      '#565f89',
    'grid':           '#2b2d3f',
    'prism_accent':   '#bb9af7',   # violet — Prism side
    'stock_accent':   '#7dcfff',   # cyan  — stock / legacy side
    'muted':          '#565f89',
    'success':        '#9ece6a',
    'danger':         '#f7768e',
}

LIGHT = {
    'bg':             '#fbfbfd',
    'bg_elevated':    '#ffffff',
    'fg':             '#1a1b26',
    'fg_muted':       '#3b425f',
    'fg_dim':         '#5e6585',
    'fg_subtle':      '#8b91ac',
    'grid':           '#e6e7ef',
    'prism_accent':   '#5a3bb5',
    'stock_accent':   '#0b7d9e',
    'muted':          '#8b91ac',
    'success':        '#2d7a3a',
    'danger':         '#a0344a',
}


def _apply(p):
    mpl.rcParams.update({
        'figure.facecolor':     'none',
        'axes.facecolor':       'none',
        'savefig.facecolor':    'none',
        'savefig.transparent':  True,
        'axes.edgecolor':       p['fg_dim'],
        'axes.labelcolor':      p['fg'],
        'axes.titlecolor':      p['fg'],
        'axes.grid':            True,
        'axes.axisbelow':       True,
        'grid.color':           p['grid'],
        'grid.linewidth':       0.8,
        'grid.linestyle':       '-',
        'grid.alpha':           0.9,
        'xtick.color':          p['fg_muted'],
        'ytick.color':          p['fg_muted'],
        'xtick.labelcolor':     p['fg_muted'],
        'ytick.labelcolor':     p['fg_muted'],
        'font.family':          ['Inter', 'DejaVu Sans', 'sans-serif'],
        'font.size':            11,
        'legend.frameon':       False,
        'legend.labelcolor':    p['fg'],
        'axes.spines.top':      False,
        'axes.spines.right':    False,
        'axes.spines.left':     True,
        'axes.spines.bottom':   True,
        'axes.prop_cycle':      mpl.cycler(color=[p['prism_accent'], p['stock_accent']]),
        'lines.linewidth':      1.6,
        'patch.linewidth':      0.8,
    })


def apply_dark():
    _apply(DARK)
    return DARK


def apply_light():
    _apply(LIGHT)
    return LIGHT
