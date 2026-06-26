"""Generate AutoMech brand assets from the source ChatGPT logo image.

Produces, in public/:
  - automech-logo.png  : full lockup, white background made transparent, trimmed
  - automech-icon.png  : left gear-"A" mark only, white made transparent, square-ish trim
  - automech-icon.svg  : SVG wrapper embedding automech-icon.png (base64)
  - automech-icon.ico  : favicon (16/32/48) from the cropped mark

Run from repo root:  python scripts/make_brand_assets.py
"""

import base64
import glob
import os
import sys

from PIL import Image

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PUBLIC = os.path.join(ROOT, "public")

# White threshold: pixels at/above this in all RGB channels become transparent.
WHITE_CUTOFF = 245


def find_source() -> str:
    matches = glob.glob(os.path.join(ROOT, "ChatGPT Image*22_51_19.png"))
    if not matches:
        matches = glob.glob(os.path.join(ROOT, "ChatGPT Image*.png"))
    if not matches:
        raise SystemExit("Source logo image not found in repo root.")
    return matches[0]


def whiten_to_transparent(img: Image.Image) -> Image.Image:
    """Make near-white pixels fully transparent; leave everything else intact."""
    img = img.convert("RGBA")
    px = img.load()
    w, h = img.size
    for y in range(h):
        for x in range(w):
            r, g, b, a = px[x, y]
            if r >= WHITE_CUTOFF and g >= WHITE_CUTOFF and b >= WHITE_CUTOFF:
                px[x, y] = (r, g, b, 0)
    return img


def trim(img: Image.Image) -> Image.Image:
    bbox = img.getbbox()  # bbox of non-zero-alpha region after transparency pass
    return img.crop(bbox) if bbox else img


def main() -> None:
    src_path = find_source()
    print(f"source: {src_path}")
    src = Image.open(src_path).convert("RGBA")
    w, h = src.size
    print(f"source size: {w}x{h}")

    # --- full wide logo ---
    logo = trim(whiten_to_transparent(src.copy()))
    logo_path = os.path.join(PUBLIC, "automech-logo.png")
    logo.save(logo_path)
    print(f"wrote {logo_path} {logo.size}")

    # --- icon: crop the gear mark on the left, then transparency + trim ---
    # The "AutoMech." wordmark starts after the gear glyph. Empirically the mark
    # occupies roughly the left 28% of the full-image width; crop generously then
    # let trim() tighten to the actual glyph bounds.
    crop_w = int(w * 0.30)
    mark = src.crop((0, 0, crop_w, h))
    mark = trim(whiten_to_transparent(mark))

    # Pad to a square canvas so it scales cleanly into square icon slots.
    mw, mh = mark.size
    side = max(mw, mh)
    square = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    square.paste(mark, ((side - mw) // 2, (side - mh) // 2))

    icon_png_path = os.path.join(PUBLIC, "automech-icon.png")
    square.save(icon_png_path)
    print(f"wrote {icon_png_path} {square.size}")

    # --- favicon .ico from the square mark ---
    ico_path = os.path.join(PUBLIC, "automech-icon.ico")
    square.save(ico_path, format="ICO", sizes=[(16, 16), (32, 32), (48, 48)])
    print(f"wrote {ico_path}")

    # --- svg wrapper embedding the icon png ---
    with open(icon_png_path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'viewBox="0 0 {side} {side}" width="{side}" height="{side}">'
        f'<image width="{side}" height="{side}" '
        f'href="data:image/png;base64,{b64}"/></svg>'
    )
    svg_path = os.path.join(PUBLIC, "automech-icon.svg")
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(svg)
    print(f"wrote {svg_path}")


if __name__ == "__main__":
    main()
