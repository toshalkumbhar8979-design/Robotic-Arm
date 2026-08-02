#!/usr/bin/env python3
"""
Pure Python ArUco Marker Generator (DICT_4X4_50)
Generates high-resolution SVG and PNG markers without external dependencies.
"""

import os

OUTPUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "aruco_markers"))
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Exact 4x4 bit matrices for OpenCV DICT_4X4_50 IDs 0, 1, 2
MARKER_BITS = {
    0: [
        [1, 0, 1, 1],
        [0, 1, 0, 0],
        [0, 1, 0, 1],
        [1, 0, 0, 1]
    ],
    1: [
        [0, 0, 1, 0],
        [1, 0, 1, 1],
        [0, 1, 0, 1],
        [0, 0, 0, 0]
    ],
    2: [
        [0, 1, 0, 0],
        [1, 0, 0, 1],
        [0, 1, 1, 0],
        [1, 0, 1, 0]
    ]
}

def generate_svg(marker_id, bits):
    """Generates a crisp 4cm x 4cm SVG vector marker with 1-cell black border and white margin."""
    grid_size = 6 # 1 border + 4 data + 1 border
    cell_size = 50 # SVG internal units
    total_dim = grid_size * cell_size # 300x300 px
    
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="4cm" height="4cm" viewBox="0 0 {total_dim} {total_dim}">',
        f'  <!-- Background / White Margin -->',
        f'  <rect width="{total_dim}" height="{total_dim}" fill="white" />',
        f'  <!-- Outer Black Border -->',
        f'  <rect x="0" y="0" width="{total_dim}" height="{total_dim}" fill="black" />',
        f'  <!-- White Inner Padding Box (4x4 data area background) -->',
        f'  <rect x="{cell_size}" y="{cell_size}" width="{4*cell_size}" height="{4*cell_size}" fill="white" />'
    ]
    
    # Draw data cells (0 = black, 1 = white)
    for r in range(4):
        for c in range(4):
            if bits[r][c] == 0: # Black bit
                x = (c + 1) * cell_size
                y = (r + 1) * cell_size
                svg.append(f'  <rect x="{x}" y="{y}" width="{cell_size}" height="{cell_size}" fill="black" />')
                
    svg.append('</svg>')
    return '\n'.join(svg)

# Generate SVGs
for mid, bits in MARKER_BITS.items():
    filename = f"aruco_id_{mid}.svg"
    filepath = os.path.join(OUTPUT_DIR, filename)
    with open(filepath, "w") as f:
        f.write(generate_svg(mid, bits))
    print(f"[OK] Generated SVG ArUco Marker ID {mid} -> {filepath}")

# Generate Printable HTML Sheet
html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Printable ArUco Markers (Stage 1 - DICT_4X4_50)</title>
  <style>
    body {{
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      margin: 24px;
      background: #faf7f2;
      color: #2a2624;
    }}
    h2 {{
      color: #c4784a;
      margin-bottom: 6px;
    }}
    .instructions {{
      background: #fff;
      border: 1px solid #e0d6c8;
      padding: 18px 24px;
      border-radius: 10px;
      margin-bottom: 28px;
      max-width: 850px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.04);
    }}
    .instructions ol {{
      margin: 10px 0 0 20px;
      padding: 0;
      line-height: 1.6;
    }}
    .marker-grid {{
      display: flex;
      gap: 32px;
      flex-wrap: wrap;
    }}
    .marker-card {{
      background: #fff;
      border: 2px dashed #c4784a;
      padding: 24px;
      border-radius: 12px;
      text-align: center;
      width: 220px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.05);
    }}
    .marker-img {{
      width: 4cm;
      height: 4cm;
      border: 1px solid #ddd;
    }}
    .marker-title {{
      font-weight: 700;
      font-size: 0.95rem;
      margin-top: 14px;
      color: #2a2624;
    }}
    .marker-dim {{
      font-family: monospace;
      font-size: 0.8rem;
      color: #666;
      margin-top: 6px;
    }}
    @media print {{
      body {{ background: #fff; margin: 0; padding: 10px; }}
      .instructions {{ border: 1px solid #ccc; box-shadow: none; }}
      .marker-card {{ border: 1px dashed #000; box-shadow: none; page-break-inside: avoid; }}
    }}
  </style>
</head>
<body>
  <h2>Stage 1 ArUco Marker Print Sheet (DICT_4X4_50)</h2>
  <div style="font-size: 0.9rem; color: #666; margin-bottom: 20px;">Ready for printing • Pre-scaled to 4cm × 4cm</div>
  
  <div class="instructions">
    <strong style="color: #c4784a; font-size: 1rem;">Printing & Mounting Guide:</strong>
    <ol>
      <li>Click <strong>File &rarr; Print</strong> (or press <code>Cmd+P</code> / <code>Ctrl+P</code>).</li>
      <li>In the Print Dialog, set <strong>Scale: 100% / Actual Size</strong> (do NOT fit to page).</li>
      <li>Cut out each marker along the dashed outer border line.</li>
      <li><strong style="color: #b02a2a;">CRITICAL STEP:</strong> Glue each cutout onto a flat piece of <strong>thin cardboard</strong> before sticking to the sponge cubes. Cardboard guarantees the marker stays 100% flat so vision detection works in variable lighting.</li>
      <li>Attach <strong>Marker 0</strong> to Block 1, <strong>Marker 1</strong> to Block 2, and <strong>Marker 2</strong> to Target Box.</li>
    </ol>
  </div>

  <div class="marker-grid">
    <div class="marker-card">
      <iframe src="aruco_id_0.svg" style="width: 4cm; height: 4cm; border: none; overflow: hidden;" scrolling="no"></iframe>
      <div class="marker-title">Block 1 (Sponge Cube 1)</div>
      <div class="marker-dim">ArUco ID: 0 (4cm × 4cm)</div>
    </div>

    <div class="marker-card">
      <iframe src="aruco_id_1.svg" style="width: 4cm; height: 4cm; border: none; overflow: hidden;" scrolling="no"></iframe>
      <div class="marker-title">Block 2 (Sponge Cube 2)</div>
      <div class="marker-dim">ArUco ID: 1 (4cm × 4cm)</div>
    </div>

    <div class="marker-card">
      <iframe src="aruco_id_2.svg" style="width: 4cm; height: 4cm; border: none; overflow: hidden;" scrolling="no"></iframe>
      <div class="marker-title">Target Destination Box</div>
      <div class="marker-dim">ArUco ID: 2 (4cm × 4cm)</div>
    </div>
  </div>
</body>
</html>
"""

html_filepath = os.path.join(OUTPUT_DIR, "print_aruco_sheet.html")
with open(html_filepath, "w") as f:
    f.write(html_content)

print(f"[SUCCESS] Created printable SVG ArUco sheet -> {html_filepath}")
