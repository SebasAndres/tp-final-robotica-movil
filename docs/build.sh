#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

out_dir="$SCRIPT_DIR/$next"
mkdir -p "$out_dir"

html="$out_dir/informe.html"
pdf="$out_dir/informe.pdf"

echo "Building $next..."

pandoc "$SCRIPT_DIR/informe.md" -o "$html" \
  --katex --embed-resources --standalone \
  --lua-filter "$SCRIPT_DIR/mermaid-filter.lua" \
  --include-in-header "$SCRIPT_DIR/mermaid-header.html" \
  --css "$SCRIPT_DIR/style.css" \
  --resource-path "$SCRIPT_DIR"

# --embed-resources embeds iframe srcs as data: URIs (fetches Drive viewer with
# session tokens that expire). Restore the original /preview URLs from the markdown.
python3 - "$SCRIPT_DIR/informe.md" "$html" <<'PYEOF'
import re, sys

md_path, html_path = sys.argv[1], sys.argv[2]

with open(md_path) as f:
    md = f.read()

# Extract Drive file IDs in order from the markdown iframes
ids = re.findall(r'drive\.google\.com/file/d/([^/]+)/preview', md)

with open(html_path) as f:
    html = f.read()

count = [0]
def fix(m):
    if count[0] >= len(ids):
        return m.group(0)
    fid = ids[count[0]]
    count[0] += 1
    return (f'<iframe role="img" class="video-demo" '
            f'src="https://drive.google.com/file/d/{fid}/preview" '
            f'width="640" height="360" allow="autoplay">')

new_html = re.sub(
    r'<iframe role="img" class="video-demo" src="data:text/html[^"]*"[^>]*>',
    fix, html)

with open(html_path, 'w') as f:
    f.write(new_html)

if count[0]:
    print(f"  Fixed {count[0]} Drive iframe(s)")
PYEOF

echo "  HTML → $html"

# HTML → PDF via headless Chrome so Mermaid and images are already rendered
google-chrome --headless=new --disable-gpu --no-sandbox \
  --no-pdf-header-footer \
  --print-to-pdf="$pdf" \
  "file://$html" 2>/dev/null

echo "  PDF  → $pdf"
