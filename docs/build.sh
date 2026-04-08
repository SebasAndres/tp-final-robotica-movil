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

echo "  HTML → $html"

# HTML → PDF via headless Chrome so Mermaid and images are already rendered
google-chrome --headless=new --disable-gpu --no-sandbox \
  --no-pdf-header-footer \
  --print-to-pdf="$pdf" \
  "file://$html" 2>/dev/null

echo "  PDF  → $pdf"
