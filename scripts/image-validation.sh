#!/bin/bash

# Image Validation Script
# Checks that all images in static/img/ are under 200KB

set -e

MAX_SIZE_KB=200
MAX_SIZE_BYTES=$((MAX_SIZE_KB * 1024))
FAILED=0
CHECKED=0

echo "🖼️  Validating images in static/img/"
echo "Maximum allowed size: ${MAX_SIZE_KB}KB"
echo ""

# Find all image files
while IFS= read -r -d '' file; do
    CHECKED=$((CHECKED + 1))
    SIZE=$(stat -c%s "$file" 2>/dev/null || stat -f%z "$file" 2>/dev/null)
    SIZE_KB=$((SIZE / 1024))

    if [ "$SIZE" -gt "$MAX_SIZE_BYTES" ]; then
        echo "❌ FAIL: $file (${SIZE_KB}KB > ${MAX_SIZE_KB}KB)"
        FAILED=$((FAILED + 1))
    else
        echo "✅ PASS: $file (${SIZE_KB}KB)"
    fi
done < <(find static/img -type f \( -name "*.png" -o -name "*.jpg" -o -name "*.jpeg" -o -name "*.gif" -o -name "*.webp" \) -print0 2>/dev/null)

echo ""
echo "Validation Summary:"
echo "  Total images checked: $CHECKED"
echo "  Passed: $((CHECKED - FAILED))"
echo "  Failed: $FAILED"

if [ "$FAILED" -gt 0 ]; then
    echo ""
    echo "❌ Image validation failed! Please optimize images over ${MAX_SIZE_KB}KB."
    exit 1
fi

echo ""
echo "✅ All images are within size limits!"
exit 0
