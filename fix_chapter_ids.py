#!/usr/bin/env python3
"""Fix chapter IDs to remove slashes (Docusaurus requirement)"""

import re
from pathlib import Path

DOCS_DIR = Path("/Users/noori/docusorus_project/hackathon1_book/my-website/docs")

def fix_chapter_id(file_path):
    """Remove path prefix from document ID"""
    with open(file_path, 'r') as f:
        content = f.read()

    # Match the frontmatter id line with slashes
    pattern = r'^id: (.*/)?(.+)$'

    def replace_id(match):
        line = match.group(0)
        # Extract just the filename part after last slash
        if '/' in line:
            parts = line.split(': ', 1)[1]  # Get part after "id: "
            simple_id = parts.split('/')[-1]  # Get part after last slash
            return f'id: {simple_id}'
        return line

    new_content = re.sub(pattern, replace_id, content, count=1, flags=re.MULTILINE)

    if new_content != content:
        with open(file_path, 'w') as f:
            f.write(new_content)
        return True
    return False

def main():
    fixed = []

    # Find all markdown files in week-* subdirectories
    for md_file in DOCS_DIR.rglob("week-*/*.md"):
        if fix_chapter_id(md_file):
            fixed.append(str(md_file.relative_to(DOCS_DIR)))

    print(f"Fixed {len(fixed)} files:")
    for f in sorted(fixed):
        print(f"  ✓ {f}")

if __name__ == "__main__":
    main()
