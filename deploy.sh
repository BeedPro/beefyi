#!/bin/bash
set -e

npm install
npm run build-pages

# Switch to pages branch (create if missing)
if git show-ref --quiet refs/heads/pages; then
  git checkout pages
else
  git switch --orphan pages
fi

# Remove all tracked files from pages (safe)
git rm -rf . || true

# Ensure Pages never uses LFS
rm -f .gitattributes

# Copy built site
cp -r _site/* .
cp _site/.domains .

# Clean up build artefacts
rm -rf _site node_modules

git add -A
git commit -m "Deploy pages"
git push -u origin pages

git checkout main
