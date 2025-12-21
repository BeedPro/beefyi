#!/bin/bash
set -e

npm install
npm run build-pages

# Switch to pages branch (create if missing)
if git show-ref --quiet refs/heads/pages; then
  git checkout pages
else
  git switch --orphan pages
  git rm -rf .
fi

# Ensure Pages never uses LFS
rm -f .gitattributes

# Clear old content
rm -rf *

# Copy built site
cp -r _site/* .
cp _site/.domains .

# Clean up
rm -rf _site node_modules

git add -A
git commit -m "Deploy pages"
git push -u origin pages

git checkout main
