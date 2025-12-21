#!/bin/bash

npm i
npm run build-pages

git switch --orphan pages || git checkout pages

git rm --cached -r . && git add -A && git commit -m "Remove pages content"

rm -rf node_modules

cp -r _site/* .
cp _site/.domains .
rm -rf _site

git add -A && git commit -m "Deploy pages"
git push -u origin pages
