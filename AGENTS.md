# AGENTS

## Repo essentials
- Stack: Eleventy 3 site (ESM, `type: module`) with Nunjucks/Markdown templates.
- Node: use Node 20 locally (`.nvmrc`), but CI runs Node 18 (`.forgejo/workflows/pages.yml`); keep changes compatible with Node 18+.
- Indentation/style baseline is tabs (`.editorconfig`).

## Source of truth layout
- Eleventy input is `content/`; output is `_site/` (`eleventy.config.js`).
- Shared templates are in `_includes/`; global data is in `_data/`; custom filters are in `_config/filters.js`.
- Blog posts live in `content/posts/`; `content/posts/posts.11tydata.js` auto-applies `tags: ["posts"]` and `layout: "layouts/post.njk"`.
- Static files under `public/` are passthrough-copied to site root.

## Commands you actually need
- Install deps: `npm ci`
- Local dev server: `npm start` (Eleventy `--serve --quiet`)
- Production build: `npm run build` (same generator command as `build-pages`)
- Debug build: `npm run debug`
- Benchmark timing: `npm run benchmark`

## Verified quirks and gotchas
- Draft handling is build-only: pages with `draft: true` are excluded only when `ELEVENTY_RUN_MODE === "build"`; they still appear during serve/dev.
- Front matter validation exists in `_data/eleventyDataSchema.js`; malformed `draft` values fail the build.
- CSS/JS are bundled from template `<style>`/`<script>` blocks via Eleventy bundles in `eleventy.config.js`; do not assume a separate bundler.
- Markdown supports GitHub-style alerts (`[!NOTE]`, `[!TIP]`, etc.) via `markdown-it-github-alerts`.

## Deployment facts that affect edits
- Auto deploy is Forgejo Actions on push to `main`: `npm ci` -> `npm run build-pages` -> force-push generated output to `pages`.
- Keep custom domain file behavior intact: workflow copies `_site/.domains` when present.
- `deploy.sh` performs a force push to `pages` and deletes the local `pages` branch afterward; treat it as destructive.

## Lint/test reality
- There is an ESLint config (`eslint.config.js`) but no npm `lint`/`test` scripts.
- If you need a focused lint check, run `npx eslint "**/*.{js,mjs,cjs}"` from repo root.
