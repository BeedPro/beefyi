# Development

This project is an [Eleventy](https://www.11ty.dev/) site.

## Requirements

- Node.js 20 locally (`.nvmrc`)
- npm

CI currently runs Node 18, so keep changes compatible with Node 18+.

## Install dependencies

```bash
npm ci
```

## Start the local dev server

```bash
npm start
```

This runs Eleventy in serve mode and rebuilds on file changes.

## Run a production build locally

```bash
npm run build
```

The generated site output is written to `_site/`.

## Manual deployment without Forgejo Actions

Build the site first:

```bash
npm run build-pages
```

Then publish the generated `_site/` contents to the `pages` branch:

```bash
git fetch origin pages:pages
git worktree add /tmp/beefyi-pages pages
git -C /tmp/beefyi-pages rm -rf .
git -C /tmp/beefyi-pages clean -fdx
cp -a _site/. /tmp/beefyi-pages/
git -C /tmp/beefyi-pages add -A
git -C /tmp/beefyi-pages commit -m "Deploy pages"
git -C /tmp/beefyi-pages push ssh://git@codeberg.org/Beed/beefyi.git pages:pages
```

If `public/.domains` exists, Eleventy copies it to `_site/.domains`, and the deployment keeps it at the root of the `pages` branch for the custom domain configuration.

## URL configuration

If the site URL changes, update the hard-coded site URL values in:

- `_data/metadata.js`
- `eleventy.config.js`

If you deploy under a repository subpath such as `https://username.codeberg.page/repository-name/`, you may also need to set `pathPrefix` in `eleventy.config.js`.

## Useful commands

```bash
npm run debug
npm run benchmark
```

## Project layout

- Source content: `content/`
- Shared templates: `_includes/`
- Global data: `_data/`
- Static passthrough files: `public/`
- Generated output: `_site/`
