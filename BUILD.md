# Local Build

This project is an [Eleventy](https://www.11ty.dev/) site.

## Requirements

- Node.js 20 locally (`.nvmrc`)
- npm

CI runs Node 18, so keep changes compatible with Node 18+.

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
