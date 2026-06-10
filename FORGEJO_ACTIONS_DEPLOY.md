# Forgejo Actions Deployment

This repository deploys to Codeberg Pages using Forgejo Actions.

## How it works

The workflow file is:

```text
.forgejo/workflows/pages.yml
```

On every push to `main`, Forgejo Actions:

1. Checks out the repository.
2. Sets up Node.js 18.
3. Runs `npm ci`.
4. Runs `npm run build-pages`.
5. Replaces the contents of the `pages` branch with `_site/`.
6. Force-pushes the `pages` branch.

Codeberg Pages then serves the published `pages` branch.

## Triggering a deployment

Push your source changes to `main`:

```bash
git push origin main
```

That is enough to trigger the workflow.

## Requirements on Codeberg

- The repository must have Forgejo Actions enabled.
- Codeberg Pages must be configured to publish from the `pages` branch.
- If using the `codeberg.page` flow, add the appropriate webhook in Codeberg.
- If using a custom domain, keep a `.domains` file at the published site root.

This repository already keeps the custom domain file in:

```text
public/.domains
```

The workflow preserves it by copying `_site/.domains` into the `pages` branch when present.

## URL configuration

If the site URL changes, update the hard-coded site URL values in:

- `_data/metadata.js`
- `eleventy.config.js`

If you deploy under a repository subpath such as `https://username.codeberg.page/repository-name/`, you may also need to set `pathPrefix` in `eleventy.config.js`.

## Notes

- Local development uses Node 20, but the workflow uses Node 18.
- The deployment step force-pushes `pages`, so treat that branch as generated output.
- Draft posts are excluded only during build mode.
