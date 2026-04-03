# beefyi

This is the source code for my blog beefyi, Beed For Your Info. Written with [11ty](https://www.11ty.dev/) using the [eleventy-base-blog](https://github.com/11ty/eleventy-base-blog) starter project. This project also adds LaTeX support via MathJax using the [eleventy-plugin-mathjax](https://github.com/tsung-ju/eleventy-plugin-mathjax).

Live site: [blog.beed.org.uk](https://blog.beed.org.uk/)

## Repository mirrors

- Primary repository: [Codeberg (Beed/beefyi)](https://codeberg.org/Beed/beefyi)
- Backup mirror: [GitHub (BeedPro/beefyi)](https://github.com/BeedPro/beefyi)

The GitHub repo is the backup mirror of the Codeberg repo, and both repositories should point to each other in docs/reference links.

## Git remote setup (push to both)

Use this setup to keep Codeberg as the primary remote while pushing to both remotes with a single `git push origin <branch>` command.

```bash
git remote add origin ssh://git@codeberg.org/Beed/beefyi.git
git remote add backup git@github.com:BeedPro/beefyi.git

# Push to Codeberg and GitHub when pushing origin
git remote set-url --push origin ssh://git@codeberg.org/Beed/beefyi.git
git remote set-url --add --push origin git@github.com:BeedPro/beefyi.git
```

Verify push targets:

```bash
git remote get-url --push --all origin
```

## Deployment (Codeberg Pages)

This site is deployed to Codeberg Pages from the `pages` branch.

### Automatic deployment (recommended)

- Workflow file: `.forgejo/workflows/pages.yml`
- Trigger: push to `main`
- Build command: `npm run build-pages`
- Publish target: `pages` branch

So after pushing `main`, Forgejo Actions builds `_site/` and replaces the contents of `pages`.

### Manual deployment to `pages`

If needed, you can deploy locally:

```bash
npm ci
npm run build-pages

# from repo root, publish generated _site output to pages branch
git fetch origin pages:pages
git worktree add /tmp/beefyi-pages pages
git -C /tmp/beefyi-pages rm -rf .
git -C /tmp/beefyi-pages clean -fdx
cp -a _site/. /tmp/beefyi-pages/
git -C /tmp/beefyi-pages add -A
git -C /tmp/beefyi-pages commit -m "Deploy pages"
git -C /tmp/beefyi-pages push ssh://git@codeberg.org/Beed/beefyi.git pages:pages
```

### Custom domain

If `_site/.domains` exists, deployment should keep it at the root of the `pages` branch so Codeberg Pages can use the custom domain configuration.
