# AutoMech Project Site

This directory contains the static AutoMech project website deployed through GitHub Pages.

## Local preview

Run either command below.

From the repository root:

```bash
python3 -m http.server 4173 --directory site
```

Or, after entering the site directory:

```bash
cd site
python3 -m http.server 4173
```

Open `http://127.0.0.1:4173` in a browser.

## Deployment

Push changes under `site/` to `main`. The workflow at
`.github/workflows/deploy-pages.yml` publishes this directory as the Pages artifact.

The repository Pages source must be set to **GitHub Actions** under
**Settings → Pages → Build and deployment**.
