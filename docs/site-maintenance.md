# Maintaining the documentation site

The MkDocs site publishes only the two-pixel NeoPixel reference in
`docs/index.md`. Existing project guides remain in this repository. The allowlist
in `mkdocs.yml` controls which documents and assets are published.

## Preview locally

Use Python 3.10 or later. From the repository root, on Windows:

```powershell
python -m venv .venv-docs
.\.venv-docs\Scripts\python.exe -m pip install -r requirements-docs.txt
.\.venv-docs\Scripts\python.exe -m mkdocs serve
```

If Python is not on PATH, the PlatformIO installation's
`C:\Users\<your-user>\.platformio\python3\python.exe` can create the environment.
Open the local address printed by MkDocs. The server uses the `/AFC-Drone/` path
to match the GitHub Pages site. Stop it with Ctrl+C.

On macOS or Linux, use `.venv-docs/bin/python` after creating the environment.
To build the same output checked in CI:

```powershell
.\.venv-docs\Scripts\python.exe -m mkdocs build --strict
```

Generated files go into the ignored `site/` directory. Do not edit those files.

## Publish on GitHub Pages

1. In the GitHub repository, open **Settings → Pages**.
2. Under **Build and deployment**, select **GitHub Actions** as the source.
3. Merge the documentation files into `main`. The **Documentation** workflow
   builds the site and deploys it. After Pages is enabled, it can also be run
   manually from **Actions → Documentation → Run workflow** on `main`.

The default address is:

<https://keweenaw-propulsion-laboratory.github.io/AFC-Drone/>

Pull requests build the documentation without deploying it. Deployment needs
GitHub Pages enabled and must satisfy any protection rules configured for the
`github-pages` environment. Update `site_url` in `mkdocs.yml` if the address changes.

## Update a pattern

The reference follows the NeoPixel settings in `src/drone.cpp`:

- `Drone::updateLEDS()` defines the nine state patterns.
- `tripleFlash()` defines the **amber breathing** watchdog override for the
  NeoPixels; its name describes the separate single-color LED.

Edit the matching row in `docs/index.md` when changing the firmware. Each
`data-led-preview` element carries its animation settings:

| Attribute | Meaning |
| --- | --- |
| `data-pattern` | `solid`, `breathe`, or `flash` |
| `data-peak` | Peak brightness, 0–255 |
| `data-on-ms` | Full rise/fall duration for breathing, or duration of each flash |
| `data-off-ms` | Dark pause after the full breathing cycle or final flash |
| `data-floor` | Minimum breathing brightness, default 0 |
| `data-flash-count` | Flashes per group, default 1 |
| `data-flash-gap-ms` | Dark gap between flashes in a group, default 0 |

The two `.led` elements define the colors, in chain order. Update the visible
description, accessible label, and brightness notes along with the attributes.
The preview engine is in `docs/javascripts/leds.js`; page styling is in
`docs/stylesheets/leds.css`.

Preview both desktop and narrow layouts. Check play/pause, reduced-motion
behavior, the no-JavaScript fallback, and single versus double flashes. Run the
strict build before merging. Add future pages deliberately to both the allowlist
and navigation in `mkdocs.yml`.
