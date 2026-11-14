# Images

| File | Size | Where it is used |
| --- | --- | --- |
| `banner.png` | 1280 × 420 | Header of `README.md` / `README.zh-CN.md` |
| `architecture.svg` | 920 × 960 | Vertical architecture diagram in the English README |
| `architecture-zh.svg` | 920 × 960 | Vertical architecture diagram in the Chinese README |
| `vlx-go-architecture.png` | 1400 × 357 | "VLX-Go model" section and `docs/VLX_GO.md` |
| `social-preview.png` | 1280 × 640 | Upload in **GitHub → Settings → Social preview** |
| `observation-dashboard.png` | 1440 × 1200 | Kept for the screenshot section (to be re-added) |
| `dashboard-live.jpg` | 1000 × 2330 | Kept for the screenshot section (to be re-added) |
| `dashboard-history.jpg` | 1000 × 2337 | Kept for the screenshot section (to be re-added) |
| `report-comparison.png` | 1100 × 626 | Kept for the screenshot section (to be re-added) |
| `plan-overlay.png` | 1280 × 930 | Planner input / route / fused path figure, kept for docs |
| `trial-controls.png` | 1440 × 1200 | Browser trial form, kept for the screenshot section |
| `trial-page.png` | 1540 × 1258 | "Running on an AS2": trial page screenshot |
| `as2-robot.jpg` | 1400 × 1327 | "Running on an AS2": the robot, camera and onboard computer |

The banner and the social card were generated locally (PIL) and contain no
third-party artwork. The screenshots and the robot photo were taken by the
authors on their own test bench; the frames show their lab. The trial page uses
a bench setting, not the shipped limits. Check them again if you publish a fork with your own hardware.

## Video and on-robot photos

Add them here when ready; the README no longer reserves a visible block, so the
markup has to be added back at the same time.

| File | What it should be |
| --- | --- |
| `demo.mp4` | A short screen recording or phone clip of the robot (H.264, ≤ 20 MB works best) |
| `demo-poster.jpg` | One frame from the clip, 16:9, used as the clickable preview |
| `robot.jpg` | One or two photos of the AS2 with the AGX Orin and camera mounted |

GitHub's README renderer does not embed `<video>`; the usual options are a poster
image that links to the MP4, a GIF, or a link to an external host.
