# Images and media

| File | Size | Where it is used |
| --- | --- | --- |
| `banner.png` | 1280 × 420 | Header of `README.md` / `README.zh-CN.md` |
| `social-preview.png` | 1280 × 640 | Upload in **GitHub → Settings → Social preview** |
| `observation-dashboard.png` | 1440 × 1200 | Quick start figure: the live dashboard (top of the page) |
| `dashboard-live.jpg` | 1000 × 2330 | Screenshots section: the whole dashboard page |
| `dashboard-history.jpg` | 1000 × 2337 | Screenshots section: history view of a saved run |
| `report-comparison.png` | 1100 × 626 | Screenshots section: offline report figure |
| `plan-overlay.png` | 1280 × 930 | Architecture figure: planner input, image route, fused path |
| `trial-controls.png` | 1440 × 1200 | "Running on an AS2": browser trial form |

The banner and the social card were generated locally (PIL) and contain no
third-party artwork. The screenshots were captured by the authors on their own
test bench; the RGB frames show their lab, and no people are identifiable in
them. Check them again if you publish a fork with your own hardware.

## Placeholders for video and on-robot photos

The README has an empty "Video and on-robot photos" block, commented out until
the files exist. Drop them here with these names and the markup can be enabled:

| File | What it should be |
| --- | --- |
| `demo.mp4` | A short screen recording or phone clip of the robot (H.264, ≤ 20 MB works best) |
| `demo-poster.jpg` | One frame from the clip, 16:9, used as the clickable preview |
| `robot.jpg` | One or two photos of the AS2 with the AGX Orin and camera mounted |

GitHub's README renderer does not embed `<video>`; the usual options are a
poster image that links to the MP4, a GIF, or a link to an external host. If the
clip is long, cut a 10–15 s segment for the GIF and keep the full MP4 as a
link.
