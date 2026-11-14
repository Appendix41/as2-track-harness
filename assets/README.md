# Images and media

| File | Size | Where it is used |
| --- | --- | --- |
| `banner.png` | 1280 × 420 | Header of `README.md` / `README.zh-CN.md` |
| `architecture.svg` | 920 × 960 | Vertical architecture diagram in the English README |
| `architecture-zh.svg` | 920 × 960 | Vertical architecture diagram in the Chinese README |
| `vlx-go-architecture.png` | 1400 × 357 | "VLX-Go model" section and `docs/VLX_GO.md` |
| `social-preview.png` | 1280 × 640 | Upload in **GitHub → Settings → Social preview** |
| `as2-robot.jpg` | 1400 × 1327 | "Running on an AS2": the robot, camera and onboard computer |
| `trial-page.png` | 1540 × 1258 | "Running on an AS2": the browser trial page |
| `dashboard-panels.png` | 1080 × 1155 | "Running on an AS2": observation page panels (captured offline) |
| `demo.mp4` | 1280 × 720, 10 s, 2.0 MB | Results: demo clip, H.264/AAC (the click target) |
| `demo.gif` | 480 × 270, 10 s, 3.2 MB | Results: inline looping preview, links to `demo.mp4` |
| `demo-poster.jpg` | 1280 × 720 | Poster frame, not referenced at the moment |
| `observation-dashboard.png` | 1440 × 1200 | Kept for a future screenshot section |
| `dashboard-live.jpg` | 1000 × 2330 | Kept for a future screenshot section |
| `dashboard-history.jpg` | 1000 × 2337 | Kept for a future screenshot section |
| `report-comparison.png` | 1100 × 626 | Kept for a future screenshot section |
| `plan-overlay.png` | 1280 × 930 | Planner input / route / fused path figure, kept for docs |
| `trial-controls.png` | 1440 × 1200 | Browser trial form, kept for a future screenshot section |

The banner and the social card were generated locally (PIL) and contain no
third-party artwork. The screenshots, the robot photo and the demo clip were taken
by the authors on their own test bench; the frames show their lab. The trial page
uses a bench setting, not the shipped limits.

## Video

GitHub does not embed `<video>` in READMEs, so the README uses a looping GIF that
links to the H.264 MP4. Keep the clip short; the GIF is 480 px wide and 6 fps to
stay around 3 MB, and the MP4 is the full 1280 × 720 version.
