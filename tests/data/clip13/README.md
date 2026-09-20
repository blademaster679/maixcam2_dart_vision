# Clip 13 sparse visual reference

This is an **AI visually reviewed reference**, not human/user-provided ground truth. It must not be described as manual human annotation.

Source: `recordings/maixcam2/2026-09-20/clip_000013_test5min/record_180fps.mp4`, 1344 x 760 pixels, nominal 180 fps. Exactly 39 frames were selected before seeing detector results, every 2 seconds from 0 to 76 seconds (frame indices 0, 360, ..., 13680). Development is 0–38 seconds; heldout is 40–76 seconds. All coordinates are in original full-resolution pixels. `armor_bbox` uses `[x_min, y_min, x_max, y_max]`.

Procedure: all 39 frames were inspected in numbered two-column context sheets (`reference_frames/contact_*.jpg`). Apparatus regions were inspected in four-times nearest-neighbor enlarged crops (`reference_frames/crops_*.jpg`). A simple independent green-excess color mask inside a visually chosen small lamp region assisted approximate center/radius placement. Existing detector outputs and the optimized algorithm were never used to create these references. All positive annotations were then visually checked on annotated crops (`reference_checked_*.jpg`); pale/fragmented lamp centers at 14, 48, 58 and 66 seconds were manually corrected after additional pixel-level inspection.

Status meaning:

- `visible`: the intended green lamp is recognizable; for armor, both upper-board bar positions can be resolved, although their pixels may appear orange or white rather than saturated red. Armor boxes span the two light-bar extents and intervening board face, excluding the green lamp. Centers/boxes are approximate, not subpixel calibrated measurements.
- `absent`: the apparatus/target is out of frame. Background green signs/reflections and unrelated LED displays remain in some of these frames and are negatives.
- `uncertain`: the apparatus may be in frame, but lamp color or two separate bars cannot be reliably resolved. Exclude these rows for the corresponding class from both positive and negative metric denominators. A green-positive frame can have armor-uncertain status. Early uncertain frames also have a coarse `target_region_bbox` for qualitative localization only.

Limitations: only one scene/video and sparse frames; no proof of generalization to new lighting, boards, distances or camera exposure. Small targets are only a few pixels across. Radius/box edges can be uncertain by several pixels because of compression, blur, blooming and exposure. This reference cannot prove frame-by-frame recall or track continuity. The video timestamp is playback time/frame index; original acquisition timestamps are a separate signal. A heldout split in the same video limits tuning leakage but is not an independent test scene.

Counts:

```json
{
  "development": {
    "green": {
      "uncertain": 7,
      "visible": 11,
      "absent": 2
    },
    "armor": {
      "uncertain": 14,
      "visible": 4,
      "absent": 2
    }
  },
  "heldout": {
    "green": {
      "visible": 14,
      "absent": 5
    },
    "armor": {
      "visible": 8,
      "absent": 5,
      "uncertain": 6
    }
  }
}
```
