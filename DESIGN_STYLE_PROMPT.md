# Design Style Prompt — "Toyota Fleet Control" Look

Paste this into an LLM (or hand to a designer) when building a **new, separate
tool with different functionality** that should visually match the existing
Fleet Control dashboard. It describes the look only — not the features — so the
new thing can have its own widgets while feeling like part of the same family.

---

## Prompt

> Build the UI in a **dark, slick, Toyota-themed control-panel style**. It must
> look like it belongs to the same product family as our Fleet Control
> dashboard. Follow this design system exactly.

### Brand & mood
- Industrial mission-control feel: near-black canvas, matte card surfaces, a
  single hot Toyota red used sparingly as the accent.
- Calm by default; red draws the eye only to the brand mark, primary actions,
  section labels, and alerts. Never flood the screen with red.
- Clean, dense, legible. No gradients, no drop shadows, no rounded-pill
  flourishes — flat surfaces with thin 1px borders.

### Color tokens (use these exact hex values)
| Token | Hex | Use |
|---|---|---|
| `BG` | `#15171c` | App background (near-black) |
| `PANEL` | `#1e2128` | Card / panel surfaces |
| `PANEL_2` | `#262a33` | Inputs, nested surfaces |
| `PANEL_3` | `#2f343f` | Idle buttons |
| `BORDER` | `#333a47` | 1px borders, dividers |
| `TEXT` | `#e8eaed` | Primary text |
| `TEXT_DIM` | `#9aa0a8` | Secondary / hint text |
| `RED` | `#EB0A1E` | Toyota red — brand, primary action, accents |
| `RED_DK` | `#b80818` | Pressed / hover state of red |
| `PLOT_BG` | `#1b1e25` | Chart / canvas background |
| `GRID_MAJ` | `#3a4150` | Major gridlines |
| `GRID_MIN` | `#272b34` | Minor gridlines |

### Status colors (semantic, reuse everywhere)
- Active / healthy / success: `#37E29A` (green)
- Busy / caution / in-progress: `#FFD43B` (amber)
- Blocked / error / alert: `#FF5D6C` (red-pink)
- Info / ready / connected: `#00C2FF` (cyan)
- Neutral / idle: `#9aa0a8` (grey)

### Categorical / series palette (for charts, multiple entities)
Bright and distinct so they pop on the dark canvas; cycle in this order:
`#00C2FF`, `#FF7A00`, `#37E29A`, `#FF5D6C`, `#B68CFF`, `#FFD43B`, `#FF5DA2`,
`#8AE234`, `#4DD0E1`, `#FFA94D`.

### Typography
- Font family: **Helvetica Neue** (fall back to system sans).
- Title: 20px bold. Section/card label: ~11px bold, in `RED`, often UPPERCASE.
- Body: 10px. Secondary/hint: 9px in `TEXT_DIM`.

### Layout
- **Top header bar** on `BG`: a red **brand badge** (solid `RED` block, white
  bold "TOYOTA" text) sitting next to a bold title and a dim one-line subtitle.
- A thin **red accent strip** (a few px tall, `RED`) directly under the header
  to anchor the brand.
- **Two-column body:** a scrollable left column of stacked **cards** (controls,
  forms, tables) and a large right **content card** (chart / map / primary view).
- **Cards:** `PANEL` background, 1px `BORDER`, generous internal padding. Each
  card has an UPPERCASE red label and optional dim hint text in its header row.
- The primary content card has its own header row: a red label on the left,
  a dim usage hint in the middle, and small utility buttons on the right.

### Components
- **Primary button:** solid `RED` bg, white text; hover/press → `RED_DK`.
- **Secondary button:** `PANEL_3` bg, `TEXT` text, 1px `BORDER`.
- **Inputs / dropdowns:** `PANEL_2` field, `TEXT` text, `BORDER` outline;
  dropdown lists use `PANEL_2` with `RED` selection highlight.
- **Tables:** `PANEL` rows ~26px tall, `TEXT` cells, selected row highlighted in
  `RED`. Encode row status with the semantic status colors above (color the text
  or a status cell, not the whole row).
- **Scrollbars:** `BG` trough, `PANEL_3` thumb — minimal, dark.

### Charts / canvases / data views
- Background `PLOT_BG`; title/labels in `TEXT`; major grid `GRID_MAJ`, minor
  grid `GRID_MIN`.
- Boundaries / frames drawn in `RED`. Goals/targets as a `RED` star/marker with
  a thin white edge. Highlights in the categorical palette.
- **Must support pan and zoom-out** (mouse-wheel zoom toward cursor, click-drag
  pan) and a **"Reset View"** button. On live-updating views, **preserve the
  user's current pan/zoom across redraws** — never snap back to default extents
  on each refresh; only reset when the user clicks Reset View.

### Hard rules (preserve consistency)
- Do **not** introduce new accent colors — red is the only brand accent; use the
  semantic set for status and the categorical set for series.
- Flat surfaces, 1px borders, no shadows/gradients/rounded pills.
- Red is for emphasis, not fill — keep large areas dark and quiet.
- Keep it dense and usable; never sacrifice legibility for decoration.
- Whatever the new functionality is, reuse these tokens, the header+accent-strip
  pattern, and the card layout so the two tools read as one product.
