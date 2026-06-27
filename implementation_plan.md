# Frontend Overhaul — Implementation Plan

Comprehensive revamp of `Index.html` to make the UI fully functional, logically organized for suspension engineering workflows, and correctly wired to all backend API endpoints.

> [!IMPORTANT]
> The existing color scheme (#0a0a0a background, #E84045 red accents, #121212 cards, Outfit/Space Mono fonts) is preserved exactly.

## Proposed Changes

### 1. Tab & Navigation Audit — Fix Dead and Broken Navigation

**Problems found:**
- Sidebar links for **Tyre**, **Spring**, **Data Logger**, and **Settings** are dead `<a href="#">` with no `onclick` handlers — they do nothing.
- The **Mapping** and **Library** header tabs in the dashboard have no click handlers.
- The `switchScreen()` function only knows about `dashboard`, `vehicle`, and `diagnostics` — there is no `screen-diagnostics` sidebar nav link (only a header tab `navTopDiagnostics`).
- The Diagnostics header tab link from the dashboard works, but there is no way back except clicking the "Telemetry" header tab inside the diagnostics screen.

**Fixes:**
- Remove the dead sidebar links (Tyre, Spring, Data Logger) — these are placeholder features with no backend support.
- Wire the **Diagnostics** screen to a proper sidebar nav link.
- Remove dead header tabs (Mapping, Library) that have no backing screens.
- Add the **Diagnostics** nav item in the sidebar with a proper `onclick="switchScreen('diagnostics')"`.
- Make the header tabs across all screens consistent: always show Telemetry / Dynamics / Optimization / Diagnostics as the top-level horizontal nav, and make them work from any screen.

---

### 2. Logical Data Input Grouping

**Current problem:** The left panel has all 5 sliders (ms, mu, k, c, kt) in a flat list with no logical separation, and the new `travel_limit_mm` / `preload_mm` fields are completely absent from the UI despite being required by the backend.

**New grouping structure for the left panel:**

#### Section A — Vehicle Parameters
- `ms` — Sprung Mass (kg)
- `mu` — Unsprung Mass (kg)

#### Section B — Suspension Componentry
- `k` — Spring Rate (N/m)
- `c` — Damping Coefficient (N·s/m)
- `kt` — Tyre Stiffness (N/m)
- `travel_limit_mm` — Suspension Travel Limit (mm) — **NEW, required by backend**
- `preload_mm` — Spring Preload (mm) — **NEW, optional**

#### Section C — Road Profile Settings
- Step / Sine / ISO 8608 selector (existing)
- Road Class, Vehicle Speed (existing for ISO 8608)
- Seed field for ISO 8608 (optional, exposed for reproducibility)

---

### 3. Output Visualization Slots

**Current problem:** The metric strip cards (ζ, ISO 2631, RMS, etc.) reference DOM IDs like `mv-zeta`, `mv-iso`, `mv-rms`, `mv-tire`, `mv-travel`, `mv-sag`, `mv-bottomout`, `comfortBand`, `zetaFill` — but **none of these elements exist in the HTML**. The `updateMetrics()` function silently no-ops because every `getElementById` returns null.

**Fixes:**
- Add a proper **Metric Strip** above the chart area with cards for:
  - Damping Ratio (ζ) with fill bar
  - ISO 2631 Weighted RMS with comfort band label
  - RMS Body Acceleration
  - RMS Tyre Force
  - Max Suspension Travel
  - Sag % and Bottom-out indicator
  - Ensemble stats (mean ± std for ISO when ISO 8608 is used)
- The three chart tabs (Overview, Dynamics, Optimization) remain as-is but with properly allocated space.
- The FRF chart under "Dynamics" also shows a **Tyre Force Transmissibility** chart alongside the existing FRF.

---

### 4. Data Wiring Fixes

**Problems found:**
- `buildRoadProfile()` builds a JSON object with a `type` field, but the backend expects a **tagged enum** with the Serde convention. E.g., the backend expects `{"Step": {"height": 0.05}}` not `{"type": "Step", "height": 0.05}`. This is the **root cause** of simulation failures — every API call sends a malformed road profile.
- `travel_limit_mm` is now **required** by the backend but is never sent in the payload.
- `getParams()` doesn't include `travel_limit_mm` or `preload_mm`.
- The FRF endpoint at `/frf` expects `travel_limit_mm` as a required field but the frontend sends no such field.
- The sweep endpoint at `/sweep` also expects `travel_limit_mm` as required.

**Fixes:**
- Fix `buildRoadProfile()` to return the correct tagged-enum JSON shape.
- Add `travel_limit_mm` and `preload_mm` to `getParams()` and all outbound payloads (`/simulate`, `/frf`, `/sweep`).
- Wire the new ensemble output fields (`iso2631_mean`, `iso2631_std`) to display when present in the response.

---

## Open Questions

> [!IMPORTANT]
> **Road Profile JSON Format:** The Rocket/Serde backend uses `#[serde(tag)]` or adjacently-tagged enums for `RoadProfileInput`. I need to verify the exact tag format by checking the `#[derive(Deserialize)]` config on the enum. I'll inspect the backend enum definition during implementation to get this exactly right. This is the single most critical data-wiring fix.

## Verification Plan

### Manual Verification
- Open the HTML file directly in the browser.
- Verify all sidebar and header nav links switch screens correctly.
- Verify all tab switching (Overview / Dynamics / Optimization) works.
- Run a simulation and confirm the metric strip updates.
- Run a Pareto sweep and confirm the chart renders.
- Check the Diagnostics screen populates after a simulation.
- Confirm that `travel_limit_mm` is included in all outgoing API payloads via browser DevTools network tab.
