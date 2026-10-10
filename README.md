# ZOPH

> A high-performance modular platform for modelling vehicle suspension behavior, analyzing ride comfort, and optimizing parameters.

[![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)](#)
[![React](https://img.shields.io/badge/react-%2320232a.svg?style=for-the-badge&logo=react&logoColor=%2361DAFB)](#)
[![License: Academic](https://img.shields.io/badge/License-Academic-blue.svg?style=for-the-badge)](#)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen?style=for-the-badge)](#)

## Demo

![ZOPH UI Placeholder](https://via.placeholder.com/800x450.png?text=UI+Screenshot+Here) 
<!-- TODO: Add a real screenshot or GIF of the web interface in action here -->

## Features

- **Fast Time-Domain Simulation:** Custom 2-DOF quarter-car model using an optimized RK4 integrator (`dt = 0.001s`).
- **Standardized Road Profiles:** Evaluate against Step, Sine, and ISO 8608 random road profiles.
- **Ride Comfort Analysis:** On-the-fly ISO 2631-1 frequency-weighted RMS acceleration calculation.
- **Analytical FRF Sweep:** Instantaneous Frequency Response Function via exact Laplace-domain matrix inversion (avoids FFT transient artifacts).
- **Multi-Objective Optimization:** 2D parameter sweeps (Spring Rate vs. Damping) with automatic Pareto front extraction.
- **SAE Standards Compliant:** Pluggable vehicle templates based on SAE J2168, J670, and J2704 guidelines.

## Architecture

```mermaid
graph TD
    A[Web UI <br> React/Vite & HTML] -->|HTTP JSON| B(Backend API <br> Rocket)
    B --> C{Validation Schema}
    B --> D[Core Physics Engine <br> suspension_core]
    D --> E[Simulate: RK4 Time Domain]
    D --> F[FRF: Analytical Frequency Sweep]
    D --> G[Sweep: Pareto Front Extractor]
    B --> H[(PostgreSQL Database)]
```

The system employs a clear separation of concerns. The interactive frontend dispatches JSON configuration payloads to a robust Rocket backend. The backend validates physical constraints before delegating the heavy mathematical lifting to the isolated `suspension_core` crate. Simulation history and metrics can be logged to a PostgreSQL database.

## Tech Stack

| Layer | Technology | Why |
| --- | --- | --- |
| **Core Engine** | Rust | Maximum performance for RK4 simulation, strong mathematical typing |
| **Backend API** | Rust (Rocket, SQLx) | Memory safety, blazing fast request handling, async PostgreSQL integration |
| **Database** | PostgreSQL | Reliable persistent storage of simulation history and parameter sets |
| **Frontend** | React (Vite) / HTML | Responsive, component-driven configuration panels and interactive charting |

## Project Structure

```text
suspension_app/
├── backend/            # Rocket API server
│   ├── Cargo.toml      # Backend dependencies (Rocket, SQLx)
│   └── src/            
│       └── main.rs     # API routes, parameter validation, DB queries
├── core/               # Physics engine (suspension_core)
│   ├── Cargo.toml
│   └── src/
│       └── lib.rs      # RK4 solver, ISO 2631-1 logic, FRF, Pareto sweeps
├── frontend/           # Legacy/alternative HTML frontend layer
├── ui/                 # Modern React/Vite UI application
│   ├── package.json    # UI dependencies (React, Tailwind)
│   └── src/            # UI components and API client code
├── compose.yml         # Docker configuration for PostgreSQL database
└── Cargo.toml          # Cargo Workspace root manifest
```

## Getting Started

### Prerequisites

- [Rust](https://rustup.rs/) (latest stable)
- [Node.js](https://nodejs.org/) & npm
- [Docker](https://www.docker.com/) (optional, for PostgreSQL DB)

### Installation & Running

**1. Start the Database**
```bash
docker-compose up -d
```
<!-- TODO: Document any sqlx database migration/setup steps needed for `simulations` table creation -->

**2. Run the Backend API**
```bash
cd backend
cargo run
```
*The server will start on port 8000.*

**3. Run the Frontend UI**
Open a new terminal session:
```bash
cd ui
npm install
npm run dev
```
*Vite will provide a localhost URL to view the interface.*

## Usage

Trigger a single quarter-car simulation programmatically via the backend API:

```bash
curl -X POST http://localhost:8000/simulate \
  -H "Content-Type: application/json" \
  -d '{
    "ms": 290.0, 
    "mu": 40.0, 
    "k": 22000.0, 
    "c": 1500.0, 
    "kt": 190000.0,
    "travel_limit_mm": 120.0,
    "road_profile": { "type": "Step", "height": 0.05 }
  }'
```

**Expected Output (Snippet):**
```json
{
  "zeta": 0.297,
  "mass_ratio": 0.138,
  "rms_body_acc": 0.842,
  "rms_tire_force": 412.5,
  "max_suspension_travel": 0.045,
  "iso2631_weighted_rms": 0.512,
  "bottom_out": false
}
```

## Configuration & Environment Variables

Create a `.env` file in the root or `backend` directory:

| Variable | Description | Default |
| --- | --- | --- |
| `DATABASE_URL` | Connection string for PostgreSQL | `postgres://postgres:Rahul1911*@localhost:5432/postgres` (implied by compose.yml) |

## Testing

The core engine includes a comprehensive test suite covering mathematical validation and edge cases.

```bash
cargo test -p core
```

## Design Decisions & Trade-offs

- **Quarter-Car 2-DOF Model:** Chosen over a full 7-DOF model to isolate fundamental spring/damper tuning and ensure sub-millisecond API response times. A 4-DOF pitch model is planned.
- **Analytical FRF Calculation:** The Frequency Response Function is computed via exact matrix inversion (Cramer's rule) in the Laplace domain rather than running a time-domain sweep and performing an FFT. This entirely eliminates transient contamination and windowing artifacts.
- **ISO 2631-1 Implementation:** The standard's complex Wk weighting filter is implemented via a simplified piecewise log-linear approximation using the Goertzel algorithm. This permits extremely efficient evaluation directly inside the time-domain processor loop.
- **Pareto Front Extraction in Core:** During parameter sweeps, the core calculates and extracts the Pareto front locally, discarding strictly dominated configurations before responding to the client. This dramatically reduces JSON payload sizes.

## Roadmap

- [ ] Implement a 4-DOF half-car pitch model (adding wheelbase, CoG height, and pitch inertia).
- [ ] Separate compression and rebound damping parameters (`c_comp`, `c_reb`).
- [ ] UI: Add visual overlays for the last 3 simulation runs on charts.
- [ ] Incorporate vehicle speed sweeps for comprehensive ISO 2631-1 comfort mapping.
- [ ] Implement a bottoming-out flag warning for when max travel limits are exceeded.

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Ensure code builds and tests pass (`cargo check && cargo test`)
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## License

This project is intended for academic and research purposes.
