# Quickstart Guide

This guide walks you through setting up ApexPathAI and launching the real-time engineering telemetry cockpit.

## Installation

### Prerequisites
- Python 3.10, 3.11, 3.12, or 3.13
- Operating System: Linux, Windows, or macOS

### Step-by-Step Setup

```bash
# 1. Clone the repository
git clone https://github.com/YourOrg/ApexPathAI.git
cd ApexPathAI

# 2. Create virtual environment
python -m venv .venv
source .venv/bin/activate       # On Linux/macOS
# or: .venv\Scripts\activate    # On Windows

# 3. Install in editable mode
pip install -e .
```

## Running the Live Cockpit

Launch the cockpit on your chosen circuit (e.g. Monza, Spa, Silverstone):

```bash
apexpath-cockpit --track Monza --port 8000
```

Open `http://127.0.0.1:8000` in your web browser.

### Key Cockpit Controls
- **Circuit Selector:** Switch instantly between 23 international Grand Prix tracks.
- **E-STOP Button:** Actuates emergency stopping, disengages autonomy, and logs critical safety incidents.
- **Simulation Controls:** Pause, Play, Step-forward, or adjust simulation playback speed (0.5x to 5.0x).
- **Proving Ground Scenarios:** Trigger automated test profiles (Nominal Raceline, Tunnel GNSS Outage, Urban Canyon Multipath).

## Running Offline Benchmarks

To evaluate state estimation accuracy across all 23 racetrack datasets:

```bash
apexpath-benchmark
```

This runs closed-loop filtering and prints the multi-track performance report including RMSE, CEP50/95, Frenet lateral deviations, and filter improvements.
