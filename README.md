# Telecommunications Ray-Tracing Software

A 2D ray-tracing simulator written in C++ that estimates Wi-Fi coverage inside an aircraft cabin. Two transmitters illuminate a grid of 20 cm × 20 cm local zones, and the simulator computes the received power and the resulting bit rate in each zone. Results are rendered in real time with SDL2 as a heat map overlaid on the cabin layout.

> **Academic project.** This software was developed as part of a university telecommunications course. It is not intended for production RF planning.

## Overview

The simulation models the horizontal plane of an aircraft cabin made of:

- **Exterior fuselage walls** in GRP (Glass-Reinforced Plastic),
- **Interior partitions** (also GRP) separating the cockpit, first class, and economy class,
- **Plastic seats** arranged in rows for both classes,
- Two Wi-Fi **transmitters** placed inside the cabin (`emetteur1`, `emetteur2` in [Simulation.h](Simulation.h#L5-L6)).

For every receiver position on the grid, the program sums the contributions of the direct ray and every single-reflection ray off each wall, applying the appropriate reflection and transmission coefficients along the way. The resulting field magnitude is converted to a power, then to dBm, and finally mapped to a colored cell on screen.

A second, simpler scene (`walls`, `emetteurTP8`, `recepteurTP8`) is kept in the source for validation against a reference exercise (TP 8).

## Method: the method of images

A ray-tracing tool simulates the paths of rays traveling from a transmitter to a receiver. The traditional approach starts from the receiver, shoots rays in every direction at a fixed angular spacing, follows their reflections and transmissions through obstacles, and keeps only the rays that end up close to the source — from which the per-ray powers are deduced.

This project instead uses the **method of images**. A ray reflected by a wall can be traced between two points by computing the image of one point with respect to that wall, then drawing the straight ray from the other point toward this image (clipped at the wall). This approach only ever computes the rays that actually connect the transmitter and the receiver, which is far more efficient than angular sampling.

## Assumptions

- Transmitters and receivers are **lossless vertical λ/2 dipoles**.
- Transmit power: **20 dBm** at **60 GHz**.
- Walls have **zero thickness for geometry** but a non-zero thickness (10 cm for the aircraft, 15 cm for TP 8) is used in the **reflection and transmission coefficient** computations.
- Aircraft fuselage and certain internal partitions are **GRP**; seats are **plastic**.
- Waves propagate **horizontally**; transmitters and receivers sit at the same height above the floor.

The following are neglected:

- Waves leaving the horizontal plane,
- Propagation delay,
- Diffraction,
- Rays having undergone more than 3 reflections (the code currently sums the direct ray plus single reflections; 2- and 3-reflection loops are present but commented out).

## Equations used

**Electric field of multipath component _n_:**

```
E_n = Γ₁Γ₂Γ₃… · T₁T₂T₃… · √(60 · G_TX(θ,φ) · P_TX) · exp(-jβd_n) / d_n
```

where the `Γᵢ` are reflection coefficients, the `Tᵢ` transmission coefficients, `G_TX` the transmitter gain in the emission direction, `P_TX` the transmit power (W), `β` the wavenumber (m⁻¹), and `d_n` the distance traveled (m).

**ERP / EIRP** — the product `G_TX(θ,φ) · P_TX` is given by:

```
EIRP[dBm] = G_TX(θ,φ)[dBi] + ERP[dBm]
EIRP[dBm] = 2.25 + ERP[dBm]
```

with ERP = 0 dBm for TP 8 and 20 dBm for the aircraft case.

**Received power (coherent sum):**

```
P_RX = λ² / (8π²·R_a) · |Σ E_n|²
```

**Average power in a local zone (incoherent sum):**

```
P_RX = λ² / (8π²·R_a) · Σ |E_n|²
```

**Perpendicular-polarization reflection coefficient:**

```
Γ⊥(θ_i) = (Z₂·cos θ_i − Z₁·cos θ_t) / (Z₂·cos θ_i + Z₁·cos θ_t)
```

**Total reflection coefficient** (accounting for multiple internal reflections inside the wall):

```
Γ_m = Γ⊥ − (1 − Γ⊥²) · [Γ⊥·e^(−2jβ_m·s)·e^(jβ·2s·sin θ_t·sin θ_i)]
           / [1 − Γ⊥²·e^(−2jβ_m·s)·e^(jβ·2s·sin θ_t·sin θ_i)]
```

with `s = l / cos θ_t`.

**Transmission coefficient through the wall:**

```
T_m(θ_i) = (1 − Γ⊥²)·e^(−γ_m·s)
         / [1 − Γ⊥²·e^(−2γ_m·s)·e^(jβ·2s·sin θ_t·sin θ_i)]
```

where `γ_m = α + jβ` is the complex propagation constant:

```
γ_m = ω·√(μ₀ε/2) · [√(1 + (σ/ωε)²) − 1]^(1/2)
    + jω·√(μ₀ε/2) · [√(1 + (σ/ωε)²) + 1]^(1/2)
```

**Power in dBm:**

```
P[dBm] = 10·log₁₀(P_RX[W] / 1 mW)
```

**Bit rate** — varies linearly with the logarithmic power between two reference points (27.5 Mb/s at −78 dBm, 4620 Mb/s at −53 dBm) and is clamped outside that range:

```
D = 27.5 Mb/s                              if P < −78 dBm
D = 183.7·P + 27.5  Mb/s                   if −78 dBm ≤ P ≤ −53 dBm
D = 4620 Mb/s                              if P > −53 dBm
```

## Project layout

| File | Role |
|---|---|
| [main.cpp](main.cpp) | SDL2 window, rendering, color mapping, and the main ray-tracing loop (`computation`) |
| [Simulation.h](Simulation.h) | Transmitter positions and wall definitions for both the aircraft scene and the TP 8 reference scene |
| [Wall.h](Wall.h) / [Wall.cpp](Wall.cpp) | `Wall` class: geometry, image with respect to a wall, segment-segment intersection, complex reflection/transmission coefficients |
| [Vector2.h](Vector2.h) / [Vector2.cpp](Vector2.cpp) | Minimal 2D vector with dot product, norm, and standard operators |
| [Constants.h](Constants.h) | Physical constants, material parameters (εᵣ, σ, thickness for GRP and plastic), frequency, wavelength, antenna resistance |
| [CMakeLists.txt](CMakeLists.txt) | Build configuration; links against SDL2 from `dependencies/SDL` |

## Building

The project depends on **SDL2** (headers and libraries expected under `dependencies/SDL/`) and uses **CMake ≥ 3.22** with **C++14**.

```bash
mkdir build && cd build
cmake ..
cmake --build .
./RayTracing
```

The CMake configuration currently links the Windows `x64` SDL2 import libraries (`SDL2.lib`, `SDL2main.lib`, `SDL2_image.lib`). On other platforms, adjust [CMakeLists.txt](CMakeLists.txt#L8-L11) to point at the appropriate SDL2 binaries (e.g. `find_package(SDL2 REQUIRED)`).

## Using the viewer

On launch, the program first computes the received power on the full grid (this can take several seconds depending on wall count — total runtime is reported on stdout) and then opens an SDL2 window showing two overlaid views of the cabin:

- The **top view** is colored by received power, normalized between the minimum and maximum observed power across the grid.
- The **bottom view** (offset by 10 m on Y) is colored by **bit rate**, using the −78 dBm / −53 dBm thresholds.

Controls:

- **Left-click and drag** to pan.
- **Mouse wheel** to zoom in and out.
- **Close the window** to quit.

Green wall segments are conductive/lossy materials (GRP); blue segments are plastic seats. Green crosses mark the two transmitter locations.

## Notes on the implementation

- The reflection / transmission coefficient computations live in [Wall.cpp:10-77](Wall.cpp#L10-L77) and follow the equations above directly, using `complex<double>` throughout.
- The image of a point with respect to a wall is computed in [Wall.cpp:79-87](Wall.cpp#L79-L87).
- Segment-segment intersection with epsilon guards against colinear/edge cases is implemented in [Wall.cpp:89-152](Wall.cpp#L89-L152).
- The main ray summation lives in [main.cpp:230-352](main.cpp#L230-L352). Loops for 2 and 3 reflections are preserved in comments and can be re-enabled at the cost of significantly longer runtimes.
