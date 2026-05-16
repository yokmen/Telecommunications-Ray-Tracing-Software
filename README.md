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

$$\underline{E_n} = \Gamma_1\Gamma_2\Gamma_3\ldots T_1T_2T_3\ldots \sqrt{60\,G_{TX}(\theta_{TX_n},\phi_{TX_n})\,P_{TX}}\;\frac{e^{-j\beta d_n}}{d_n}$$

with $\underline{E}_n$ the electric field detected at the receiver from multipath component $n$, $\Gamma_i$ the reflection coefficients, $T_i$ the transmission coefficients, $G_{TX}$ the gain of the transmitting antenna in the emission direction, $P_{TX}$ the transmitter power [W], $\beta$ the wavenumber [m$^{-1}$], and $d_n$ the distance traveled [m].

**ERP and EIRP** — the product $G_{TX}(\theta_{TX_n},\phi_{TX_n}) \cdot P_{TX}$ is given by the ERP (Effective Radiated Power) / EIRP (Effective Isotropic Radiated Power) pair:

$$\begin{cases} \text{EIRP[dBm]} = G_{TX}(\theta,\phi)\text{[dBi]} + \text{ERP[dBm]} \\ \text{EIRP[dBm]} = 2.25 + \text{ERP[dBm]} \end{cases}$$

where ERP equals 0 dBm in the TP 8 case and 20 dBm in the aircraft case.

**Received power:**

$$P_{RX} = \frac{\lambda^2}{8\pi^2 R_a} \left|\sum_{n=1}^N \underline{E_n}\right|^2$$

with $\lambda$ the wavelength [m] and $R_a$ the antenna resistance [$\Omega$].

**Average power in a local zone:**

$$P_{RX} = \frac{\lambda^2}{8\pi^2 R_a} \sum_{n=1}^N \left|\underline{E_n}\right|^2$$

**Reflection coefficient for perpendicular polarization:**

$$\Gamma_\perp(\theta_i) = \frac{Z_2\cos\theta_i - Z_1\cos\theta_t}{Z_2\cos\theta_i + Z_1\cos\theta_t}$$

where $Z_i$ is the impedance of medium $i$ [$\Omega$], $\theta_i$ is the angle of incidence and $\theta_t$ is the angle of refraction.

**Total reflection coefficient:**

$$\Gamma_m = \Gamma_\perp - (1-\Gamma_\perp^2)\,\frac{\Gamma_\perp\, e^{-2j\beta_m s}\, e^{j\beta\, 2s\sin\theta_t\sin\theta_i}}{1-\Gamma_\perp^2\, e^{-2j\beta_m s}\, e^{j\beta\, 2s\sin\theta_t\sin\theta_i}}$$

where $s = \dfrac{l}{\cos\theta_t}$ is the propagation distance.

**Transmission coefficient through the wall:**

$$T_m(\theta_i) = \frac{\bigl(1-\Gamma_\perp^2(\theta_i)\bigr)\, e^{-\gamma_m s}}{1-\Gamma_\perp^2(\theta_i)\, e^{-2\gamma_m s}\, e^{j\beta\, 2s\sin\theta_t\sin\theta_i}}$$

where $\gamma_m$ is the complex propagation constant:

$$\gamma_m = \alpha + j\beta = \omega\sqrt{\frac{\mu_0\epsilon}{2}}\left[\sqrt{1+\left(\frac{\sigma}{\omega\epsilon}\right)^2} - 1\right]^{1/2} + j\,\omega\sqrt{\frac{\mu_0\epsilon}{2}}\left[\sqrt{1+\left(\frac{\sigma}{\omega\epsilon}\right)^2} + 1\right]^{1/2}$$

**Received power in logarithmic scale:**

$$P\,[\text{dBm}] = 10\log\frac{P_{RX}[\text{W}]}{1\,\text{mW}}$$

**Bit rate** — varies linearly with the logarithmic power. It equals 27.5 Mb/s at −78 dBm and can be assumed equal to 27.5 Mb/s for lower powers since the reported rate is not sufficient. The maximum value is 4620 Mb/s at −53 dBm:

$$D = \begin{cases} 27.5\ \text{Mb/s}, & P < -78\ \text{dBm} \\ \dfrac{4620 - 27.5}{-53 - (-78)}\,P + 27.5 = (183.7\,P + 27.5)\ \text{Mb/s}, & -78\ \text{dBm} \leq P \leq -53\ \text{dBm} \\ 4620\ \text{Mb/s}, & P > -53\ \text{dBm} \end{cases}$$

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
