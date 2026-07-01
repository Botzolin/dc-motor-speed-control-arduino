# DC Motor PI Speed Control — Identification, Loop Shaping & Real-Time Implementation

Complete automatic control project for regulating the speed of a DC motor with encoder, from **system identification**, through **PI controller design via loop shaping** in the frequency domain, **Simulink validation**, to **real-time implementation on Arduino**.

## Contents

| File | Description |
|---|---|
| `data/identificare_sistem.csv` | Experimental step-response data (time, PWM, RPM) — used for identification |
| `matlab/identificare_planta.mlx` | Process model identification (1st-order transfer function) from experimental data |
| `matlab/identificare_regulator_loopshaping.mlx` | PI controller design via loop shaping, Bode/phase margin analysis, discretization |
| `simulink/simulare_regulator_simulink.slx` | Simulink model for closed-loop validation (plant + controller) |
| `firmware/motor_pi_controller.ino` | Arduino firmware: encoder reading (interrupt), discrete PI controller, serial communication |
| `demonstration/demonstration_video` | Demonstration video for the project |

## 1. System Identification

Data: PWM → RPM step response, sampled at `Ts ≈ 50 ms` (`data/identificare_sistem.csv`).

Identified model (`procest`, 1st-order process, no dead time):

$$
P(s) = \frac{0.8311}{0.04964\,s + 1}
$$

→ time constant $\tau \approx 49.6\ \text{ms}$, static gain $K_p \approx 0.83$.

## 2. Controller Design — Loop Shaping

**Performance requirements:**
- Sampling time: `Ts = 0.05 s`
- Overshoot: 0–10%
- Steady-state error ≈ 0
- Actuator saturation: PWM ∈ [0, 255]
- Robust phase margin

**Method:** zero placement + gain for the desired crossover frequency $\omega_c$.

```matlab
s = tf('s');
P = 0.8311/(0.04964*s + 1);

wc = 20;           % desired crossover frequency [rad/s]
z  = wc/2;         % PI zero placement
C0 = (s + z)/s;
K  = 1/abs(evalfr(P*C0, 1j*wc));   % gain for |L(jwc)| = 1
C  = K*C0;
```

The resulting continuous controller is discretized (Tustin, `Ts = 0.05 s`) for implementation:

```matlab
C_disc = c2d(C, Ts, 'tustin');
```

Recursive form of the discrete controller implemented on Arduino:

$$
u[k] = u[k-1] + b_0\,e[k] + b_1\,e[k-1]
$$

with $b_0 = 1.896$, $b_1 = -1.137$ (coefficients resulting from discretization).

## 3. Simulink Validation

`simulink/simulare_regulator_simulink.slx` — closed-loop model (reference → PI controller → identified plant → feedback), used to confirm behavior (overshoot, settling time, steady-state error) before hardware implementation.

## 4. Arduino Implementation (real-time)

`firmware/motor_pi_controller.ino`:
- Reads motor speed from the incremental encoder (PPR = 293) via rising-edge interrupt
- Computes RPM every `SAMPLE_MS = 50 ms`
- Receives speed reference via Serial (`rpm_ref`)
- Applies the discrete PI controller (recursive form above)
- Saturates PWM to [0, 255]
- Sends over Serial: time, reference, measured RPM (for logging/plotting)

**Hardware:**
- DC motor with incremental encoder (293 PPR)
- Motor driver with `RPWM`/`LPWM` inputs (e.g. H-bridge type BTS7960/L298N)
- Arduino (Uno/Mega/Nano — any board with external interrupt on the encoder pin)

### Pinout

| Signal | Arduino Pin |
|---|---|
| RPWM (driver) | D5 |
| LPWM (driver) | D6 |
| Encoder | D2 (interrupt) |

## How to Run the Project

1. **Identification**: run `matlab/identificare_planta.mlx` (requires System Identification Toolbox) on `data/identificare_sistem.csv`
2. **Controller design**: run `matlab/identificare_regulator_loopshaping.mlx` (requires Control System Toolbox)
3. **Validation**: open `simulink/simulare_regulator_simulink.slx` in Simulink and simulate
4. **Hardware**: upload `firmware/motor_pi_controller.ino` to Arduino, connect the motor/encoder per the pinout in the .ino file, send the speed reference (RPM) via Serial Monitor at 115200 baud

## Requirements

- MATLAB with Control System Toolbox and System Identification Toolbox
- Simulink
- Arduino IDE

## License

[MIT](LICENSE)
