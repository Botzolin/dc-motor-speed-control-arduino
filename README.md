# DC Motor PI Speed Control — Identification, Loop Shaping & Real-Time Implementation

Proiect complet de control automat pentru reglarea vitezei unui motor DC cu encoder, de la **identificarea sistemului**, prin **proiectarea regulatorului PI prin loop shaping** în domeniul frecvenței, **validare în Simulink**, până la **implementare în timp real pe Arduino**.

## Arhitectura proiectului

```mermaid
flowchart LR
    A[Motor DC + Encoder] -->|date PWM/RPM| B[Identificare sistem<br/>MATLAB]
    B -->|model P(s)| C[Proiectare PI<br/>loop shaping]
    C -->|C(s) -> C z| D[Validare Simulink<br/>buclă închisă]
    D -->|coeficienți discreți| E[Implementare<br/>Arduino timp real]
    E -->|date PWM/RPM| A
```

## Conținut

| Fișier | Descriere |
|---|---|
| `data/identificare_sistem.csv` | Date experimentale răspuns la treaptă (timp, PWM, RPM) — folosite pentru identificare |
| `matlab/identificare_planta.mlx` | Identificarea modelului de proces (funcție de transfer de ordin 1) din datele experimentale |
| `matlab/identificare_regulator_loopshaping.mlx` | Proiectarea regulatorului PI prin loop shaping, analiză Bode/margine de fază, discretizare |
| `simulink/simulare_regulator_simulink.slx` | Model Simulink pentru validarea buclei închise (plantă + regulator) |
| `firmware/motor_pi_controller.ino` | Firmware Arduino: citire encoder (interrupt), regulator PI discret, comunicație serială |

## 1. Identificare sistem

Date: răspuns la treaptă PWM → RPM, eșantionate la `Ts ≈ 50 ms` (`data/identificare_sistem.csv`).

Model identificat (`procest`, proces de ordin 1, fără timp mort):

$$
P(s) = \frac{0.8311}{0.04964\,s + 1}
$$

→ constantă de timp $\tau \approx 49.6\ \text{ms}$, câștig static $K_p \approx 0.83$.

## 2. Proiectare regulator — Loop Shaping

**Cerințe de performanță:**
- Timp de eșantionare: `Ts = 0.05 s`
- Overshoot: 0–10%
- Eroare staționară ≈ 0
- Saturare actuator: PWM ∈ [0, 255]
- Margine de fază robustă

**Metodă:** plasare zero + câștig pentru frecvența de tăiere dorită $\omega_c$.

```matlab
s = tf('s');
P = 0.8311/(0.04964*s + 1);

wc = 20;           % frecventa de taiere dorita [rad/s]
z  = wc/2;         % plasare zerou PI
C0 = (s + z)/s;
K  = 1/abs(evalfr(P*C0, 1j*wc));   % gain pentru |L(jwc)| = 1
C  = K*C0;
```

Regulatorul continuu rezultat e discretizat (Tustin, `Ts = 0.05 s`) pentru implementare:

```matlab
C_disc = c2d(C, Ts, 'tustin');
```

Forma recursivă a regulatorului discret implementată pe Arduino:

$$
u[k] = u[k-1] + b_0\,e[k] + b_1\,e[k-1]
$$

cu $b_0 = 1.896$, $b_1 = -1.137$ (coeficienții rezultați din discretizare).

## 3. Validare Simulink

`simulink/simulare_regulator_simulink.slx` — model buclă închisă (referință → regulator PI → plantă identificată → feedback), folosit pentru a confirma comportamentul (overshoot, timp de stabilizare, eroare staționară) înainte de implementarea hardware.

## 4. Implementare Arduino (timp real)

`firmware/motor_pi_controller.ino`:
- Citește viteza motorului din encoder (PPR = 293) via interrupt pe front crescător
- Calculează RPM la fiecare `SAMPLE_MS = 50 ms`
- Primește referința de viteză prin Serial (`rpm_ref`)
- Aplică regulatorul PI discret (forma recursivă de mai sus)
- Saturare PWM la [0, 255]
- Trimite pe Serial: timp, referință, RPM măsurat (pentru logging/plotare)

**Hardware:**
- Motor DC cu encoder incremental (293 PPR)
- Driver motor cu intrări `RPWM`/`LPWM` (ex. punte H tip BTS7960/L298N)
- Arduino (Uno/Mega/Nano — orice placă cu interrupt extern pe pinul encoderului)

### Pinout

| Semnal | Pin Arduino |
|---|---|
| RPWM (driver) | D5 |
| LPWM (driver) | D6 |
| Encoder | D2 (interrupt) |

## Cum rulezi proiectul

1. **Identificare**: rulează `matlab/identificare_planta.mlx` (necesită System Identification Toolbox) pe `data/identificare_sistem.csv`
2. **Design regulator**: rulează `matlab/identificare_regulator_loopshaping.mlx` (necesită Control System Toolbox)
3. **Validare**: deschide `simulink/simulare_regulator_simulink.slx` în Simulink și simulează
4. **Hardware**: încarcă `firmware/motor_pi_controller.ino` pe Arduino, conectează motorul/encoderul conform pinout-ului, trimite referința de viteză (RPM) prin Serial Monitor la 115200 baud

## Cerințe

- MATLAB cu Control System Toolbox și System Identification Toolbox
- Simulink
- Arduino IDE

## Licență

[MIT](LICENSE)
