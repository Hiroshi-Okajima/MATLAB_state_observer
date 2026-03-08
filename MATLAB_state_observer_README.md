# MATLAB_state_observer

**MATLAB codes for state observer and state estimation** — from fundamental Luenberger observers to advanced multi-rate and outlier-robust methods.

This repository accompanies the comprehensive guide: **[State Observer and State Estimation: A Comprehensive Guide](https://blog.control-theory.com/entry/state-observer-estimation)** on [blog.control-theory.com](https://blog.control-theory.com/).

**Author**: [Hiroshi Okajima](https://www.control-theory.com/en), Associate Professor, Kumamoto University, Japan

---

## Repository Structure

```
MATLAB_state_observer/
│
├── 01_luenberger_observer/          Continuous-time and discrete-time Luenberger observer
│   ├── main_continuous.m
│   ├── main_discrete.m
│   └── README.md
│
├── 02_observer_based_control/       Observer-based feedback control (separation principle)
│   ├── main_observer_feedback.m
│   └── README.md
│
├── 03_kalman_filter/                Kalman filter (standard, steady-state)
│   ├── main_kalman.m
│   └── README.md
│
├── 04_hinf_filter/                  H-infinity filter design via LMI
│   ├── main_hinf_filter.m
│   └── README.md
│
├── 05_multirate_observer/           Multi-rate state observer (IEEE Access 2023)
│   ├── main_multirate_observer.m
│   └── README.md
│
├── 06_multirate_feedback/           Observer-based feedback for multi-rate systems (IEEE Access 2023)
│   ├── main_multirate_feedback.m
│   └── README.md
│
├── 07_multirate_sysid/              Multirate system identification (JRM 2025)
│   ├── main_multirate_sysid.m
│   └── README.md
│
├── 08_multirate_kalman/             Multi-rate Kalman filter via LMI (arXiv 2026)
│   ├── main_multirate_kalman.m
│   └── README.md
│
├── 09_mcv_observer/                 MCV observer: outlier-robust estimation (JCMSI 2021)
│   ├── main_mcv_observer.m
│   └── README.md
│
└── common/                          Shared utility functions
    ├── cyclic_reformulation.m
    ├── build_V_matrix.m
    └── lmi_utils.m
```

---

## Folder Descriptions and Related Papers

### 01_luenberger_observer — Fundamental State Observer

Basic Luenberger observer implementation for both continuous-time and discrete-time LTI systems. Demonstrates observer gain design via pole placement and the convergence of estimation error.

- **Blog article**: [State Observer for State Space Model](https://blog.control-theory.com/entry/2024/10/01/143305)
- **Blog article**: [State Observer: Understanding the Basic Mechanism](https://blog.control-theory.com/entry/2024/02/28/100201)

### 02_observer_based_control — Separation Principle

Observer-based state feedback control. Demonstrates the separation principle: independent design of controller gain K and observer gain L.

- **Blog article**: [State Observer for State Space Model](https://blog.control-theory.com/entry/2024/10/01/143305)

### 03_kalman_filter — Optimal Stochastic Estimation

Standard discrete-time Kalman filter for systems with process noise and measurement noise. Includes steady-state Kalman filter and continuous-time Kalman-Bucy filter.

- **Blog article**: [Kalman Filter: From Basic Algorithm to Multi-Rate Extensions](https://blog.control-theory.com/entry/kalman-filter)

### 04_hinf_filter — Robust H∞ Filter via LMI

H-infinity filter design using Linear Matrix Inequalities (LMIs). Minimizes the worst-case estimation error gain without requiring statistical noise assumptions.

- **Blog article**: [H-infinity Filter: Robust State Estimation Using LMI Optimization](https://blog.control-theory.com/entry/h-infinity-filter)
- **Blog article**: [Linear Matrix Inequalities (LMIs) and Controller Design](https://blog.control-theory.com/entry/lmi-eng)
- **Research page**: [Linear Matrix Inequality](https://www.control-theory.com/en/linear-matrix-inequality)

### 05_multirate_observer — Multi-Rate State Observer

Periodically time-varying state observer for systems where sensors operate at different sampling rates. Observer gains designed via LMI optimization of the l2-induced norm.

- **Paper**: H. Okajima, Y. Hosoe and T. Hagiwara, "[State Observer Under Multi-Rate Sensing Environment and Its Design Using l2-Induced Norm](https://ieeexplore.ieee.org/document/10054014)," IEEE Access (2023)
- **Blog article**: [State Observer Under Multi-Rate Sensing Environment](https://blog.control-theory.com/entry/multirate-observer-eng)
- **MATLAB File Exchange**: [State Estimation under Multi-Rate Sensing: IEEE ACCESS 2023](https://jp.mathworks.com/matlabcentral/fileexchange/182941-state-estimation-under-multi-rate-sensing-ieee-access-2023)
- **Code Ocean**: [Multi-Rate System Code](https://codeocean.com/capsule/3611894/tree/v1)
- **Research page**: [Multi-rate System](https://www.control-theory.com/en/multi-rate-system)

### 06_multirate_feedback — Observer-Based Feedback for Multi-Rate Systems

Complete observer-based feedback controller for multi-rate systems using cyclic reformulation.

- **Paper**: H. Okajima, K. Arinaga and A. Hayashida, "[Design of observer-based feedback controller for multi-rate systems with various sampling periods using cyclic reformulation](https://ieeexplore.ieee.org/document/10304152)," IEEE Access (2023)
- **Blog article**: [State Observer Under Multi-Rate Sensing Environment](https://blog.control-theory.com/entry/multirate-observer-eng)
- **Research page**: [Multi-rate System](https://www.control-theory.com/en/multi-rate-system)

### 07_multirate_sysid — Multirate System Identification

System identification under multirate sensing environments using cyclic reformulation and subspace identification (N4SID). Works with arbitrary (non-periodic) inputs.

- **Paper**: H. Okajima, R. Furukawa and N. Matsunaga, "[System Identification Under Multirate Sensing Environments](https://doi.org/10.20965/jrm.2025.p1102)," Journal of Robotics and Mechatronics, Vol. 37, No. 5, pp. 1102–1112 (2025) **(Open Access)**
- **Blog article**: [System Identification Under Multirate Sensing Environments](https://blog.control-theory.com/entry/multirate-sysid-eng)
- **Research page**: [Multi-rate System](https://www.control-theory.com/en/multi-rate-system)

### 08_multirate_kalman — Multi-Rate Kalman Filter via LMI

Steady-state Kalman filter design for multirate systems using LMI optimization with cyclic reformulation.

- **Paper**: H. Okajima, "LMI Optimization Based Multirate Steady-State Kalman Filter Design," [arXiv:2602.01537](https://arxiv.org/abs/2602.01537) (2026, submitted)
- **Research page**: [Multi-rate System](https://www.control-theory.com/en/multi-rate-system)

### 09_mcv_observer — MCV Observer (Outlier-Robust)

Median of Candidate Vectors (MCV) observer for state estimation robust to sensor outliers. Multiple estimation candidates are generated and the median operation selects one unaffected by outliers.

- **Paper**: H. Okajima, Y. Kaneda and N. Matsunaga, "[State estimation method using median of multiple candidates for observation signals including outliers](https://doi.org/10.1080/18824889.2021.1985702)," SICE JCMSI, Vol. 14, No. 1, pp. 257–267 (2021) **(Open Access)**
- **Blog article**: [Outlier-Robust State Estimation: MCV Observer](https://blog.control-theory.com/entry/mcv-observer-eng)
- **Blog article**: [State Estimation Unaffected by Sensor Outliers: MCV Approach](https://blog.control-theory.com/entry/2024/10/01/093531)
- **MATLAB File Exchange**: [Outlier-Robust State Estimator: JCMSI 2021](https://jp.mathworks.com/matlabcentral/fileexchange/182942-outlier-robust-state-estimator-jcmsi-2021)
- **Research page**: [MCV Observer for Overcoming Outliers](https://www.control-theory.com/en/mcv-observer)

---

## Requirements

- **MATLAB** (R2020a or later recommended)
- **Control System Toolbox**
- **Robust Control Toolbox** (for LMI-based designs in `04_hinf_filter`, `05_multirate_observer`, `08_multirate_kalman`)

---

## How to Use

1. Navigate to the folder of interest (e.g., `cd 05_multirate_observer`)
2. Open the `main_*.m` file in MATLAB
3. Run the script — each folder is self-contained
4. See the `README.md` in each folder for paper references and parameter explanations

---

## Related Resources

### Blog (blog.control-theory.com)

| Topic | Link |
|-------|------|
| **Hub: State Observer and State Estimation** | [blog.control-theory.com/entry/state-observer-estimation](https://blog.control-theory.com/entry/state-observer-estimation) |
| State Observer: Basic Mechanism | [blog.control-theory.com/entry/2024/02/28/100201](https://blog.control-theory.com/entry/2024/02/28/100201) |
| State Observer for State Space Model | [blog.control-theory.com/entry/2024/10/01/143305](https://blog.control-theory.com/entry/2024/10/01/143305) |
| Kalman Filter | [blog.control-theory.com/entry/kalman-filter](https://blog.control-theory.com/entry/kalman-filter) |
| H-infinity Filter | [blog.control-theory.com/entry/h-infinity-filter](https://blog.control-theory.com/entry/h-infinity-filter) |
| System Identification: Obtaining Dynamical Model | [blog.control-theory.com/entry/2024/10/03/151451](https://blog.control-theory.com/entry/2024/10/03/151451) |
| Multi-Rate Observer | [blog.control-theory.com/entry/multirate-observer-eng](https://blog.control-theory.com/entry/multirate-observer-eng) |
| MCV Observer | [blog.control-theory.com/entry/mcv-observer-eng](https://blog.control-theory.com/entry/mcv-observer-eng) |
| LMI and Controller Design | [blog.control-theory.com/entry/lmi-eng](https://blog.control-theory.com/entry/lmi-eng) |
| Model Error Compensator (MEC) | [blog.control-theory.com/entry/model-error-compensator-eng](https://blog.control-theory.com/entry/model-error-compensator-eng) |
| Discretization | [blog.control-theory.com/entry/discretization-eng](https://blog.control-theory.com/entry/discretization-eng) |

### Research Pages (www.control-theory.com)

- [Multi-rate System](https://www.control-theory.com/en/multi-rate-system)
- [MCV Observer for Overcoming Outliers](https://www.control-theory.com/en/mcv-observer)
- [Linear Matrix Inequality](https://www.control-theory.com/en/linear-matrix-inequality)
- [Model Error Compensator](https://www.control-theory.com/en/model-error-compensator)
- [Publications](https://www.control-theory.com/en/publications)

### MATLAB File Exchange

- [State Estimation under Multi-Rate Sensing: IEEE ACCESS 2023](https://jp.mathworks.com/matlabcentral/fileexchange/182941-state-estimation-under-multi-rate-sensing-ieee-access-2023)
- [Outlier-Robust State Estimator: JCMSI 2021](https://jp.mathworks.com/matlabcentral/fileexchange/182942-outlier-robust-state-estimator-jcmsi-2021)

### Code Ocean

- [Multi-Rate System Code](https://codeocean.com/capsule/3611894/tree/v1)

### Video

- [YouTube: Control Engineering Channel](https://www.youtube.com/channel/UC121T0-DD2KBuqxWx2GGRkg)
- [Video Portal](https://www.portal.control-theory.com/)

---

## Migration Note

This repository consolidates and extends the previous repository [MATLAB_state_estimation](https://github.com/Hiroshi-Okajima/MATLAB_state_estimation) (MCV observer code). The original repository remains available but is no longer updated. New code and updates will be added here.

---

## GitHub Topics (for discoverability)

`state-observer` `state-estimation` `kalman-filter` `luenberger-observer` `h-infinity` `lmi` `multirate-systems` `sensor-fusion` `robust-estimation` `control-engineering` `matlab` `matlab-codes` `observer-design` `cyclic-reformulation` `outlier-robust`

---

## Citation

If you use these codes in your research, please cite the corresponding paper(s) listed in each folder's README.

---

## Author

**Hiroshi Okajima** — Associate Professor, Graduate School of Science and Technology, Kumamoto University. Member of SICE, ISCIE, and IEEE.

- [www.control-theory.com](https://www.control-theory.com/en)
- [Blog](https://blog.control-theory.com/)
- [YouTube](https://www.youtube.com/channel/UC121T0-DD2KBuqxWx2GGRkg)
- [GitHub](https://github.com/Hiroshi-Okajima)

---

## License

MIT License
