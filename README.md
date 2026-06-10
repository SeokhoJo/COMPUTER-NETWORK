# COMPUTER-NETWORK

## Latency-Critical Network Simulation for Autonomous Vehicle Platooning

This repository contains the source code for a computer network simulation project that evaluates how latency-critical networking technologies can improve emergency message delivery in an autonomous vehicle platooning scenario.

The project focuses on **TSN**, **DetNet**, **URLLC**, and **hybrid wired/wireless communication** under normal and failure-prone network conditions.

> **Demo Video:** [Project Demo Video](YOUR_VIDEO_LINK_HERE)

---

## 1. Problem Statement

Autonomous vehicle platooning requires fast and reliable communication between vehicles. When the lead vehicle suddenly brakes, the warning message must reach the following vehicles before the safety distance becomes too small.

In this context, network delay is not just a performance issue. It can directly affect the physical behavior of vehicles. If a critical packet is delayed or lost, the following vehicle may react too late, which can lead to a rear-end collision.

Therefore, this project investigates the following question:

> How can latency-critical network technologies reduce delay, improve reliability, and support collision avoidance in a vehicle platooning scenario?

---

## 2. Project Motivation

Traditional best-effort communication does not guarantee deterministic delay or reliable packet delivery. For safety-critical applications such as autonomous vehicle platooning, this limitation becomes significant.

Emergency braking messages have strict timing requirements. Even a small increase in delay or packet loss can reduce the available reaction time of the following vehicles.

To address this problem, this project compares several communication approaches:

* Baseline communication
* TSN-based wired communication
* TSN + DetNet deterministic wired communication
* URLLC-based wireless communication
* Hybrid wired/wireless communication

The goal is not only to compare network-level metrics such as delay and jitter, but also to connect those metrics to application-level safety outcomes such as collision occurrence and remaining vehicle gap.

---

## 3. Proposed Solution

This project applies latency-critical networking concepts to an emergency braking platoon scenario.

### 3.1 TSN: Time-Sensitive Networking

TSN is used to reduce delay and jitter in wired communication. In this project, TSN represents a deterministic wired networking mechanism that prioritizes safety-critical traffic.

The main idea is that emergency braking messages should not be treated as ordinary best-effort packets. Instead, they should be delivered with higher priority and more predictable timing.

### 3.2 DetNet: Deterministic Networking

DetNet extends deterministic communication across network paths. In this project, DetNet is used together with TSN to improve reliability and deterministic forwarding for critical traffic.

DetNet is especially meaningful when the network must support predictable delivery even under failure-prone conditions.

### 3.3 URLLC: Ultra-Reliable Low-Latency Communication

URLLC is used to represent reliable low-latency wireless communication between vehicles. In the platooning scenario, URLLC supports fast V2V message delivery when wireless communication is required.

This project models URLLC at a simulation level to evaluate how reliable wireless delivery can help following vehicles receive emergency braking messages in time.

### 3.4 Hybrid Communication

The hybrid approach combines wired and wireless communication paths. The purpose is to evaluate whether multiple communication mechanisms can improve robustness when one path becomes unstable.

The hybrid scenario is the most important part of this project because it compares different combinations of TSN, DetNet, and URLLC under normal and failure conditions.

---

## 4. Repository Structure

The current repository is organized as follows:

```text
COMPUTER-NETWORK/
├── README.md
└── src/
    ├── normal/
    │   └── run_baseline_platoon.py
    │
    ├── TSN/
    │   ├── run_tsn_platoon.py
    │   └── run_tsn_failure_platoon.py
    │
    ├── TSN+DetNet/
    │   ├── run_tsn_detnet_platoon.py
    │   └── run_tsn_detnet_failure_platoon.py
    │
    ├── URLLC/
    │   ├── run_wireless_urllc.py
    │   └── run_wireless_urllc_failure.py
    │
    └── HYBRID/
        ├── normal/
        │   ├── run_hybrid_baseline_platoon.py
        │   ├── run_hybrid_full_platoon.py
        │   ├── run_hybrid_tsn_platoon.py
        │   ├── run_hybrid_tsn_detnet_platoon.py
        │   ├── run_hybrid_tsn_urllc_platoon.py
        │   └── run_hybrid_urllc_platoon.py
        │
        ├── wired_failure/
        │   └── hybrid wired-failure simulation scripts
        │
        ├── wireless_failure/
        │   └── hybrid wireless-failure simulation scripts
        │
        │
        └── combined_failure/
            ├── run_hybrid_baseline_combined_failure_platoon.py
            ├── run_hybrid_full_combined_failure_platoon.py
            ├── run_hybrid_tsn_combined_failure_platoon.py
            ├── run_hybrid_tsn_detnet_combined_failure_platoon.py
            ├── run_hybrid_tsn_urllc_combined_failure_platoon.py
            └── run_hybrid_urllc_combined_failure_platoon.py
```

---

## 5. Simulation Design

The simulation is based on a vehicle platooning scenario.

A lead vehicle performs emergency braking. After this event, an emergency message is transmitted to the following vehicles. Each vehicle reacts after receiving the message and applying a reaction delay.

The main simulation logic is:

1. Vehicles move in a platoon.
2. The lead vehicle detects an emergency braking event.
3. A critical warning message is transmitted.
4. Following vehicles receive the message through wired, wireless, or hybrid paths.
5. Vehicles react based on message reception timing.
6. The simulation checks whether collision occurs.

This design connects network behavior with physical vehicle safety.

---

## 6. Network Scenarios

This project evaluates four major network conditions.

| Scenario         | Description                                                                                     |
| ---------------- | ----------------------------------------------------------------------------------------------- |
| Normal           | Both wired and wireless communication paths operate normally.                                   |
| Wired Failure    | The wired path is degraded or partially failed, while wireless communication remains available. |
| Wireless Failure | Wireless V2V communication is degraded, while the wired path remains available.                 |
| Combined Failure | Both wired and wireless communication paths experience failures.                                |

The combined failure scenario is the most extreme case because both communication paths are degraded.

---

## 7. Compared Network Configurations

The project compares multiple network configurations.

| Configuration | Main Idea                                                       |
| ------------- | --------------------------------------------------------------- |
| Baseline      | Best-effort communication without deterministic control         |
| TSN           | Wired low-latency communication using time-sensitive scheduling |
| TSN + DetNet  | Deterministic wired communication with improved reliability     |
| URLLC         | Reliable low-latency wireless V2V communication                 |
| TSN + URLLC   | Combination of wired TSN and wireless URLLC                     |
| Full Hybrid   | TSN + DetNet + URLLC combined                                   |

The full hybrid configuration is designed to provide the strongest fault tolerance by combining deterministic wired networking and reliable wireless communication.

---

## 8. Evaluation Metrics

The simulation evaluates both network-level and safety-level metrics.

| Metric           | Meaning                                                          |
| ---------------- | ---------------------------------------------------------------- |
| End-to-End Delay | Time required for a critical message to reach the receiver       |
| Jitter           | Variation in packet delay                                        |
| Packet Reception | Whether each vehicle successfully receives the emergency message |
| Collision Result | Whether a rear-end collision occurs                              |
| Minimum Gap      | Minimum distance between vehicles during the scenario            |
| Final Gap        | Remaining vehicle distance after stopping                        |
| Reaction Map     | Mapping of which vehicles reacted and when                       |

This is important because a network with low delay is not automatically safe. The final goal is to determine whether the communication system allows vehicles to react early enough to avoid collision.

---

## 9. Result Interpretation

The core interpretation of this project is that communication performance directly affects vehicle safety.

If the emergency braking message arrives early, the following vehicles can react in time. If the message is delayed or lost, the following vehicles continue moving without braking, which reduces the safety gap and increases collision risk.

The expected trend is:

```text
Lower delay + higher packet reception
        ↓
Earlier vehicle reaction
        ↓
Larger remaining safety gap
        ↓
Lower collision risk
```

Therefore, the simulation results should be interpreted from both perspectives:

1. **Network perspective**: delay, jitter, packet reception
2. **Safety perspective**: collision result, minimum gap, final gap

---

## 10. Referenced Result Graphs

The following graphs are recommended for explaining the result in a professional report or presentation.

| Graph                             | Purpose                                                                                  |
| --------------------------------- | ---------------------------------------------------------------------------------------- |
| Delay Comparison Graph            | Compares end-to-end delay across Baseline, TSN, DetNet, URLLC, and Hybrid configurations |
| Collision Result Graph            | Shows whether each spacing and scenario caused collision                                 |
| Minimum Gap Graph                 | Shows the minimum distance between vehicles during emergency braking                     |
| Failure Scenario Comparison Graph | Compares Normal, Wired Failure, Wireless Failure, and Combined Failure                   |
| Hybrid Configuration Graph        | Compares Baseline, TSN, TSN+DetNet, URLLC, TSN+URLLC, and Full Hybrid                    |

If graph images are added later, the recommended directory structure is:

```text
assets/
├── delay_comparison.png
├── collision_result.png
├── minimum_gap_comparison.png
├── failure_scenario_comparison.png
└── hybrid_configuration_comparison.png
```

Recommended Markdown format:

```md
![Delay Comparison](assets/delay_comparison.png)
![Collision Result](assets/collision_result.png)
![Minimum Gap Comparison](assets/minimum_gap_comparison.png)
![Failure Scenario Comparison](assets/failure_scenario_comparison.png)
![Hybrid Configuration Comparison](assets/hybrid_configuration_comparison.png)
```

---

## 11. Logical Deduction

The simulation is based on the following logical chain:

1. Emergency braking requires fast message delivery.
2. Network delay increases the reaction time of following vehicles.
3. Packet loss prevents some vehicles from reacting.
4. Delayed or missing reactions reduce the safety gap.
5. A smaller safety gap increases the probability of collision.
6. Therefore, improving latency and reliability can improve platoon safety.
7. TSN improves deterministic wired latency.
8. DetNet improves deterministic reliability.
9. URLLC improves wireless low-latency delivery.
10. Hybrid communication improves fault tolerance by combining multiple paths.

This reasoning connects communication-layer performance with physical-layer safety outcomes.

---

## 12. Trade-offs

### 12.1 TSN

TSN can reduce delay and jitter for critical traffic. However, it requires scheduling and configuration. This makes it suitable for controlled wired environments, but it may be less flexible in highly dynamic environments.

### 12.2 DetNet

DetNet improves deterministic delivery and can provide better reliability. However, deterministic routing and packet replication can increase configuration complexity and network overhead.

### 12.3 URLLC

URLLC improves low-latency wireless delivery. However, real wireless environments are affected by signal interference, mobility, distance, and channel conditions. Therefore, URLLC is powerful but difficult to guarantee perfectly in every real-world condition.

### 12.4 Hybrid Approach

The hybrid approach improves fault tolerance by combining wired and wireless paths. However, it also increases system complexity because multiple technologies must be coordinated.

The main trade-off is:

```text
Higher reliability and lower latency
        vs.
Higher implementation complexity and resource overhead
```

---

## 13. Key Findings

The project supports the following conclusions:

* Best-effort communication is not sufficient for safety-critical platooning scenarios.
* TSN is effective for reducing wired communication delay and jitter.
* DetNet improves deterministic reliability when combined with TSN.
* URLLC is important when wireless V2V communication must deliver emergency messages quickly.
* Hybrid communication is more robust than relying on a single network path.
* The most extreme scenario is combined failure, where both wired and wireless paths are degraded.
* Network performance should be evaluated together with vehicle safety outcomes.

---

## 14. Limitations

This project is based on simulation rather than real vehicle deployment. Therefore, the results should be interpreted as a controlled comparison of networking mechanisms, not as a complete real-world guarantee.

The URLLC model focuses on the principle of reliable low-latency wireless communication. It does not fully implement all physical-layer details of real 5G URLLC.

The vehicle platooning scenario is also simplified to focus on the relationship between communication delay and emergency braking safety.

---

## 15. Future Work

Future work may include:

* More realistic wireless channel modeling
* Larger vehicle platoons
* Multi-lane traffic scenarios
* More detailed 5G C-V2X modeling
* More advanced DetNet replication strategies
* Real-time visualization dashboard
* Automated result graph generation
* Comparison with IEEE 802.11p-based V2V communication

---

## 16. How to Run

Each simulation script can be executed directly with Python.

Example:

```bash
python src/normal/run_baseline_platoon.py
```

TSN example:

```bash
python src/TSN/run_tsn_platoon.py
```

TSN + DetNet example:

```bash
python "src/TSN+DetNet/run_tsn_detnet_platoon.py"
```

URLLC example:

```bash
python src/URLLC/run_wireless_urllc.py
```

Hybrid full example:

```bash
python src/HYBRID/normal/run_hybrid_full_platoon.py
```

Combined failure full hybrid example:

```bash
python src/HYBRID/combined_failure/run_hybrid_full_combined_failure_platoon.py
```

---

## 17. Demo Video

The project demonstration video is available here:

[Project Demo Video](https://drive.google.com/file/d/14GkgQ983STuGmYNchsO-v1lvOU54-8WZ/view?usp=drive_link)

---

## 18. References

* IEEE 802.1 Time-Sensitive Networking Task Group, *Time-Sensitive Networking*
* IETF DetNet Working Group, *Deterministic Networking Architecture*
* 3GPP, *Ultra-Reliable and Low-Latency Communications*
* SUMO Documentation, *Simulation of Urban MObility*
* OMNeT++ Documentation
* INET Framework Documentation
* ns-3 Network Simulator Documentation

---

## 19. Summary

This project demonstrates that latency-critical networking is essential for autonomous vehicle platooning. By comparing Baseline, TSN, DetNet, URLLC, and Hybrid configurations, the simulation shows how communication delay and packet loss can influence physical vehicle safety.

The main conclusion is that hybrid communication provides stronger robustness under failure conditions because it combines deterministic wired communication and reliable low-latency wireless communication.
