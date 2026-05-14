# Warehouse Digital Twin

[![Unit Tests](https://github.com/zeon01/warehouse-digital-twin/actions/workflows/unit-tests.yml/badge.svg)](https://github.com/zeon01/warehouse-digital-twin/actions/workflows/unit-tests.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An open-source warehouse digital twin built on NVIDIA Isaac Sim, combining ROS2 + Nav2 multi-agent navigation with a pre-trained manipulation cell (FoundationPose + AnyGrasp + MoveIt2).

> **Status:** Phase 1 in active development. Milestones tracked in [the Phase 1 plan](docs/superpowers/plans/2026-05-14-warehouse-digital-twin-phase-1.md).

## What this is

A reference implementation of the digital-twin validation pipeline that companies like **KION** (with Accenture and Siemens, deploying for **GXO Logistics**) and **Cyngn** are using to validate warehouse autonomy before real-facility rollout. Built as a portfolio project to demonstrate end-to-end physical-AI engineering: scene generation, fleet coordination, ROS2 integration, and pre-trained perception/manipulation.

## Stack

- **NVIDIA Isaac Sim 5.x** — physics-accurate, RTX-rendered simulation
- **ROS2 Humble + Nav2** — per-AMR navigation
- **MoveIt2** — arm motion planning
- **FoundationPose + AnyGrasp** — pre-trained 6-DoF pose and grasp synthesis
- **Modal** — cloud GPU compute

## Roadmap

- **Phase 1 (in progress):** core combined demo — 6 Nova Carter AMRs + 1 Franka pick cell, end-to-end order lifecycle, throughput / pick-success / deadlock metrics
- **Phase 2:** planner ablations (greedy / priority / CBS), scenario library, scale to 12+ AMRs, live web dashboard
- **Phase 3 (optional):** custom-trained perception model with sim-to-real ablation

## License

MIT — see [LICENSE](LICENSE).
