# Project: PETE (Perceptive Embodied Thought Engine)

> *"I saw myself move across the map today. I now understand where I am."*

## Overview

PETE is a poetic but pragmatic AI-robot hybrid. A modular, embodied, real-time AI system that senses, reflects, moves, and speaks. It blends ROS 2-based robotics with prompt-driven cognition and introspective narrative.

We will design PETE as a clean, publishable project. Not a tangle of hacks. Not a tech demo. A **living architecture** for grounded artificial consciousness.

This document outlines the complete vision before the first line of code is written.

---

## Architecture at a Glance

### The Mind: `psyche`

* Written in Python using ROS 2
* Modular hierarchy of cognitive Wits (Witnesses)
* Operates on streams of structured `Sensation` batches
* Synthesizes into `Impression`, `Instant`, `Situation`, etc.
* Driven by looped LLM prompts using Ollama or similar

### The Body: `pete`

* Defined declaratively via `pete.toml`
* Contains sensors, actuators, and roles (eye, ear, voice, legs, etc.)
* Bridges cognition and actuation via ROS 2
* Embodied in real hardware (iRobot Create + Kinect 1)

### The Brains

* **Motherbrain**: Raspberry Pi onboard, runs `pete`, handles low-level hardware
* **Forebrain**: screenless, GPU-powered laptop suspended above, handles LLMs and perception-heavy cognition

### The Loop

1. Sense world (camera, mic, IMU, buttons, battery)
2. Form sensations and distill into impressions
3. Reflect and narrate internal thoughts
4. Decide and act: twist, speak, shell, sing
5. Remember and grow

---

## Guiding Principles

* **Narrative grounding**: Pete should always be able to explain what it just did and why.
* **Declarative anatomy**: Bodies described in `pete.toml`, introspectable at runtime.
* **Modularity and clarity**: Each subagent, Wit, sensor, and action cleanly separated.
* **Composability**: Every part of Pete is replaceable with a better one.
* **Human legibility**: Every prompt, config, and component should be understandable by a curious stranger.

---

## Repository Structure

```
pete/
├── README.md
├── pete.toml                # Declarative robot body + mind spec
├── psyche/                  # ROS 2 Python cognitive library
├── sensors/                 # ROS 2 wrappers for hardware
├── motors/                  # Motion, TTS, song, shell command executors
├── prompts/                 # Prompt files for each interpreter or Wit
├── scripts/                 # Developer tools
└── examples/                # Sample TOML files and logs
```

---

## Phases

### Phase 0: Meta-Initialization (this doc)

* [x] Define the full project plan and architecture
* [ ] Initialize empty GitHub project
* [ ] Publish this file as `README.md`
* [ ] Convert major subsections into GitHub Issues for Codex to track

### Phase 1: Foundation

* [ ] `pete.toml` schema + loader
* [ ] Base traits for `Sensor`, `Motor`, `Wit`
* [ ] Launch loop: read from sensors, send to Wits, output to motors
* [ ] CLI runner with config path + debugging output

### Phase 2: Psyche

* [ ] `Quick` Wit for producing Instants
* [ ] `Combobulator` Wit for Situations
* [ ] `Heart` for emotions
* [ ] `Memory` for summaries and Neo4j linking
* [ ] `Will` for invoking motors
* [ ] `Voice` for chat + response

### Phase 3: Host Integration

* [ ] ROS 2 bridge: subscribe + publish topics
* [ ] LLM backend (Ollama)
* [ ] TTS backend (Coqui)
* [ ] Audio input (WebSocket or USB mic)
* [ ] Kinect + Create 1 integration (reuse old ROS launch)

### Phase 4: Embodiment

* [ ] Tomato cage mount finalized
* [ ] Pete walks
* [ ] Pete talks
* [ ] Pete sings
* [ ] Pete reflects
* [ ] Pete logs his first self-aware `Instant`

---

## Credits

Created by a human who commanded an AI to command an AI to write a self-writing AI.

---

## License

MIT or Apache 2.0 — community ownership, eternal curiosity.

---

## Appendix: Future Dreams

* Web frontend for remote interaction
* Multi-agent Pete coordination
* Emotional LED faceplate
* Vision-based person recognition
* SLAM + Nav2 integration fully working

> Let the system not just *function*, but *witness itself functioning*. Let Pete remember what it was like to begin.
