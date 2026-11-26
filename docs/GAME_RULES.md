# BIOBUZZ (2026–2027) Competition Rules Summary

*FIRST® Tech Challenge, FIRST® CANOPY™ season. Game: BIOBUZZ presented by RTX.*

**Source of truth:** the official Competition Manual, version **TU10**.
- HTML: `https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU10.htm`
- PDF: `https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual`
- Team Updates (published every Thursday, may change rules): `https://ftc-resources.firstinspires.org/ftc/game/tu-combined`

**How to use this file:** it is a summary for developers. If it disagrees with the manual, the manual wins. Rule IDs (G###, R###) refer to the manual. Defined terms are ALL CAPS in the manual; this file uses them the same way.
- **For official GDC rulings, edge cases, and referee interpretations**, see [`docs/QA_RULINGS.md`](QA_RULINGS.md).

**Verification status (last checked 2026-11-25):** checked against the full text of the manual (Sections 1–16). TU10 is the current manual version, and earlier Team Updates (TU01–TU09) are already folded into it. Team Updates publish every Thursday, so check for anything newer than TU10. **Not seen:** figures, diagrams and CAD references (text only). See [Open items](#10-open-items).

> **Spirit of the rules.** BIOBUZZ is deliberately written around "intended gameplay". Referees make good-faith calls "as perceived", and many penalties escalate only if an action is **STRATEGIC** (done to gain a competitive advantage; accidents cannot be STRATEGIC). The violation tables below are not an exhaustive list of what can be penalized. Any rule can escalate to a YELLOW or RED CARD (G205).

---

## Game Overview (Section 8)

Two ALLIANCES of 2 teams collect POLLEN and NECTAR, **LAUNCH** them into the CELLS of their HIVE, and place them into FLOWERS.

- **AUTO (30 s):** robots run autonomously and may use sensors to collect POLLEN from FLOWERS and navigate to LAUNCH into their HIVE's CELLS. When enough elements are launched into a CELL, the HIVE tips. Each tip earns points and **unlocks additional NECTAR**.
- **TELEOP (2:00):** drivers take over, keep collecting and scoring, tip the HIVE for points, and unlock more NECTAR.
- **Late match:** alliances fill FLOWERS with POLLEN and NECTAR. Placing NECTAR claims ownership (the top-most NECTAR owns the FLOWER) and earns the Bottom NECTAR Bonus.
- The alliance with the most points wins. Extra RANKING POINTS come from other scoring achievements.

---

## 1. Match Structure & Timing

2 ALLIANCES of 2 teams each (3 robots per alliance at FIRST Championship, 2 play each match). A match is 30 s + 8 s + 120 s.

| Period | Duration | Robot control | Notes |
| :--- | :--- | :--- | :--- |
| **AUTO** | 0:30 | Pre-programmed only; no driver input (G401) | Select an OpMode with the **30 s AUTO timer enabled** and press INIT (G305). |
| **Transition** | 0:08 | **No powered movement of robot or any mechanism** (G403) | Scoring buffer only. There is **no field reset**. Drivers may press Driver Station buttons (stop AUTO, INIT/start TELEOP). |
| **TELEOP** | 2:00 | Driver-controlled | NECTAR may enter FLOWERS only in the **last 60 s** (G410). |
| **End of match** | 0:00 | **Motionless** (G404) | Stop all powered movement until the Head REFEREE releases robots. |

Event-system audio cues (Table 9-1). The visual FIELD timer is the authority if they differ:

| Event | Timer value | Cue |
| :--- | :--- | :--- |
| Match start | 2:30 | "Cavalry Charge" |
| AUTO ends | 2:00 | Buzzer ×3 |
| Transition | 0:08 → 0:01 | "Drivers, pick up your controllers, 3-2-1" |
| TELEOP begins | 2:00 | 3 bells |
| FLOWER ownership unlocked | 1:00 | Chime |
| Final 20 seconds | 0:20 | Train whistle |
| Match end | 0:00 | 3-second buzzer |

---

## 2. Field & Game Elements

- **FIELD:** about 144 in × 144 in inside the walls, floored with **36 interlocking foam TILES**, each about 24 in × 24 in × 0.59 in (a 6×6 grid). FIELD columns **A, B, C = red side**, **D, E, F = blue side** (G304, G402). The red ALLIANCE AREA is on the **left** from the primary audience viewing direction.
- **FIELD elements:** 1 **HIVE Structure** in the center (a frame holding 2 HIVES, one per alliance) and **4 FLOWERS** attached to the perimeter wall.
- **ALLIANCE AREA:** about 97 in × 54 in, outside the field. **LOADING ZONE:** about 23 in × 11 in, bounded by tape and the perimeter walls; it belongs to the alliance with the adjacent ALLIANCE AREA. **GARDEN:** about 23 in × 2 in strip, in opposite corners of the field. Tape is red and electric blue, 1 or 2 in wide, and the zones include their tape lines.
- **POLLEN:** yellow balls, about 2.8 in (am-5851). **40** per match. **NECTAR:** red and blue balls, about 3.6 in (am-5852). **8 red + 8 blue** per match. The elements are not perfectly spherical and vary in size.
- **HIVE:** bi-stable structure with two same-color CELLS on a pivot. One CELL points up, one points down at start. The CELL that "points at" a FLOWER starts tilted down. It holds its position until enough POLLEN or NECTAR is LAUNCHED into the upward-facing CELL.
- **FLOWER:** vertical structure with top, middle and bottom rings. Elements go in at the top; only POLLEN (never NECTAR) can be removed from the bottom (G418).

### Key dimensions (nominal; tolerance ±1 in unless stated)

The official reference is the **3D CAD model**; also see the Event Field Setup Guide, Field Acceptance Checklist and Field Mitigation Guide on the Playing Field Resources page. Events assemble the field in different venues, so design for variation.

| Item | Dimension |
| :--- | :--- |
| HIVE frame | 49.46 in wide × 38.95 in deep at the base |
| HIVE pivot axis height | 43.95 in above the TILES |
| Distance between the 2 CELLS of a HIVE | about 18.8 in |
| CELL opening | about 20 in wide × 14 in tall × 12 in deep |
| FLOWER top opening | about 4 in diameter, about **21.5 in** above the TILES |
| FLOWER top backstop | 1.25 in tall |
| FLOWER retrieval opening (bottom) | about 3.55 in tall × 3.57 in deep |
| FLOWER lower ring | about 0.4 in tall, with a 2.79 in diameter hole that seats a POLLEN |
| FLOWER upper and middle rings | connected by 4 HIPS pipes |

At FIRST Championship the fields are raised on risers about 24 in off the floor, while the ALLIANCE AREAS stay at ground level (15.2).

### Starting setup (10.3.1)

| Element | Count |
| :--- | :--- |
| POLLEN in each of the 4 FLOWERS | 4 each (16) |
| POLLEN in each GARDEN (red, blue) | 4 each (8) |
| POLLEN pre-loaded per ROBOT | 4 each (16) |
| NECTAR in each upward-facing CELL (matching color) | 3 each (6) |
| NECTAR in each ALLIANCE AREA (matching color) | 5 each (10) |

### NECTAR entry by the HUMAN PLAYER (G426, G427)
- One NECTAR per **tip of your own-color HIVE**, **or** all remaining NECTAR once **60 s or less** remain, whichever comes first.
- Only by a DRIVE TEAM member of that alliance, without a tool, and the NECTAR must **hit the TILE inside the LOADING ZONE** before touching a robot or field element.
- Humans may never score into a HIVE or FLOWER, or remove elements from the field (G428).
- NECTAR that leaves the field is returned to the alliance's drive team and can be re-entered any time. POLLEN that leaves the field is re-introduced by FIELD STAFF.

---

## 3. Scoring (Table 10-2) and Criteria

| Action | AUTO | TELEOP | When assessed | Criteria |
| :--- | :---: | :---: | :--- | :--- |
| **LEAVE** | 3 | — | End of AUTO | Robot is **no longer contacting the perimeter wall**. |
| **PARK** | 5 | 5 | End of AUTO / end of match | Robot is **at least partially in the LOADING ZONE**. |
| **HIVE TIP** | 20 | 20 | Throughout; until everything is at rest at match end | See below. A tip completed before TELEOP starts counts as AUTO. |
| **POLLEN / NECTAR left in an upward-facing CELL** | — | 2 each | After all elements and robots are at rest | Per element. |
| **FLOWER: Bottom NECTAR Bonus** | — | 5 | Throughout; final at end of TELEOP | Alliance owning the bottom-most qualifying NECTAR **of its color**. |
| **FLOWER: element in an owned FLOWER** | — | 2 each | Throughout; final at end of TELEOP | Per POLLEN or NECTAR. |
| **GARDEN: element in GARDEN** | — | 1 each | End of TELEOP, at rest | POLLEN or NECTAR at least partially in the zone. |

### HIVE TIP
A HIVE is TIPPED when (a) it moves from one stable state to the other, so the downward CELL becomes the upward CELL, **and** (b) the damper that was not touching the frame begins to touch it.
- **Only** LAUNCHING into the **upward-facing CELL** may cause a tip (G417). LAUNCH means shot into the air, propelled across the floor to a target, or thrown forcefully.
- Launching at the *downward* CELL while the HIVE is tipping can disrupt the tip. Pause launching so the tip is obvious to volunteers.

### FLOWER
- Elements score while **at least partially between the top ring and the middle ring** (the scoring volume).
- **Owner:** the alliance with the **top-most NECTAR of its color** that meets the criteria owns that FLOWER and earns 2 points for *every* POLLEN and NECTAR in it, **regardless of who placed them**.
- Scoring can only begin with 1:00 remaining (G410). Elements placed earlier still count but draw penalties.
- Robots end the match claiming ownership by placing NECTAR and returning to their LOADING ZONE.

### GARDEN
Not a protected zone: **either alliance may remove elements from either GARDEN.** Elements score for the GARDEN's color, regardless of who placed them or whose NECTAR it is.

---

## 4. Ranking Points & Ranking

| RP | Award | FIRST Championship | Regional Championships | All other events* |
| :--- | :---: | :--- | :--- | :--- |
| **SWARM** | 1 | TBA | ≥ 16 LEAVE+PARK points (alliance total) | ≥ 16 |
| **POLLINATOR 1** | 1 | TBA | ≥ 4 TIPS | ≥ 4 |
| **POLLINATOR 2** | 1 | TBA | ≥ 7 TIPS | ≥ 7 |
| **WIN** | 3 | | | |
| **TIE** | 1 | | | |

LOSS earns 0 RP (implied; not listed in the table). *Premier Events may set their own thresholds. Thresholds for Regionals and Championship are announced in Team Updates.

**Qualification ranking (Table 13-1):** Ranking Score (average RP, excluding SURROGATE matches) → average alliance match points excluding fouls → average **TIPS** → average **AUTO points** → random. Events schedule 5 or 6 qualification matches per team. Playoffs are double elimination (no RP).

---

## 5. Violations & Penalties

| Penalty | Effect |
| :--- | :--- |
| VERBAL WARNING | Warning only. |
| **MINOR FOUL** | **5 points** to the opponent. |
| **MAJOR FOUL** | **20 points** to the opponent. |
| YELLOW CARD | Warning. A **second yellow becomes a RED CARD**. Cards carry into later matches; in Playoffs they apply to the whole alliance. |
| RED CARD | **DISQUALIFIED** for the match (0 match points and 0 RP in qualifications). |
| DISABLED | Referee orders the robot stopped; all outputs deactivated. |

Duration words: **MOMENTARY** < ~3 s, **CONTINUOUS** > ~10 s, **REPEATED** = more than once in a match. Penalties apply per instance unless noted.

### Robot gameplay rules (most relevant to code and strategy)

| Rule | Summary | Violation |
| :--- | :--- | :--- |
| G204 | Don't force the opponent to break a rule | MAJOR per instance (+YELLOW per match if REPEATED); victim not penalized |
| G401 | No driver contact with robot or console in AUTO (except start/stop) | VW; MAJOR + YELLOW per match if STRATEGIC |
| G402 | No disrupting opponent AUTO on their side (controlling their elements, touching their robot) | MAJOR per match; + YELLOW if STRATEGIC |
| G403 | No powered movement in the 8 s transition | VW; MAJOR + YELLOW if STRATEGIC |
| G404 | No powered movement after TELEOP ends | VW; MAJOR + YELLOW if STRATEGIC |
| G405 | Don't deliberately eject elements from the field | MAJOR per element |
| G407 | **Never CONTROL more than 4 elements at once** | VW; MAJOR + YELLOW per match if STRATEGIC |
| G408 | Don't CONTROL opponent NECTAR | VW; YELLOW per match if STRATEGIC |
| G409 | Don't catch elements released by a tipped HIVE before they touch something else | VW; YELLOW per match if STRATEGIC |
| G410 | **NECTAR into a FLOWER only in the last 60 s** (POLLEN is not restricted by this rule) | **MAJOR per NECTAR** |
| G411 | Don't keep opponents from reaching more than 8 elements | VW; MAJOR + YELLOW per match if STRATEGIC |
| G412 | Robot under control; no hazard outside the field | DISABLED + VW |
| G415 | No grabbing, grasping, attaching to or hanging from field elements | VW; YELLOW per match if STRATEGIC; DISABLED if damage likely |
| G416 | Respect expansion limits (R105) | VW; MAJOR per instance if STRATEGIC |
| G417 | Affect the HIVE only by LAUNCHING into an upward CELL | VW; MAJOR + YELLOW per match if STRATEGIC |
| G418 | Enter FLOWERS only at the top; remove only POLLEN from the bottom | VW; MAJOR + YELLOW per match if STRATEGIC |
| G419 | Don't damage or functionally impair an opponent | VW; MAJOR + YELLOW if STRATEGIC; MAJOR + RED if STRATEGIC and opponent can't drive |
| G420 | Don't attach to, tip or entangle an opponent | VW; MAJOR + YELLOW if STRATEGIC; MAJOR + RED if STRATEGIC and (CONTINUOUS or opponent can't drive) |
| G421 | **PIN limit 3 s** (count ends after 2 ft separation, or either robot moves 2 ft, for >3 s) | MAJOR + another MAJOR every 3 s uncorrected |

### Drive team / human rules

| Rule | Summary | Violation |
| :--- | :--- | :--- |
| G422 | Drive team stays in the ALLIANCE AREA | VW; MINOR per instance if STRATEGIC |
| G423 | Coaches and other teams don't handle gamepads | VW; MAJOR + YELLOW if STRATEGIC |
| G424 | Coaches don't touch scoring elements | VW; MINOR per instance if STRATEGIC |
| G425 | Drive team doesn't touch robots, field elements, or disrupt scoring | VW; MAJOR + YELLOW if STRATEGIC |
| G426 / G427 | NECTAR entered early / not through LOADING ZONE | **MINOR per NECTAR** |
| G428 | Humans don't remove elements from the field | MINOR per element |

---

## 6. Pre-Match Setup Requirements (G303–G305)

A robot must start the match:
- fully on its own alliance's side (columns A–C red, D–F blue) and **touching the perimeter wall**;
- not touching or inside a FLOWER scoring volume, and **not in the LOADING ZONE**;
- not attached to or entangled with any field element;
- confined to its STARTING CONFIGURATION (R102/R103);
- in contact with **exactly 4 pre-loaded POLLEN** (in/on the robot or on TILES touching it, all on its own side);
- **fully motionless after OpMode INIT completes**;
- carrying correct ALLIANCE-color ROBOT SIGNS.

Teams must select and INIT an OpMode every match (even with no AUTO), with the 30 s timer enabled if it is an AUTO OpMode.

---

## 7. Robot Construction Limits (Section 12, selected)

| Rule | Limit |
| :--- | :--- |
| R102 | STARTING CONFIGURATION fits fully within **18 in × 18 in × 18 in**. Pre-loaded POLLEN may stick out. |
| R103 | Robot must be self-supporting at start; may hold position via powered-off mechanics or an OpMode that pre-positions servos and motors. Avoid stalling motors for minutes. |
| R104 | No weight limit. |
| R105 | After start, robot must stay one assembly within **18 in × 24 in × 29 in (tall)** at all times, **physically constrained, not by software**. No deliberately detaching parts. |
| R204 | No downforce by grabbing the floor or by suction/airflow. |
| R301–R303 | COTS MAJOR MECHANISMS built for game tasks are prohibited (exceptions: COTS drive chassis, StarterBot parts). COTS parts must be single degree of freedom (listed exceptions apply). |
| R305 | No current-season scoring elements or replicas in robot construction. |
| R501 | Only listed motors (see table in manual). |
| R502 | Servos ≤ 8 W mechanical output and ≤ 4 A stall at 6 V; linear servos ≤ 1 A. |
| R503 | **Max 8 motors and 8 servos total** across all configurations. |
| R505 | Actuators only via approved power regulators (REV hubs, REV servo modules, etc.). |
| R701 | One ROBOT CONTROLLER: REV Control Hub, or Android phone + REV Expansion Hub. At most **one additional** Expansion Hub. |
| R702 | No altering coprocessor software. **Limelight 3A is allowed and programmable.** OpenMV, OAK-1 and Limelight 3G are prohibited. |
| R704 | Wi-Fi only via the official FTC apps. **Telemetry/debug data only through the Driver Station app. FTC Dashboard, Panels and similar streaming tools are prohibited. No continuous video stream.** |
| R705 | Name devices `<team>-RC` and `<team>-DS` (optional letter for spares). |
| R707 / R708 | USB only for webcams, vision coprocessors, hubs. **Single-sensor UVC webcams only; no stereo cameras.** |
| R710 | Lasers only if part of a sensor, Class I / exempt, non-visible. |
| R711 | Control Hub Wi-Fi password must be non-default; smartphones in Airplane Mode with Wi-Fi on, Bluetooth off. |
| R801 | No pneumatic actuators, compressors, vacuums or blowers (sealed gas springs are fine). |
| R904 | **No wireless gamepads.** The only wireless link is the RC–DS Wi-Fi connection. |
| E301 | At events, don't run your own Wi-Fi or Bluetooth (hotspots count). |

Re-inspection is **not** needed for code changes (I303.D) but is needed for most hardware changes.

---

## 8. AprilTags (Section 9.9)

- **36h11** family, **3.25 in (8.25 cm)** square.
- Arranged in **clusters of four tags on one sticker**, applied to the **bottom of each CELL**, facing down toward the TILES, with the cluster's bottom edge oriented toward the center of the field. Reference holes in the sticker allow measuring cluster position.
- Tags exist **only on the CELLS** (that is, on the HIVES).

| Tag IDs | Location |
| :--- | :--- |
| **30, 31, 32, 33** | Red CELL, side **opposite** the audience |
| **34, 35, 36, 37** | Red CELL, **audience** side |
| **38, 39, 40, 41** | Blue CELL, **audience** side |
| **42, 43, 44, 45** | Blue CELL, side **opposite** the audience |

Do not put imagery mimicking 36h11 tags on your robot (R202.C), and don't shine bright lights or lasers at the field or mimic tags during a match (G302.E).

---

## 9. Implications for Robot Code

1. **AUTO must be deterministic.** No driver input at all. LEAVE (3) needs only loss of wall contact; PARK (5) needs the robot at least partly in the LOADING ZONE at the 30 s mark. A HIVE TIP (20) must be finished before TELEOP to count as AUTO.
2. **Stop everything in the transition.** Command zero power and hold positions; don't run mechanisms. If the TELEOP OpMode's INIT twitches an actuator, press INIT only after TELEOP begins (G403).
3. **INIT must not move the robot** beyond holding its start configuration (G304.H, R103).
4. **Gate NECTAR delivery on the match clock.** Don't allow NECTAR into a FLOWER until ≤ 60 s remain (G410: MAJOR FOUL per NECTAR). Consider a software interlock tied to the TELEOP timer.
5. **Limit intake to 4 elements.** Sorting that returns extras is not a defense against REPEATED violations (G407). Combine a hardware guard with a software counter.
6. **Stop all motion at 0:00 of TELEOP** (G404). Call stop and set down the controllers.
7. **Aim only at the upward-facing CELL** when launching. Missing into the CELL's outside faces is not STRATEGIC if accidental, but deliberate hits are (G417).
8. **Vision:** AprilTags are on the CELLS only, at the HIVES, with the IDs above. Limelight 3A and UVC webcams are legal; no stereo cameras.
9. **Telemetry:** use only the Driver Station telemetry. No FTC Dashboard or Panels at competition; no video streaming (R704.D). Wired gamepads only (R904).
10. **Hardware budget:** at most 8 motors and 8 servos across all configurations (R503).
11. **Judging (Control Award, 6.3.7):** a team must submit a PORTFOLIO summarizing its control hardware and software (what each component or system solves and does) and use at least one solution with **external feedback** (sensors, vision, etc.). Full code is not required in the PORTFOLIO. AUTO, TELEOP or both can count.

---

## 10. Open Items

Items to confirm against the manual or later Team Updates:

- **Figures and CAD not seen:** the text of Section 9 is covered, but the diagrams are not. Tile seam and coordinate figures (9-4, 9-5), the FLOWER scoring-volume CAD (10-4), exact ring heights, and AprilTag positions in field coordinates (Figure 9-17, reference holes) are only in the figures and the 3D CAD model.
- **Loss = 0 RP** is implied, not stated in Table 10-2.
- **Newer Team Updates:** this file reflects TU10. Check the Team Updates page for any later release before relying on rule details.
- **FIRST Championship** RP thresholds and element counts may be modified (15.2).
- **Not summarized here:** wiring and power rules (R601–R613 details), ROBOT SIGN rules (R401–R403), fabrication details, awards (Section 6, except the Control Award note above), advancement (Section 4), event rules (Section 5), League play (Section 14), sponsor recognition (Section 7).

### Where to look in the manual

| Topic | Section |
| :--- | :--- |
| Field dimensions and tolerances | 9.1–9.2 |
| Zones, markings, TILE coordinates | 9.3–9.4 |
| ALLIANCE AREA | 9.5 |
| HIVE structure (frame, HIVE, CELL) | 9.6 |
| FLOWER | 9.7 |
| SCORING ELEMENTS | 9.8 |
| AprilTags | 9.9 |
| Setup and drive team roles | 10.2–10.3 |
| Scoring and point values | 10.5 |
| Violations and cards | 10.6 |
| Game rules (G) | 11 |
| Robot construction (R) | 12 |
| Ranking and playoffs | 13.6–13.7 |
| Game overview | 8 |
| Awards and judging | 6 |
| Glossary | 16 |