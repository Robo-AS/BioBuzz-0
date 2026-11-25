# BIOBUZZ (2026–2027) Official Q&A Rulings & Interpretations

*Official interpretations from the FIRST Tech Challenge Game Design Committee (GDC).*  
*Source Reference: `docs/reference/Team Q&A - FIRST Game Q&A.html`*

---

## 1. Autonomous & Timing Edge Cases

### LEAVE Evaluation Timing (Q#21, Q#28, Q#47)
- **Evaluated ONLY at End of AUTO (Q#21)**: LEAVE is **not** a real-time event. The referee evaluates: *"Is the robot touching the perimeter wall when the 30-second buzzer sounds?"*
  - **CRITICAL AUTONOMOUS RISK**: If a robot leaves the perimeter wall, performs actions, and then drives to the LOADING ZONE to PARK and ends up **touching the perimeter wall at $t = 30\text{ s}$, LEAVE points (3 pts) are NOT awarded!**
  - **Autonomous Code Guideline**: Program all Auto parking waypoints to stop a few inches short of physical contact with the perimeter wall.
- **Transitive Contact is Permitted (Q#47)**: If at the end of AUTO the robot is not directly touching the wall, but a POLLEN or NECTAR is wedged between the robot and the wall touching both, **LEAVE is awarded**.
- **Partner Assistance (Q#28)**: If a robot is pushed or pulled off the wall by its alliance partner during Auto, **LEAVE is awarded**.

### NECTAR Introduction During AUTO (Q#11)
- If an alliance causes their HIVE to TIP during AUTO, the **Human Player IS permitted to introduce a NECTAR into the Loading Zone during AUTO** (per Section 10.1 and G426).

---

## 2. Element Control & Possession Limits (G407 & G408)

### No "Momentary Exception" for Over-Possession (Q#3, Q#4, Q#23, Q#29)
- **Zero Grace Period**: The rules do **not** provide a 3-second momentary exception for controlling a 5th element or opponent NECTAR. Any control of an illegal element is an immediate violation (Verbal Warning minimum).
- **Active Sorting / Rejection Trap (STRATEGIC Escalation)**: The GDC clarified that designing an intake system that routinely intakes 5+ elements and "reverses" to spit one back out, or routinely intakes opponent NECTAR to color-sort and eject it, **will be ruled a STRATEGIC violation (MAJOR FOUL + YELLOW CARD)** because it is a foreseeable, deliberate competitive advantage.
- **Robot Design Requirement**: Intakes and hoppers should use mechanical limiters or optical sensors to physically stop intaking once 4 elements are held.

### Elements Under Chassis or Intake Count as CONTROLLED (Q#34)
- Any element fully supported by, trapped in, or stuck under an intake/chassis is considered **CONTROLLED** under Section 16, counting toward the 4-element limit.

### Shot Deflection is NOT Control (Q#2, Q#40)
- Deflecting an opponent's shot with a defensive plate or shield does **not** count as "controlling" the deflected ball, provided the deflection is not deliberately aimed into a specific preferred target.

---

## 3. Defense & Field Interactions

### Shot-Blocking Mechanisms are Legal (Q#2, Q#8, Q#20)
- Extending a shield or plate (strictly within the 18"×24"×29" expansion envelope) to block or deflect an opponent's shots into the HIVE is **legal** and not an inherent violation of G407, G408, or competition spirit.
- Must not cause physical damage or functional impairment (G419) or entangle/tip the opponent (G420).

### High-Contact Gameplay Allowed Around Flowers (Q#42)
- BIOBUZZ is an interactive, high-contact game. Pushing, contacting, or playing defense against an opponent interacting with a FLOWER or retrieving pollen from the bottom is **legal**, provided it does not violate G419 (damage/impairment), G420 (entanglement/tipping), or G421 (pinning $> 3\text{ s}$).

### FLOWER De-Scoring is STRICTLY PROHIBITED (Q#43)
- Intentionally shooting or placing a NECTAR into the top of a FLOWER with the purpose of knocking out an opponent's already-scored NECTAR is a **STRATEGIC violation of G418 (MAJOR FOUL + YELLOW CARD)** because NECTAR may never be removed from a FLOWER.

### FLOWER Scoring Volume Pipe Bulge (Q#30)
- NECTAR resting on the outside of the FLOWER pipes that bulges into the 4.03" scoring volume counts as scored. However, deliberately holding an element against the outside of the pipes is a strategic violation of G418.

### Intaking Under Tipped HIVE (Q#17, Q#35)
- Catching or deflecting elements falling directly out of a tipping HIVE before they touch the tiles is illegal (G409).
- **Legal Autonomous Strategy (Q#35)**: Positioning the robot with its intake spinning just outside the drop zone to immediately collect elements **the moment they hit the floor tiles** is **100% LEGAL**.

---

## 4. HIVE Tipping Dynamics (Q#12, Q#46)

### Continuous Launching During Tip (Q#12)
- Robots do **not** have to wait for the HIVE to come to rest before launching more elements. Tipping points are awarded as soon as the damper contacts the frame, regardless of how many elements were used. (Caution: launching at a downward-facing cell during a tip can disrupt the motion).

### HIVE Calibration as an ARENA FAULT (Q#46)
- If a HIVE requires 9 POLLEN to tip instead of the specified 8 due to calibration drift, and this affects the match outcome or Ranking Points, it qualifies as an **ARENA FAULT (T201)** and the match will be replayed upon alliance request.

---

## 5. Human Player & Loading Zone (G425, G426, G427)

- **Dropping NECTAR (Q#6)**: The Drive Team member must drop NECTAR into the Loading Zone so that it touches the tile without the human contacting any element on the tile.
- **Rolling / Throwing Allowed (Q#7)**: Drive team members may throw or roll NECTAR into the field, provided:
  1. It is released before touching the tile.
  2. It contacts the Loading Zone tile first before rolling out into the field.
  3. It does not enter or score in a HIVE or FLOWER directly.
- **Nectar Tray Mobility (Q#48)**: The plastic storage tray for NECTAR may be repositioned anywhere convenient within the Alliance Area.
- **Out-of-Bounds Re-entry (Q#36)**: NECTAR that leaves the field and is returned to the drive team may be re-entered immediately without waiting for another HIVE tip.

---

## 6. Robot Construction & Inspection (Section 12)

### Software Limits for Expansion are ILLEGAL (Q#44 / R105)
- **R105 requires physical/mechanical constraints**: A mechanism that is mechanically capable of extending beyond 24" horizontal or 29" vertical is **ILLEGAL**, even if software/encoders limit travel during a match. Mechanical hard-stops or physical travel limits are mandatory.

### Floor Friction Brakes (Q#45 / R204)
- Deploying a non-gripping 3D-printed or rubber friction pad onto the foam tiles to resist being pushed is **legal under R204**, provided it does not generate active perpendicular downforce (vacuum/suction) or damage the tiles (G406).

### USB Cameras & Powered Hubs (Q#1 / R602, R707)
- Powering legal UVC webcams from a powered USB hub with a COTS USB battery pack is legal and does not violate R602.

### HuskyLens 2 Rulings (Q#26 / R702, R708)
- HuskyLens 2 is legal **ONLY via I2C with Wi-Fi streaming completely disabled**. USB connection is illegal (not UVC compatible). Standard REV cables have different pinouts and **will damage the Control Hub**; a custom pinout cable is required.

### Gamepads & Controllers (Q#16, Q#31, Q#41)
- The approved gamepad list from prior seasons has been removed. Any wired USB HID controller (gamepads, joysticks, flight sticks) supported by Android is allowed.
- Controllers held or worn by drivers are exempt from the Operator Console sizing box (Q#41).

### Servos & Sensors (Q#22, Q#50 / R504)
- Salvaging the internal potentiometer from a legal servo and using it as a standalone analog position sensor is legal (treated as a custom sensor under R607).
