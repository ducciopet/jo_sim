# Paper hand-off, 25/11/2026: from the coordinator (laptop session) to the paper session (sins-paper-4c)

The paper draft is in `~/Desktop/SINS_paper`. It was last edited 19-20/11 and is now partly out of date with the system and the evaluation. The user asked to hand all paper updates to the paper session. This file lists what changed, where it is documented, and what to update now versus later.

Sources of truth, all in `~/Desktop/jo_sim_sins`:
- `eval/PIPELINE_FINAL.md`: the plan of record, approved by the user on 24/11, with the user's decisions at the bottom.
- `eval/FINAL_RUN_SPEC.md`: datasets, roles, metrics, statistics and runtime for the final runs.
- `eval/diagnostics/lab_queue_17nov.md`: chain results, sections 19-35.
- `eval/STATUS.md`.
- Code: the sins branch `c5-semantic-birth`, frozen as tag `paper-v1` on 05/12.

Rules for the update:
- Do NOT put final numbers in before the freeze (05/12) and the final runs (06-13/12). Use `\tbd{}` for every parameter and number that is not frozen.
- Do not rewrite PAPER_PLAN.md (user rule).
- Write in English, in the paper's existing style. Mark every change so the user can review it, one git commit per section, if the folder is a git repo; otherwise a dated note in README.md.

## 1. Update NOW: Method (sections/04_method.tex)

### 1a. Semantic leaves: the depth-window/histogram text is obsolete
- **Current text:** "points histogrammed along the viewing depth and only one depth cluster retained, grown from a seed bin…", plus the "nearest depth cluster spanning half the box width" rule.
- **Why it is obsolete:** the user removed every distance window on YOLO-mask points ("the biggest and most obvious error", 20/11).
- **The new leaf:**
  - the largest **3D-connected group** of the mask points (`yolo_leaf_connected`);
  - two points link if they are closer than `max(r0, k·depth)`, with `r0 = 0.5 m` and `k = 1.2 ×` the LiDAR ring spacing in rad (OS1-128: 0.0074; VLP-16: 0.042; HDL-64: 0.0084);
  - no depth cap.
- **Box fallback:** when the mask has too few points, the box points are split into connected groups (`yolo_leaf_fallback_connected`). The nearest group by median depth is kept if it has ≥ min points and spans ≥ the minimum fraction of the box width; there is no window. Code: `sins/src/preprocessing_node.cpp` (`connectedGroups`, `largestConnected`).
- **Figure `fig:leaves`:** panel (b), the "histogram along the viewing depth", must become "connected groups in 3D".

### 1b. Refinement can now enlarge an object
- **Current text:** "Refinement therefore never enlarges a box", and Proposition `prop:leaf` (subset) with its proof.
- **What changed:** with `yolo_refine_merge_fragments`, which is part of the base since 23/11, a refining leaf also **absorbs other unclaimed fused objects** that lie inside its box. "Inside" means ≥ camera_min_points and ≥ 50% of their points within the leaf box + 0.10 m margin. They are absorbed before the cut.
- **Physical reason:** a car seen by the LiDAR in pieces (glass, dark paint, ring gaps), covered by one YOLO mask.
- **Effect:** CODa vehicle "piece" false positives 549 → 302, pooled MOTA about +2.
- **Restate the proposition:** the refined object is a subset of the points of the fused objects inside the leaf box, i.e. the union of its pieces; no point outside the leaf box + margin is added; one leaf still labels at most one object.
- Also the **bus size cap**: maximum box extent [12, 6, 4] m and volume 120 m³, so that a merged bus or truck is not discarded. Update `tab:params`.

### 1c. Tracking: the static anchor (new, part of the base since 24/11)
- **Parameters:** `track_static_anchor_radius` 0.5 m, `track_static_anchor_claim_radius` 1.0 m.
- **Rule:** a confirmed track that never showed motion (no dynamic class, never published dynamic, empty dynamic history) may re-acquire a detection farther than max(0.5 m, half its footprint) from its last observation ONLY if no other moving confirmed track predicts that detection within 1.0 m.
- **Physical reason:** a still object does not jump. On lab seq 1950 a static neighbour track whose own cluster vanished jumped 1.3 m onto a passing person, and the person's track died.
- **Effect:** CODa +1.2 pooled (every rep positive); lab 92.9 → 95.6 and much more stable.
- **Rejected first version:** without the claim condition it re-bore every partially seen still object (−9 on one window). Worth one sentence as a design note.

### 1d. Tested and REJECTED, all switchable and default off
These are candidates for "negative results" or a footnote:

| Option | Chain | Sign test (≥ 19/27 needed) | Notes |
|---|---|---|---|
| Masked splitter (`split_masked`) | D | −0.1 | |
| Faster motion proof for person-sized tracks (`track_person_size_confirm_ticks`) | D | −0.4 | +32 person ID switches |
| Disocclusion travel exemption (`track_disocclusion_travel_window`) | F | 14/27 | |
| Extent association of pieces to moving tracks (`track_extent_assoc_margin`) | G | 11/27 | |
| Vehicle-anchored fragment merge | H | running | Class-specific thresholds, so NOT adoptable under the user's rule; a class-agnostic variant HV2 only if H shows a gain |

### 1e. Leave as is, but mark `\tbd`
Every parameter in `tab:params` and in the text (thresholds, radii) until the freeze; the TODOs that refer to the frozen profile.

## 2. Update NOW: Experimental setup (sections/05_experimental_setup.tex)

### 2a. Datasets and roles (FINAL_RUN_SPEC section 2)
- **UNSEEN (the paper's headline):**
  - **KITTI tracking training**: 21 sequences, HDL-64 + left colour camera, depth = scan projected online; evaluated in the camera view only.
  - **DOALS**: 8 sequences, handheld OS1-64, **no camera**, so SINS runs LiDAR-only (no semantics, no PD); point-IoU metric of the Dynablox authors.
  - **CODa sequences 2, 3, 18**: never opened; downloaded 24/11.
- **Exposed, declared:**
  - CODa 11/12: used during development 10-17/11; the sentence "held-out data, evaluated once" is no longer true and must be rewritten;
  - JRDB: the held-out 6 were run in early November.
- **Development:** CODa tuning (seq 0 w1-3, 10 w1-5, 16 w1), the street, the lab (Qualisys).
- **Commented line 15:** says "JRDB, SiT and CrowdBot were considered and dropped". JRDB is back, so update it.
- **Table `tab:datasets` and figure `fig:platform-data`:** add KITTI, DOALS and JRDB.

### 2b. Sensor setups: a new paragraph and table (user 25/11, important)
One SINS behaviour across all setups (sensor/scale overrides only):
- lab: VLP-16 + RealSense RGB-D;
- CODa: OS1-128 + ZED / stereo;
- KITTI, JRDB: LiDAR + RGB only, depth from the projected scan;
- DOALS: LiDAR only.

Contrast this with the baselines, each tied to one setup:
- LV-DOT needs RGB-D (≈0 on CODa outdoors);
- CenterPoint trackers need in-domain training (zero-shot CenterPoint+SimpleTrack has negative MOTA on 10 of 15 CODa windows);
- Dynablox and M-detector are LiDAR-only, with no semantics or PD.

Planned table: "sensor configuration × dataset × method (runs / why not)". See the memory note and PIPELINE_FINAL "Paper emphasis".

### 2c. Evaluation
- **Primary metric:** e3 MOTA-all (CLEAR-MOT, 1 m centre gate; annotated rows of undefined motion state = ignored regions).
- **Secondary:** IDF1; **HOTA / DetA / AssA** (TrackEval's algorithm with a centre-distance similarity S = max(0, 1 − d/1 m); state this); moving-only; PD; e3ib (ignored objects cover their whole box; declared as introduced after the ghost analysis).
- **Vehicles:** 2 m gate sensitivity.
- **KITTI:** also the official TrackEval HOTA where a method outputs image boxes.
- **DOALS:** the authors' point IoU (0.5-20 m, mean over the 10 annotated scans per sequence). For box methods, the points inside boxes in the `dynamic` state; a second column with D+PD. The GT labels persons by appearance (state this).
- **Statistics:**
  - **claims:** paired bootstrap over windows/sequences of the pooled-MOTA difference (95% CI; "better" only if the CI excludes 0, otherwise "comparable"), plus a sign test over windows × reps;
  - **fix adoption during development:** sign test ≥ 19/27 on 9 windows × 3 reps.
  - Script: `eval/scripts/analysis/bootstrap_claims.py`.
- **Reps:** SINS x3 (CODa tuning x5, lab x6); nondeterministic baselines x3.
- **Runtime:** on the laptop after the freeze, unloaded, with the hardware stated.

### 2d. Baselines and fidelity (replace the numbers in the text; table `tab:baselines`)

| Method | Fidelity result | Notes |
|---|---|---|
| Dynablox | **Reproduced**: DOALS object IoU 85.7 (3 reps, 85.73/85.74/85.73) vs 86.0 in the paper | Upstream SIGSEGV at shutdown after the scores are written: a known bug, results valid |
| M-detector | **Reproduced**: KITTI frame-out IoU 0.728 (3 reps: 0.726/0.732/0.727) vs 0.746 | Authors' own FAST-LIO pipeline; nondeterministic segfault in 3.3% of sequence runs, rerun and reported |
| DeepFusionMOT | KITTI Car HOTA 77.9 (training) vs 75.5 (test server) | |
| EagerMOT | KITTI training, Car HOTA 78.04 / MOTA 87.25 vs test server 74.39 / 87.82; Pedestrian HOTA 48.20 / MOTA 62.17 vs 39.38 / 49.82 | Consistent; training vs hidden test; the trainval PointGNN saw these sequences |
| SimpleTrack | **Reproduced**: nuScenes val AMOTA 0.696 vs 0.687 (2 Hz val) | On KITTI with public PointGNN detections: Car HOTA 76.3 / MOTA 84.1; no paper number |
| CenterPoint (UniMT CODa weights) | mAP **not confirmed**: Pedestrian consistent; Car/Cyclist protocol unstated | The authors' test GT is not public; their GT excludes parked bikes from Cyclist |
| CMT / BEVFusion | **Skipped** | The model code of the CODa checkpoints was not released |
| LV-DOT | Runs where RGB-D exists | CODa 16_w1 uses stereo-SGBM depth (declared); ≈0 on CODa |
| Zero-shot CenterPoint (nuScenes weights) + SimpleTrack | New, as the fair zero-shot rival | CODa intensity = 0, single sweep |

Also: **EMMC** (EagerMOT multi-camera with CODa-trained CenterPoint) is the **in-domain upper reference** on CODa.

## 3. Update NOW: framing (introduction, abstract, discussion)
**The honest CODa picture** (development data; exact numbers come after the final runs):
- SINS is **comparable** to EMMC overall (e3 MOTA-all, 9 tuning windows: 59.5 vs 57.5 all, 57.0 vs 55.9 testing; bootstrap CI ≈ [−21, +16]), with NO training on CODa.
- **Better** on crowded pedestrian sequences (seq 0: 55-65 vs 36; seq 16: 62-64 vs 54).
- **Worse** on vehicle-heavy seq 10 (51-56 vs 69): fragmented vehicles.
- Seq 16 holds 50% of the pooled GT.
- Do NOT claim superiority on CODa. The strengths are:
  - no in-domain training;
  - one behaviour across sensor setups;
  - pedestrians in crowds and the PD state;
  - unseen datasets;
  - the mocap lab;
  - runtime.

**Limitations to state:**
- fragmented vehicles out of the camera FOV;
- far, sparse persons (no cluster ~3.6% of moving persons);
- crowd identity swaps between neighbouring persons;
- unclassified out-of-FOV persons held static (~5%);
- range-boundary rows (1.7%, a metric effect for all methods).

## 4. LATER (after 13-16/12)
Results tables per dataset (per sequence + pooled + CI, MOTA/IDF1/HOTA), the leave-one-out ablation (FINAL_RUN_SPEC section 6), runtime, qualitative figures and videos (`datasets_SINS/derived/videos/`), discussion. The coordinator will deliver a results package (JSONs + summary) after the final runs.

## 5. Small cleanups noticed
- `sections/07_downstream.tex` and `08_conclusion.tex` are unused (README says they can go).
- `05_experimental_setup.tex` line 43 TODO: the evaluation is now e3. Line 89 TODO: the testing-frames evaluation is implemented (`--frames testing`).
- `05_experimental_setup.tex` line 86, "M-detector cannot process 128-beam CODa at 10 Hz and runs at a quarter of real time": still true (0.25x); keep it.
