# SINS: pipeline to the end of the tests (v2, 24/11/2026)

Status: v2, after two independent reviews (methodology; execution/risk), both "approve with changes". Awaiting the user's
approval. PIPELINE.md stays as history.

## 0. Where we are (24/11, verified on the JSONs)

**Base:** `r1lccbma` (sins `c5-semantic-birth` c15a337).
**Running:** chain F (FT = disocclusion travel exemption).

**CODa tuning, MOTA-all e3**

| Window set | EMMC (CODa-trained CenterPoint) | SINS EB, 3 reps |
|---|---|---|
| seq 0, all frames (1653 GT rows) | 36.5 | 65.1 / 64.9 / 55.4 |
| seq 10, all frames (4593 GT rows) | **69.3** | 51.1 / 56.4 / 53.7 |
| seq 16, all frames (6285 GT rows = 50% of the pool) | 54.3 | 61.9 / 63.5 / 64.1 |
| pooled 9 windows, all / testing | 57.5 / 55.9 | 59.5 / 57.0 (means) |
| pooled without seq 16, all frames | 60.6 | 54.8 / 58.6 / 54.1 |

Reading:
- SINS and EMMC are **comparable overall**. SINS is better on seq 0 and 16, clearly worse on seq 10 (vehicle-heavy).
- A bootstrap over windows of the gap gives a 95% CI of about [-21, +16]. "Ahead of EMMC" is NOT supported.
- EMMC is in-domain: the CODa split is per frame, so testing frames are 100 ms from CenterPoint training frames.

**Data exposure (correcting the v1 draft):**
- CODa 11/12 was already run with SINS profiles on 10-17/11, and some decisions used it.
- The JRDB held-out 6 were run with J2-J5.
- Lab and street served as vetoes in every chain.
- So all current SINS data is development data. On 11/12, EMMC (71.4) is expected to win clearly.

## 1. Principles (fixed by the user; unchanged)
- Physical, switchable, class-agnostic fixes; nothing learned on point clouds.
- One YOLO per camera, online 10 Hz.
- One behaviour indoor/outdoor (scale/sensor parameters only).
- Universal evaluation: e3 MOTA-all primary.
- Faithful baselines.
- No tests on the laptop.

**New in v2 (reviews):**
- e3ib is declared as post hoc: it was introduced after the SINS ghost analysis.
- Tuning tables are labelled "development".
- The paper's headline must come from data never used for any decision.

## 2. Statistical rules (replace the 20/11 rule, from chain G on; chain F is still judged by the old rule)
- **Adoption of a fix.** Paired comparison on the 9 windows x 3 reps = 27 pairs of error counts (FP+FN+IDSW), testing and all frames.
  - Adopt if the arm wins ≥ 19/27 pairs (one-sided sign test, p ≈ 0.03) on both.
  - Vetoes: per sequence (drop larger than the reference's range) and street.
  - Lab: no clear regression.
  - Reason: the old rule compared a paired gain with an unpaired spread, and was overridden 3 times.
- **Claims vs baselines.**
  - Paired cluster bootstrap over windows (sequences where possible) of pooled MOTA.
  - "Better" only if the 95% CI excludes 0, otherwise "comparable".
  - A sign test over windows x reps is reported next to it.
  - Always report per-sequence numbers.
- **Metrics in every table:** MOTA (primary), IDF1, HOTA (TrackEval); moving-only and PD as secondary columns; a 2 m gate sensitivity for vehicles.

## 3. The pipeline

### P0. Write down before executing (laptop, 24-26/11)
`eval/FINAL_RUN_SPEC.md`, covering:
- dataset by dataset: sequences/splits, frame rate, range, eval flags, ignore rules;
- exact NAS paths and tags; the per-run manifest (commit, profile md5, CPU load, dropped scans);
- who scores what, and where (lab PC scores everything from the NAS; the laptop only assembles from repo JSONs);
- the YOLO weights for baseline 2D inputs (yolo26s / yolo26s-seg, on the NAS);
- DOALS:
  - which SINS state counts as "dynamic": D only, D+PD declared as a second column;
  - the box-to-point rule;
  - timestamp matching to the 10 annotated scans;
  - the 20 m object IoU;
  - the GT labels persons by appearance;
- JRDB: the held-out list (exposed, declared), 15 Hz, `--range 15`, upper LiDAR, 5 YOLOs on a shared GPU;
- KITTI tracking: Car + Pedestrian, HOTA/MOTA with TrackEval and our MOTA.

### P1. Development, with dataset preparation IN PARALLEL (24/11 -> 04/12)
**Lab PC, development:**
- chain F (decide by the old rule + the sign test, reported);
- then at most 2 chains, the last one launched by 01/12.
- **Priority target: seq 10.** That is where SINS loses about 15 points to EMMC: vehicles, then crowd swaps. Run a seq-10 error diagnosis on EB first (offline, err_decomp + funnel, vehicles first).
- Crowd swaps: diagnosis closed by 28/11, otherwise a documented limitation.

**Lab PC, smoke tests before the freeze** (one never-scored debug sequence each): JRDB bytes-cafe, one DOALS sequence, one KITTI tracking sequence. They check that the run finishes and that TF, outputs and latency are sane.

**Laptop:**
- DOALS converter (ROS1 bag, poses, deskew) and the DOALS LiDAR-only profile;
- KITTI tracking converter + profile (HDL-64 + left camera);
- JRDB profile rebuilt from the current base (the old jrdb_indoor/outdoor profiles break the one-behaviour rule);
- the final-run and leave-one-out scripts.

### P2. Freeze (05/12)
- Tag `paper-v1` (sins + jo_sim).
- Commit the profiles of every dataset with an md5 before their first run; no reruns after seeing scores.
- Dry run of the final script (1 window per dataset).
- Dead-code cleanup after the paper.

### P3. Final SINS runs (lab PC, 06/12 -> 13/12)
- **Unseen data, the headline:**
  - KITTI tracking training (21 seq, never run with SINS);
  - DOALS (8 seq, LiDAR-only);
  - **new CODa sequences never on the NAS** (needs a user decision: download 2-3 of the 17 missing sequences, which must have GT);
  - JRDB = "seen" (declared).
- **Development data**, reported and labelled:
  - CODa tuning 9 windows x5 reps;
  - CODa 11/12 (exposed, declared);
  - street;
  - lab x6.
- Runtime: a quiet-window latency run on the lab PC (no other users' jobs, load logged). Needs the user's OK, or a laptop exception.
- Disk: stream inputs one sequence at a time from the NAS; one named scratch folder pre-authorised for deletion by the user; 3 GB guard.

### P4. Leave-one-out ablation (lab PC, 14-16/12)
From paper-v1, CODa x3, one chain, about 8 arms:
- each adopted fix off: connected leaf / fallback / bus cap / merge_fragments / static anchor / (FT);
- disocclusion off (contribution 1 in PAPER_PLAN);
- YOLO off (LiDAR-only);
- [one camera vs N where applicable].

### P5. Baselines (external PC, in parallel, priority order, deadline 13/12, cells may end "partial/skipped")
1. CODa gaps, in this order:
   - **zero-shot CenterPoint + SimpleTrack (required: it is the fair zero-shot rival);**
   - Dynablox / M-detector reps 2-3;
   - LV-DOT 16_w1 (until it exists, LV-DOT is excluded from the pooled table).
2. KITTI (M-detector running; then Dynablox; SimpleTrack/EagerMOT/DeepFusionMOT with public KITTI detections).
3. DOALS.
4. New CODa sequences (if the user approves the download).
5. JRDB.
6. Street.
7. Lab.

EMMC is labelled the "in-domain upper reference" on CODa. CMT/BEVFusion are likely skipped (unreleased code): declare it.

### P6. Paper material (laptop, from repo JSONs, 17/12 -> 23/12)
- tables per dataset (per-sequence + pooled, bootstrap CI, IDF1/HOTA);
- ablation; runtime; a failure-case section (seq 10 vehicles, far sparse persons, crowd swaps, out-of-FOV unclassified, DOALS LiDAR-only);
- videos; the SINS page;
- hand-off to sins-paper-4c.

### Contingency: 17/12 -> 10/01 for reruns after a post-freeze bug. Writing starts by mid-January (submission 22/02).

## 4. Who does what
- **Lab PC:** chains, diagnostics, smoke tests, final runs, scoring, videos.
- **External PC:** baselines by priority, manifests, fidelity labels.
- **Laptop:** code (compile only), converters/profiles, FINAL_RUN_SPEC, statistics scripts, tables/page, coordination, backups.
- **User:** the decisions below, disk OKs, downloads.

## 5. Decisions needed from the user
1. Development stop 04/12, freeze 05/12, contingency until 10/01: OK?
2. **Download 2-3 new CODa sequences** (never seen, with GT) as the main unseen CODa test? Size to be checked.
3. KITTI tracking and DOALS as unseen SINS datasets (converters to be written now)?
4. The new statistical rules (sign test ≥ 19/27 for fixes; bootstrap CI for claims): OK?
5. Latency: a quiet window on the lab PC (other users' jobs paused), or a laptop exception?
6. Pre-authorise deletion of one named scratch folder on the lab PC for the final runs?

## 6. Risks
- **SINS loses clearly on seq 10 and on 11/12 to the in-domain EMMC.** The paper must not claim superiority on CODa. The strengths to show:
  - no in-domain training;
  - better on crowded pedestrian scenes (seq 0/16);
  - unseen datasets;
  - PD state, mocap lab, runtime.
- **A never-run dataset pipeline breaks after the freeze.** Mitigation: smoke tests in P1 and the contingency window.
- **Disk on both PCs:** the external PC has 37 GB free, not 133; its KITTI zips (~52 GB) in ~/Downloads can go after the NAS md5, with the user's OK there.
- **The laptop has no NAS mount** right now (GVFS dropped): tables are built from repo JSONs.
