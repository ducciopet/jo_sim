# Is there prior work with a "still but movable" (PD) state? Literature check, 18/11 (web search, to be re-checked on Scholar before submission)

Claim to support: no published, code-available online tracker outputs, per tracked object, three states
(moving / still-but-movable / static) combining semantic movability with motion evidence, as SINS does (D / PD / S).

## Families found, and why they differ from PD
1. **Moving-object segmentation (MOS), LiDAR.** Binary moving vs non-moving per point; the papers explicitly state the
   goal of separating *moving* objects from *movable but static* ones (parked cars, standing people), which are then
   treated as static. No object state, no PD.
   - Chen et al., "Moving Object Segmentation in 3D LiDAR Data" (LMNet), RA-L 2021, https://arxiv.org/abs/2105.08971
   - Mersch et al., 4DMOS, RA-L 2022, http://www.ipb.uni-bonn.de/pdfs/mersch2022ral.pdf ; MapMOS, RA-L 2023,
     https://www.ipb.uni-bonn.de/pdfs/mersch2023ral.pdf
   - Chen et al., "Automatic Labeling to Generate Training Data for Online LiDAR-based MOS", https://arxiv.org/abs/2201.04501
   - Arora et al., "Mapping the Static Parts of Dynamic Scenes from 3D LiDAR", ECMR 2021,
     https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/arora2021ecmr.pdf
2. **Semantic dynamic SLAM ("a priori dynamic" classes).** Movable classes are masked as dynamic whatever their motion
   (DynaSLAM, Bescos et al., RA-L 2018); later work notes this wrongly removes parked cars and hurts pose estimation.
   Movability without motion state: the opposite failure of MOS. E.g. Semantic Flow-guided Motion Removal,
   https://arxiv.org/abs/2010.06876 ; TwistSLAM++, https://arxiv.org/abs/2209.07888 ; RF-LIO, https://arxiv.org/abs/2206.09463
3. **Semi-static / semi-dynamic mapping.** Objects static during a SLAM cycle but not forever (parked cars, materials);
   a map-level category for localization, not an online per-object state for navigation.
   - Review of dynamic object filtering in 3D LiDAR SLAM (semi-dynamic class), https://pmc.ncbi.nlm.nih.gov/articles/PMC10821332/
   - Groh et al., indoor parking localization with semi-static objects, FUSION 2014,
     https://www.mad.tf.fau.de/files/2017/06/2014-Groh-FUSION-RIP.pdf
   - POV-SLAM (semi-static environments), https://arxiv.org/abs/2307.00488 ; Transitional Grid Maps,
     https://arxiv.org/abs/2401.06518 ; SD-SLAM (static / semi-static / dynamic landmarks, venue to verify)
4. **Model-free tracking that notes "potentially moving" objects.** Wang, Posner, Newman, "Model-Free Detection and
   Tracking of Dynamic Objects with 2D Lidar", IJRR 2015,
   https://www.robots.ox.ac.uk/~mobile/Papers/2015IJRR_wang.pdf : states that detecting only instantaneously moving
   objects neglects potentially moving ones; motion states are estimated, but there is no semantic movability and no
   still-but-movable output state. Closest classic precedent: cite it.
5. **Closest recent work.** Chen, Huang, Cho, "Dynamic Object Detection and Tracking in Construction: A Fisheye Camera
   and LiDAR Sensor Fusion Model", arXiv 2607.06896 (8 Jul 2026), https://arxiv.org/abs/2607.06896 : quadruped,
   LiDAR + fisheye; identifies the dynamic -> "temporarily static" transition as a failure mode (tracks decay and are
   lost) and handles it by freezing position updates. No released code found; state definitions to read in the PDF.
6. **Datasets with motion attributes (not methods).** nuScenes annotates attributes such as pedestrian standing/moving
   and vehicle parked/stopped/moving (from memory: verify); trackers evaluated on it do not output them. CODa does not
   annotate motion state (hence our derived label, e3).

## Suggested wording
"To our knowledge, no online tracker with released code reports, per object, whether a movable object is currently
still: MOS methods [1] treat it as static, semantic dynamic SLAM [2] treats it as dynamic, semi-static mapping [3]
handles it at map level, and model-free trackers [4, 5] lose or freeze it. SINS outputs it explicitly (PD)."

Open: a Scholar/Semantic Scholar pass with "temporarily static", "stationary pedestrian", "stopped vehicle state",
"movable object tracking" before submission; read 2607.06896 in full.
