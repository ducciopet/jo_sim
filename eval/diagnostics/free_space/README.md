# Free-space probe (17/11, diagnosis only; SINS untouched)

Question (user): would a memory of "space seen empty" help tell moving objects from SINS's residual false positives?
Script: free_space_probe.py (causal; Dynablox-like "ever free": a 0.2 m voxel becomes ever-free after 5 consecutive
free ray observations with no hit, any hit resets; query = fraction of an object's points above the ground in voxels
ever-free at least 0.5 s earlier). 9 CODa tuning windows, 20 000 rays per scan, odometry of the window.
Groups: gt_moving / gt_still (CODa annotations, 0.3 m/s over +-0.5 s), fp_struct (I2P residual moving FP, no
annotation within 1.5 m, <= 20 m; ~70 % static structure per the visual check).

| rows with >= 5 points | n | median free fraction | >= 0.7 |
|---|---|---|---|
| gt_moving | 10480 | 0.99 | 66 % |
| gt_still | 7758 | 0.39 | 23 % |
| fp_struct | 225 | 0.45 | 31 % |
AUC moving vs fp_struct 0.67, vs still 0.72 (excluding the first 5-10 s of map warm-up: 0.69-0.70).

By object density (points in the object):
| points | AUC vs fp_struct | AUC vs still | median moving / still / fp |
|---|---|---|---|
| 5-30 | 0.37 | 0.47 | 0.08 / 0.36 / 0.66 |
| 30-100 | 0.45 | 0.42 | 0.23 / 0.38 / 0.38 |
| >= 100 | 0.83 | 0.90 | 1.00 / 0.42 / 0.44 |

Reading: strong signal for dense (near) objects, none for sparse (far) ones: far voxels are rarely crossed by enough
rays to become ever-free (subsampling, voxel 0.2 m, 5-observation streak). Moving objects with a low free fraction have
a median of 41 points against 324. Ray update in Python 190-280 ms/scan (not representative of C++).
