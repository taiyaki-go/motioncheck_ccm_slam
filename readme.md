# Motion Check CCM-SLAM
**CCM-SLAM-Enhanced**

This repository is an **extended version of [CCM-SLAM](https://github.com/VIS4ROB-lab/ccm_slam)** (GPLv3).
We have added new methods to improve accuracy, especially in XXX scenarios.

**Contribution**
- Global Bundle Adjustment (GBA) is a widely used optimization process intended
to correct errors and improve estimation accuracy. While GBA often enhances trajectory accuracy, in some
cases it can introduce substantial degradation.
- To address this problem, we propose Motion Check (MC), a method that compares the estimated trajectories pre- and post-GBA. When a significant discrepancy in trajectory shape is observed, the method judges that GBA has caused a deterioration in accuracy. By identifying points where accuracy degrades due to GBA and applying appropriate corrective actions, Motion Check effectively suppresses such deterioration.
- Motion Check demonstrated its effectiveness by reducing the maximum trajectory error by up to 81.52 cm and the root mean square error by up to 3.40 cm by using EuRoC dataset (MH01, MH02, MH03)

**Changes from CCM-SLAM**
The 56 Files in include/cslam and src are changed. We have not modified the content of communications between the server and agent.
