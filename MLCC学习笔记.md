# MLCC学习笔记

> [!NOTE] 要点总结
> 1. 提出了一种**自适应体素化**技术，将多个 LiDAR 外参校准转化为为 LiDAR 束调整 (BA) 问题。
>    - 为了在小型 FoV 传感器之间创建足够的共视特征，在传感器平台中引入了运动，使得每个传感器将在不同时间扫描相同的区域（因此是特征）
>    - 为了减少 LiDAR 之间特征对应匹配的时间消耗，实现了自适应体素化，以动态地将点云分割成多个体素，以便每个体素中只有一个平面特征
>    - 为了进一步加速特征对应匹配，继承了上述自适应体素图来提取 LiDAR 边缘特征。
> 2. 通过推导二阶成本函数，进一步提高了非线性最小二乘问题的求解时间和精度。

<!-- 
> [!TIP]
> Helpful advice for doing things better or more easily.

> [!IMPORTANT]
> Key information users need to know to achieve their goal.
-->

## 原文及翻译如下

> Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras Using Adaptive Voxelization

使用自适应体素化对多个小 FoV(视场角) LiDAR 和摄像头进行无目标外参校准

> **Abstract** — Determining the extrinsic parameter between multiple light detection and rangings (LiDARs) and cameras is essential for autonomous robots, especially for solid-state LiDARs, where each LiDAR unit has a very small field-of-view (FoV), and multiple units are often used collectively. The majority of extrinsic calibration methods are proposed for 360° mechanical spinning LiDARs where the FoV overlap with other LiDAR or camera sensors is assumed. A few research works have been focused on the calibration of small FoV LiDARs and cameras nor on the improvement of the calibration speed. In this work, we consider the problem of extrinsic calibration among small FoV LiDARs, and cameras, with the aim to shorten the total calibration time and further improve the calibration precision.

**摘要** —— 确定多个激光雷达 (LiDAR) 与相机之间的外参对于自主机器人至关重要，特别是对于固态 LiDAR，其中每个 LiDAR 单元都有非常小的视场 (FoV)，并且通常将多个单元一起使用。大多数外参校准方法都是针对 360° 机械旋转 LiDAR 提出的，其中假设 FoV 与其他 LiDAR 或相机传感器重叠。一些研究工作集中在小 FoV LiDAR 和相机的校准上，也没有集中在校准速度的提高上。在本文中，我们考虑小 FoV LiDAR 和相机之间的外参校准问题，旨在缩短总校准时间并进一步提高校准精度。

> We first implement an adaptive voxelization technique in the extraction and matching of LiDAR feature points. Such a process could avoid the redundant creation of k-d trees in LiDAR extrinsic calibration and extract LiDAR feature points in a more reliable and fast manner than existing methods. We then formulate the multiple LiDAR extrinsic calibration into a LiDAR bundle adjustment (BA) problem. By deriving the cost function up to second order, the solving time and precision of the nonlinear least square problem are further boosted. Our proposed method has been verified on data collected in four targetless scenes and under two types of solid-state LiDARs with a completely different scanning pattern, density, and FoV. The robustness of our work has also been validated under eight initial setups, with each setup containing 100 independent trials. Compared with the state-ofthe-art methods, our work has increased the calibration speed 15 times for LiDAR-LiDAR extrinsic calibration (averaged result from 100 independent trials) and 1.5 times for LiDAR–camera extrinsic calibration (averaged result from 50 independent trials) while remaining accurate. To benefit the robotics community, we have also open-sourced our implementation code on GitHub.

我们首先在 LiDAR 特征点的提取和匹配中实现了一种自适应体素化技术。这一过程可避免 LiDAR 外参校准中冗余创建 k-d 树，并以比现有方法更可靠、更快速的方式提取 LiDAR 特征点。然后，我们将多个 LiDAR 外参校准公式化为 LiDAR 束调整 (BA) 问题。通过推导二阶成本函数，进一步提高了非线性最小二乘问题的求解时间和精度。我们提出的方法已经在四个无目标场景中收集的数据以及两种扫描模式、密度和 FoV 完全不同的固态 LiDAR 下得到验证。我们工作的稳健性也在八个初始设置下得到验证，每个设置包含 100 次独立试验。与最先进的方法相比，我们的工作在保持准确性的同时将 LiDAR-LiDAR 外参校准的速度提高了 15 倍（100 次独立试验的平均结果），将 LiDAR-相机外参校准的速度提高了 1.5 倍（50 次独立试验的平均结果）。为了造福机器人社区，我们还在 GitHub 上开源了我们的实现代码。

## I. INTRODUCTION

> LiDAR and camera sensors, due to their superior characteristics in direct spatial ranging and rich color information conveying, have been increasingly used in autonomous driving [1], [2], navigation [3], [4], and high-resolution mapping [5] applications. One drawback of the current 360◦ mechanical spinning light detection and ranging (LiDAR) is their dramatic high cost, preventing their massive application in industry. Solid-state LiDAR [6] has a much lower cost while achieving a denser point cloud within its field-of-view (FoV). However, solid-state LiDARs are of small FoV that multiple solid-state LiDARs need to be combined to achieve a similar FoV coverage as the mechanical spinning LiDAR. This setup necessitates precise extrinsic calibration among the LiDARs and cameras.

激光雷达和摄像头传感器由于其在直接空间测距和丰富色彩信息传达方面的优异特性，已越来越多地用于自动驾驶 [1]、[2]、导航 [3]、[4] 和高分辨率测绘 [5] 应用。当前 360°机械旋转激光 (LiDAR) 的一个缺点是其成本极高，阻碍了其在工业中的大规模应用。固态激光雷达 [6] 成本低得多，同时在其视场 (FoV) 内实现更密集的点云。然而，固态激光雷达的视场小，需要组合多个固态激光雷达才能实现与机械旋转激光雷达类似的视场覆盖范围。这种设置需要在激光雷达和摄像头之间进行精确的外参校准。

> Several challenges reside in the extrinsic calibration involving small FoV LiDARs. 
> 1. Limited FoV Overlap Among the Sensors and the Precision Requirement: Current methods usually require the existence of a common FoV between each pair of sensors [7]–[10] such that each feature is viewed by all sensors. In real-world applications, this FoV overlap might be minimal or not even exist due to the small FoVs of solid-state LiDARs and their numerous sensor mounting positions. The accuracy requirement of the calibration results, e.g., the consistency and colorization of the point cloud (see Fig. 1), is thus more challenging.
> 2. Computation Time Demands: For general iterative closest point (ICP)-based LiDAR extrinsic calibration approaches [5], [11], the extrinsic is optimized by aligning the point cloud from all LiDARs and maximizing the point cloud’s consistency. The increase in the number of LiDARs implies that the feature point correspondence searching will be more time-consuming. This is due to the reason that each feature point needs to search for and match with nearby feature points using a k-d tree, which contains the whole point cloud. In the LiDAR–camera extrinsic calibration, a larger amount of LiDAR points will also lead to more computation time in the LiDAR feature extraction.

涉及小 FoV LiDAR 的外参校准存在一些挑战。
1. 传感器之间的 FoV 重叠有限和精度要求：当前的方法通常要求每对传感器之间存在一个共同的 FoV [7]–[10]，以便所有传感器都可以看到每个特征。在实际应用中，由于固态 LiDAR 的 FoV 较小且传感器安装位置众多，这种 FoV 重叠可能很小甚至不存在。因此，校准结果的精度要求（例如点云的一致性和彩色化（见图 1））更具挑战性。
2. 计算时间要求：对于一般的基于迭代最近点 (ICP) 的 LiDAR 外参校准方法 [5]，[11]，通过对齐来自所有 LiDAR 的点云并最大化点云的一致性来优化外参校准。 LiDAR 数量的增加意味着特征点对应搜索将更加耗时。这是因为每个特征点都需要使用包含整个点云的 k-d 树来搜索并与附近的特征点匹配。在 LiDAR-相机外参校准中，LiDAR 点的数量越多，LiDAR 特征提取的计算时间也越多。

> [图片1]()

图 1. (a) 密集彩色点云，其中 LiDAR 位姿和外参通过我们提出的方法进行了优化。其他视角的视图展示在 (b) 左侧和 (c) 右侧。我们的实验视频可在 https://youtu.be/PaiYgAXl9iY 上找到。

> To address the above challenges, we propose a fast and targetless approach for extrinsic calibration of multiple small FoV LiDARs and cameras. To create enough covisible features among the small FoV sensors, we introduce motions to the sensor platform such that each sensor will scan the same area (hence features) at different times. We first calibrate the extrinsic among LiDARs (and simultaneously estimate the LiDAR poses) by registering their point cloud using an efficient bundle adjustment (BA) method we recently proposed [4]. To reduce time consumption in feature correspondence matching among LiDARs, we implement an adaptive voxelization to dynamically segment the point cloud into multiple voxels so that only one plane feature resides in each voxel (see Section III-B). We then calibrate the extrinsic between the cameras and LiDARs by matching the covisible features between the images and the above-reconstructed point cloud. To further accelerate the feature correspondence matching, we inherit the above adaptive voxel map to extract LiDAR edge features. In summary, our contributions are listed as follows.
> 1. We propose a targetless extrinsic calibration pipeline for multiple small FoV LiDARs and cameras that share very few or even no FoV overlap. We formulate LiDAR extrinsic calibration into a BA problem and implement an adaptive voxelization technique into the LiDAR feature extraction and matching process. The overall pipeline enjoys higher calibration precision and computation efficiency.
> 2. We verify our proposed work on data collected in various test scenes by LiDARs of different scanning patterns, FoVs, and point densities. When compared to various state-of-the-art methods, our proposed work could boost the speed by 15 times for multiple LiDAR calibration and 1.5 times for multiple LiDAR–camera calibration. Meanwhile, our proposed work maintains high calibration precision, with the average translation and rotation errors down to 6 mm and 0.09◦ for LiDAR–camera and 8 mm and 0.2◦ for LiDAR-LiDAR.
> 3. We open-source our implementation in ROS on GitHub1 to benefit the robotics community.

为了解决上述挑战，我们提出了一种快速、无目标的方法来校准多个小型 FoV LiDAR 和相机的外参。为了在小型 FoV 传感器之间创建足够的共视特征，我们在传感器平台中引入了运动，使得每个传感器将在不同时间扫描相同的区域（因此是特征）。我们首先通过使用我们最近提出的高效束调整 (BA) 方法 [4] 配准它们的点云来校准 LiDAR 之间的外参（并同时估计 LiDAR 位姿）。为了减少 LiDAR 之间特征对应匹配的时间消耗，我们实现了自适应体素化，以动态地将点云分割成多个体素，以便每个体素中只有一个平面特征（参见第 III-B 节）。然后，我们通过匹配图像和上述重建的点云之间的共视特征来校准相机和 LiDAR 之间的外参。为了进一步加速特征对应匹配，我们继承了上述自适应体素图来提取 LiDAR 边缘特征。综上所述，我们的贡献如下。
1. 我们提出了一种无目标外参校准流程，用于多个小 FoV LiDAR 和相机，这些 LiDAR 和相机共享很少甚至没有 FoV 重叠。我们将 LiDAR 外参校准公式化为 BA 问题，并将自适应体素化技术应用于 LiDAR 特征提取和匹配过程。整个流程具有更高的校准精度和计算效率。
2. 我们在各种测试场景中通过不同扫描模式、FoV 和点密度的 LiDAR 收集的数据验证了我们提出的工作。与各种最先进的方法相比，我们提出的工作可以将多个 LiDAR 校准的速度提高 15 倍，将多个 LiDAR-相机校准的速度提高 1.5 倍。同时，我们提出的工作保持了较高的校准精度，LiDAR-相机的平均平移和旋转误差降至 6 毫米和 0.09°，LiDAR-LiDAR 的平均平移和旋转误差降至 8 毫米和 0.2°。
3. 我们在 GitHub(https://github.com/hku-mars/mlcc) 上开源了我们的 ROS 实现，以造福机器人社区。

## II. RELATED WORKS

### A. LiDAR–LiDAR Extrinsic Calibration

> The extrinsic calibration methods between multiple LiDARs could be divided into motion-based and motionless approaches. Motion-based approaches assume that each sensor undergoes the same rigid motion in each time interval [2], [12], [13] and transform the extrinsic calibration into a hand-eye problem [14]. Levinson and Thrun [15], Maddern et al [16], and Billah and Farrell [17] also introduced external inertial navigation sensors to facilitate the motion estimation of LiDARs. The calibration precision of these approaches is easily affected by the accuracy of the LiDAR odometry results, which might be unreliable. Motionless methods have been discussed in [7], [8] where the authors attach retroreflective tapes to the surface of calibration targets to create and facilitate the feature extraction among multiple LiDARs.These approaches require prior preparation work and FoV overlap between LiDARs, which is unpractical in real-world applications.

多个 LiDAR 之间的外参校准方法可分为基于运动的方法和静止方法。基于运动的方法假设每个传感器在每个时间间隔内都经历相同的刚性运动 [2]，[12]，[13]，并将外参校准转化为手眼问题 [14]。Levinson 和 Thrun [15]、Maddern 等人 [16] 以及 Billah 和 Farrell [17] 还引入了外部惯性导航传感器以促进 LiDAR 的运动估计。这些方法的校准精度很容易受到 LiDAR 里程计结果准确性的影响，这可能不可靠。[7]，[8] 中讨论了静止方法，其中作者将反射胶带贴到校准目标的表面以创建和促进多个 LiDAR 之间的特征提取。这些方法需要事先进行准备工作，并且 LiDAR 之间的 FoV 重叠，这在实际应用中是不切实际的。

> In our previous work [5], a simple rotational movement is introduced to eliminate the requirement of FoV overlap, as each onboard sensor could percept the same region of interest. Then, the extrinsic parameter is calibrated, along with the estimation of LiDAR poses, by optimizing the consistency of the point cloud map with ICP registration. The main problem within [5] is that the ICP registration always registers one scan to the other, leading to an iterative process where only one optimization variable (e.g., extrinsic or LiDAR poses) can be optimized (by registering the point cloud affected by the variable under optimization to the rest). Such an iterative procedure is prolonged to converge. Moreover, at each iteration, the ICP-based feature correspondence matching process might be very time-consuming. As for each point-to-plane correspondence, ICP needs to either search inside a k-d tree containing the entire point cloud or create a k-d tree containing the local point cloud every time before searching.

在我们之前的工作 [5] 中，引入了一个简单的旋转运动来消除 FoV 重叠的要求，因为每个机载传感器都可以感知相同的感兴趣区域。然后，通过优化点云图与 ICP 配准的一致性，校准外参以及估计 LiDAR 位姿。[5] 中的主要问题是 ICP 配准总是将一个扫描配准到另一个扫描，导致一个迭代过程，其中只有一个优化变量（例如，外参或 LiDAR 位姿）可以优化（通过将受优化变量影响的点云配准到其余变量）。这种迭代过程被延长以收敛。此外，在每次迭代中，基于 ICP 的特征对应匹配过程可能非常耗时。对于每个点到平面的对应关系，ICP 需要在包含整个点云的 k-d 树内搜索，或者每次搜索之前创建包含局部点云的 k-d 树。

> In this work, we formulate the extrinsic calibration into a BA problem [4], where all the optimization variables (both extrinsic and LiDAR poses) are optimized concurrently by registering points into their corresponding plane. When compared to other plane adjustment techniques [18], [19], the BA technique we use does not estimate the plane parameters in the optimization process but solves for them analytically in a closed-form solution prior to the optimization iteration. The removal of plane parameters from the optimization iteration lowers the dimension significantly and leads to very efficient multiview registration. To match points corresponding to the same plane, we implement an adaptive voxelization technique [4] to replace the k-d tree in [5]. As only one plane feature exists in each voxel, our proposed work significantly saves the computation time in correspondence searching while remaining accurate (see Section III-B).

在本研究中，我们将外参校准公式化为 BA 问题 [4]，其中通过将点配准到其对应平面来同时优化所有优化变量（外参和 LiDAR 位姿）。与其他平面调整技术 [18]、[19] 相比，我们使用的 BA 技术不会在优化过程中估计平面参数，而是在优化迭代之前以闭式解的形式对其进行分析求解。从优化迭代中移除平面参数可显著降低维度，并实现非常高效的多视图配准。为了匹配对应于同一平面的点，我们实现了一种自适应体素化技术 [4] 来取代 [5] 中的 k-d 树。由于每个体素中只存在一个平面特征，我们提出的工作在保持准确性的同时显著节省了对应搜索的计算时间（参见第 III-B 节）。

### B. LiDAR–Camera Extrinsic Calibration

> The extrinsic calibration between LiDAR and camera could be mainly divided into target-based and targetless methods. In target-based approaches, the geometric features, e.g., edges and surfaces, are extracted from artificial geometric solids [20]–[22] or chessboard [23], [24] using intensity and color information. These features are matched either automatically or manually and are solved with nonlinear optimization tools. Jeong et al [25] established the constraints using the crosswalk features on the streets; however, this method is essentially target-based as the parallelism characteristic of the crosswalk is used. Since extra calibration targets and manual work are needed, these methods are less practical compared with targetless solutions.

LiDAR 与相机之间的外参校准主要可分为基于目标的方法和无目标的方法。在基于目标的方法中，使用强度和颜色信息从人造几何体 [20]–[22] 或棋盘 [23], [24] 中提取几何特征（例如边缘和表面）。这些特征可以自动或手动匹配，并使用非线性优化工具进行求解。Jeong 等人 [25] 使用街道上的人行横道特征建立了约束；然而，这种方法本质上是基于目标的，因为利用了人行横道的平行性特性。由于需要额外的校准目标和手动工作，这些方法与无目标解决方案相比不太实用。

> The targetless methods could be further divided into motion-based and motionless approaches. In motion-based methods, the initial extrinsic parameter is usually estimated by the motion information and refined by the appearance information. Nagy et al [26] reconstructed a point cloud from images using the structure from motion (SfM) to determine the initial extrinsic parameter and refine it by backprojecting LiDAR points onto the image plane. Taylor and Nieto [13] and Park et al [27] initialized the extrinsic parameter by hand-eye calibration and optimized it by minimizing the reprojection error between images and LiDAR scans. In motionless approaches, only the edge features that coexist in both sensors’ FoV are extracted and matched. Then, the extrinsic parameter is optimized by minimizing the reprojected edge-to-edge distances [9], [28]–[30] or by maximizing the mutual information between the backprojected LiDAR points and the images [10].

无目标方法可以进一步分为基于运动的方法和静止的方法。在基于运动的方法中，初始外参通常由运动信息估计，并由外观信息细化。Nagy 等人 [26] 使用运动结构 (SfM) 从图像重建点云，以确定初始外参，并通过将 LiDAR 点反投影到图像平面上对其进行细化。Taylor 和 Nieto [13] 和 Park 等人 [27] 通过手眼校准初始化外参，并通过最小化图像和 LiDAR 扫描之间的重新投影误差对其进行优化。在静止方法中，仅提取和匹配两个传感器的 FoV 中共存的边缘特征。然后，通过最小化重新投影的边到边距离 [9]、[28]–[30] 或最大化反投影的 LiDAR 点和图像之间的互信息来优化外参 [10]。

> Our proposed work is targetless and creates covisible features by moving the sensor suite to multiple poses, hence allowing extrinsic calibration between LiDAR and cameras even when they have no overlap, a circumstance that was not solved in prior works [10], [13], [29]. Moreover, compared with our previous work [28] which extracts LiDAR edge features using the RANSAC algorithm, this work extracts edge features using the same adaptive voxelization already computed in the LiDAR extrinsic calibration, which is more competitive in computation time and calibration precision.Compared with [10] which uses LiDAR intensity information as a feature, our work uses more reliable 3-D edge information and is more computationally efficient and accurate (see Section IV). Moreover, our work does not require the common FoV between sensors.

我们提出的工作是无目标的，通过将传感器套件移动到多个位姿来创建共视特征，因此即使没有重叠，也可以在 LiDAR 和相机之间进行外参校准，这种情况在以前的工作 [10]、[13]、[29] 中没有得到解决。此外，与我们之前使用 RANSAC 算法提取 LiDAR 边缘特征的工作 [28] 相比，这项工作使用已经在 LiDAR 外参校准中计算的相同自适应体素化来提取边缘特征，这在计算时间和校准精度方面更具竞争力。与使用 LiDAR 强度信息作为特征的 [10] 相比，我们的工作使用了更可靠的 3-D 边缘信息，计算效率更高，更准确（见第 IV 节）。此外，我们的工作不需要传感器之间的共同视场。

## III. METHODOLOGY

### A. Overview

> [图片2]()

图 2. 两个相对指向的传感器之间旋转产生的 FoV 重叠。两个传感器 $L_i$ 和 $L_j/C_k$ 的原始设置没有 FoV 重叠。随着旋转运动的引入，所有传感器在不同时间内扫描同一区域。

> Let ${^B_A}\mathbf{T} = ({^B_A}\mathbf{R}, {^B_A}\mathbf{t}) \in SE(3)$ represent the rigid transformation from frame $A$ to frame $B$, where ${^B_A}\mathbf{R} \in SO(3)$ and ${^B_A}\mathbf{t} \in \mathbb{R}^3$ are the rotation and translation, respectively. We denote $\mathcal{L} = \{L_0, L_1,\cdots, L_{n-1} \}$ the set of $n$ LiDARs, where $L_0$ represents the base LiDAR for reference, $\mathcal{C} = \{C_0, C_1,\cdots, C_h\}$ represents the set of $h$ cameras, $\mathcal{E}_L = \{^{L_0}_{L_1}\mathbf{T}, ^{L_0}_{L_2}\mathbf{T},\cdots, ^{L_0}_{L_{n-1}}\mathbf{T}\}$ represents the set of LiDAR extrinsic parameters, and $\mathcal{E}_C = \{^{C_0}_{L_0}\mathbf{T}, ^{C_1}_{L_0}\mathbf{T},\cdots, ^{C_h}_{L_0}\mathbf{T}\}$ represents the set of LiDAR–camera extrinsic parameters. To create covisible features between multiple LiDARs and cameras that may share no FoV overlap, we rotate the robot platform to $m$ poses such that the same region of interest is scanned by all sensors (see Fig. 2). Denote $\mathcal{T} = \{t_0, t_1,\cdots, t_{m-1} \}$ the time for each of the $m$ poses and the pose of the base LiDAR at the initial time as the global frame, i.e., $^G_{L_0}\mathbf{T}_{t_0} = \mathbf{I}_{4\times4}$. Denote $\mathcal{S} = \{^{G}_{L_0}\mathbf{T}_{t_1}, ^{G}_{L_0}\mathbf{T}_{t_2},\cdots, ^{G}_{L_0}\mathbf{T}_{t_{m-1}}\}$ the set of the base LiDAR poses in global frame. The point cloud patch scanned by LiDAR $L_i \in \mathcal{L}$ at time $t_j \in \mathcal{T}$ is denoted by $\mathcal{P}_{L_i,t_j}$ , which is in $L_i$’s local frame. This point cloud patch could be transformed to global frame:
> $$\tag{1} \begin{aligned}^G\mathcal{P}_{L_i,t_j}& = ^G_{L_i}\mathbf{T}_{t_j}\mathcal{P}_{L_i,t_j} \\ & \triangleq \{^G_{L_i}\mathbf{R}_{t_j}\mathbf{p}_{L_i,t_j} + ^G_{L_i}\mathbf{t}_{t_j}, \forall \mathbf{p}_{L_i,t_j} \in \mathcal{P}_{L_i,t_j}\}\end{aligned}$$

令 ${^B_A}\mathbf{T} = ({^B_A}\mathbf{R}, {^B_A}\mathbf{t}) \in SE(3)$ 表示从 $A$ 帧到 $B$ 帧的刚性变换，其中 ${^B_A}\mathbf{R} \in SO(3)$ 和 ${^B_A}\mathbf{t} \in \mathbb{R}^3$ 分别表示旋转和平移。我们用 $\mathcal{L} = \{L_0, L_1,\cdots, L_{n-1} \}$ 表示 $n$ 个 LiDAR 的集合，其中 $L_0$ 表示用于参考的基准 LiDAR，$\mathcal{C} = \{C_0, C_1,\cdots, C_h\}$ 表示 $h$ 个相机的集合， $\mathcal{E}_L = \{^{L_0}_{L_1}\mathbf{T}, ^{L_0}_{L_2}\mathbf{T},\cdots, ^{L_0}_{L_{n-1}}\mathbf{T}\}$ 表示 LiDAR 外参的集合，$\mathcal{E}_C = \{^{C_0}_{L_0}\mathbf{T}, ^{C_1}_{L_0}\mathbf{T},\cdots, ^{C_h}_{L_0}\mathbf{T}\}$ 表示 LiDAR–相机外参的集合。为了在可能没有 FoV 重叠的多个 LiDAR 和相机之间创建共视特征，我们将机器人平台旋转到 $m$ 个位姿，以便所有传感器扫描相同的感兴趣区域（见图 2）。用 $\mathcal{T} = \{t_0, t_1,\cdots, t_{m-1} \}$ 表示为 $m$ 个位姿中的每一个的时间，并将基准 LiDAR 在初始时间的位姿表示为全局帧，即 $^G_{L_0}\mathbf{T}_{t_0} = \mathbf{I}_{4\times4}$ 。用 $\mathcal{S} = \{^{G}_{L_0}\mathbf{T}_{t_1}, ^{G}_{L_0}\mathbf{T}_{t_2}, \cdots, ^{G}_{L_0}\mathbf{T}_{t_{m-1}}\}$ 表示全局帧中基准 LiDAR 位姿的集合。LiDAR $L_i \in \mathcal{L}$ 在时间 $t_j \in \mathcal{T}$ 扫描的点云块用 $\mathcal{P}_{L_i,t_j}$ 表示，它在 $L_i$ 的局部帧中。可以通过以下方式将此点云块转换为全局帧：
$$\tag{1} \begin{aligned}^G\mathcal{P}_{L_i,t_j}& = ^G_{L_i}\mathbf{T}_{t_j}\mathcal{P}_{L_i,t_j} \\ & \triangleq \{^G_{L_i}\mathbf{R}_{t_j}\mathbf{p}_{L_i,t_j} + ^G_{L_i}\mathbf{t}_{t_j}, \forall \mathbf{p}_{L_i,t_j} \in \mathcal{P}_{L_i,t_j}\}\end{aligned}$$

> In our proposed approach of multisensor calibration, we sequentially calibrate $\mathcal{E}_L$ and $\mathcal{E}_C$ . In the first step, we simultaneously estimate the LiDAR extrinsic $\mathcal{E}_L$ and the base LiDAR pose trajectory $\mathcal{S}$ based on an efficient multiview registration (see Section III-C). In the second step, we calibrate $\mathcal{E}_C$ by matching the depth-continuous edges extracted from images and the above-reconstructed point cloud (see Section III-D). Lying in the center of both LiDAR and camera extrinsic calibration is an adaptive map, which finds correspondence among LiDAR and camera measurements efficiently (Section III-B).

在我们提出的多传感器校准方法中，我们依次校准 $\mathcal{E}_L$ 和 $\mathcal{E}_C$ 。在第一步中，我们基于有效的多视图配准同时估计 LiDAR 外参 $\mathcal{E}_L$ 和基本 LiDAR 位姿轨迹 $\mathcal{S}$ （参见第 III-C 节）。在第二步中，我们通过匹配从图像中提取的深度连续边缘和上面重建的点云来校准 $\mathcal{E}_C$ （参见第 III-D 节）。LiDAR 和相机外参校准的中心是一张自适应图，它可以有效地找到 LiDAR 和相机测量值之间的对应关系（第 III-B 节）。

### B. Adaptive Voxelization

> To find the correspondences among different LiDAR scans, we assume that the initial base LiDAR trajectory $\mathcal{S}$ , LiDAR extrinsic $\mathcal{E}_L$ , and camera extrinsic $\mathcal{E}_C$ are available. The initial base LiDAR trajectory $\mathcal{S}$ could be obtained by an online LiDAR simultaneous localization and mapping (SLAM) (e.g., [3]), and the initial extrinsic could be obtained from the CAD design or a rough hand-eye calibration [14]. Our previous work [5] extracts edge and plane feature points from each LiDAR scan and matches them to the nearby edge and plane points in the map by a k-nearest neighbor (k-NN) search. This would repeatedly build a k-d tree of the global map at each iteration. In this article, we use a more efficient voxel map proposed in [4] to create correspondences among all LiDAR scans.

为了找到不同 LiDAR 扫描之间的对应关系，我们假设初始基础 LiDAR 轨迹 $\mathcal{S}$ 、LiDAR 外参 $\mathcal{E}_L$ 和相机外参 $\mathcal{E}_C$ 是可用的。初始基础 LiDAR 轨迹 $\mathcal{S}$ 可以通过在线 LiDAR 同时定位和地图构建 (SLAM)（例如 [3]）获得，初始外参 $\mathcal{E}_L$ 可以通过 CAD 设计或粗略的手眼校准获得 [14]。我们之前的工作 [5] 从每个 LiDAR 扫描中提取边缘和平面特征点，并通过 k 最近邻 (k-NN) 搜索将它们与地图中附近的边缘和平面点进行匹配。这将在每次迭代时重复构建全局地图的 k-d 树。在本文中，我们使用 [4] 中提出的更高效的体素图来建立所有 LiDAR 扫描之间的对应关系。

> The voxel map is built by cutting the point cloud (registered using the current $\mathcal{S}$ and $\mathcal{E}_L$ ) into small voxels such that all points in a voxel roughly lie on a plane (with some adjustable tolerance). The main problem of the fixed-resolution voxel map is that if the resolution is high, the segmentation would be too time-consuming, while if the resolution is too low, multiple small planes in the environments falling into the same voxel would not be segmented. To best adapt to the environment, we implement an adaptive voxelization process. More specifically, the entire map is first cut into voxels with a preset size (usually large, e.g., 4 m). Then, for each voxel, if the contained points from all LiDAR scans roughly form a plane (by checking the ratio between eigenvalues), it is treated as a planar voxel; otherwise, they will be divided into eight octants, where each will be examined again until the contained points roughly form a plane or the voxel size reaches the preset minimum lower bound. Moreover, the adaptive voxelization is performed directly on the LiDAR raw points, so no prior feature points extraction is needed as in [5].

体素图的构建方法是将点云（使用当前的 $\mathcal{S}$ 和 $\mathcal{E}_L$ 配准）切割成小体素，使得体素中的所有点大致位于一个平面上（具有一些可调整的公差）。固定分辨率体素图的主要问题是，如果分辨率很高，分割会太耗时，而如果分辨率太低，环境中落入同一个体素的多个小平面将无法分割。为了最好地适应环境，我们实现了一个自适应体素化过程。更具体地说，首先将整个地图切割成具有预设大小（通常很大，例如 4 米）的体素。然后，对于每个体素，如果所有 LiDAR 扫描中包含的点大致形成一个平面（通过检查特征值之间的比率），则将其视为平面体素；否则，它们将被分成八个卦限，每个卦限将再次检查，直到包含的点大致形成一个平面或体素大小达到预设的最小下限。此外，自适应体素化直接在 LiDAR 原始点上进行，因此不需要像 [5] 中那样事先提取特征点。

> [图片3]()

图 3. (a) 使用自适应体素化分割的 LiDAR 点云。同一体素内的点颜色相同。虚线白色矩形中的点的详细自适应体素化可以在 (b) 彩色点和 (c) 原始点中查看。初始体素化的默认大小为 4 m，最小体素大小为 0.25 m。

> Fig. 3 shows a typical result of the adaptive voxelization process in a complicated campus environment. As can be seen, this process is able to segment planes of different sizes, including large planes on the ground, medium planes on the building walls, and tiny planes on tree crowns.

图 3是复杂校园环境下自适应体素化过程的典型结果。可以看出，该过程能够分割出不同大小的平面，包括地面上的大平面、建筑物墙壁上的中等平面以及树冠上的微小平面。

### C. Multi-LiDAR Extrinsic Calibration

>With adaptive voxelization, we can obtain a set of voxels of different sizes. Each voxel contains points that are roughly on a plane and creates a planar constraint for all LiDAR poses that have points in this voxel. More specifically, considering the $l$th voxel consisting of a group of points $\mathcal{P}_l = \{^G\mathbf{p}_{L_i,t_j}\}$ scanned by $L_i \in \mathcal{L}$ at times $t_j \in \mathcal{T}$ , we define a point cloud consistency indicator $c_l(^G_{L_i}\mathbf{T}_{t_j})$ , which forms a factor on $\mathcal{S}$ and $\mathcal{E}_L$ , as shown in Fig. 4(a). Then, the base LiDAR trajectory and extrinsic are estimated by optimizing the factor graph. A natural choice for the consistency indicator $c_l(\cdot)$ would be the summed Euclidean distance between each $^G\mathbf{p}_{L_i,t_j}$ to the plane to be estimated [see Fig. 4(b)]. Taking account of all such indicators within the voxel map, we could formulate the problem as 
> $$\tag{2} \arg\min\limits_{\mathcal{S},\mathcal{E_L},\mathbf{n}_l,\mathbf{q}_l} \sum\limits_l\underbrace{\left(\frac{1}{N_l}\sum^{N_l}_{k=l}(\mathbf{n}^T_l(^G\mathbf{p}_k-\mathbf{q}_l))^2\right)}_{l\text{th factor}}$$

通过自适应体素化，我们可以获得一组不同大小的体素。每个体素包含大致位于同一平面上的点，并为所有包含在此体素中的点的 LiDAR 位姿创建平面约束。更具体地说，考虑到第 $l$个体素由 $L_i \in \mathcal{L}$ 在时间 $t_j \in \mathcal{T}$ 扫描的一组点 $\mathcal{P}_l = \{^G\mathbf{p}_{L_i,t_j}\}$ 组成，我们定义一个点云一致性指标 $c_l(^G_{L_i}\mathbf{T}_{t_j})$ ，它形成 $\mathcal{S}$ 和 $\mathcal{E}_L$ 上的因子，如图 4（a）所示。然后，通过优化因子图来估计基本 LiDAR 轨迹和外参。一致性指标 $c_l(\cdot)$ 的自然选择是每个 $^G\mathbf{p}_{L_i,t_j}$ 到要估计的平面之间的欧几里得距离之和[见图 4（b）]。考虑到体素图内的所有这些指标，我们可以将问题表述为 
$$\tag{2} \arg\min\limits_{\mathcal{S},\mathcal{E_L},\mathbf{n}_l,\mathbf{q}_l} \sum\limits_l\underbrace{\left(\frac{1}{N_l}\sum^{N_l}_{k=l}(\mathbf{n}^T_l(^G\mathbf{p}_k-\mathbf{q}_l))^2\right)}_{l\text{th factor}}$$

> where $^G\mathbf{p}_k \in \mathcal{P}_l$ , $N_l$ is the total number of points in $\mathcal{P}_l$ , $\mathbf{n}_l$ is the normal vector of the plane, and $\mathbf{q}_l$ is a point on this plane.

其中 $^G\mathbf{p}_k \in \mathcal{P}_l$ ， $N_l$ 是 $\mathcal{P}_l$ 中的点的总数， $\mathbf{n}_l$ 是平面的法向量， $\mathbf{q}_l$ 是该平面上的一个点。

> [图片4]()

图 4. (a) 与 $\mathcal{S}$ 和 $\mathcal{E}_L$ 有关的第 $l$个因子项 $L_i \in \mathcal{L}$ 和 $t_j \in \mathcal{T}$ 。(b) 点 $^G\mathbf{p}_k$ 到平面 $\pi$ 的距离。

> It is noticed that the optimization variables $(\mathbf{n}_l,\mathbf{q}_l)$ in (2) could be analytically solved (see Appendix A) and the resultant cost function (3) is over the LiDAR pose $^G_{L_i}\mathbf{T}_{t_j}$ (hence the base LiDAR trajectory $\mathcal{S}$ and extrinsic $\mathcal{E}_L$ ) only as follows: 
> $$\tag{3} \arg\min\limits_{\mathcal{S},\mathcal{E}_L}\sum_l{\lambda_3(\mathbf{A}_l)}$$

值得注意的是，(2) 中的优化变量 $(\mathbf{n}_l,\mathbf{q}_l)$ 可以通过分析求解（参见附录 A），并且由此产生的成本函数 (3) 仅适用于 LiDAR 位姿 $^G_{L_i}\mathbf{T}_{t_j}$ （因此是基本 LiDAR 轨迹 $\mathcal{S}$ 和外参 $\mathcal{E}_L$ ），如下所示： 
$$\tag{3} \arg\min\limits_{\mathcal{S},\mathcal{E}_L}\sum_l{\lambda_3(\mathbf{A}_l)}$$

> where $\lambda_3(\mathbf{A}_l)$ denotes the minimal eigenvalue of matrix $\mathbf{A}_l$ defined as 
> $$\tag{4} \mathbf{A}_l=\frac{1}{N_l}\sum^{N_l}_{k=1}{{^G\mathbf{p}_k}{^G\mathbf{p}^T_k}-{\mathbf{q}^*_l}{\mathbf{q}^{*T}_l}} , \mathbf{q}^*_l=\frac{1}{N_l}\sum^{N_l}_{k=1}{^G\mathbf{p}_k}$$

其中 $\lambda_3(\mathbf{A}_l)$ 表示矩阵 $\mathbf{A}_l$ 的最小特征值，定义为 
$$\tag{4} \mathbf{A}_l=\frac{1}{N_l}\sum^{N_l}_{k=1}{{^G\mathbf{p}_k}{^G\mathbf{p}^T_k}-{\mathbf{q}^*_l}{\mathbf{q}^{*T}_l}} , \mathbf{q}^*_l=\frac{1}{N_l}\sum^{N_l}_{k=1}{^G\mathbf{p}_k}$$

> To allow efficient optimization in (3), we derive the closed-form derivatives with respect to the optimization variable $\mathbf{x}$ up to second order (the detailed derivation from (3) to (5) is elaborated in Appendix B) 
> $$\tag{5} \lambda_3(\mathbf{x}\boxplus\delta\mathbf{x})\approx\lambda_3(\mathbf{x})+\mathbf{\bar{J}}\delta\mathbf{x}+\frac{1}{2}\delta\mathbf{x}^T\mathbf{\bar{H}}\delta\mathbf{x}$$

为了在 (3) 中实现有效优化，我们推导了关于优化变量 x 的二阶闭式导数（从 (3) 到 (5) 的详细推导在附录 B 中详细说明） 
$$\tag{5} \lambda_3(\mathbf{x}\boxplus\delta\mathbf{x})\approx\lambda_3(\mathbf{x})+\mathbf{\bar{J}}\delta\mathbf{x}+\frac{1}{2}\delta\mathbf{x}^T\mathbf{\bar{H}}\delta\mathbf{x}$$

> where $\mathbf{\bar{J}}$ is the Jacobian matrix and $\mathbf{\bar{H}}$ is the Hessian matrix. $\delta\mathbf{x}$ is a small perturbation of the optimization variable $\mathbf{x}$ 
> $$\tag*{} \mathbf{x}=[\ \underbrace{\cdots\ {^G_{L_0}\mathbf{R}_{t_j}}  {^G_{L_0}\mathbf{t}_{t_j}}\ \cdots}_{\mathcal{S}} \ \underbrace{\cdots\ {^{L_0}_{L_i}\mathbf{R}}   {^{L_0}_{L_i}\mathbf{t}}\ \cdots}_{\mathcal{E}_L} \ ]$$

其中 $\mathbf{\bar{J}}$ 是雅可比矩阵， $\mathbf{\bar{H}}$ 是 Hessian 矩阵。 $\delta\mathbf{x}$ 是优化变量 $\mathbf{x}$ 的小扰动 
$$\tag*{} \mathbf{x}=[\ \underbrace{\cdots\ {^G_{L_0}\mathbf{R}_{t_j}}  {^G_{L_0}\mathbf{t}_{t_j}}\ \cdots}_{\mathcal{S}} \ \underbrace{\cdots\ {^{L_0}_{L_i}\mathbf{R}}   {^{L_0}_{L_i}\mathbf{t}}\ \cdots}_{\mathcal{E}_L} \ ]$$

> Then, the optimal $\mathbf{x}^*$ could be determined by iteratively solving (6) with the LM method and updating the $\delta\mathbf{x}$ to $\mathbf{x}$ 
> $$\tag{6} (\mathbf{\bar{H}} + \mu\mathbf{I})\delta\mathbf{x} = -\mathbf{\bar{J}}^T$$

然后，可以通过使用 LM 方法迭代求解（6）并将 $\delta\mathbf{x}$ 更新为 $\mathbf{x}$ 来确定最优 $\mathbf{x}^*$ 
$$\tag{6} (\mathbf{\bar{H}} + \mu\mathbf{I})\delta\mathbf{x} = -\mathbf{\bar{J}}^T$$

### D. LiDAR–Camera Extrinsic Calibration

> With the LiDAR extrinsic parameter $\mathcal{E}_L$ and pose trajectory $\mathcal{S}$ computed above, we obtain a dense global point cloud by transforming all LiDAR points to the base LiDAR frame. Then, the extrinsic $\mathcal{E}_C$ is optimized by minimizing the summed distance between the backprojected LiDAR edge feature points and the image edge feature points. Two types of LiDAR edge points could be extracted from the point cloud. One is the depth-discontinuous edge between the foreground and background objects, and the other is the depth-continuous edge between two neighboring nonparallel planes. As explained in our previous work [28], depth-discontinuous edges suffer from foreground inflation and bleeding points phenomenon; we hence use depth-continuous edges to match the point cloud and images.

利用上面计算出的 LiDAR 外参 $\mathcal{E}_L$ 和位姿轨迹 $\mathcal{S}$ ，我们将所有 LiDAR 点变换到基础 LiDAR 帧，从而获得密集的全局点云。然后，通过最小化反投影 LiDAR 边缘特征点与图像边缘特征点之间的总距离来优化外参 $\mathcal{E}_C$ 。可以从点云中提取两种类型的 LiDAR 边缘点。一种是前景和背景物体之间的深度不连续边缘，另一种是两个相邻非平行平面之间的深度连续边缘。如我们之前的工作 [28] 所述，深度不连续边缘会出现前景膨胀和出血点现象；因此，我们使用深度连续边缘来匹配点云和图像。

> In [28], the LiDAR point cloud is segmented into voxels with uniform sizes, and the planes inside each voxel are estimated by the RANSAC algorithm. In contrast, our method uses the same adaptive voxel map obtained in Section III-B. We calculate the angle between their containing plane normals for every two adjacent voxels. If this angle exceeds a threshold, the intersection line of these two planes is extracted as the depth-continuous edge, as shown in Fig. 5. We choose to implement the Canny algorithm for image edge features to detect and extract.

在 [28] 中，LiDAR 点云被分割成大小均匀的体素，每个体素内的平面由 RANSAC 算法估计。相比之下，我们的方法使用第 III-B 节中获得的相同自适应体素图。我们计算每两个相邻体素包含的平面法线之间的角度。如果该角度超过阈值，则提取这两个平面的交线作为深度连续边缘，如图 5 所示。我们选择实施 Canny 算法来检测和提取图像边缘特征。

> [图片5]()

图 5. 深度连续 LiDAR 边缘特征提取比较。（a）真实世界图像。（b）该场景的原始点云。（c）使用 [28] 中的方法提取的边缘，其中黄色圆圈表示错误估计。（d）使用自适应体素化提取的边缘。

> Suppose that $^G\mathbf{p}_i$ represents the $i$th point from a LiDAR edge feature extracted above in global frame. With pinhole camera and its distortion model, $^G\mathbf{p}_i$ is projected onto the image taken by camera $C_l$ at $t_j$ , i.e., $\mathbf{I}_{l,j}$ by 
> $$\tag{7} {^{\mathbf{I}_{l,j}}\mathbf{p}_i} = \mathbf{f}{\left(\boldsymbol{\pi}{\left({^{C_l}_{L_0}\mathbf{T}}{{\left({^{G}_{L_0}\mathbf{T}_{t_j}}\right)}^{-1}}{^G\mathbf{p}_i}\right)}\right)}$$

假设 $^G\mathbf{p}_i$ 表示全局帧中上述提取的 LiDAR 边缘特征的第 $i$个点。利用针孔相机及其畸变模型，将 $^G\mathbf{p}_i$ 投影到相机 $C_l$ 在 $t_j$ 拍摄的图像上，即 $\mathbf{I}_{l,j}$ ，通过以下方式
$$\tag{7} {^{\mathbf{I}_{l,j}}\mathbf{p}_i} = \mathbf{f}{\left(\boldsymbol{\pi}{\left({^{C_l}_{L_0}\mathbf{T}}{{\left({^{G}_{L_0}\mathbf{T}_{t_j}}\right)}^{-1}}{^G\mathbf{p}_i}\right)}\right)}$$

> where $\mathbf{f}(\cdot)$ is the camera distortion model and $\boldsymbol{\pi}(\cdot)$ is the projection model. Let $\mathcal{I}_i$ represent the set of images that capture the point $^G\mathbf{p}_i$ , i.e., $\mathcal{I}_i = {\mathbf{I}_{l,j}}$ . For each ${^{\mathbf{I}_{l,j}}\mathbf{p}_i}$ , the $\kappa$ nearest image edge feature points $\mathbf{q}_{k}$ on $\mathbf{I}_{l,j}$ are searched. The normal vector $\mathbf{n}_{i,l,j}$ of the edge formed by these $\kappa$ points is thus the eigenvector corresponding to the minimum eigenvalue of $\mathbf{A}_{i,l,j}$ that 
> $$\tag{8} \mathbf{A}_{i,l,j} = {\sum\limits^\kappa_{k=1}{\left(\mathbf{q}_k-\mathbf{q}_{i,l,j}\right)}{\left(\mathbf{q}_k-\mathbf{q}_{i,l,j}\right)^T}},\quad \mathbf{q}_{i,l,j} = \frac{1}{\kappa}\sum\limits^\kappa_{k=1}{\mathbf{q}_k} \ .$$

其中 $\mathbf{f}(\cdot)$ 为相机畸变模型， $\mathbf{\pi}(\cdot)$ 为投影模型。令 $\mathcal{I}_i$ 表示捕捉点 $^G\mathbf{p}_i$ 的图像集，即 $\mathcal{I}_i = {\mathbf{I}_{l,j}}$ 。对于每个 ${^{\mathbf{I}_{l,j}}\mathbf{p}_i}$ ，在 $\mathbf{I}_{l,j}$ 上搜索 $\kappa$ 个最近的图像边缘特征点 $\mathbf{q}_{k}$ 。因此，这 $\kappa$ 个点形成的边缘的法向量 $\mathbf{n}_{i,l,j}$ 就是 $\mathbf{A}_{i,l,j}$ 的最小特征值所对应的特征向量，该特征值满足 
$$\tag{8} \mathbf{A}_{i,l,j} = {\sum\limits^\kappa_{k=1}{\left(\mathbf{q}_k-\mathbf{q}_{i,l,j}\right)}{\left(\mathbf{q}_k-\mathbf{q}_{i,l,j}\right)^T}},\quad \mathbf{q}_{i,l,j} = \frac{1}{\kappa}\sum\limits^\kappa_{k=1}{\mathbf{q}_k}\ .$$

> The residual originated from this LiDAR camera correspondence is defined as 
> $$\tag{9} \mathbf{r}_{i,l,j} = \mathbf{n}_{i,l,j}^T\left({^{\mathbf{I}_{l,j}}\mathbf{p}_i} - \mathbf{q}_{i,l,j} \right)$$

来自该 LiDAR 相机对应的残差定义为 
$$\tag{9} \mathbf{r}_{i,l,j} = \mathbf{n}_{i,l,j}^T\left({^{\mathbf{I}_{l,j}}\mathbf{p}_i} - \mathbf{q}_{i,l,j} \right)$$

> Collecting all such correspondences, the extrinsic $\mathcal{E}_C$ calibration problem could be formulated as 
> $$\tag{10} \mathcal{E}^{*}_{C} = \arg\min\limits_{\mathcal{E}_{C}} {\sum_i {\sum_{\mathbf{I}_{l,j} \in \mathcal{I}_i} \left( \mathbf{n}^T_{i,l,j} \left( {^{\mathbf{I}_{l,j}}\mathbf{p}_i} - {\mathbf{q}_{i,l,j}} \right) \right)}}$$

收集所有这些对应关系，外参 $\mathcal{E}_C$ 校准问题可以表述为 
$$\tag{10} \mathcal{E}^{*}_{C} = \arg\min\limits_{\mathcal{E}_{C}} {\sum_i {\sum_{\mathbf{I}_{l,j} \in \mathcal{I}_i} \left( \mathbf{n}^T_{i,l,j} \left( {^{\mathbf{I}_{l,j}}\mathbf{p}_i} - {\mathbf{q}_{i,l,j}} \right) \right)}}$$

> Inspecting the residual in (9), we find that ${^{\mathbf{I}_{l,j}}\mathbf{p}_i}$ is dependent on LiDAR poses ${^G_{L_0}\mathbf{T}_{t_i}}$ . This is due to the reason that LiDARs may have FoV overlap with cameras at different times (as in Fig. 2). Since ${^G_{L_0}\mathbf{T}_{t_i}} \in \mathcal{S}$ has been well estimated from Section III-C, we keep them fixed in this step. Moreover, $\mathbf{n}_{i,l,j}$ and $\mathbf{q}_{i,l,j}$ are also implicitly dependent on $\mathcal{E}_C$ since both $\mathbf{n}_{i,l,j}$ and $\mathbf{q}_{i,l,j}$ are related to nearest neighbor search. The complete derivative of (10) to the variable $\mathcal{E}_C$ would be too complicated. In this article, to simplify the optimization problem, we ignore the influence of camera extrinsic on $\mathbf{n}_{i,l,j}$ and $\mathbf{q}_{i,l,j}$ . This strategy works well in practice as detailed in Section IV-B.

检查 (9) 中的残差，我们发现 ${^{\mathbf{I}_{l,j}}\mathbf{p}_i}$ 依赖于 LiDAR 位姿 ${^G_{L_0}\mathbf{T}_{t_i}}$ 。这是因为 LiDAR 的 FoV 可能在不同时间与相机重叠（如图 2 所示）。由于 ${^G_{L_0}\mathbf{T}_{t_i}} \in \mathcal{S}$ 已从第 III-C 节得到很好的估计，因此我们在此步骤中保持它们不变。此外， $\mathbf{n}_{i,l,j}$ 和 $\mathbf{q}_{i,l,j}$ 也隐式依赖于 EC，因为 $\mathbf{n}_{i,l,j}$ 和 $\mathbf{q}_{i,l,j}$ 都与最近邻搜索有关。对变量 EC 进行 (10) 的完全导数过于复杂。在本文中，为了简化优化问题，我们忽略了相机外参对 $\mathbf{n}_{i,l,j}$ 和 $\mathbf{q}_{i,l,j}$ 的影响。如第 IV-B 节所述，该策略在实践中效果很好。

> The nonlinear optimization (10) is solved with LM method by approximating the residuals with their first-order derivatives (11). The optimal $\mathcal{E}^*_C$ is then obtained by iteratively solving (11) and updating $\delta\mathbf{x}$ to $\mathbf{x}$ using the $\boxplus$ operation [31] 
> $$\tag{11} \delta\mathbf{x} = -{{\left( {\mathbf{J}^T}\mathbf{J} + \mu\mathbf{I} \right)}^{-1}}{\mathbf{J}^T}\mathbf{r}$$
> where 
> $$\begin{aligned} \tag*{} \delta\mathbf{x} & = \left[\ \cdots \ ^{C_l}_{L_0}\boldsymbol{\phi}^T \quad {\delta}{^{C_l}_{L_0}\mathbf{t}^T} \ \cdots \ \right]^T \in \mathbb{R}^{6h} \\ \mathbf{x} & = \left[\ \cdots \ ^{C_l}_{L_0}\mathbf{R} \quad {^{C_l}_{L_0}\mathbf{t}} \ \cdots \ \right] \\ \mathbf{J} & = \left[\ \cdots \ \mathbf{J}^{T}_{p}\ \cdots\ \right]^T \\ \mathbf{r} & = \left[\ \cdots \ \mathbf{r}_{p}\ \cdots\ \right]^T \end{aligned}$$

非线性优化 (10) 可通过 LM 方法求解，即用残差的一阶导数 (11) 进行近似。然后通过迭代求解 (11) 并使用 $\boxplus$ 运算[31]将 $\delta\mathbf{x}$ 更新为 $\mathbf{x}$ 来获得最优 $\mathcal{E}^*_C$ 
$$\tag{11} \delta\mathbf{x} = -{\left( {\mathbf{J}^T}\mathbf{J} + \mu\mathbf{I} \right)}^{-1}{\mathbf{J}^T}\mathbf{r}$$
其中 
$$\begin{aligned} \tag*{} \delta\mathbf{x} & = \left[\ \cdots \ ^{C_l}_{L_0}\boldsymbol{\phi}^T \quad {\delta}{^{C_l}_{L_0}\mathbf{t}^T} \ \cdots \ \right]^T \in \mathbb{R}^{6h} \\ \mathbf{x} & = \left[\ \cdots \ ^{C_l}_{L_0}\mathbf{R} \quad {^{C_l}_{L_0}\mathbf{t}} \ \cdots \ \right] \\ \mathbf{J} & = \left[\ \cdots \ \mathbf{J}^{T}_{p}\ \cdots\ \right]^T \\ \mathbf{r} & = \left[\ \cdots \ \mathbf{r}_{p}\ \cdots\ \right]^T \end{aligned}$$

> with $\mathbf{J}_p$ and $\mathbf{r}_p$ being the sum of $\mathbf{J}_{i,l,j}$ and $\mathbf{r}_{i,l,j}$ , respectively, when $l = p$ 
> $$\begin{aligned} \tag{12} \mathbf{J}_{i,l,j} & = {\mathbf{n}^{T}_{i,l,j}}{\frac{\partial\mathbf{f}(\mathbf{p})}{\partial\mathbf{p}}}{\frac{\partial\boldsymbol{\pi}(\mathbf{P})}{\partial\mathbf{P}}} \left[ -{^{C_l}_{L_0}\mathbf{R}}{\left( {^{L_0}\mathbf{p}_i} \right)}^{\land} \quad \mathbf{I}\right] \in \mathbb{R}^{1 \times 6} \\ {^{L_0}\mathbf{p}_i} & = {\left( {^{G}_{L_0}\mathbf{T}_{t_i}} \right)}^{-1}{^G\mathbf{p}_i} \end{aligned}$$ 

当 $l = p$ 时， $\mathbf{J}_p$ 和 $\mathbf{r}_p$ 分别是 $\mathbf{J}_{i,l,j}$ 和 $\mathbf{r}_{i,l,j}$ 之和 
$$\begin{aligned} \tag{12} \mathbf{J}_{i,l,j} & = {\mathbf{n}^{T}_{i,l,j}}{\frac{\partial\mathbf{f}(\mathbf{p})}{\partial\mathbf{p}}}{\frac{\partial\boldsymbol{\pi}(\mathbf{P})}{\partial\mathbf{P}}} \left[ -{^{C_l}_{L_0}\mathbf{R}}{\left( {^{L_0}\mathbf{p}_i} \right)}^{\land} \quad \mathbf{I}\right] \in \mathbb{R}^{1 \times 6} \\ {^{L_0}\mathbf{p}_i} & = {\left( {^{G}_{L_0}\mathbf{T}_{t_i}} \right)}^{-1}{^G\mathbf{p}_i} \end{aligned}$$

### E. Calibration Pipeline

> [图片6]()

图 6. 我们提出的方法的工作流程：多激光雷达外校准（浅蓝色区域）和激光雷达-相机外校准（浅绿色区域）。自适应体素化在黄色矩形包围的步骤中生效。

The workflow of our proposed multisensor calibration is shown in Fig. 6. At the beginning of the calibration, the base LiDAR’s raw point cloud is processed by a LiDAR inertial odometry and mapping (LOAM) algorithm [3] to obtain the initial base LiDAR trajectory $\mathcal{S}$ . Then, the raw point cloud of all LiDARs is segmented by time into point cloud patches, i.e., $\mathcal{P}_{L_i,t_j}, L_i \in \mathcal{L}, t_j \in \mathcal{T}$ that is collected under the pose $^G_{L_i}\mathbf{T}_{t_j}$ .

我们提出的多传感器校准的工作流程如图 6 所示。在校准开始时，基准 LiDAR 的原始点云由 LiDAR 惯性里程计和地图绘制 (LOAM) 算法 [3] 处理，以获得初始基准 LiDAR 轨迹 $\mathcal{S}$ 。然后，所有 LiDAR 的原始点云按时间分割成点云块，即在位姿 $^G_{L_i}\mathbf{T}_{t_j}$ 下收集的$\mathcal{P}_{L_i,t_j}, L_i \in \mathcal{L}, t_j \in \mathcal{T}$。

In multi-LiDAR extrinsic calibration, the base LiDAR poses $\mathcal{S}$ are first optimized using the base LiDAR’s point cloud patches $\mathcal{P}_{L_0,t_j}$ . It is noticed that only $\mathcal{S}$ is involved and optimized in (3). Then, the extrinsic $\mathcal{E}_L$ are calibrated by aligning the point cloud from the LiDAR to be calibrated with those from the base LiDAR. In this stage’s problem formulation (3), $\mathcal{S}$ is fixed at the optimized values from the previous stage, and only $\mathcal{E}_L$ is optimized. Finally, both $\mathcal{S}$ and $\mathcal{E}_L$ are jointly optimized using the entire point cloud patches. In each iteration of the optimization (over $\mathcal{S}$ , $\mathcal{E}_L$ , or both), the adaptive voxelization (as described in Section III-B) is performed with the current value of $\mathcal{S}$ and $\mathcal{E}_L$ . Moreover, the Hessian matrix $\mathbf{H}$ has a computation complexity of $O(N^2)$ , where $N$ is the number of points. In practice, to reduce this computational complexity, we downsample the number of points scanned from the same LiDAR to 4 in each voxel. Such a process would lower the time complexity of the proposed algorithm to $O(N_\text{voxel})$ , where $N_\text{voxel}$ is the total number of adaptive voxels. In Section IV-A1 [experiment (2)], $N_\text{voxel} \approx 9 \times 10^3$ , which is greatly smaller than the total number of raw LiDAR points in this scene, i.e., $N_\text{points} \approx 4 \times 10^7$ .

在多激光雷达外参标定中，首先使用基础激光雷达的点云块 $\mathcal{P}_{L_0,t_j}$ 优化基础激光雷达位姿 $\mathcal{S}$ 。值得注意的是，在 (3) 中仅涉及并优化了 $\mathcal{S}$ 。然后，通过将待标定激光雷达的点云与基础激光雷达的点云对齐来标定外参 $\mathcal{E}_L$ 。在此阶段的问题公式 (3) 中， $\mathcal{S}$ 固定在上一阶段的优化值，并且仅优化 $\mathcal{E}_L$ 。最后，使用整个点云块联合优化 $\mathcal{S}$ 和 $\mathcal{E}_L$ 。在优化的每次迭代中（针对 $\mathcal{S}$ 、 $\mathcal{E}_L$ 或两者），使用 $\mathcal{S}$ 和 $\mathcal{E}_L$ 的当前值执行自适应体素化（如第 III-B 节所述）。此外，Hessian 矩阵 $\mathbf{H}$ 的计算复杂度为 $O(N^2)$ ，其中 $N$ 是点数。实际上，为了降低这种计算复杂度，我们将从同一 LiDAR 扫描的点数下采样到每个体素中的 4 个。这样的过程将使所提算法的时间复杂度降低到 $O(N_\text{voxel})$ ，其中 $N_\text{voxel}$ 是自适应体素的总数。在 IV-A1 节 [实验 (2)] 中， $N_\text{voxel} \approx 9 \times 10^3$ ，这大大小于此场景中原始 LiDAR 点的总数，即 $N_\text{points} \approx 4 \times 10^7$ 。

In multi-LiDAR–camera extrinsic calibration, the adaptive voxel map obtained with $\mathcal{S}^*$ and $\mathcal{E}^*_L$ in the previous step is used to extract the depth-continuous edges (Section III-D). Then, those 3-D edges are backprojected onto each image using the extrinsic parameter $\mathcal{E}_C$ and are matched with 2-D Canny edges extracted from the image. By minimizing the residuals defined by these two edges, we iteratively solve for the optimal $\mathcal{E}^*_C$ with the Ceres Solver.

在多 LiDAR 相机外参校准中，上一步中用$\mathcal{S}^*$和$\mathcal{E}^*_L$获得的自适应体素图用于提取深度连续边缘（第 III-D 节）。然后，使用外参$\mathcal{E}_C$将这些 3-D 边缘反向投影到每幅图像上，并与从图像中提取的 2-D Canny 边缘进行匹配。通过最小化这两个边缘定义的残差，我们使用 Ceres Solver 迭代求解最佳$\mathcal{E}^*_C$ 。

> [!TIP]
> 第四章仅翻译笔者认为比较重要的内容，并不全文翻译。

## IV. EXPERIMENTS AND RESULTS

> We have verified our proposed work with the data collected in two random test scenes on our campus, as shown in Fig. 8. Scene-1 is a square in front of the main library building with moving pedestrians, and scene-2 is an open area near a garden. The calibration data are collected in both scenes by rotating the sensor suite slightly for more than 360° and keeping this platform still every few degrees. Keeping the robot platform still during data collection enables us to acquire a dense enough point cloud from each LiDAR at each pose and also eliminates the problem caused by motion distortion and time synchronization. The timestamps $\mathcal{T}$ are manually selected so that only the point cloud and image data are chosen when the robot platform is still. During sensor suite rotation, a dedicated LOAM algorithm loam-livox [3] is called to estimate a rough LiDAR pose trajectory $\mathcal{S}_{\text{init}}$ , which serves as the initial pose in our factor graph optimization. Moreover, to obtain an initial estimate of the extrinsic $\mathcal{E}_{L_{\text{init}}}$ , we collected another data, the initialization data, which are collected in scene-1 with an “8”-figure path. Similarly, loam-livox is used to estimate each LiDAR’s trajectory, based on which the extrinsic is solved by a standard hand-eye calibration. All experiments are conducted on a desktop computer with an i7-9700K processor and 32-GB RAM.

我们已经使用校园内两个随机测试场景中收集的数据验证了我们提出的工作，如图 8 所示。场景 1 是主图书馆建筑前的一个广场，有移动的行人，场景 2 是花园附近的空地。通过将传感器套件稍微旋转 360 度以上并每隔几度保持该平台静止，可以在两个场景中收集校准数据。在数据收集期间保持机器人平台静止使我们能够从每个 LiDAR 在每个位姿下获取足够密集的点云，还可以消除运动失真和时间同步引起的问题。时间戳 $\mathcal{T}$ 是手动选择的，以便在机器人平台静止时仅选择点云和图像数据。在传感器套件旋转期间，调用专用的 LOAM 算法 loam-livox [3] 来估计粗略的 LiDAR 位姿轨迹 $\mathcal{S}_{\text{init}}$ ，该轨迹作为我们因子图优化中的初始位姿。此外，为了获得外参 $\mathcal{E}_{L_{\text{init}}}$ 的初始估计，我们收集了另一个数据，即初始化数据，这些数据是在场景 1 中以“8”字形路径收集的。同样，使用 loam-livox 估计每个 LiDAR 的轨迹，在此基础上通过标准手眼校准求解外参。所有实验均在配备 i7-9700K 处理器和 32 GB RAM 的台式计算机上进行。

## V. CONCLUSION

> In this article, we proposed a targetless extrinsic calibration method for multiple small FoV LiDARs and cameras. Unlike existing ICP-based methods, which rely on the k-d tree in LiDAR feature correspondences matching, our proposed work implemented an adaptive voxel map to store and search for the feature points to save the calibration time. We also formulated the multiple LiDAR extrinsic calibration into a BA problem and derived the cost function up to second order to boost the solving process. In LiDAR–camera extrinsic calibration, we reused the above constructed adaptive voxel map to shorten LiDAR plane feature extraction and edge feature estimation time. Compared with the RANSAC-based methods, our work improved both computation efficiency and accuracy. It is believed that this open-sourced work will benefit the community of autonomous navigation robots and highresolution mapping, especially when the sensor setups include small FoV LiDARs with few or even no FoV overlap.

在本文中，我们提出了一种针对多个小视场 LiDAR 和相机的无目标外参校准方法。与现有的基于 ICP 的方法（它们依赖于 LiDAR 特征对应匹配中的 k-d 树）不同，我们提出的工作实现了自适应体素图来存储和搜索特征点以节省校准时间。我们还将多个 LiDAR 外参校准公式化为 BA 问题，并推导出二阶成本函数以加速求解过程。在 LiDAR-相机外参校准中，我们重用了上面构建的自适应体素图来缩短 LiDAR 平面特征提取和边缘特征估计时间。与基于 RANSAC 的方法相比，我们的工作提高了计算效率和准确性。相信这项开源工作将使自主导航机器人和高分辨率测绘社区受益，尤其是当传感器设置包括视场重叠很少甚至没有的小型视场 LiDAR 时。

> Though no external calibration target is required, it is noted that our proposed work relies on the existence of natural plane features (structured building walls, ground, and so on) in the calibration environment. The precision and robustness of the extrinsic calibration among LiDARs and cameras are based on the correct extraction of LiDAR plane features. Thus, our proposed work is less reliable in unstructured scenes (e.g., country field, mountain valley, or forest). Given appropriate calibration scenes with sufficient plane features, it is believed that our proposed work could produce both fast and accurate calibration results. In our future work, we wish to take the sensor measurement’s noise model and camera intrinsic parameters into consideration.

虽然不需要外参校准目标，但需要注意的是，我们提出的工作依赖于校准环境中自然平面特征（结构化建筑物墙壁、地面等）的存在。LiDAR 和相机之间的外参校准的精度和稳健性取决于 LiDAR 平面特征的正确提取。因此，我们提出的工作在非结构化场景（例如乡村田野、山谷或森林）中不太可靠。如果具有足够的平面特征，则我们相信我们提出的工作可以产生快速而准确的校准结果。在未来的工作中，我们希望考虑传感器测量的噪声模型和相机固有参数。

## APPENDIX A

未完待续......