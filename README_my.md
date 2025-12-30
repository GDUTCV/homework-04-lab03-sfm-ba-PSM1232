# 练习4：运动恢复结构

## 学生信息
* **姓名:** [彭思敏]
* **学号:** [2112504465]

## 项目概览
本项目完成了计算机视觉课程练习4的所有内容，实现了一个完整的增量式SfM管道，主要包含以下三个模块：
1.  **预处理:** 实现了 SIFT 特征提取、特征匹配 (Ratio Test) 以及基于 RANSAC 的几何验证（计算本质矩阵），构建了场景图 (Scene Graph)。
2.  **增量式重建:** 实现了从两视图初始化到不断注册新视图的过程，包括 PnP姿态估计和三角化恢复 3D 点。
3.  **光束法平差:** 使用非线性最小二乘法优化相机位姿和3D点坐标，以最小化重投影误差。

## 网络资源使用说明
根据作业要求，我在下方列出了在完成作业过程中参考的外部资源和代码思路：

### 1. 特征匹配与几何验证 (`preprocess.py`)
* **SIFT与KNN匹配:**
    * 关于 `cv2.BFMatcher` 和 `knnMatch` 的用法，查阅了OpenCV 官方教程。关于 Lowe's Ratio Test 的实现（`m.distance < 0.7 * n.distance`），我是直接参考文档中的示例代码写的。
* **本质矩阵 Essential Matrix:**
    * 在使用 `cv2.findEssentialMatrix` 时，查阅了CSDN上关于RANSAC去除误匹配的文章来确定参数设置。

### 2. 增量式 SfM (`sfm.py`)
* **旋转向量与矩阵转换:**
    * 在实现 `cv2.solvePnP` 后，需要将返回的旋转向量转换为旋转矩阵。参考了AI说明的关于“OpenCV PnP 姿态解算”，明确了它们之间的转换关系。
* **三角化 (Triangulation):**
    * 理解 `cv2.triangulatePoints` 的输入输出格式（它输出的是 4D 齐次坐标，需要除以最后一维归一化）。

### 3. 光束法平差 (`bundle_adjustment.py`)
* **残差计算 (Residuals):**
    * 借助AI写出正确的投影公式（P = K[R|t]然后做透视除法）。
    * 关于如何使用 `scipy.optimize.least_squares`，我参考了SciPy 官方文档，关于如何将相机参数和3D点坐标变成一个一维数组传给优化器。

## 总结
本次作业让我完整跑通了SfM的流程。
* **核心体会：** 代码中最容易出错的地方是坐标系的变换和数组的维度（比如 `N x 3` 还是 `3 x N`）。
* **BA的作用：** 在没有运行 Bundle Adjustment 时，生成的点云虽然大致有形状，但细节比较杂乱，加上 BA 后，重投影误差显著下降，重建出的神庙结构更加清晰、紧凑。

## 运行说明
1.  **环境依赖:**
    * 安装 `numpy`, `opencv-python`, `scipy`, `open3d`。
2.  **运行步骤:**
    * **第一步预处理:** `python preprocess.py --dataset temple`、`python preprocess.py --dataset mini-temple`
        * *注：如果遇到多进程报错，我在代码里把 `num_workers` 改为了 0。*
    * **第二步SfM重建:**
        * 不带BA(速度快): `python sfm.py --dataset mini-temple`
        * 带BA(精度高): `python sfm.py --dataset mini-temple --ba`
    * **第三步可视化:** `python visualize.py`
        * 这会弹出一个Open3D窗口显示生成的稀疏点云。