[README.md](https://github.com/user-attachments/files/26658897/README.md)
# 卡爾曼濾波器 & 擴展卡爾曼濾波器 — 完整教學（繁體中文完整版）

> 適用對象：具備基礎線性代數的工程師與學生  
> 目標：建立 KF/EKF 的統計直覺，並理解每個矩陣的物理意義  

---

## 1. 統計基礎

### 1.1 變異數與標準差

$$
\sigma^2 = \mathbb{E}[(x - \mu)^2]
$$

- $\mu$：平均值  
- $\sigma^2$：變異數  
- $\sigma$：標準差  

👉 直覺：$\sigma = 2m$ → 約 68% 落在 ±2m

---

### 1.2 協方差矩陣

$$
\text{cov}(x_1,x_2)=\mathbb{E}[(x_1-\mu_1)(x_2-\mu_2)]
$$

$$
\Sigma =
\begin{bmatrix}
\sigma_1^2 & cov(x_1,x_2) \\
cov(x_2,x_1) & \sigma_2^2
\end{bmatrix}
$$

👉 Σ = 不確定性地圖

---

### 1.3 精度矩陣

$$
\Lambda = \Sigma^{-1}
$$

👉 Σ = 不確定  
👉 Σ⁻¹ = 信心

---

### 1.4 高斯分布

$$
p(x) \propto \exp\left(-\frac{1}{2}(x-\mu)^T \Sigma^{-1}(x-\mu)\right)
$$

---

### 1.5 馬氏距離

$$
d^2 = (x-\mu)^T \Sigma^{-1}(x-\mu)
$$

👉 會考慮不確定性

---

## 2. 卡爾曼濾波器

### 2.1 系統模型

$$
x_k = A x_{k-1} + B u_k + w_k
$$

$$
z_k = H x_k + v_k
$$

---

### 2.2 Predict

$$
\hat{x}_k^- = A\hat{x}_{k-1} + Bu_k
$$

$$
P_k^- = A P_{k-1} A^T + Q
$$

---

### 2.3 Update

$$
y_k = z_k - H\hat{x}_k^-
$$

$$
S_k = H P_k^- H^T + R
$$

$$
K_k = P_k^- H^T S_k^{-1}
$$

$$
\hat{x}_k = \hat{x}_k^- + K_k y_k
$$

$$
P_k = (I - K_k H) P_k^-
$$

---

### 2.4 Kalman Gain

$$
K = P H^T (HPH^T + R)^{-1}
$$

| 條件 | 結果 |
|------|------|
| R 小 | 信感測器 |
| R 大 | 信模型 |
| P 大 | 信感測器 |
| P 小 | 信模型 |

---

### 2.5 1D 範例

$$
K = \frac{4}{4+1}=0.8
$$

$$
x = 10.8,\quad P=0.8
$$

---

## 3. EKF

### 3.1 非線性模型

$$
x_k = f(x_{k-1},u_k)
$$

$$
z_k = h(x_k)
$$

---

### 3.2 Jacobian

$$
F = \frac{\partial f}{\partial x},\quad H=\frac{\partial h}{\partial x}
$$

---

### 3.3 EKF

Predict:
$$
\hat{x}_k^- = f(\hat{x}_{k-1})
$$

$$
P_k^- = F P F^T + Q
$$

Update:
$$
y = z - h(\hat{x})
$$

---

## 4. 調參

### R
來自感測器 variance

### Q
- IMU noise
- 經驗 tuning

---

## 5. 無人機應用

狀態：

$$
x=[p,v,attitude,bias]
$$

---

## 6. 核心直覺

👉 KF = 用「信心」加權融合

---

## 7. 記憶

Predict:
- A → 會去哪
- Q → 多不確定

Update:
- H → 看到什麼
- R → 多不準
- K → 信誰
