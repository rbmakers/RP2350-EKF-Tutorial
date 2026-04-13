# 卡爾曼濾波器（KF）與擴展卡爾曼濾波器（EKF）完整教材

> **適用對象**：電機工程、航空工程、嵌入式系統開發者  
> **版本**：2025 年版｜傳統中文  
> **應用實例**：慣性導航（INS）、四行程電子燃油噴射（4-Stroke EFI）引擎控制

---

## 目錄

1. [統計學基礎](#1-統計學基礎)  
   1.1 隨機變數與機率分佈  
   1.2 期望值、變異數與協方差  
   1.3 條件機率與貝氏定理  
   1.4 高斯分佈的特殊性質  
2. [狀態空間模型](#2-狀態空間模型)  
3. [卡爾曼濾波器（KF）](#3-卡爾曼濾波器kf)  
   3.1 系統假設  
   3.2 五大遞推方程式  
   3.3 卡爾曼增益推導  
   3.4 誤差協方差更新推導  
4. [擴展卡爾曼濾波器（EKF）](#4-擴展卡爾曼濾波器ekf)  
   4.1 線性化動機  
   4.2 雅可比矩陣  
   4.3 EKF 完整演算法  
5. [KF 與 EKF 比較表](#5-kf-與-ekf-比較表)  
6. [應用一：慣性導航系統（INS/GNSS 融合）](#6-應用一慣性導航系統insgnss-融合)  
7. [應用二：四行程 EFI 引擎控制](#7-應用二四行程-efi-引擎控制)  
8. [調參指南與常見問題](#8-調參指南與常見問題)  
9. [參考文獻](#9-參考文獻)

---

## 1. 統計學基礎

### 1.1 隨機變數與機率分佈

**隨機變數（Random Variable）** 是一個將樣本空間映射到實數軸的函數。在狀態估計問題中，感測器讀數、系統雜訊皆為隨機變數。

**機率密度函數（PDF）** 滿足：

$$\Large \int_{-\infty}^{+\infty} p(x)\, dx = 1, \quad p(x) \geq 0$$

---

### 1.2 期望值、變異數與協方差

**期望值（Mean）**：

$$\Large \mu = E[x] = \int_{-\infty}^{+\infty} x\, p(x)\, dx$$

**變異數（Variance）**：

$$\Large \sigma^2 = E\bigl[(x - \mu)^2\bigr] = E[x^2] - \mu^2$$

**協方差（Covariance）**：衡量兩個隨機變數的線性相關程度。

$$\Large \text{Cov}(x, y) = E\bigl[(x - \mu_x)(y - \mu_y)\bigr]$$

**協方差矩陣（Covariance Matrix）**：對 $n$ 維隨機向量 $\mathbf{x}$，

$$\Large \mathbf{P} = E\bigl[(\mathbf{x} - \boldsymbol{\mu})(\mathbf{x} - \boldsymbol{\mu})^\top\bigr]$$

其中 $\mathbf{P}$ 為對稱正半定矩陣（Symmetric Positive Semi-Definite），對角元素為各分量的變異數，非對角元素為協方差。

---

### 1.3 條件機率與貝氏定理

**條件機率（Conditional Probability）**：

$$\Large p(x \mid y) = \frac{p(x, y)}{p(y)}$$

**全機率定理（Total Probability Theorem）**：

$$\Large p(y) = \int p(y \mid x)\, p(x)\, dx$$

**貝氏定理（Bayes' Theorem）**：

$$\Large \boxed{p(x \mid y) = \frac{p(y \mid x)\, p(x)}{p(y)}}$$

> **直覺解釋**：  
> - $p(x)$：**先驗（Prior）**，觀測前對狀態的信念  
> - $p(y \mid x)$：**似然（Likelihood）**，給定狀態 $x$ 觀測到 $y$ 的機率  
> - $p(x \mid y)$：**後驗（Posterior）**，觀測到 $y$ 後對狀態的信念  
>
> 卡爾曼濾波器正是在**高斯假設**下，對貝氏定理的最優閉合形式解。

---

### 1.4 高斯分佈的特殊性質

**單變數高斯分佈（Univariate Gaussian）**：

$$\Large \mathcal{N}(x;\, \mu, \sigma^2) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\!\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$$

**多變數高斯分佈（Multivariate Gaussian）**：

$$\Large \mathcal{N}(\mathbf{x};\, \boldsymbol{\mu}, \mathbf{P}) = \frac{1}{(2\pi)^{n/2}|\mathbf{P}|^{1/2}} \exp\!\left(-\frac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^\top \mathbf{P}^{-1} (\mathbf{x}-\boldsymbol{\mu})\right)$$

**高斯分佈的關鍵性質**：

| 性質 | 說明 |
|------|------|
| **線性封閉性** | 高斯的線性變換仍為高斯 |
| **乘積封閉性** | 兩高斯的乘積（歸一化後）仍為高斯 |
| **邊際分佈** | 聯合高斯的邊際分佈仍為高斯 |
| **條件分佈** | 聯合高斯的條件分佈仍為高斯 |

**兩高斯乘積的結果**（卡爾曼更新步驟的核心）：

設 $p_1 = \mathcal{N}(\mu_1, \sigma_1^2)$，$p_2 = \mathcal{N}(\mu_2, \sigma_2^2)$，則：

$$\Large p_1 \cdot p_2 \propto \mathcal{N}\!\left(\frac{\sigma_2^2 \mu_1 + \sigma_1^2 \mu_2}{\sigma_1^2 + \sigma_2^2},\; \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2}\right)$$

合併均值可改寫為：

$$\Large \mu_{\text{post}} = \mu_1 + \underbrace{\frac{\sigma_1^2}{\sigma_1^2 + \sigma_2^2}}_{K} (\mu_2 - \mu_1)$$

這正是**卡爾曼增益 $K$** 的一維原型。

---

## 2. 狀態空間模型

線性時變離散系統的狀態空間表示：

**過程方程式（Process / State Transition Equation）**：

$$\Large \mathbf{x}_k = \mathbf{F}_{k-1}\, \mathbf{x}_{k-1} + \mathbf{B}_{k-1}\, \mathbf{u}_{k-1} + \mathbf{w}_{k-1}$$

**觀測方程式（Measurement / Observation Equation）**：

$$\Large \mathbf{z}_k = \mathbf{H}_k\, \mathbf{x}_k + \mathbf{v}_k$$

**符號定義表**：

| 符號 | 維度 | 名稱 | 說明 |
|------|------|------|------|
| $\mathbf{x}_k$ | $n \times 1$ | 狀態向量 | 系統真實狀態（隱變數） |
| $\mathbf{F}_{k-1}$ | $n \times n$ | 狀態轉移矩陣 | 描述系統動態 |
| $\mathbf{B}_{k-1}$ | $n \times m$ | 控制輸入矩陣 | 控制信號如何影響狀態 |
| $\mathbf{u}_{k-1}$ | $m \times 1$ | 控制向量 | 已知輸入 |
| $\mathbf{w}_{k-1}$ | $n \times 1$ | 過程雜訊 | $\sim \mathcal{N}(\mathbf{0}, \mathbf{Q}_{k-1})$ |
| $\mathbf{z}_k$ | $p \times 1$ | 觀測向量 | 感測器量測值 |
| $\mathbf{H}_k$ | $p \times n$ | 觀測矩陣 | 狀態如何映射到觀測 |
| $\mathbf{v}_k$ | $p \times 1$ | 觀測雜訊 | $\sim \mathcal{N}(\mathbf{0}, \mathbf{R}_k)$ |
| $\mathbf{Q}_{k-1}$ | $n \times n$ | 過程雜訊協方差 | 模型不確定性 |
| $\mathbf{R}_k$ | $p \times p$ | 觀測雜訊協方差 | 感測器雜訊 |

**雜訊假設**：

$$\Large E[\mathbf{w}_k] = \mathbf{0}, \quad E[\mathbf{w}_k \mathbf{w}_j^\top] = \mathbf{Q}_k \delta_{kj}$$

$$\Large E[\mathbf{v}_k] = \mathbf{0}, \quad E[\mathbf{v}_k \mathbf{v}_j^\top] = \mathbf{R}_k \delta_{kj}$$

$$\Large E[\mathbf{w}_k \mathbf{v}_j^\top] = \mathbf{0} \quad \forall\, k, j$$

其中 $\delta_{kj}$ 為克羅內克三角函數（Kronecker delta），雜訊之間互不相關。

---

## 3. 卡爾曼濾波器（KF）

### 3.1 系統假設

卡爾曼濾波器在以下**四個假設**下為最優估計器（BLUE — Best Linear Unbiased Estimator）：

| 假設 | 數學表述 |
|------|----------|
| **線性系統** | $\mathbf{F}$、$\mathbf{H}$ 為線性矩陣 |
| **高斯雜訊** | $\mathbf{w} \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$，$\mathbf{v} \sim \mathcal{N}(\mathbf{0}, \mathbf{R})$ |
| **初始狀態高斯** | $\mathbf{x}_0 \sim \mathcal{N}(\hat{\mathbf{x}}_0, \mathbf{P}_0)$ |
| **雜訊獨立** | $\mathbf{w}$、$\mathbf{v}$、$\mathbf{x}_0$ 兩兩獨立 |

---

### 3.2 五大遞推方程式

KF 演算法分為**預測（Predict）**與**更新（Update）**兩個階段，共五個方程式。

#### 📌 預測階段（Time Update / Prediction Step）

**① 先驗狀態估計**：

$$\Large \hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_{k-1}\, \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{B}_{k-1}\, \mathbf{u}_{k-1}$$

**② 先驗誤差協方差**：

$$\Large \mathbf{P}_{k|k-1} = \mathbf{F}_{k-1}\, \mathbf{P}_{k-1|k-1}\, \mathbf{F}_{k-1}^\top + \mathbf{Q}_{k-1}$$

#### 📌 更新階段（Measurement Update / Correction Step）

**③ 卡爾曼增益（Kalman Gain）**：

$$\Large \boxed{\mathbf{K}_k = \mathbf{P}_{k|k-1}\, \mathbf{H}_k^\top \Bigl(\mathbf{H}_k\, \mathbf{P}_{k|k-1}\, \mathbf{H}_k^\top + \mathbf{R}_k\Bigr)^{-1}}$$

**④ 後驗狀態估計**：

$$\Large \hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \underbrace{\Bigl(\mathbf{z}_k - \mathbf{H}_k\, \hat{\mathbf{x}}_{k|k-1}\Bigr)}_{\text{新息（Innovation）}\; \tilde{\mathbf{y}}_k}$$

**⑤ 後驗誤差協方差**：

$$\Large \mathbf{P}_{k|k} = \bigl(\mathbf{I} - \mathbf{K}_k\, \mathbf{H}_k\bigr)\, \mathbf{P}_{k|k-1}$$

> **數值穩定形式（Joseph Form）**：  
> 為確保 $\mathbf{P}_{k|k}$ 的對稱正定性，在實作中建議使用：
>
> $$\Large \mathbf{P}_{k|k} = \bigl(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k\bigr)\, \mathbf{P}_{k|k-1}\, \bigl(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k\bigr)^\top + \mathbf{K}_k\, \mathbf{R}_k\, \mathbf{K}_k^\top$$

---

### 3.3 卡爾曼增益推導

卡爾曼增益的目標是**最小化後驗誤差協方差的跡（trace）**，即最小化估計誤差的總變異數。

**定義後驗估計誤差**：

$$\Large \mathbf{e}_{k|k} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k}$$

代入更新方程式 ④：

$$\Large \mathbf{e}_{k|k} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1} - \mathbf{K}_k\bigl(\mathbf{z}_k - \mathbf{H}_k\, \hat{\mathbf{x}}_{k|k-1}\bigr)$$

代入 $\mathbf{z}_k = \mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k$：

$$\Large \mathbf{e}_{k|k} = \bigl(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k\bigr)\underbrace{\bigl(\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}\bigr)}_{\mathbf{e}_{k|k-1}} - \mathbf{K}_k \mathbf{v}_k$$

**計算後驗協方差** $\mathbf{P}_{k|k} = E[\mathbf{e}_{k|k}\, \mathbf{e}_{k|k}^\top]$：

由於 $\mathbf{e}_{k|k-1}$ 與 $\mathbf{v}_k$ 獨立（互不相關）：

$$\Large \mathbf{P}_{k|k} = \bigl(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k\bigr)\mathbf{P}_{k|k-1}\bigl(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k\bigr)^\top + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^\top$$

展開：

$$\Large \mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k\mathbf{H}_k\mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1}\mathbf{H}_k^\top\mathbf{K}_k^\top + \mathbf{K}_k\bigl(\mathbf{H}_k\mathbf{P}_{k|k-1}\mathbf{H}_k^\top + \mathbf{R}_k\bigr)\mathbf{K}_k^\top$$

**對 $\mathbf{K}_k$ 求最佳化**，對 $\text{tr}(\mathbf{P}_{k|k})$ 微分並令其為零：

$$\Large \frac{\partial\, \text{tr}(\mathbf{P}_{k|k})}{\partial\, \mathbf{K}_k} = -2\mathbf{P}_{k|k-1}\mathbf{H}_k^\top + 2\mathbf{K}_k\bigl(\mathbf{H}_k\mathbf{P}_{k|k-1}\mathbf{H}_k^\top + \mathbf{R}_k\bigr) = \mathbf{0}$$

解得：

$$\Large \boxed{\mathbf{K}_k = \mathbf{P}_{k|k-1}\,\mathbf{H}_k^\top\,\bigl(\mathbf{H}_k\,\mathbf{P}_{k|k-1}\,\mathbf{H}_k^\top + \mathbf{R}_k\bigr)^{-1}}$$

將最優 $\mathbf{K}_k$ 代回，可得方程式 ⑤ 的簡化形式：

$$\Large \mathbf{P}_{k|k} = \bigl(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k\bigr)\mathbf{P}_{k|k-1}$$

**卡爾曼增益的物理意義**：

$$\Large \mathbf{K}_k = \frac{\text{模型不確定性（先驗協方差的觀測方向分量）}}{\text{模型不確定性} + \text{感測器不確定性}}$$

- 當 $\mathbf{R}_k \to \mathbf{0}$（感測器非常精確）：$\mathbf{K}_k \to \mathbf{H}_k^{-1}$，完全信任觀測  
- 當 $\mathbf{R}_k \to \infty$（感測器非常嘈雜）：$\mathbf{K}_k \to \mathbf{0}$，完全信任預測  
- 當 $\mathbf{P}_{k|k-1} \to \mathbf{0}$（模型非常精確）：$\mathbf{K}_k \to \mathbf{0}$，完全信任預測

---

### 3.4 誤差協方差更新推導

**預測步驟的協方差傳播**：

定義先驗估計誤差：

$$\Large \mathbf{e}_{k|k-1} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}$$

代入 $\mathbf{x}_k = \mathbf{F}_{k-1}\mathbf{x}_{k-1} + \mathbf{w}_{k-1}$ 及 $\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_{k-1}\hat{\mathbf{x}}_{k-1|k-1}$：

$$\Large \mathbf{e}_{k|k-1} = \mathbf{F}_{k-1}\underbrace{(\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1})}_{\mathbf{e}_{k-1|k-1}} + \mathbf{w}_{k-1}$$

由於 $\mathbf{e}_{k-1|k-1}$ 與 $\mathbf{w}_{k-1}$ 獨立：

$$\Large \mathbf{P}_{k|k-1} = E[\mathbf{e}_{k|k-1}\,\mathbf{e}_{k|k-1}^\top] = \mathbf{F}_{k-1}\,\mathbf{P}_{k-1|k-1}\,\mathbf{F}_{k-1}^\top + \mathbf{Q}_{k-1}$$

此即方程式 ②。

---

## 4. 擴展卡爾曼濾波器（EKF）

### 4.1 線性化動機

現實系統多為非線性：

$$\Large \mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_{k-1}) + \mathbf{w}_{k-1}$$

$$\Large \mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k$$

其中 $f(\cdot)$ 為非線性狀態轉移函數，$h(\cdot)$ 為非線性觀測函數。

**EKF 的核心思想**：在當前估計點附近，對非線性函數進行**一階泰勒展開（First-Order Taylor Expansion）**，將其線性化，再套用 KF 框架。

對 $f$ 在 $\hat{\mathbf{x}}_{k-1|k-1}$ 展開：

$$\Large f(\mathbf{x}) \approx f(\hat{\mathbf{x}}_{k-1|k-1}) + \underbrace{\frac{\partial f}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k-1|k-1}}}_{\mathbf{F}_{k-1}} (\mathbf{x} - \hat{\mathbf{x}}_{k-1|k-1})$$

對 $h$ 在 $\hat{\mathbf{x}}_{k|k-1}$ 展開：

$$\Large h(\mathbf{x}) \approx h(\hat{\mathbf{x}}_{k|k-1}) + \underbrace{\frac{\partial h}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k|k-1}}}_{\mathbf{H}_k} (\mathbf{x} - \hat{\mathbf{x}}_{k|k-1})$$

---

### 4.2 雅可比矩陣（Jacobian Matrix）

**狀態轉移雅可比矩陣**（$n \times n$）：

$$\Large \mathbf{F}_{k-1} = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k-1|k-1}} = \begin{bmatrix} \dfrac{\partial f_1}{\partial x_1} & \cdots & \dfrac{\partial f_1}{\partial x_n} \\ \vdots & \ddots & \vdots \\ \dfrac{\partial f_n}{\partial x_1} & \cdots & \dfrac{\partial f_n}{\partial x_n} \end{bmatrix}_{\hat{\mathbf{x}}_{k-1|k-1}}$$

**觀測雅可比矩陣**（$p \times n$）：

$$\Large \mathbf{H}_k = \frac{\partial h}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k|k-1}} = \begin{bmatrix} \dfrac{\partial h_1}{\partial x_1} & \cdots & \dfrac{\partial h_1}{\partial x_n} \\ \vdots & \ddots & \vdots \\ \dfrac{\partial h_p}{\partial x_1} & \cdots & \dfrac{\partial h_p}{\partial x_n} \end{bmatrix}_{\hat{\mathbf{x}}_{k|k-1}}$$

---

### 4.3 EKF 完整演算法

#### 📌 初始化

$$\Large \hat{\mathbf{x}}_{0|0} = E[\mathbf{x}_0], \quad \mathbf{P}_{0|0} = E\bigl[(\mathbf{x}_0 - \hat{\mathbf{x}}_{0|0})(\mathbf{x}_0 - \hat{\mathbf{x}}_{0|0})^\top\bigr]$$

#### 📌 預測階段

**① EKF 先驗狀態估計**（使用非線性函數）：

$$\Large \hat{\mathbf{x}}_{k|k-1} = f\bigl(\hat{\mathbf{x}}_{k-1|k-1},\, \mathbf{u}_{k-1}\bigr)$$

**② EKF 先驗誤差協方差**（使用雅可比矩陣）：

$$\Large \mathbf{P}_{k|k-1} = \mathbf{F}_{k-1}\, \mathbf{P}_{k-1|k-1}\, \mathbf{F}_{k-1}^\top + \mathbf{Q}_{k-1}$$

#### 📌 更新階段

**③ EKF 卡爾曼增益**：

$$\Large \mathbf{K}_k = \mathbf{P}_{k|k-1}\, \mathbf{H}_k^\top \Bigl(\mathbf{H}_k\, \mathbf{P}_{k|k-1}\, \mathbf{H}_k^\top + \mathbf{R}_k\Bigr)^{-1}$$

**④ EKF 後驗狀態估計**（創新使用非線性觀測函數）：

$$\Large \hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \Bigl(\mathbf{z}_k - h\bigl(\hat{\mathbf{x}}_{k|k-1}\bigr)\Bigr)$$

**⑤ EKF 後驗誤差協方差**：

$$\Large \mathbf{P}_{k|k} = \bigl(\mathbf{I} - \mathbf{K}_k\, \mathbf{H}_k\bigr)\, \mathbf{P}_{k|k-1}$$

> **關鍵差異**：EKF 的預測步驟 ① 使用**非線性函數** $f(\cdot)$ 計算均值，  
> 但協方差傳播步驟 ② 使用**雅可比矩陣** $\mathbf{F}_{k-1}$ 進行線性化傳播。  
> 同理，更新步驟 ④ 的新息使用**非線性函數** $h(\cdot)$，而增益計算使用**雅可比矩陣** $\mathbf{H}_k$。

---

## 5. KF 與 EKF 比較表

### 5.1 演算法結構對比

| 項目 | 卡爾曼濾波器（KF） | 擴展卡爾曼濾波器（EKF） |
|------|-------------------|------------------------|
| **適用系統** | 線性系統 | 非線性系統（一階線性化） |
| **狀態轉移** | $\mathbf{F}\hat{\mathbf{x}}_{k-1}$ | $f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1})$ |
| **觀測函數** | $\mathbf{H}\hat{\mathbf{x}}_k$ | $h(\hat{\mathbf{x}}_{k\|k-1})$ |
| **協方差預測** | $\mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$ | $\mathbf{F}_J\mathbf{P}\mathbf{F}_J^\top + \mathbf{Q}$（雅可比） |
| **增益計算** | $\mathbf{P}\mathbf{H}^\top(\mathbf{H}\mathbf{P}\mathbf{H}^\top+\mathbf{R})^{-1}$ | $\mathbf{P}\mathbf{H}_J^\top(\mathbf{H}_J\mathbf{P}\mathbf{H}_J^\top+\mathbf{R})^{-1}$（雅可比） |
| **最優性** | 全域最優（線性高斯假設下） | 局部最優（線性化誤差存在） |
| **計算量** | 低（矩陣乘法） | 中等（需計算雅可比矩陣） |
| **實現複雜度** | 簡單 | 中等（需推導雅可比解析式） |
| **收斂性** | 保證收斂（穩定線性系統） | 不保證（高度非線性時可能發散） |

### 5.2 雜訊矩陣調整準則

| 矩陣 | 物理意義 | 調大效果 | 調小效果 | 典型初值策略 |
|------|----------|----------|----------|-------------|
| $\mathbf{Q}$（過程雜訊協方差） | 模型不確定性 | 更快追蹤動態變化（響應靈敏） | 估計更平滑（抗雜訊） | 依模型誤差估計 |
| $\mathbf{R}$（觀測雜訊協方差） | 感測器雜訊 | 更信任預測（忽略感測器） | 更信任感測器（快速修正） | 感測器規格書 Allan Variance |
| $\mathbf{P}_0$（初始協方差） | 初始估計不確定性 | 初始快速修正 | 初始收斂慢 | 通常設大值（保守） |

### 5.3 KF 系列濾波器家族對比

| 濾波器 | 全名 | 核心方法 | 適用場景 | 計算代價 |
|--------|------|----------|----------|----------|
| **KF** | Kalman Filter | 精確線性代數 | 線性高斯系統 | $O(n^2)$ |
| **EKF** | Extended KF | 一階泰勒展開 | 弱非線性系統 | $O(n^2)$ + 雅可比 |
| **UKF** | Unscented KF | Sigma 點採樣 | 中等非線性 | $O(n^3)$（Sigma 點） |
| **PF** | Particle Filter | 蒙地卡羅採樣 | 強非線性/非高斯 | $O(N_p n)$（粒子數） |
| **IEKF** | Iterated EKF | 多次重線性化 | EKF 不收斂時 | $O(I \cdot n^2)$（迭代） |
| **ESKF** | Error-State KF | 誤差狀態建模 | IMU 積分（慣性導航） | $O(n^2)$（小誤差線性化） |

---

## 6. 應用一：慣性導航系統（INS/GNSS 融合）

### 6.1 問題描述

**慣性導航系統（INS）** 使用加速度計（Accelerometer）和陀螺儀（Gyroscope）積分計算位置與姿態，但存在**積分漂移**（Drift）。**GNSS**（如 GPS）提供絕對位置但更新率低（1–10 Hz）且有多路徑雜訊。EKF 可融合兩者，取長補短。

### 6.2 狀態向量設計

定義 15 維誤差狀態（Error-State EKF）：

$$\Large \boldsymbol{\delta x} = \begin{bmatrix} \boldsymbol{\delta p} \\ \boldsymbol{\delta v} \\ \boldsymbol{\delta \psi} \\ \boldsymbol{\delta b}_a \\ \boldsymbol{\delta b}_g \end{bmatrix} \in \mathbb{R}^{15}$$

| 子向量 | 維度 | 含義 |
|--------|------|------|
| $\boldsymbol{\delta p}$ | $3 \times 1$ | 位置誤差（NED 座標系，m） |
| $\boldsymbol{\delta v}$ | $3 \times 1$ | 速度誤差（m/s） |
| $\boldsymbol{\delta \psi}$ | $3 \times 1$ | 姿態誤差（小角度，rad） |
| $\boldsymbol{\delta b}_a$ | $3 \times 1$ | 加速度計偏差（m/s²） |
| $\boldsymbol{\delta b}_g$ | $3 \times 1$ | 陀螺儀偏差（rad/s） |

### 6.3 非線性狀態方程式

在 NED 座標系下，IMU 積分的連續時間方程式：

$$\Large \dot{\mathbf{p}} = \mathbf{v}$$

$$\Large \dot{\mathbf{v}} = \mathbf{R}_{bn}\bigl(\tilde{\mathbf{a}}_b - \mathbf{b}_a - \mathbf{n}_a\bigr) + \mathbf{g}$$

$$\Large \dot{\mathbf{q}} = \frac{1}{2}\boldsymbol{\Omega}\bigl(\tilde{\boldsymbol{\omega}}_b - \mathbf{b}_g - \mathbf{n}_g\bigr)\mathbf{q}$$

$$\Large \dot{\mathbf{b}}_a = \mathbf{n}_{ba}, \quad \dot{\mathbf{b}}_g = \mathbf{n}_{bg}$$

其中：
- $\mathbf{R}_{bn}$：機體到導航座標系的旋轉矩陣（由四元數 $\mathbf{q}$ 計算）
- $\tilde{\mathbf{a}}_b$：加速度計讀數（含偏差與雜訊）
- $\tilde{\boldsymbol{\omega}}_b$：陀螺儀讀數（含偏差與雜訊）
- $\mathbf{g}$：重力向量（$[0, 0, 9.81]^\top$ m/s²）

### 6.4 EKF 線性化（誤差狀態傳播）

對誤差狀態建立連續時間線性化方程式：

$$\Large \dot{\boldsymbol{\delta x}} = \mathbf{F}_c\, \boldsymbol{\delta x} + \mathbf{G}_c\, \mathbf{n}$$

離散化後（零階保持，採樣週期 $T_s$）：

$$\Large \mathbf{F}_{k-1} = e^{\mathbf{F}_c T_s} \approx \mathbf{I} + \mathbf{F}_c T_s + \frac{(\mathbf{F}_c T_s)^2}{2} + \cdots$$

連續時間雅可比矩陣的關鍵子塊：

$$\Large \mathbf{F}_c = \begin{bmatrix} \mathbf{0}_{3} & \mathbf{I}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} \\ \mathbf{0}_{3} & \mathbf{0}_{3} & -\mathbf{R}_{bn}[\tilde{\mathbf{a}}_b - \mathbf{b}_a]_\times & -\mathbf{R}_{bn} & \mathbf{0}_{3} \\ \mathbf{0}_{3} & \mathbf{0}_{3} & -[\tilde{\boldsymbol{\omega}}_b - \mathbf{b}_g]_\times & \mathbf{0}_{3} & -\mathbf{I}_{3} \\ \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} \\ \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} \end{bmatrix}$$

其中 $[\cdot]_\times$ 為反對稱矩陣（skew-symmetric matrix）。

### 6.5 觀測模型（GNSS 更新）

GNSS 觀測到絕對位置與速度（此處以位置更新為例）：

$$\Large \mathbf{z}_k^{\text{GNSS}} = \begin{bmatrix} \mathbf{p}_k^{\text{GNSS}} \end{bmatrix} = h(\boldsymbol{\delta x}) + \mathbf{v}_k$$

觀測雅可比矩陣（選取位置部分）：

$$\Large \mathbf{H}_k = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \end{bmatrix} \in \mathbb{R}^{3 \times 15}$$

GNSS 觀測雜訊協方差（依 CEP 規格）：

$$\Large \mathbf{R}_{\text{GNSS}} = \begin{bmatrix} \sigma_N^2 & 0 & 0 \\ 0 & \sigma_E^2 & 0 \\ 0 & 0 & \sigma_D^2 \end{bmatrix} \approx \begin{bmatrix} 4 & 0 & 0 \\ 0 & 4 & 0 \\ 0 & 0 & 16 \end{bmatrix} \text{ m}^2$$

（垂直方向精度通常較差）

### 6.6 INS/GNSS 融合系統框架

```
┌─────────────────────────────────────────────────────┐
│                     IMU (200 Hz)                    │
│  加速度計: ã_b   陀螺儀: ω̃_b                        │
└──────────────────────┬──────────────────────────────┘
                       │ 高頻積分
                       ▼
┌─────────────────────────────────────────────────────┐
│              INS 機械化計算 (Mechanization)          │
│  p̂, v̂, q̂ = ∫ f(ã_b, ω̃_b) dt                     │
└──────────────────────┬──────────────────────────────┘
                       │ 預測狀態 (200 Hz)
                       ▼
┌─────────────────────────────────────────────────────┐
│              EKF 預測步驟 (200 Hz)                  │
│  x̂_{k|k-1}, P_{k|k-1}                              │
└──────────────────────┬──────────────────────────────┘
                       │ 當 GNSS 資料到達 (1~10 Hz)
         ┌─────────────┤
         │ GNSS 觀測  ▼
         │  ┌──────────────────────────────────────┐
         │  │          EKF 更新步驟                 │
         │  │  K_k, x̂_{k|k}, P_{k|k}              │
         └──┤  誤差狀態回饋修正 INS 狀態            │
            └──────────────────────────────────────┘
```

### 6.7 UAV 常用參數範例

| 參數 | 典型值 | 備註 |
|------|--------|------|
| IMU 更新率 | 200–1000 Hz | BMI088 可達 400 Hz |
| GNSS 更新率 | 1–10 Hz | u-blox M9N 預設 1 Hz |
| $\sigma_a$（加速度計雜訊） | 0.1–1.0 mg/√Hz | 見 Allan 方差圖 |
| $\sigma_g$（陀螺儀雜訊） | 0.01–0.1 °/s/√Hz | — |
| $\sigma_{ba}$（加速度計偏差不穩定性） | 0.01–0.5 mg | — |
| $\sigma_{bg}$（陀螺儀偏差不穩定性） | 1–10 °/hr | — |
| GNSS 水平精度（CEP） | 1.5–3 m | 開放天空 |

---

## 7. 應用二：四行程 EFI 引擎控制

### 7.1 問題描述

四行程汽油引擎（4-Stroke Gasoline Engine）的電子燃油噴射（Electronic Fuel Injection, EFI）需要精確估計：
1. **引擎轉速（RPM）**：用於點火正時（Ignition Timing）與噴油時序
2. **節氣門位置與進氣量（MAP）**：計算空燃比（AFR, Air-Fuel Ratio）
3. **排氣溫度（EGT）**：防止過熱（Overtemperature Protection）
4. **汽缸壓力狀態**：感測器雜訊需要濾波

感測器雜訊來源包括：引擎振動、電磁干擾（EMI）、熱噪聲（Thermal Noise）等。EKF 在此扮演**狀態估計器**角色，提供高品質的估計供控制器使用。

### 7.2 四行程引擎行程回顧

| 行程 | 曲軸角度 | 活塞動作 | 閥門狀態 | 備註 |
|------|---------|---------|---------|------|
| **進氣（Intake）** | 0°–180° TDC→BDC | 下行 | 進氣閥開 | 空燃混合氣進入 |
| **壓縮（Compression）** | 180°–360° BDC→TDC | 上行 | 全閉 | 混合氣壓縮 |
| **做功（Power/Combustion）** | 360°–540° TDC→BDC | 下行 | 全閉 | 點火做功 |
| **排氣（Exhaust）** | 540°–720° BDC→TDC | 上行 | 排氣閥開 | 廢氣排出 |

點火時機通常在壓縮行程末端，上死點前（BTDC）10°–35°，依轉速與負荷調整。

### 7.3 狀態向量設計

定義 5 維狀態向量：

$$\Large \mathbf{x}_k = \begin{bmatrix} \omega_k \\ \dot{\omega}_k \\ T_{\text{EGT},k} \\ P_{\text{MAP},k} \\ \lambda_k \end{bmatrix}$$

| 狀態 | 單位 | 物理意義 |
|------|------|----------|
| $\omega_k$ | rad/s | 曲軸角速度（引擎轉速） |
| $\dot{\omega}_k$ | rad/s² | 角加速度 |
| $T_{\text{EGT},k}$ | °C | 排氣溫度 |
| $P_{\text{MAP},k}$ | kPa | 進氣歧管壓力 |
| $\lambda_k$ | — | 過量空氣係數（Lambda，目標 = 1.0） |

### 7.4 非線性狀態方程式

**轉速動態**（牛頓旋轉定律 + 燃燒扭矩）：

$$\Large \omega_{k} = \omega_{k-1} + T_s \dot{\omega}_{k-1} + w_{\omega}$$

$$\Large \dot{\omega}_{k} = \frac{1}{J}\Bigl[T_{\text{comb}}(\omega_{k-1}, \lambda_{k-1}, P_{k-1}) - T_{\text{load}} - b\,\omega_{k-1}\Bigr] + w_{\dot{\omega}}$$

燃燒扭矩的非線性函數：

$$\Large T_{\text{comb}}(\omega, \lambda, P) = \eta_{\text{vol}}(\omega)\cdot \frac{V_d}{4\pi}\cdot P\cdot Q_{lhv}\cdot \frac{1}{\lambda\cdot A_s\cdot (1+A_s)}$$

**EGT 動態**（一階熱模型）：

$$\Large T_{\text{EGT},k} = T_{\text{EGT},k-1} + T_s \cdot \frac{T_{\text{comb\_gas}} - T_{\text{EGT},k-1}}{\tau_{\text{therm}}} + w_T$$

其中 $\tau_{\text{therm}}$ 為熱時間常數（典型值 0.5–2 s）。

**MAP 動態**（進氣歧管填充動態）：

$$\Large P_{\text{MAP},k} = P_{\text{MAP},k-1} + T_s \cdot \frac{R_{\text{air}} T_{\text{int}}}{V_{\text{man}}} \bigl(\dot{m}_{\text{thr}} - \dot{m}_{\text{eng}}\bigr) + w_P$$

**Lambda 動態**（空燃比控制迴路）：

$$\Large \lambda_k = \lambda_{k-1} + T_s \cdot k_\lambda \bigl(\lambda_{\text{cmd}} - \lambda_{k-1}\bigr) + w_\lambda$$

### 7.5 觀測方程式

可用感測器與觀測模型：

$$\Large \mathbf{z}_k = \begin{bmatrix} z_{\text{Hall}} \\ z_{\text{MAP}} \\ z_{\text{EGT}} \\ z_{\text{O}_2} \end{bmatrix} = \begin{bmatrix} f_{\text{Hall}}(\omega_k) \\ P_{\text{MAP},k} \\ T_{\text{EGT},k} \\ g(\lambda_k) \end{bmatrix} + \mathbf{v}_k$$

**Hall 感測器觀測函數**（每過一個齒觸發一次）：

$$\Large z_{\text{Hall}} = \frac{N_t \cdot \omega_k}{2\pi} + v_{\text{Hall}}$$

其中 $N_t$ 為觸發齒數（例如 36-1 缺齒輪盤，$N_t = 35$）。

**窄帶氧感測器（Narrowband O₂ Sensor）觀測函數**：

$$\Large g(\lambda) = \begin{cases} 0.1 \text{ V} & \lambda > 1.0 + \epsilon \\ 0.9 \text{ V} & \lambda < 1.0 - \epsilon \\ 0.45 + 4(\lambda - 1.0) & \text{過渡區} \end{cases}$$

此為高度非線性函數，EKF 線性化尤為重要。

### 7.6 觀測雅可比矩陣

$$\Large \mathbf{H}_k = \frac{\partial \mathbf{h}}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k|k-1}} = \begin{bmatrix} \dfrac{N_t}{2\pi} & 0 & 0 & 0 & 0 \\[8pt] 0 & 0 & 0 & 1 & 0 \\[8pt] 0 & 0 & 1 & 0 & 0 \\[8pt] 0 & 0 & 0 & 0 & \dfrac{\partial g}{\partial \lambda}\bigg|_{\hat{\lambda}} \end{bmatrix}$$

其中 $\dfrac{\partial g}{\partial \lambda}$ 在過渡區約為 $4$ V，飽和區為 $0$（需注意可觀測性問題）。

### 7.7 雜訊協方差矩陣設計

**過程雜訊協方差**（依各狀態的模型不確定性）：

$$\Large \mathbf{Q} = \text{diag}\!\left(\sigma_\omega^2,\; \sigma_{\dot\omega}^2,\; \sigma_T^2,\; \sigma_P^2,\; \sigma_\lambda^2\right) = \text{diag}\!\left(0.01,\; 100,\; 25,\; 1,\; 0.001\right)$$

**觀測雜訊協方差**（依感測器規格）：

$$\Large \mathbf{R} = \text{diag}\!\left(\sigma_{\text{Hall}}^2,\; \sigma_{\text{MAP}}^2,\; \sigma_{\text{EGT}}^2,\; \sigma_{O_2}^2\right) = \text{diag}\!\left(4,\; 0.25,\; 100,\; 0.01\right)$$

### 7.8 EKF 在 EFI 中的應用流程

```
曲軸位置觸發（每0.5°或每齒）
        │
        ▼
┌───────────────────────────────────────────────────────┐
│                  EKF 預測步驟                         │
│  x̂_{k|k-1} = f(x̂_{k-1}, u_{k-1})                   │
│  P_{k|k-1} = F J · P_{k-1} · F J^T + Q              │
└──────────────────────────┬────────────────────────────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
         Hall 觸發      MAP 量測     EGT 量測  (+ O₂)
              │            │            │
              └────────────┼────────────┘
                           │
                           ▼
┌───────────────────────────────────────────────────────┐
│                  EKF 更新步驟                         │
│  K_k = P H^T (H P H^T + R)^{-1}                      │
│  x̂_{k|k} = x̂_{k|k-1} + K(z - h(x̂_{k|k-1}))        │
│  P_{k|k} = (I - K H) P_{k|k-1}                       │
└──────────────────────────┬────────────────────────────┘
                           │
              ┌────────────┼─────────────────┐
              │            │                 │
       點火正時計算    噴油脈寬計算        EGT 過熱保護
       (ω̂, θ̂_crank)  (λ̂, P̂_MAP)        (T̂_EGT > 850°C?)
              │            │                 │
              └────────────┼─────────────────┘
                           │
                      ECU 輸出執行
```

### 7.9 點火正時計算示例

利用 EKF 估計的 $\hat{\omega}_k$ 與 $\hat{\dot{\omega}}_k$，預測點火時刻對應的曲軸角度：

**最優點火提前角（MBT — Maximum Brake Torque）估算**：

$$\Large \theta_{\text{ign}} = \theta_{\text{TDC}} - \Delta\theta_{\text{BTDC}}$$

$$\Large \Delta\theta_{\text{BTDC}} = a_0 + a_1 \hat{\omega} + a_2 P_{\text{MAP}} + a_3 T_{\text{coolant}}$$

（$a_0, a_1, a_2, a_3$ 為發動機台架標定參數）

**考慮角加速度的點火預測修正**：

當 $\hat{\dot{\omega}}_k \neq 0$ 時，從當前角度 $\theta_{\text{now}}$ 到目標角度 $\theta_{\text{ign}}$ 的到達時間：

$$\Large \Delta t_{\text{ign}} = \frac{-\hat{\omega}_k + \sqrt{\hat{\omega}_k^2 + 2\hat{\dot{\omega}}_k(\theta_{\text{ign}} - \theta_{\text{now}})}}{\hat{\dot{\omega}}_k}$$

此修正在急加速/急減速時尤為重要，可降低點火時序誤差約 3–8°。

### 7.10 EGT 過熱保護邏輯

$$\Large \text{Protection} = \begin{cases} \text{增濃}(\Delta\lambda = -0.05) & \hat{T}_{\text{EGT}} > T_{\text{warn}} = 800\,°\text{C} \\ \text{緊急增濃}(\Delta\lambda = -0.15) & \hat{T}_{\text{EGT}} > T_{\text{crit}} = 850\,°\text{C} \\ \text{斷油} & \hat{T}_{\text{EGT}} > T_{\text{cutoff}} = 900\,°\text{C} \end{cases}$$

使用 EKF 估計值（而非原始感測器值）的優勢：EGT 感測器（K 型熱電偶）響應時間 0.5–3 s，且易受振動雜訊影響。EKF 融合熱模型後，可提前預測 EGT 趨勢，實現**預測性保護（Predictive Protection）**而非**反應性保護（Reactive Protection）**。

---

## 8. 調參指南與常見問題

### 8.1 調參流程

```
Step 1: 離線確認感測器雜訊（R 矩陣）
  └─ 靜止量測感測器輸出，計算變異數
  
Step 2: 評估過程模型誤差（Q 矩陣）
  └─ 模擬與實測對比；模型殘差 → Q
  
Step 3: 設定初始協方差（P₀）
  └─ 通常設大值（保守），如 diag(100, 100, ...)
  
Step 4: 驗證可觀測性（Observability）
  └─ rank([H; HF; HF²; ...]) = n?
  
Step 5: 在線監控新息（Innovation）序列
  └─ ỹ_k 應為白雜訊且均值為零
  
Step 6: 細化調整直到 NIS（歸一化新息平方）合格
  └─ NIS = ỹ_k^T (H P H^T + R)^{-1} ỹ_k ~ χ²(p)
```

### 8.2 歸一化新息平方（NIS）測試

**NIS（Normalized Innovation Squared）**是驗證濾波器一致性的關鍵指標：

$$\Large \text{NIS}_k = \tilde{\mathbf{y}}_k^\top \mathbf{S}_k^{-1} \tilde{\mathbf{y}}_k, \quad \mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^\top + \mathbf{R}_k$$

若濾波器調參正確，$\text{NIS}_k \sim \chi^2(p)$，其中 $p$ 為觀測維度。

| NIS 結果 | 診斷 | 調整建議 |
|----------|------|----------|
| NIS 持續偏大 | 濾波器過度自信 | 增大 $\mathbf{Q}$ 或 $\mathbf{R}$ |
| NIS 持續偏小 | 濾波器過度保守 | 減小 $\mathbf{Q}$ 或 $\mathbf{R}$ |
| NIS 通過 $\chi^2$ 檢定 | 調參合格 | 維持現狀 |
| NIS 呈現週期性 | 模型未捕捉週期動態 | 檢查 $\mathbf{F}$ 矩陣 |

### 8.3 常見問題排查

| 問題現象 | 可能原因 | 解決方案 |
|----------|----------|----------|
| $\mathbf{P}_{k\|k}$ 負定或非對稱 | 數值誤差累積 | 改用 Joseph Form；使用 Cholesky 分解 |
| 估計值發散 | $\mathbf{Q}$ 過小或線性化誤差太大 | 增大 $\mathbf{Q}$；改用 UKF；加 IEKF 迭代 |
| 估計更新緩慢 | $\mathbf{K}$ 過小（$\mathbf{R}$ 過大） | 重新標定感測器；適當減小 $\mathbf{R}$ |
| EKF 於高動態時失效 | 一階線性化誤差過大 | 提高積分頻率；改用 UKF/二階 EKF |
| 偏差不收斂 | 偏差可觀測性不足 | 添加激勵（Excitation）；檢查可觀測性矩陣 |
| 點火正時抖動 | 轉速估計雜訊過大 | 調小 $Q_\omega$；增加 Hall 解析度 |

---

## 9. 參考文獻

1. **Kalman, R. E.** (1960). "A New Approach to Linear Filtering and Prediction Problems." *Journal of Basic Engineering*, 82(1), 35–45.

2. **Grewal, M. S., & Andrews, A. P.** (2015). *Kalman Filtering: Theory and Practice Using MATLAB* (4th ed.). Wiley-IEEE Press.

3. **Thrun, S., Burgard, W., & Fox, D.** (2005). *Probabilistic Robotics*. MIT Press. *(第 3 章：高斯濾波器)*

4. **Titterton, D., & Weston, J.** (2004). *Strapdown Inertial Navigation Technology* (2nd ed.). IET.

5. **Heywood, J. B.** (1988). *Internal Combustion Engine Fundamentals*. McGraw-Hill.

6. **Rhudy, M. B., Gu, Y., Gross, J., & Napolitano, M. R.** (2013). "Evaluation of Matrix Square Root Operations for UKF within a UAV GPS/INS Sensor Fusion Application." *International Journal of Navigation and Observation*.

7. **Simon, D.** (2006). *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*. Wiley-Interscience. *(第 13–15 章：EKF)*

8. **RP2350 Datasheet** (2024). Raspberry Pi Ltd. *(硬體實作平台參考)*

---

> **版權聲明**：本教材由傳統中文撰寫，供教學與研究使用。  
> **最後更新**：2025 年｜適用於 RB-RP2354A EFI/ECU 韌體開發教育用途

