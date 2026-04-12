# 卡爾曼濾波器與擴展卡爾曼濾波器完整教學

> **繁體中文版** · 涵蓋：協方差 · 精確度矩陣 · 馬氏距離 · KF · EKF · 四元數 · Joseph 式更新  
> **硬體平台**：RP2354A · BMI088 · BMM350 · BMP580 · GNSS INS 導航

---

## 目錄

1. [統計基礎概念](#第一章統計基礎概念)
2. [卡爾曼濾波器（KF）](#第二章卡爾曼濾波器kf)
3. [擴展卡爾曼濾波器（EKF）](#第三章擴展卡爾曼濾波器ekf)
4. [四元數姿態表示](#第四章四元數姿態表示)
5. [Joseph 式協方差更新](#第五章joseph-式協方差更新)
6. [16 維狀態 EKF 設計](#第六章16-維狀態-ekf-設計)
7. [感測器模型](#第七章感測器模型)
8. [調參指南](#第八章調參指南)
9. [總結與記憶要訣](#第九章總結與記憶要訣)

---

## 第一章　統計基礎概念

卡爾曼濾波器的本質是以高斯分布為工具的**貝葉斯估計器**，它用精確度矩陣加權融合各資訊來源。理解以下統計概念是掌握濾波器的前提。

---

### 1.1　變異數與標準差

**變異數（σ²）** 衡量量測值相對均值的平均平方偏差：

$
\sigma^2 = \mathbb{E}\left[(x - \mu)^2\right]
$


| 量 | 符號 | 單位 | 意義 |
|----|------|------|------|
| 均值 | μ | 原始量 | 期望值 |
| 變異數 | σ² | 原始量² | 平均平方偏差 |
| 標準差 | σ | 原始量 | 1-sigma 區間半寬 |

> 💡 **直覺**：若 GPS σ = 2 m，約 68% 的讀數落在真實位置 ±2 m 內。

---

### 1.2　協方差與協方差矩陣

**協方差**衡量兩個隨機變數線性相關的程度：

$$
\mathrm{cov}(x_i, x_j) = \mathbb{E}\!\left[(x_i-\mu_i)(x_j-\mu_j)\right]
$$


**協方差矩陣 Σ** 收集所有變數對的協方差：

$$
\Sigma = \begin{pmatrix}
\sigma_1^2 & \mathrm{cov}(x_1,x_2) & \cdots \\
\mathrm{cov}(x_2,x_1) & \sigma_2^2 & \cdots \\
\vdots & \vdots & \ddots
\end{pmatrix}
$$


**關鍵性質：**
- **對稱**：$\mathrm{cov}(x_i, x_j) = \mathrm{cov}(x_j, x_i)$
- **正半定**：所有特徵值 ≥ 0
- **對角線** = 各變數的變異數

---

### 1.3　精確度矩陣（逆協方差矩陣）

$$
\Lambda = \Sigma^{-1}
$$


| 概念 | 矩陣 | 意義 | 數值示例 |
|------|------|------|----------|
| 不確定性 | Σ（協方差） | 量測有多「分散」 | σ²=4 → σ=2 m |
| 可信度 | Λ=Σ⁻¹（精確度） | 量測有多「可信」 | Λ=0.25 → 低可信 |

> 💡 **精確度矩陣是 KF「資訊加權」的核心**——精確度越高，融合時貢獻越大。  
> 在卡爾曼增益公式中，$(HPH^\top + R)^{-1}$ 就是對總不確定性求精確度。

---

### 1.4　多變量高斯分布

$$
p(\mathbf{x}) \propto \exp\!\left(-\tfrac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^\top \Sigma^{-1}(\mathbf{x}-\boldsymbol{\mu})\right)
$$


精確度矩陣 $\Sigma^{-1}$ 出現在指數項，決定分布的「形狀」（方向＋大小）。

> 💡 **關鍵性質**：兩個高斯分布之積仍是高斯分布。這是 KF「預測×更新」能夠合併為單一高斯估計的數學根基。

---

### 1.5　馬氏距離（Mahalanobis Distance）

$$
d_M^2 = (\mathbf{x}-\boldsymbol{\mu})^\top \,\Sigma^{-1}\, (\mathbf{x}-\boldsymbol{\mu})
$$


| 距離類型 | 公式 | 特性 |
|----------|------|------|
| 歐氏距離 $d_E^2$ | $(x-\mu)^\top I (x-\mu)$ | 所有方向等權 |
| 馬氏距離 $d_M^2$ | $(x-\mu)^\top \Sigma^{-1} (x-\mu)$ | 以不確定性縮放 |

> 💡 **實例**：GPS 誤差 5 m，若 σ²=25，馬氏距離=1（正常）；若 σ²=0.01，馬氏距離=500（異常值）。馬氏距離在 KF 新息步驟中就是「量測帶來的驚訝程度」。

---

## 第二章　卡爾曼濾波器（KF）

卡爾曼濾波器（Kalman Filter）是**線性高斯系統下的最優最小變異數估計器**。通過反覆執行「預測」與「更新」兩步驟，在系統模型和感測器量測之間取得最佳平衡。

---

### 2.1　系統模型

**狀態轉移方程（過程模型）：**

$$
\mathbf{x}_k = A\,\mathbf{x}_{k-1} + B\,\mathbf{u}_k + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(0, Q)
$$


**量測方程（觀測模型）：**

$$
\mathbf{z}_k = H\,\mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(0, R)
$$


---

### 2.2　矩陣速查表


| 矩陣 | 名稱 | 意義 | 計算方式 | 一句話記憶 |
|------|------|------|----------|-----------|
| **A**/**F** | 狀態轉移 | 系統如何演化 | 物理/運動學模型 | 「我認為世界如何運動」 |
| **B** | 控制矩陣 | 輸入如何影響狀態 | 系統輸入模型 | 「控制指令的效果」 |
| **Q** | 過程雜訊協方差 | 模型不確定性 | 感測器噪聲密度 × Δt | 「我對模型有多不確定」 |
| **H** | 量測矩陣 | 狀態→量測映射 | 感測器觀測模型 | 「感測器量測什麼」 |
| **R** | 量測雜訊協方差 | 感測器不確定性 | 感測器規格/實測 | 「我對感測器有多不信任」 |
| **P** | 狀態誤差協方差 | 當前估計不確定性 | 初始化後逐步更新 | 「我現在有多不確定」 |
| **K** | 卡爾曼增益 | 最佳融合權重 | $K = PH^\top(HPH^\top+R)^{-1}$ | 「如何在模型與感測器間取捨」 |

---

### 2.3　兩步驟演算法


#### 步驟一：預測（Predict）

$$
\hat{\mathbf{x}}_k^- = A\,\hat{\mathbf{x}}_{k-1} + B\,\mathbf{u}_k
$$
$$
P_k^- = A\,P_{k-1}\,A^\top + Q
$$


> **Q 矩陣的作用**：每次預測後，協方差 P 增加 Q。Q 越大 → 對模型越不信任 → 更依賴感測器修正。

#### 步驟二：更新（Update）

$$
\mathbf{y}_k = \mathbf{z}_k - H\hat{\mathbf{x}}_k^- \quad \text{（新息）}
$$
$$
S_k = H P_k^- H^\top + R \quad \text{（新息協方差）}
$$


$$
K_k = P_k^- H^\top S_k^{-1} \quad \text{（卡爾曼增益）}
$$


$$
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k\,\mathbf{y}_k
$$
$$
P_k = (I - K_k H)\,P_k^-
$$


---

### 2.4　卡爾曼增益的直覺理解


卡爾曼增益 K 是**精確度加權的融合因子**：

| 條件 | K 的趨勢 | 效果 |
|------|----------|------|
| R↓（感測器精確） | K 增大 | 更信任感測器 |
| R↑（感測器噪音大） | K 減小 | 更信任模型 |
| P↑（模型不確定） | K 增大 | 快速修正 |
| P↓（模型可靠） | K 減小 | 維持預測 |

---

### 2.5　矩陣關係圖


---

### 2.6　一維數值範例

| 步驟 | 計算式 | 結果 |
|------|--------|------|
| 先驗估計 | x̂⁻=10 m，P⁻=4 m² | — |
| 量測值 | z=11 m，R=1 m² | — |
| 新息 | y = 11−10 | **y = 1 m** |
| 新息協方差 | S = 4+1 | **S = 5 m²** |
| 卡爾曼增益 | K = 4/5 | **K = 0.8** |
| 更新狀態 | x̂ = 10+0.8×1 | **x̂ = 10.8 m** |
| 更新協方差 | P = (1−0.8)×4 | **P = 0.8 m²** |

> 💡 感測器精確度高（R=1 < P=4），估計偏向量測值 11 m。不確定性從 4 m² 降至 0.8 m²，減少 **80%**。

---

## 第三章　擴展卡爾曼濾波器（EKF）

標準 KF 要求線性系統。無人機等系統高度非線性（四元數積分、旋轉矩陣、GPS 座標轉換），EKF 通過**雅可比矩陣**在當前估計點做一階泰勒展開近似。

---

### 3.1　KF 與 EKF 對比


| 特性 | KF | EKF |
|------|----|----|
| 系統模型 | 線性 $f(x)=Ax$ | 非線性 $f(x)$ |
| 量測模型 | 線性 $h(x)=Hx$ | 非線性 $h(x)$ |
| 協方差傳播 | $APA^\top + Q$ | $FPF^\top + Q$（雅可比） |
| 最優性 | 精確最優（線性高斯） | 一階近似，可能次優 |
| 計算量 | 較低 | 中等（每步計算雅可比） |
| 發散風險 | 低 | 高度非線性時較高 |

---

### 3.2　雅可比矩陣線性化

$$
F_k = \left.\dfrac{\partial f}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}_{k-1}} \qquad
H_k = \left.\dfrac{\partial h}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}_k^-}
$$


> ⚠️ **重要**：F 和 H 在**每個時步**根據當前估計點重新計算。這是 EKF 比 KF 計算量略高的原因，也是在強非線性區域可能不準確的根源。

---

### 3.3　EKF 預測步驟

$$
\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_k) \quad \text{（用真實非線性函數）}
$$
$$
P_k^- = F_k\,P_{k-1}\,F_k^\top + Q \quad \text{（用雅可比 F 傳播協方差）}
$$


---

### 3.4　多感測器融合架構


| 更新通道 | 量測向量 z | 模型型式 | 更新頻率 |
|----------|-----------|----------|----------|
| GPS 位置+速度 | [pN,pE,pD,vN,vE,vD] | 線性（H 固定） | ~5 Hz |
| 氣壓計高度 | [pD 偏差] | 線性（H 固定） | 50 Hz |
| 磁力計方向 | [bx,by,bz] 歸一化 | 非線性（H 逐步計算） | 100 Hz |

---

## 第四章　四元數姿態表示

本系統 EKF 採用四元數替代歐拉角，是避免萬向節鎖、保持數值穩定的核心決策。

---

### 4.1　歐拉角 vs 四元數


| 性質 | 歐拉角 ZYX | 四元數 q=[q₀,q₁,q₂,q₃] |
|------|-----------|------------------------|
| 參數數量 | 3 個角度 | 4 個元素（需 \|q\|=1） |
| 萬向節鎖 | 俯仰角 ±90° 奇異 | **無奇異** |
| 積分方式 | T矩陣（含 tan θ） | 旋轉向量精確積分 |
| 計算量 | 需要 sin/cos | 純乘法 |
| 插值 | 不連續 | SLERP 球面線性插值 |

---

### 4.2　四元數運動學方程

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\Omega(\tilde{\boldsymbol{\omega}})\,\mathbf{q}, \qquad
\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_{imu} - \mathbf{b}_g
$$

$$
\Omega(\boldsymbol{\omega}) = \begin{pmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{pmatrix}
$$

![四元數運動學](eq/eq_quat_kine.png)

---

### 4.3　精確旋轉向量積分

$$
\delta\mathbf{q} = \left[\cos\tfrac{|\boldsymbol{\omega}|\Delta t}{2},\;
\hat{\boldsymbol{\omega}}\sin\tfrac{|\boldsymbol{\omega}|\Delta t}{2}\right], \quad
\mathbf{q}_{k} = \delta\mathbf{q} \otimes \mathbf{q}_{k-1}
$$

![精確積分](eq/eq_quat_integrate.png)

相比一階歐拉近似 `q += q̇·Δt`，旋轉向量積分在每步消除累積歸一化誤差，適合 500 Hz 以上的高頻 IMU 積分。

---

### 4.4　旋轉矩陣 C_bn

$$
C_{bn} = \begin{pmatrix}
q_0^2{+}q_1^2{-}q_2^2{-}q_3^2 & 2(q_1q_2{-}q_0q_3) & 2(q_1q_3{+}q_0q_2) \\
2(q_1q_2{+}q_0q_3) & q_0^2{-}q_1^2{+}q_2^2{-}q_3^2 & 2(q_2q_3{-}q_0q_1) \\
2(q_1q_3{-}q_0q_2) & 2(q_2q_3{+}q_0q_1) & q_0^2{-}q_1^2{-}q_2^2{+}q_3^2
\end{pmatrix}
$$

![旋轉矩陣 Cbn](eq/eq_Cbn.png)

> 💡 C_bn 計算僅需乘法和加法，無需三角函數。在 RP2354A M33 FPU 上約 ~5 µs。

---

## 第五章　Joseph 式協方差更新

---

### 5.1　標準式 vs Joseph 式

**標準式：**
$$
P^+_{\text{標準}} = (I - KH)\,P^-
$$

**Joseph 式：**
$$
P^+_J = (I - KH)\,P^-(I - KH)^\top + K\,R\,K^\top
$$

![Joseph 式](eq/eq_joseph.png)

| 性質 | 標準式 | Joseph 式 |
|------|--------|-----------|
| 保證對稱 | 需另行對稱化 | 由構造自動保證 |
| 保證正定 | 可能負定（發散） | 由構造保證 |
| 次優 K 下正確 | 否 | **是** |
| 計算量 | 低（教學用） | 約 2×（可接受） |

> 💡 **為何選 Joseph 式**：磁力計（100 Hz）和氣壓計（50 Hz）的長時間高頻更新可能使標準式協方差逐漸喪失正定性，Joseph 式從根本上防止此問題。

---

## 第六章　16 維狀態 EKF 設計

---

### 6.1　狀態向量

$$
\mathbf{x}_{16} = \bigl[\,
\underbrace{p_N,p_E,p_D}_{\text{位置NED}},\,
\underbrace{v_N,v_E,v_D}_{\text{速度NED}},\,
\underbrace{q_0,q_1,q_2,q_3}_{\text{四元數}},\,
\underbrace{b_{ax},b_{ay},b_{az}}_{\text{加速計偏差}},\,
\underbrace{b_{gx},b_{gy},b_{gz}}_{\text{陀螺儀偏差}}\,\bigr]
$$

![狀態向量](eq/eq_state_vec.png)

| 分組 | 維度 | 單位 | 說明 |
|------|------|------|------|
| NED 位置 | 3 | m | 相對起飛點的平面地球近似 |
| NED 速度 | 3 | m/s | 北東下座標系 |
| 四元數 | 4 | — | Hamilton 慣例，純量在前，\|q\|=1 |
| 加速計偏差 | 3 | m/s² | 隨機遊走追蹤 |
| 陀螺儀偏差 | 3 | rad/s | 隨機遊走追蹤 |

---

### 6.2　Jacobian F（16×16）分塊結構

| 分塊位置 | 大小 | 物理意義 | 計算方式 |
|----------|------|----------|----------|
| F[0:3, 3:6] | 3×3 | dp/dv | I·Δt |
| F[3:6, 6:10] | 3×4 | dv/dq | ∂(C_bn·f̃)/∂q 解析微分 |
| F[3:6, 10:13] | 3×3 | dv/db_a | −C_bn·Δt |
| F[6:10, 6:10] | 4×4 | dq/dq | I + ½Ω(ω̃)·Δt |
| F[6:10, 13:16] | 4×3 | dq/db_g | −½Ξ(q)·Δt |
| 偏差對偏差 | 6×6 | 隨機遊走 | 單位矩陣 |

其餘分塊均為零矩陣，這個稀疏結構大幅減少 F·P·Fᵀ 的計算量。

---

## 第七章　感測器模型

---

### 7.1　氣壓高度計（BMP580）

$$
h = 44330\!\left[1 - \!\left(\dfrac{P}{P_0}\right)^{\!0.190295}\right] \;\mathrm{(m)}
$$

![氣壓高度公式](eq/eq_baro.png)

- **P₀**：海平面參考氣壓，起飛時對齊 GNSS 高度
- **有效範圍**：< 11 km（對流層）
- **BMP580 特性**：20-bit 壓力，IIR-4 濾波，50 Hz 輸出

---

### 7.2　磁力計（BMM350）非線性量測

量測模型（非線性）：
```
h(q) = C_bn^T(q) · m̂_ned
```

量測雅可比 H_mag（3×16）僅四元數欄（列 6-9）非零：

$$
H_{\text{mag}}[:,6:10] = \left.\dfrac{\partial(C_{bn}^\top \hat{m})}{\partial \mathbf{q}}\right|_{\hat{q}}
$$

> ⚠️ **校準要求**：使用前必須完成硬鐵（Hard-Iron）偏差校準（圖8飛行軌跡）和軟鐵（Soft-Iron）校準（橢球擬合）。

---

### 7.3　GNSS 位置量測

量測向量：`z_GPS = [pN, pE, pD, vN, vE, vD]`

H 矩陣（線性，固定不變）：
```
H_GPS = [I₃  0₃  0₄  0₃  0₃]   （前3行選位置）
         [0₃  I₃  0₄  0₃  0₃]   （後3行選速度）
```

**LLA → NED 平面地球近似**（有效範圍 ≤ 10 km）：
- N = Δφ · R_earth
- E = Δλ · R_earth · cos(φ₀)
- D = −(h − h₀)

---

## 第八章　調參指南

![調參指南](diag/diag_tuning.png)

### 8.1　症狀→原因→解決

| 症狀 | 可能原因 | 解決方案 |
|------|----------|----------|
| 估計滯後真實值 | Q 太小（過度信任模型） | 增大 Q 的對角元素 |
| 輸出噪聲過大 | Q 太大 | 減小 Q |
| 完全忽略感測器 | R 設定太大 | 減小對應 R |
| 濾波器發散 | P₀ 或 Q 太小 | 增大初始 P₀ 和 Q |
| 四元數失範化 | 更新後未歸一化 | 每次更新後呼叫 `quat_normalise()` |
| 協方差負定 | 未用 Joseph 式 | 改用 `P⁺=(I-KH)P(I-KH)ᵀ+KRKᵀ` |
| GPS 更新造成跳躍 | 未做新息閘值 | 加入 χ² 異常值拒絕 |

---

### 8.2　Q 矩陣調參參考

| Q 分量 | 物理意義 | 推薦初始值 | 調整方向 |
|--------|----------|------------|----------|
| Q_vel | 未建模加速度（風力） | ~0.1 m²/s³·Δt | 滯後→增大；抖動→減小 |
| Q_quat | 姿態角速隨機遊走 | ~5e-4 rad²/s·Δt | 參考陀螺儀 ARW 規格 |
| Q_bias_a | 加速計偏差不穩定性 | ~1e-6 m²/s⁴·Δt | 參考 Bias Instability 規格 |
| Q_bias_g | 陀螺儀偏差不穩定性 | ~1e-8 rad²/s³·Δt | 參考 Bias Instability 規格 |

---

### 8.3　R 矩陣調參參考

| R 分量 | 感測器 | 推薦初始值 | 說明 |
|--------|--------|------------|------|
| R_GPS_pos_H | GNSS 水平位置 | 4.0 m²（2m 1σ） | 依 GPS CEP 規格 |
| R_GPS_pos_V | GNSS 垂直位置 | 9.0 m²（3m 1σ） | 垂直精度較差 |
| R_GPS_vel | GNSS 速度 | 0.25 m²/s² | 通常比位置精確 |
| R_baro | BMP580 氣壓高度 | 0.25 m²（0.5m 1σ） | IIR-4 後實測方差 |
| R_mag | BMM350 磁力計 | 0.04（歸一化） | 受磁性干擾可動態調整 |

---

## 第九章　總結與記憶要訣

> **設計哲學一句話**：以精確度（不確定性之逆）為權重，最優融合所有資訊來源。

---

### 9.1　矩陣一句話記憶

| 矩陣 | 一句話記憶 |
|------|-----------|
| **A / F** | 「我認為世界如何運動」（模型/雅可比） |
| **B** | 「控制指令如何影響狀態」 |
| **Q** | 「我對自己的模型有多不確定」 |
| **H** | 「感測器實際量測的是什麼」 |
| **R** | 「我對感測器有多不信任」 |
| **P** | 「我對當前估計有多不確定」 |
| **K** | 「如何在模型與感測器之間取得最佳平衡」 |
| **Σ⁻¹** | 「精確度越高，貢獻越大」 |

---

### 9.2　三條核心要點

1. **KF vs EKF**：KF 用線性矩陣 A, H；EKF 用非線性函數 f(·), h(·) 傳播狀態，用雅可比 F, H 傳播協方差（一階近似）。

2. **四元數的優勢**：無萬向節鎖奇異性；旋轉向量精確積分消除累積歸一化誤差；C_bn 計算僅需乘法。

3. **Joseph 式的必要性**：對任意次優的 K 均保持 P 對稱正定；長時間高頻感測器更新時防止濾波器發散。

---

### 9.3　公式速查

| 公式 | 圖片 |
|------|------|
| 多變量高斯分布 | ![](eq/eq_gaussian.png) |
| 馬氏距離 | ![](eq/eq_mahalanobis.png) |
| 卡爾曼增益 | ![](eq/eq_kalman_gain.png) |
| KF 完整更新 | ![](eq/eq_kf_update.png) |
| Joseph 式更新 | ![](eq/eq_joseph.png) |
| 四元數運動學 | ![](eq/eq_quat_kine.png) |
| 精確旋轉積分 | ![](eq/eq_quat_integrate.png) |
| 旋轉矩陣 C_bn | ![](eq/eq_Cbn.png) |
| 氣壓高度計公式 | ![](eq/eq_baro.png) |

---

## 參考文獻

1. Welch & Bishop (2006), *"An Introduction to the Kalman Filter"*, UNC Chapel Hill.
2. Maybeck, P.S. (1979), *"Stochastic Models, Estimation and Control"*, Vol. 1.
3. Trawny & Roumeliotis (2005), *"Indirect Kalman Filter for 3D Attitude Estimation"*.
4. PX4 EKF2 文件 — <https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html>

---

*本文件由 RB-RP2354A 飛控導航系統教學系列整理*

# Kalman Filter & Extended Kalman Filter — A Complete Tutorial

> **Target audience:** Engineers and students with basic linear algebra background.  
> **Goal:** Build solid intuition for the statistics behind KF/EKF, then connect every equation to a physical meaning.

---

## Table of Contents

1. [Statistical Foundations](#1-statistical-foundations)
   - 1.1 Variance and Standard Deviation
   - 1.2 Covariance and the Covariance Matrix
   - 1.3 The Inverse Covariance (Precision) Matrix
   - 1.4 Gaussian (Normal) Distributions
   - 1.5 Mahalanobis Distance
2. [Kalman Filter (KF)](#2-kalman-filter-kf)
   - 2.1 The Core Problem
   - 2.2 System Model
   - 2.3 Matrix Glossary
   - 2.4 The Two-Step Algorithm
   - 2.5 Kalman Gain — The Key Insight
   - 2.6 Worked Example (1D)
3. [Extended Kalman Filter (EKF)](#3-extended-kalman-filter-ekf)
   - 3.1 Why KF Is Not Enough
   - 3.2 Linearisation via Jacobians
   - 3.3 EKF Algorithm
   - 3.4 KF vs EKF Comparison
4. [Matrix Reference Cheat Sheet](#4-matrix-reference-cheat-sheet)
5. [Tuning Guidelines](#5-tuning-guidelines)
6. [Application: Drone / INS Sensor Fusion](#6-application-drone--ins-sensor-fusion)
7. [Intuition Summary](#7-intuition-summary)

---

## 1. Statistical Foundations

Before touching filter equations, it is essential to understand the statistics that underpin every matrix in a Kalman Filter.

---

### 1.1 Variance and Standard Deviation

**Variance** σ² measures how spread out a set of measurements is around the mean:

```
σ² = E[ (x − μ)² ]
```

| Quantity | Symbol | Meaning |
|----------|--------|---------|
| Mean | μ | Expected (average) value |
| Variance | σ² | Average squared deviation from the mean |
| Standard deviation | σ | Square root of variance; same units as x |

**Intuition:** If a GPS reports position with σ = 2 m, roughly 68% of readings fall within ±2 m of truth.

---

### 1.2 Covariance and the Covariance Matrix

**Covariance** between two scalar variables x₁ and x₂:

```
cov(x₁, x₂) = E[ (x₁ − μ₁)(x₂ − μ₂) ]
```

- Positive: both tend to increase together
- Negative: one increases while the other decreases
- Zero: no linear relationship

For a state vector **x** = [x₁, x₂, …, xₙ]ᵀ, the **covariance matrix** Σ collects all pairwise covariances:

```
         ┌ σ₁²        cov(x₁,x₂)  ···  cov(x₁,xₙ) ┐
Σ  =     │ cov(x₂,x₁) σ₂²         ···  cov(x₂,xₙ) │
         │      ⋮                  ⋱        ⋮       │
         └ cov(xₙ,x₁) cov(xₙ,x₂) ···  σₙ²         ┘
```

**Key properties:**
- Σ is always **symmetric**: cov(xᵢ, xⱼ) = cov(xⱼ, xᵢ)
- Σ is always **positive semi-definite**: all eigenvalues ≥ 0
- Diagonal elements = individual variances
- Off-diagonal elements = how variables co-vary

**Intuition:** Σ is a complete map of *uncertainty* in your state estimate.

---

### 1.3 The Inverse Covariance (Precision) Matrix

The **precision matrix** (or **information matrix**) is defined as:

```
Λ = Σ⁻¹
```

| Concept | Matrix | Meaning |
|---------|--------|---------|
| Uncertainty | Σ | "How wrong could I be?" |
| Precision | Σ⁻¹ | "How confident am I?" |

**Scalar example:**

```
σ² = 4  →  Σ⁻¹ = 1/4 = 0.25   (low precision, high uncertainty)
σ² = 0.01 →  Σ⁻¹ = 100          (high precision, low uncertainty)
```

**Key properties of Λ = Σ⁻¹:**

1. **Encodes confidence, not spread** — large diagonal entry = tightly constrained variable
2. **Reveals conditional independence** — if Λᵢⱼ = 0, then xᵢ and xⱼ are *conditionally independent* given all other variables
3. **Natural weighting in fusion** — in sensor fusion, more precise sensors receive higher weight proportional to Σ⁻¹

**Why this matters for filtering:**

The Kalman Gain formula contains `(HPHᵀ + R)⁻¹`, which is precisely an inverse covariance — it weights the correction by how reliable the measurement is relative to the prediction.

---

### 1.4 Gaussian (Normal) Distributions

The Kalman Filter is the **optimal estimator** when noise is Gaussian. A multivariate Gaussian is:

```
p(x) ∝ exp( −½ (x − μ)ᵀ Σ⁻¹ (x − μ) )
```

Notice:
- The **covariance Σ** shapes the distribution (wide = uncertain)
- The **precision Σ⁻¹** appears in the exponent — it directly weights the "distance" from the mean

**Key property:** The product of two Gaussians is also a Gaussian. This is why combining a prediction (one Gaussian) with a measurement (another Gaussian) yields a new, sharper Gaussian — the foundation of KF.

---

### 1.5 Mahalanobis Distance

The term inside the Gaussian exponent is called the **Mahalanobis distance**:

```
d²_M = (x − μ)ᵀ Σ⁻¹ (x − μ)
```

Compare with ordinary Euclidean distance:
```
d²_E = (x − μ)ᵀ I (x − μ)      ← treats all directions equally
d²_M = (x − μ)ᵀ Σ⁻¹ (x − μ)   ← scales by uncertainty
```

**Intuition:**
- A 5 m GPS error is *small* if GPS variance is 25 m² (Mahalanobis ≈ 1.0)
- A 5 m GPS error is *large* if GPS variance is 0.01 m² (Mahalanobis = 500)

The Mahalanobis distance normalises deviations by the expected spread — used in the KF innovation step.

---

## 2. Kalman Filter (KF)

### 2.1 The Core Problem

You want to know the **true state** of a system (position, velocity, attitude…), but you only have:
- A **noisy model** of how the system evolves
- **Noisy sensor measurements**

The Kalman Filter finds the **optimal minimum-variance estimate** by fusing both sources, weighting each by its uncertainty.

---

### 2.2 System Model

The KF assumes:

**State transition (process) model:**
```
xₖ = A xₖ₋₁ + B uₖ + wₖ       wₖ ~ N(0, Q)
```

**Measurement model:**
```
zₖ = H xₖ + vₖ                  vₖ ~ N(0, R)
```

Where:
- **xₖ** — state vector at time k (what we want to know)
- **zₖ** — measurement vector (what the sensor reports)
- **uₖ** — control input (e.g., commanded thrust)
- **wₖ** — process noise (model imperfection)
- **vₖ** — measurement noise (sensor imperfection)

---

### 2.3 Matrix Glossary

| Matrix | Name | Size | Meaning | How to Obtain |
|--------|------|------|---------|---------------|
| **A** | State transition | n×n | How state evolves from k−1 to k | Physics / kinematics model |
| **B** | Control input | n×m | Maps control input u to state change | System input model |
| **Q** | Process noise covariance | n×n | Uncertainty in the model | IMU noise density; tuned |
| **H** | Measurement matrix | p×n | Maps state space → measurement space | Sensor observation model |
| **R** | Measurement noise covariance | p×p | Sensor uncertainty | Sensor datasheet / calibration |
| **P** | State (error) covariance | n×n | Current estimate uncertainty | Initialised; propagated by filter |
| **K** | Kalman gain | n×p | Optimal blending weight | Computed each timestep |

*(n = state dimension, m = control dimension, p = measurement dimension)*

---

### 2.4 The Two-Step Algorithm

The filter cycles between **Predict** and **Update** at every timestep.

#### Step 1 — Predict

Propagate state forward using the model:

```
x̂ₖ⁻ = A x̂ₖ₋₁ + B uₖ          ← predicted state (a priori)
Pₖ⁻  = A Pₖ₋₁ Aᵀ + Q           ← predicted covariance
```

- `Aᵀ` rotates the covariance ellipsoid through the model transformation
- `Q` is *added* because model errors grow uncertainty

#### Step 2 — Update (Correct)

Fuse the prediction with the new measurement:

```
Innovation:       yₖ = zₖ − H x̂ₖ⁻           ← residual (how surprised we are)
Innovation cov:   Sₖ = H Pₖ⁻ Hᵀ + R          ← total uncertainty in measurement space
Kalman gain:      Kₖ = Pₖ⁻ Hᵀ Sₖ⁻¹           ← optimal weight
State update:     x̂ₖ = x̂ₖ⁻ + Kₖ yₖ          ← corrected state (a posteriori)
Cov update:       Pₖ = (I − Kₖ H) Pₖ⁻         ← corrected covariance
```

**Numerical tip:** The *Joseph form* is more numerically stable for the covariance update:
```
Pₖ = (I − Kₖ H) Pₖ⁻ (I − Kₖ H)ᵀ + Kₖ R Kₖᵀ
```

---

### 2.5 Kalman Gain — The Key Insight

```
Kₖ = Pₖ⁻ Hᵀ (H Pₖ⁻ Hᵀ + R)⁻¹
```

Think of K as a *dial* between two extremes:

```
         K → 0                           K → H⁻¹
    (ignore sensor)                  (trust sensor fully)

         ↑                                 ↑
    R is large                        R is small
  (noisy sensor)                  (precise sensor)
  or Pₖ⁻ is small              or Pₖ⁻ is large
  (model confident)              (model uncertain)
```

| Condition | Kalman Gain | Effect |
|-----------|-------------|--------|
| R ↓ (precise sensor) | K increases | Larger correction from measurement |
| R ↑ (noisy sensor) | K decreases | Rely more on prediction |
| P ↑ (uncertain model) | K increases | Trust sensor more |
| P ↓ (confident model) | K decreases | Trust model more |

**Fundamental relationship:** K is a ratio of *precisions* — it's the inverse covariance doing its work.

---

### 2.6 Worked Example (1D)

**Scenario:** Estimating position of a robot. One step shown.

**Given:**
- Prior estimate: x̂ = 10 m, P = 4 m²
- Measurement: z = 11 m, R = 1 m²
- No motion (A = 1, Q = 0 for simplicity)

**Predict:**
```
x̂⁻ = 10 m        P⁻ = 4 m²
```

**Update:**
```
y = 11 − 10 = 1 m
S = 4 + 1 = 5 m²
K = 4 / 5 = 0.8
x̂ = 10 + 0.8 × 1 = 10.8 m
P  = (1 − 0.8) × 4 = 0.8 m²
```

**Result:** The filter moved from 10 m toward 11 m (measurement), landing at 10.8 m — weighted toward the more precise measurement. Uncertainty dropped from 4 m² to 0.8 m².

---

## 3. Extended Kalman Filter (EKF)

### 3.1 Why KF Is Not Enough

The KF assumes **linear** system and measurement models. Real systems are almost never linear:

| Real System | Nonlinearity |
|-------------|-------------|
| Drone attitude | Euler angle kinematics, quaternion integration |
| IMU integration | Trigonometric rotation matrices |
| GPS → local frame | Coordinate transformations |
| Barometric altitude | Nonlinear pressure model |

Using KF on these systems would propagate errors incorrectly because `A` would misrepresent the true dynamics.

---

### 3.2 Linearisation via Jacobians

The EKF replaces linear matrices A and H with **local linear approximations** — the Jacobian matrices of the nonlinear functions.

**Nonlinear models:**
```
xₖ = f(xₖ₋₁, uₖ) + wₖ         ← nonlinear process function
zₖ = h(xₖ) + vₖ                ← nonlinear measurement function
```

**Jacobian of f evaluated at current estimate x̂:**
```
        ∂f₁/∂x₁  ∂f₁/∂x₂  ···  ∂f₁/∂xₙ
F  =    ∂f₂/∂x₁  ∂f₂/∂x₂  ···  ∂f₂/∂xₙ      evaluated at x̂ₖ₋₁
            ⋮                        ⋮
        ∂fₙ/∂x₁  ∂fₙ/∂x₂  ···  ∂fₙ/∂xₙ
```

**Jacobian of h:**
```
        ∂h₁/∂x₁  ∂h₁/∂x₂  ···  ∂h₁/∂xₙ
H  =    ∂h₂/∂x₁  ∂h₂/∂x₂  ···  ∂h₂/∂xₙ      evaluated at x̂ₖ⁻
            ⋮                        ⋮
        ∂hₚ/∂x₁  ∂hₚ/∂x₂  ···  ∂hₚ/∂xₙ
```

**Physical meaning:** The Jacobian is the *best linear fit* to the nonlinear function at the current operating point. Errors grow when the state moves far from that point (highly nonlinear regions).

---

### 3.3 EKF Algorithm

#### Predict

```
x̂ₖ⁻ = f(x̂ₖ₋₁, uₖ)              ← propagate using nonlinear function
Pₖ⁻  = Fₖ Pₖ₋₁ Fₖᵀ + Q          ← propagate covariance using Jacobian F
```

#### Update

```
Innovation:       yₖ = zₖ − h(x̂ₖ⁻)            ← nonlinear predicted measurement
Innovation cov:   Sₖ = Hₖ Pₖ⁻ Hₖᵀ + R
Kalman gain:      Kₖ = Pₖ⁻ Hₖᵀ Sₖ⁻¹
State update:     x̂ₖ = x̂ₖ⁻ + Kₖ yₖ
Cov update:       Pₖ = (I − Kₖ Hₖ) Pₖ⁻
```

**Key difference from KF:**
- State propagation uses the true nonlinear `f(·)` and `h(·)`
- Covariance propagation uses the Jacobians `F` and `H`
- Jacobians are **recomputed at every timestep**

---

### 3.4 KF vs EKF Comparison

| Feature | Kalman Filter (KF) | Extended Kalman Filter (EKF) |
|---------|-------------------|------------------------------|
| System model | Linear: xₖ = Axₖ₋₁ | Nonlinear: xₖ = f(xₖ₋₁) |
| Measurement model | Linear: zₖ = Hxₖ | Nonlinear: zₖ = h(xₖ) |
| Covariance propagation | A P Aᵀ + Q | F P Fᵀ + Q (Jacobian F) |
| Innovation | zₖ − H x̂ₖ⁻ | zₖ − h(x̂ₖ⁻) |
| Optimality | Exact (for linear Gaussian) | Approximate (first-order) |
| Computational cost | Low | Moderate (Jacobian computation) |
| Typical applications | Linear tracking, simple 1D systems | Robotics, drones, SLAM, INS |
| Divergence risk | Low | Can diverge for strongly nonlinear systems |

---

## 4. Matrix Reference Cheat Sheet

### Complete Matrix Table

| Matrix | Full Name | Role | Definition / Calculation | Design Source |
|--------|-----------|------|--------------------------|--------------|
| **A** (or **F** in EKF) | State transition | Predicts how state evolves | Physics equations; Jacobian of f in EKF | System dynamics model |
| **B** | Control input matrix | Maps uₖ → state | Derived from system input model | Often omitted if no control |
| **Q** | Process noise covariance | Model uncertainty; uncertainty grows | Derived from noise density; tuned empirically | IMU noise specs; dynamics uncertainty |
| **H** | Measurement matrix | Maps state → sensor space | Sensor observation model; Jacobian of h in EKF | What the sensor physically measures |
| **R** | Measurement noise covariance | Sensor uncertainty | Sensor datasheet; measured variance | Calibration data |
| **P** | State error covariance | Current estimation uncertainty | Initialised large; updated each cycle | Filter propagation |
| **K** | Kalman gain | Fusion weight | K = P⁻ Hᵀ (HP⁻Hᵀ + R)⁻¹ | Computed automatically |
| **S** | Innovation covariance | Total uncertainty in measurement space | S = H P⁻ Hᵀ + R | Intermediate computation |
| **y** | Innovation / residual | How surprising the measurement is | y = z − H x̂⁻ | Used in state update |
| **I** | Identity matrix | Neutral element | Standard n×n identity | Fixed |

---

### Flow Diagram

```
                ┌──────────────────────────────────────┐
                │              PREDICT                 │
                │                                      │
                |  x̂ₖ₋₁, Pₖ₋₁ ──►  x̂ₖ⁻ = A x̂ₖ₋₁ + B uₖ   │
                │  Pₖ⁻  = A Pₖ₋₁ Aᵀ + Q                 │
                └─────────────────┬────────────────────┘
                                  │  x̂ₖ⁻, Pₖ⁻
                                  ▼
                ┌──────────────────────────────────────┐
                │              UPDATE                  │
                │                                      │
                |  zₖ ──►   y = zₖ − H x̂ₖ⁻              │
                │  S = H Pₖ⁻ Hᵀ + R                    │
                │  K = Pₖ⁻ Hᵀ S⁻¹                      │
                │  x̂ₖ = x̂ₖ⁻ + K y                      │
                │  Pₖ  = (I − KH) Pₖ⁻                  │
                └─────────────────┬────────────────────┘
                                  │  x̂ₖ, Pₖ
                                  ▼
                         (next timestep)
```

---

### Relationships Among Matrices

```
Q  ──────────────────────►  Pₖ⁻  ◄──── A, Pₖ₋₁
                                │
                        ┌───────┘
                        │
           H ──────────►│──────────────► S ◄──── R
                        │                │
                        └──────► K ◄─────┘
                                 │
                     x̂ₖ⁻ ──────►│──────────► x̂ₖ
                         y ──────┘
```

---

## 5. Tuning Guidelines

Tuning Q and R is the primary engineering task when deploying a KF/EKF. P is usually initialised large and converges quickly.

### R — Measurement Noise Covariance

R is the easier matrix to set. Use sensor specifications:

| Sensor | Typical Variance |
|--------|-----------------|
| GPS position | 1–25 m² (depends on signal quality) |
| Barometer altitude | 0.1–1 m² |
| Magnetometer heading | 0.01–0.1 rad² |
| Optical flow | depends on altitude |

**Rule:** Set R from real measurement variance. **Never set R = 0** (singular system).

### Q — Process Noise Covariance

Q is harder. Common approaches:

1. **From sensor noise density:** For an IMU with accelerometer noise density `n` [m/s²/√Hz], over Δt seconds:
   ```
   Q_accel ≈ n² × Δt
   ```

2. **From physical reasoning:** Unmodelled forces (wind, vibration) contribute residual acceleration. Estimate their typical magnitude.

3. **Empirical tuning:** Start with a diagonal Q. Increase diagonal entries if the filter is too slow to track; decrease if output is too noisy.

### P — Initial State Covariance

```
P₀ = diag(σ²_pos, σ²_vel, σ²_att, ...)
```

- **Large P₀** → fast initial convergence, first few estimates may be noisy
- **Small P₀** → slow adaptation, risk of wrong initial lock
- **Typical practice:** P₀ = diagonal with values ≈ initial uncertainty squared

### Tuning Heuristics Summary

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Estimate lags behind truth | Q too small or R too small | Increase Q |
| Estimate is too noisy | Q too large | Decrease Q |
| Sensor ignored | R too large | Decrease R |
| Estimate diverges | Q too small (overconfident model) | Increase Q or re-check A/F |
| Slow convergence | P₀ too small | Initialise P₀ larger |

---

## 6. Application: Drone / INS Sensor Fusion

### Typical State Vector

```
x = [ px, py, pz,        ← position (m)
      vx, vy, vz,        ← velocity (m/s)
      φ, θ, ψ,           ← roll, pitch, yaw (rad)
      bax, bay, baz,     ← accelerometer bias (m/s²)
      bgx, bgy, bgz ]    ← gyroscope bias (rad/s)
```

State dimension n = 15 (typical EKF for a drone INS)

### Sensor Roles

| Sensor | Update Rate | Role in Filter | Notes |
|--------|------------|----------------|-------|
| IMU (accel + gyro) | 500–2000 Hz | **Prediction** step | High rate; integrates attitude & velocity |
| GPS | 1–10 Hz | Position/velocity **update** | Absolute reference; latency matters |
| Barometer | 50–100 Hz | Altitude **update** | Good vertical; drifts with weather |
| Magnetometer | 50–100 Hz | Yaw **update** | Susceptible to magnetic interference |
| Optical flow | 100–400 Hz | Velocity **update** (indoor) | Requires height knowledge |

### Sensor Fusion Logic

```
┌─────────────────────────────────────────────────────────────┐
│  IMU @ 1 kHz                                                │
│  ──────────► EKF Predict every 1 ms                        │
│              (f integrates IMU, F = kinematic Jacobian)     │
└─────────────────────────────┬───────────────────────────────┘
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
         GPS @ 5 Hz     Baro @ 50 Hz    Mag @ 50 Hz
         EKF Update     EKF Update      EKF Update
         (pos, vel)     (alt)           (yaw)
```

### Why Inverse Covariance Matters Here

At fusion time, the Kalman Gain effectively computes:

```
K ∝ P⁻¹_prediction × (P_prediction + R_sensor)⁻¹
```

- When the **IMU drifts** (P grows large) → K increases → **GPS correction dominates**
- When **GPS is noisy** (R large) → K decreases → **IMU dead reckoning trusted more**
- The filter autonomously adjusts trust based on the ratio of uncertainties

This is the inverse covariance working as a *reliability weight* in real time.

---

## 7. Intuition Summary

### The Big Picture

```
╔══════════════════════════════════════════════════════════════╗
║                  WHAT IS A KALMAN FILTER?                   ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║   You have TWO sources of information:                      ║
║                                                              ║
║   1. A MODEL: "Physics says I should be here"               ║
║      → Encoded in A (or f), with uncertainty Q              ║
║                                                              ║
║   2. A SENSOR: "My instrument reads this"                   ║
║      → Encoded in H (or h), with uncertainty R              ║
║                                                              ║
║   The KF asks: "Given both, what is my BEST estimate?"      ║
║                                                              ║
║   Answer: Weight each source by its PRECISION (Σ⁻¹)        ║
║   The more precise source receives more weight.             ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
```

### One-Sentence Meanings

| Matrix | One-sentence meaning |
|--------|----------------------|
| **A / F** | "This is how I think the world moves" |
| **B** | "This is how my commands affect the world" |
| **Q** | "This is how wrong my motion model might be" |
| **H** | "This is what my sensor actually measures" |
| **R** | "This is how noisy my sensor is" |
| **P** | "This is how uncertain I am right now" |
| **K** | "This decides which source to believe more" |
| **Σ⁻¹** | "This is how confidently I weight information" |

### Memory Anchor

```
PREDICT:    A, B  →  where are we going?
            Q     →  how much do we doubt the model?

UPDATE:     H     →  what does the sensor see?
            R     →  how much do we doubt the sensor?
            P     →  how uncertain are we right now?
            K     →  the judge: model vs sensor
```

---

*Document prepared as an engineering tutorial for KF/EKF applications in embedded systems and drone sensor fusion.*

*References: Welch & Bishop (2006), "An Introduction to the Kalman Filter"; Maybeck (1979), "Stochastic Models, Estimation and Control"; PX4 EKF2 source documentation.*
