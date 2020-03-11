## 2. 图像去畸变
见代码
## 3. 双目视差的使用
见代码
## 4. 矩阵运算微分
### ① 
$$
\mathrm{d}(\mathbf{Ax})/\mathrm{d}\mathbf{x} = \frac{\mathbf{A}\mathrm{d}\mathbf{x}}{\mathrm{d}\mathbf{x}} = \mathbf{A}
$$

### ②
证法一:
$$
\mathrm{d}(\mathbf{x^TAx})/\mathrm{d}\mathbf{x} = \frac{\mathrm{d}(\mathbf{x})^T\mathbf{Ax} + \mathbf{x}^T\mathbf{A}\mathrm{d}(\mathbf{x})}{\mathrm{d}\mathbf{x}} = (\mathbf{Ax})^T + \mathbf{x}^T\mathbf{A} = \mathbf{x}^T(\mathbf{A}^T + \mathbf{A})
$$

证法二:
$$
\begin{aligned}
&\mathrm{d}(\mathbf{x^TAx})/\mathrm{d}\mathbf{x} = \frac{\partial (\mathbf{x^TAx})}{\partial \mathbf{x}^T} = P_{1 \times N}\\
&P_k = \frac{\partial (\mathbf{x}_i \mathbf{A}_{ij} \mathbf{x}_j)}{\partial \mathbf{x}_k} = \mathbf{A}_{kj} \mathbf{x}_j + \mathbf{x}_i \mathbf{A}_{ik}\\
&\Rightarrow P = (\mathbf{Ax})^T + \mathbf{x}^T\mathbf{A}
\end{aligned}
$$

### ③
证法一:
对于左边

$$
\mathbf{x^TAx} = \mathbf{x}_i \mathbf{A}_{ij} \mathbf{x}_j
$$

对于右边

$$
\begin{aligned}
(\mathbf{xx^T})_{ij} &= \mathbf{x}_i \mathbf{x}_j\\
(\mathbf{A}\mathbf{xx^T})_{ik} &= \mathbf{A}_{ij} \mathbf{x}_j\mathbf{x}_k\\
\mathrm{trace} (\mathbf{A}\mathbf{xx^T}) &= \sum_{i} (\mathbf{A}\mathbf{xx^T})_{ii} = \mathbf{A}_{ij} \mathbf{x}_j\mathbf{x}_i
\end{aligned}
$$

故而相等.
注: 这里有些地方使用了张量的表示法, 省略了求和符号.

## 5. 高斯牛顿法的曲线拟合
见代码
## 6. 批量最大似然估计
①

$$
H = \begin{bmatrix} -1 & 1& 0 & 0\\
0 & -1 & 1 & 0\\
0 & 0 & -1 & 1\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

②

$$
W = \begin{bmatrix}
Q & 0 & 0 & 0 & 0 & 0\\
0 & Q & 0 & 0 & 0 & 0\\
0 & 0 & Q & 0 & 0 & 0\\
0 & 0 & 0 & R & 0 & 0\\
0 & 0 & 0 & 0 & R & 0\\
0 & 0 & 0 & 0 & 0 & R
\end{bmatrix}
$$

③ 有唯一解.
对$E = \frac{1}{2}(\mathbf{z}-\mathbf{H x})^{\mathrm{T}} \mathbf{W}^{-1}(\mathbf{z}-\mathbf{H x})$求偏导, 令其等于0:

$$
\begin{aligned}
-(Z - H\mathbf{x})^T W^{-1}H = 0\\
\Rightarrow (H^T W^{-1}H)^T\mathbf{x} = (Z W^{-1}H)^T\\
\Rightarrow \mathbf{x} = (H^T W^{-1}H)^{-T}(Z W^{-1}H)^T
\end{aligned}
$$