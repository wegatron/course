对于$\boldsymbol{R} \exp \left(\boldsymbol{p}^{\wedge}\right) \boldsymbol{R}^{\mathrm{T}}=\exp \left((\boldsymbol{R} \boldsymbol{p})^{\wedge}\right)$

令$\boldsymbol{p} = t \cdot \boldsymbol{q}$, $t \in \mathbb{R}$, 在$t=0$处对$t$求导,有
$$
\begin{aligned}
\left.\frac{d}{d t}\right|_{t=0}\left[\boldsymbol{R} \cdot \exp (t \cdot \boldsymbol{q}) \cdot\boldsymbol{R}^T\right] &= \left.\frac{d}{d t}\right|_{t=0} \exp (\left(\boldsymbol{R} \cdot \mathbf{t} \cdot \boldsymbol{q}\right)_{\times})\\
\left.\boldsymbol{R} \cdot \frac{d}{d t}\right|_{t=0}\left[\mathbf{I}+(t \cdot \boldsymbol{q})_{\times}+O\left(t^{2}\right)\right] \cdot\boldsymbol{R}^T &= \left.\frac{d}{d t}\right|_{t=0}\left[\mathbf{I}+\left(\boldsymbol{R} \cdot t \cdot \boldsymbol{q}\right)_{\times}+O\left(t^{2}\right)\right] \\
\boldsymbol{R} \cdot (t \cdot \boldsymbol{q})_{\times} \cdot \boldsymbol{R}^T &= \left(\boldsymbol{R} \cdot t \cdot \boldsymbol{q}\right)_{\times} \\
\boldsymbol{R} \cdot \boldsymbol{q}^{\wedge} \cdot \boldsymbol{R}^T &= (\boldsymbol{R} \cdot \boldsymbol{q})^{\wedge}
\end{aligned}
$$