# Full Bundle Adjustment
## frame work
* 整个框架搭建
    * problem build_up
        * edge definition
            * camera projection [done]
            * partial differential [done]
                (这里使用了ceres的自动求导, 文档上显示会比优化过的解析求导慢40%)
        * build_up [done]
        * add noise & solve
        * verify

## todo
- [ ] g2o 不同的linear solver有什么不同
```c++
    if(params.linear_solver == "dense_schur" ){
        linearSolver = new g2o::LinearSolverDense<BalBlockSolver::PoseMatrixType>();
    }
    else if(params.linear_solver == "sparse_schur"){
        linearSolver = new g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>();
        dynamic_cast<g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);  // AMD ordering , only needed for sparse cholesky solver
    }
```
