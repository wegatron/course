clear
imu_hes_raw = csvread("/home/wegatron/tmp/imu_hes.csv");
filled_imu_hes = abs(imu_hes_raw) > 1e-3;

proj_hes_raw = csvread("/home/wegatron/tmp/proj_hes.csv");
filled_proj_hes = abs(proj_hes_raw) > 1e-3;

marg_hes_raw = csvread("/home/wegatron/tmp/marg_hes.csv");
filled_marg_hes = abs(marg_hes_raw) > 1e-3;

sum_filled = filled_imu_hes + 2*filled_proj_hes + 4*filled_marg_hes;

% visualize
figure(1, 'position',[0,0,900,900]);
colormap jet;

subplot(2, 2,1);
imagesc(filled_imu_hes, climits=[0,1]);
title("filled imu")

subplot(2,2,2)
imagesc(filled_proj_hes, climits=[0,1]);
title("filled proj")

subplot(2,2,3)
imagesc(filled_marg_hes, climits=[0,1]);
title('filled marg');

subplot(2,2,4)
imagesc(sum_filled, climits=[0,7]);
title('sum');