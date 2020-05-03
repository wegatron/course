# VINS HW2
## FeatureManager 特征点信息打印
```
[ INFO] [1588057962.513280229]: fid	start_frame	used_num
[ INFO] [1588057962.513454559]: 18	    0	        11 
[ INFO] [1588057962.513505923]: 20	    0	        11 
[ INFO] [1588057962.513548119]: 37	    0	        11 
[ INFO] [1588057962.513585360]: 51	    0	        11 
[ INFO] [1588057962.513622740]: 75	    0	        10 
[ INFO] [1588057962.513650251]: 78	    0	        11 
[ INFO] [1588057962.513673677]: 119	    0	        11 
[ INFO] [1588057962.513693993]: 150	    0	        11 
[ INFO] [1588057962.513716585]: 175	    0	        11 
...
```

## 分析slideWindow
1. marg最老帧
    滑窗向前滚动, 所有还能追踪到的feature的start_frame - 1(原start_frame为0, 则不变).
    在marg最老帧之前:
    ```
    [ INFO] [1588058358.155395373]: fid	start_frame	used_num
    [ INFO] [1588058358.155736758]: 14	0	11 
    [ INFO] [1588058358.155882702]: 17	0	11 
    [ INFO] [1588058358.156086342]: 20	0	11 
    [ INFO] [1588058358.156224264]: 36	0	11 
    [ INFO] [1588058358.156430343]: 48	0	11 
    [ INFO] [1588058358.156566983]: 51	0	11 
    [ INFO] [1588058358.156755745]: 56	0	11 
    ...
    [ INFO] [1588058358.175896477]: 871	2	8 
    [ INFO] [1588058358.175963747]: 872	2	8 
    [ INFO] [1588058358.175997245]: 873	2	8 
    [ INFO] [1588058358.176029359]: 876	2	8 
    [ INFO] [1588058358.176059319]: 877	2	8 
    [ INFO] [1588058358.176160833]: 879	2	8 
    [ INFO] [1588058358.176189639]: 881	2	2 
    [ INFO] [1588058358.176534796]: 883	2	2 
    ...
    ```
    marg最老帧之后:
    ```
    [ INFO] [1588058358.242563390]: 14	0	11 
    [ INFO] [1588058358.243015126]: 17	0	11 
    [ INFO] [1588058358.243046917]: 20	0	11 
    [ INFO] [1588058358.243122117]: 36	0	11 
    [ INFO] [1588058358.243179911]: 48	0	11 
    [ INFO] [1588058358.243206606]: 51	0	11 
    [ INFO] [1588058358.243265632]: 56	0	11 
    ...
    [ INFO] [1588058358.247694214]: 871	1	9 
    [ INFO] [1588058358.247752301]: 872	1	9 
    [ INFO] [1588058358.247811393]: 873	1	9 
    [ INFO] [1588058358.247901183]: 876	1	9 
    [ INFO] [1588058358.247926512]: 877	1	9 
    [ INFO] [1588058358.248013461]: 879	1	9 
    [ INFO] [1588058358.248038626]: 881	1	2 
    [ INFO] [1588058358.248097249]: 883	1	2 
    ```
2. marg最新帧
    能追踪到的feature的start_frame不变, 跟踪情况也不变.

    marg最新帧之前
    ```
    [ INFO] [1588058359.304245161]: fid	start_frame	used_num
    [ INFO] [1588058359.304279708]: 14	0	11 
    [ INFO] [1588058359.304311143]: 17	0	11 
    [ INFO] [1588058359.304331907]: 36	0	11 
    [ INFO] [1588058359.304348767]: 51	0	11 
    [ INFO] [1588058359.304368176]: 56	0	7 
    ...
    [ INFO] [1588058359.306319906]: 1550	1	4 
    [ INFO] [1588058359.306338962]: 1551	1	3 
    [ INFO] [1588058359.306358249]: 1559	1	9 
    [ INFO] [1588058359.306375126]: 1567	1	3 
    [ INFO] [1588058359.306391441]: 1570	1	5 
    ...
    [ INFO] [1588058359.306510505]: 1585	2	1 
    [ INFO] [1588058359.306526824]: 1586	2	2 
    [ INFO] [1588058359.306542967]: 1587	2	2 
    [ INFO] [1588058359.306559938]: 1602	2	8 
    [ INFO] [1588058359.306577515]: 1603	2	1 
    [ INFO] [1588058359.306593791]: 1607	2	5 
    ```

    marg最新帧之后
    ```
    [ INFO] [1588058359.346307283]: 14	0	11 
    [ INFO] [1588058359.346323111]: 17	0	11 
    [ INFO] [1588058359.346341228]: 36	0	11 
    [ INFO] [1588058359.346358538]: 51	0	11 
    ...
    [ INFO] [1588058359.348124457]: 1550	1	4 
    [ INFO] [1588058359.348140202]: 1551	1	3 
    [ INFO] [1588058359.348158488]: 1559	1	10 
    [ INFO] [1588058359.348174511]: 1567	1	3 
    [ INFO] [1588058359.348192104]: 1570	1	5 
    ```

## 因子图
程序输出:
```
[ INFO] [1588078108.455935703]: --------------------frame-------------------
[ INFO] [1588078108.456372164]: marg_residual: imu_3_pose imu_4_pose imu_5_pose IMU_0bias_ba_bg imu_0_pose camera_imu_ex_pose imu_8_pose imu_7_pose imu_6_pose imu_2_pose imu_1_pose 
[ INFO] [1588078108.456483450]: imu_residual_0: imu_0_pose imu_0_bias_ba_bg imu_1_pose imu_1_bias_ba_bg
[ INFO] [1588078108.456571492]: imu_residual_1: imu_1_pose imu_1_bias_ba_bg imu_2_pose imu_2_bias_ba_bg
[ INFO] [1588078108.456597752]: imu_residual_2: imu_2_pose imu_2_bias_ba_bg imu_3_pose imu_3_bias_ba_bg
[ INFO] [1588078108.456673207]: imu_residual_3: imu_3_pose imu_3_bias_ba_bg imu_4_pose imu_4_bias_ba_bg
[ INFO] [1588078108.456721312]: imu_residual_4: imu_4_pose imu_4_bias_ba_bg imu_5_pose imu_5_bias_ba_bg
[ INFO] [1588078108.456785942]: imu_residual_5: imu_5_pose imu_5_bias_ba_bg imu_6_pose imu_6_bias_ba_bg
[ INFO] [1588078108.456813758]: imu_residual_6: imu_6_pose imu_6_bias_ba_bg imu_7_pose imu_7_bias_ba_bg
[ INFO] [1588078108.456828667]: imu_residual_7: imu_7_pose imu_7_bias_ba_bg imu_8_pose imu_8_bias_ba_bg
[ INFO] [1588078108.456859504]: imu_residual_8: imu_8_pose imu_8_bias_ba_bg imu_9_pose imu_9_bias_ba_bg
[ INFO] [1588078108.456885120]: imu_residual_9: imu_9_pose imu_9_bias_ba_bg imu_10_pose imu_10_bias_ba_bg
[ INFO] [1588078108.456901287]: imu_residual cnt=10
[ INFO] [1588078108.458022987]: visual_residual: imu_0-imu_1 feature_cnt=128
[ INFO] [1588078108.458048668]: visual_residual: imu_0-imu_10 feature_cnt=61
[ INFO] [1588078108.458066458]: visual_residual: imu_0-imu_2 feature_cnt=107
[ INFO] [1588078108.458086410]: visual_residual: imu_0-imu_3 feature_cnt=102
[ INFO] [1588078108.458103211]: visual_residual: imu_0-imu_4 feature_cnt=96
[ INFO] [1588078108.458119052]: visual_residual: imu_0-imu_5 feature_cnt=93
[ INFO] [1588078108.458134060]: visual_residual: imu_0-imu_6 feature_cnt=80
[ INFO] [1588078108.458148467]: visual_residual: imu_0-imu_7 feature_cnt=73
[ INFO] [1588078108.458163642]: visual_residual: imu_0-imu_8 feature_cnt=68
[ INFO] [1588078108.458178384]: visual_residual: imu_0-imu_9 feature_cnt=62
[ INFO] [1588078108.458192672]: visual_residual: imu_1-imu_10 feature_cnt=4
[ INFO] [1588078108.458210423]: visual_residual: imu_1-imu_2 feature_cnt=8
[ INFO] [1588078108.458225120]: visual_residual: imu_1-imu_3 feature_cnt=8
[ INFO] [1588078108.458241105]: visual_residual: imu_1-imu_4 feature_cnt=7
[ INFO] [1588078108.458255849]: visual_residual: imu_1-imu_5 feature_cnt=7
[ INFO] [1588078108.458270497]: visual_residual: imu_1-imu_6 feature_cnt=5
[ INFO] [1588078108.458284861]: visual_residual: imu_1-imu_7 feature_cnt=5
[ INFO] [1588078108.458303946]: visual_residual: imu_1-imu_8 feature_cnt=5
[ INFO] [1588078108.458320352]: visual_residual: imu_1-imu_9 feature_cnt=5
[ INFO] [1588078108.458344029]: visual_residual: imu_2-imu_10 feature_cnt=4
[ INFO] [1588078108.458359532]: visual_residual: imu_2-imu_3 feature_cnt=10
[ INFO] [1588078108.458387792]: visual_residual: imu_2-imu_4 feature_cnt=9
[ INFO] [1588078108.458405238]: visual_residual: imu_2-imu_5 feature_cnt=8
[ INFO] [1588078108.458422877]: visual_residual: imu_2-imu_6 feature_cnt=7
[ INFO] [1588078108.458448565]: visual_residual: imu_2-imu_7 feature_cnt=7
[ INFO] [1588078108.458463992]: visual_residual: imu_2-imu_8 feature_cnt=4
[ INFO] [1588078108.458482184]: visual_residual: imu_2-imu_9 feature_cnt=4
[ INFO] [1588078108.458499862]: visual_residual: imu_3-imu_10 feature_cnt=5
[ INFO] [1588078108.458518757]: visual_residual: imu_3-imu_4 feature_cnt=15
[ INFO] [1588078108.458537372]: visual_residual: imu_3-imu_5 feature_cnt=12
[ INFO] [1588078108.458555900]: visual_residual: imu_3-imu_6 feature_cnt=12
[ INFO] [1588078108.458571891]: visual_residual: imu_3-imu_7 feature_cnt=12
[ INFO] [1588078108.458590969]: visual_residual: imu_3-imu_8 feature_cnt=8
[ INFO] [1588078108.458608002]: visual_residual: imu_3-imu_9 feature_cnt=7
[ INFO] [1588078108.458626262]: visual_residual: imu_4-imu_10 feature_cnt=4
[ INFO] [1588078108.458642586]: visual_residual: imu_4-imu_5 feature_cnt=8
[ INFO] [1588078108.458660418]: visual_residual: imu_4-imu_6 feature_cnt=5
[ INFO] [1588078108.458677382]: visual_residual: imu_4-imu_7 feature_cnt=4
[ INFO] [1588078108.458695341]: visual_residual: imu_4-imu_8 feature_cnt=4
[ INFO] [1588078108.458711652]: visual_residual: imu_4-imu_9 feature_cnt=4
[ INFO] [1588078108.458732002]: visual_residual: imu_5-imu_10 feature_cnt=2
[ INFO] [1588078108.458750607]: visual_residual: imu_5-imu_6 feature_cnt=6
[ INFO] [1588078108.458767547]: visual_residual: imu_5-imu_7 feature_cnt=4
[ INFO] [1588078108.458785702]: visual_residual: imu_5-imu_8 feature_cnt=4
[ INFO] [1588078108.458802059]: visual_residual: imu_5-imu_9 feature_cnt=2
[ INFO] [1588078108.458820383]: visual_residual: imu_6-imu_10 feature_cnt=4
[ INFO] [1588078108.458837281]: visual_residual: imu_6-imu_7 feature_cnt=7
[ INFO] [1588078108.458864479]: visual_residual: imu_6-imu_8 feature_cnt=5
[ INFO] [1588078108.458880141]: visual_residual: imu_6-imu_9 feature_cnt=4
[ INFO] [1588078108.458897640]: visual_residual: imu_7-imu_10 feature_cnt=10
[ INFO] [1588078108.458915143]: visual_residual: imu_7-imu_8 feature_cnt=15
[ INFO] [1588078108.458931909]: visual_residual: imu_7-imu_9 feature_cnt=10
[ INFO] [1588078108.458951027]: visual measurement count: 1150
```

因子图:
![factor.jpg](factor.jpg)

这里Marg因子与IMU_Pose都相关表示被marg掉的最老帧与滑窗中的所有帧都有共视关系. 只与IMU_0的bias相关, 因为被marg掉的帧只与滑窗中的第0帧有IMU残差约束.

## MarginalizationInfo中的变量统计
| id | addr(long) | global_size | addr(double*) | idx(local_size) | 变量 |
| :--: | :----------: | :-----------: | :-------------: | :---------------: | :----: |
| 0 | 93837826993104 | 1 | 0x7f3e0c17b920 | 0 | Feature_125
| 1 | 93837826993096 | 1 | 0x7f3e0c17a380 | 1 | Feature_124
| 2 | 93837826993088 | 1 | 0x7f3e0c178b80 | 2 | Feature_123
| 3 | 93837826993080 | 1 | 0x7f3e0c178420 | 3 | Feature_122
| 4 | 93837826993072 | 1 | 0x7f3e0c176c20 | 4 | Feature_121
| 5 | 93837826993064 | 1 | 0x7f3e0c176720 | 5 | Feature_120
| 6 | 93837826993056 | 1 | 0x7f3e0c454400 | 6 | Feature_119
| 7 | 93837826993048 | 1 | 0x7f3e0c452c00 | 7 | Feature_118
| 8 | 93837826993040 | 1 | 0x7f3e0c452700 | 8 | Feature_117
| 9 | 93837826993032 | 1 | 0x7f3e0c450f00 | 9 | Feature_116
| 10 | 93837826993024 | 1 | 0x7f3e0c44f700 | 10 | Feature_115
| 11 | 93837826993016 | 1 | 0x7f3e0c44df00 | 11 | Feature_114
| 12 | 93837826993008 | 1 | 0x7f3e0c44c700 | 12 | Feature_113
| 13 | 93837826993000 | 1 | 0x7f3e0c552490 | 13 | Feature_112
| 14 | 93837826992992 | 1 | 0x7f3e0c550c90 | 14 | Feature_111
| 15 | 93837826992984 | 1 | 0x7f3e0c54f490 | 15 | Feature_110
| 16 | 93837826992976 | 1 | 0x7f3e0c54dc90 | 16 | Feature_109
| 17 | 93837826992968 | 1 | 0x7f3e0c54c490 | 17 | Feature_108
| 18 | 93837826992960 | 1 | 0x7f3e0c54ac90 | 18 | Feature_107
| 19 | 93837826992952 | 1 | 0x7f3e0c7189e0 | 19 | Feature_106
| 20 | 93837826992944 | 1 | 0x7f3e0c7171e0 | 20 | Feature_105
| 21 | 93837826992936 | 1 | 0x7f3e0c7159e0 | 21 | Feature_104
| 22 | 93837826992928 | 1 | 0x7f3e0c715280 | 22 | Feature_103
| 23 | 93837826992920 | 1 | 0x7f3e0c713a80 | 23 | Feature_102
| 24 | 93837826992912 | 1 | 0x7f3e0c712280 | 24 | Feature_101
| 25 | 93837826992904 | 1 | 0x7f3e0c710a80 | 25 | Feature_100
| 26 | 93837826992896 | 1 | 0x7f3e0c4715b0 | 26 | Feature_99
| 27 | 93837826992888 | 1 | 0x7f3e0c46fdb0 | 27 | Feature_98
| 28 | 93837826992880 | 1 | 0x7f3e0c46e5b0 | 28 | Feature_97
| 29 | 93837826992872 | 1 | 0x7f3e0c46cdb0 | 29 | Feature_96
| 30 | 93837826992864 | 1 | 0x7f3e0c46b5b0 | 30 | Feature_95
| 31 | 93837826992856 | 1 | 0x7f3e0c410440 | 31 | Feature_94
| 32 | 93837826992848 | 1 | 0x7f3e0c40ec40 | 32 | Feature_93
| 33 | 93837826992840 | 1 | 0x7f3e0c40d440 | 33 | Feature_92
| 34 | 93837826992832 | 1 | 0x7f3e0c40cf40 | 34 | Feature_91
| 35 | 93837826992824 | 1 | 0x7f3e0c40b740 | 35 | Feature_90
| 36 | 93837826992816 | 1 | 0x7f3e0c40ad80 | 36 | Feature_89
| 37 | 93837826992808 | 1 | 0x7f3e0c409580 | 37 | Feature_88
| 38 | 93837826992800 | 1 | 0x7f3e0c0db9b0 | 38 | Feature_87
| 39 | 93837826992792 | 1 | 0x7f3e0c0db4b0 | 39 | Feature_86
| 40 | 93837826992784 | 1 | 0x7f3e0c0da890 | 40 | Feature_85
| 41 | 93837826992776 | 1 | 0x7f3e0c0d9090 | 41 | Feature_84
| 42 | 93837826992768 | 1 | 0x7f3e0c0d8470 | 42 | Feature_83
| 43 | 93837826992760 | 1 | 0x7f3e0c0d6c70 | 43 | Feature_82
| 44 | 93837826992752 | 1 | 0x7f3e0c0d5470 | 44 | Feature_81
| 45 | 93837826992744 | 1 | 0x7f3e0c0d4ab0 | 45 | Feature_80
| 46 | 93837826992736 | 1 | 0x7f3e0c20a910 | 46 | Feature_79
| 47 | 93837826992728 | 1 | 0x7f3e0c209f50 | 47 | Feature_78
| 48 | 93837826992720 | 1 | 0x7f3e0c208750 | 48 | Feature_77
| 49 | 93837826992712 | 1 | 0x7f3e0c206f50 | 49 | Feature_76
| 50 | 93837826992368 | 1 | 0x7f3e0c140930 | 50 | Feature_33
| 51 | 93837826992360 | 1 | 0x7f3e0c1401d0 | 51 | Feature_32
| 52 | 93837826992352 | 1 | 0x7f3e0c7302a0 | 52 | Feature_31
| 53 | 93837826992344 | 1 | 0x7f3e0c72f8e0 | 53 | Feature_30
| 54 | 93837826992336 | 1 | 0x7f3e0c72e0e0 | 54 | Feature_29
| 55 | 93837826992328 | 1 | 0x7f3e0c72c8e0 | 55 | Feature_28
| 56 | 93837826992320 | 1 | 0x7f3e0c72b0e0 | 56 | Feature_27
| 57 | 93837826992312 | 1 | 0x7f3e0c7298e0 | 57 | Feature_26
| 58 | 93837826992304 | 1 | 0x7f3e0c7293e0 | 58 | Feature_25
| 59 | 93837826992296 | 1 | 0x7f3e0c156d20 | 59 | Feature_24
| 60 | 93837826992288 | 1 | 0x7f3e0c155520 | 60 | Feature_23
| 61 | 93837826992280 | 1 | 0x7f3e0c153d20 | 61 | Feature_22
| 62 | 93837826992272 | 1 | 0x7f3e0c152520 | 62 | Feature_21
| 63 | 93837826992264 | 1 | 0x7f3e0c150d20 | 63 | Feature_20
| 64 | 93837826992256 | 1 | 0x7f3e0c0ea350 | 64 | Feature_19
| 65 | 93837826992248 | 1 | 0x7f3e0c0e8db0 | 65 | Feature_18
| 66 | 93837826992240 | 1 | 0x7f3e0c0e7cd0 | 66 | Feature_17
| 67 | 93837826992232 | 1 | 0x7f3e0c0e64d0 | 67 | Feature_16
| 68 | 93837826992224 | 1 | 0x7f3e0c0e4cd0 | 68 | Feature_15
| 69 | 93837826992216 | 1 | 0x7f3e0c0e3950 | 69 | Feature_14
| 70 | 93837826992128 | 1 | 0x7f3e0c4a3e60 | 70 | Feature_3
| 71 | 93837826992120 | 1 | 0x7f3e0c230970 | 71 | Feature_2
| 72 | 93837826992112 | 1 | 0x7f3e0c2301d0 | 72 | Feature_1
| 73 | 93837826992104 | 1 | 0x7f3e0c4d85f0 | 73 | Feature_0
| 74 | 93837826992192 | 1 | 0x7f3e0c542c30 | 74 | Feature_11
| 75 | 93837826992136 | 1 | 0x7f3e0c4a4320 | 75 | Feature_4
| 76 | 93837826992144 | 1 | 0x7f3e0c4a5440 | 76 | Feature_5
| 77 | 93837826992152 | 1 | 0x7f3e0c4a6520 | 77 | Feature_6
| 78 | 93837826992160 | 1 | 0x7f3e0c4a69e0 | 78 | Feature_7
| 79 | 93837826992168 | 1 | 0x7f3e0c53f730 | 79 | Feature_8
| 80 | 93837826990696 | 7 | 0x7f3e0c1ac400 | 80 | imu_0_pose
| 81 | 93837826992592 | 1 | 0x7f3e0c31fc10 | 86 | Feature_61
| 82 | 93837826992176 | 1 | 0x7f3e0c540a70 | 87 | Feature_9
| 83 | 93837826992184 | 1 | 0x7f3e0c541b50 | 88 | Feature_10
| 84 | 93837826992576 | 1 | 0x7f3e0c0b1030 | 89 | Feature_59
| 85 | 93837826992200 | 1 | 0x7f3e0c5441d0 | 90 | Feature_12
| 86 | 93837826992208 | 1 | 0x7f3e0c544df0 | 91 | Feature_13
| 87 | 93837826992376 | 1 | 0x7f3e0c142130 | 92 | Feature_34
| 88 | 93837826992384 | 1 | 0x7f3e0c143930 | 93 | Feature_35
| 89 | 93837826992392 | 1 | 0x7f3e0c145130 | 94 | Feature_36
| 90 | 93837826992400 | 1 | 0x7f3e0c1cd810 | 95 | Feature_37
| 91 | 93837826992408 | 1 | 0x7f3e0c1cf010 | 96 | Feature_38
| 92 | 93837826992416 | 1 | 0x7f3e0c1d0810 | 97 | Feature_39
| 93 | 93837826992424 | 1 | 0x7f3e0c1d2010 | 98 | Feature_40
| 94 | 93837826992432 | 1 | 0x7f3e0c1d2770 | 99 | Feature_41
| 95 | 93837826992440 | 1 | 0x7f3e0c1d35f0 | 100 | Feature_42
| 96 | 93837826992448 | 1 | 0x7f3e0c1d4df0 | 101 | Feature_43
| 97 | 93837826992456 | 1 | 0x7f3e0c3dbf40 | 102 | Feature_44
| 98 | 93837826992464 | 1 | 0x7f3e0c3dd740 | 103 | Feature_45
| 99 | 93837826992472 | 1 | 0x7f3e0c3def40 | 104 | Feature_46
| 100 | 93837826992480 | 1 | 0x7f3e0c3e0740 | 105 | Feature_47
| 101 | 93837826992488 | 1 | 0x7f3e0c3e15c0 | 106 | Feature_48
| 102 | 93837826992496 | 1 | 0x7f3e0c25a030 | 107 | Feature_49
| 103 | 93837826992504 | 1 | 0x7f3e0c25b7c0 | 108 | Feature_50
| 104 | 93837826992512 | 1 | 0x7f3e0c25cfc0 | 109 | Feature_51
| 105 | 93837826992520 | 1 | 0x7f3e0c25e7c0 | 110 | Feature_52
| 106 | 93837826992528 | 1 | 0x7f3e0c25ffc0 | 111 | Feature_53
| 107 | 93837826992536 | 1 | 0x7f3e0c2610a0 | 112 | Feature_54
| 108 | 93837826992544 | 1 | 0x7f3e0c0ab030 | 113 | Feature_55
| 109 | 93837826992552 | 1 | 0x7f3e0c0ac830 | 114 | Feature_56
| 110 | 93837826992560 | 1 | 0x7f3e0c0ae030 | 115 | Feature_57
| 111 | 93837826992568 | 1 | 0x7f3e0c0af830 | 116 | Feature_58
| 112 | 93837826992584 | 1 | 0x7f3e0c0b2830 | 117 | Feature_60
| 113 | 93837826992600 | 1 | 0x7f3e0c320370 | 118 | Feature_62
| 114 | 93837826992608 | 1 | 0x7f3e0c321b70 | 119 | Feature_63
| 115 | 93837826992616 | 1 | 0x7f3e0c323370 | 120 | Feature_64
| 116 | 93837826992624 | 1 | 0x7f3e0c325080 | 121 | Feature_65
| 117 | 93837826992632 | 1 | 0x7f3e0c475220 | 122 | Feature_66
| 118 | 93837826992640 | 1 | 0x7f3e0c476a20 | 123 | Feature_67
| 119 | 93837826991312 | 9 | 0x7f3e0c449d80 | 124 | IMU_0bias_ba_bg
| 120 | 93837826992648 | 1 | 0x7f3e0c477d60 | 133 | Feature_68
| 121 | 93837826992656 | 1 | 0x7f3e0c479560 | 134 | Feature_69
| 122 | 93837826992664 | 1 | 0x7f3e0c47ad60 | 135 | Feature_70
| 123 | 93837826992672 | 1 | 0x7f3e0c47b220 | 136 | Feature_71
| 124 | 93837826992680 | 1 | 0x7f3e0c47c800 | 137 | Feature_72
| 125 | 93837826992688 | 1 | 0x7f3e0c47ccc0 | 138 | Feature_73
| 126 | 93837826992696 | 1 | 0x7f3e0c203f50 | 139 | Feature_74
| 127 | 93837826992704 | 1 | 0x7f3e0c205750 | 140 | Feature_75
| 128 | 93837826990920 | 7 | 0x7f3e0c585c80 | 141 | imu_4_pose
| 129 | 93837826990864 | 7 | 0x7f3e0c585ce0 | 147 | imu_3_pose
| 130 | 93837826991256 | 7 | 0x7f3e0c1ac280 | 153 | imu_10_pose
| 131 | 93837827000104 | 7 | 0x7f3e0c1ac580 | 159 | camera_imu_ex_pose
| 132 | 93837826990752 | 7 | 0x7f3e0c585d40 | 165 | imu_1_pose
| 133 | 93837826991384 | 9 | 0x7f3e0c449c30 | 171 | IMU_1bias_ba_bg
| 134 | 93837826990808 | 7 | 0x7f3e0c585dc0 | 180 | imu_2_pose
| 135 | 93837826990976 | 7 | 0x7f3e0c1ad660 | 186 | imu_5_pose
| 136 | 93837826991032 | 7 | 0x7f3e0c585e20 | 192 | imu_6_pose
| 137 | 93837826991088 | 7 | 0x7f3e0c585e80 | 198 | imu_7_pose
| 138 | 93837826991144 | 7 | 0x7f3e0c585ee0 | 204 | imu_8_pose
| 139 | 93837826991200 | 7 | 0x7f3e0c1ac680 | 210 | imu_9_pose

[ INFO] [1588122014.006334338]: marginalization, pos: 216, m: 141, n: 75, size: 140

$\mathrm{pos} = 2(M_0, M_1) \times 9 + 11(P) \times 6 + 6(T_{bc}) + 126(\lambda) = 216$
$\mathrm{m} = 126(\lambda) + 6(\mathrm{imu\_pose\_0}) + 9(M_0)= 141$, 所有Feature和imu_0的位姿+bias被marg掉
$\mathrm{n} = \mathrm{pos -m} = 75$

## Marg最老帧时Hessian矩阵示意图
Hessian示意图:
![hessian](hessian.png)

参数参照表:
| id | addr(long) | global_size | addr(double*) | idx(local_size) | 变量 |
| :--: | :----------: | :-----------: | :-------------: | :---------------: | :----: |
| 0 | 94252237372280 | 1 | 0x7fd95c05d740 | 0 | Feature_106
| 1 | 94252237372272 | 1 | 0x7fd95c05d460 | 1 | Feature_105
| 2 | 94252237372264 | 1 | 0x7fd95c05d1c0 | 2 | Feature_104
| 3 | 94252237372256 | 1 | 0x7fd95c05cd00 | 3 | Feature_103
| 4 | 94252237372248 | 1 | 0x7fd95c05ca20 | 4 | Feature_102
| 5 | 94252237372240 | 1 | 0x7fd95c05c560 | 5 | Feature_101
| 6 | 94252237372232 | 1 | 0x7fd95c2292b0 | 6 | Feature_100
| 7 | 94252237372224 | 1 | 0x7fd95c227cd0 | 7 | Feature_99
| 8 | 94252237372216 | 1 | 0x7fd95c227a30 | 8 | Feature_98
| 9 | 94252237372208 | 1 | 0x7fd95c227790 | 9 | Feature_97
| 10 | 94252237372200 | 1 | 0x7fd95c2272d0 | 10 | Feature_96
| 11 | 94252237372192 | 1 | 0x7fd95c2268d0 | 11 | Feature_95
| 12 | 94252237372184 | 1 | 0x7fd95c226630 | 12 | Feature_94
| 13 | 94252237372176 | 1 | 0x7fd95c226390 | 13 | Feature_93
| 14 | 94252237372168 | 1 | 0x7fd95c225ed0 | 14 | Feature_92
| 15 | 94252237372160 | 1 | 0x7fd95c2eeac0 | 15 | Feature_91
| 16 | 94252237372152 | 1 | 0x7fd95c2ee100 | 16 | Feature_90
| 17 | 94252237372144 | 1 | 0x7fd95c2ed740 | 17 | Feature_89
| 18 | 94252237372136 | 1 | 0x7fd95c2ebf40 | 18 | Feature_88
| 19 | 94252237372128 | 1 | 0x7fd95c116f50 | 19 | Feature_87
| 20 | 94252237372120 | 1 | 0x7fd95c115750 | 20 | Feature_86
| 21 | 94252237372112 | 1 | 0x7fd95c114fb0 | 21 | Feature_85
| 22 | 94252237372104 | 1 | 0x7fd95c114af0 | 22 | Feature_84
| 23 | 94252237372096 | 1 | 0x7fd95c114810 | 23 | Feature_83
| 24 | 94252237372088 | 1 | 0x7fd95c114350 | 24 | Feature_82
| 25 | 94252237372080 | 1 | 0x7fd95c113e50 | 25 | Feature_81
| 26 | 94252237372072 | 1 | 0x7fd95c2a0300 | 26 | Feature_80
| 27 | 94252237372064 | 1 | 0x7fd95c29fba0 | 27 | Feature_79
| 28 | 94252237372056 | 1 | 0x7fd95c29f6a0 | 28 | Feature_78
| 29 | 94252237372048 | 1 | 0x7fd95c29ef40 | 29 | Feature_77
| 30 | 94252237372040 | 1 | 0x7fd95c29d740 | 30 | Feature_76
| 31 | 94252237371696 | 1 | 0x7fd95c368880 | 31 | Feature_33
| 32 | 94252237371688 | 1 | 0x7fd95c0d87f0 | 32 | Feature_32
| 33 | 94252237371680 | 1 | 0x7fd95c0d7e30 | 33 | Feature_31
| 34 | 94252237371672 | 1 | 0x7fd95c0d6630 | 34 | Feature_30
| 35 | 94252237371664 | 1 | 0x7fd95c27cbb0 | 35 | Feature_29
| 36 | 94252237371656 | 1 | 0x7fd95c27c450 | 36 | Feature_28
| 37 | 94252237371648 | 1 | 0x7fd95c27b830 | 37 | Feature_27
| 38 | 94252237371640 | 1 | 0x7fd95c27a030 | 38 | Feature_26
| 39 | 94252237371632 | 1 | 0x7fd95c317190 | 39 | Feature_25
| 40 | 94252237371624 | 1 | 0x7fd95c315990 | 40 | Feature_24
| 41 | 94252237371616 | 1 | 0x7fd95c2f60b0 | 41 | Feature_23
| 42 | 94252237371608 | 1 | 0x7fd95c2f48b0 | 42 | Feature_22
| 43 | 94252237371600 | 1 | 0x7fd95c2f43b0 | 43 | Feature_21
| 44 | 94252237371592 | 1 | 0x7fd95c289ef0 | 44 | Feature_20
| 45 | 94252237371584 | 1 | 0x7fd95c289070 | 45 | Feature_19
| 46 | 94252237371576 | 1 | 0x7fd95c287870 | 46 | Feature_18
| 47 | 94252237371568 | 1 | 0x7fd95c098220 | 47 | Feature_17
| 48 | 94252237371560 | 1 | 0x7fd95c0973a0 | 48 | Feature_16
| 49 | 94252237371552 | 1 | 0x7fd95c095ba0 | 49 | Feature_15
| 50 | 94252237371544 | 1 | 0x7fd95c2e66e0 | 50 | Feature_14
| 51 | 94252237371456 | 1 | 0x7fd95c13c300 | 51 | Feature_3
| 52 | 94252237371448 | 1 | 0x7fd95c13ab00 | 52 | Feature_2
| 53 | 94252237371440 | 1 | 0x7fd95c311bd0 | 53 | Feature_1
| 54 | 94252237371432 | 1 | 0x7fd95c0635a0 | 54 | Feature_0
| 55 | 94252237371520 | 1 | 0x7fd95c1d6a80 | 55 | Feature_11
| 56 | 94252237371464 | 1 | 0x7fd95c1198c0 | 56 | Feature_4
| 57 | 94252237371472 | 1 | 0x7fd95c11b0c0 | 57 | Feature_5
| 58 | 94252237371480 | 1 | 0x7fd95c201660 | 58 | Feature_6
| 59 | 94252237371488 | 1 | 0x7fd95c202e60 | 59 | Feature_7
| 60 | 94252237371496 | 1 | 0x7fd95c134120 | 60 | Feature_8
| 61 | 94252237370024 | 7 | 0x7fd95c329a40 | 61 | imu_0_pose
| 62 | 94252237371920 | 1 | 0x7fd95c1c2280 | 67 | Feature_61
| 63 | 94252237371504 | 1 | 0x7fd95c135920 | 68 | Feature_9
| 64 | 94252237371512 | 1 | 0x7fd95c137120 | 69 | Feature_10
| 65 | 94252237371904 | 1 | 0x7fd95c129140 | 70 | Feature_59
| 66 | 94252237371528 | 1 | 0x7fd95c1d8280 | 71 | Feature_12
| 67 | 94252237371536 | 1 | 0x7fd95c2e4ee0 | 72 | Feature_13
| 68 | 94252237371704 | 1 | 0x7fd95c368fe0 | 73 | Feature_34
| 69 | 94252237371712 | 1 | 0x7fd95c36a7e0 | 74 | Feature_35
| 70 | 94252237371720 | 1 | 0x7fd95c158030 | 75 | Feature_36
| 71 | 94252237371728 | 1 | 0x7fd95c159830 | 76 | Feature_37
| 72 | 94252237371736 | 1 | 0x7fd95c37adc0 | 77 | Feature_38
| 73 | 94252237371744 | 1 | 0x7fd95c37c5c0 | 78 | Feature_39
| 74 | 94252237371752 | 1 | 0x7fd95c37ddc0 | 79 | Feature_40
| 75 | 94252237371760 | 1 | 0x7fd95c1ea930 | 80 | Feature_41
| 76 | 94252237371768 | 1 | 0x7fd95c1ec130 | 81 | Feature_42
| 77 | 94252237371776 | 1 | 0x7fd95c108110 | 82 | Feature_43
| 78 | 94252237371784 | 1 | 0x7fd95c109910 | 83 | Feature_44
| 79 | 94252237371792 | 1 | 0x7fd95c26afd0 | 84 | Feature_45
| 80 | 94252237371800 | 1 | 0x7fd95c26c7d0 | 85 | Feature_46
| 81 | 94252237371808 | 1 | 0x7fd95c26dfd0 | 86 | Feature_47
| 82 | 94252237371816 | 1 | 0x7fd95c26e4d0 | 87 | Feature_48
| 83 | 94252237371824 | 1 | 0x7fd95c23d510 | 88 | Feature_49
| 84 | 94252237371832 | 1 | 0x7fd95c23ed10 | 89 | Feature_50
| 85 | 94252237371840 | 1 | 0x7fd95c240510 | 90 | Feature_51
| 86 | 94252237371848 | 1 | 0x7fd95c081060 | 91 | Feature_52
| 87 | 94252237371856 | 1 | 0x7fd95c082860 | 92 | Feature_53
| 88 | 94252237371864 | 1 | 0x7fd95c11cc70 | 93 | Feature_54
| 89 | 94252237371872 | 1 | 0x7fd95c11e470 | 94 | Feature_55
| 90 | 94252237371880 | 1 | 0x7fd95c11fc70 | 95 | Feature_56
| 91 | 94252237371888 | 1 | 0x7fd95c126d20 | 96 | Feature_57
| 92 | 94252237371896 | 1 | 0x7fd95c128520 | 97 | Feature_58
| 93 | 94252237371912 | 1 | 0x7fd95c1c0a80 | 98 | Feature_60
| 94 | 94252237371928 | 1 | 0x7fd95c1c2ea0 | 99 | Feature_62
| 95 | 94252237371936 | 1 | 0x7fd95c09fdb0 | 100 | Feature_63
| 96 | 94252237371944 | 1 | 0x7fd95c0a15b0 | 101 | Feature_64
| 97 | 94252237371952 | 1 | 0x7fd95c103d30 | 102 | Feature_65
| 98 | 94252237371960 | 1 | 0x7fd95c104e10 | 103 | Feature_66
| 99 | 94252237371968 | 1 | 0x7fd95c105310 | 104 | Feature_67
| 100 | 94252237370640 | 9 | 0x7fd95c1dbe50 | 105 | IMU_0bias_ba_bg
| 101 | 94252237371976 | 1 | 0x7fd95c105810 | 114 | Feature_68
| 102 | 94252237371984 | 1 | 0x7fd95c1068f0 | 115 | Feature_69
| 103 | 94252237371992 | 1 | 0x7fd95c33eaf0 | 116 | Feature_70
| 104 | 94252237372000 | 1 | 0x7fd95c33fbd0 | 117 | Feature_71
| 105 | 94252237372008 | 1 | 0x7fd95c3413d0 | 118 | Feature_72
| 106 | 94252237372016 | 1 | 0x7fd95c29caa0 | 119 | Feature_73
| 107 | 94252237372024 | 1 | 0x7fd95c29cf60 | 120 | Feature_74
| 108 | 94252237372032 | 1 | 0x7fd95c29d240 | 121 | Feature_75
| 109 | 94252237370248 | 7 | 0x7fd95c208f40 | 122 | imu_4_pose
| 110 | 94252237370192 | 7 | 0x7fd95c208e00 | 128 | imu_3_pose
| 111 | 94252237370712 | 9 | 0x7fd95c1dbd00 | 134 | IMU_1bias_ba_bg
| 112 | 94252237370080 | 7 | 0x7fd95c1beab0 | 143 | imu_1_pose
| 113 | 94252237379432 | 7 | 0x7fd95c380a70 | 149 | camera_imu_ex_pose
| 114 | 94252237370584 | 7 | 0x7fd95c3118d0 | 155 | imu_10_pose
| 115 | 94252237370136 | 7 | 0x7fd95c298510 | 161 | imu_2_pose
| 116 | 94252237370304 | 7 | 0x7fd95c209080 | 167 | imu_5_pose
| 117 | 94252237370360 | 7 | 0x7fd95c139e00 | 173 | imu_6_pose
| 118 | 94252237370416 | 7 | 0x7fd95c20a100 | 179 | imu_7_pose
| 119 | 94252237370472 | 7 | 0x7fd95c1f28c0 | 185 | imu_8_pose
| 120 | 94252237370528 | 7 | 0x7fd95c208cc0 | 191 | imu_9_pose