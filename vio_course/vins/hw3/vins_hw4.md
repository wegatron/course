# VINS HW3
## 初始化
打印初始化中的参考帧序号, 并统计SfM中参考帧跟其他帧(滑窗中除参考帧外的所有帧)三角化出来的点数.
在函数`GlobalSFM::construct`打印这些信息即可
```bash
!!!!!!!!!!########### reference frame:0 triangulated points with reference frame:61
!!!!!!!!!!########### reference frame:3 triangulated points with reference frame:93
!!!!!!!!!!########### reference frame:0 triangulated points with reference frame:113
[ INFO] [1589190660.642448169]: Not enough features or parallax; Move device around
!!!!!!!!!!########### reference frame:4 triangulated points with reference frame:69
!!!!!!!!!!########### reference frame:0 triangulated points with reference frame:78
!!!!!!!!!!########### reference frame:0 triangulated points with reference frame:63
```

## 闭环检测和优化
在`PoseGraph::optimize4DoF()`中打印输出, 闭环因子图相关信息:
```bash
[ INFO] [1589206421.386109327]: loop optimize!!!
[ INFO] [1589206421.386166402]: fixed pose: 0
[ INFO] [1589206421.386199431]: sequence residual 1 with 0
[ INFO] [1589206421.386222006]: sequence residual 2 with 1
[ INFO] [1589206421.386242949]: sequence residual 2 with 0
[ INFO] [1589206421.386267035]: sequence residual 3 with 2
[ INFO] [1589206421.386290462]: sequence residual 3 with 1
[ INFO] [1589206421.386310109]: sequence residual 3 with 0
[ INFO] [1589206421.386331728]: sequence residual 4 with 3
[ INFO] [1589206421.386353466]: sequence residual 4 with 2
[ INFO] [1589206421.386372858]: sequence residual 4 with 1
[ INFO] [1589206421.386393131]: sequence residual 4 with 0
[ INFO] [1589206421.386414496]: sequence residual 5 with 4
[ INFO] [1589206421.386434930]: sequence residual 5 with 3
[ INFO] [1589206421.386453016]: sequence residual 5 with 2
[ INFO] [1589206421.386474527]: sequence residual 5 with 1
[ INFO] [1589206421.386493634]: sequence residual 6 with 5
[ INFO] [1589206421.386513158]: sequence residual 6 with 4
[ INFO] [1589206421.386532088]: sequence residual 6 with 3
[ INFO] [1589206421.386553072]: sequence residual 6 with 2
[ INFO] [1589206421.386575697]: sequence residual 7 with 6
[ INFO] [1589206421.386594245]: sequence residual 7 with 5
[ INFO] [1589206421.386611112]: sequence residual 7 with 4
[ INFO] [1589206421.386630882]: sequence residual 7 with 3
[ INFO] [1589206421.386652280]: sequence residual 8 with 7
[ INFO] [1589206421.386673408]: sequence residual 8 with 6
[ INFO] [1589206421.386689097]: sequence residual 8 with 5
[ INFO] [1589206421.386707853]: sequence residual 8 with 4
[ INFO] [1589206421.386726561]: sequence residual 9 with 8
[ INFO] [1589206421.386748245]: sequence residual 9 with 7
[ INFO] [1589206421.386769444]: sequence residual 9 with 6
[ INFO] [1589206421.386787541]: sequence residual 9 with 5
[ INFO] [1589206421.386807677]: sequence residual 10 with 9
[ INFO] [1589206421.386828121]: sequence residual 10 with 8
[ INFO] [1589206421.386851123]: sequence residual 10 with 7
[ INFO] [1589206421.386869307]: sequence residual 10 with 6
[ INFO] [1589206421.386891915]: sequence residual 11 with 10
[ INFO] [1589206421.386909980]: sequence residual 11 with 9
[ INFO] [1589206421.386931633]: sequence residual 11 with 8
[ INFO] [1589206421.386949566]: sequence residual 11 with 7
[ INFO] [1589206421.386970183]: sequence residual 12 with 11
[ INFO] [1589206421.386987189]: sequence residual 12 with 10
[ INFO] [1589206421.387007415]: sequence residual 12 with 9
[ INFO] [1589206421.387027003]: sequence residual 12 with 8
[ INFO] [1589206421.387048516]: sequence residual 13 with 12
[ INFO] [1589206421.387065108]: sequence residual 13 with 11
[ INFO] [1589206421.387082301]: sequence residual 13 with 10
[ INFO] [1589206421.387102524]: sequence residual 13 with 9
[ INFO] [1589206421.387124273]: sequence residual 14 with 13
[ INFO] [1589206421.387143646]: sequence residual 14 with 12
[ INFO] [1589206421.387159768]: sequence residual 14 with 11
[ INFO] [1589206421.387180693]: sequence residual 14 with 10
[ INFO] [1589206421.387201753]: sequence residual 15 with 14
[ INFO] [1589206421.387223463]: sequence residual 15 with 13
[ INFO] [1589206421.387240842]: sequence residual 15 with 12
[ INFO] [1589206421.387259011]: sequence residual 15 with 11
[ INFO] [1589206421.387279015]: sequence residual 16 with 15
[ INFO] [1589206421.387301099]: sequence residual 16 with 14
[ INFO] [1589206421.387320022]: sequence residual 16 with 13
[ INFO] [1589206421.387351659]: sequence residual 16 with 12
[ INFO] [1589206421.387372516]: sequence residual 17 with 16
[ INFO] [1589206421.387393333]: sequence residual 17 with 15
[ INFO] [1589206421.387412812]: sequence residual 17 with 14
[ INFO] [1589206421.387432945]: sequence residual 17 with 13
[ INFO] [1589206421.387450510]: sequence residual 18 with 17
[ INFO] [1589206421.387469937]: sequence residual 18 with 16
[ INFO] [1589206421.387488723]: sequence residual 18 with 15
[ INFO] [1589206421.387511329]: sequence residual 18 with 14
[ INFO] [1589206421.387530848]: sequence residual 19 with 18
[ INFO] [1589206421.387549071]: sequence residual 19 with 17
[ INFO] [1589206421.387567153]: sequence residual 19 with 16
[ INFO] [1589206421.387587433]: sequence residual 19 with 15
[ INFO] [1589206421.387610404]: sequence residual 20 with 19
[ INFO] [1589206421.387628928]: sequence residual 20 with 18
[ INFO] [1589206421.387645579]: sequence residual 20 with 17
[ INFO] [1589206421.387665609]: sequence residual 20 with 16
[ INFO] [1589206421.387685751]: sequence residual 21 with 20
[ INFO] [1589206421.387707494]: sequence residual 21 with 19
[ INFO] [1589206421.387724057]: sequence residual 21 with 18
[ INFO] [1589206421.387743040]: sequence residual 21 with 17
[ INFO] [1589206421.387764107]: sequence residual 22 with 21
[ INFO] [1589206421.387785848]: sequence residual 22 with 20
[ INFO] [1589206421.387803658]: sequence residual 22 with 19
[ INFO] [1589206421.387821671]: sequence residual 22 with 18
[ INFO] [1589206421.387844830]: sequence residual 23 with 22
[ INFO] [1589206421.387865146]: sequence residual 23 with 21
[ INFO] [1589206421.387887046]: sequence residual 23 with 20
[ INFO] [1589206421.387903685]: sequence residual 23 with 19
[ INFO] [1589206421.388335183]: sequence residual 24 with 23
[ INFO] [1589206421.388424428]: sequence residual 24 with 22
[ INFO] [1589206421.388493488]: sequence residual 24 with 21
[ INFO] [1589206421.388560730]: sequence residual 24 with 20
[ INFO] [1589206421.388626946]: sequence residual 25 with 24
[ INFO] [1589206421.388693423]: sequence residual 25 with 23
[ INFO] [1589206421.388758722]: sequence residual 25 with 22
[ INFO] [1589206421.388823404]: sequence residual 25 with 21
[ INFO] [1589206421.388892464]: sequence residual 26 with 25
[ INFO] [1589206421.388959100]: sequence residual 26 with 24
[ INFO] [1589206421.389021861]: sequence residual 26 with 23
[ INFO] [1589206421.389086146]: sequence residual 26 with 22
[ INFO] [1589206421.389154264]: sequence residual 27 with 26
[ INFO] [1589206421.389218178]: sequence residual 27 with 25
[ INFO] [1589206421.389282236]: sequence residual 27 with 24
[ INFO] [1589206421.389347680]: sequence residual 27 with 23
[ INFO] [1589206421.389412314]: sequence residual 28 with 27
[ INFO] [1589206421.389476890]: sequence residual 28 with 26
[ INFO] [1589206421.389565730]: sequence residual 28 with 25
[ INFO] [1589206421.390521798]: sequence residual 28 with 24
[ INFO] [1589206421.390637659]: sequence residual 29 with 28
[ INFO] [1589206421.390782802]: sequence residual 29 with 27
[ INFO] [1589206421.391087873]: sequence residual 29 with 26
[ INFO] [1589206421.391236329]: sequence residual 29 with 25
[ INFO] [1589206421.391452518]: sequence residual 30 with 29
[ INFO] [1589206421.391574414]: sequence residual 30 with 28
[ INFO] [1589206421.391642786]: sequence residual 30 with 27
[ INFO] [1589206421.391800382]: sequence residual 30 with 26
[ INFO] [1589206421.391836373]: sequence residual 31 with 30
[ INFO] [1589206421.391863550]: sequence residual 31 with 29
[ INFO] [1589206421.391887025]: sequence residual 31 with 28
[ INFO] [1589206421.391909205]: sequence residual 31 with 27
[ INFO] [1589206421.391932965]: sequence residual 32 with 31
[ INFO] [1589206421.391954301]: sequence residual 32 with 30
[ INFO] [1589206421.391970189]: sequence residual 32 with 29
[ INFO] [1589206421.391991537]: sequence residual 32 with 28
[ INFO] [1589206421.392010028]: sequence residual 33 with 32
[ INFO] [1589206421.392032638]: sequence residual 33 with 31
[ INFO] [1589206421.392055035]: sequence residual 33 with 30
[ INFO] [1589206421.392078946]: sequence residual 33 with 29
[ INFO] [1589206421.392103041]: sequence residual 34 with 33
[ INFO] [1589206421.392129756]: sequence residual 34 with 32
[ INFO] [1589206421.392207132]: sequence residual 34 with 31
[ INFO] [1589206421.392226357]: sequence residual 34 with 30
[ INFO] [1589206421.392245263]: sequence residual 35 with 34
[ INFO] [1589206421.392263823]: sequence residual 35 with 33
[ INFO] [1589206421.392279594]: sequence residual 35 with 32
[ INFO] [1589206421.392296962]: sequence residual 35 with 31
[ INFO] [1589206421.392315008]: sequence residual 36 with 35
[ INFO] [1589206421.392336686]: sequence residual 36 with 34
[ INFO] [1589206421.392354874]: sequence residual 36 with 33
[ INFO] [1589206421.392374480]: sequence residual 36 with 32
[ INFO] [1589206421.392395404]: sequence residual 37 with 36
[ INFO] [1589206421.392413809]: sequence residual 37 with 35
[ INFO] [1589206421.392431780]: sequence residual 37 with 34
[ INFO] [1589206421.392450202]: sequence residual 37 with 33
[ INFO] [1589206421.392468990]: sequence residual 38 with 37
[ INFO] [1589206421.392487753]: sequence residual 38 with 36
[ INFO] [1589206421.392505401]: sequence residual 38 with 35
[ INFO] [1589206421.392524179]: sequence residual 38 with 34
[ INFO] [1589206421.392546026]: sequence residual 39 with 38
[ INFO] [1589206421.392564277]: sequence residual 39 with 37
[ INFO] [1589206421.392583057]: sequence residual 39 with 36
[ INFO] [1589206421.392602109]: sequence residual 39 with 35
[ INFO] [1589206421.392620936]: sequence residual 40 with 39
[ INFO] [1589206421.392639125]: sequence residual 40 with 38
[ INFO] [1589206421.392656355]: sequence residual 40 with 37
[ INFO] [1589206421.392674687]: sequence residual 40 with 36
[ INFO] [1589206421.392693621]: sequence residual 41 with 40
[ INFO] [1589206421.392711836]: sequence residual 41 with 39
[ INFO] [1589206421.392729459]: sequence residual 41 with 38
[ INFO] [1589206421.392747438]: sequence residual 41 with 37
[ INFO] [1589206421.392767811]: sequence residual 42 with 41
[ INFO] [1589206421.392785834]: sequence residual 42 with 40
[ INFO] [1589206421.393040469]: sequence residual 42 with 39
[ INFO] [1589206421.393069206]: sequence residual 42 with 38
[ INFO] [1589206421.393090532]: sequence residual 43 with 42
[ INFO] [1589206421.393110442]: sequence residual 43 with 41
[ INFO] [1589206421.393127244]: sequence residual 43 with 40
[ INFO] [1589206421.393147116]: sequence residual 43 with 39
[ INFO] [1589206421.393168765]: sequence residual 44 with 43
[ INFO] [1589206421.393187753]: sequence residual 44 with 42
[ INFO] [1589206421.393204735]: sequence residual 44 with 41
[ INFO] [1589206421.393223861]: sequence residual 44 with 40
[ INFO] [1589206421.393246117]: sequence residual 45 with 44
[ INFO] [1589206421.393265985]: sequence residual 45 with 43
[ INFO] [1589206421.393282602]: sequence residual 45 with 42
[ INFO] [1589206421.393301543]: sequence residual 45 with 41
[ INFO] [1589206421.393321469]: sequence residual 46 with 45
[ INFO] [1589206421.393341519]: sequence residual 46 with 44
[ INFO] [1589206421.393359113]: sequence residual 46 with 43
[ INFO] [1589206421.393378435]: sequence residual 46 with 42
[ INFO] [1589206421.393398492]: sequence residual 47 with 46
[ INFO] [1589206421.393417783]: sequence residual 47 with 45
[ INFO] [1589206421.393435073]: sequence residual 47 with 44
[ INFO] [1589206421.393454072]: sequence residual 47 with 43
[ INFO] [1589206421.393473164]: sequence residual 48 with 47
[ INFO] [1589206421.393491977]: sequence residual 48 with 46
[ INFO] [1589206421.393511078]: sequence residual 48 with 45
[ INFO] [1589206421.393530099]: sequence residual 48 with 44
[ INFO] [1589206421.393549711]: sequence residual 49 with 48
[ INFO] [1589206421.393568084]: sequence residual 49 with 47
[ INFO] [1589206421.393589069]: sequence residual 49 with 46
[ INFO] [1589206421.393611781]: sequence residual 49 with 45
[ INFO] [1589206421.393650332]: sequence residual 50 with 49
[ INFO] [1589206421.393687210]: sequence residual 50 with 48
[ INFO] [1589206421.393726659]: sequence residual 50 with 47
[ INFO] [1589206421.393767756]: sequence residual 50 with 46
[ INFO] [1589206421.393816444]: sequence residual 51 with 50
[ INFO] [1589206421.393866345]: sequence residual 51 with 49
[ INFO] [1589206421.393915829]: sequence residual 51 with 48
[ INFO] [1589206421.393957123]: sequence residual 51 with 47
[ INFO] [1589206421.394006583]: sequence residual 52 with 51
[ INFO] [1589206421.394069765]: sequence residual 52 with 50
[ INFO] [1589206421.394600026]: sequence residual 52 with 49
[ INFO] [1589206421.394644023]: sequence residual 52 with 48
[ INFO] [1589206421.394677911]: sequence residual 53 with 52
[ INFO] [1589206421.394711109]: sequence residual 53 with 51
[ INFO] [1589206421.394741922]: sequence residual 53 with 50
[ INFO] [1589206421.394768867]: sequence residual 53 with 49
[ INFO] [1589206421.394799545]: sequence residual 54 with 53
[ INFO] [1589206421.394957201]: sequence residual 54 with 52
[ INFO] [1589206421.395006960]: sequence residual 54 with 51
[ INFO] [1589206421.395050419]: sequence residual 54 with 50
[ INFO] [1589206421.395519403]: sequence residual 55 with 54
[ INFO] [1589206421.395540321]: sequence residual 55 with 53
[ INFO] [1589206421.395554827]: sequence residual 55 with 52
[ INFO] [1589206421.395568398]: sequence residual 55 with 51
[ INFO] [1589206421.395582605]: sequence residual 56 with 55
[ INFO] [1589206421.395595279]: sequence residual 56 with 54
[ INFO] [1589206421.395607711]: sequence residual 56 with 53
[ INFO] [1589206421.395620445]: sequence residual 56 with 52
[ INFO] [1589206421.395633711]: sequence residual 57 with 56
[ INFO] [1589206421.395645422]: sequence residual 57 with 55
[ INFO] [1589206421.395656948]: sequence residual 57 with 54
[ INFO] [1589206421.395668527]: sequence residual 57 with 53
[ INFO] [1589206421.395684148]: loop residual 0 with 57
[ INFO] [1589206421.395698064]: sequence residual 58 with 57
[ INFO] [1589206421.395709787]: sequence residual 58 with 56
[ INFO] [1589206421.395720877]: sequence residual 58 with 55
[ INFO] [1589206421.395731963]: sequence residual 58 with 54
[ INFO] [1589206421.395744701]: loop residual 1 with 58
[ INFO] [1589206421.395758699]: sequence residual 59 with 58
[ INFO] [1589206421.395770590]: sequence residual 59 with 57
[ INFO] [1589206421.395781865]: sequence residual 59 with 56
[ INFO] [1589206421.395792988]: sequence residual 59 with 55
[ INFO] [1589206421.395804595]: loop residual 1 with 59
[ INFO] [1589206421.395817742]: sequence residual 60 with 59
[ INFO] [1589206421.395829198]: sequence residual 60 with 58
[ INFO] [1589206421.395840406]: sequence residual 60 with 57
[ INFO] [1589206421.395851614]: sequence residual 60 with 56
[ INFO] [1589206421.395863048]: loop residual 0 with 60
[ INFO] [1589206421.395877146]: sequence residual 61 with 60
[ INFO] [1589206421.395889146]: sequence residual 61 with 59
[ INFO] [1589206421.395900311]: sequence residual 61 with 58
[ INFO] [1589206421.395911383]: sequence residual 61 with 57
[ INFO] [1589206421.395923042]: loop residual 1 with 61
[ INFO] [1589206421.395935965]: sequence residual 62 with 61
[ INFO] [1589206421.395947389]: sequence residual 62 with 60
[ INFO] [1589206421.395958697]: sequence residual 62 with 59
[ INFO] [1589206421.395969767]: sequence residual 62 with 58
[ INFO] [1589206421.395981231]: loop residual 1 with 62
[ INFO] [1589206421.395994140]: sequence residual 63 with 62
[ INFO] [1589206421.396005552]: sequence residual 63 with 61
[ INFO] [1589206421.396016595]: sequence residual 63 with 60
[ INFO] [1589206421.396027627]: sequence residual 63 with 59
[ INFO] [1589206421.396039511]: loop residual 1 with 63
[ INFO] [1589206421.396055607]: sequence residual 64 with 63
[ INFO] [1589206421.396067449]: sequence residual 64 with 62
[ INFO] [1589206421.396078543]: sequence residual 64 with 61
[ INFO] [1589206421.396091624]: sequence residual 64 with 60
[ INFO] [1589206421.396103936]: loop residual 1 with 64
[ INFO] [1589206421.396117900]: sequence residual 65 with 64
[ INFO] [1589206421.396129144]: sequence residual 65 with 63
[ INFO] [1589206421.396140008]: sequence residual 65 with 62
[ INFO] [1589206421.396150852]: sequence residual 65 with 61
[ INFO] [1589206421.396162358]: loop residual 1 with 65
[ INFO] [1589206421.396175865]: sequence residual 66 with 65
[ INFO] [1589206421.396189830]: sequence residual 66 with 64
[ INFO] [1589206421.396216720]: sequence residual 66 with 63
[ INFO] [1589206421.396266488]: sequence residual 66 with 62
[ INFO] [1589206421.396395215]: loopsequence residual 68 with 67
[ INFO] [1589206421.396584107]: sequence residual 68 with 66
[ INFO] [1589206421.396600405]: sequence residual 68 with 65
[ INFO] [1589206421.396612516]: sequence residual 68 with 64
[ INFO] [1589206421.396626143]: loop residual 1 with 68
[ INFO] [1589206421.396647111]: sequence residual 69 with 68
[ INFO] [1589206421.396659657]: sequence residual 69 with 67
[ INFO] [1589206421.396670909]: sequence residual 69 with 66
[ INFO] [1589206421.396682609]: sequence residual 69 with 65
[ INFO] [1589206421.396695770]: sequence residual 70 with 69
[ INFO] [1589206421.396707095]: sequence residual 70 with 68
[ INFO] [1589206421.396718439]: sequence residual 70 with 67
[ INFO] [1589206421.396729529]: sequence residual 70 with 66
[ INFO] [1589206421.396742062]: sequence residual 71 with 70
[ INFO] [1589206421.396753475]: sequence residual 71 with 69
[ INFO] [1589206421.396764675]: sequence residual 71 with 68
[ INFO] [1589206421.396775924]: sequence residual 71 with 67
[ INFO] [1589206421.396787019]: loop residual 1 with 71
[ INFO] [1589206421.396799774]: sequence residual 72 with 71
[ INFO] [1589206421.396811134]: sequence residual 72 with 70
[ INFO] [1589206421.396822068]: sequence residual 72 with 69
[ INFO] [1589206421.396833086]: sequence residual 72 with 68
[ INFO] [1589206421.396844754]: loop residual 0 with 72
[ INFO] [1589206421.396857532]: sequence residual 73 with 72
[ INFO] [1589206421.396868853]: sequence residual 73 with 71
[ INFO] [1589206421.396880018]: sequence residual 73 with 70
[ INFO] [1589206421.396891052]: sequence residual 73 with 69
[ INFO] [1589206421.396902948]: loop residual 1 with 73
```
总共17调闭环边, 因子图:
![loop_factor](loop_factor.png)