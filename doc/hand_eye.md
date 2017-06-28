# eye to hand calibration
将标定板固定在机械臂末端，通过机械臂正运动学可以计算出机器人坐标系(世界坐标系)到机械臂末端的齐次变换矩阵wMe；通过摄像头可以计算出标定板的位姿cMo；移动机械臂末端，采取多组wMe、cMo；根据多组wMe、cMo可计算出wMc。

具体操作：
1. Step 1：记录cMo，wMe

```bash
./detect_board_charuco -w=5 -h=7 --sl=0.04 --ml=0.02 -d=10  -c=calibrationPara.yml -ci=0
```
上述命令将读取calibration.yml中摄像头内参矩阵r_intin，畸变参数r_coeffs，识别5*7，字典类型为10，外框长4cm，内框长2cm的charuco表定板；每按一次s键就保存当前cMo到cMo_save.txt，同时应该人工记录当前机器人末端位姿wMe。

2 Step 2：计算wMc
将多个cMo和wMe分别保存到cMo.txt、wMe.txt，并在文件首行写入"type nums"，其中type = 0表示数据是齐次变幻矩阵形式，type = 1是XYZRPY形式；nums表示个数。
运行以下命令，结果将保存到wMc.txt:
```bash
./hand_eye 2 # 2表示eye to hand, 1 表示eye in hand
```

# eye in hand calibration
将标定板固定在地面不动，多次移动机械臂，记录机械臂末端姿态wMe和标定板在相机坐标系下的姿态cMo；根据多组wMe、cMo可计算出机械臂末端到相机坐标系的齐次变幻矩阵eMc。

具体操作和eye to hand calibration类似，只需将最后一步改成
```bash
./hand_eye 1
```
