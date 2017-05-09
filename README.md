## 相机标定流程

1. 使用get_image获取图片，保存与pic路径下。编译格式为（以下相似）：

   ```shell
   g++ -I /usr/local/Cellar/opencv/2.4.13.1/include srcFile.cpp `pkg-config --libs opencv --cflags` -o targetFile
   ```

2. 使用imagelist_creator生成图片列表。编译格式同上。使用时指明路径与列表输出文件：

   ```shell
   ./imagelist_creator imgList.yaml ./pic/*.png
   ```

3. 使用calibration.cpp进行标定。编译格式同上。使用时设定参数如下：

   ```shell
   ./calibrator -w 6 -h 9 -s 0.026162  -o para.yml -op -oe imgList.yaml
   ```

   得到`para.yml`即为标定结果。

4. 可用calibrate_image查看标定结果。

