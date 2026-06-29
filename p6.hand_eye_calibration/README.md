## 概述

相机标定总共分三步，抓取照片，计算和验证。  
抓取照片： collect_data.py  
计算标定参数： compute.py  
验证标定参数：validate.py  

### 过程
- 配置  

修改config.yaml，选择eye_in_hand还是eye_to_hand。  

- 抓取照片  
```sh
python collect_data.py
```

- 计算标定参数  
```sh
python compute.py --data data2026062904 --mode eye_to_hand
```

- 验证标定参数  
```sh
python validate.py --cam -0.067 -0.048 0.783 --gripper 0.2515 --above 0.01 --clearance 0.005 --speed 20 --ip 192.168.1.18 --port 8080
python validate.py --cam -0.013 0.059 0.375 --gripper 0.2515 --above 0.01 --clearance 0.0 --speed 20 --ip 192.168.1.18 --port 8080
```

### 参考
[睿尔曼机械臂标定说明](./rm65b标定说明.md)  
