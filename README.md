# 智能汽车练习代码

TC264（英飞凌 AURIX）智能车循迹程序，学习过程中的练习代码，基于 AURIX Development Studio（ADS）与 iLLD 库。

## 内容

- `code/control.c` —— 2026-10-17 初始版本：纯开环控制
  - 5 ms PIT 中断驱动
  - 摄像头第 35–60 行循迹：取中线、平均误差 + 差分，PD 控制舵机
  - 电机固定占空比直接驱动（`dianjizhuandong`）
  - 编码器与速度闭环尚未加入

## 开发环境

- IDE：AURIX Development Studio
- 芯片：TC264（双核）
- 外设：MT9V034 摄像头、舵机、直流电机驱动板
- 语言：C
写在后面：笔者为该领域初学者，创建该仓库的目的仅是个人学习归档，仅供参考
