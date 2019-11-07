# StructSolver
 结构力学求解器 Structure Mechanics Solver

要使用求解器，首先应该打开命令行，输入求解器的名称，然后加入要求解的结构对应的文件。

文件格式为：

先声明杆件（基础base无需也不应声明），杆件名称后可加上参数方程表示的方程和参数定义域表示杆件形状，格式为：
pole name (x = f(t)), (y = g(t)), (a, b)

再声明约束（a b为杆件名称，基础为base），杆件名称后须加上约束的坐标，链杆约束后还需提供角度制的角度，格式如下
lever a b x1 y1 53
pin a base x2 y2
rigid b base x3 y3
