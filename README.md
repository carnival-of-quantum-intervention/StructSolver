# StructSolver
 结构力学求解器 Structure Mechanics Solver

推荐使用命令行来调用该程序。
Using this program in command line is recommended.

有两种求解模式，分别是体系模式和结构模式。
There are 2 solving modes, system or structure.

你可以用命令行参数来指定求解模式（"?struct"和"?system"），也可以文件后缀名暗示（.struct和.system）。
You can appoint a mode by cmd arguments("?struct" and "?system"), or just imply the mode by the suffix of filename(.struct and .system).

对于体系模式：
As for system mode:

先声明杆件（基础base无需也不应声明），杆件名称后可加上外力，格式为：
pole name f(xf, yf) = (fx, fy)

再声明约束（a b为杆件名称，基础为base），杆件名称后须加上约束的坐标，链杆约束后还需提供角度制的角度，格式如下
lever a b x1 y1 53
pin a base x2 y2
rigid b base x3 y3

对于结构模式：
As for struture mode:

