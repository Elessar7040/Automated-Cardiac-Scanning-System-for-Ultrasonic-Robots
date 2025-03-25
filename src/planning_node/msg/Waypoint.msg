# 锚点类型
uint8 TYPE_INIT=1      # 初始点
uint8 TYPE_WORK=2      # 工作点
uint8 TYPE_END=3       # 结束点
uint8 waypoint_type

# 锚点工作模式
uint8 MODE_PICK=1      # 取料
uint8 MODE_LOAD=2      # 上料
uint8 MODE_UNLOAD=3    # 下料
uint8 MODE_STACK=4     # 码垛
uint8 waypoint_mode

# 航点位姿
geometry_msgs/Pose waypoint_pose

# 航点规划模式
uint8 PLAN_STOP=1      # 到点停止
uint8 PLAN_SMOOTH=2    # 平滑过渡
uint8 PLAN_FORCE=3     # 力反馈停止
uint8 waypoint_plan_mode

# 航点行为
uint8 ACTION_ALGORITHM=1    # 启动算法
uint8 ACTION_END_CONTROL=2  # 末端控制
uint8 action_type

# 算法模式
uint8 ALG_RECOGNITION=1     # 识别算法
uint8 algorithm_mode

# 末端执行器
uint8 END_SUCTION=1        # 吸盘
uint8 END_PROBE=2        # 探针
uint8 end_effector_type
uint32 end_effector_id     # 末端夹具型号
float32 end_effector_action # 末端夹具动作

# 设备控制
uint8 EQUIP_BENDER=1       # 折弯机
uint8 EQUIP_PUNCH=2        # 冲压机
uint8 equipment_type
uint32 equipment_id        # 设备ID 