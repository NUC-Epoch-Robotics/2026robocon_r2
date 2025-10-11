# config/field_config.py
"""
场地配置参数
根据ABU Robocon 2026规则书定义
"""

# 场地尺寸参数 (毫米)
FIELD_WIDTH = 12000  # 场地宽度
FIELD_HEIGHT = 8000  # 场地高度

# 区域尺寸参数
MC_WIDTH = 3000      # 武器组装区宽度
MC_HEIGHT = 4000     # 武器组装区高度

MF_WIDTH = 6000      # 梅花林区宽度
MF_HEIGHT = 4000     # 梅花林区高度

ARENA_WIDTH = 3000   # 竞技场区宽度
ARENA_HEIGHT = 4000  # 竞技场区高度

# 梅花林区块参数
MF_BLOCK_SIZE = 1200  # 每个区块大小 (1200x1200mm)
MF_BLOCK_HEIGHT_LEVEL1 = 200  # 一级区块高度
MF_BLOCK_HEIGHT_LEVEL2 = 400  # 二级区块高度
MF_BLOCK_HEIGHT_LEVEL3 = 600  # 三级区块高度

# 竞技场参数
ARENA_RAMP_LENGTH = 1500  # 坡道长度
ARENA_RAMP_ANGLE = 15     # 坡道角度 (度)
ARENA_RAMP_HEIGHT = 400   # 坡道高度差

ARENA_TIC_TAC_TOE_LENGTH = 1620  # 井字棋场地长度
ARENA_TIC_TAC_TOE_BASE_WIDTH = 320  # 井字棋底座宽度
ARENA_TIC_TAC_TOE_HEIGHT = 400   # 井字棋场地高度
ARENA_TIC_TAC_TOE_CELL_OUTER_SIZE = 540  # 格子外部长宽
ARENA_TIC_TAC_TOE_CELL_INNER_SIZE = 500  # 格子内部长宽
ARENA_TIC_TAC_TOE_CELL_DEPTH = 320   # 格子深度

# KFS参数
KFS_SIZE = 350  # KFS立方体尺寸 (350x350x350mm)
KFS_WEIGHT = 630  # KFS重量 (克)

# 机器人参数
R1_START_SIZE_X = 1000  # R1起始尺寸 X方向
R1_START_SIZE_Y = 1000  # R1起始尺寸 Y方向
R1_START_SIZE_Z = 1000  # R1起始尺寸 Z方向

R1_MAX_SIZE_X = 1000   # R1最大尺寸 X方向
R1_MAX_SIZE_Y = 1800   # R1最大尺寸 Y方向
R1_MAX_SIZE_Z = 1300   # R1最大尺寸 Z方向

R2_START_SIZE_X = 800   # R2起始尺寸 X方向
R2_START_SIZE_Y = 800   # R2起始尺寸 Y方向
R2_START_SIZE_Z = 800   # R2起始尺寸 Z方向

R2_MAX_SIZE_X = 800    # R2最大尺寸 X方向
R2_MAX_SIZE_Y = 1300   # R2最大尺寸 Y方向
R2_MAX_SIZE_Z = 1300   # R2最大尺寸 Z方向

# 武器参数
WEAPON_STAFF_LENGTH = 1000  # 棍子长度
WEAPON_MAX_LENGTH = 1295    # 最大组装武器长度

# 颜色定义 (RGB)
COLORS = {
    "RED_START_ZONE": (223, 34, 34),
    "BLUE_START_ZONE": (50, 0, 255),
    "MF_LEVEL1": (41, 82, 16),
    "MF_LEVEL2": (42, 113, 56),
    "MF_LEVEL3": (152, 166, 80),
    "ARENA_RAMP": (192, 189, 182),
    "TIC_TAC_TOE_RACK": (255, 255, 255),
    "USED_WEAPON_AREA": (255, 255, 0),
}
