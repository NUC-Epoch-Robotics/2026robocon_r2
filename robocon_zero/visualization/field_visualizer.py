# visualization/field_visualizer.py
"""
场地可视化工具
使用matplotlib创建2D场地视图
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from typing import List, Tuple, Optional
from kf_decision.types import GameState, RobotPose, KungFuScroll, KFSType
from config.field_config import *


class FieldVisualizer:
    """场地可视化器"""
    
    def __init__(self, figsize: Tuple[int, int] = (12, 8)):
        self.fig, self.ax = plt.subplots(1, 1, figsize=figsize)
        self.fig.suptitle("ABU Robocon 2026 - Kung Fu Quest")
        
    def draw_field(self):
        """绘制场地"""
        # 绘制场地边界
        field_rect = patches.Rectangle(
            (0, 0), FIELD_WIDTH, FIELD_HEIGHT,
            linewidth=2, edgecolor='black', facecolor='none'
        )
        self.ax.add_patch(field_rect)
        
        # 绘制区域边界
        # 武器组装区 (左侧)
        mc_rect = patches.Rectangle(
            (0, 0), MC_WIDTH, MC_HEIGHT,
            linewidth=1, edgecolor='blue', facecolor='lightblue', alpha=0.3
        )
        self.ax.add_patch(mc_rect)
        self.ax.text(MC_WIDTH/2, MC_HEIGHT/2, "MC", ha='center', va='center', fontsize=12)
        
        # 梅花林区 (中间)
        mf_rect = patches.Rectangle(
            (MC_WIDTH, 0), MF_WIDTH, MF_HEIGHT,
            linewidth=1, edgecolor='green', facecolor='lightgreen', alpha=0.3
        )
        self.ax.add_patch(mf_rect)
        self.ax.text(MC_WIDTH + MF_WIDTH/2, MF_HEIGHT/2, "MF", ha='center', va='center', fontsize=12)
        
        # 竞技场区 (右侧)
        arena_rect = patches.Rectangle(
            (MC_WIDTH + MF_WIDTH, 0), ARENA_WIDTH, ARENA_HEIGHT,
            linewidth=1, edgecolor='red', facecolor='lightcoral', alpha=0.3
        )
        self.ax.add_patch(arena_rect)
        self.ax.text(MC_WIDTH + MF_WIDTH + ARENA_WIDTH/2, ARENA_HEIGHT/2, "ARENA", ha='center', va='center', fontsize=12)
        
        # 绘制梅花林区块
        self._draw_mf_blocks()
        
        # 绘制竞技场井字棋
        self._draw_tic_tac_toe()
        
    def _draw_mf_blocks(self):
        """绘制梅花林区块"""
        # 计算区块数量
        blocks_x = MF_WIDTH // MF_BLOCK_SIZE
        blocks_y = MF_HEIGHT // MF_BLOCK_SIZE
        
        # 绘制区块 (简化为不同颜色的矩形)
        for i in range(int(blocks_x)):
            for j in range(int(blocks_y)):
                # 简化处理，实际应该根据规则书中的三级高度设置不同颜色
                if (i + j) % 3 == 0:  # 一级区块
                    color = 'green'
                    alpha = 0.4
                elif (i + j) % 3 == 1:  # 二级区块
                    color = 'darkgreen'
                    alpha = 0.6
                else:  # 三级区块
                    color = 'forestgreen'
                    alpha = 0.8
                    
                block_rect = patches.Rectangle(
                    (MC_WIDTH + i * MF_BLOCK_SIZE, j * MF_BLOCK_SIZE),
                    MF_BLOCK_SIZE, MF_BLOCK_SIZE,
                    linewidth=0.5, edgecolor='black', facecolor=color, alpha=alpha
                )
                self.ax.add_patch(block_rect)
                
    def _draw_tic_tac_toe(self):
        """绘制竞技场井字棋"""
        # 井字棋位置 (简化)
        tic_tac_toe_x = MC_WIDTH + MF_WIDTH + ARENA_WIDTH/2
        tic_tac_toe_y = ARENA_HEIGHT/2
        
        # 绘制井字棋网格
        for i in range(4):  # 垂直线
            self.ax.plot(
                [tic_tac_toe_x - ARENA_TIC_TAC_TOE_LENGTH/2 + i * ARENA_TIC_TAC_TOE_CELL_OUTER_SIZE/3,
                 tic_tac_toe_x - ARENA_TIC_TAC_TOE_LENGTH/2 + i * ARENA_TIC_TAC_TOE_CELL_OUTER_SIZE/3],
                [tic_tac_toe_y - ARENA_TIC_TAC_TOE_LENGTH/2,
                 tic_tac_toe_y + ARENA_TIC_TAC_TOE_LENGTH/2],
                'black', linewidth=2
            )
            
        for i in range(4):  # 水平线
            self.ax.plot(
                [tic_tac_toe_x - ARENA_TIC_TAC_TOE_LENGTH/2,
                 tic_tac_toe_x + ARENA_TIC_TAC_TOE_LENGTH/2],
                [tic_tac_toe_y - ARENA_TIC_TAC_TOE_LENGTH/2 + i * ARENA_TIC_TAC_TOE_CELL_OUTER_SIZE/3,
                 tic_tac_toe_y - ARENA_TIC_TAC_TOE_LENGTH/2 + i * ARENA_TIC_TAC_TOE_CELL_OUTER_SIZE/3],
                'black', linewidth=2
            )
            
    def draw_robots(self, state: GameState):
        """绘制机器人"""
        # 绘制R1 (蓝色)
        r1_circle = patches.Circle(
            (state.r1_pose.x, state.r1_pose.y),
            R1_START_SIZE_X/2,
            color='blue', alpha=0.7
        )
        self.ax.add_patch(r1_circle)
        self.ax.text(state.r1_pose.x, state.r1_pose.y, "R1", ha='center', va='center', color='white')
        
        # 绘制R2 (红色)
        r2_circle = patches.Circle(
            (state.r2_pose.x, state.r2_pose.y),
            R2_START_SIZE_X/2,
            color='red', alpha=0.7
        )
        self.ax.add_patch(r2_circle)
        self.ax.text(state.r2_pose.x, state.r2_pose.y, "R2", ha='center', va='center', color='white')
        
    def draw_kfs(self, mf_slots: dict):
        """绘制KFS"""
        for slot_name, kfs in mf_slots.items():
            # 简化处理，实际应该根据具体位置绘制
            color = 'yellow' if kfs.kfs_type == KFSType.REAL else 'gray'
            kfs_rect = patches.Rectangle(
                (kfs.x, kfs.y),
                KFS_SIZE, KFS_SIZE,
                linewidth=1, edgecolor='black', facecolor=color, alpha=0.8
            )
            self.ax.add_patch(kfs_rect)
            
    def draw_weapons(self, weapons: List):
        """绘制武器"""
        for weapon in weapons:
            # 简化处理
            weapon_line = plt.Line2D(
                [weapon.x, weapon.x + 100],
                [weapon.y, weapon.y],
                linewidth=3, color='brown'
            )
            self.ax.add_line(weapon_line)
            
    def update_view(self, state: GameState):
        """更新视图"""
        self.ax.clear()
        self.draw_field()
        self.draw_robots(state)
        self.draw_kfs(state.mf_slots)
        self.draw_weapons(state.weapons)
        
        # 设置坐标轴
        self.ax.set_xlim(0, FIELD_WIDTH)
        self.ax.set_ylim(0, FIELD_HEIGHT)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
    def show(self):
        """显示图形"""
        plt.show()
        
    def save(self, filename: str):
        """保存图形"""
        self.fig.savefig(filename, dpi=300, bbox_inches='tight')


# 全局可视化器实例
visualizer = FieldVisualizer()


def visualize_game_state(state: GameState):
    """可视化游戏状态"""
    visualizer.update_view(state)
    visualizer.show()


def save_game_state_visualization(state: GameState, filename: str):
    """保存游戏状态可视化"""
    visualizer.update_view(state)
    visualizer.save(filename)
