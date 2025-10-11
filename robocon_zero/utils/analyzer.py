# utils/analyzer.py
"""
数据分析工具
用于分析机器人行为和策略性能
"""

import json
import statistics
from typing import List, Dict, Any, Tuple
from dataclasses import dataclass, asdict
from kf_decision.types import GameState
from utils.logger import LogEntry


@dataclass
class PerformanceMetrics:
    """性能指标"""
    total_time: float  # 总执行时间 (毫秒)
    total_distance: float  # 总移动距离 (毫米)
    total_commands: int  # 总命令数
    violations: int  # 违规次数
    kfs_collected: int  # 收集的KFS数量
    weapons_assembled: int  # 组装的武器数量
    arena_kfs_placed: int  # 竞技场放置的KFS数量


@dataclass
class StrategyAnalysis:
    """策略分析结果"""
    strategy_name: str
    run_count: int
    avg_metrics: PerformanceMetrics
    best_metrics: PerformanceMetrics
    worst_metrics: PerformanceMetrics
    success_rate: float  # 成功率


class DataAnalyzer:
    """数据分析器"""
    
    def __init__(self):
        self.runs_data: List[Dict[str, Any]] = []
        
    def add_run_data(self, run_data: Dict[str, Any]):
        """添加一次运行的数据"""
        self.runs_data.append(run_data)
        
    def analyze_single_run(self, log_entries: List[LogEntry], game_states: List[GameState]) -> PerformanceMetrics:
        """分析单次运行的性能"""
        # 计算总时间
        if log_entries:
            total_time = (log_entries[-1].timestamp - log_entries[0].timestamp) * 1000  # 转换为毫秒
        else:
            total_time = 0
            
        # 计算总距离（需要从游戏状态中获取）
        total_distance = 0
        total_commands = len(log_entries)
        
        # 计算违规次数
        violations = sum(1 for entry in log_entries if entry.level == "WARNING")
        
        # 计算收集的KFS数量
        kfs_collected = 0
        if game_states:
            # 假设最后状态的KFS数量就是收集的数量
            final_state = game_states[-1]
            kfs_collected = len(final_state.r1_kfs) + len(final_state.r2_kfs)
            
        # 计算组装的武器数量
        weapons_assembled = 0
        if game_states:
            final_state = game_states[-1]
            weapons_assembled = len(final_state.assembled_weapons)
            
        # 计算竞技场放置的KFS数量
        arena_kfs_placed = 0
        # 这需要更复杂的逻辑来分析井字棋状态
        
        return PerformanceMetrics(
            total_time=total_time,
            total_distance=total_distance,
            total_commands=total_commands,
            violations=violations,
            kfs_collected=kfs_collected,
            weapons_assembled=weapons_assembled,
            arena_kfs_placed=arena_kfs_placed
        )
        
    def analyze_multiple_runs(self, strategy_name: str) -> StrategyAnalysis:
        """分析多次运行的结果"""
        if not self.runs_data:
            raise ValueError("没有运行数据可供分析")
            
        # 提取所有性能指标
        metrics_list = [run["metrics"] for run in self.runs_data if "metrics" in run]
        
        if not metrics_list:
            raise ValueError("运行数据中没有性能指标")
            
        # 计算平均指标
        avg_metrics = self._calculate_average_metrics(metrics_list)
        
        # 找到最好和最差的指标
        best_metrics = self._find_best_metrics(metrics_list)
        worst_metrics = self._find_worst_metrics(metrics_list)
        
        # 计算成功率
        success_count = sum(1 for run in self.runs_data if run.get("success", False))
        success_rate = success_count / len(self.runs_data)
        
        return StrategyAnalysis(
            strategy_name=strategy_name,
            run_count=len(self.runs_data),
            avg_metrics=avg_metrics,
            best_metrics=best_metrics,
            worst_metrics=worst_metrics,
            success_rate=success_rate
        )
        
    def _calculate_average_metrics(self, metrics_list: List[PerformanceMetrics]) -> PerformanceMetrics:
        """计算平均性能指标"""
        if not metrics_list:
            return PerformanceMetrics(0, 0, 0, 0, 0, 0, 0)
            
        avg_total_time = statistics.mean(m.total_time for m in metrics_list)
        avg_total_distance = statistics.mean(m.total_distance for m in metrics_list)
        avg_total_commands = statistics.mean(m.total_commands for m in metrics_list)
        avg_violations = statistics.mean(m.violations for m in metrics_list)
        avg_kfs_collected = statistics.mean(m.kfs_collected for m in metrics_list)
        avg_weapons_assembled = statistics.mean(m.weapons_assembled for m in metrics_list)
        avg_arena_kfs_placed = statistics.mean(m.arena_kfs_placed for m in metrics_list)
        
        return PerformanceMetrics(
            total_time=avg_total_time,
            total_distance=avg_total_distance,
            total_commands=int(avg_total_commands),
            violations=int(avg_violations),
            kfs_collected=int(avg_kfs_collected),
            weapons_assembled=int(avg_weapons_assembled),
            arena_kfs_placed=int(avg_arena_kfs_placed)
        )
        
    def _find_best_metrics(self, metrics_list: List[PerformanceMetrics]) -> PerformanceMetrics:
        """找到最好的性能指标"""
        if not metrics_list:
            return PerformanceMetrics(0, 0, 0, 0, 0, 0, 0)
            
        # 最短时间
        best_time = min(m.total_time for m in metrics_list)
        # 最少违规
        best_violations = min(m.violations for m in metrics_list)
        # 最多KFS收集
        best_kfs = max(m.kfs_collected for m in metrics_list)
        # 最多武器组装
        best_weapons = max(m.weapons_assembled for m in metrics_list)
        
        # 这里简化处理，实际应该根据具体目标选择最好的指标
        best_metric = min(metrics_list, key=lambda m: m.total_time)
        return best_metric
        
    def _find_worst_metrics(self, metrics_list: List[PerformanceMetrics]) -> PerformanceMetrics:
        """找到最差的性能指标"""
        if not metrics_list:
            return PerformanceMetrics(0, 0, 0, 0, 0, 0, 0)
            
        # 这里简化处理，实际应该根据具体目标选择最差的指标
        worst_metric = max(metrics_list, key=lambda m: m.total_time)
        return worst_metric
        
    def generate_report(self, analysis: StrategyAnalysis) -> str:
        """生成分析报告"""
        report = f"""
策略分析报告
================

策略名称: {analysis.strategy_name}
运行次数: {analysis.run_count}
成功率: {analysis.success_rate:.2%}

平均性能指标:
  总时间: {analysis.avg_metrics.total_time:.2f} ms
  总距离: {analysis.avg_metrics.total_distance:.2f} mm
  总命令数: {analysis.avg_metrics.total_commands}
  违规次数: {analysis.avg_metrics.violations}
  收集KFS: {analysis.avg_metrics.kfs_collected}
  组装武器: {analysis.avg_metrics.weapons_assembled}
  竞技场KFS: {analysis.avg_metrics.arena_kfs_placed}

最好性能指标:
  总时间: {analysis.best_metrics.total_time:.2f} ms
  总距离: {analysis.best_metrics.total_distance:.2f} mm
  总命令数: {analysis.best_metrics.total_commands}
  违规次数: {analysis.best_metrics.violations}
  收集KFS: {analysis.best_metrics.kfs_collected}
  组装武器: {analysis.best_metrics.weapons_assembled}
  竞技场KFS: {analysis.best_metrics.arena_kfs_placed}

最差性能指标:
  总时间: {analysis.worst_metrics.total_time:.2f} ms
  总距离: {analysis.worst_metrics.total_distance:.2f} mm
  总命令数: {analysis.worst_metrics.total_commands}
  违规次数: {analysis.worst_metrics.violations}
  收集KFS: {analysis.worst_metrics.kfs_collected}
  组装武器: {analysis.worst_metrics.weapons_assembled}
  竞技场KFS: {analysis.worst_metrics.arena_kfs_placed}
        """
        return report.strip()
        
    def save_analysis(self, analysis: StrategyAnalysis, filename: str):
        """保存分析结果到文件"""
        data = {
            "strategy_name": analysis.strategy_name,
            "run_count": analysis.run_count,
            "success_rate": analysis.success_rate,
            "avg_metrics": asdict(analysis.avg_metrics),
            "best_metrics": asdict(analysis.best_metrics),
            "worst_metrics": asdict(analysis.worst_metrics)
        }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)


class ComparisonAnalyzer:
    """策略比较分析器"""
    
    def __init__(self):
        self.analyzers: Dict[str, DataAnalyzer] = {}
        
    def add_strategy_data(self, strategy_name: str, run_data: Dict[str, Any]):
        """添加策略的运行数据"""
        if strategy_name not in self.analyzers:
            self.analyzers[strategy_name] = DataAnalyzer()
        self.analyzers[strategy_name].add_run_data(run_data)
        
    def compare_strategies(self) -> Dict[str, StrategyAnalysis]:
        """比较所有策略"""
        results = {}
        for strategy_name, analyzer in self.analyzers.items():
            try:
                analysis = analyzer.analyze_multiple_runs(strategy_name)
                results[strategy_name] = analysis
            except ValueError as e:
                print(f"分析策略 {strategy_name} 时出错: {e}")
        return results
        
    def generate_comparison_report(self) -> str:
        """生成比较报告"""
        analyses = self.compare_strategies()
        
        if not analyses:
            return "没有足够的数据进行比较"
            
        report = "策略比较报告\n==============\n\n"
        
        # 按成功率排序
        sorted_analyses = sorted(analyses.items(), key=lambda x: x[1].success_rate, reverse=True)
        
        for i, (strategy_name, analysis) in enumerate(sorted_analyses, 1):
            report += f"{i}. {strategy_name}\n"
            report += f"   成功率: {analysis.success_rate:.2%}\n"
            report += f"   平均时间: {analysis.avg_metrics.total_time:.2f} ms\n"
            report += f"   平均KFS收集: {analysis.avg_metrics.kfs_collected}\n"
            report += f"   平均武器组装: {analysis.avg_metrics.weapons_assembled}\n\n"
            
        return report.strip()


# 全局分析器实例
analyzer = DataAnalyzer()
comparator = ComparisonAnalyzer()


def get_analyzer() -> DataAnalyzer:
    """获取全局分析器实例"""
    return analyzer


def get_comparator() -> ComparisonAnalyzer:
    """获取全局比较分析器实例"""
    return comparator
