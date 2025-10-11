# Robocon Zero - 机器人竞赛决策系统

## 项目概述

Robocon Zero 是一个专为Robocon机器人竞赛设计的智能决策系统。该系统采用模块化架构，集成了路径规划、决策制定、状态管理和可视化等核心功能，为机器人竞赛提供完整的决策支持解决方案。

### 核心特性

- **智能路径规划**：支持A*算法和简单路径规划器
- **多策略决策系统**：基于决策树和行为树的智能决策
- **模块化架构**：采用策略模式和命令模式，支持插件化扩展
- **实时可视化**：提供场地可视化和路径显示功能
- **全面测试框架**：支持单元测试、集成测试和快照测试
- **基准测试**：性能评估和优化分析工具
- **状态管理**：完整的游戏状态跟踪和管理系统

## 系统架构

### 核心模块

```
robocon_zero/
├── executor/          # 执行器核心模块
├── kf_decision/       # 决策系统模块
├── algorithms/        # 算法库模块
├── visualization/     # 可视化模块
├── commands/          # 命令系统模块
├── config/           # 配置管理模块
├── utils/            # 工具模块
├── tests/            # 测试框架
├── demos/            # 演示程序
└── bench/            # 基准测试
```

### 架构设计原则

1. **模块化设计**：各功能模块独立，便于维护和扩展
2. **插件化架构**：支持策略插件的动态注册和加载
3. **接口隔离**：明确定义模块间的接口契约
4. **依赖注入**：通过注册表模式管理组件依赖
5. **事件驱动**：基于命令模式实现松耦合通信

## 安装指南

### 系统要求

- Python 3.8+
- pip 包管理器
- Git 版本控制

### 依赖安装

```bash
# 克隆项目
git clone https://github.com/your-repo/robocon_zero.git
cd robocon_zero

# 安装依赖
pip install -e .
```

### 项目依赖

```toml
[tool.poetry.dependencies]
python = "^3.8"
numpy = "^1.21.0"
matplotlib = "^3.5.0"
pytest = "^7.0.0"
pytest-snapshot = "^0.9.0"
```

## 快速开始

### 基本使用

```python
from executor.core import Executor
from kf_decision.registry import PolicyRegistry
from visualization.field_visualizer import FieldVisualizer

# 初始化执行器
executor = Executor()

# 注册策略
registry = PolicyRegistry()
policy = registry.get_policy("r1_policy")

# 运行决策
game_state = {
    "current_stage": "auto_stage",
    "robot_position": {"x": 0, "y": 0},
    "target_position": {"x": 10, "y": 10}
}

decision = policy.make_decision(game_state)
print(f"决策结果: {decision}")
```

### 运行演示

```bash
# 运行综合演示
python demos/comprehensive_demo.py

# 运行可视化演示
python test_visualizer.py

# 运行基准测试
python benchmark.py
```

## API文档

### 核心执行器 (Executor)

```python
class Executor:
    """系统核心执行器，负责协调各模块工作"""
    
    def __init__(self, config_path: str = None):
        """初始化执行器
        
        Args:
            config_path: 配置文件路径
        """
        pass
    
    def run(self, game_state: dict) -> dict:
        """执行决策流程
        
        Args:
            game_state: 游戏状态字典
            
        Returns:
            执行结果字典
        """
        pass
```

### 策略接口 (Policy)

```python
class Policy(ABC):
    """策略基类，定义策略接口"""
    
    @abstractmethod
    def make_decision(self, game_state: dict) -> dict:
        """制定决策
        
        Args:
            game_state: 当前游戏状态
            
        Returns:
            决策结果字典
        """
        pass
    
    @property
    @abstractmethod
    def name(self) -> str:
        """策略名称"""
        pass
```

### 路径规划器 (PathPlanner)

```python
class AStarPathPlanner:
    """A*路径规划算法实现"""
    
    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int], 
                  obstacles: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """规划路径
        
        Args:
            start: 起始位置
            goal: 目标位置
            obstacles: 障碍物列表
            
        Returns:
            路径点列表
        """
        pass
```

### 决策树 (DecisionTree)

```python
class DecisionTree:
    """决策树算法实现"""
    
    def __init__(self, max_depth: int = 10, min_samples_split: int = 2):
        """初始化决策树
        
        Args:
            max_depth: 最大深度
            min_samples_split: 分裂最小样本数
        """
        pass
    
    def fit(self, X: np.ndarray, y: np.ndarray):
        """训练决策树
        
        Args:
            X: 特征矩阵
            y: 标签向量
        """
        pass
    
    def predict(self, X: np.ndarray) -> np.ndarray:
        """预测
        
        Args:
            X: 特征矩阵
            
        Returns:
            预测结果
        """
        pass
```

## 策略系统

### 内置策略

#### R1策略 (R1Policy)
专为R1机器人设计的策略，支持多个竞赛阶段：

- **自动阶段 (auto_stage)**：自动执行预设动作
- **手动阶段 (manual_stage)**：响应操作员指令
- **MC阶段 (mc_stage)**：执行特定任务逻辑

```python
class R1Policy(Policy):
    def make_decision(self, game_state: dict) -> dict:
        current_stage = game_state.get("current_stage")
        
        if current_stage == "auto_stage":
            return self._handle_auto_stage(game_state)
        elif current_stage == "manual_stage":
            return self._handle_manual_stage(game_state)
        elif current_stage == "mc_stage":
            return self._handle_mc_stage(game_state)
```

#### 简单策略 (SimplePolicy)
基础策略实现，适用于简单场景：

```python
class SimplePolicy(Policy):
    def make_decision(self, game_state: dict) -> dict:
        # 简单的决策逻辑
        return {
            "action": "move",
            "target": game_state.get("target_position"),
            "speed": 1.0
        }
```

### 自定义策略开发

```python
from kf_decision.policy import Policy
from kf_decision.registry import PolicyRegistry

class MyCustomPolicy(Policy):
    @property
    def name(self):
        return "my_custom_policy"
    
    def make_decision(self, game_state: dict) -> dict:
        # 实现自定义决策逻辑
        return {
            "action": "custom_action",
            "parameters": {}
        }

# 注册自定义策略
registry = PolicyRegistry()
registry.register(MyCustomPolicy())
```

## 路径规划

### A*算法

```python
from algorithms.path_planner import AStarPathPlanner

planner = AStarPathPlanner()
path = planner.plan_path(
    start=(0, 0),
    goal=(10, 10),
    obstacles=[(2, 2), (3, 3), (4, 4)]
)

print(f"规划路径: {path}")
```

### 简单路径规划器

```python
from algorithms.path_planner import SimplePathPlanner

planner = SimplePathPlanner()
path = planner.plan_path(
    start=(0, 0),
    goal=(10, 10),
    obstacles=[]
)
```

## 可视化系统

### 场地可视化

```python
from visualization.field_visualizer import FieldVisualizer

visualizer = FieldVisualizer()
visualizer.setup_field(width=20, height=20)
visualizer.add_robot(position=(5, 5), color='red')
visualizer.add_target(position=(15, 15), color='green')
visualizer.add_obstacles([(7, 7), (8, 8), (9, 9)])
visualizer.show()
```

### 路径可视化

```python
# 可视化规划路径
path = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
visualizer.add_path(path, color='blue')
visualizer.show()
```

## 测试框架

### 运行测试

```bash
# 运行所有测试
pytest

# 运行特定测试文件
pytest tests/test_new_features.py

# 运行策略测试
pytest tests/test_policy.py

# 运行R1策略测试
pytest tests/test_r1_policy.py

# 生成测试报告
pytest --html=report.html --self-contained-html
```

### 快照测试

```python
def test_policy_snapshot(snapshot):
    """策略输出快照测试"""
    policy = R1Policy()
    game_state = {
        "current_stage": "auto_stage",
        "robot_position": {"x": 0, "y": 0},
        "target_position": {"x": 10, "y": 10}
    }
    
    result = policy.make_decision(game_state)
    snapshot.assert_match(result)
```

### 测试覆盖率

```bash
# 运行覆盖率测试
pytest --cov=kf_decision --cov=algorithms --cov=executor

# 生成覆盖率报告
pytest --cov=kf_decision --cov-report=html
```

## 基准测试

### 性能基准测试

```python
from bench.runner import BenchmarkRunner

runner = BenchmarkRunner()
results = runner.run_benchmarks([
    "path_planning",
    "decision_making",
    "state_management"
])

print(f"基准测试结果: {results}")
```

### 自定义基准测试

```python
def custom_benchmark():
    """自定义性能测试"""
    import time
    
    start_time = time.time()
    # 执行测试逻辑
    end_time = time.time()
    
    return {
        "execution_time": end_time - start_time,
        "memory_usage": get_memory_usage()
    }

runner.register_benchmark("custom", custom_benchmark)
```

## 配置管理

### 场地配置

```python
from config.field_config import FieldConfig

config = FieldConfig(
    width=20,
    height=20,
    obstacles=[(2, 2), (3, 3), (4, 4)],
    start_positions={"R1": (0, 0), "R2": (19, 19)},
    target_positions={"T1": (10, 10), "T2": (15, 15)}
)
```

### 日志配置

```python
from utils.logger import setup_logger

logger = setup_logger(
    name="robocon",
    level="INFO",
    log_file="logs/robocon.log"
)
```

## 开发指南

### 项目结构

```
robocon_zero/
├── algorithms/          # 算法实现
│   ├── __init__.py
│   ├── decision_tree.py
│   └── path_planner.py
├── commands/            # 命令模式实现
│   ├── __init__.py
│   └── commands.py
├── config/              # 配置管理
│   ├── __init__.py
│   └── field_config.py
├── executor/            # 核心执行器
│   ├── __init__.py
│   └── core.py
├── kf_decision/         # 决策系统
│   ├── __init__.py
│   ├── abc.py          # 抽象基类
│   ├── policy.py       # 策略基类
│   ├── r1_policy.py    # R1策略
│   ├── registry.py     # 策略注册表
│   ├── simple_policy.py # 简单策略
│   └── types.py        # 类型定义
├── utils/               # 工具模块
│   ├── __init__.py
│   ├── analyzer.py     # 分析器
│   ├── logger.py       # 日志器
│   └── rule_validator.py # 规则验证器
├── visualization/       # 可视化
│   └── field_visualizer.py
├── tests/               # 测试
│   ├── __init__.py
│   ├── test_new_features.py
│   ├── test_policy.py
│   └── test_r1_policy.py
├── demos/               # 演示
│   ├── comprehensive_demo.py
│   └── demo_new_features.py
└── bench/               # 基准测试
    ├── __init__.py
    └── runner.py
```

### 编码规范

1. **命名规范**：
   - 类名使用驼峰命名法：`ClassName`
   - 函数名使用小写加下划线：`function_name`
   - 常量使用大写加下划线：`CONSTANT_NAME`

2. **类型注解**：
   - 所有公共API必须包含类型注解
   - 使用`from typing import`导入类型

3. **文档字符串**：
   - 所有公共类和方法必须包含docstring
   - 使用Google风格文档字符串

4. **测试要求**：
   - 所有新功能必须包含单元测试
   - 核心功能必须包含集成测试
   - 策略类必须包含快照测试

### 贡献指南

1. **Fork项目**并创建特性分支
2. **遵循编码规范**和最佳实践
3. **添加测试**确保代码覆盖率
4. **更新文档**包含API变更
5. **提交Pull Request**并描述变更内容

## 故障排除

### 常见问题

#### 策略注册失败
```python
# 确保策略正确继承自Policy基类
class MyPolicy(Policy):
    @property
    def name(self):
        return "my_policy"  # 必须实现name属性
```

#### 路径规划无结果
```python
# 检查障碍物是否阻挡了所有路径
path = planner.plan_path(start, goal, obstacles)
if not path:
    print("无法找到有效路径，请检查障碍物设置")
```

#### 可视化显示异常
```python
# 确保matplotlib后端正确设置
import matplotlib
matplotlib.use('TkAgg')  # 或其他适合的后端
```

### 调试工具

```python
from utils.logger import get_logger

logger = get_logger(__name__)
logger.debug("调试信息")
logger.info("一般信息")
logger.warning("警告信息")
logger.error("错误信息")
```