#   D* 算法扩展版本
### D* Lite
在D*算法的基础上，通过引入更精确的启发式函数和优化的搜索策略来提高算法的性能。

### Focused D*
针对D*算法中的全局更新操作，提出了基于焦点搜索的策略，仅更新与机器人当前位置和目标位置相邻的部分地图，从而减少计算量和更新频率。

### Incremental D*
通过将地图的改变建模为增量更新，而不是重新规划整个路径，从而提高算法的实时性和效率。

### Adaptive D*
通过在线调整启发式函数的权重来平衡搜索的速度和质量，以适应不同场景下的需求。

### Hierarchical D*
通过引入层次化搜索策略，将全局搜索分解为多个子问题，降低算法的时间复杂度。

### Anytime D*
通过使用增量启发式函数，Anytime D*允许在任何时间停止算法并返回最优或次优路径，从而使算法更具实时性。