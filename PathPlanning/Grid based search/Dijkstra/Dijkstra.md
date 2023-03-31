#   Dijkstra 算法扩展版本
### 双向Dijkstra算法（Bidirectional Dijkstra Algorithm）
该算法同时从起点和终点开始搜索，直到两个搜索过程相遇。该算法可以显著降低搜索时间，特别是在稠密图中。

### A算法（A-star Algorithm）
该算法利用启发式函数来指导搜索过程，以减少搜索的节点数。通过引入启发式函数，A算法可以更快地找到最短路径。

### D算法（D-star Algorithm）
该算法是对A算法的一种扩展，它可以处理动态环境中的路径规划问题。在D*算法中，当地图发生改变时，只需要重新计算受影响的区域，而不需要重新计算整个地图。

### Contraction Hierarchies算法（CH算法）
该算法通过缩小网络规模和建立超级节点的方法来加速最短路径计算。通过将节点分层，CH算法可以有效地减少搜索的节点数。

### CHD算法（Combining Contraction Hierarchies with Dijkstra Algorithm）
该算法是对CH算法的一种改进，它结合了CH算法和Dijkstra算法的优点，既可以快速计算最短路径，又可以避免CH算法的局限性。

### Preprocessing Hierarchies算法（PH算法）
该算法利用预处理技术来加速最短路径计算。在PH算法中，通过将节点分层，预处理出不同层之间的最短路径，然后在查询时只需要在相邻层之间进行搜索，从而大大降低了搜索时间。

### Hub Labeling算法
该算法也是一种基于预处理的算法，它通过构建Hub节点和Label来实现快速最短路径计算。在该算法中，通过标记每个节点的Hub和它到其他Hub的最短路径，然后在查询时只需要在相邻的Hub之间进行搜索，从而实现快速的最短路径计算。