#   Breadth-First Search (BFS)算法扩展版本
### 双向BFS
从起始顶点和目标顶点同时开始BFS，直到两个搜索相遇。

### 加权BFS
在图中引入边权，使得每条边的权重表示该边的代价或距离。在遍历过程中，根据边权选择优先遍历代价小或距离近的邻接顶点。

### 迭代加深BFS
在BFS的基础上，加入深度限制，依次增加深度限制来遍历更深的层次。

### 01 BFS
用于求解0/1图中的最短路径问题，其中每条边的权值为0或1。

### 分层BFS
按照层次遍历图，记录每个顶点所在的层数，便于后续处理。

### 随机漫步BFS
在BFS的基础上，加入随机漫步，以一定的概率随机选择未访问的邻接顶点进行遍历，避免遍历过程中陷入死循环。

### 并行BFS
利用并行计算的优势，同时对多个起始顶点进行BFS，提高算法效率。