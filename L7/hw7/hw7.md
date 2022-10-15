## 2.Bundle Adjustment
**2.1 文献阅读**

*1.为何说Bundle Adjustment is slow是不对的？*
Such statements often appear in papers introducing yet another heuristic Structure from Motion (SFM) iteration. The claimed slowness is almost always due to the unthinking use of a general-purpose optimization routine that completely ignores the problem structure and sparseness. Real bundle routines are much more efficient than this, and usually considerably more efficient and flexible than the newly suggested mothod. That is why bundle adjustment remains the dominant structure refinement technique for real applications, after 40 years of research.
总结：忽略了问题本身的结构与稀疏性就可能导致"Bundle Adjustment is slow"

*2.BA中有哪些需要注意参数化的地方？Pose和Point各有哪些参数化方式？有何优缺点。*
