# P5.3 F3八字实际验证

最新[全面修复与最终验收](p53_comprehensive_validation.md)：R11 F3已完成60s、MINCO100%、最长59.950s、零回退，完整目标FOV100%。以下保留历史轮次结论。

F2第3轮真实通过后才进入F3。首次F3在Planner启动前停止，未发生MINCO接管：
原FOLLOW初次接近时RGB可见、深度为DEPTH_RATIO_LOW，监视器误用锁定后200ms失锁规则。
原始失败轮 `data/experiments/20261013_p53_closed_loop_03/f3_h16_01/` 已封存。

修复仅涉及实验driver：首次原FOLLOW取得锁定前允许原有60s总收敛上限，未锁定显式不能
累计settling；实际锁定后仍按200ms短暂恢复观察，MINCO无宽限；其他健康/物理故障立即停止。
原harness首次FOLLOW后60s总上限和稳定5s门禁保留。

生产控制源码和全部飞行参数与已通过F2完全相同。新资格
`docs/current/audit/p53_f3_revalidation/` 已验证仅driver及其测试改变，完整回归1778通过、
1跳过，独立源码审查15项通过。此前四包构建对应的全部生产源SHA保持不变。

额外一次有界F3使用独立子目录 `data/experiments/20261013_p53_closed_loop_03/f3_retry_02/f3_h16_01/`，
要求同轮实际F2仍通过、原F3尝试已存在、当前资格通过以及新的排他claim；不覆盖原失败。
本次正在执行，尚无完整60s及20～60s稳态验收结论。
