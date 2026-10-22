# P765 独立审查及一次修复收敛

Base 33a267f，审查未提交的P765源码/新文件；审查过程只读，未启动DDS或模拟器，未重复全套测试。

最初Important：ROI缓存可重复/跨reset使用；detector generation检查和publish不原子；runtime identity来自错误进程。另发现SESSION_END顺序、ROI仅清单存在即成功、逐核CPU采样缺失；E1多流缺少同run绑定。

修复：exact detector自身status身份；one-shot与per-streamrollback/epoch/receipt拒绝；发布锁内generation检查；末尾SESSION_END及postflight数据/hash/正常结束核验与失败退出码；逐CPU raw ticks；同parent/run_id与inputSHA绑定。均先定向重现，修复后复测。

最终审查原文：
“Confirmed my findings are addressed. No remaining Important defect found within the requested fix scope. Reviewed atomic publication, detector identity binding, ROI consumption/reset/chronology checks, terminal ordering, postflight failure exit, and E1 stream binding. git diff --check passes. No files changed or tests repeated.”

审查不能授予真实3D、实时性或动力学资格。后续唯一完整回归使用checks/source_frozen.json指纹；若源码变化必须重新标记验证失效。
