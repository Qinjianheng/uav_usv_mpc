# P7.6 候选诊断证据（未获飞行资格）

精确代码起点 944c684ef36389743e856bad19cdabb7868b4655。此包只选入静态几何、真实 YOLO 权重/训练来源、独立图像测试及无飞控 ROS 感知图数据，不是 A/B/C/D 飞行验收。

原始会话在 sources.json 逐文件列出；SHA256SUMS 校验本包。models/ 中的 ONNX 是当前合同模型的同字节恢复副本，可恢复到合同 model_file 指定路径。独立 CPU venv、完整数据集和训练中间件仍保存在忽略的 data/experiments/20261020_p76_*，不声称这些环境随 Git 传输。

主/备用布局均存在机体遮挡；当前 PX4 未导出实际推力反馈，未完成 Body-rate 校准。不得用此包批准飞行。

最初失败的pytest原始输出含终端尾随空格，以 verification/pytest_initial_failure.txt.gz 无损归档；使用 gzip -dc 可恢复原始字节。sources.json 同时记录原始和压缩文件SHA，未格式化原始失败证据。
