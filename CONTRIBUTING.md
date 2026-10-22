# 提交规范

骨架 <type>(<scope>): <描述>

feat 新增功能 / 新内容
fix 修 bug
refactor 重构，行为不变
docs 文档
test 测试
build 构建 / 环境 / 依赖
chore 杂项

type = 这次改动的**本质**（不看文件数 / 行数 / 耗时）；判据是读者会怎么用它：
feat·fix = 行为变了（要留意）｜refactor = 行为不该变（可跳过）｜docs·test·build·chore = 不影响功能。
"对外" = 别人能观察到的（库 = 公开头文件里的 API；固件 = 引脚 / 协议 / 时序 / 日志）。

怎么选（上一条为"否"才往下）：

1. 对外行为变了？ 新能力 = feat；纠正错误 = fix
2. 没变、动的是代码 / 结构 → refactor
3. 没变、也不是代码 → 测试 = test；文档 / 注释 = docs；构建 / 依赖 / 工具链 = build；其余 = chore

最容易搞错：删对外功能 = feat、删内部死代码 = refactor；只有"只改注释 / 文档"才用 docs；
测试暴露的产品代码修复 = fix；chore 是兜底，变多说明白名单缺项。

正文三段（除"纯 docs 单行"外必填；用 trailer 语法，因此可被 grep）：
Why: 改之前哪里不对 / 要解决什么问题（一句话）
Verified: 怎么证明没坏（命令 + 结果，例：frame_test.py 5/5）
Left: 已知未做 / 风险 / 下次入口（没有就写 none）

git log --grep='^Left:' # 列出全部悬空项 ← 复盘入口
git log --grep='^Why:' # 列出每个决定的动机

颗粒度（一个提交 = 一个语义单位）：
· `git revert <sha>` 之后工程仍能构建
· 同一条里不混"改名"和"改行为"
· 不许空改动：新建文件必须带最小有效内容
· >3 文件 或 >100 行 → 先自问能不能拆
（依据：Linux SubmittingPatches「一个提交只做一件事」）

工作流（每完成一个语义单位走一遍，不要攒到会话末尾）：
① 读规划文件的「下一步」→ 今天只做这一件
② 先写思路 → 编码
③ 验：0 warning + 相关验收脚本
④ 收：<leader>gs 看 diff → 只暂存属于这件事的 hunk → 提交（正文三段）
⑤ 记：把规划文件里那一行状态改掉（→ ✅ / 写进 Left）

可用scope：rx_dma main cubemx script settings readme deps cmake git style tx_uart_it timer servo

自检清单：
□ 只做了一件事吗？能单独回滚吗？
□ 标题能填进 "If applied, this commit will ___" 吗？
□ 冒号是半角 + 空格吗？没句号吗？
□ type 从白名单里选的吗？
□ scope 是一个名词吗？
□ 描述是动词开头吗？
□ git diff --cached 看过一遍了吗？

✅ feat(rx_dma): 新增半帧超时检测
❌ update(添加了超时检测功能)：超时阈值设置为100ms，只在空闲中断触发的...
