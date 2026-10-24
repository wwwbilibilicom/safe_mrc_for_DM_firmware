# 开发文档

`dev_docs/` 用于保存项目迁移、构建、调试和协作过程中产生的工作文档。

目录约定：

- `standards/`：提交信息、代码协作和评审规范。
- `migration/`：Linux/CMake/CubeMX 迁移记录和验证结果。
- `notes/`：临时技术调查、实验记录和待办事项。

文档尽量记录背景、决策、验证命令和未完成的风险，避免只记录最终结论。

## 文档入口

- [提交信息规范](standards/commit-message.md)
- [提交消息模板](standards/commit-template.txt)
- [Linux/CMake 迁移状态](migration/linux-cmake.md)
- [原迁移设计](../docs/superpowers/specs/2026-10-24-linux-cmake-migration-design.md)
- [原迁移计划](../docs/superpowers/plans/2026-10-24-linux-cmake-migration.md)

原迁移设计与计划保留在现有路径；后续工作记录放入本目录对应分类。
