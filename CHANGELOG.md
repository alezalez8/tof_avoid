# Changelog

Формат — [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/), версии — [SemVer](https://semver.org/lang/ru/) (SW-10).
Версия прошивки задаётся в `core/include/core/version.h`.

## [0.1.0] — 2026-10-23 — E0 «Каркас»

### Добавлено

- Структура репозитория по CLAUDE.md §4 / разделу 11 ТЗ.
- CMake: корневой `CMakeLists.txt`, `CMakePresets.json` (схема v3) с пресетами `host-debug`, `host-coverage`,
  `target-check`; тулчейн Cortex-M4F (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`, `-Os`) из STM32CubeCLT.
- target-check печатает размер `libtof_core.a` и проверяет, что float-код использует FPU и нет double-хелперов.
- Сабмодули: `third_party/mavlink/c_library_v2` (ardupilotmega), `third_party/unity` (v2.7.0).
- `core/`: версия (semver + git-хеш, SW-10), заглушка `geometry` (хелперы углов).
- Юнит-тесты Unity + CTest: `test_version`, `test_geometry`.
- Скрипты `tools/scripts/`: `build_host.sh`, `test.sh`, `coverage.sh` (порог 90 %), `cppcheck.sh`,
  `build_target_check.ps1`; задачи VS Code в `.vscode/tasks.json`.
- Документы: `docs/traceability.md` (все ID ТЗ и T-00…T-27), шаблон в `docs/deviations.md`, `README.md`.
