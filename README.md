# tof_avoid

Модуль обнаружения препятствий: STM32F4 + 3 × VL53L8CX (SPI) → один виртуальный proximity-сенсор
`OBSTACLE_DISTANCE` по MAVLink 2 → Cube Orange+ (ArduCopter 4.7.2, штатный BendyRuler).

- ТЗ: [docs/TZ_3.1.md](docs/TZ_3.1.md)
- Трассируемость: [docs/traceability.md](docs/traceability.md)
- Отклонения: [docs/deviations.md](docs/deviations.md)
- Правила для агента: [CLAUDE.md](CLAUDE.md)
- Окружение машин: [docs/env/](docs/env/)

## Состояние

Этап E0 (каркас). Модель платы не выбрана (`BOARD` не задан), поэтому target-сборка — только проверка
тулчейна (`target-check`): `core/` как статическая библиотека для Cortex-M4F.

## Начало работы

```sh
git clone https://github.com/alezalez8/tof_avoid.git
cd tof_avoid
git submodule update --init --recursive
```

Host (WSL2 Ubuntu: gcc, cmake ≥ 3.22, ninja, gcovr, cppcheck):

```sh
tools/scripts/build_host.sh   # сборка host-debug
tools/scripts/test.sh         # юнит-тесты (CTest)
tools/scripts/coverage.sh     # покрытие core/, порог 90 %
tools/scripts/cppcheck.sh     # статический анализ
```

Target-check (Windows, STM32CubeCLT в PATH):

```powershell
tools\scripts\build_target_check.ps1
```

Те же действия — задачи VS Code (`Terminal → Run Task`).

## Структура

| Каталог | Содержимое |
| --- | --- |
| `core/` | Ядро на C11 без HAL: вся логика, тестируется на ПК |
| `bsp/` | `bsp.h` и `bsp/<board>/` (после E2) |
| `drivers/` | platform-слой ULD, LCD |
| `third_party/` | ST ULD, MAVLink c_library_v2, Unity — без изменений |
| `app/` | `main.c`, сборки DEBUG/FLIGHT |
| `tests/host/` | Юнит-тесты ядра |
| `tools/` | Эмулятор для SITL, декодер логов, скрипты |
| `params/` | Параметры FC и умолчания модуля |
| `docs/` | ТЗ, окружение, трассируемость, отчёты |
