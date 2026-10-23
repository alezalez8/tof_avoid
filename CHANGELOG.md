# Changelog

Формат — [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/), версии — [SemVer](https://semver.org/lang/ru/) (SW-10).
Версия прошивки задаётся в `core/include/core/version.h`.

## [Unreleased] — E1 «Ядро», часть a

### Добавлено

- `params` + `crc32` (SW-07): единая таблица параметров раздела 15 (имя, тип, умолчание, min, max), проверка диапазонов и
  межпараметрических ограничений, слоты A/B с заголовком (magic, версия формата, длина, порядковый номер, CRC схемы, CRC-32),
  запись в неактивный слот с проверкой чтением, только при disarmed; оба слота невалидны → безопасные умолчания и событие.
- `geometry` (MAP-01, MAP-02, MAP-04): углы зон по 5.1, монтаж Rz·Ry·Rx + t_m, выравнивание по ATTITUDE, d и ψ в корпусной СК,
  угловая протяжённость зоны, высота точки h_p, ORIENT_TRANSFORM (поворот/зеркало) с обратным преобразованием.
- `frame.h` (SNS-06, SNS-09): кадр датчика с метаданными и шестью выходами ULD; `quality` (MAP-03): валидность цели, до 2 целей на зону.
- `cell_fsm` (MAP-13): автомат удержания и снятия препятствия по таблице 5.3.
- `sensor_health` (SNS-11, MAP-10): решение о переинициализации одного датчика, FAILED с повтором раз в 10 с, OK/DEGRADED/FAILED.
- `fusion` (MAP-05, MAP-09, MAP-11): сетка 72 × 5° по протяжённости зон + CAL_TOL_DEG, правила перекрытия, окно слияния,
  время ячейки по самому старому вкладу, STALE, состояние карты, экспорт по 5.2.
- `tof_core`: API ядра — кадр датчика и `now_us` на входе, здоровье и карта на выходе.
- Host-тесты: `test_params` (мок NOR-flash с обрывом питания на каждом байте), `test_geometry` (весь список критерия MAP-02),
  `test_quality`, `test_cell_fsm` (каждый переход 5.3), `test_sensor_health`, `test_fusion`, `test_tof_core`.
- `docs/deviations.md`: DEV-02…DEV-10 — толкования неоднозначностей ТЗ, ожидают решения человека.

### Исправлено по независимой проверке

- Кадр с меткой времени чуть позже `now_us` больше не отбрасывается как опоздавший (MAP-09).
- Слияние не ждёт датчик в переинициализации (MAP-12).
- Кандидат CLR — по 5.3 независимо от включения CLEAR_MEASURED; при выключенном снятие идёт в UNKNOWN через 3 кадра (DEV-02).
- Отсчёт удержания не идёт назад от более старого кадра (MAP-13); протяжённость зон хранится в `fusion_t` (MAP-02).
- Сохранение параметров пересканирует слоты (T-27 при сбое чтения на старте); проверка ROW_MASK; REQ-теги; тесты MAP-02 дополнены.

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
