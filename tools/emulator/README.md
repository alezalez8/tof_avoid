# tools/emulator/ — E3, SITL

Проверка настоящего ядра в контуре с ArduCopter 4.7.2 SITL (раздел 13 ТЗ), а не его копии на Python.

```
scene.py ──UDP кадры зон (sil_proto)──▶ tof_sil (core/ + bsp_sim) ──TCP 5813 = SERIAL2──▶ ArduCopter SITL -I 5
    ▲                                                                                      │
    └──────────── поза: ATTITUDE, LOCAL_POSITION_NED (TCP 5812 = SERIAL1) ◀────────────────┤
pytest-сценарий (GCS, TCP 5810 = SERIAL0): режимы, миссии, RC, сбор логов ◀────────────────┘
```

| Файл | Что |
| --- | --- |
| `tof_sil/` | Host-исполняемый файл: `core/` без изменений + имитация платы (время, UART как TCP-клиент с темпом `UART_BAUD` и моделью DMA, кадры по UDP, слоты параметров в файле, переинициализация датчика по SNS-11). Лог — JSON Lines: карты (t1…t5, фаза, сектора), события, каждый исходящий пакет. Собирается пресетом `host-debug` |
| `sil_proto.py` | Формат UDP-пакетов (зеркало `tof_sil/sil_proto.h`) |
| `scene.py` | Сцена: плоскость земли, боксы, столбы, стены; 3 датчика 45° × 45°, 4×4, азимуты из параметров, оптика 0,20 м над землёй; 5×5 лучей на зону, до 2 целей на зону; шум, пропуски, засветка, задержка, отказ датчика, фантомы; поза из SITL на середину экспозиции. Команды во время прогона — JSON по UDP |
| `harness.py` | Стенд одного сценария: SITL (`-I 5`, `-w`, рабочий каталог `build/sitl_run/<сценарий>/`), модуль, сцена, тестовый GCS (tlog), чтение журналов FC |
| `scenarios/test_T*.py` | Сценарии pytest по тестам ТЗ; результаты — `build/sitl_run/results/*.json` |
| `make_reports.py` | Итоговая таблица и отчёты `docs/reports/T-xx_<дата>.md` (анализ — из `report_notes/T-xx.md`) |
| `run_all.sh` | Прогон сценариев по порядку, таблица PASS/FAIL, отчёты |
| `tests/test_scene.py` | Проверки сцены без SITL |

Запуск (WSL2, venv `~/venvs/tof_avoid`):

```
tools/scripts/build_host.sh
tools/emulator/run_all.sh            # все сценарии
tools/emulator/run_all.sh T26        # один
python -m pytest -q tools/emulator/tests
```

Дерево ArduPilot только читается: путь берётся из `TOF_ARDUPILOT` или по `sim_vehicle.py` в PATH. Параметры FC —
`params/sitl_tof.param` (тот же набор, что `params/fc_tof_v1.param`, плюс SITL), параметры модуля — `params/module_sitl.json`.
