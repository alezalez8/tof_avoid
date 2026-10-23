# Окружение: DESKTOP-1UCIRT1

Зафиксировано на этапе S, 2026-10-23. Обновлять при изменении версий.

## Windows

| Элемент | Значение |
| --- | --- |
| ОС | Windows 11 Home 10.0.26200 |
| Пользователь | `DESKTOP-1UCIRT1\ALEX` (в группе администраторов, сессии без повышения) |
| winget | v1.29.380 (`%LOCALAPPDATA%\Microsoft\WindowsApps` — в конце пользовательского PATH) |
| VS Code | 1.143.1, профиль Default |
| Git | 2.54.0.windows.1, credential.helper = manager (GCM), system `core.autocrlf=true` (переопределяется `.gitattributes`) |
| Python | 3.14.0 (`%LOCALAPPDATA%\Programs\Python\Python314`) — для проекта не используется |

### STM32CubeCLT 1.22.0 (`C:\ST\STM32CubeCLT_1.22.0`) — первый в системном PATH

| Инструмент | Версия | Путь |
| --- | --- | --- |
| arm-none-eabi-gcc | 14.3.1 (GNU Tools for STM32 14.3.rel1.20251027-0700) | `GNU-tools-for-STM32\bin` |
| cmake | 4.3.1 | `CMake\bin` |
| ninja | 1.13.2 | `Ninja\bin` |
| make | из CubeCLT | `Make\bin` |
| STM32_Programmer_CLI | 2.23.0 | `STM32CubeProgrammer\bin` |
| ST-LINK_gdbserver | из CubeCLT | `STLink-gdb-server\bin` |
| stlink-server | 2.1.1-1 | `C:\Program Files (x86)\STMicroelectronics\stlink_server` |

Драйверы ST-Link (stlink_dbg_winusb, stlink_bridge_winusb) установлены; устройство на этапе S не подключалось.

Другие установленные тулчейны, не в PATH: STM32CubeIDE 1.18.0, STM32CubeMX 6.17.0,
STM32CubeProgrammer 2.22.0 (отдельно), Keil µVision, Atmel ARM Toolchain, Arduino IDE 2.3.10.
В PATH после CubeCLT есть `make.exe` Embarcadero — перекрыт CubeCLT.

### Расширения VS Code для проекта

| Расширение | Версия |
| --- | --- |
| anthropic.claude-code | 2.1.309 |
| ms-vscode.cpptools (единственный движок IntelliSense) | 1.34.4 |
| ms-vscode.cmake-tools | 1.24.42 |
| marus25.cortex-debug (+ mcu-debug.*) | 1.12.1 |
| ms-vscode-remote.remote-wsl | 0.104.5 |
| ms-python.python | 2026.10.0 |

Пользовательский `settings.json`: `cortex-debug.armToolchainPath`, `stlinkPath`, `stm32cubeprogrammer` указывают
на CubeCLT 1.22.0; `C_Cpp.default.configurationProvider = ms-vscode.cmake-tools`; `cmake.generator = Ninja`.

## WSL2

| Элемент | Значение |
| --- | --- |
| WSL | 3.0.3.0, ядро 6.18.54.1-microsoft-standard-WSL2, systemd включён |
| Дистрибутив | Ubuntu 22.04.3 LTS (по умолчанию), пользователь `aleks` |
| gcc | 11.4.0 (через ccache, `/usr/lib/ccache/gcc`) |
| cmake | 3.22.1 — **`CMakePresets.json` версии схемы ≤ 3** |
| ninja | 1.10.1 |
| gcovr | 5.0 |
| cppcheck | 2.7 |
| git | 2.34.1 |
| python3 | 3.10.12 |
| venv проекта | `~/venvs/tof_avoid` — pymavlink 2.4.50, pytest 9.1.1 |

Вызов: `wsl -e bash -lc "cd /mnt/d/stm32/tof_avoid && <команда>"`
или без жёсткого пути: `wsl --cd <путь репозитория в Windows> -e bash -l tools/scripts/<скрипт>.sh` (так делают задачи VS Code).

Версии перепроверены на этапе E0 (2026-10-23): gcc 11.4.0, cmake 3.22.1, ninja 1.10.1, gcovr 5.0, cppcheck 2.7,
python3 3.10.12 — без изменений. gcovr — системный (в venv проекта его нет); скрипты активируют venv, если он есть.

## Сборка (этап E0)

| Что | Где | Команда |
| --- | --- | --- |
| host-debug: сборка + тесты | WSL2 | `tools/scripts/build_host.sh`, `tools/scripts/test.sh` |
| host-coverage: отчёт и порог 90 % для `core/` | WSL2 | `tools/scripts/coverage.sh` → `build/host-coverage/coverage/index.html` |
| cppcheck | WSL2 | `tools/scripts/cppcheck.sh` |
| target-check: `core/` для Cortex-M4F, размер и проверка FPU | Windows, CubeCLT | `tools/scripts/build_target_check.ps1` |

Каталоги сборки: `build/<preset>` в корне репозитория (host — из WSL, target — из Windows; не смешивать в одном каталоге).
Тулчейн target-check — `cmake/arm-none-eabi-cortex-m4f.cmake`, инструменты берутся из PATH.

## third_party (этап E0)

| Компонент | Версия |
| --- | --- |
| mavlink/c_library_v2 | submodule, commit `6a776056` (2026-10-22, mavlink rev `eec88242`), диалект ardupilotmega, wire 2.0 |
| Unity | submodule, тег v2.7.0 (`7f1a686a`) |
| ST ULD (STSW-IMG040) | **нет** в `third_party/vl53l8cx_uld` — нужен архив от человека до E4 |

## ArduPilot SITL (только чтение)

| Элемент | Значение |
| --- | --- |
| Дерево | `/home/aleks/ardupilot-4.7` |
| Версия | тег `Copter-4.7.2`, commit `fb773567896014da007bca58358704d4a0d8516f`, detached HEAD, рабочее дерево и сабмодули чистые (проверено 2026-10-23) |
| Бинарник | `build/sitl/bin/arducopter` (собран 2026-10-23 15:20), `ArduCopter V4.7.2` |
| sim_vehicle.py | `~/ardupilot-4.7/Tools/autotest/sim_vehicle.py` (в PATH) |
| Другое дерево | `/home/aleks/ardupilot` — master 4.5.0-dev (2024), проектом не используется |

Правила: дерево не менять, не пересобирать, ветки не переключать без разрешения человека.

Изоляция SITL tof_avoid от других проектов (в т. ч. `claude_prog`): свой рабочий каталог внутри проекта
(`build/sitl_run/`, в `.gitignore`), свои параметры (`params/`), свой номер инстанса и порты.
Предварительно: `sim_vehicle.py -v ArduCopter -I 5 --use-dir=<проект>/build/sitl_run --no-rebuild`
(инстанс 5: базовые порты +50, TCP 5810). Окончательная команда фиксируется на этапе E3.

## Remote

`https://github.com/alezalez8/tof_avoid.git`, ветка `main`.
