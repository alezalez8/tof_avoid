# Трассируемость: требования ТЗ 3.1 → код → тесты

Статусы: **todo** — не реализовано или не закрыто критерием ТЗ; **impl** — код есть, тесты по критерию ТЗ ещё не пройдены;
**pass** — критерий ТЗ выполнен (юнит-тесты и/или отчёт `docs/reports/T-xx_*.md`).
В колонке «Файл / функция» для E0 указаны заготовки каркаса; статус требования от них не меняется.

Для требований, у которых критерий ТЗ включает тесты HIL/стенда, статус остаётся **impl**, пока эти тесты не пройдены,
даже если host-часть (T-00) проходит.

Обновляется в том же коммите, что и код. Последнее обновление: 2026-10-23, этап E1a.

## SNS — сенсорная подсистема

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| SNS-01 | Официальный ST ULD (STSW-IMG040) без изменений, версия зафиксирована; свой только platform-слой | `third_party/vl53l8cx_uld` (отсутствует) | Сборка с неизменённым ULD | todo |
| SNS-02 | SPI: SPI_I2C_N = 1, CS на датчик, LPn = 1; топология шины — параметр сборки | — | T-02 | todo |
| SNS-03 | Блоб прошивки датчика во Flash один раз для трёх датчиков | — | Map-файл | todo |
| SNS-04 | Готовность трёх датчиков ≤ 3 с | — | T-03 | todo |
| SNS-05 | 4×4 (по умолчанию) / 8×8; частота — параметр | — | T-03 | todo |
| SNS-06 | Ближайшая цель, 2 цели на зону, только нужные выходы ULD | `core/include/core/frame.h` (`tof_zone_t`: 6 выходов SNS-06), `TOF_TARGETS_PER_ZONE` в `config.h`; `quality_zone_eval` (≤ 2 цели) | T-00: `test_MAP03_zone_*` — pass; размер кадра в логе — E4 | impl |
| SNS-07 | Расписание захвата SIMULTANEOUS / STAGGERED | — | T-04 | todo |
| SNS-08 | Готовность кадра по INT или опрос ≤ 5 мс | — | T-05 | todo |
| SNS-09 | Метаданные кадра: ID датчика, номер, время мкс, режим, частота | `core/include/core/frame.h` (`tof_frame_t`); `core/src/tof_core.c`: `tof_core_push_frame` (проверка режима/частоты, счётчик пропусков номеров) | T-00: `test_tof_core` `test_SNS09_*` — pass; T-05 — E5 | impl |
| SNS-10 | Таймаут SPI-транзакции ≤ 10 мс | — | Ревью + T-07 | todo |
| SNS-11 | Отказ датчика → переинициализация только его; ≤ 1 с; FAILED с повтором 10 с | `core/src/sensor_health.c`: `sh_on_error`, `sh_update`, `sh_on_reinit_done`, `sh_take_reinit_request` (решение в ядре; переинициализацию выполняет драйвер — E4/E5) | T-00: `test_sensor_health` `test_SNS11_*` — pass; T-07 — E5 | impl |
| SNS-12 | Ориентация матрицы (поворот/зеркало) на датчик — в конфигурации | `ORIENT_TRANSFORM_L/C/R` в `params_table.h`; `core/src/geometry.c`: `geo_orient_apply`, `geo_zone_from_raw` | T-00: `test_MAP02_orient_*` — pass; T-06 — E7 | impl |
| SNS-13 | Калибровка Xtalk, хранение во Flash | — | T-08 | todo |

## MAP — обработка и карта

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| MAP-01 | Системы координат и знаки по 5.1 | `core/src/geometry.c`: `geo_rot_x/y/z`, `geo_dir_sensor`, `geo_zone_alpha_deg`, `geo_zone_beta_deg` | T-00: `test_MAP01_*` — pass; T-06 — E7 | impl |
| MAP-02 | Направления зон и преобразования ZYX; угловая протяжённость зоны | `core/src/geometry.c`: `geo_mount_init` (Rz·Ry·Rx), `geo_sensor_to_body`, `geo_body_to_level`, `geo_zone_extent`, `geo_point_height`, `geo_orient_apply/inverse` | T-00: `test_geometry` `test_MAP02_*` — весь список критерия (yaw/pitch/roll ±, −θ/0/+θ, тангаж/крен ±15°, высоты 0,2/1/7 м, зеркалирование, перекрытие, XYZ, обратимость) — pass | pass |
| MAP-03 | Валидность цели: status ∈ VALID_STATUS, sigma, signal | `core/src/quality.c`: `quality_target_valid`, `quality_zone_eval` | T-00: `test_quality` `test_MAP03_*` — pass; T-10 — E7 | impl |
| MAP-04 | Горизонтальная дистанция и азимут в корпусной СК; ROW_MASK | `core/src/geometry.c`: `geo_horiz_dist`, `geo_azimuth_deg`, `geo_zone_target`; `core/src/fusion.c`: `classify_zone` (d с учётом t_m), `fusion_init` (ROW_MASK) | T-00: `test_MAP04_*`, `test_MAP02_three_sensors_zone_azimuths` — pass | pass |
| MAP-05 | Сетка 72 × 5°, зоны по протяжённости + CAL_TOL_DEG, без интерполяции | `core/src/fusion.c`: `fusion_init` (маски сектор→зоны), `step_sector` (правила перекрытия 5.3), `fusion_export` | T-00: `test_fusion` `test_MAP05_*` — pass | pass |
| MAP-06 | GROUND_SUSPECT по MAP-14 и MAP-18 | — | — | todo |
| MAP-07 | GROUND_POLICY только в FLIGHT; DROP запрещён до T-22, T-25, T-26 | — | Конфигурация + T-22 | todo |
| MAP-08 | Исключён, заменён MAP-13 | — | — | n/a |
| MAP-09 | Объединение кадров в FUSION_WINDOW_MS; время ячейки по самому старому вкладу | `core/src/fusion.c`: `fusion_ready`, `fusion_run`, `step_sector` (`cell_t_us`), `fusion_export_state` (STALE) | T-00: `test_fusion` `test_MAP09_*` — pass | pass |
| MAP-10 | Здоровье датчика OK / DEGRADED / FAILED; ячейки неисправного — INV | `core/src/sensor_health.c`: `sh_update` (health), `sh_on_frame`; `core/src/fusion.c`: `fusion_push_frame` (доля валидных зон), `fusion_run` (FAILED → INV) | T-00: `test_MAP10_*` в `test_sensor_health`, `test_fusion` — pass; T-07 — E5 | impl |
| MAP-11 | Состояние карты OK / DEGRADED / LOST | `core/src/fusion.c`: `fusion_map_state` | T-00: `test_MAP11_*` — pass | pass |
| MAP-12 | Задержки t1…t5 по p99 | — | T-11 | todo |
| MAP-13 | Автомат удержания и снятия препятствия (таблица 5.3) | `core/src/cell_fsm.c`: `cell_fsm_step` (`step_occ`, `step_clr`, `step_inv`), `cell_cfg_from_params` | T-00: `test_cell_fsm` `test_MAP13_*` — каждый переход таблицы 5.3 отдельным тестом — pass | pass |
| MAP-14 | «Луч — земля» по протяжённости зоны, GROUND_TOL_M | — | Юнит-тест + T-22, T-25 | todo |
| MAP-15 | Источник h_tof по фазам | — | Юнит-тест + T-26 | todo |
| MAP-16 | Автомат полётных фаз | — | Юнит-тест + T-26 | todo |
| MAP-17 | Политика экспорта по фазам | — | T-25, T-26 | todo |
| MAP-18 | Две цели в зоне классифицируются отдельно; земля — только если все цели совместимы | — | Юнит-тест + T-25 | todo |
| MAP-19 | Классификация земли не подтверждена до T-25 и T-26 | — | T-25, T-26 | todo |

## ICD — интерфейс MAVLink

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| ICD-01 | Кодирование полей по таблицам раздела 6 | `third_party/mavlink/c_library_v2` (ardupilotmega) подключена | Юнит-тест кодера + T-12 | todo |
| ICD-02 | Один OBSTACLE_DISTANCE на кадр карты | — | T-12 | todo |
| ICD-03 | Парсер устойчив к мусору, счётчик ошибок CRC | — | Юнит-тест (fuzz) | todo |
| ICD-04 | Команды от GCS не исполняются; COMMAND_LONG на compid 196 → UNSUPPORTED | — | Юнит-тест | todo |
| ICD-05 | UART через DMA без блокировки цикла | — | T-11 | todo |

## FLT — отказоустойчивость

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| FLT-01 | Обнаруживаемые отказы, обнаружение ≤ 300 мс | — | T-07, T-14 | todo |
| FLT-02 | При LOST — прекращение OBSTACLE_DISTANCE, HEARTBEAT CRITICAL | — | T-14 | todo |
| FLT-03 | Команда удержания DO_SET_MODE ≤ 100 мс, повтор 500 мс × 5; успех только по HEARTBEAT | — | T-15 | todo |
| FLT-04 | Эпизод отказа, не более одного успешного перевода, повторный AUTO пилотом | — | T-15 | todo |
| FLT-05 | Запреты на команды (disarmed, наземные фазы, ручные режимы, иные команды) | — | Юнит-тест + T-16 | todo |
| FLT-06 | «TOF HOLD REJECTED» при неисполнении перевода | — | T-15 | todo |
| FLT-07 | IWDG ≤ 500 мс, BOR, причина сброса | — | T-17 | todo |
| FLT-08 | Рестарт в полёте → удержание и «TOF REBOOTED IN FLIGHT» | — | T-17 | todo |
| FLT-09 | Рестарт до первой валидной карты ≤ 3 с | — | T-17 | todo |
| FLT-10 | «Виртуальная стена» FAULT_WALL (по умолчанию 0) | — | T-18 | todo |
| FLT-11 | Сбой подсистемы не останавливает остальные | — | T-07, T-11 | todo |

## FC — параметры ArduPilot

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| FC-01 | `params/fc_tof_v1.param` с версией прошивки FC | — | Ревью | todo |
| FC-02 | В ручных режимах препятствие не меняет управление | — | T-16 | todo |
| FC-03 | В AUTO/GUIDED/RTL — штатная реакция BendyRuler; неизвестные сектора и вне FoV проверены | — | T-13 | todo |

## SPD — скорость и тормозной путь

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| SPD-01 | Измерить R, T, a | — | T-10, T-11, T-20, T-23 | todo |
| SPD-02 | Рабочая скорость = min(2 м/с, решение условия) | — | Акт T-23 | todo |
| SPD-03 | Таблица «условия → R → скорость» в ограничениях | — | Ревью | todo |
| SPD-04 | Время обнаружения, смещение и СКО, доля валидных зон, пропуски | — | T-10, T-20 | todo |
| SPD-05 | Адаптивное ограничение скорости — вне V1 | — | — | n/a |

## DIA — диагностика

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| DIA-01 | LCD 4 строки: фаза, режим, связь, тревога; L/C/R | — | T-09 | todo |
| DIA-02 | LCD LCD_RATE_HZ порциями ≤ 2 мс, неблокирующий, отключаемый | — | T-11 | todo |
| DIA-03 | События STATUSTEXT (префикс «TOF», ≤ 50 символов) | — | T-19 | todo |
| DIA-04 | Правила STATUSTEXT: смена состояния, гистерезис, лимиты частоты | — | Юнит-тест + T-19, T-25 | todo |
| DIA-05 | Пороги WARN_CM / CRIT_CM — параметры | — | Конфигурация | todo |
| DIA-06 | Отладочный лог через UART/USB CDC с DMA | — | T-10, T-11 | todo |
| DIA-07 | Утилита декодирования лога и метрик на Python | `tools/logdecode/` (пусто) | Логи T-10, T-11 | todo |
| DIA-08 | Счётчики в лог раз в 1 с | — | T-11 | todo |
| DIA-09 | Отчёт при старте: версия, git-хеш, ID МК, BOARD, ULD, датчики, слот, сброс | `core/src/version.c` (версия и хеш) | T-17 | todo |

## SW — программная архитектура

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| SW-01 | Сборка CubeCLT + CMake + Ninja без IDE | `CMakeLists.txt`, `CMakePresets.json`, `cmake/arm-none-eabi-cortex-m4f.cmake`, `tools/scripts/build_target_check.ps1` (пока только `target-check`) | Сборка из командной строки | todo |
| SW-02 | core/: C11, без HAL, без malloc/рекурсии, только float; покрытие ≥ 90 % | `cmake/TofCompilerFlags.cmake` (`-Wdouble-promotion`, `-Wvla`), `cmake/check_target_lib.cmake` (нет double-хелперов), `tools/scripts/coverage.sh`; API ядра `core/src/tof_core.c` (время — аргументом) | Отчёт покрытия E1a: строки 99,1 %, ветви 96,4 % — pass | pass |
| SW-03 | `bsp.h` — единственная точка доступа к железу | — (E2) | Ревью | todo |
| SW-04 | ULD и MAVLink в third_party без изменений; platform-слой через bsp.h | `third_party/mavlink/c_library_v2` (submodule) | Diff с оригиналом | todo |
| SW-05 | Кооперативный суперцикл, худшая итерация ≤ 5 мс | — | T-11 | todo |
| SW-06 | Flash и RAM ≤ 80 % | — | Map-файл + T-11 | todo |
| SW-07 | Таблица параметров, слоты A/B с CRC-32 | `core/include/core/params_table.h` (таблица), `core/src/params.c`: `params_set_*`, `params_validate`, `params_store_load`, `params_store_save`; `core/src/crc32.c` | T-00: `test_params` (`test_SW07_*`, в т. ч. обрыв питания на каждом байте записи) — pass; T-17, T-27 — E6 | impl |
| SW-08 | Сборки DEBUG / FLIGHT; assert → лог и управляемый сброс | — | T-11 | todo |
| SW-09 | `-Wall -Wextra -Werror`, cppcheck без warning | `cmake/TofCompilerFlags.cmake`, `tools/scripts/cppcheck.sh` | Скрипт CI: host и target-check без предупреждений, cppcheck 2.7 без замечаний (E1a) — pass | pass |
| SW-10 | Semver + git-хеш в прошивке; CHANGELOG.md | `core/include/core/version.h`, `core/src/version.c`: `tof_version_string`, `tof_version_git_hash`, `tof_version_number`; `CHANGELOG.md` | T-00: `test_SW10_*` | todo |
| SW-11 | Точки расширения для будущего планировщика | — | Ревью | todo |

## HW — электрика и механика

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| HW-01 | Ресурсы платы: SPI, UART с DMA, отладочный UART, GPIO, SWD | `docs/pinout.md` (E2) | Ревью pinout | todo |
| HW-02 | Уровни Pololu, 5V-tolerant выводы | — | T-01 | todo |
| HW-03 | Общая шина SPI с CS или три SPI — по T-02 | — | T-02 | todo |
| HW-04 | Кабели ≤ 20 см, резисторы, SPI 2–3 МГц | — | T-02 | todo |
| HW-05 | Отдельный BEC 5 В для датчиков | — | T-01, T-24 | todo |
| HW-06 | Кронштейн, строго горизонтально, углы ±25/30/40° | — | T-06, T-21 | todo |
| HW-07 | Высота оптики и положения датчиков/дальномера в параметрах | — | Протокол сборки | todo |
| HW-08 | Окна без виньетирования; Xtalk при стекле | — | T-08 | todo |
| HW-09 | UART к FC: перекрёст, общая земля, 3,3 В | — | T-12 (HIL) | todo |
| HW-10 | SWD, тестовые точки, ключ питания датчика для T-07 | — | Наличие на макете | todo |
| HW-11 | Камера OV7725 не используется в полётной сборке | — | `docs/pinout.md` | todo |

## Тесты T-00…T-27

| ID | Уровень | Исполнитель | Этап | Проверяет | Отчёт | Статус |
| --- | --- | --- | --- | --- | --- | --- |
| T-00 | Host | Агент | E0 запуск, E1 приёмка | Юнит-тесты ядра, покрытие ≥ 90 % | — | todo |
| T-01 | Стенд | Человек | E7 | HW-02, HW-05 | — | todo |
| T-02 | HIL | Агент + человек | E5 | SNS-02, HW-03, HW-04 | — | todo |
| T-03 | HIL | Агент | E5 | SNS-04, SNS-05 | — | todo |
| T-04 | Стенд | Человек + агент | E7 | SNS-07 | — | todo |
| T-05 | HIL | Агент | E5 | SNS-08, SNS-09 | — | todo |
| T-06 | Стенд | Человек | E7 | MAP-01, SNS-12, HW-06 | — | todo |
| T-07 | HIL | Агент + человек | E5 | SNS-10, SNS-11, MAP-10, FLT-01, FLT-11 | — | todo |
| T-08 | Стенд | Человек | E7 | SNS-13, HW-08 | — | todo |
| T-09 | HIL | Человек | E7 | DIA-01 | — | todo |
| T-10 | Стенд | Человек + агент | E7 | MAP-03, SPD-01, SPD-04, DIA-06, H-13 | — | todo |
| T-11 | HIL | Агент | E6 | MAP-12, ICD-05, SW-05, SW-06, SW-08, DIA-02, DIA-06, DIA-08 | — | todo |
| T-12 | SITL, HIL | Агент | E3, E6 | ICD-01, ICD-02, HW-09, H-10 | — | todo |
| T-13 | SITL | Агент | E3 | FC-03, H-08 | — | todo |
| T-14 | SITL, HIL | Агент | E3, E6 | FLT-01, FLT-02 | — | todo |
| T-15 | SITL | Агент | E3 | FLT-03, FLT-04, FLT-06 | — | todo |
| T-16 | SITL | Агент | E3 | FLT-05, FC-02 | — | todo |
| T-17 | HIL | Агент | E6 | FLT-07, FLT-08, FLT-09, SW-07, DIA-09 | — | todo |
| T-18 | SITL | Агент | E3 | FLT-10, H-09 | — | todo |
| T-19 | SITL, HIL | Агент | E3, E6 | DIA-03, DIA-04 | — | todo |
| T-20 | Стенд | Человек + агент | E7 | SPD-01, SPD-04, H-05 | — | todo |
| T-21 | Стенд | Человек | E7 | HW-06, H-06 | — | todo |
| T-22 | Стенд | Человек + агент | E7 | MAP-07, MAP-14, H-07 | — | todo |
| T-23 | Полёт | Человек | E8 | SPD-01, SPD-02, H-12 | — | todo |
| T-24 | Стенд | Человек | E7 | HW-05 | — | todo |
| T-25 | HIL, стенд | Агент + человек | E6, E7 | MAP-14, MAP-17, MAP-18, MAP-19, DIA-04 | — | todo |
| T-26 | SITL, затем полёт | Агент, человек | E3, E8 | **Блокирующий.** MAP-15, MAP-16, MAP-17, MAP-19, H-14 | — | todo |
| T-27 | HIL | Агент | E6 | SW-07 | — | todo |
