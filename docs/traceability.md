# Трассируемость: требования ТЗ 3.1 → код → тесты

Статусы: **todo** — не реализовано или не закрыто критерием ТЗ; **impl** — код есть, тесты по критерию ТЗ ещё не пройдены;
**pass** — критерий ТЗ выполнен (юнит-тесты и/или отчёт `docs/reports/T-xx_*.md`).
В колонке «Файл / функция» для E0 указаны заготовки каркаса; статус требования от них не меняется.

Для требований, у которых критерий ТЗ включает тесты HIL/стенда, статус остаётся **impl**, пока эти тесты не пройдены,
даже если host-часть (T-00) проходит.

Обновляется в том же коммите, что и код. Последнее обновление: 2026-10-24, этап E3 (T-12, T-26; остановлен по DEV-14).

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
| SNS-12a | 5.6: ORIENT_CONFIRMED = 0 → карта не OK (максимум DEGRADED); STATUSTEXT «TOF ORIENT NOT CONFIRMED» при старте | `core/src/fusion.c`: `fusion_map_state_cfg`; `core/src/tof_core.c`: `orient_not_confirmed_event`; `core/src/scheduler.c`: `sched_init` → `EV_ONE_ORIENT`; текст — `core/src/events.c` | T-00: `test_SNS12a_*`, `test_DIA03_startup_messages` (текст в линии) — pass; T-06 — E7 | impl |
| SNS-13 | Калибровка Xtalk, хранение во Flash | — | T-08 | todo |

## MAP — обработка и карта

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| MAP-01 | Системы координат и знаки по 5.1 | `core/src/geometry.c`: `geo_rot_x/y/z`, `geo_dir_sensor`, `geo_zone_alpha_deg`, `geo_zone_beta_deg` | T-00: `test_MAP01_*` — pass; T-06 — E7 | impl |
| MAP-02 | Направления зон и преобразования ZYX; угловая протяжённость зоны | `core/src/geometry.c`: `geo_mount_init` (Rz·Ry·Rx), `geo_sensor_to_body`, `geo_body_to_level`, `geo_zone_extent`, `geo_point_height`, `geo_orient_apply/inverse` | T-00: `test_geometry` `test_MAP02_*` — весь список критерия (yaw/pitch/roll ±, −θ/0/+θ, тангаж/крен ±15°, высоты 0,2/1/7 м, зеркалирование, перекрытие, XYZ, обратимость) — pass | pass |
| MAP-03 | Валидность цели: status ∈ VALID_STATUS, sigma, signal | `core/src/quality.c`: `quality_target_valid`, `quality_zone_eval` | T-00: `test_quality` `test_MAP03_*` — pass; T-10 — E7 | impl |
| MAP-04 | Горизонтальная дистанция и азимут в корпусной СК; ROW_MASK | `core/src/geometry.c`: `geo_horiz_dist`, `geo_azimuth_deg`, `geo_zone_target`; `core/src/fusion.c`: `classify_zone` (d с учётом t_m), `fusion_init` (ROW_MASK) | T-00: `test_MAP04_*`, `test_MAP02_three_sensors_zone_azimuths` — pass | pass |
| MAP-05 | Сетка 72 × 5°, зоны по протяжённости + CAL_TOL_DEG, без интерполяции | `core/src/fusion.c`: `fusion_init` (маски сектор→зоны), `step_sector` (правила перекрытия 5.3), `fusion_export` | T-00: `test_fusion` `test_MAP05_*` — pass | pass |
| MAP-05a | 5.6: параллакс — точка с MOUNT_XYZ_M, протяжённость в азимутах от центра корпуса на измеренной дальности | `core/src/geometry.c`: `geo_zone_extent_at_range`; `core/src/fusion.c`: `mark_target`, `step_sector` | T-00: `test_MAP05a_*` (`test_geometry`, `test_fusion`) — pass | pass |
| MAP-06 | GROUND_SUSPECT по MAP-14 и MAP-18 | `core/src/ground.c`: `ground_target_compatible`; `core/src/fusion.c`: `classify_zone` (`fusion_zone_t.ground`, `n_ground_zones`), `cell_t.ground_suspect` | T-00: `test_ground` `test_MAP18_*`, `test_MAP14_*` — pass | impl |
| MAP-07 | GROUND_POLICY только в FLIGHT; DROP запрещён до T-22, T-25, T-26 | `core/src/phase.c`: `phase_export_ground` (DROP только в FLIGHT и при валидной высоте) | T-00: `test_MAP17_export_policy_table`, `test_phase_drop` `test_MAP07_drop_only_in_flight_with_height` — pass; T-22 — E7 | impl |
| MAP-07a | 5.6: DROP заблокирован флагом сборки TOF_ALLOW_GROUND_DROP (по умолчанию 0), без флага отвергается при загрузке | `core/include/core/config.h`, `params_table.h` (max GROUND_POLICY); `core/src/phase.c`: `phase_export_ground` (`#if TOF_ALLOW_GROUND_DROP`) | T-00: `test_MAP07a_drop_rejected_without_build_flag`, `test_params_drop`, `test_MAP07a_drop_inactive_without_build_flag`, `test_phase_drop` — pass | pass |
| MAP-08 | Исключён, заменён MAP-13 | — | — | n/a |
| MAP-09 | Объединение кадров в FUSION_WINDOW_MS; время ячейки по самому старому вкладу | `core/src/fusion.c`: `fusion_ready`, `fusion_run`, `step_sector` (`cell_t_us`), `fusion_export_state` (STALE) | T-00: `test_fusion` `test_MAP09_*` — pass | pass |
| MAP-09a | 5.6: STAGGERED — слияние на каждом кадре с кадрами остальных ≤ VALID_DATA_MAX_AGE_MS, t1 = время кадра-триггера; SIMULTANEOUS — по окну | `core/src/fusion.c`: `fusion_run_staggered`, `fusion_run` (`t1_us`); `core/src/tof_core.c`: `tof_core_push_frame` | T-00: `test_MAP09a_*` (`test_tof_core`) — pass; T-11 — E6 | impl |
| MAP-10 | Здоровье датчика OK / DEGRADED / FAILED; ячейки неисправного — INV | `core/src/sensor_health.c`: `sh_update` (health), `sh_on_frame`; `core/src/fusion.c`: `fusion_push_frame` (доля валидных зон), `fusion_run` (FAILED → INV) | T-00: `test_MAP10_*` в `test_sensor_health`, `test_fusion` — pass; T-07 — E5 | impl |
| MAP-10a | 5.6: доля валидных зон только среди зон с целью; DEGRADED по медиане ambient > AMBIENT_DEGRADED (0 — выкл.) | `core/src/quality.c` (`has_target`), `core/src/fusion.c`: `fusion_push_frame`, `ambient_median`; `core/src/sensor_health.c`: `sh_on_frame` | T-00: `test_MAP10a_*` — pass; T-20 — E7 | impl |
| MAP-11 | Состояние карты OK / DEGRADED / LOST | `core/src/fusion.c`: `fusion_map_state` | T-00: `test_MAP11_*` — pass | pass |
| MAP-12 | Задержки t1…t5 по p99 | `core/src/scheduler.c`: `on_new_map` (t1 — `fusion_t.t1_us`, t2 — `tof_core_t.t_push_us`, t3 — `fusion_t.t_map_us`), `sched_tx_take` (t4), `sched_tx_done` (t5), запись `sched_map_log_t` | T-00: `test_MAP12_t1_t5_stamps` (порядок отметок, полная запись) — pass; нормы p99 — T-11 (E6) | impl |
| MAP-13 | Автомат удержания и снятия препятствия (таблица 5.3) | `core/src/cell_fsm.c`: `cell_fsm_step` (`step_occ`, `step_clr`, `step_inv`), `cell_cfg_from_params`; запись в лог при истечении удержания: `fusion_t.hold_expired` → `core/src/scheduler.c`: `on_new_map` → `EV_LOG_HOLD_EXPIRED` (вид карты, сектор, время) | T-00: `test_cell_fsm` `test_MAP13_*` — каждый переход таблицы 5.3 — pass; `test_MAP13_hold_expiry_logged`, `test_MAP13_real_view_ground_does_not_release` — pass | pass |
| MAP-13a | 5.6: CLR при выкл. CLEAR_MEASURED — только от валидной цели дальше max; nb = 0 — INV; при вкл. — ambient ≤ AMBIENT_CLEAR_MAX | `core/src/fusion.c`: `classify_zone` | T-00: `test_MAP13a_*` (`test_fusion`) — pass | pass |
| MAP-14 | «Луч — земля» по протяжённости зоны, GROUND_TOL_M | `core/src/ground.c`: `ground_min_height` (точный максимум z_L по протяжённости + CAL_TOL_DEG), `ground_target_compatible`; `core/src/fusion.c`: `classify_zone` | T-00: `test_ground` `test_MAP14_*` (сверка с перебором, 20 см на земле, протяжённость вместо центрального луча, тангаж ±15°, крен, нет ориентации) — pass; T-22, T-25 — E6/E7 | impl |
| MAP-14a | 5.6: высоты от центра корпуса, h_p = h_body − z_L(p_B) с t_m; h_body по дальномеру с RF_XYZ_M, на земле — TOF_HEIGHT_GROUND_M с MOUNT_XYZ_M | `core/src/geometry.c`: `geo_point_height`, `geo_body_height_from_sensor`, `geo_body_height_from_rf`; `core/src/ground.c`: `ground_body_height` (источник по фазе); `core/src/scheduler.c`: `sched_push_frame` (дальномер из `mavlink_rx`, ATTITUDE на момент кадра) | T-00: `test_MAP14a_*`, `test_MAP15_body_height_values`, `test_MAP17_flight_keep_exports_ground` — pass | pass |
| MAP-14b | 5.7: CAL_TOL_DEG — только по азимуту, по углу места — CAL_TOL_EL_DEG (0,5°) | `params_table.h`: `CAL_TOL_EL_DEG` (формат слота 3); `core/src/ground.c`: `ground_min_height` (`half_a` / `half_b`); `core/src/scheduler.c`: `sched_push_frame` | T-00: `test_MAP14b_tolerances_by_axis`, `test_MAP14_extent_not_centre_ray`, `test_MAP14_min_height_matches_sampling` — pass; T-06, T-25 | impl |
| 5.1 (DEV-12 п. 6) | ATTITUDE интерполируется на середину экспозиции: t_ready − `EXPOSURE_OFFSET_MS` (0 = половина периода кадра) | `params_table.h`: `EXPOSURE_OFFSET_MS` (формат слота 4); `core/src/scheduler.c`: `sched_capture_time_us`, `sched_push_frame`, лог карты (`att_valid`, `pitch_rad`, `roll_rad`) | T-00: `test_MAP14_attitude_at_mid_exposure` — pass; уточнение смещения — T-05, T-22 | impl |
| MAP-15 | Источник h_tof по фазам | `core/src/ground.c`: `ground_height_source`, `ground_body_height`; `core/src/scheduler.c`: `sched_push_frame` (контекст земли кадра), `phase_height` (высота для автомата фаз) | T-00: `test_MAP15_*` — pass; T-26 SITL (`docs/reports/T-26_2026-10-24.md`): a, b, d, e — pass, c — **fail** (DEV-14) | impl |
| MAP-15b | 5.7: TAKEOFF / IN_AIR / LANDING без валидной высоты > 500 мс — земля не распознаётся, IN_AIR → FLIGHT, TOF_HEIGHT_GROUND_M в воздухе не используется | `core/src/mavlink_rx.c`: `mavrx_rf_valid` (последний валидный отсчёт не старше 500 мс); `core/src/ground.c`: `ground_height_source`; `core/src/phase.c`: `in_air`; `core/src/scheduler.c`: `phase_height` | T-00: `test_MAP15b_*` (`test_phase`, `test_scheduler`), `test_MAP15_height_source_by_phase`, `test_MAP15_rangefinder` — pass; T-26 SITL (`docs/reports/T-26_2026-10-24.md`): a, b, d, e — pass, c — **fail** (DEV-14): при дальномере ниже минимума на земле MAP-15b выключает распознавание в TAKEOFF и после касания; d — FLIGHT без высоты выше 7 м | impl |
| MAP-16 | Автомат полётных фаз | `core/src/phase.c`: `phase_update` (`next_known`, `in_air` — подтверждение 1 с с гистерезисом), `phase_init`; `core/src/scheduler.c`: `update_phase` (вход от `mavlink_rx`, сброс ячеек и эпизодов в GROUND_RECOVER) | T-00: `test_phase` `test_MAP16_*` — каждый переход; `test_MAP16_ground_recover_resets_cells` — pass; T-26 SITL a…e: последовательность фаз 5.4 и задержка ≤ 0,2 с от `landed_state` FC — pass во всех подпунктах; T-26 в целом не пройден (DEV-14) | impl |
| MAP-16a | 5.7: без гистерезиса по высоте; вход в FLIGHT — 1 с на PHASE_H_FLIGHT_M (или MAP-15b); выход — только LANDING / ON_GROUND / PHASE_UNKNOWN | `core/src/phase.c`: `next_known` (FLIGHT при LS = IN_AIR / TAKEOFF и при disarm в воздухе), `in_air` (перезапуск подтверждения при любом h < порога), `phase_update` (возврат из PHASE_UNKNOWN — только по правилу входа) | T-00: `test_MAP16a_*`, `test_MAP16_flight_confirmation_restarts_on_dip` — pass; T-26 SITL: вход в FLIGHT через 1,0–1,1 с после начала первого участка (замечание O-3 в DEV-14) | impl |
| MAP-17 | Политика экспорта по фазам | `core/src/phase.c`: `phase_export_ground`, `phase_hold_allowed`; `core/src/fusion.c`: два вида карты `FUSION_VIEW_ALL` / `FUSION_VIEW_REAL`, `fusion_export_view`; `core/src/scheduler.c`: `on_new_map` (выбор вида по фазе) | T-00: `test_MAP17_*` (`test_phase` — каждая строка таблицы 5.4; `test_ground` — реальное препятствие за землёй; `test_scheduler` — земля 65535 на стоянке, KEEP в FLIGHT, 65535 в LANDING) — pass; T-26 SITL (`docs/reports/T-26_2026-10-24.md`): a, b, d, e — pass, c — **fail** (DEV-14); T-25 — E6/E7 | impl |
| MAP-18 | Две цели в зоне классифицируются отдельно; земля — только если все цели совместимы | `core/src/fusion.c`: `classify_zone` (каждая цель — `ground_target_compatible`; `zone.ground`, `cls_real`), `mark_target` | T-00: `test_MAP18_*` (зона земли, смешанная зона «земля + низкая цель») — pass; T-25 — E6/E7 | impl |
| MAP-19 | Классификация земли не подтверждена до T-25 и T-26 | `core/include/core/ground.h` (статус в комментарии модуля); DROP заблокирован (MAP-07a) | T-00: `test_MAP19_low_object_near_ground_not_distinguished` (фиксирует ограничение модели); T-26 SITL (`docs/reports/T-26_2026-10-24.md`): a, b, d, e — pass, c — **fail** (DEV-14); T-25 — E6/E7 | impl |

## ICD — интерфейс MAVLink

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| ICD-01 | Кодирование полей по таблицам раздела 6 | `core/src/mavlink_tx.c`: `mavtx_obstacle_distance`, `mavtx_heartbeat`, `mavtx_statustext`, `mavtx_set_message_interval`, `mavtx_hold_mode`, `mavtx_named_value_int`, `mavtx_command_ack`; `core/src/mavlink_rx.c`: `dispatch` (HEARTBEAT FC, EXTENDED_SYS_STATE, ATTITUDE, DISTANCE_SENSOR 25, COMMAND_ACK) | T-00: `test_mavlink` `test_ICD01_*` (каждое поле OBSTACLE_DISTANCE, MAVLink 2 без флагов несовместимости) — pass; T-12 SITL a, b (`docs/reports/T-12_2026-10-24.md`) — pass | pass |
| ICD-02 | Один OBSTACLE_DISTANCE на кадр карты | `core/src/scheduler.c`: `on_new_map` (один пакет на объединённую карту, ограничение `MAV_DISTANCE_RATE_HZ` — маркерная корзина, в среднем) | T-00: `test_ICD02_one_message_per_map`, `test_ICD01_distance_rate_limit` — pass; T-12 SITL: 100 пакетов за 10 с, все уникальны и дошли до FC, 9,99 Гц — pass; T-12 HIL — E6 | impl |
| ICD-03 | Парсер устойчив к мусору, счётчик ошибок CRC | `core/src/mavlink_rx.c`: `mavrx_feed` (`mavlink_frame_char_buffer` со своими буферами, `stats.crc_errors`, `stats.parse_errors`) | T-00: `test_ICD03_*` (fuzz: мусор, мусор между пакетами с разрезанием, обрывы и битые биты, одиночная ошибка CRC) — pass | pass |
| ICD-04 | Команды от GCS не исполняются; COMMAND_LONG на compid 196 → UNSUPPORTED | `core/src/mavlink_rx.c`: `refuse`, `mavrx_take_nack` (COMMAND_LONG и COMMAND_INT); `core/src/mavlink_tx.c`: `mavtx_command_ack`; `core/src/scheduler.c`: `handle_rx` | T-00: `test_ICD04_commands_refused`, `test_ICD04_unsupported_on_wire` — pass | pass |
| ICD-05 | UART через DMA без блокировки цикла | — | T-11 | todo |

## FLT — отказоустойчивость

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| FLT-01 | Обнаруживаемые отказы, обнаружение ≤ 300 мс | `core/src/scheduler.c`: `detect_faults` (FAILED, DEGRADED, нет карты > 2 × VALID_DATA_MAX_AGE_MS, LOST, ATTITUDE/высота при DROP в FLIGHT, внутренняя ошибка: переполнение очереди TX, `sched_set_internal_fault`) | T-00: `test_FLT03_end_to_end_stale_map` (> 300 мс, команда на том же проходе), `test_FLT01_internal_fault` — pass; T-07, T-14 | impl |
| FLT-02 | При LOST — прекращение OBSTACLE_DISTANCE, HEARTBEAT CRITICAL | `core/src/scheduler.c`: `on_new_map` (LOST → без пакета), `periodic_tx` (CRITICAL при FAILED или LOST) | T-00: `test_FLT02_lost_stops_distance` — pass; T-14 — E3 | impl |
| FLT-03 | Команда удержания DO_SET_MODE ≤ 100 мс, повтор 500 мс × 5; успех только по HEARTBEAT | `core/src/fault_fsm.c`: `fault_update`, `run_series` (команда на том же шаге, что и обнаружение; 5 передач через 500 мс; успех — режим в HEARTBEAT после команды); `core/src/mavlink_tx.c`: `mavtx_hold_mode` | T-00: `test_fault_fsm` `test_FLT03_*` — pass; T-15 — E3 | impl |
| FLT-03a | 5.7: первая передача и до 5 повторов через 500 мс, всего не более 6 | `core/include/core/fault_fsm.h`: `FAULT_MAX_SENDS` = 6; `core/src/fault_fsm.c`: `run_series` | T-00: `test_FLT03a_retry_500ms_x5_then_rejected`, `test_FLT06_end_to_end_rejected` — pass; T-15 — E3 | impl |
| FLT-04 | Эпизод отказа, не более одного успешного перевода, повторный AUTO пилотом | `core/src/fault_fsm.c`: `fault_update` (эпизод до исчезновения отказа), `run_series` (SUCCEEDED / REJECTED / ABORTED — больше без команд, `FAULT_EVT_PILOT_RESUMED`), `fault_reset_episode` (GROUND_RECOVER) | T-00: `test_FLT04_*` — pass; T-15 — E3 | impl |
| FLT-05 | Запреты на команды (disarmed, наземные фазы, ручные режимы, иные команды) | `core/src/fault_fsm.c`: `run_series` (`allowed`), `fault_is_auto_mode`; `core/src/phase.c`: `phase_hold_allowed`; `core/src/mavlink_tx.c`: из команд строятся только SET_MESSAGE_INTERVAL и DO_SET_MODE в BRAKE/LOITER | T-00: `test_FLT05_*` — pass; T-16 — E3 | impl |
| FLT-06 | «TOF HOLD REJECTED» при неисполнении перевода | `core/src/fault_fsm.c`: `run_series` (`FAULT_EVT_HOLD_REJECTED` через 500 мс после 5-й передачи); текст — `events` | T-00: `test_FLT03_retry_500ms_x5_then_rejected` — pass; T-15 — E3 | impl |
| FLT-07 | IWDG ≤ 500 мс, BOR, причина сброса | — | T-17 | todo |
| FLT-08 | Рестарт в полёте → удержание и «TOF REBOOTED IN FLIGHT» | `core/src/mavlink_rx.c`: `first_hb_evt`; `core/src/fault_fsm.c`: `fault_update` (`reboot_latch`, `FAULT_ON_REBOOT`) | T-00: `test_FLT08_*` — pass; T-17 — E6 | impl |
| FLT-09 | Рестарт до первой валидной карты ≤ 3 с | — | T-17 | todo |
| FLT-10 | «Виртуальная стена» FAULT_WALL (по умолчанию 0) | `core/src/fault_fsm.c`: `fault_wall_apply` (сектора FAILED-датчика → min_distance); `params_table.h`: `FAULT_WALL` = 0 | T-00: `test_FLT10_virtual_wall` — pass; T-18 — E3 | impl |
| FLT-11 | Сбой подсистемы не останавливает остальные | — | T-07, T-11 | todo |

## FC — параметры ArduPilot

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| FC-01 | `params/fc_tof_v1.param` с версией прошивки FC | `params/fc_tof_v1.param` (ArduCopter V4.7.2, fb773567; имена сверены с 4.7.2, DEV-13), `params/sitl_tof.param` (тот же набор + SITL) | Ревью человеком; все имена существуют в 4.7.2 (проверка в SITL) | impl |
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
| DIA-03 | События STATUSTEXT (префикс «TOF», ≤ 50 символов) | `core/src/events.c`: `channel_text`, `oneshot_text` (WARN/CRIT/CLEAR, датчик L/C/R LOST/OK, DATA STALE, MAP LOST, HOLD SENT/REJECTED, REBOOTED IN FLIGHT, FC LINK LOST, PHASE UNKNOWN, PARAMS DEFAULT, READY с версией, ORIENT NOT CONFIRMED) | T-00: `test_events` `test_DIA03_*` — pass; T-19 — E3 | impl |
| DIA-04 | Правила STATUSTEXT: смена состояния, гистерезис, лимиты частоты | `core/src/events.c`: `ev_obstacle` (WARN_HYST_CM; в наземных фазах — только реальные препятствия), `ev_next_statustext` (смена состояния, дедупликация, ≤ 1/с на тип, ≤ 3/с всего, приоритет CRITICAL) | T-00: `test_DIA04_*` — pass; T-19, T-25 | impl |
| DIA-05 | Пороги WARN_CM / CRIT_CM — параметры | `params_table.h`: `WARN_CM` 250, `CRIT_CM` 120, `WARN_HYST_CM` 30; `core/src/events.c`: `ev_cfg_from_params` | T-00: `test_DIA05_thresholds` — pass | pass |
| DIA-06 | Отладочный лог через UART/USB CDC с DMA | Структуры записей: `core/include/core/scheduler.h` `sched_map_log_t` (карта, фаза, вид, ориентация, режим FC, t1…t5), `core/include/core/events.h` `ev_log_t` (удержание, COMMAND_ACK, фазы, отказы); переполнение — счётчик; транспорт и формат — после E2 | T-00: `test_DIA06_log_ring`, `test_MAP12_t1_t5_stamps` — pass; T-10, T-11 | todo |
| DIA-07 | Утилита декодирования лога и метрик на Python | `tools/logdecode/` (пусто) | Логи T-10, T-11 | todo |
| DIA-08 | Счётчики в лог раз в 1 с | — | T-11 | todo |
| DIA-09 | Отчёт при старте: версия, git-хеш, ID МК, BOARD, ULD, датчики, слот, сброс | `core/src/version.c` (версия и хеш) | T-17 | todo |

## SW — программная архитектура

| ID | Требование (кратко) | Файл / функция | Тест | Статус |
| --- | --- | --- | --- | --- |
| SW-01 | Сборка CubeCLT + CMake + Ninja без IDE | `CMakeLists.txt`, `CMakePresets.json`, `cmake/arm-none-eabi-cortex-m4f.cmake`, `tools/scripts/build_target_check.ps1` (пока только `target-check`) | Сборка из командной строки | todo |
| SW-02 | core/: C11, без HAL, без malloc/рекурсии, только float; покрытие ≥ 90 % | `cmake/TofCompilerFlags.cmake` (`-Wdouble-promotion`, `-Wvla`), `cmake/check_target_lib.cmake` (нет double-хелперов), `tools/scripts/coverage.sh`; API ядра `core/src/tof_core.c` (время — аргументом) | Отчёт покрытия E1a + 5.6: строки 99,3 %, ветви 96,0 % — pass | pass |
| SW-03 | `bsp.h` — единственная точка доступа к железу | — (E2) | Ревью | todo |
| SW-04 | ULD и MAVLink в third_party без изменений; platform-слой через bsp.h | `third_party/mavlink/c_library_v2` (submodule) | Diff с оригиналом | todo |
| SW-05 | Кооперативный суперцикл, худшая итерация ≤ 5 мс | `core/src/scheduler.c`: `sched_step` (один проход без ожиданий, время аргументом), очередь TX без блокировки (`enqueue`, `sched_tx_take`, `sched_tx_done`); суперцикл `app/` — после E2 | T-11 | todo |
| SW-06 | Flash и RAM ≤ 80 % | — | Map-файл + T-11 | todo |
| SW-07 | Таблица параметров, слоты A/B с CRC-32 | `core/include/core/params_table.h` (таблица), `core/src/params.c`: `params_set_*`, `params_validate`, `params_store_load`, `params_store_save`; `core/src/crc32.c`; событие умолчаний: `sched_init(…, params_defaults)` → «TOF PARAMS DEFAULT» | T-00: `test_params` (`test_SW07_*`, в т. ч. обрыв питания на каждом байте записи), `test_DIA03_startup_messages` — pass; T-17, T-27 — E6 | impl |
| SW-07a | 5.6: пределы min/max временные, таблица в docs/params.md на ревью | `tools/params_md/params_md.c`, `tools/scripts/params_md.sh`, `docs/params.md` | CTest `test_SW07a_params_md_up_to_date` — pass; ревью человеком — ожидается | impl |
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
| T-00 | Host | Агент | E0 запуск, E1 приёмка | Юнит-тесты ядра, покрытие ≥ 90 % | E1a + 5.6 + E1b + 5.7: 16 наборов Unity, 289 тестов + проверка `docs/params.md` — pass; покрытие `core/` строк 99,0 %, ветвей 93,8 %; cppcheck чист; target-check собирается | pass |
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
| T-12 | SITL, HIL | Агент | E3, E6 | ICD-01, ICD-02, HW-09, H-10 | `docs/reports/T-12_2026-10-24.md`: SITL a, b — pass; H-10 подтверждена (PreArm PRX1 No Data) | SITL pass, HIL todo |
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
| T-26 | SITL, затем полёт | Агент, человек | E3, E8 | **Блокирующий.** MAP-15, MAP-16, MAP-17, MAP-19, H-14 | `docs/reports/T-26_2026-10-24.md`: a, b, d, e — pass; c (дальномер ниже минимума на земле) — fail; DEV-14 | **SITL fail** |
| T-27 | HIL | Агент | E6 | SW-07 | — | todo |
