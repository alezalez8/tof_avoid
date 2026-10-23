# Статус работы — 2026-10-23, внедрение уточнений ТЗ 5.6 (после E1a)

## Сделано

- `docs/TZ_3.1.md`: подраздел 5.6 вставлен дословно (коммит `5e1c1c9`).
- Код (не закоммичен до этого wip-коммита, тесты под 5.6 ещё не обновлены):
  - **MAP-13a** — `classify_zone` в `core/src/fusion.c`: при выключенном CLEAR_MEASURED CLR только от валидной цели дальше max,
    зона без цели — INV; при включённом CLR только при ambient ≤ AMBIENT_CLEAR_MAX.
  - **MAP-09a** — `fusion_run_staggered` (слияние на каждом кадре, последние кадры остальных не старше VALID_DATA_MAX_AGE_MS,
    t1 = время кадра-триггера, `fusion_t.t1_us`); вызов из `tof_core_push_frame` при `SCHEDULE = STAGGERED`.
    Для SIMULTANEOUS t1 = самый старый использованный кадр.
  - **MAP-05a** — `geo_zone_extent_at_range` + `mark_target`: протяжённость каждой цели в азимутах от центра корпуса на
    измеренной дальности с t_m; карта по секторам `fusion_frame_t.sec_occ[72]`.
  - **MAP-10a** — доля валидных зон только среди зон с целью (`quality_zone_t.has_target`); медиана ambient,
    новый параметр `AMBIENT_DEGRADED` (0 = выключен), `sh_on_frame(..., ambient_high, ...)` с тем же таймером DEGRADED_TIME_MS.
  - **MAP-14a** (ядро) — `geo_point_height` = h_body − z_L(p_B) с t_m; `geo_body_height_from_sensor`, `geo_body_height_from_rf`.
  - **MAP-07a** — флаг сборки `TOF_ALLOW_GROUND_DROP` (config.h, по умолчанию 0) = max у `GROUND_POLICY`: DROP отвергается
    при set/загрузке.
  - **SNS-12a** (ядро) — параметр `ORIENT_CONFIRMED` (0); `fusion_map_state_cfg` не даёт OK; `tof_core_t.orient_not_confirmed_event`.
  - `PARAMS_FORMAT_VERSION` = 2.

## Тесты — падают (ожидают поведение до 5.6, нужно обновить)

| Тест | Причина |
| --- | --- |
| `test_quality` `test_MAP03_zone_without_targets` | зона без цели теперь `zone_valid = false`, проверять `has_target` (MAP-10a) |
| `test_geometry` `test_MAP02_mount_offset_xyz` | высота теперь с t_m: ожидать 0,52 (MAP-14a) |
| `test_fusion` `test_MAP10_valid_fraction_of_frame` | пустые зоны не считаются: доля 0 из 4 → ожидать 0 (или добавить валидные зоны) |
| `test_fusion` `test_MAP13_map_default_release_by_clr_to_unknown` | пустые зоны теперь INV → нужен вариант с целью дальше max |
| `test_tof_core` `test_MAP09_core_one_map_per_frame_period`, `test_SNS11_core_one_sensor_stops` | MAP_OK недостижим при ORIENT_CONFIRMED = 0: в тестах ставить 1 или ожидать DEGRADED |

Также проверить, не сломаны ли косвенно другие тесты fusion из-за MAP-13a (`clr_requires_all_zones`, `row_mask`,
`high_ambient` используют включённый CLEAR — должны пройти).

## Не написано

- Тесты на каждое уточнение: MAP-13a (выкл./вкл., nb = 0, ambient), MAP-09a (STAGGERED: каждый кадр → карта, отбор по возрасту,
  t1), MAP-05a (смещение t_m сдвигает сектора на малой дальности, при t_m = 0 совпадает с направлениями), MAP-10a (доля, медиана,
  AMBIENT_DEGRADED), MAP-14a (h_body от датчика и от дальномера), MAP-07a (DROP отвергается; отдельная сборка теста с
  `-DTOF_ALLOW_GROUND_DROP=1`), SNS-12a (DEGRADED при 0, OK при 1, флаг события), SW-07a.
- **SW-07a**: генератор `docs/params.md` из таблицы (host-утилита + проверка актуальности в CTest).
- `docs/deviations.md`: закрыть DEV-02…DEV-11 со ссылкой на 5.6; `docs/traceability.md`: строки MAP-13a, MAP-09a, MAP-05a,
  MAP-10a, MAP-14a, MAP-07a, SNS-12a, SW-07a; `CHANGELOG.md`.
- cppcheck, покрытие, target-check после изменений не прогонялись.

## Остановился на

Код всех пунктов 5.6 в ядре написан (последний — `tof_core`); первый прогон тестов: 6 падений (таблица выше).

## Следующий шаг

1. Обновить 6 падающих тестов под 5.6.
2. Написать тесты на каждое уточнение (список выше), затем генератор `docs/params.md`.
3. cppcheck, покрытие ≥ 90 %, target-check; закрыть DEV, обновить traceability и CHANGELOG; коммит и push.
4. В E1b: h_body от дальномера через mavlink_rx (MAP-14a), STATUSTEXT «TOF ORIENT NOT CONFIRMED» через events (SNS-12a),
   экспорт DROP в фазе FLIGHT (MAP-07a / MAP-17).
