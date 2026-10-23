# Статус работы — 2026-10-23

## Готово: уточнения ТЗ 5.6 после E1a

Все пункты 5.6 реализованы в ядре и покрыты тестами; DEV-02…DEV-11 закрыты (`docs/deviations.md`), строки MAP-13a, MAP-09a,
MAP-05a, MAP-10a, MAP-14a, MAP-07a, SNS-12a, SW-07a — в `docs/traceability.md`.

- Тесты: 9 наборов, 171 тест + `test_SW07a_params_md_up_to_date` — pass; отдельная сборка `test_params_drop`
  с `TOF_ALLOW_GROUND_DROP = 1`.
- Покрытие `core/`: строки 99,3 %, ветви 96,0 %; cppcheck — без замечаний; target-check — собирается, double нет.

## Перенесено в E1b (по указанию человека)

- MAP-14a: выбор источника h_body по фазам и высота от дальномера через `mavlink_rx` (расчёт `geo_body_height_from_rf` готов).
- SNS-12a: STATUSTEXT «TOF ORIENT NOT CONFIRMED» через `events` (флаг `tof_core_t.orient_not_confirmed_event` готов).
- MAP-07a: экспорт DROP в фазе FLIGHT — вместе с MAP-17.
- MAP-13: запись в лог при истечении удержания; SW-07: событие «оба слота невалидны» — модуль `events`.

## Ожидает человека

- Ревью временных пределов параметров в `docs/params.md` (SW-07a).
- ORIENT_TRANSFORM по UM3109 и `ORIENT_CONFIRMED = 1` после T-06.
- Подтверждение дополнения DEV-01 (модули `crc32`, `sensor_health`, `tof_core`, `frame.h`, `config.h`).

## Следующий шаг

E1b: земля и фазы (MAP-14…MAP-19), MAVLink TX/RX (ICD), автомат отказов (FLT), события (DIA).
