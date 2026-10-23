# third_party/

Сторонний код без изменений (SW-04). Правки — только через обёртки вне этого каталога.

| Каталог | Источник | Версия |
| --- | --- | --- |
| `mavlink/c_library_v2` | git submodule, github.com/mavlink/c_library_v2, диалект `ardupilotmega` | commit в `git submodule status` |
| `unity` | git submodule, github.com/ThrowTheSwitch/Unity | тег v2.7.0 |
| `vl53l8cx_uld` | ST STSW-IMG040 (требует логина на st.com) | **отсутствует** — архив кладёт человек |

После клонирования: `git submodule update --init --recursive`.
