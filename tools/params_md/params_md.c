/* Host tool (SW-07a): prints the parameter table of core/include/core/params_table.h as Markdown.
 *   params_md                 -> table on stdout
 *   params_md --check FILE    -> exit 1 if FILE differs from the generated table
 */
#include <stdio.h>
#include <string.h>

#include "core/params.h"

#define OUT_MAX 32768

static const char *type_name(param_type_t t)
{
    switch (t) {
    case PARAM_TYPE_U8:
        return "u8";
    case PARAM_TYPE_U16:
        return "u16";
    case PARAM_TYPE_U32:
        return "u32";
    default:
        return "f32";
    }
}

static int fmt_value(char *dst, size_t n, const param_def_t *d, param_value_t v)
{
    if (d->type == PARAM_TYPE_F32) {
        return snprintf(dst, n, "%g", (double)v.f);
    }
    if (strstr(d->name, "MASK") != NULL || strcmp(d->name, "VALID_STATUS") == 0) {
        return snprintf(dst, n, "0x%X", (unsigned)v.u);
    }
    return snprintf(dst, n, "%u", (unsigned)v.u);
}

static size_t generate(char *out, size_t n)
{
    size_t len = 0u;
    len += (size_t)snprintf(out + len, n - len,
        "# Параметры модуля\n\n"
        "Генерируется из `core/include/core/params_table.h` утилитой `tools/params_md` "
        "(`tools/scripts/params_md.sh`); вручную не править.\n"
        "Пределы min/max — временные, на ревью человеком (ТЗ 5.6, SW-07a). Версия формата слота: %u.\n"
        "Умолчания `LCD_ENABLE` и `LOG_LEVEL` — для сборки DEBUG; `GROUND_POLICY` max = 1 только при "
        "`TOF_ALLOW_GROUND_DROP = 1` (MAP-07a).\n\n"
        "| # | Имя | Тип | Умолчание | Min | Max |\n"
        "| --- | --- | --- | --- | --- | --- |\n",
        (unsigned)PARAMS_FORMAT_VERSION);
    for (unsigned i = 0; i < (unsigned)PARAM_COUNT && len < n; i++) {
        const param_def_t *d = params_def((param_id_t)i);
        char def[24];
        char mn[24];
        char mx[24];
        (void)fmt_value(def, sizeof def, d, d->def);
        (void)fmt_value(mn, sizeof mn, d, d->min);
        (void)fmt_value(mx, sizeof mx, d, d->max);
        len += (size_t)snprintf(out + len, n - len, "| %u | `%s` | %s | %s | %s | %s |\n", i, d->name,
                                type_name(d->type), def, mn, mx);
    }
    return len;
}

int main(int argc, char **argv)
{
    static char out[OUT_MAX];
    const size_t len = generate(out, sizeof out);
    if (argc == 3 && strcmp(argv[1], "--check") == 0) {
        static char file[OUT_MAX];
        FILE *f = fopen(argv[2], "rb");
        if (f == NULL) {
            fprintf(stderr, "params_md: cannot open %s\n", argv[2]);
            return 1;
        }
        const size_t got = fread(file, 1u, sizeof file, f);
        (void)fclose(f);
        if (got != len || memcmp(file, out, len) != 0) {
            fprintf(stderr, "params_md: %s is out of date, run tools/scripts/params_md.sh\n", argv[2]);
            return 1;
        }
        return 0;
    }
    (void)fwrite(out, 1u, len, stdout);
    return 0;
}
