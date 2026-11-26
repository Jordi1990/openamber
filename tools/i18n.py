#!/usr/bin/env python3
"""Display translations for the OpenAmber LVGL UI.

The UI is written in Dutch. One build contains all languages (nl, de, en); the display language
is a select entity ("Display taal") that can be changed at runtime. `ui_language` (nl, de or en)
only sets the language compiled into the widgets and the default of that select.

* YAML texts (`text:`, `title:`, `placeholder_text:`, dropdown `options:`) use substitutions,
  for example `text: "${t_algemeen}"`. src/openamber/ui/i18n/<lang>.yaml defines them.
* Texts set from C++ (lambdas and the UI headers) go through `oa_tr("Dutch text")`, which looks
  the text up in src/openamber/ui/i18n/oa_i18n.h and returns it unchanged when there is no
  translation. Values of selects and other entities are never translated.
* On a language change oa_i18n_apply() walks the LVGL tree and swaps label, dropdown, table and
  placeholder texts (see src/openamber/ui/ui_language.yaml).

src/openamber/ui/i18n/translations.yaml is the single source; the language files are generated.

Usage:
  tools/i18n.py generate   write i18n/<lang>.yaml and oa_i18n.h from translations.yaml
  tools/i18n.py convert    replace Dutch literals that have a translation (YAML -> ${t_...},
                           lambdas -> oa_tr(...)) and add missing keys to translations.yaml
  tools/i18n.py check      exit 1 when generated files are stale or a translated literal is unconverted
"""
import os, re, sys, json, unicodedata

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src', 'openamber')
I18N = os.path.join(SRC, 'ui', 'i18n')
TRANSLATIONS = os.path.join(I18N, 'translations.yaml')
LANGS = ['nl', 'de', 'en']
SCAN_DIRS = [os.path.join(SRC, 'ui'), os.path.join(SRC, 'ble')]

# ---------------------------------------------------------------- translations.yaml (small, fixed format)

def _yq(s):
    return json.dumps(s, ensure_ascii=False)

def load_translations():
    entries, cur = [], None
    if not os.path.exists(TRANSLATIONS):
        return entries
    for line in open(TRANSLATIONS, encoding='utf-8'):
        line = line.rstrip('\n')
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        m = re.match(r'^(- )?\s*(key|use|nl|de|en):\s*(.*)$', line)
        if not m:
            raise SystemExit(f'translations.yaml: cannot parse line: {line}')
        if m.group(1):
            cur = {}
            entries.append(cur)
        val = m.group(3).strip()
        cur[m.group(2)] = json.loads(val) if val.startswith('"') else val
    return entries

def save_translations(entries):
    out = ['# Display texts of the OpenAmber UI. nl is the text in the YAML/C++ source.',
           '# key: substitution name for texts used in YAML (text/title/options); texts without a key',
           '# are only used from C++ via oa_tr(). use: state marks entity states that the UI passes to',
           '# oa_tr() at runtime (for example the defrost phase); they are kept although no literal uses them.',
           '# Run tools/i18n.py generate after editing.', '']
    for e in entries:
        first = True
        for f in ['key', 'use', 'nl', 'de', 'en']:
            if f not in e:
                continue
            v = e[f] if f in ('key', 'use') else _yq(e[f])
            out.append(('- ' if first else '  ') + f'{f}: {v}')
            first = False
    open(TRANSLATIONS, 'w', encoding='utf-8').write('\n'.join(out) + '\n')

# ---------------------------------------------------------------- generation

def c_lit(s):
    return '"' + s.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'

LANG_NAMES = {'nl': 'Nederlands', 'de': 'Deutsch', 'en': 'English'}

H_RUNTIME = r'''
static int oa_i18n_lang_ = 0;       // selected display language: 0 nl, 1 de, 2 en
static int oa_i18n_prev_lang_ = 0;  // language the widgets currently show

static inline const char *oa_i18n_str_(uint16_t e) { return OA_I18N_ROWS[e >> 2][e & 3]; }

// Row of a display text in any language (-1 if unknown). Prefers a match in column `pref`.
static int oa_i18n_row(const char *text, int pref) {
  if (text == nullptr || *text == '\0') return -1;
  size_t lo = 0, hi = OA_I18N_INDEX_COUNT;
  while (lo < hi) {
    size_t mid = (lo + hi) / 2;
    if (strcmp(oa_i18n_str_(OA_I18N_INDEX[mid]), text) < 0) lo = mid + 1; else hi = mid;
  }
  int row = -1;
  for (size_t i = lo; i < OA_I18N_INDEX_COUNT && strcmp(oa_i18n_str_(OA_I18N_INDEX[i]), text) == 0; i++) {
    if (row < 0) row = OA_I18N_INDEX[i] >> 2;
    if ((OA_I18N_INDEX[i] & 3) == pref) return OA_I18N_INDEX[i] >> 2;
  }
  return row;
}

// Translation of a Dutch display text into the selected language, or the text itself.
static const char *oa_tr(const char *text) {
  if (oa_i18n_lang_ == 0) return text;
  int row = oa_i18n_row(text, 0);
  return row < 0 ? text : OA_I18N_ROWS[row][oa_i18n_lang_];
}

// Translation of a display text in any language into the selected language, or nullptr.
static const char *oa_i18n_to(const char *text) {
  int row = oa_i18n_row(text, oa_i18n_prev_lang_);
  return row < 0 ? nullptr : OA_I18N_ROWS[row][oa_i18n_lang_];
}

static void oa_i18n_set_lang(int lang) {
  if (lang >= 0 && lang < 3) oa_i18n_lang_ = lang;
}

#ifdef USE_LVGL
// Translates labels, dropdown options, table cells and textarea placeholders below `obj`.
static void oa_i18n_apply_obj_(lv_obj_t *obj) {
  if (obj == nullptr) return;
  if (lv_obj_check_type(obj, &lv_label_class)) {
    const char *cur = lv_label_get_text(obj);
    const char *t = oa_i18n_to(cur);
    if (t != nullptr && strcmp(t, cur) != 0) lv_label_set_text(obj, t);
  } else if (lv_obj_check_type(obj, &lv_dropdown_class)) {
    const char *opts = lv_dropdown_get_options(obj);
    if (opts != nullptr) {
      std::string out, line;
      bool changed = false;
      for (const char *p = opts;; p++) {
        if (*p == '\n' || *p == '\0') {
          const char *t = oa_i18n_to(line.c_str());
          if (t != nullptr && line != t) { out += t; changed = true; } else out += line;
          line.clear();
          if (*p == '\0') break;
          out += '\n';
        } else {
          line += *p;
        }
      }
      if (changed) {
        uint32_t sel = lv_dropdown_get_selected(obj);
        lv_dropdown_set_options(obj, out.c_str());
        lv_dropdown_set_selected(obj, sel);
      }
    }
  } else if (lv_obj_check_type(obj, &lv_table_class)) {
    for (uint32_t r = 0; r < lv_table_get_row_count(obj); r++)
      for (uint32_t c = 0; c < lv_table_get_column_count(obj); c++) {
        const char *cur = lv_table_get_cell_value(obj, r, c);
        const char *t = oa_i18n_to(cur);
        if (t != nullptr && strcmp(t, cur) != 0) lv_table_set_cell_value(obj, r, c, t);
      }
  } else if (lv_obj_check_type(obj, &lv_textarea_class)) {
    const char *cur = lv_textarea_get_placeholder_text(obj);
    const char *t = oa_i18n_to(cur);
    if (t != nullptr && strcmp(t, cur) != 0) lv_textarea_set_placeholder_text(obj, t);
    return;  // the text itself is user input
  }
  for (uint32_t i = 0; i < lv_obj_get_child_count(obj); i++) oa_i18n_apply_obj_(lv_obj_get_child(obj, i));
}

// Switches every widget below the roots to the selected language.
static void oa_i18n_apply(std::initializer_list<lv_obj_t *> roots) {
  for (lv_obj_t *root : roots) oa_i18n_apply_obj_(root);
  oa_i18n_prev_lang_ = oa_i18n_lang_;
}
#endif
'''

def gen_files(entries):
    files = {}
    for lang in LANGS:
        y = [f'# Generated by tools/i18n.py from translations.yaml - do not edit.',
             f'# Texts compiled into the widgets for ui_language: {lang}. The display language can be',
             '# changed at runtime (select "Display taal"); see ui_language.yaml.',
             'substitutions:', f'  ui_language_option: "{LANG_NAMES[lang]}"']
        for e in entries:
            if e.get('key'):
                y.append(f"  {e['key']}: {_yq(e[lang])}")
        files[os.path.join(I18N, f'{lang}.yaml')] = '\n'.join(y) + '\n'
    # one table with all languages, plus an index over every string sorted bytewise for binary search
    pairs = []
    for r, e in enumerate(entries):
        seen = set()
        for c, lang in enumerate(LANGS):
            if e[lang] not in seen:
                seen.add(e[lang]); pairs.append((e[lang].encode('utf-8'), r * 4 + c))
    pairs.sort()
    h = ['// Generated by tools/i18n.py from translations.yaml - do not edit.',
         '// Display texts in all languages (nl, de, en); the language is selected at runtime.',
         '#pragma once', '#include <cstdint>', '#include <cstring>', '#include <string>',
         '#include <initializer_list>', '',
         'static const char *const OA_I18N_ROWS[][3] = {']
    for e in entries:
        h.append('    {' + ', '.join(c_lit(e[l]) for l in LANGS) + '},')
    h += ['};', '', 'static const uint16_t OA_I18N_INDEX[] = {']
    for i in range(0, len(pairs), 12):
        h.append('    ' + ', '.join(str(v) for _, v in pairs[i:i + 12]) + ',')
    h += ['};', f'static const size_t OA_I18N_INDEX_COUNT = {len(pairs)};']
    files[os.path.join(I18N, 'oa_i18n.h')] = '\n'.join(h) + '\n' + H_RUNTIME
    return files

# ---------------------------------------------------------------- scanning

Y_KV = re.compile(r'(?<![A-Za-z_])(text|title|placeholder_text)\s*:\s*("(?:[^"\\]|\\.)*"|\'(?:[^\']|\'\')*\')')
Y_ITEM = re.compile(r'^(\s*)-\s+("(?:[^"\\]|\\.)*"|\'(?:[^\']|\'\')*\')\s*(#.*)?$')
Y_ROW = re.compile(r'^\s*-\s*\[(.*)\]\s*$')   # table rows: - ["Label", "-"]
Y_QUOTED = re.compile(r'"(?:[^"\\]|\\.)*"')
C_LIT = re.compile(r'"((?:[^"\\]|\\.)*)"')
# Lines where literals are entity values (select options, state machine states) or logging.
C_SKIP_LINE = re.compile(r'set_option\(|current_option\(\)|\.state\s*[!=]=|ESP_LOG|logger\.log|opts\[\]|state_machine_state')
OA_TR_CALL = re.compile(r'oa_tr\(\s*"((?:[^"\\]|\\.)*)"\s*\)')

def yaml_unescape(q):
    if q[0] == "'":
        return q[1:-1].replace("''", "'")
    return json.loads(q) if '\\U' not in q else q[1:-1]

def c_unescape(raw):
    return re.sub(r'\\(.)', lambda m: {'n': '\n', 't': '\t'}.get(m.group(1), m.group(1)), raw)

def yaml_files():
    out = []
    for d in SCAN_DIRS:
        for dp, _, fs in os.walk(d):
            if os.path.abspath(dp).startswith(I18N):
                continue
            out += [os.path.join(dp, f) for f in fs if f.endswith('.yaml')]
    return sorted(out)

def header_files():
    out = []
    for d in SCAN_DIRS:
        for dp, _, fs in os.walk(d):
            if not os.path.abspath(dp).startswith(I18N):
                out += [os.path.join(dp, f) for f in fs if f.endswith('.h')]
    return sorted(out)

def _entity_block(lines, i, ind):
    for j in range(i - 1, -1, -1):
        l = lines[j]
        if not l.strip():
            continue
        jind = len(l) - len(l.lstrip())
        if jind < ind - 2 or (jind < ind and not l.lstrip().startswith('- ')):
            return False
        if 'platform:' in l:
            return True
    return False

def scan(lines):
    """Yield (line, start, end, text, ctx) for literals; ctx 'y' = YAML scalar, 'c' = C++ literal."""
    in_lambda, lam_ind, opt_ind = False, 0, None
    for i, l in enumerate(lines):
        st = l.strip()
        ind = len(l) - len(l.lstrip())
        if in_lambda:
            if st and ind <= lam_ind:
                in_lambda = False
            else:
                if not C_SKIP_LINE.search(l):
                    for m in C_LIT.finditer(l):
                        if not l[:m.start()].endswith('oa_tr('):
                            yield i, m.start(), m.end(), c_unescape(m.group(1)), 'c'
                continue
        if re.search(r':\s*(!lambda\s*)?[|>]-?\s*$', l):
            in_lambda, lam_ind = True, ind
            continue
        m = re.search(r'(?:!lambda|lambda:)\s+([\'"])(.*)\1\s*$', l)
        if m:
            if not C_SKIP_LINE.search(l):
                for mm in C_LIT.finditer(m.group(2)):
                    if m.group(2)[:mm.start()].endswith('oa_tr('):
                        continue
                    yield i, m.start(2) + mm.start(), m.start(2) + mm.end(), c_unescape(mm.group(1)), 'c'
            continue
        if re.match(r'^\s*-?\s*options\s*:\s*$', l):
            # Only LVGL dropdown/roller options are display texts; options of entities (select
            # platform) are values that Home Assistant and the controllers use.
            opt_ind = None if _entity_block(lines, i, ind) else ind
            continue
        if opt_ind is not None and st and ind <= opt_ind and not st.startswith('- '):
            opt_ind = None
        mr = Y_ROW.match(l)
        if mr:
            for mm in Y_QUOTED.finditer(l, mr.start(1), mr.end(1)):
                yield i, mm.start(), mm.end(), yaml_unescape(mm.group(0)), 'y'
            continue
        mi = Y_ITEM.match(l)
        if mi and opt_ind is not None:
            q = mi.group(2)
            s = l.index(q)
            yield i, s, s + len(q), yaml_unescape(q), 'y'
            continue
        for m in Y_KV.finditer(l):
            q = m.group(2)
            yield i, m.start(2), m.end(2), yaml_unescape(q), 'y'

def slug(s):
    s = unicodedata.normalize('NFKD', s).encode('ascii', 'ignore').decode().lower()
    s = re.sub(r'[^a-z0-9]+', '_', s).strip('_')
    return 't_' + (s[:40].rstrip('_') or 'x')

# ---------------------------------------------------------------- commands

def convert(dictionary=None):
    entries = load_translations()
    by_nl = {e['nl']: e for e in entries}
    if dictionary:
        for nl, (de, en) in dictionary.items():
            by_nl.setdefault(nl, {'nl': nl, 'de': de, 'en': en})
    keys = {e['key'] for e in by_nl.values() if e.get('key')}
    used = set()
    changed = 0
    for path in yaml_files():
        text = open(path, encoding='utf-8-sig').read()
        lines = text.split('\n')
        occ = list(scan(lines))
        for i, s, e, t, ctx in sorted(occ, key=lambda o: (o[0], -o[1])):
            ent = by_nl.get(t)
            if not ent:
                continue
            used.add(t)
            if ctx == 'y':
                if not ent.get('key'):
                    k = slug(t)
                    n = 2
                    while k in keys:
                        k = f'{slug(t)}_{n}'; n += 1
                    ent['key'] = k
                    keys.add(k)
                lines[i] = lines[i][:s] + '"${' + ent['key'] + '}"' + lines[i][e:]
            else:
                lines[i] = lines[i][:s] + 'oa_tr(' + lines[i][s:e] + ')' + lines[i][e:]
            changed += 1
        new = '\n'.join(lines)
        if new != text:
            open(path, 'w', encoding='utf-8').write(new)
    # keep the entries that are used: YAML substitutions and oa_tr("...") calls in YAML and headers
    used_keys = set()
    for path in yaml_files() + header_files():
        content = open(path, encoding='utf-8-sig').read()
        for m in OA_TR_CALL.finditer(content):
            used.add(c_unescape(m.group(1)))
        used_keys.update(re.findall(r'\$\{(t_\w+)\}', content))
    used.update(k for k, v in by_nl.items() if v.get('key') in used_keys or v.get('use') == 'state')
    save_translations([v for k, v in by_nl.items() if k in used])
    print(f'converted {changed} literals')

def generate():
    for path, content in gen_files(load_translations()).items():
        open(path, 'w', encoding='utf-8').write(content)
        print('wrote', os.path.relpath(path, ROOT))

def check():
    ok = True
    for path, content in gen_files(load_translations()).items():
        if not os.path.exists(path) or open(path, encoding='utf-8').read() != content:
            print('stale:', os.path.relpath(path, ROOT)); ok = False
    entries = load_translations()
    by_nl = {e['nl'] for e in entries}
    for path in yaml_files():
        lines = open(path, encoding='utf-8-sig').read().split('\n')
        for i, s, e, t, ctx in scan(lines):
            if t in by_nl and '${' not in lines[i][s:e]:
                print(f'unconverted: {os.path.relpath(path, ROOT)}:{i + 1}: {t!r}'); ok = False
    for e in entries:
        for lang in ('de', 'en'):
            fmt = lambda t: re.findall(r'%[-+ #0-9.]*[a-z]', t.replace('%%', ''))
            if fmt(e['nl']) != fmt(e[lang]):
                print(f'format mismatch ({lang}): {e["nl"]!r}'); ok = False
    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'generate':
        generate()
    elif cmd == 'convert':
        d = json.load(open(sys.argv[2], encoding='utf-8')) if len(sys.argv) > 2 else None
        convert(d)
    elif cmd == 'check':
        check()
    else:
        print(__doc__); sys.exit(2)
