#!/usr/bin/env bash
# project-dump.sh — объединяет файлы проекта в один текстовый дамп (UTF-8 без BOM).

set -euo pipefail

PROJECT_ROOT="$(pwd)"
OUTPUT_FILE="$PROJECT_ROOT/project_dump.txt"

# Папки, которые не включаем
EXCLUDE_DIRS=(
  '.git' '.svn' '.hg'
  'node_modules' 'bower_components'
  'dist' 'build' 'out' 'bin' 'obj' 'target'
  '.venv' 'venv' 'env' '__pycache__'
  '.idea' '.vscode' '.vs'
  'coverage' '.next' '.nuxt' '.cache'
)

# Расширения, которые не включаем
EXCLUDE_EXT=(
  '.exe' '.dll' '.so' '.dylib' '.bin' '.obj' '.o' '.a' '.lib' '.pdb'
  '.png' '.jpg' '.jpeg' '.gif' '.bmp' '.ico' '.svg' '.webp'
  '.pdf' '.zip' '.tar' '.gz' '.7z' '.rar' '.xz' '.bz2'
  '.mp3' '.mp4' '.mov' '.avi' '.mkv' '.wav' '.flac'
  '.woff' '.woff2' '.ttf' '.eot' '.otf'
  '.class' '.jar' '.war' '.ear' '.pyc' '.pyo'
  '.sqlite' '.db' '.mdb' '.lock'
)

# Имена файлов (без расширения), которые не включаем
EXCLUDE_NAMES=(
  'license' 'licence' 'licenses' 'licences'
  'copying' 'copyright' 'notice' 'authors' 'contributors'
  'patents' 'unlicense'
)
LICENSE_EXT=('.md' '.txt' '.rst' '.html' '')

# Проверка: входит ли значение в массив
in_array() {
  local needle="$1"; shift
  local item
  for item in "$@"; do
    [[ "$item" == "$needle" ]] && return 0
  done
  return 1
}

is_excluded_path() {
  local rel="$1"
  local part
  IFS='/' read -ra parts <<< "$rel"
  for part in "${parts[@]}"; do
    if in_array "$part" "${EXCLUDE_DIRS[@]}"; then
      return 0
    fi
  done

  local file_name name_no_ext ext
  file_name="$(basename -- "$rel")"
  name_no_ext="${file_name%.*}"
  name_no_ext="${name_no_ext,,}"   # в нижний регистр
  if [[ "$file_name" == "$name_no_ext" ]]; then
    ext=""
  else
    ext=".${file_name##*.}"
    ext="${ext,,}"
  fi

  local lname
  lname="$(echo "$file_name" | tr '[:upper:]' '[:lower:]')"
  if in_array "$name_no_ext" "${EXCLUDE_NAMES[@]}" && in_array "$ext" "${LICENSE_EXT[@]}"; then
    return 0
  fi
  if in_array "$lname" "${EXCLUDE_NAMES[@]}"; then
    return 0
  fi

  if in_array "$ext" "${EXCLUDE_EXT[@]}"; then
    return 0
  fi
  return 1
}

# Собираем список файлов
declare -a FILES=()

if [[ -d "$PROJECT_ROOT/.git" ]] && command -v git >/dev/null 2>&1; then
  echo "Git-репозиторий найден. Беру только отслеживаемые файлы (git ls-files)."
  # Аналог core.quotepath=false: не экранировать кириллицу и прочие не-ASCII пути.
  old_quote="$(git -C "$PROJECT_ROOT" config --get core.quotepath || true)"
  git -C "$PROJECT_ROOT" config core.quotepath false
  # -z: NUL-разделитель — безопасно для любых имён (включая переводы строк)
  while IFS= read -r -d '' f; do
    FILES+=("$f")
  done < <(git -C "$PROJECT_ROOT" ls-files -z)
  if [[ -z "$old_quote" ]]; then
    git -C "$PROJECT_ROOT" config --unset core.quotepath || true
  else
    git -C "$PROJECT_ROOT" config core.quotepath "$old_quote"
  fi
else
  echo "Git не найден. Обхожу все файлы рекурсивно."
  while IFS= read -r -d '' f; do
    FILES+=("${f#"$PROJECT_ROOT"/}")
  done < <(find "$PROJECT_ROOT" -type f -print0)
fi

# Пишем дамп в UTF-8 без BOM
: > "$OUTPUT_FILE"

for rel in "${FILES[@]}"; do
  [[ -z "$rel" ]] && continue
  rel="${rel//\\//}"   # на всякий случай: обратные слэши -> прямые

  if is_excluded_path "$rel"; then
    continue
  fi

  full="$PROJECT_ROOT/$rel"
  [[ -f "$full" ]] || continue
  [[ "$full" == "$OUTPUT_FILE" ]] && continue

  printf '===== %s =====\n' "$rel" >> "$OUTPUT_FILE"
  # Читаем как UTF-8, без ANSI-fallback. При ошибке — сообщаем и продолжаем.
  if ! cat -- "$full" >> "$OUTPUT_FILE"; then
    printf '[Ошибка чтения файла: %s]\n' "$full" >> "$OUTPUT_FILE"
  fi
  printf '\n' >> "$OUTPUT_FILE"
done

echo "Готово: $OUTPUT_FILE"
