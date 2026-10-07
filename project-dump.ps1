# merge-project.ps1
$ErrorActionPreference = "Stop"

$ProjectRoot = (Get-Location).Path
$OutputFile  = Join-Path $ProjectRoot "project_dump.txt"

# UTF-8 без BOM — и для чтения, и для записи
$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)

# Папки, которые не включаем
$ExcludeDirs = @(
  '.git', '.svn', '.hg',
  'node_modules', 'bower_components',
  'dist', 'build', 'out', 'bin', 'obj', 'target',
  '.venv', 'venv', 'env', '__pycache__',
  '.idea', '.vscode', '.vs',
  'coverage', '.next', '.nuxt', '.cache'
)

# Расширения, которые не включаем
$ExcludeExt = @(
  '.exe','.dll','.so','.dylib','.bin','.obj','.o','.a','.lib','.pdb',
  '.png','.jpg','.jpeg','.gif','.bmp','.ico','.svg','.webp',
  '.pdf','.zip','.tar','.gz','.7z','.rar','.xz','.bz2',
  '.mp3','.mp4','.mov','.avi','.mkv','.wav','.flac',
  '.woff','.woff2','.ttf','.eot','.otf',
  '.class','.jar','.war','.ear','.pyc','.pyo',
  '.sqlite','.db','.mdb','.lock'
)

# Имена файлов (без расширения), которые не включаем
$ExcludeNames = @(
  'license', 'licence', 'licenses', 'licences',
  'copying', 'copyright', 'notice', 'authors', 'contributors',
  'patents', 'unlicense'
)
$LicenseExt = @('.md', '.txt', '.rst', '.html', '')

function Test-ExcludedPath($RelativePath) {
  $parts = $RelativePath -split '[\\/]'
  foreach ($part in $parts) {
    if ($ExcludeDirs -contains $part) { return $true }
  }

  $fileName  = [System.IO.Path]::GetFileName($RelativePath)
  $nameNoExt = [System.IO.Path]::GetFileNameWithoutExtension($RelativePath).ToLowerInvariant()
  $ext       = [System.IO.Path]::GetExtension($RelativePath).ToLowerInvariant()

  if ($ExcludeNames -contains $nameNoExt -and $LicenseExt -contains $ext) { return $true }
  if ($ExcludeNames -contains $fileName.ToLowerInvariant()) { return $true }

  if ($ExcludeExt -contains $ext) { return $true }
  return $false
}

# Чтение файла как UTF-8 с автоопределением BOM.
# Если BOM нет — трактуем как UTF-8 (без ANSI-fallback).
function Read-TextFileUtf8($FullPath) {
  $reader = New-Object System.IO.StreamReader($FullPath, $Utf8NoBom, $true)
  try {
    return $reader.ReadToEnd()
  } finally {
    $reader.Close()
  }
}

$files = @()

if (Test-Path (Join-Path $ProjectRoot ".git")) {
  Write-Host "Git-репозиторий найден. Беру только отслеживаемые файлы (git ls-files)."
  # git ls-files отдаёт пути; чтобы корректно читать имена с кириллицей,
  # переключаем вывод git в UTF-8 (для git >= 2.x достаточно установить quotepath=false)
  $oldQuote = git -C $ProjectRoot config --get core.quotepath
  git -C $ProjectRoot config core.quotepath false | Out-Null
  $files = git -C $ProjectRoot ls-files
  if ($null -eq $oldQuote) {
    git -C $ProjectRoot config --unset core.quotepath | Out-Null
  } else {
    git -C $ProjectRoot config core.quotepath $oldQuote | Out-Null
  }
} else {
  Write-Host "Git не найден. Обхожу все файлы рекурсивно."
  $files = Get-ChildItem -LiteralPath $ProjectRoot -Recurse -File | ForEach-Object {
    $_.FullName.Substring($ProjectRoot.Length).TrimStart('\','/')
  }
}

# Пишем дамп в UTF-8 без BOM
$writer = New-Object System.IO.StreamWriter($OutputFile, $false, $Utf8NoBom)
try {
  foreach ($rel in $files) {
    if ([string]::IsNullOrWhiteSpace($rel)) { continue }

    $rel = $rel -replace '/', '\'
    if (Test-ExcludedPath $rel) { continue }

    $full = Join-Path $ProjectRoot $rel
    if (-not (Test-Path -LiteralPath $full -PathType Leaf)) { continue }
    if ($full -eq $OutputFile) { continue }

    $writer.WriteLine("===== $rel =====")
    try {
      $content = Read-TextFileUtf8 $full
    } catch {
      $content = "[Ошибка чтения файла: $($_.Exception.Message)]"
    }
    $writer.WriteLine($content)
    $writer.WriteLine()
  }
} finally {
  $writer.Close()
}

Write-Host "Готово: $OutputFile"