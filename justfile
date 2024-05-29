_default:
  @just --list --unsorted

init:
  @just _clone_embassy
  @just update_embassy

update:
  @echo "> Pulling updates from Embassy repo..."
  @echo "Current REV:" `git -C ../embassy rev-parse HEAD`
  git -C ../embassy fetch

_clone_embassy:
  @echo "> Trying to clone Embassy repo..."
  -git clone -q https://github.com/embassy-rs/embassy.git ../embassy
