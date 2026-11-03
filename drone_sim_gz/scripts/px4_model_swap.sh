# Sourced by install_into_px4_gz.sh and v3_marker/scripts/install_tof.sh (2026-10-31, codex model-install-atomicity reply-2).
# swap_model <src_dir> <models_dir> <name>: the model is copied once into <models_dir>/.versions/<name>.<content hash> and
# <models_dir>/<name> becomes a symlink to it; switching versions is one rename of a new symlink over the old (mv -T), so a gz
# starting at that moment sees either the whole old copy or the whole new one, never a half-copied directory. Old versions stay in
# .versions (not deleted). The first call on a real directory moves it to $ARC once (that single step is not atomic; it runs under
# the caller's lock; if the link cannot be made the directory is moved back). Prints unchanged | updated | migrated; non-zero
# return = nothing switched (the previous version stays in place). Callers hold the lock ($MODELS/.install.lock).
swap_model(){
  local src=$1 dir=$2 name=$3 h vd cur r=updated
  h=$(cd "$src" && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum | cut -c1-12)
  vd=".versions/$name.$h"; mkdir -p "$dir/.versions"
  if [ ! -d "$dir/$vd" ]; then rm -rf "$dir/$vd.tmp.$$"; cp -r "$src" "$dir/$vd.tmp.$$" && mv -T "$dir/$vd.tmp.$$" "$dir/$vd" || return 1; fi
  cur=$(readlink "$dir/$name" 2>/dev/null || true)
  [ "$cur" = "$vd" ] && { echo unchanged; return 0; }
  local old=""
  if [ -e "$dir/$name" ] && [ ! -L "$dir/$name" ]; then
    old="$ARC/${name}_$(date +%H%M%S)_$$"; mkdir -p "$ARC" && mv "$dir/$name" "$old" || return 1; r=migrated
  fi
  if ! { ln -sfn "$vd" "$dir/.$name.link.$$" && mv -T "$dir/.$name.link.$$" "$dir/$name"; }; then
    rm -f "$dir/.$name.link.$$"                       # the old link (if any) is untouched: rename is all-or-nothing
    [ -n "$old" ] && mv "$old" "$dir/$name"           # migration: put the real directory back
    return 1
  fi
  echo $r
}
