#!/bin/bash
# 2026-10-31 (codex model-install-atomicity): this was a byte copy of repo_export/scripts/install_into_px4_gz.sh whose source
# paths (<this workspace>/gz/models/starling2, qvix_r_module) do not exist here, so it deleted the installed models and then
# failed to copy. The canonical installer lives in repo_export (models from repo_export/gz); this copy only forwards to it.
exec "$HOME/starling_sim/repo_export/scripts/install_into_px4_gz.sh" "$@"
