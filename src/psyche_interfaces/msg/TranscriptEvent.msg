# Unified UI event stream combining partial, final, and HQ transcript updates.
# ``payload_json`` is deliberately small and versioned by the downstream server;
# it typically carries ``segment_id`` and text fields encoded as UTF-8 JSON.
uint8 KIND_PARTIAL=0                  # live partial hypothesis update
uint8 KIND_FINAL=1                    # finalised segment ready for display
uint8 KIND_HQ_PATCH=2                 # high-quality diff applied to stable text
builtin_interfaces/Time stamp_emitted # timestamp at emission
uint8 kind                            # one of the KIND_* constants
string payload_json                   # small JSON payload describing the update
