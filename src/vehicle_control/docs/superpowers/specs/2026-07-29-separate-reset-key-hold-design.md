# Separate MORAI Reset Key Hold Design

## Goal

Prevent MORAI from missing the `i` reset input without lengthening the `q`
control-mode transitions.

## Design

Add a `reset_key_hold` timing option dedicated to the reset key. Keep the
existing `key_hold` option for the three `q` control-mode key presses.

- `key_hold`: remain at `0.01` seconds for fast mode transitions.
- `reset_key_hold`: default to `0.12` seconds for reliable `i` input.
- `mode_settle`: use `0.25` seconds so MORAI finishes entering Manual mode
  before receiving `i`.
- `mode_settle`, `builtin_settle`, and all MORAI UDP ports remain unchanged.

The reset command builder will use `reset_key_hold` only for `reset_key` and
will continue to use `key_hold` for `control_toggle_key`.

## Configuration

Expose `reset_key_hold` under `morai_sim_reset_node` in `cyvox_mx.yaml`. The
node will load it independently from `key_hold`. Keep `mode_settle` independent
from the fast Built-In transition timing.

## Testing

Update the reset-command unit test first so it expects a 120 ms hold for `i`,
a 250 ms Manual settle, and 10 ms holds for every `q`. Confirm that this test
fails before changing production code, then run the complete `vehicle_control`
test suite.

## Scope

Only files inside `src/vehicle_control` are changed. No port numbers or other
packages are modified.
