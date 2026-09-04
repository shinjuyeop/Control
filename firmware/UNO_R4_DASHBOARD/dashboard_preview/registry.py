"""Ordered mapping from GUI variant names to renderer methods."""

VARIANT_METHODS = {
    "gt3_vitals": "_draw_gt3_vitals",
    "cup_light": "_draw_cup_light",
    "dual_dial": "_draw_dual_dial",
    "tape_gauges": "_draw_tape_gauges",
    "history_scope": "_draw_history_scope",
    "load_envelope": "_draw_load_envelope",
    "manual_tft": "_draw_manual_tft",
    "auto_gt3": "_draw_auto_gt3",
}

VARIANTS = ("baseline", *VARIANT_METHODS)
MODE_PAGE_VARIANTS = frozenset(("manual_tft", "auto_gt3"))
