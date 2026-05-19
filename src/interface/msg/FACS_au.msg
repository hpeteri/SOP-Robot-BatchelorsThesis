# FACS_au.msg
# Defines an Action Unit activation command for the FACS expression system.
# Multiple AUs can be blended simultaneously by the FACS node.

int32 au_id          # FACS Action Unit number (e.g., 1=inner brow raise, 6=cheek raise, 12=lip pull)
float64 intensity    # 0.0 = neutral, 1.0 = full activation
int32 priority       # Arbitration: higher priority overrides lower (3 > 2 > 1)
int64 duration_nanosecs  # Transition duration in nanoseconds
