# High-level outcome category. Consumers should branch on this field first.
uint8 SUCCESS=0
uint8 RETRYABLE_FAILURE=1
uint8 FATAL_FAILURE=2
uint8 CANCELLED=3
uint8 TIMEOUT=4

# Machine-readable reason. NONE is used when the outcome needs no finer reason.
uint16 NONE=0
uint16 PREVIEW_COMPLETE=1
uint16 BUSY=2
uint16 INVALID_GOAL=3
uint16 EXECUTION_DISABLED=4
uint16 SERVER_UNAVAILABLE=5
uint16 SDK_NOT_READY=6
uint16 FEEDBACK_STALE=7
uint16 TF_UNAVAILABLE=8
uint16 OUT_OF_RANGE=9
uint16 IK_NO_RESPONSE=10
uint16 CONTROL_CONFLICT=11
uint16 STOP_FAILED=12
uint16 INTERNAL_ERROR=13
uint16 EXECUTION_STATE_UNKNOWN=14
uint16 POSE_EXECUTION_UNSAFE=15

uint8 outcome
uint16 reason
string message

# True only if this request caused a physical motion target to be published.
bool executed
