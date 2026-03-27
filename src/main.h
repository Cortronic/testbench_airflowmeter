

typedef enum {
  MT_SELECT = 0,
  MT_SELECT_OPERATION = 1,
  MT_OPERATION = 2,
  MT_CAL_FLOW = 3,
  MT_TUNE_PID_FLOW = 4,
  MT_TUNE_PID_BALANCE = 5,
  MT_ADJUST_OFFSETS = 6,
} ModeType;
extern ModeType modeType;

typedef enum {
  FOLLOW_PULL_FAN = 0,
  FOLLOW_PUSH_FAN = 1,
} FollowFanType;
extern FollowFanType followFanType;

typedef enum {
  OM_SPEED_PULL_FAN = 0,
  OM_SPEED_PUSH_FAN = 1,
  OM_FLOW_PULL_FAN = 2,
  OM_FLOW_PUSH_FAN = 3,
  OM_POWER_PULL_FAN = 4,
  OM_POWER_PUSH_FAN = 5,
} OperationMode;
extern OperationMode operationMode;

typedef enum {
  PID_TUNE_NONE = 0,
  PID_TUNE_P = 1,
  PID_TUNE_I = 2,
  PID_TUNE_D = 3,
} PidTuneType;
extern PidTuneType pidTuneType;
