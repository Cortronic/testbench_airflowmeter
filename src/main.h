

typedef enum {
  MT_SELECT = 0,
  MT_OPERATION = 1,
  MT_SELECT_OPERATION = 2,
  MT_CAL_FLOW = 3,
  MT_TUNE_PID_FLOW = 4,
  MT_TUNE_PID_BALANCE = 5,
  MT_ADJUST_OFFSETS = 6,
  MT_SET_VENTURI_CONSTANTS = 7,
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
} OperatingMode;
extern OperatingMode operatingMode;

typedef enum {
  PID_TUNE_NONE = 0,
  PID_TUNE_P = 1,
  PID_TUNE_I = 2,
  PID_TUNE_D = 3,
  PID_TUNE_ALPHA = 4,
  PID_TUNE_BETA = 5,
  PID_TUNE_DAMPING = 6,
  PID_TUNE_TREND_LIMIT = 7,
} PidTuneType;
extern PidTuneType pidTuneType;

typedef enum {
  VENTURI_CONSTANTS_SET_NONE = 0,
  VENTURI_CONSTANTS_SET_DIA_INLET = 1,
  VENTURI_CONSTANTS_SET_DIA_THROAT = 2,
  VENTURI_CONSTANTS_SET_SMOOTHING_FACTOR = 3,
} VenturiConstantsType;
extern VenturiConstantsType venturiConstantsType;

// Keys for saving settings in non-volatile storage
#define KEY_VENTURI_INLET_DIAMETER "inletDia"
#define KEY_VENTURI_THROAT_DIAMETER "throadDia"
#define KEY_VENTURI_CD "Cd"
#define KEY_VENTURI_SMOOTHING_FACTOR "smoothFactor"
#define KEY_OFFSET_BALANCE_PRESSURE_SENSOR "offsetBalance"
#define KEY_OFFSET_VENTURI_PRESSURE_SENSOR "offsetVenturi"
#define KEY_KP_FLOW "KpFlow"
#define KEY_KI_FLOW "KiFlow"
#define KEY_KD_FLOW "KdFlow"
#define KEY_ALPHA_FLOW "flowAlpha"
#define KEY_BETA_FLOW "flowBeta"
#define KEY_DAMPING_FLOW "flowDamping"
#define KEY_TREND_LIMIT_FLOW "flowTrendLimit"
#define KEY_KP_BALANCE "KpBalance"
#define KEY_KI_BALANCE "KiBalance"
#define KEY_KD_BALANCE "KdBalance"
#define KEY_BALANCE_ALPHA "balanceAlpha"
#define KEY_BALANCE_BETA "balanceBeta"
#define KEY_BALANCE_DAMPING     "balanceDamping"
#define KEY_BALANCE_TREND_LIMIT "balanceTrendLim"

// PID controller parameters
#define PID_KP_MIN 0.0
#define PID_KP_MAX 50.0
#define PID_KP_STEP 0.01
#define PID_KP_DECIMALS 2
#define PID_KI_MIN 0.0
#define PID_KI_MAX 50.0
#define PID_KI_STEP 0.01
#define PID_KI_DECIMALS 2
#define PID_KD_MIN 0.0
#define PID_KD_MAX 50.0
#define PID_KD_STEP 0.01
#define PID_KD_DECIMALS 2
#define ALPHA_MIN 0.0
#define ALPHA_MAX 1.0
#define ALPHA_STEP 0.01
#define ALPHA_DECIMALS 2
#define BETA_MIN 0.0
#define BETA_MAX 1.0
#define BETA_STEP 0.01
#define BETA_DECIMALS 2
#define DAMPING_MIN 0.0
#define DAMPING_MAX 1.0
#define DAMPING_STEP 0.01
#define DAMPING_DECIMALS 2
#define TREND_LIMIT_MIN 0.0
#define TREND_LIMIT_MAX 100.0
#define TREND_LIMIT_STEP 0.1
#define TREND_LIMIT_DECIMALS 1
