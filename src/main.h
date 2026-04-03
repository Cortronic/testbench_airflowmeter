

typedef enum {
  MT_SELECT = 0,
  MT_SELECT_OPERATION = 1,
  MT_OPERATION = 2,
  MT_CAL_FLOW = 3,
  MT_TUNE_PID_FLOW = 4,
  MT_TUNE_PID_BALANCE = 5,
  MT_ADJUST_OFFSETS = 6,
  MT_SET_DIAMETERS = 7,
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

typedef enum {
  SET_DIA_NONE = 0,
  SET_DIA_INLET = 1,
  SET_DIA_THROAT = 2,
} SetDiameterType;
extern SetDiameterType setDiameterType;

typedef struct {
  float inletDiameter; // Diameter of the inlet of the venturi in meters
  float throatDiameter; // Diameter of the throat in meters
  float areaInlet; // Cross-sectional area of the inlet in square meters
  float areaThroat; // Cross-sectional area of the throat in square meters
  float betaRatio; // Ratio of throat diameter to inlet diameter (dimensionless)
  float betaCoefficient; // Coefficient for calculating beta ratio effects (1 - pow(betaRatio, 4))
  float dischargeCoefficient; // Discharge coefficient of the venturi (dimensionless)
} VenturiConstants;
extern VenturiConstants venturi;

// Keys for saving settings in non-volatile storage
#define KEY_VENTURI_INLET_DIAMETER "inletDia"
#define KEY_VENTURI_THROAT_DIAMETER "throadDia"
#define KEY_VENTURI_CD "Cd"
#define KEY_OFFSET_BALANCE_PRESSURE_SENSOR "offsetBalance"
#define KEY_OFFSET_VENTURI_PRESSURE_SENSOR "offsetVenturi"
#define KEY_KP_FLOW "KpFlow"
#define KEY_KI_FLOW "KiFlow"
#define KEY_KD_FLOW "KdFlow"
#define KEY_KP_BALANCE "KpBalance"
#define KEY_KI_BALANCE "KiBalance"
#define KEY_KD_BALANCE "KdBalance"
