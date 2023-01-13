from enum import Enum

class rf_protocol:

    class ENUM_FUNCTION_CODE(Enum):
        ping = lambda msg: (msg[7])
        homing = lambda msg: (msg[6])
        moving = lambda msg: ([
            int.from_bytes(msg[6:10], 'little', signed=True), # 实际位置
            int.from_bytes(msg[10:14], 'little', signed=True), # 实际速度
            int.from_bytes(msg[14:16], 'little', signed=True), # 实际扭矩
            msg[16]]) # 传感器保护
        sort_action = lambda msg: (msg[6])
        sort_sensor_state = lambda msg: (msg[6])
        pick_box_action = lambda msg: (msg[7])
        pick_pin_action = lambda msg: (msg[6])
        pick_chain_homing = lambda msg: (msg[6])
        pick_sensor_state = lambda msg: ([msg[6], msg[7]])
        moving_read = lambda msg: (int.from_bytes(msg[6:10], 'little', signed=True))
        pick_chain_control = lambda msg: (int.from_bytes(msg[6:10], 'little', signed=True))
        halt_action = lambda msg: (msg[6])
        pick_sensor_cheeck_loaded = lambda msg: (msg[6])
        pick_sensor_cheeck_unload = lambda msg: (msg[6])
        

        FUNCTION_CODE_PING = {"value": 0x00, "func": ping}
        FUNCTION_CODE_HOMING = {"value": 0x01, "func": homing}
        FUNCTION_CODE_MOVING = {"value": 0x02, "func": moving}
        FUNCTION_CODE_SORT_ACTION = {"value": 0x03, "func": sort_action}  # SORT装货抛货
        FUNCTION_CODE_MOVING_CANCEL = {"value": 0x04}  # 机器人停止移动
        FUNCTION_CODE_SORT_SENSOR_STATE = {"value": 0x05, "func": sort_sensor_state}  # SORT传感器状态
        FUNCTION_CODE_PICK_BOX_ACTION = {"value": 0x06, "func": pick_box_action}  # 机器人拉还箱
        FUNCTION_CODE_PICK_PIN_ACTION = {"value": 0x07, "func": pick_pin_action}  # PICK，PIN IN，PIN OUT
        FUNCTION_CODE_PICK_CHAIN_HOMING = {"value": 0x08, "func": pick_chain_homing}  # 机器人链条回原点
        FUNCTION_CODE_PICK_SENSOR_STATE = {"value": 0x09, "func": pick_sensor_state}  # PICK机器人传感器状态
        FUNCTION_CODE_PICK_MOVING_READ = {"value": 0x0A, "func": moving_read}  # 读机器人主电机位置
        FUNCTION_CODE_PICK_CHAIN_DIRECT_CONTROL = {"value": 0x0C, "func": pick_chain_control}  # 链条直接控制
        FUNCTION_CODE_HALT_ACTION = {"value": 0x76, "func": halt_action}  # 机器人暂停，恢复
        FUNCTION_CODE_PICK_SENSOR_CHECK_LOADED = {"value": 0x87, "func": pick_sensor_cheeck_loaded}
        FUNCTION_CODE_PICK_SENSOR_CHECK_LOADING = {"value": 0x88, }  # 拉箱中传感器检查
        FUNCTION_CODE_PICK_SENSOR_CHECK_UNLOADED = {"value": 0x89, "func": pick_sensor_cheeck_unload}  # 还箱完成传感器检查
        FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE = {"value": 0xDC, "func": pick_box_action} # 机器人拉还箱
        FUNCTION_CODE_DEBUG = {"value": 0xDD, }  # DEBUG
        FUNCTION_CODE_CONFIGURATION = {"value": 0xC0, }  # 读，写配置

    class ENUM_STATE_CMD(Enum):
        SHUTTLE_STATE_CMD_READ = 0x00
        SHUTTLE_STATE_CMD_ONLINE = 0x01

    class ENUM_SHUTTLE_STATE(Enum):
        SHUTTLE_STATE_BOOTUP = {"value": 0x00, "chinese": "启动"} 
        SHUTTLE_STATE_STANDBY = {"value": 0x01, "chinese": "待机"} 
        SHUTTLE_STATE_ONLINE = {"value": 0x02, "chinese": "上线"}
        SHUTTLE_STATE_CONFIG_ERROR = {"value": 0x07, "chinese": "机器人配置错误"}
        SHUTTLE_STATE_FAULT = {"value": 0x08, "chinese": "机器人处于错误状态"}
        SHUTTLE_STATE_LOW_BOOTUP_VOLTAGE = {"value": 0x09, "chinese": "机器人启动电压低"} 
        SHUTTLE_STATE_BELT_INIT_ERROR = {"value": 0x0A, "chinese": "链条电机伺服初始化错误"}
        SHUTTLE_STATE_MAIN_SERVO_INIT_ERROR = {"value": 0x0B, "chinese": "机器人主电机伺服初始化错误"}
        SHUTTLE_STATE_MAIN_SERVO_BRAKE_FEEDBACK_ERROR = {"value": 0x0C, "chinese": "机器人抱闸状态错误"}
        SHUTTLE_STATE_MAIN_SERVO_BRAKE_SIGNAL_ERROR = {"value": 0x0D, "chinese": "机器人伺服抱闸状态错误"}
        SHUTTLE_STATE_ESTOP_TRIGGERED = {"value": 0x0E, "chinese": "机器人急停触发"}
        SHUTTLE_STATE_BRAKE_HARDWARE_OVERRIDE_TRIGGERED = {"value": 0x0F, "chinese": "机器人硬件解闸失效"}

    class ENUM_HOMING_CMD(Enum):
        ING_CMD_START_AND_STATE = 0x00
        HOMING_CMD_RESET_STATE = 0x01
        HOMING_CMD_DELIVER_POSITIVE = 0x02  # only used in horizontal vehicle
        HOMING_CMD_DELIVER_NEGATIVE = 0x03  # only used in horizontal vehicle
        HOMING_CMD_START_REVERSE_AND_STATE = 0x04

    class ENUM_HOMING_STATE(Enum):
        HOMING_STATE_IDLE = {"value": 0x00, "chinese": "空闲"}
        HOMING_STATE_IN_PROGRESS = {"value": 0x01, "chinese": "对原点中"}
        HOMING_STATE_COMPLETED = {"value": 0x02, "chinese": "对原点完成"}
        HOMING_STATE_COMPLETED_LIFTER_C = {"value": 0x03, "chinese": "对原点完成举升机构C面"}  # only used in horizontal vehicle
        HOMING_STATE_COMPLETED_LIFTER_D = {"value": 0x04, "chinese": "对原点完成举升机构D面"}  # only used in horizontal vehicle
    
    class ENUM_SORT_ACTION_CMD(Enum):
        SORT_ACTION_CMD_READ_STATE = 0x00
        SORT_ACTION_CMD_LOAD_FROM_A = 0x01
        SORT_ACTION_CMD_LOAD_FROM_B = 0x02
        SORT_ACTION_CMD_UNLOAD_TO_A = 0x05
        SORT_ACTION_CMD_UNLOAD_TO_B = 0x06

    class ENUM_SORT_ACTION_FLAG(Enum):
        SORT_ACTION_FLAG_NORMAL_UNLOAD = 0x00
        SORT_ACTION_FLAG_FORCE_UNLOAD = 0x01

    class ENUM_SORT_ACTION_STATE(Enum):
        SORT_ACTION_STATE_LOADING = {"value": 0x03, "chinese": "装货中"}
        SORT_ACTION_STATE_LOADED = {"value": 0x04, "chinese": "装货完成"}
        SORT_ACTION_STATE_UNLOADING = {"value": 0x07, "chinese": "卸货中"}
        SORT_ACTION_STATE_UNLOADED = {"value": 0x08, "chinese": "卸货完成"}
        SORT_ACTION_STATE_LOAD_ERROR = {"value": 0x09, "chinese": "装货错误"}
        SORT_ACTION_STATE_UNLOAD_ERROR = {"value": 0x0A, "chinese": "卸货错误"}
        SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO = {"value": 0x0B, "chinese": "装货超时没有货物"}
        SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO = {"value": 0x0C, "chinese": "装货超时有货物"}
        SORT_ACTION_STATE_LOAD_OVERLENGTH = {"value": 0x0D, "chinese": "装货包裹超长"}
        SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO = {"value": 0x0E, "chinese": "卸货超时没有货物"}
        SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO = {"value": 0x0F, "chinese": "卸货超时有货物"}
        SORT_ACTION_STATE_LOAD_UNKNOW_PROFILE = {"value": 0x10, "chinese": "装货未知的属性"}
        SORT_ACTION_STATE_UNLOAD_UNKNOW_PROFILE = {"value": 0x11, "chinese": "卸货未知的属性"}
        SORT_ACTION_STATE_LOAD_SENSOR_ERROR = {"value": 0x12, "chinese": "装货传感器错误"}

    class ENUM_SORT_SENSOR_STATE(Enum):
        SORT_SENSOR_BITMASK_A_SIDE = {"value": 0x01, "chinese": "A面装货传感器触发"}
        SORT_SENSOR_BITMASK_CENTER = {"value": 0x02, "chinese": "中间装货传感器触发"}
        SORT_SENSOR_BITMASK_B_SIDE = {"value": 0x04, "chinese": "B面装货传感器触发"}
        SORT_SENSOR_BITMASK_HOMING = {"value": 0x08, "chinese": "主轴原点触发"}
        SORT_SENSOR_BITMASK_AVOIDANCE = {"value": 0x10, "chinese": "避障触发"}
        SORT_SENSOR_BITMASK_LIFTER_HOMING = {"value": 0x20, "chinese": "提升机原点触发"}

    class ENUM_PICK_BOX_ACTION(Enum):
        PICK_BOX_ACTION_CMD_READ_BOX_STATE = 0x80
        PICK_BOX_ACTION_CMD_LOAD_FROM_A = 0x81
        PICK_BOX_ACTION_CMD_LOAD_FROM_B = 0x82
        PICK_BOX_ACTION_CMD_UNLOAD_TO_A = 0x83
        PICK_BOX_ACTION_CMD_UNLOAD_TO_B = 0x84
        PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_A = 0x85
        PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_B = 0x86
        PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_A = 0x87
        PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_B = 0x88
        PICK_BOX_ACTION_CMD_LOADED_TO_CENTER = 0x89
        PICK_BOX_ACTION_CMD_CENTER_TO_LOADED = 0x8A
        PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR = 0x8B # 可恢复错误，称重恢复，垂直定位失败恢复
        PICK_BOX_ACTION_CMD_RESET_LOAD_UNLOAD_ERROR = 0x8C # 不可恢复错误，拉还箱失败
        PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_A = 0x8D  # A拉B还
        PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_B = 0x8E  # B拉A还
        PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK = 0x8F  # 换边结束确认指令
        PICK_BOX_ACTION_CHAIN_POSITION_RECOVER = 0x90  # 0xDC还箱命令完成后链条电机回原点
         

    class ENUM_PICK_BOX_STATES(Enum):
        PICK_BOX_NO_BOX = {"value": 0, "chinese": "还箱成功"}
        PICK_BOX_LOADED = {"value": 1, "chinese": "拉箱成功"}
        PICK_BOX_LOADING = {"value": 2, "chinese": "拉箱中"}
        PICK_BOX_RETURNING = {"value": 3, "chinese": "还箱中"}
        PICK_BOX_UNKNOWN = {"value": 4, "chinese": "未知的状态"}
        PICK_BOX_CENTERED = {"value": 5, "chinese": "箱子居中"}
        PICK_BOX_ERROR_RECOVERABLE = {"value": 6, "chinese": "可恢复错误"}
        PICK_BOX_LOAD_FAIL = {"value": 7, "chinese": "拉箱失败"}
        PICK_BOX_UNLOAD_FAIL = {"value": 8, "chinese": "还箱失败"}
        PICK_BOX_LOAD_OVERWEIGHT = {"value": 9, "chinese": "拉箱称重超重"}
        PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED = {"value": 11, "chinese": "换勾成功"}
        PICK_BOX_CHANGE_CLAW_SIDE_FAIL = {"value": 12, "chinese": "换勾失败"}
        PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL = {"value": 13, "chinese": "可恢复错误-抓钩抓紧失败"}
        PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL = {"value": 14, "chinese": "可恢复错误-托盘上钩失败"}
        PICK_BOX_ERROR_RECOVERABEL_CHAIN_TORQUE_EXCESS = {"value": 15, "chinese": "可恢复错误-还箱链条扭矩过大"}
        PICK_BOX_UNLOADED_AT_ENTRANCE = {"value": 16, "chinese": "0xDC还箱完成"}
        PICK_BOX_CHAIN_POSITION_RECOVER_ERROR = {"value": 17, "chinese": "0xDC链条复位超时"}

    class ENUM_PICK_PIN_ACTION_CMD(Enum):
        PICK_PIN_ACTION_CMD_IN = 0x60
        PICK_PIN_ACTION_CMD_OUT = 0x61

    class ENUM_PICK_PIN_STATES(Enum):
        PICK_PIN_IN = {"value": 0, "chinese": "PIN 缩进"}
        PICK_PIN_OUT = {"value": 1, "chinese": "PIN 伸出"}
        PICK_PIN_UNKNOWN = {"value": 2, "chinese": "PIN 未知的状态"}
        PICK_PIN_FAIL = {"value": 3, "chinese": "PIN 错误"}

    class ENUM_PICK_CHAIN_HOMING_CMD(Enum):
        PICK_CHAIN_HOMING_CMD_READ = 0x00
        PICK_CHAIN_HOMING_CMD_START = 0x01
        PICK_CHAIN_HOMING_CMD_CLEAR_STATE = 0x02
        PICK_CHAIN_HOMING_CMD_DIRECT = 0x03

    class ENUM_PICK_CHAIN_HOMING_STATES(Enum):
        PICK_CHAIN_HOMING_UNKNOWN = {"value": 0, "chinese": "未知的状态"}
        PICK_CHAIN_HOMING_IN_PROGRESS = {"value": 1, "chinese": "链条重置中"}
        PICK_CHAIN_HOMING_COMPLETED = {"value": 2, "chinese": "链条重置完成"}
        PICK_CHAIN_HOMING_ERROR = {"value": 3, "chinese": "链条错误"}

    class ENUM_PICK_SENSOR_BYTE1_STATE(Enum):
        PICK_SENSOR_BITMASK_HOMING = 0x01
        PICK_SENSOR_BITMASK_CHAIN_HOMING = 0x02
        PICK_SENSOR_BITMASK_AVOIDANCE = 0x04

    class ENUM_PICK_SENSOR_BYTE2_STATE(Enum):
        PICK_SENSOR_BITMASK_A1 = 0x01
        PICK_SENSOR_BITMASK_A2 = 0x02
        PICK_SENSOR_BITMASK_A3 = 0x04
        PICK_SENSOR_BITMASK_A4 = 0x08
        PICK_SENSOR_BITMASK_A5 = 0x10
        PICK_SENSOR_BITMASK_A6 = 0x20

    class ENUM_PICK_CHAIN_DIRECT_CTRL_CMD(Enum):
        CHAIN_DIRECT_CONTROL_READ = 0
        CHAIN_DIRECT_CONTROL_SET_TARGET = 1
        CLAW_3_DIRECT_CONTROL_READ = 3
        CLAW_4_DIRECT_CONTROL_READ = 4

    class ENUM_PICK_SENSOR_CHECK_LOAD_STATES(Enum):
        PICK_SENSOR_LOAD_STATE_OK = {"value": 0, "chinese": "传感器拉箱状态ok"}
        PICK_SENSOR_LOAD_STATE_POSITION_ERROR = {"value": 1, "chinese": "传感器拉箱状态位置错误"}
        PICK_SENSOR_LOAD_STATE_ERROR = {"value": 100, "chinese": "传感器拉箱状态错误"}

    class ENUM_PICK_SENSOR_CHECK_UNLOADED(Enum):
        PICK_SENSOR_UNLOAD_STATE_OK = {"value": 2, "chinese": "传感器还箱状态ok"}
        PICK_SENSOR_UNLOAD_STATE_POSITION_ERROR = {"value": 3, "chinese": "传感器还箱状态位置错误"}
        PICK_SENSOR_UNLOAD_STATE_EMPTY = {"value": 4, "chinese": "04"}
        PICK_SENSOR_UNLOAD_STATE_INPLACE_ERROR = {"value": 5, "chinese": "05"}
        PICK_SENSOR_UNLOAD_STATE_EDGE_ERROR = {"value": 6, "chinese": "06"}
        PICK_SENSOR_UNLOAD_STATE_ERROR = {"value": 7, "chinese": "传感器还箱状态错误"}

    class ENUM_HALT_ACTION_CMD(Enum):
        HALT_CMD_RESUME = 0x00 # 恢复
        HALT_CMD_HALT = 0x01 # 急停

    class ENUM_HALT_STATE(Enum):
        HALT_STATE_ERROR_HALT = {"value": 0x00, "chinese": "机器人错误停止"}
        HALT_STATE_HALT = {"value": 0x01, "chinese": "机器人停止"}
        HALT_STATE_RUNNING = {"value": 0x02, "chinese": "机器人恢复运行"}

    class ENUM_DEBUG_CMD(Enum):
        SORT_DEBUG_CMD_BELT_STOP = 0xD0
        SORT_DEBUG_CMD_BELT_LOAD_FROM_A = 0xD1
        SORT_DEBUG_CMD_BELT_LOAD_FROM_B = 0xD2
        SORT_DEBUG_CMD_BELT_UNLOAD_TO_A = 0xD3
        SORT_DEBUG_CMD_BELT_UNLOAD_TO_B = 0xD4
        SORT_DEBUG_CMD_SENSOR_POWER_ON = 0xD5
        SORT_DEBUG_CMD_SENSOR_POWER_OFF = 0xD6
        RTCP_PICK_CHAIN_POS_CPS_CONTROL = 0xD7
        RTCP_PICK_INTERLOCK_POS_CPS_CONTROL = 0xD8

    class ENUM_TIME_OUT(Enum):
        RECV_DATA_TIME_OUT = {"value": 0xFF, "chinese": "数据接收超时"}