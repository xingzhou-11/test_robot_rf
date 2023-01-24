from rf_protocol import rf_protocol

class parameters:
    state = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PING.value["value"],
        "cmd": rf_protocol.ENUM_STATE_CMD.SHUTTLE_STATE_CMD_ONLINE.value,
        "finally_state_enum": rf_protocol.ENUM_SHUTTLE_STATE
    }

    homing = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_HOMING.value["value"],
        "cmd": rf_protocol.ENUM_HOMING_CMD.HOMING_CMD_RESET_STATE.value,
        "finally_state_enum": rf_protocol.ENUM_HOMING_STATE,
        "wait": []
    }

    moving = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_MOVING.value["value"],
        "absolute_target": 0,
        "velocity_in_counts": 0,
        "acceleration_in_counts": 2.5,
        "deceleration_in_counts": 2.5,
        "rescue_mode": 0,  # sort救援模式 0关闭 1打开
        "select_avoidance_direction": 0,  # 避障 0向前 1向后
        "target_position": 1,  # 0绝对位置不为目标位置 1绝对位置为目标位置
        "track_direction": 0,  # 0水平轨道 1竖直轨道
        "self-correct": 0,  # 自我校正 0关闭 1打开
        "reset_position_check": 0,  # 复位位置差 0关闭 1开启
        "enter_lier": 0,  # 进入升降机 0关闭 1打开
        "desired_position": 0
    }

    sort_action = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_SORT_ACTION.value["value"],
        "cmd": rf_protocol.ENUM_SORT_ACTION_CMD.SORT_ACTION_CMD_LOAD_FROM_A.value,
        "force_unload": rf_protocol.ENUM_SORT_ACTION_FLAG.SORT_ACTION_FLAG_NORMAL_UNLOAD.value,
        "finally_state_enum": rf_protocol.ENUM_SORT_ACTION_STATE,
        "wait": []
    }

    moving_cancel = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_MOVING_CANCEL.value["value"]
    }
    
    sort_sensor_state = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_SORT_SENSOR_STATE.value["value"]
    }

    pick_box_action = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
        "cmd": rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
        "decoupled_absolute_position": 9000,
        "aligned_absolute_position": 0,  # 对齐位置
        "leave_box_slot_absolute_position": 0,  # 防撞位置
        "column's_direction": 1,  # 0上升列 1下降列
        "vertical_correction": 1,  # 垂直修正 0关闭 1打开
        "load_weigh": 0,  # 拉箱称重 0关闭 1打开
        "definition_full_load": 0,  # 满载阈值标定 0关闭 1打开
        "check_space": 0,  # 抓钩换边检测两面空间 0关闭 1打开
        "finally_state_enum": rf_protocol.ENUM_PICK_BOX_STATES,
        "wait": [] # 允许退出的状态列表
    }

    pick_pin_action = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_PIN_ACTION.value["value"],
        "cmd": rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value,
        "finally_state_enum": rf_protocol.ENUM_PICK_PIN_STATES,
        "wait": []
    }

    pick_chain_homing = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_CHAIN_HOMING.value["value"],
        "cmd": rf_protocol.ENUM_PICK_CHAIN_HOMING_CMD.PICK_CHAIN_HOMING_CMD_START.value,
        "finally_state_enum": rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES,
        "wait": []
    }

    pick_senson_state = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_SENSOR_STATE.value["value"]
    }

    moving_read = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_MOVING_READ.value["value"]
    }

    pick_chain_direct_control = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_CHAIN_DIRECT_CONTROL.value["value"],
        "cmd": rf_protocol.ENUM_PICK_CHAIN_DIRECT_CTRL_CMD.CHAIN_DIRECT_CONTROL_SET_TARGET.value["value"],
        "absolute_target_position": 0
    }

    halt_atcion = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_HALT_ACTION.value["value"],
        "cmd": rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value,
        "finally_state_enum": rf_protocol.ENUM_HALT_STATE,
        "wait": []
    }

    sensor_check = {
        "func": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_SENSOR_CHECK_LOADED.value["value"],
        "finally_state_enum": rf_protocol.ENUM_SORT_SENSOR_STATE,
        "wait": []
    }

    robot_end_state = {
        "state": rf_protocol.ENUM_SHUTTLE_STATE.SHUTTLE_STATE_ONLINE.value["value"]
    }