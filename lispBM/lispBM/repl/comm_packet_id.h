typedef enum {
	COMM_FW_VERSION							= 0,
	COMM_JUMP_TO_BOOTLOADER					= 1,
	COMM_ERASE_NEW_APP						= 2,
	COMM_WRITE_NEW_APP_DATA					= 3,
	COMM_GET_VALUES							= 4,
	COMM_SET_DUTY							= 5,
	COMM_SET_CURRENT						= 6,
	COMM_SET_CURRENT_BRAKE					= 7,
	COMM_SET_RPM							= 8,
	COMM_SET_POS							= 9,
	COMM_SET_HANDBRAKE						= 10,
	COMM_SET_DETECT							= 11,
	COMM_SET_SERVO_POS						= 12,
	COMM_SET_MCCONF							= 13,
	COMM_GET_MCCONF							= 14,
	COMM_GET_MCCONF_DEFAULT					= 15,
	COMM_SET_APPCONF						= 16,
	COMM_GET_APPCONF						= 17,
	COMM_GET_APPCONF_DEFAULT				= 18,
	COMM_SAMPLE_PRINT						= 19,
	COMM_TERMINAL_CMD						= 20,
	COMM_PRINT								= 21,
	COMM_ROTOR_POSITION						= 22,
	COMM_EXPERIMENT_SAMPLE					= 23,
	COMM_DETECT_MOTOR_PARAM					= 24,
	COMM_DETECT_MOTOR_R_L					= 25,
	COMM_DETECT_MOTOR_FLUX_LINKAGE			= 26,
	COMM_DETECT_ENCODER						= 27,
	COMM_DETECT_HALL_FOC					= 28,
	COMM_REBOOT								= 29,
	COMM_ALIVE								= 30,
	COMM_GET_DECODED_PPM					= 31,
	COMM_GET_DECODED_ADC					= 32,
	COMM_GET_DECODED_CHUK					= 33,
	COMM_FORWARD_CAN						= 34,
	COMM_SET_CHUCK_DATA						= 35,
	COMM_CUSTOM_APP_DATA					= 36,
	COMM_NRF_START_PAIRING					= 37,
	COMM_GPD_SET_FSW						= 38,
	COMM_GPD_BUFFER_NOTIFY					= 39,
	COMM_GPD_BUFFER_SIZE_LEFT				= 40,
	COMM_GPD_FILL_BUFFER					= 41,
	COMM_GPD_OUTPUT_SAMPLE					= 42,
	COMM_GPD_SET_MODE						= 43,
	COMM_GPD_FILL_BUFFER_INT8				= 44,
	COMM_GPD_FILL_BUFFER_INT16				= 45,
	COMM_GPD_SET_BUFFER_INT_SCALE			= 46,
	COMM_GET_VALUES_SETUP					= 47,
	COMM_SET_MCCONF_TEMP					= 48,
	COMM_SET_MCCONF_TEMP_SETUP				= 49,
	COMM_GET_VALUES_SELECTIVE				= 50,
	COMM_GET_VALUES_SETUP_SELECTIVE			= 51,
	COMM_EXT_NRF_PRESENT					= 52,
	COMM_EXT_NRF_ESB_SET_CH_ADDR			= 53,
	COMM_EXT_NRF_ESB_SEND_DATA				= 54,
	COMM_EXT_NRF_ESB_RX_DATA				= 55,
	COMM_EXT_NRF_SET_ENABLED				= 56,
	COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP	= 57,
	COMM_DETECT_APPLY_ALL_FOC				= 58,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN			= 59,
	COMM_ERASE_NEW_APP_ALL_CAN				= 60,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN			= 61,
	COMM_PING_CAN							= 62,
	COMM_APP_DISABLE_OUTPUT					= 63,
	COMM_TERMINAL_CMD_SYNC					= 64,
	COMM_GET_IMU_DATA						= 65,
	COMM_BM_CONNECT							= 66,
	COMM_BM_ERASE_FLASH_ALL					= 67,
	COMM_BM_WRITE_FLASH						= 68,
	COMM_BM_REBOOT							= 69,
	COMM_BM_DISCONNECT						= 70,
	COMM_BM_MAP_PINS_DEFAULT				= 71,
	COMM_BM_MAP_PINS_NRF5X					= 72,
	COMM_ERASE_BOOTLOADER					= 73,
	COMM_ERASE_BOOTLOADER_ALL_CAN			= 74,
	COMM_PLOT_INIT							= 75,
	COMM_PLOT_DATA							= 76,
	COMM_PLOT_ADD_GRAPH						= 77,
	COMM_PLOT_SET_GRAPH						= 78,
	COMM_GET_DECODED_BALANCE				= 79,
	COMM_BM_MEM_READ						= 80,
	COMM_WRITE_NEW_APP_DATA_LZO				= 81,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO		= 82,
	COMM_BM_WRITE_FLASH_LZO					= 83,
	COMM_SET_CURRENT_REL					= 84,
	COMM_CAN_FWD_FRAME						= 85,
	COMM_SET_BATTERY_CUT					= 86,
	COMM_SET_BLE_NAME						= 87,
	COMM_SET_BLE_PIN						= 88,
	COMM_SET_CAN_MODE						= 89,
	COMM_GET_IMU_CALIBRATION				= 90,
	COMM_GET_MCCONF_TEMP					= 91,

	// Custom configuration for hardware
	COMM_GET_CUSTOM_CONFIG_XML				= 92,
	COMM_GET_CUSTOM_CONFIG					= 93,
	COMM_GET_CUSTOM_CONFIG_DEFAULT			= 94,
	COMM_SET_CUSTOM_CONFIG					= 95,

	// BMS commands
	COMM_BMS_GET_VALUES						= 96,
	COMM_BMS_SET_CHARGE_ALLOWED				= 97,
	COMM_BMS_SET_BALANCE_OVERRIDE			= 98,
	COMM_BMS_RESET_COUNTERS					= 99,
	COMM_BMS_FORCE_BALANCE					= 100,
	COMM_BMS_ZERO_CURRENT_OFFSET			= 101,

	// FW updates commands for different HW types
	COMM_JUMP_TO_BOOTLOADER_HW				= 102,
	COMM_ERASE_NEW_APP_HW					= 103,
	COMM_WRITE_NEW_APP_DATA_HW				= 104,
	COMM_ERASE_BOOTLOADER_HW				= 105,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW		= 106,
	COMM_ERASE_NEW_APP_ALL_CAN_HW			= 107,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW		= 108,
	COMM_ERASE_BOOTLOADER_ALL_CAN_HW		= 109,

	COMM_SET_ODOMETER						= 110,

	// Power switch commands
	COMM_PSW_GET_STATUS						= 111,
	COMM_PSW_SWITCH							= 112,

	COMM_BMS_FWD_CAN_RX						= 113,
	COMM_BMS_HW_DATA						= 114,
	COMM_GET_BATTERY_CUT					= 115,
	COMM_BM_HALT_REQ						= 116,
	COMM_GET_QML_UI_HW						= 117,
	COMM_GET_QML_UI_APP						= 118,
	COMM_CUSTOM_HW_DATA						= 119,
	COMM_QMLUI_ERASE						= 120,
	COMM_QMLUI_WRITE						= 121,

	// IO Board
	COMM_IO_BOARD_GET_ALL					= 122,
	COMM_IO_BOARD_SET_PWM					= 123,
	COMM_IO_BOARD_SET_DIGITAL				= 124,

	COMM_BM_MEM_WRITE						= 125,
	COMM_BMS_BLNC_SELFTEST					= 126,
	COMM_GET_EXT_HUM_TMP					= 127,
	COMM_GET_STATS							= 128,
	COMM_RESET_STATS						= 129,

	// Lisp
	COMM_LISP_READ_CODE						= 130,
	COMM_LISP_WRITE_CODE					= 131,
	COMM_LISP_ERASE_CODE					= 132,
	COMM_LISP_SET_RUNNING					= 133,
	COMM_LISP_GET_STATS						= 134,
	COMM_LISP_PRINT							= 135,

	COMM_BMS_SET_BATT_TYPE					= 136,
	COMM_BMS_GET_BATT_TYPE					= 137,

	COMM_LISP_REPL_CMD						= 138,
	COMM_LISP_STREAM_CODE					= 139,

	COMM_FILE_LIST							= 140,
	COMM_FILE_READ							= 141,
	COMM_FILE_WRITE							= 142,
	COMM_FILE_MKDIR							= 143,
	COMM_FILE_REMOVE						= 144,

	COMM_LOG_START							= 145,
	COMM_LOG_STOP							= 146,
	COMM_LOG_CONFIG_FIELD					= 147,
	COMM_LOG_DATA_F32						= 148,

	COMM_SET_APPCONF_NO_STORE				= 149,
	COMM_GET_GNSS							= 150,

	COMM_LOG_DATA_F64						= 151,

	COMM_LISP_RMSG							= 152,

	//Placeholders for pinlock commands
	//COMM_PINLOCK1							= 153,
	//COMM_PINLOCK2							= 154,
	//COMM_PINLOCK3							= 155,

	COMM_SHUTDOWN							= 156,
} COMM_PACKET_ID;