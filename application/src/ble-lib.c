#include "ble-lib.h"

LOG_MODULE_REGISTER(blelib, LOG_LEVEL_DBG);

// define a semaphore to make sure that bt_init_ok() completes before proceeding
// this is probably overkill, but it's a good practice
static K_SEM_DEFINE(bt_init_ok, 1, 1);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)
#define NOMINAL_BATTERY_VOLT_MV 3700  // this might make more sense to have in main.c

// extern means this is defined elsewhere (in this case, main.c)
extern int32_t temperature_degC;  // temperature data characteristic will read from this variable
extern struct k_event errors; // errors characteristic will read from this variable

enum bt_data_notifications_enabled notifications_enabled;


/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

/* Scan response data */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL), 
};

/* UUID of the Remote Service */
// Project ID: 065 (3rd entry)
// BLE MFG ID = 0x02DF (4th entry)
/*  UUID of Data Characteristic  */
struct bt_uuid_128 remote_temperature_data_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48));
/*  UUID of Errors Characteristic  */
struct bt_uuid_128 remote_err_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48));
/*  UUID of Message Characteristic   */
struct bt_uuid_128 remote_msg_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xe9ea0003, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48));

/* Setup services */
BT_GATT_SERVICE_DEFINE(remote_srv,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERVICE),
    BT_GATT_CHARACTERISTIC(&remote_temperature_data_uuid.uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    read_temperature_data_cb, NULL, &temperature_degC),
    BT_GATT_CCC(ccc_cfg_changed_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&remote_err_uuid.uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                    read_error_cb, NULL, &errors.events),
    BT_GATT_CCC(ccc_cfg_changed_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&remote_msg_uuid.uuid,
                    BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                    BT_GATT_PERM_WRITE,
                    NULL, on_write, NULL),
    BT_GATT_CCC(ccc_cfg_changed_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

struct bt_conn *current_conn;
struct bt_conn_cb bluetooth_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

struct bt_remote_srv_cb remote_service_callbacks = {
    .notif_changed = on_notif_changed,
    .data_rx = on_data_rx,
};

/* Callbacks */
void on_data_rx(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
    // copy the data to a temporary buffer and append a NULL character
    uint8_t temp_str[len+1];
    memcpy(temp_str, data, len);
    temp_str[len] = 0x00;  // append NULL character at the end
    LOG_INF("BT received data on conn %p. Len: %d", (void *)conn, len);
    LOG_INF("Data: %s", temp_str);
}

void on_connected(struct bt_conn *conn, uint8_t ret) {
    if (ret) {
        LOG_ERR("Connection error: %d", ret);
        return;
    }
    LOG_INF("BT connected");
    current_conn = bt_conn_ref(conn);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("BT disconnected (reason: %d)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

void on_notif_changed(enum bt_data_notifications_enabled status) {
    if (status == BT_DATA_NOTIFICATIONS_ENABLED) {
        LOG_INF("BT notifications enabled");
    } else {
        LOG_INF("BT notifications disabled");
    }
}

const struct bt_gatt_attr *attr = &remote_srv.attrs[2];

void ccc_cfg_changed_cb(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications: %s", notif_enabled? "enabled":"disabled");
    notifications_enabled = notif_enabled? BT_DATA_NOTIFICATIONS_ENABLED:BT_DATA_NOTIFICATIONS_DISABLED;
    if (remote_service_callbacks.notif_changed) {
        remote_service_callbacks.notif_changed(notifications_enabled);
    }
}

/* These callbacks are used when a read request is made to the remote service */
ssize_t read_temperature_data_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temperature_degC, sizeof(temperature_degC));
}

ssize_t read_error_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &errors.events, sizeof(errors.events));
}

ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    LOG_INF("Received data, handle %d, conn %p", attr->handle, (void *)conn);
    
    if (remote_service_callbacks.data_rx) {
        remote_service_callbacks.data_rx(conn, buf, len);
    }
    return len;
}


void on_sent(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(user_data);
    LOG_INF("Notification sent on connection %p", (void *)conn);
}

void bt_ready(int ret) {
    if (ret) {
        LOG_ERR("bt_enable returned %d", ret);
    }
    // release the thread once initialized
    k_sem_give(&bt_init_ok);
}

static int settings_runtime_load(void)
{
#if defined(CONFIG_BT_DIS_SETTINGS)
	settings_runtime_set("bt/dis/model",
			     CONFIG_BT_DIS_MODEL,
			     sizeof(CONFIG_BT_DIS_MODEL));
	settings_runtime_set("bt/dis/manuf",
			     CONFIG_BT_DIS_MANUF,
			     sizeof(CONFIG_BT_DIS_MANUF));
#if defined(CONFIG_BT_DIS_SERIAL_NUMBER)
	settings_runtime_set("bt/dis/serial",
			     CONFIG_BT_DIS_SERIAL_NUMBER_STR,
			     sizeof(CONFIG_BT_DIS_SERIAL_NUMBER_STR));
#endif
#if defined(CONFIG_BT_DIS_SW_REV)
	settings_runtime_set("bt/dis/sw",
			     CONFIG_BT_DIS_SW_REV_STR,
			     sizeof(CONFIG_BT_DIS_SW_REV_STR));
#endif
#if defined(CONFIG_BT_DIS_FW_REV)
	settings_runtime_set("bt/dis/fw",
			     CONFIG_BT_DIS_FW_REV_STR,
			     sizeof(CONFIG_BT_DIS_FW_REV_STR));
#endif
#if defined(CONFIG_BT_DIS_HW_REV)
	settings_runtime_set("bt/dis/hw",
			     CONFIG_BT_DIS_HW_REV_STR,
			     sizeof(CONFIG_BT_DIS_HW_REV_STR));
#endif
#endif
	return 0;
}


int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_srv_cb *remote_cb) {
    LOG_INF("Initializing Bluetooth");
    if ((bt_cb == NULL) | (remote_cb == NULL)) {
        LOG_ERR("Invalid callback(s)");
        return -NRFX_ERROR_NULL;
    }
    bt_conn_cb_register(bt_cb);
    remote_service_callbacks.notif_changed = remote_cb->notif_changed;
    remote_service_callbacks.data_rx = remote_cb->data_rx;
    int ret = bt_enable(bt_ready);
    if (ret) {
        LOG_ERR("bt_enable returned %d", ret);
        return ret;
    }

    // hold the thread until Bluetooth is initialized
    k_sem_take(&bt_init_ok, K_FOREVER);
    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        settings_load();
    }
    ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Could not start advertising (ret = %d)", ret);
        return ret;
    }
    return ret;
}

/* BAS Functions */
uint8_t bluetooth_get_battery_level(void) {
    uint8_t battery_level;
    battery_level =  bt_bas_get_battery_level();
    LOG_INF("Battery Level: %d pct", battery_level);
    return battery_level;
}

void bluetooth_set_battery_level(int32_t raw_mV) {
    LOG_DBG("Raw Battery: %d mV", raw_mV);
    
    float normalized_level = (float)raw_mV * 100 / NOMINAL_BATTERY_VOLT_MV; //100
    
    LOG_INF("Normalized Battery Level: %lf", (double)normalized_level);
    int err = bt_bas_set_battery_level((int)normalized_level);
    if (err) {
        LOG_ERR("BAS set error (err = %d)", err);
    }
}
void notify_heart_rate(uint8_t heart_rate) {
    if (bt_hrs_notify(heart_rate) == 0) {
        LOG_INF("Heart rate notified: %d BPM", heart_rate);
    } else {
        LOG_ERR("Failed to notify heart rate.");
    }
}

int send_BT_notification(struct bt_conn *conn, uint8_t *value, uint16_t length) {
    if (!conn) {
        LOG_ERR("Invalid connection");
        return -EINVAL;
    }

    struct bt_gatt_notify_params params = {0};

    params.attr = &remote_srv.attrs[2]; // Update index based on your characteristic
    params.data = value;
    params.len = length;

    int ret = bt_gatt_notify_cb(conn, &params);
    if (ret) {
        LOG_ERR("Failed to send notification: %d", ret);
    } else {
        LOG_INF("Notification sent successfully");
    }
    return ret;
}

void check_and_send_temperature_notification(void) {
    if (current_conn && notifications_enabled == BT_DATA_NOTIFICATIONS_ENABLED) {
        int ret = send_BT_notification(current_conn, (uint8_t *)&temperature_degC, sizeof(temperature_degC));
        if (ret) {
            LOG_ERR("Failed to send temperature notification: %d", ret);
        } else {
            LOG_INF("Temperature notification sent.");
        }
    } else {
        LOG_WRN("No active connection or notifications not enabled, skipping notification.");
    }
}

/* Declare ecg_attrs and ecg_data */
static struct bt_gatt_attr ecg_attrs[] = {
    /* Add your ECG attributes here */
};

static uint8_t ecg_data[20]; // Adjust the size as needed

void send_ecg_notification(void) {
    if (current_conn && notifications_enabled == BT_DATA_NOTIFICATIONS_ENABLED) {
        struct bt_gatt_notify_params params = {0}; // Initialize the params structure

        params.attr = &ecg_attrs[0]; // Update index based on your ECG characteristic
        params.data = ecg_data; // Use the ECG data buffer
        params.len = sizeof(ecg_data); // Set the length of the data to send

        int ret = bt_gatt_notify_cb(current_conn, &params);
        if (ret < 0) {
            LOG_ERR("Failed to send ECG notification: %d", ret);
        } else {
            LOG_INF("ECG notification sent.");
        }
    } else {
        LOG_WRN("No active connection or notifications not enabled, ECG notification skipped.");
    }
}

void send_error_notification(void) {
    if (current_conn) {
        int ret = bt_gatt_notify(current_conn, &remote_err_uuid, &errors.events, sizeof(errors.events));
        if (ret < 0) {
            LOG_ERR("Failed to send error notification: %d", ret);
        } else {
            LOG_INF("Error notification sent.");
        }
    } else {
        LOG_WRN("No active connection, error notification skipped.");
    }
}

