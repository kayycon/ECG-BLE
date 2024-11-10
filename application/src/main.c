#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h> // host controller interface
#include <zephyr/bluetooth/services/bas.h>  // battery service GATT
#include <zephyr/settings/settings.h>

/* Define macros for UUIDs of the Remote Service
   Project ID: 001 (3rd entry)
   MFG ID = 0x01FF (4th entry)
*/
#define BT_UUID_REMOTE_SERV_VAL \
        BT_UUID_128_ENCODE(0xe9ea0000, 0xe19b, 0x0001, 0x01FF, 0xc7907585fc48)
#define BT_UUID_REMOTE_SERVICE 			BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)

/* UUID of the Data Characteristic */
#define BT_UUID_REMOTE_DATA_CHRC_VAL \
        BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x0001, 0x01FF, 0xc7907585fc48)
#define BT_UUID_REMOTE_DATA_CHRC 		BT_UUID_DECLARE_128(BT_UUID_REMOTE_DATA_CHRC_VAL)

/* UUID of the Message Characteristic */
#define BT_UUID_REMOTE_MESSAGE_CHRC_VAL \
        BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x0001, 0x01FF, 0xc7907585fc48)
#define BT_UUID_REMOTE_MESSAGE_CHRC 	BT_UUID_DECLARE_128(BT_UUID_REMOTE_MESSAGE_CHRC_VAL) 

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)
#define BLE_DATA_POINTS 600 // limited by MTU

// enumeration to keep track of the state of the notifications
enum bt_data_notifications_enabled {
    BT_DATA_NOTIFICATIONS_ENABLED,
    BT_DATA_NOTIFICATIONS_DISABLED,
};
enum bt_data_notifications_enabled notifications_enabled;

// declare struct of the BLE remove service callbacks
struct bt_remote_srv_cb {
    void (*notif_changed)(enum bt_data_notifications_enabled status);
    void (*data_rx)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
}; 
static struct bt_remote_srv_cb remote_service_callbacks;

// blocking thread semaphore to wait for BLE to initialize
static K_SEM_DEFINE(bt_init_ok, 1, 1);

/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

/* Scan response data */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL), 
};

/* Function Declarations */
static ssize_t read_data_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
void data_ccc_cfg_changed_cb(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
void bluetooth_set_battery_level(int level, int nominal_batt_mv);
uint8_t bluetooth_get_battery_level(void);

/* Setup BLE Services */
BT_GATT_SERVICE_DEFINE(remote_srv,
BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERVICE),
BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_DATA_CHRC,
                BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_READ,
                read_data_cb, NULL, NULL),
BT_GATT_CCC(data_ccc_cfg_changed_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_MESSAGE_CHRC,
                BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                BT_GATT_PERM_WRITE,
                NULL, on_write, NULL),
);

/* BLE Callback Functions */
void data_ccc_cfg_changed_cb(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications: %s", notif_enabled? "enabled":"disabled");

    notifications_enabled = notif_enabled? BT_DATA_NOTIFICATIONS_ENABLED:BT_DATA_NOTIFICATIONS_DISABLED;

    if (remote_service_callbacks.notif_changed) {
        remote_service_callbacks.notif_changed(notifications_enabled);
    }
}

static ssize_t read_data_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temperature_data, sizeof(temperature_data));
}

static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
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

int send_data_notification(struct bt_conn *conn, uint8_t *value, uint16_t length) {
    int ret = 0;

    struct bt_gatt_notify_params params = {0};
    const struct bt_gatt_attr *attr = &remote_srv.attrs[2];

    params.attr = attr;
    params.data = &value;
    params.len = length;
    params.func = on_sent;

    ret = bt_gatt_notify_cb(conn, &params);

    return ret;
}

/* Initialize the BLE Connection */
int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_srv_cb *remote_cb) {
    LOG_INF("Initializing Bluetooth");

    if ((bt_cb == NULL) | (remote_cb == NULL)) {
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

/* Function Declarations */
static struct bt_conn *current_conn;
void on_connected(struct bt_conn *conn, uint8_t ret);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_notif_changed(enum bt_data_notifications_enabled status);
void on_data_rx(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

/* Setup Callbacks */
struct bt_conn_cb bluetooth_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

struct bt_remote_srv_cb remote_service_callbacks = {
    .notif_changed = on_notif_changed,
    .data_rx = on_data_rx,
};

void on_data_rx(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
    uint8_t temp_str[len+1];
    memcpy(temp_str, data, len);
    temp_str[len] = 0x00; // manually append NULL character at the end

    LOG_INF("BT received data on conn %p. Len: %d", (void *)conn, len);
    LOG_INF("Data: %s", temp_str);
}

void on_connected(struct bt_conn *conn, uint8_t ret) {
    if (ret) { LOG_ERR("Connection error: %d", ret); }
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
    }
    else {
        LOG_INF("BT notifications disabled");
    }
}

void main(void) {
    /* Initialize Bluetooth */
    int err = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
    if (err) {
        LOG_ERR("BT init failed (err = %d)", err);
    }

    // do stuff, write to "data" array

    // send a notification that "data" is ready to be read...
    err = send_data_notification(current_conn, data, 1);
    if (err) {
        LOG_ERR("Could not send BT notification (err: %d)", err);
    }
    else {
        LOG_INF("BT data transmitted.");
    }


    /* Example of how to use the Battery Service
       `normalized_level` comes from an ADC reading of the battery voltage
       this function populates the BAS GATT with information
    */
    err = bt_bas_set_battery_level((int)normalized_level);
    if (err) {
        LOG_ERR("BAS set error (err = %d)", err);
    }

    // this function retrieves BAS GATT with information
    battery_level =  bt_bas_get_battery_level();
}

// You get to implement the HR GATT!