/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017-2020 Nick Moore
 * Copyright (c) 2018 shawwwn <shawwwn1@gmail.com>
 * Copyright (c) 2020-2021 Glenn Moloney @glenn20
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_mesh.h"
#include "esp_wifi.h"

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/objarray.h"
#include "py/binary.h"

#include "mpconfigport.h"

#if MICROPY_PY_ESPMESH

#include "modnetwork.h"
#include "modespmesh.h"


/* From mesh_event_id_t in ESP-IDF components/esp_wifi/include/esp_mesh.h */
static const uint16_t MESH_EVENTS[] = {
    MP_QSTR_MESH_EVENT_STARTED,                 /**< mesh is started */
    MP_QSTR_MESH_EVENT_STOPPED,                 /**< mesh is stopped */
    MP_QSTR_MESH_EVENT_CHANNEL_SWITCH,          /**< channel switch */
    MP_QSTR_MESH_EVENT_CHILD_CONNECTED,         /**< a child is connected on softAP interface */
    MP_QSTR_MESH_EVENT_CHILD_DISCONNECTED,      /**< a child is disconnected on softAP interface */
    MP_QSTR_MESH_EVENT_ROUTING_TABLE_ADD,       /**< routing table is changed by adding newly joined children */
    MP_QSTR_MESH_EVENT_ROUTING_TABLE_REMOVE,    /**< routing table is changed by removing leave children */
    MP_QSTR_MESH_EVENT_PARENT_CONNECTED,        /**< parent is connected on station interface */
    MP_QSTR_MESH_EVENT_PARENT_DISCONNECTED,     /**< parent is disconnected on station interface */
    MP_QSTR_MESH_EVENT_NO_PARENT_FOUND,         /**< no parent found */
    MP_QSTR_MESH_EVENT_LAYER_CHANGE,            /**< layer changes over the mesh network */
    MP_QSTR_MESH_EVENT_TODS_STATE,              /**< state represents whether the root is able to access external IP network.
                                                     This state is a manual event that needs to be triggered with esp_mesh_post_toDS_state(). */
    MP_QSTR_MESH_EVENT_VOTE_STARTED,            /**< the process of voting a new root is started either by children or by the root */
    MP_QSTR_MESH_EVENT_VOTE_STOPPED,            /**< the process of voting a new root is stopped */
    MP_QSTR_MESH_EVENT_ROOT_ADDRESS,            /**< the root address is obtained. It is posted by mesh stack automatically. */
    MP_QSTR_MESH_EVENT_ROOT_SWITCH_REQ,         /**< root switch request sent from a new voted root candidate */
    MP_QSTR_MESH_EVENT_ROOT_SWITCH_ACK,         /**< root switch acknowledgment responds the above request sent from current root */
    MP_QSTR_MESH_EVENT_ROOT_ASKED_YIELD,        /**< the root is asked yield by a more powerful existing root. If self organized is disabled
                                                     and this device is specified to be a root by users, users should set a new parent
                                                     for this device. if self organized is enabled, this device will find a new parent
                                                     by itself, users could ignore this event. */
    MP_QSTR_MESH_EVENT_ROOT_FIXED,              /**< when devices join a network, if the setting of Fixed Root for one device is different
                                                     from that of its parent, the device will update the setting the same as its parent's.
                                                     Fixed Root Setting of each device is variable as that setting changes of the root. */
    MP_QSTR_MESH_EVENT_SCAN_DONE,               /**< if self-organized networking is disabled, user can call esp_wifi_scan_start() to trigger
                                                     this event, and add the corresponding scan done handler in this event. */
    MP_QSTR_MESH_EVENT_NETWORK_STATE,           /**< network state, such as whether current mesh network has a root. */
    MP_QSTR_MESH_EVENT_STOP_RECONNECTION,       /**< the root stops reconnecting to the router and non-root devices stop reconnecting to their parents. */
    MP_QSTR_MESH_EVENT_FIND_NETWORK,            /**< when the channel field in mesh configuration is set to zero, mesh stack will perform a
                                                     full channel scan to find a mesh network that can join, and return the channel value
                                                     after finding it. */
    MP_QSTR_MESH_EVENT_ROUTER_SWITCH,           /**< if users specify BSSID of the router in mesh configuration, when the root connects to another
                                                     router with the same SSID, this event will be posted and the new router information is attached. */
    MP_QSTR_MESH_EVENT_PS_PARENT_DUTY,          /**< parent duty */
    MP_QSTR_MESH_EVENT_PS_CHILD_DUTY,           /**< child duty */
    MP_QSTR_MESH_EVENT_PS_DEVICE_DUTY,          /**< device duty */
};

// Allowed values: MESH_TOPO_TREE or MESH_TOPO_CHAIN
static const esp_mesh_topology_t DEFAULT_MESH_TOPOLOGY = MESH_TOPO_TREE;

// Max 25 layers for tree, 100 for chain
static const int DEFAULT_MESH_MAX_LAYER = 6;

// Mesh ID, must be unique for each network
static const uint8_t DEFAULT_MESH_ID[] = {0x77, 0x77, 0x77, 0x77, 0x77, 0x77};

// SoftAP settings
static const wifi_auth_mode_t DEFAULT_MESH_AP_AUTHMODE = WIFI_AUTH_WPA2_PSK;
static const uint8_t DEFAULT_MESH_AP_CONNECTIONS = 6;
static const uint8_t DEFAULT_MESH_NON_MESH_AP_CONNECTIONS = 0;

// Power save mode
static const bool DEFAULT_MESH_PS = true;
static const int DEFAULT_MESH_PS_DEVICE_DUTY = 10;
// can be MESH_PS_DEVICE_DUTY_REQUEST or MESH_PS_DEVICE_DUTY_DEMAND
static const int DEFAULT_MESH_PS_DEVICE_DUTY_REQ = MESH_PS_DEVICE_DUTY_REQUEST;
static const int DEFAULT_MESH_PS_NWK_DUTY = 10;
static const int DEFAULT_MESH_PS_NWK_DUTY_DURATION = -1;
// can be MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE or MESH_PS_NETWORK_DUTY_APPLIED_PARTIAL
static const int DEFAULT_MESH_PS_NWK_DUTY_APPLIED = MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE;

// The data structure for the espmesh_singleton.
typedef struct _esp_espmesh_obj_t {
    mp_obj_base_t base;

    bool initialized;
    esp_netif_t *netif_sta;
    esp_netif_t *netif_ap;

    esp_mesh_topology_t topology;
    int max_layer;
    wifi_auth_mode_t ap_authmode;
    mesh_cfg_t mesh_cfg;
    bool ps;
    int ps_device_duty;
    int ps_device_duty_req;
    int ps_nwk_duty;
    int ps_nwk_duty_duration;
    int ps_nwk_duty_applied;

    mp_obj_t mesh_event_handler;
} esp_espmesh_obj_t;

const mp_obj_type_t esp_espmesh_type;

// ### Initialization and Config functions
//

// Return a pointer to the ESPMesh module singleton
static esp_espmesh_obj_t *_get_singleton() {
    return MP_STATE_PORT(espmesh_singleton);
}

// Allocate and initialize the ESPMesh module as a singleton.
// Returns the initialized espmesh_singleton.
static mp_obj_t espmesh_make_new(const mp_obj_type_t *type, size_t n_args,
                                 size_t n_kw, const mp_obj_t *all_args) {

    // The espmesh_singleton must be defined in MICROPY_PORT_ROOT_POINTERS
    // (see mpconfigport.h) to prevent memory allocated here from being
    // garbage collected.
    // NOTE: on soft reset the espmesh_singleton MUST be set to NULL and the
    // ESP-MESH functions de-initialized (see main.c).
    esp_espmesh_obj_t *self = MP_STATE_PORT(espmesh_singleton);
    if (self != NULL) {
        return self;
    }

    self = m_new_obj(esp_espmesh_obj_t);
    self->base.type = &esp_espmesh_type;
    self->initialized = false;
    self->netif_sta = NULL;
    self->netif_ap = NULL;

    self->topology = DEFAULT_MESH_TOPOLOGY;
    self->max_layer = DEFAULT_MESH_MAX_LAYER;
    self->ap_authmode = DEFAULT_MESH_AP_AUTHMODE;
    // this typedef is necessary because the macro expands to a compound literal (initializer list)
    self->mesh_cfg = (mesh_cfg_t) MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *) &self->mesh_cfg.mesh_id, DEFAULT_MESH_ID, 6);
    self->mesh_cfg.mesh_ap.max_connection = DEFAULT_MESH_AP_CONNECTIONS;
    self->mesh_cfg.mesh_ap.nonmesh_max_connection = DEFAULT_MESH_NON_MESH_AP_CONNECTIONS;

    self->mesh_cfg.router.ssid_len = 0;
    self->mesh_cfg.router.password[0] = '\0';
    self->mesh_cfg.channel = 0;
    self->mesh_cfg.mesh_ap.password[0] = '\0';

    self->ps = DEFAULT_MESH_PS;
    self->ps_device_duty = DEFAULT_MESH_PS_DEVICE_DUTY;
    self->ps_device_duty_req = DEFAULT_MESH_PS_DEVICE_DUTY_REQ;
    self->ps_nwk_duty = DEFAULT_MESH_PS_NWK_DUTY;
    self->ps_nwk_duty_duration = DEFAULT_MESH_PS_NWK_DUTY_DURATION;
    self->ps_nwk_duty_applied = DEFAULT_MESH_PS_NWK_DUTY_APPLIED;

    self->mesh_event_handler = mp_const_none;

    // Set the global singleton pointer for the espmesh protocol.
    MP_STATE_PORT(espmesh_singleton) = self;

    return self;
}

// forward declare the event handlers
static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);


// Initialize ESP-MESH and Wi-Fi and register handlers.
static void espmesh_init(mp_obj_t _) {
    esp_espmesh_obj_t *self = _get_singleton();

    if (self->initialized) {
        return;
    }
    self->initialized = true;

    // Check that the mesh configuration has been set
    if (self->mesh_cfg.router.ssid_len == 0) {
        mp_raise_msg(&mp_type_ValueError, "SSID not set");
    }
    if (self->mesh_cfg.router.password[0] == '\0') {
        mp_raise_msg(&mp_type_ValueError, "password not set");
    }
    if (self->mesh_cfg.channel == 0) {
        mp_raise_msg(&mp_type_ValueError, "channel not set");
    }

    // Network interface can only be initialized once (even across soft
    // reboots), so check if it has already been initialized. It could have
    // been initialized elsewhere (such as by the network module), but we
    // can't check that here.
    static int netif_initialized = 0;
    if (!netif_initialized) {
        check_esp_err(esp_netif_init());
        netif_initialized = 1;
    }

    // event loop has already been created by mp_task, so we don't need to
    // call esp_event_loop_create_default() here.

    // create network interfaces for mesh (we really only need the station,
    // but we save both so we can clean up later)
    check_esp_err(esp_netif_create_default_wifi_mesh_netifs(&self->netif_sta, &self->netif_ap));

    // initialize the Wi-Fi stack
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    check_esp_err(esp_wifi_init(&cfg));
    check_esp_err(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    check_esp_err(esp_wifi_start());

    // initialize the mesh stack
    check_esp_err(esp_mesh_init());
    check_esp_err(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    check_esp_err(esp_mesh_set_topology(self->topology));
    /*  set mesh max layer according to the topology */
    check_esp_err(esp_mesh_set_max_layer(self->max_layer));
    check_esp_err(esp_mesh_set_vote_percentage(1));
    check_esp_err(esp_mesh_set_xon_qsize(128));

    if (self->ps) {
        /* Enable mesh PS function */
        check_esp_err(esp_mesh_enable_ps());
        /* better to increase the associate expired time, if a small duty cycle is set. */
        check_esp_err(esp_mesh_set_ap_assoc_expire(60));
        /* better to increase the announce interval to avoid too much
           management traffic, if a small duty cycle is set. */
        check_esp_err(esp_mesh_set_announce_interval(600, 3300));
    } else {
        /* Disable mesh PS function */
        check_esp_err(esp_mesh_disable_ps());
        check_esp_err(esp_mesh_set_ap_assoc_expire(10));
    }

    /* mesh softAP */
    check_esp_err(esp_mesh_set_ap_authmode(self->ap_authmode));
    check_esp_err(esp_mesh_set_config(&self->mesh_cfg));
    /* mesh start */
    check_esp_err(esp_mesh_start());

    if (self->ps) {
        /* set the device active duty cycle. (default:10, MESH_PS_DEVICE_DUTY_REQUEST) */
        check_esp_err(esp_mesh_set_active_duty_cycle(self->ps_device_duty, self->ps_device_duty_req));
        /* set the network active duty cycle. (default:10, -1, MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE) */
        check_esp_err(esp_mesh_set_network_duty_cycle(self->ps_nwk_duty,
                                                      self->ps_nwk_duty_duration,
                                                      self->ps_nwk_duty_applied));
    }
}

// ESPMESH software stack, disable callbacks
// Note: this function is called from main.c:mp_task() to cleanup before soft
// reset, so cannot be declared static and must guard against self == NULL;.
void espmesh_deinit(mp_obj_t _) {
    esp_espmesh_obj_t *self = _get_singleton();
    if (self != NULL && self->initialized) {
        // unregister event handler
        esp_event_handler_unregister(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler);
        esp_mesh_stop();
        esp_mesh_deinit();
        esp_wifi_stop();
        esp_wifi_deinit();
        esp_netif_destroy_default_wifi(self->netif_sta);
        esp_netif_destroy_default_wifi(self->netif_ap);
        self->initialized = false;
    }
}

static mp_obj_t espmesh_active(size_t n_args, const mp_obj_t *args) {
    esp_espmesh_obj_t *self = _get_singleton();
    if (n_args > 1) {
        if (mp_obj_is_true(args[1])) {
            espmesh_init(self);
        } else {
            espmesh_deinit(self);
        }
    }
    return mp_obj_new_bool(self->initialized);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(espmesh_active_obj, 1, 2, espmesh_active);

// ESPMesh.config(['param'|param=value, ..])
// Get or set configuration values. Supported config params:
//    ssid: SSID of the router that the root node will connect to
//    password: Router password
//    channel: Router Wi-Fi channel
//    ap_password: Mesh SoftAP password
//    power_save: Enable power save mode
static mp_obj_t espmesh_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    esp_espmesh_obj_t *self = _get_singleton();
    enum {
        ARG_get,
        ARG_ssid,
        ARG_password,
        ARG_channel,
        ARG_ap_password,
        ARG_power_save,
    };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_ssid, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_password, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_channel, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_ap_password, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_power_save, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = self->ps} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    size_t len;
    const char *p;

    if (args[ARG_ssid].u_obj != MP_OBJ_NULL) {
        p = mp_obj_str_get_data(args[ARG_ssid].u_obj, &len);
        if (len > sizeof(self->mesh_cfg.router.ssid) - 1) {
            mp_raise_ValueError(MP_ERROR_TEXT("SSID too long"));
        }
        memcpy(self->mesh_cfg.router.ssid, p, len);
        self->mesh_cfg.router.ssid[len] = '\0';
        self->mesh_cfg.router.ssid_len = len;
    }
    if (args[ARG_password].u_obj != MP_OBJ_NULL) {
        p = mp_obj_str_get_data(args[ARG_password].u_obj, &len);
        if (len > sizeof(self->mesh_cfg.router.password) - 1) {
            mp_raise_ValueError(MP_ERROR_TEXT("password too long"));
        }
        memcpy(self->mesh_cfg.router.password, p, len);
        self->mesh_cfg.router.password[len] = '\0';
    }
    if (args[ARG_channel].u_int != -1) {
        self->mesh_cfg.channel = args[ARG_channel].u_int;
    }
    if (args[ARG_ap_password].u_obj != MP_OBJ_NULL) {
        p = mp_obj_str_get_data(args[ARG_ap_password].u_obj, &len);
        if (len > sizeof(self->mesh_cfg.mesh_ap.password) - 1) {
            mp_raise_ValueError(MP_ERROR_TEXT("AP password too long"));
        }
        memcpy(self->mesh_cfg.mesh_ap.password, p, len);
        self->mesh_cfg.mesh_ap.password[len] = '\0';
    }
    self->ps = args[ARG_power_save].u_bool;

    if (args[ARG_get].u_obj == MP_OBJ_NULL) {
        return mp_const_none;
    }
    // check if it is a string
    if (!mp_obj_is_str(args[ARG_get].u_obj)) {
        mp_raise_ValueError(MP_ERROR_TEXT("config param must be a string"));
    }
#define QS(x) (uintptr_t)MP_OBJ_NEW_QSTR(x)
    // Return the value of the requested parameter
    uintptr_t name = (uintptr_t)args[ARG_get].u_obj;
    if (name == QS(MP_QSTR_ssid)) {
        return mp_obj_new_str((char *)self->mesh_cfg.router.ssid, self->mesh_cfg.router.ssid_len);
    } else if (name == QS(MP_QSTR_password)) {
        return mp_obj_new_str((char *)self->mesh_cfg.router.password, strlen((char *)self->mesh_cfg.router.password));
    } else if (name == QS(MP_QSTR_channel)) {
        return MP_OBJ_NEW_SMALL_INT(self->mesh_cfg.channel);
    } else if (name == QS(MP_QSTR_ap_password)) {
        return mp_obj_new_str((char *)self->mesh_cfg.mesh_ap.password, strlen((char *)self->mesh_cfg.mesh_ap.password));
    } else if (name == QS(MP_QSTR_power_save)) {
        return mp_obj_new_bool(self->ps);
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("unknown config param"));
    }
#undef QS

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(espmesh_config_obj, 1, espmesh_config);

// ESPMesh.register_event_handler(callback)
// Set callback function to be invoked when a mesh event occurs.
static mp_obj_t espmesh_register_event_handler(size_t n_args, const mp_obj_t *args) {
    esp_espmesh_obj_t *self = _get_singleton();
    mp_obj_t callback = args[1];
    if (callback != mp_const_none && !mp_obj_is_callable(callback)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid handler"));
    }
    self->mesh_event_handler = callback;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(espmesh_register_event_handler_obj, 2, 2, espmesh_register_event_handler);


// Callback triggered when an ESP Mesh event occurs.
// Schedules the user callback if one has been registered (Espmesh.config()).
static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    esp_espmesh_obj_t *self = _get_singleton();

    // convert the mesh event code to a string before passing it to the callback
    mp_obj_t event;
    if (event_id >= 0 && event_id < MP_ARRAY_SIZE(MESH_EVENTS)) {
        event = MP_OBJ_NEW_QSTR(MESH_EVENTS[event_id]);
    } else {
        event = mp_obj_new_int(event_id);
    }

    if (self->mesh_event_handler != mp_const_none) {
        // TODO: also pass the event_data to the callback
        mp_sched_schedule(self->mesh_event_handler, event);
    }
}

static const mp_rom_map_elem_t esp_espmesh_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&espmesh_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&espmesh_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_register_event_handler), MP_ROM_PTR(&espmesh_register_event_handler_obj) },
};
static MP_DEFINE_CONST_DICT(esp_espmesh_locals_dict, esp_espmesh_locals_dict_table);

static const mp_rom_map_elem_t espmesh_globals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__espmesh) },
    { MP_ROM_QSTR(MP_QSTR_ESPMeshBase), MP_ROM_PTR(&esp_espmesh_type) },
};
static MP_DEFINE_CONST_DICT(espmesh_globals_dict, espmesh_globals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    esp_espmesh_type,
    MP_QSTR_ESPMeshBase,
    MP_TYPE_FLAG_NONE,
    make_new, espmesh_make_new,
    locals_dict, &esp_espmesh_locals_dict
    );

const mp_obj_module_t mp_module_espmesh = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&espmesh_globals_dict,
};

MP_REGISTER_MODULE(MP_QSTR__espmesh, mp_module_espmesh);
MP_REGISTER_ROOT_POINTER(struct _esp_espmesh_obj_t *espmesh_singleton);

#endif // MICROPY_PY_ESPMESH
