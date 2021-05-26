#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/queue.h>
#include <signal.h>
#include <glib.h>
#include <semaphore.h>
#include <time.h>
#include <assert.h>

#include "gattlib.h"


#define BLE_SCAN_TIMEOUT   	4
#define MAX_PAC_SIZE		20


#define DEVICE_NAME		"SMARTFP"
#define MAX_FLAG_CNT		20



char tx_char_uuid[]="6e407f01-b5a3-f393-e0a9-e50e24dcca9e";
char rx_char_uuid[]="6e407f02-b5a3-f393-e0a9-e50e24dcca9e";


uuid_t tx_uuid;
uuid_t rx_uuid;
char buf[MAX_PAC_SIZE];

uint8_t conn_flag[MAX_FLAG_CNT] = {0, };
uint8_t flag_idx = 0;



static GMainLoop* m_main_loop;




typedef void (*ble_discovered_device_t)(const char* addr, const char* name);

// We use a mutex to make the BLE connections synchronous
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

LIST_HEAD(listhead, connection_t) g_ble_connections;
struct connection_t {
	pthread_t thread;
	char* addr;
	LIST_ENTRY(connection_t) entries;
};






static void on_user_abort(int arg){
	g_main_loop_quit(m_main_loop);
}



void disconnect_handler(void* arg){
	printf("disconn handler called...\n");
	*(int*)arg = 0;

	exit(0);
}




#define IDX_SEQ			2
#define IDX_LEN			3
#define IDX_CMD			4
#define IDX_DEV_ID		5
#define IDX_SEN1_1		6
#define IDX_SEN1_2		7
#define IDX_SEN2_1		8
#define IDX_SEN2_2		9
#define IDX_SEN3_1		10
#define IDX_SEN3_2		11
#define IDX_SEN4_1		12
#define IDX_SEN4_2		13
#define IDX_ACT_HIST		14
#define IDX_HOUR		15
#define IDX_MIN			16


#define CMD_RT_END		0x0C

uint8_t dev_id = 0;
uint16_t sen1_tmp, sen2_tmp, sen3_tmp, sen4_tmp;
uint16_t sen1_avg, sen2_avg, sen3_avg, sen4_avg;
int32_t lux_tmp, uvi_tmp, lux_avg, uvi_avg;
uint8_t act_hist, hour, min;

void notification_handler(const uuid_t* uuid, const uint8_t* data, size_t data_length, void* user_data){
	int i;
	uint8_t seq_end_flag = 0;


	/*
	printf("noti : ");
	for(i = 0; i < data_length; i++){
		printf("%02x ", data[i]);
	}
	printf("\n");*/

	dev_id = data[IDX_DEV_ID];


	if(data[IDX_CMD] == CMD_RT_END){
		switch(data[IDX_SEQ]){
		case 0x14 :
			sen1_tmp = (data[IDX_SEN1_1] << 8) + data[IDX_SEN1_2];
			sen2_tmp = (data[IDX_SEN2_1] << 8) + data[IDX_SEN2_2];
			sen3_tmp = (data[IDX_SEN3_1] << 8) + data[IDX_SEN3_2];
			sen4_tmp = (data[IDX_SEN4_1] << 8) + data[IDX_SEN4_2];
			break;
		case 0x24 :
			sen1_avg = (data[IDX_SEN1_1] << 8) + data[IDX_SEN1_2];
			sen2_avg = (data[IDX_SEN2_1] << 8) + data[IDX_SEN2_2];
			sen3_avg = (data[IDX_SEN3_1] << 8) + data[IDX_SEN3_2];
			sen4_avg = (data[IDX_SEN4_1] << 8) + data[IDX_SEN4_2];
			break;
		case 0x34 :
			lux_tmp = (data[IDX_SEN1_1] << 24) + (data[IDX_SEN1_2] << 16) 
				+ (data[IDX_SEN2_1] <<  8) + (data[IDX_SEN2_2]);
			uvi_tmp = (data[IDX_SEN3_1] << 24) + (data[IDX_SEN3_2] << 16) 
				+ (data[IDX_SEN4_1] <<  8) + (data[IDX_SEN4_2]);
			break;
		case 0x44 :
			lux_avg = (data[IDX_SEN1_1] << 24) + (data[IDX_SEN1_2] << 16) 
				+ (data[IDX_SEN2_1] <<  8) + (data[IDX_SEN2_2]);
			uvi_avg = (data[IDX_SEN3_1] << 24) + (data[IDX_SEN3_2] << 16) 
				+ (data[IDX_SEN4_1] <<  8) + (data[IDX_SEN4_2]);

			act_hist = data[IDX_ACT_HIST];
			hour = data[IDX_HOUR];
			min = data[IDX_MIN];

			seq_end_flag = 1;
			break;
		default :
			fprintf(stderr, "unexpected seq data.\n");
			exit(0);
		}
	}





	if(seq_end_flag == 1){
		printf("%2d:%2d :: %d, %d, %d, %d, %d, %d :: %d, %d, %d, %d, %d, %d :: %d\n", hour, min,
			       	sen1_tmp, sen2_tmp, sen3_tmp, sen4_tmp, lux_tmp, uvi_tmp,
			       	sen1_avg, sen2_avg, sen3_avg, sen4_avg, lux_avg, uvi_avg, act_hist);

		seq_end_flag = 0;
	}

}




static void *ble_connect_device(void *arg) {
	struct connection_t *connection = arg;
	char* addr = connection->addr;
	gatt_connection_t* gatt_connection;
	gattlib_primary_service_t* services;
	gattlib_characteristic_t* characteristics;
	int services_count, characteristics_count;
	char uuid_str[MAX_LEN_UUID_STR + 1];
	int ret, i;

	size_t len;
	uint8_t local_flag_idx;


	local_flag_idx = flag_idx++;
	conn_flag[local_flag_idx] = 1;







	pthread_mutex_lock(&g_mutex);

	printf("------------START %s ---------------\n", addr);

	gatt_connection = gattlib_connect(NULL, addr, GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT);
	if (gatt_connection == NULL) {
		fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		goto connection_exit;
	} else {
		puts("Succeeded to connect to the bluetooth device.");
		gattlib_string_to_uuid(tx_char_uuid, strlen(tx_char_uuid), &tx_uuid);
		gattlib_string_to_uuid(rx_char_uuid, strlen(rx_char_uuid), &rx_uuid);
	}


	gattlib_register_on_disconnect(gatt_connection, disconnect_handler, (void*)(conn_flag+local_flag_idx));

	gattlib_register_notification(gatt_connection, notification_handler, NULL);
	ret = gattlib_notification_start(gatt_connection, &tx_uuid);
	if(ret){
		fprintf(stderr, "fail to start noti\n");
		goto disconnect_exit;
	}
	else 	goto connection_exit;



disconnect_exit:
	gattlib_disconnect(gatt_connection);

connection_exit:
	printf("------------DONE %s ---------------\n", addr);
	pthread_mutex_unlock(&g_mutex);
	return NULL;
}

static void ble_discovered_device(void *adapter, const char* addr, const char* name, void *user_data) {
	struct connection_t *connection;
	int ret;

	if (name) {
		printf("Discovered %s - '%s'\n", addr, name);
	} else {
		printf("Discovered %s\n", addr);
	}


	if(name != NULL && strcmp(name, DEVICE_NAME) == 0){
		connection = malloc(sizeof(struct connection_t));
		if (connection == NULL) {
			fprintf(stderr, "Failt to allocate connection.\n");
			return;
		}
		connection->addr = strdup(addr);
	

		ret = pthread_create(&connection->thread, NULL,	ble_connect_device, connection);
		if (ret != 0) {
			fprintf(stderr, "Failt to create BLE connection thread.\n");
			free(connection);
			return;
		}
		LIST_INSERT_HEAD(&g_ble_connections, connection, entries);
	}
}
int main(int argc, const char *argv[]) {
	const char* adapter_name;
	void* adapter;
	int ret;

	if (argc == 1) {
		adapter_name = NULL;
	} else if (argc == 2) {
		adapter_name = argv[1];
	} else {
		fprintf(stderr, "%s [<bluetooth-adapter>]\n", argv[0]);
		return 1;
	}

	LIST_INIT(&g_ble_connections);

	ret = gattlib_adapter_open(adapter_name, &adapter);
	if (ret) {
		fprintf(stderr, "ERROR: Failed to open adapter.\n");
		return 1;
	}

	pthread_mutex_lock(&g_mutex);
	ret = gattlib_adapter_scan_enable(adapter, ble_discovered_device, BLE_SCAN_TIMEOUT, NULL /* user_data */);
	if (ret) {
		fprintf(stderr, "ERROR: Failed to scan.\n");
		goto EXIT;
	}

	gattlib_adapter_scan_disable(adapter);

	puts("Scan completed");
	pthread_mutex_unlock(&g_mutex);

	// Wait for the thread to complete
	while (g_ble_connections.lh_first != NULL) {
		struct connection_t* connection = g_ble_connections.lh_first;
		pthread_join(connection->thread, NULL);
		LIST_REMOVE(g_ble_connections.lh_first, entries);
		free(connection->addr);
		free(connection);
	}


	if(flag_idx == 0) return 0;


	signal(SIGINT, on_user_abort);

	m_main_loop = g_main_loop_new(NULL, 0);
	g_main_loop_run(m_main_loop);

	g_main_loop_unref(m_main_loop);




EXIT:
	gattlib_adapter_close(adapter);
	return ret;
}
