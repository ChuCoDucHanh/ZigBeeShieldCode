#include <zb_znp.h>
#include "zb_zcl.h"
#include "task_zigbee.h"
#include "zb_defs.h"
#include <string.h>
#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <SoftwareSerial.h>
#include <avr/interrupt.h>

//#define DBG_ZB_FRAME
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define TOUCH_SENSOR_MILESTONE	1
#define TOUCH_COUNTER_MAX		40
#define TOUCH_COUNTER_UPDATE	18000		// 30'
#define NUM_TOUCH				11
//SoftwareSerial znp_serial(2, 3);
zb_znp zigbee_network(&Serial1);
Adafruit_MPR121 cap = Adafruit_MPR121();
void zigbee_data_structure_set(uint8_t * zigbee_package, uint16_t sig, uint8_t * data, uint8_t len);
void zigbee_send_data_up();
void zigbee_send_data_down();
static void task_timer_100ms();
static void task_zigbee();
void test_touch();
static int interupt_timer_100ms_count = 0;

uint16_t control_switch_address;
unsigned char led = 13;
// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

uint8_t debug = 1;
uint16_t debug_count = 0;

uint8_t send_status = 0xFE;

typedef struct {
	uint16_t dumy1; // dummy1 = 4
	uint8_t dumy2;  // dummy1 = 12
	uint8_t cmd[30];
	uint16_t short_addr;
} touch_data_t;

typedef struct {
	uint8_t last_status;
	uint16_t counter;
	uint16_t counter_update;
	uint8_t status;
	uint8_t touch_sens[NUM_TOUCH];
} touch_t;

touch_t touch;

int zb_znp::zigbee_message_handler(zigbee_msg_t& zigbee_msg) {
	/* zigbee start debug message */
	Serial.print("[ZB msg] len: ");
	Serial.print(zigbee_msg.len);
	Serial.print(" cmd0: ");
	Serial.print(zigbee_msg.cmd0, HEX);
	Serial.print(" cmd1: ");
	Serial.print(zigbee_msg.cmd1, HEX);
	Serial.print(" data: ");
	for (int i = 0; i < zigbee_msg.len; i++) {
		Serial.print(zigbee_msg.data[i], HEX);
		Serial.print(" ");
	}
	Serial.println("");
	/* zigbee stop debug message */

	uint16_t zigbee_cmd = BUILD_UINT16(zigbee_msg.cmd1, zigbee_msg.cmd0);

	switch(zigbee_cmd) {
	case ZDO_MGMT_LEAVE_REQ: {
		Serial.println("ZDO_MGMT_LEAVE_REQ");
	}
		break;

	case ZB_RECEIVE_DATA_INDICATION: {
		Serial.println("ZB_RECEIVE_DATA_INDICATION");
	}
		break;

	case ZDO_MGMT_PERMIT_JOIN_RSP: {
		Serial.println("ZDO_MGMT_PERMIT_JOIN_RSP");
		ZdoMgmtPermitJoinRspInd_t* ZdoMgmtPermitJoinRspInd = (ZdoMgmtPermitJoinRspInd_t*)zigbee_msg.data;
		Serial.print("\tsrcaddr: ");
		Serial.println(ZdoMgmtPermitJoinRspInd->srcaddr);
		Serial.print("\tstatus: ");
		Serial.println(ZdoMgmtPermitJoinRspInd->status);
	}
		break;

	case ZDO_TC_DEV_IND: {
		Serial.println("ZDO_TC_DEV_IND");
	}
		break;

	case AF_DATA_REQUEST_IND: {
		Serial.println("AF_DATA_REQUEST_IND");
		uint8_t* status = (uint8_t*)zigbee_msg.data;
		Serial.print("\tstatus: ");
		Serial.println(*status);
	}
		break;

	case AF_DATA_CONFIRM: {
		Serial.println("AF_DATA_CONFIRM");
		afDataConfirm_t* afDataConfirm = (afDataConfirm_t*)zigbee_msg.data;
		Serial.print("\tstatus: ");
		Serial.println(afDataConfirm->status);
		Serial.print("\tendpoint: ");
		Serial.println(afDataConfirm->endpoint);
		Serial.print("\ttransID: ");
		Serial.println(afDataConfirm->transID);
	}
		break;

	case AF_INCOMING_MSG: {
		afIncomingMSGPacket_t* st_af_incoming_msg = (afIncomingMSGPacket_t*)zigbee_msg.data;
		Serial.println("AF_INCOMING_MSG");

#if defined (DBG_ZB_FRAME)
		char buf[9];
		char buf1[18];
		Serial.print("group_id: ");
		sprintf(buf, "%04x", st_af_incoming_msg->group_id);
		Serial.println(buf);

		Serial.print("cluster_id: ");
		sprintf(buf, "%04x", st_af_incoming_msg->cluster_id);
		Serial.println(buf);

		Serial.print("src_addr: ");
		sprintf(buf, "%04x", st_af_incoming_msg->src_addr);
		Serial.println(buf);

		Serial.print("src_endpoint: ");
		Serial.println(st_af_incoming_msg->src_endpoint, HEX);

		Serial.print("dst_endpoint: ");
		Serial.println(st_af_incoming_msg->dst_endpoint, HEX);

		Serial.print("was_broadcast: ");
		Serial.println(st_af_incoming_msg->was_broadcast, HEX);

		Serial.print("link_quality: ");
		Serial.println(st_af_incoming_msg->link_quality, HEX);

		Serial.print("security_use: ");
		Serial.println(st_af_incoming_msg->security_use, HEX);

		Serial.print("time_stamp: ");
		sprintf(buf1, "%08x", st_af_incoming_msg->time_stamp);
		Serial.println(buf1);

		Serial.print("trans_seq_num: ");
		Serial.println(st_af_incoming_msg->trans_seq_num, HEX);

		Serial.print("len: ");
		Serial.println(st_af_incoming_msg->len, HEX);

		Serial.print("data: ");
		for (int i = 0 ; i < st_af_incoming_msg->len ; i++) {
			Serial.print(st_af_incoming_msg->payload[i], HEX);
			Serial.print(" ");
		}
		Serial.println(" ");
#endif
		switch (st_af_incoming_msg->cluster_id) {
		case ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY: {
			Serial.println("ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY");
			uint16_t retHum = (uint16_t) ((st_af_incoming_msg->payload[st_af_incoming_msg->len - 1] * 256) + \
					st_af_incoming_msg->payload[st_af_incoming_msg->len - 2]);

			// Ví dụ: retHum = 6789, thì giá trị trả về là 67,89 %
			Serial.print(retHum / 100); // Lấy Trước dấu phẩy -> 67
			Serial.print(",");
			Serial.println(retHum % 100); // Lấy sau dấu phẩy -> 89
		}
			break;

		case ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT: {
			Serial.println("ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT");
			uint16_t retTemp = (uint16_t) ((st_af_incoming_msg->payload[st_af_incoming_msg->len - 1] * 256) + \
					st_af_incoming_msg->payload[st_af_incoming_msg->len - 2]);

			// Ví dụ: retTemp = 2723, thì giá trị trả về là 27,23 *C
			Serial.print(retTemp/100); // Lấy Trước dấu phẩy -> 27
			Serial.print(",");
			Serial.println(retTemp%100); // Lấy sau dấu phẩy -> 23
		}
			break;

		case ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING: {
			Serial.println("ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING");
			uint8_t retOccu = st_af_incoming_msg->payload[st_af_incoming_msg->len - 1];
			Serial.println(retOccu);
		}
			break;

		case ZCL_CLUSTER_ID_GEN_BASIC:
			Serial.println("ZCL_CLUSTER_ID_GEN_ON_OFF");
			uint8_t retGenOnOff;
			if (st_af_incoming_msg->len > 9) {
				control_switch_address = st_af_incoming_msg->src_addr;
				//				retGenOnOff = st_af_incoming_msg->payload[st_af_incoming_msg->len - 8];
				retGenOnOff = st_af_incoming_msg->payload[9];
				Serial.print("data: ");
				for (int i = 0 ; i < st_af_incoming_msg->len ; i++) {
					Serial.print(st_af_incoming_msg->payload[i]);
					Serial.print(" ");
				}
				Serial.println("");
				Serial.println(retGenOnOff);
				if(retGenOnOff == 0) {
					digitalWrite(led, LOW);
				}
				else if(retGenOnOff == 1) {
					digitalWrite(led, HIGH);
				}
			}
			else {
				Serial.print("data: ");
				for (int i = 0 ; i < st_af_incoming_msg->len ; i++) {
					Serial.print(st_af_incoming_msg->payload[i]);
					Serial.print(" ");
				}
				retGenOnOff = st_af_incoming_msg->payload[st_af_incoming_msg->len - 1];
				Serial.println(retGenOnOff);
			}
			break;

		default:
			break;
		}
	}
		break;

	case ZDO_MGMT_LEAVE_RSP: {
		Serial.println("ZDO_MGMT_LEAVE_RSP");
	}
		break;
	}
}

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
	cli();														// tắt ngắt toàn cục
	/* Reset Timer/Counter1 */
	TCCR1A = 0;
	TCCR1B = 0;
	TIMSK1 = 0;

	/* Setup Timer/Counter1 */
	TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);			//	prescale = 1024 and CTC mode 4
	// timer 1 = 100ms
	OCR1A = 1562.5;												/* initialize OCR1A (Arduino runs at 16Mhz -> 1024/16Mhz = 6.4x10^-5)
																	T/C1 counts from 0 to 65535 is  6.4x10^-5 * 65535 = 4.19424s
																	We need to creat 3s so be content with 4.19424s (3s < 4.19424s) -> 3s/6.4x10^-5 = 46875 count.*/
	TIMSK1 = (1 << OCIE1A);
	sei();														// cho phép ngắt toàn cục

	while (!Serial) { // needed to keep leonardo/micro from starting too fast!
		delay(10);
	}

	if (!cap.begin(0x5A)) {
		Serial.println("MPR121 not found, check wiring?");
		while (1);
	}
	Serial.println("MPR121 found!");

	pinMode(led, OUTPUT);

	/* Khởi động router */
	Serial.println("\nstart_router");
	/* (opt = 0)
	 * to normal start router, and keep configures.
	 * (opt = 1)
	 * to force start router, reset configures to default.
	 * (opt = 2)
	 * to auto start router.
	 */
	if (zigbee_network.start_router(2) == 0) {
		Serial.println("start router successfully");
	}
	else {
		Serial.println("start router error");
	}

}

/* ký tự tạm để xử lý yêu cầu từ terminal */
char serial_cmd;

void loop() {
	/* hàm update() phải được gọi trong vòng lặp để xử lý các gói tin nhận được từ ZigBee Shield */
	zigbee_network.update();
	currtouched = cap.touched();
	/* Kiểm tra / thực hiện các lệnh từ terminal */
	if (Serial.available()) {
		serial_cmd = Serial.read();

		switch(serial_cmd) {
		/* Gửi request cho router join vào coordinator */
		case '1': {
			Serial.println("bdb_start_commissioning");
			zigbee_network.bdb_start_commissioning(COMMISSIONING_MODE_STEERING, 1, 1);
		}
			break;
		case '2': {
			Serial.println("person count down -> send data");
			zigbee_send_data_down();
		}
			break;
		case '3': {
			Serial.println("person count up -> send data");
			zigbee_send_data_up();
		}
		default:
			break;
		}
	}

	if (interupt_timer_100ms_count) {
		cli();
		interupt_timer_100ms_count = 0;
		sei();
		task_timer_100ms();
	}
	task_zigbee();

}

void test_touch() {
	for (uint8_t i = 0; i <= NUM_TOUCH; i++) {
		touch.touch_sens[i] = (!currtouched & _BV(i));
	}
	for(uint8_t j = 0; j <= NUM_TOUCH; j++) {
		Serial.print(touch.touch_sens[j]);
		Serial.print("\t");
	}
	Serial.println(" ");
}

void task_timer_100ms() {
	for (uint8_t i = 1; i <= NUM_TOUCH; i++) {
		touch.touch_sens[i] = (currtouched & _BV(i));
	}

	if ((touch.touch_sens[4] > TOUCH_SENSOR_MILESTONE) || (touch.touch_sens[5] > TOUCH_SENSOR_MILESTONE) \
			|| (touch.touch_sens[6] > TOUCH_SENSOR_MILESTONE) || (touch.touch_sens[7] > TOUCH_SENSOR_MILESTONE)) {

		touch.last_status = 1;
	}
	else if ((touch.touch_sens[4] < TOUCH_SENSOR_MILESTONE) && (touch.touch_sens[5] < TOUCH_SENSOR_MILESTONE) \
			 && (touch.touch_sens[6] < TOUCH_SENSOR_MILESTONE) && (touch.touch_sens[7] < TOUCH_SENSOR_MILESTONE)) {

		touch.last_status = 0;
	}

	if (touch.last_status == 1) {
		touch.counter ++;
		touch.counter_update ++;

		if (touch.counter > TOUCH_COUNTER_MAX) {
			touch.counter = 0;
			touch.status = touch.last_status;
		}

		if(touch.counter_update > TOUCH_COUNTER_UPDATE) {
			touch.counter_update = 0;
			touch.status = 3;
			Serial.println("THE SYSTEM'S ACTIVE ...");
			Serial.println("Status: Chair exist ...");

		}
	}

	if (touch.last_status == 0) {
		touch.counter ++;
		touch.counter_update ++;

		if (touch.counter > TOUCH_COUNTER_MAX) {
			touch.counter = 0;
			touch.status = touch.last_status;
		}

		if(touch.counter_update > TOUCH_COUNTER_UPDATE) {
			touch.counter_update = 0;
			touch.status = 3;
			Serial.println("THE SYSTEM'S ACTIVE ...");
			Serial.println("Status: Chair empty ...");

		}
	}

	if (debug == 1) {
		debug_count++;

		if (debug_count == 10) {
			debug_count = 0;
			Serial.print("counter: ");
			Serial.print(touch.counter);
			Serial.print("\t");
			Serial.print("counter_update: ");
			Serial.print(touch.counter_update);
			Serial.print("\t");
			for (uint8_t i = 4; i <= 7; i++) {
				Serial.print(touch.touch_sens[i]);
				Serial.print("\t");
			}
			Serial.println(" ");
		}
	}
}

void task_zigbee() {
	if (send_status != touch.status) {
		send_status = touch.status;
		if (send_status == 1) {
			Serial.println("person count down -> send data");
			zigbee_send_data_down();
		}
		else if (send_status == 0) {
			Serial.println("person count up -> send data");
			zigbee_send_data_up();
		}
	}
}
void zigbee_data_structure_set(uint8_t * zigbee_package, uint16_t sig, uint8_t * data, uint8_t len) {
	if (len > 120) {
		Serial.println("len > 120");
	}

	for (uint8_t index = 0; index < 120; index++) {
		zigbee_package[index] = 0;
	}

	zigbee_package[0] = 4;
	zigbee_package[1] = 0;
	zigbee_package[2] = 12;

	zigbee_package[3]  = 0x00;
	zigbee_package[4]  = 0x0A;
	zigbee_package[5]  = HI_UINT16(sig);
	zigbee_package[6]  = LO_UINT16(sig);
	zigbee_package[7]  = 0x41;
	zigbee_package[8]  = len;

	memcpy((uint8_t* )&zigbee_package[9], data, len);

}

void zigbee_send_data_up() {
	uint8_t zigbee_data[ZIGBEE_PACKAGE_MAX_LEN];
	uint8_t data[1];

	data[0] = 0;

	zigbee_data_structure_set((uint8_t *)&zigbee_data[0], ATTRID_XIAOMI_SENS_STATUS_ON_OFF,\
			(uint8_t *)&data[0], 1);

	Serial.println(" ");

	static uint8_t trans_id_time = 0;

	af_data_request_t st_af_data_request;
	st_af_data_request.cluster_id    = ZCL_CLUSTER_ID_MS_IOT_LAB_SMART_CHAIR_DEVICE;
	st_af_data_request.dst_address   = 0000;
	st_af_data_request.dst_endpoint  = 0x01;
	st_af_data_request.src_endpoint  = 0x01;
	st_af_data_request.trans_id      = trans_id_time++;
	st_af_data_request.options       = 0x10;
	st_af_data_request.radius        = 0x0F;
	st_af_data_request.len           = sizeof(uint8_t) + 9;
	st_af_data_request.data          = zigbee_data;

	zigbee_network.send_af_data_req(st_af_data_request);
}

void zigbee_send_data_down() {
	uint8_t zigbee_data[ZIGBEE_PACKAGE_MAX_LEN];
	uint8_t data[1];

	data[0] = 1;

	zigbee_data_structure_set((uint8_t *)&zigbee_data[0], ATTRID_XIAOMI_SENS_STATUS_ON_OFF,\
			(uint8_t *)&data[0], 1);

	Serial.println(" ");

	static uint8_t trans_id_time = 0;

	af_data_request_t st_af_data_request;
	st_af_data_request.cluster_id    = ZCL_CLUSTER_ID_MS_IOT_LAB_SMART_CHAIR_DEVICE;
	st_af_data_request.dst_address   = 0000;
	st_af_data_request.dst_endpoint  = 0x01;
	st_af_data_request.src_endpoint  = 0x01;
	st_af_data_request.trans_id      = trans_id_time++;
	st_af_data_request.options       = 0x10;
	st_af_data_request.radius        = 0x0F;
	st_af_data_request.len           = sizeof(uint8_t) + 9;
	st_af_data_request.data          = zigbee_data;

	zigbee_network.send_af_data_req(st_af_data_request);
}

ISR (TIMER1_COMPA_vect) {
	cli();
	interupt_timer_100ms_count++;
	sei();
}
