#include "zephyr/sys/printk.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <sys/errno.h>
#include <ctype.h>


LOG_MODULE_REGISTER(nicerf_sa868);

static const struct device *chgdev = DEVICE_DT_GET(DT_NODELABEL(charger));
// static const struct device *trxdev = DEVICE_DT_GET(DT_NODELABEL(transceiver));

// for now, just use the zephyr shell UART.
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
static const struct device *uartdev = DEVICE_DT_GET(UART_DEVICE_NODE);

char sa868_ctdss_code[100][4] = {
	"023I", "025I", "026I", "031I", "032I", "043I", "047I", "051I", "054I", "065I", 
	"071I", "072I", "073I", "074I", "114I", "115I", "116I", "125I", "131I", "132I", 
	"134I", "143I", "152I", "155I", "156I", "162I", "165I", "172I", "174I", "205I",
	"223I", "226I", "243I", "244I", "245I", "251I", "261I", "263I", "265I", "271I",
	"306I", "311I", "315I", "331I", "343I", "346I", "351I", "364I", "365I", "371I", 
	"411I", "412I", "413I", "423I", "431I", "432I", "445I", "464I", "465I", "466I",
	"503I", "506I", "516I", "532I", "546I", "565I", "606I", "612I", "624I", "627I", 
	"631I", "632I", "654I", "662I", "664I", "703I", "712I", "723I", "731I", "732I", 
	"734I", "743I", "754I"
};

uint32_t sa868_ctdss_pattern[100] = {
	0x640E37, 0x540F6B, 0x340DD3, 0x4C0FC5, 0x2C0D7D, 0x620B6D, 0x720DF8, 0x4A0A9F,
	0x1A097B, 0x560C5D, 0x4E0CF3, 0x2E0E4B, 0x6E0B3A, 0x1E0F17, 0x190BD6, 0x590EA7, 
	0x390C1F, 0x550EF0, 0x4D0E5E, 0x2D0CE6, 0x1D0DBA, 0x630AF6, 0x2B09BC, 0x5B0D91, 
	0x3B0F29, 0x2709EB, 0x570DC6, 0x2F0FD0, 0x1F0E8C, 0x508CBB, 0x648B8B, 0x34886F, 
	0x628ED1, 0x128AFC, 0x528F8D, 0x4A8F23, 0x468F74, 0x6688BD, 0x5689E1, 0x4E894F, 
	0x318F98, 0x498D8E, 0x598B1B, 0x4D8BE2, 0x638F4A, 0x338CAE, 0x4B8EB8, 0x178D0B, 
	0x57887A, 0x4F88D4, 0x484B77, 0x2849CF, 0x684CBE, 0x644CE9, 0x4C4D1B, 0x2C4FA3, 
	0x5248EF, 0x164BF2, 0x564E83, 0x364C3B, 0x614B1E, 0x3148FA, 0x394EC1, 0x2D4E38, 
	0x334BCC, 0x574F18, 0x30CCDD, 0x28CC73, 0x14CD78, 0x74CFC0, 0x4CC8A7, 0x2CCA1F, 
	0x1ACE19, 0x26CF12, 0x16CE4E, 0x61CEA2, 0x29CDE8, 0x65C8CE, 0x4DC93C, 0x2DCB84, 
	0x1DCAD8, 0x63CD94, 0x1BCF82
};

/* 
* CONNECT: Detect the module is in normal working status.
* SWEEP: Set the frequency to scan
* SETGROUP: Set the working parameters of the module
* SETVOLUME: Set the volume level of the module
* RSSI: Read Received Signal Strength Indicator value of the module
*/
enum instruction_set {
	CONNECT = 0, // AT+DMOCONNECT
	SWEEP, // S+SWEEP=?
	SETGROUP, // AT+DMOSETGROUP=?
	SETVOLUME, // AT+DMOSETVOLUME=?
	RSSI, // AT+RSSI?
	SETFILTER // AT+SETFILTER=?
};

#define MSG_SIZE 50
/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uartdev)) {
		return;
	}

	if (!uart_irq_rx_ready(uartdev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uartdev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

int read_in_response(enum instruction_set instruction, char *response) {
	int val = -1; // command response value begins invalid
	switch(instruction) {
		case(CONNECT):
			// +DMOCONNECT:0 -> Normal working status
			sscanf(response, "+DMOCONNECT:%d\r\n", &val);
			break;
		case(SWEEP):
			// S=0 -> The frequency to be scanned has a signal
			// S=1 -> There is no signal to sweep the frequency
			sscanf(response, "S=%d\r\n", &val);
			break;
		case(SETGROUP):
			// +DMOSETGROUP:0 -> Successfully set working parameters
			// +DMOSETGROUP:1 -> Data setting is out of range
			sscanf(response, "+DMOSETGROUP:%d\r\n", &val);
			break;
		case(SETVOLUME):
			// +DMOSETVOLUME:0 -> Successfully set volume level
			// +DMOSETVOLUME:1 -> Failed volume level setup
			sscanf(response, "+DMOSETVOLUME:%d\r\n", &val);
			break;
		case(RSSI):
			// RSSI:0bXXX -> Current signal strength value
			sscanf(response, "RSSI:%3d", &val);
			break;
		case(SETFILTER):
			// +DMOSETFILTER:0 -> Successfully set module filter
			// +DMOSETFILTER:1 -> Failed filter setup 
			sscanf(response, "+DMOSETFILTER:%d\r\n", &val);
			break;
	}
	return val;
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uartdev, buf[i]);
	}
}

int main(void)
{
	/*
	 * Texas Instruments BQ24190 initialization
	 */
	if (chgdev == NULL) {
		printk("Error: no charger device found.\n");
		return 0;
	}

	if (!device_is_ready(chgdev)) {
		printk("Error: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       chgdev->name);
		return 0;
	}

	/*
	 * NiceRF SA868 initialization
	 */
	// https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/drivers/uart/echo_bot/src/main.c

	if (!device_is_ready(uartdev)) {
		printk("Error: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       uartdev->name);
    	return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uartdev, serial_cb, NULL);
	
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uartdev);

	/* send data over the UART*/

	//char queuedCommand[50] = "AT+DMOSETGROUP=0,415.1250,415.1250,0012,4,0013\r\n";

	char queuedCommand[17] = "AT+DMOCONNECT\r\n";

	print_uart(queuedCommand);

	// wait for response
	char response[MSG_SIZE];
	while (k_msgq_get(&uart_msgq, &response, K_FOREVER) == 0);

	// read in the response command value

	enum instruction_set instruction = CONNECT;

	int result = read_in_response(instruction, response);

	printk("Response from SA868: %d", result);
	return 0;
}