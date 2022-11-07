#include "RTE_Components.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "mcu.h"
#include "mongoose.h"

static void uart_logger(char c, void *param) {
  uart_write_byte(param, c);
}

uint64_t mg_millis(void) {  // Declare our own uptime function
    return osKernelGetTickCount();
}

static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_HTTP_MSG) {
	  struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    if (mg_http_match_uri(hm, "/api/debug")) {
      int level = mg_json_get_long(hm->body, "$.level", MG_LL_DEBUG);
      mg_log_set(level);
      mg_http_reply(c, 200, "", "Debug level set to %d\n", level);
		} else {
			mg_http_reply(c, 200, "", "%s\n", "hi");
		}
  }
}

static void mytask(void * param) {
	uint16_t led = PIN('B', 7);
	gpio_set_mode(led, GPIO_MODE_OUTPUT);
	mg_log_set_fn(uart_logger, UART3);
	
	netInitialize();
	
  struct mg_mgr mgr;
  mg_mgr_init(&mgr);   
  mg_http_listen(&mgr, "http://0.0.0.0:80", fn, &mgr);

  for (;;) {
		//MG_INFO(("poll... %u", 123));
		gpio_toggle(led);
    mg_mgr_poll(&mgr, 750);
		//	mg_connect(&mgr, "tcp://192.168.0.66:1234", fn, &mgr);

  }
	
	for (;;) {
		//uart_write_buf(UART3, "hi\r\n", 4);
		MG_INFO(("hi... %u", HAL_GetTick()));
		gpio_toggle(led);
		osDelay(200);
	}
}

int main(void) {
	//HAL_Init();
	clock_init();
	systick_init(FREQ / 1000);
	uart_init(UART3, 115200);
	
  osKernelInitialize ();
  size_t stack_size = 4096;
  void *stk = malloc(stack_size);
  const osThreadAttr_t attr = { .stack_mem  = stk, .stack_size = stack_size };
  osThreadNew(mytask, NULL, &attr);
  osKernelStart();
}