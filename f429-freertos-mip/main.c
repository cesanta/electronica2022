// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "mcu.h"
#include "mongoose.h"

// HTTP server event handler function
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
  (void) fn_data;
}

static void server(void *args) {
  // Initialise Ethernet. Enable MAC GPIO pins, see
  // https://www.farnell.com/datasheets/2014265.pdf section 6.10
  uint16_t pins[] = {PIN('A', 1),  PIN('A', 2),  PIN('A', 7),
                     PIN('B', 13), PIN('C', 1),  PIN('C', 4),
                     PIN('C', 5),  PIN('G', 11), PIN('G', 13)};
  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
    gpio_init(pins[i], GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_INSANE,
              GPIO_PULL_NONE, 11);
  }

  NVIC_EnableIRQ(61);                           // Setup Ethernet IRQ handler
  RCC->APB2ENR |= BIT(14);                      // Enable SYSCFG
  SYSCFG->PMC |= BIT(23);                       // Use RMII. Goes first!
  RCC->AHB1ENR |= BIT(25) | BIT(26) | BIT(27);  // Enable Ethernet clocks
  RCC->AHB1RSTR |= BIT(25);                     // ETHMAC force reset
  RCC->AHB1RSTR &= ~BIT(25);                    // ETHMAC release reset

  struct mg_mgr mgr;        // Initialise Mongoose event manager
  mg_mgr_init(&mgr);        // and attach it to the MIP interface
  mg_log_set(MG_LL_DEBUG);  // Set log level

  // Initialise Mongoose network stack
  // Specify MAC address, and use 0 for IP, mask, GW - i.e. use DHCP
  // For static configuration, specify IP/mask/GW in network byte order
  MG_INFO(("Initializing Ethernet driver"));
  struct mip_cfg c = {.mac = {0, 0, 1, 2, 3, 5}, .ip = 0, .mask = 0, .gw = 0};
  struct mip_driver_stm32 driver_data = {.mdc_cr = 4};  // See driver_stm32.h
  mip_init(&mgr, &c, &mip_driver_stm32, &driver_data);

  MG_INFO(("Starting Mongoose v%s", MG_VERSION));    // Tell the world
  mg_http_listen(&mgr, "http://0.0.0.0", fn, &mgr);  // Web listener
  while (args == NULL) mg_mgr_poll(&mgr, 1000);      // Infinite event loop
  mg_mgr_free(&mgr);                                 // Unreachable
}

static void blinker(void *args) {
  uint16_t led = PIN('B', 7);
  gpio_init(led, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_MEDIUM,
            GPIO_PULL_NONE, 0);
  for (;;) {
    gpio_toggle(led);
    vTaskDelay(pdMS_TO_TICKS(750));
    MG_INFO(("blink %s,  RAM: %u", (char *) args, xPortGetFreeHeapSize()));
  }
}

int main(void) {
  clock_init();               // Set clock to max of 180 MHz
  systick_init(FREQ / 1000);  // Tick every 1 ms
  uart_init(UART3, 115200);   // Initialise UART
  xTaskCreate(blinker, "blinker", 512, ":)", configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(server, "server", 8192, 0, configMAX_PRIORITIES - 1, NULL);
  vTaskStartScheduler();  // This blocks
  return 0;
}
