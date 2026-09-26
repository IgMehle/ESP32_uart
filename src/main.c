/*
 * ------------------------------------------------
 * uart_esp32_sdk.c — Versión usando el driver de alto nivel de ESP-IDF
 * ------------------------------------------------
 * Comparación directa con la versión de bajo nivel (uart_ll_* + ISR manual):
 *
 * ANTES (bajo nivel)                            AHORA (driver de alto nivel)
 * --------------------------------------------  --------------------------------------------
 * ring_buffer_t rx_buffer / tx_buffer            Ring buffers internos del driver (RX y TX)
 * uart_isr_register() + ISR_uart0()              uart_driver_install() + cola de eventos
 * flag_new_line + parseo manual de '\r'/'\n'     UART_PATTERN_DET (el hardware busca el patrón)
 * uart_getc() / buffer_pop()                     uart_read_bytes()
 * uart_write() + habilitar IRQ de TX a mano      uart_write_bytes() (usa el buffer del driver)
 * while(1) haciendo polling de uart_new_line()   Tarea FreeRTOS bloqueada en xQueueReceive()
 * Chequeo manual de RXFIFO_OVF en el ISR         Evento UART_FIFO_OVF / UART_BUFFER_FULL
 *
 * Nota: uart_isr_register() y uart_driver_install() son mutuamente excluyentes.
 * Acá usamos EXCLUSIVAMENTE la API de alto nivel: no se toca uart_ll_* en ningún lado.
 * ------------------------------------------------
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

// PINOUT
// IMPORTANTE: se usa UART2 y NO UART0. UART0 está cableado por hardware a los
// pines GPIO1 (TX) / GPIO3 (RX), que van al CP2102/CH340 de la placa para
// flasheo y consola de debug (logs del IDF, idf.py monitor). Remapear UART0
// a otros pines por software no mueve el chip USB-serie: seguís perdiendo la
// consola porque el periférico deja de hablar por 1/3.
// UART1/UART2 no tienen pines fijos y se asignan libremente por matriz de GPIO.
// Nota: en módulos WROVER (con PSRAM), evitar GPIO16/17 (bus SPI de la PSRAM);
// en WROOM están libres.
#define UART_PORT       UART_NUM_2
#define UART_RX_PIN     16
#define UART_TX_PIN     17

// BUFFERS (los administra el driver, ya no son ring_buffer_t propios)
#define UART_RX_BUF_SIZE    1024
#define UART_TX_BUF_SIZE    1024
#define UART_QUEUE_SIZE     20
#define UART_LINE_BUF_SIZE  128

// Carácter que marca fin de línea (equivalente conceptual a tu flag_new_line)
#define LINE_END_CHAR   '\n'

static const char *TAG = "uart_sdk";
static QueueHandle_t uart_queue;
static const char PROMPT[] = ">> ";

// ------------------------------------------------
// INIT
// ------------------------------------------------
static void uart_init(uart_port_t uart, uint32_t baudrate)
{
    uart_config_t config = {
        .baud_rate  = baudrate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart, &config));
    ESP_ERROR_CHECK(uart_set_pin(uart, UART_TX_PIN, UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Reemplaza TODO tu bloque de ring buffers propios + uart_isr_register():
    // el driver reserva sus propios buffers internos y registra su propio ISR.
    ESP_ERROR_CHECK(uart_driver_install(uart, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
                                         UART_QUEUE_SIZE, &uart_queue, 0));

    // Reemplaza tu flag_new_line + parseo manual de CR/LF dentro del ISR:
    // el hardware busca LINE_END_CHAR solo y avisa por la cola de eventos.
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(uart, LINE_END_CHAR,
                                                       /*chr_num=*/1,
                                                       /*chr_tout=*/9, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(uart, UART_QUEUE_SIZE));
}

// ------------------------------------------------
// WRITE (bloqueante hasta que entra al buffer de TX del driver;
// ya no hace falta un ring buffer de TX propio ni habilitar IRQ a mano)
// ------------------------------------------------
static inline void uart_write(uart_port_t uart, const char *str)
{
    uart_write_bytes(uart, str, strlen(str));
}

// ------------------------------------------------
// TAREA que reemplaza tu while(1) + polling de uart_new_line()
// ------------------------------------------------
static void uart_event_task(void *arg)
{
    uart_event_t event;
    char line_buf[UART_LINE_BUF_SIZE];

    uart_write(UART_PORT, PROMPT);

    for (;;) {
        // Bloquea la tarea hasta que el driver reporte un evento: sin polling.
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (event.type) {

        case UART_PATTERN_DET: {
            // Posición del '\n' dentro del ring buffer interno del driver
            int pos = uart_pattern_pop_pos(UART_PORT);
            if (pos == -1) {
                // La cola de patrones se llenó y el driver perdió la posición:
                // vaciamos el buffer para no quedar desalineados.
                uart_flush_input(UART_PORT);
                break;
            }

            int len = uart_read_bytes(UART_PORT, (uint8_t *)line_buf,
                                       pos, pdMS_TO_TICKS(100));
            line_buf[len] = '\0';

            // Descarto el carácter de fin de línea que quedó en el buffer
            uint8_t discard;
            uart_read_bytes(UART_PORT, &discard, 1, pdMS_TO_TICKS(100));

            // Si el terminal manda CRLF, saco el '\r' final
            if (len > 0 && line_buf[len - 1] == '\r') {
                line_buf[len - 1] = '\0';
            }

            uart_write(UART_PORT, "\r\n");

            if (line_buf[0] != '\0') {
                if (strcmp(line_buf, "ping") == 0) {
                    uart_write(UART_PORT, "< PONG\r\n");
                } else {
                    uart_write(UART_PORT, "< NACK\r\n");
                }
            }

            uart_write(UART_PORT, PROMPT);
            break;
        }

        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            // Equivalente a tu chequeo manual de RXFIFO_OVF en el ISR: acá el
            // driver ya reseteó el FIFO de hardware, nosotros solo vaciamos el
            // buffer lógico y resincronizamos la detección de patrón.
            ESP_LOGW(TAG, "Overflow de RX, reseteando buffers");
            uart_flush_input(UART_PORT);
            xQueueReset(uart_queue);
            uart_pattern_queue_reset(UART_PORT, UART_QUEUE_SIZE);
            break;

        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
            ESP_LOGW(TAG, "Error de trama/paridad");
            break;

        default:
            // UART_DATA llega también mientras se completa la línea, pero como
            // procesamos por UART_PATTERN_DET, lo ignoramos acá.
            break;
        }
    }
}

// ------------------------------------------------
void app_main(void)
{
    uart_init(UART_PORT, 115200);
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
}