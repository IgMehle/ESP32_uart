/*
 * ------------------------------------------------
 * uart_esp32_sdk.c — Versión usando el driver de alto nivel de ESP-IDF
 * ------------------------------------------------
 * Comparación con la versión de bajo nivel (uart_ll_* + ISR manual):
 *
 * ANTES (bajo nivel)                            AHORA (driver de alto nivel)
 * --------------------------------------------  --------------------------------------------
 * ring_buffer_t rx_buffer / tx_buffer            Ring buffers internos del driver (RX y TX)
 * uart_isr_register() + ISR_uart0()              uart_driver_install() + cola de eventos
 * flag_new_line + parseo manual de '\r'/'\n'     Parseo de línea en la tarea, evento UART_DATA
 * buffer_unpush() para backspace (0x08/0x7F)     Misma lógica, ahora en la tarea (con echo)
 * uart_getc() / buffer_pop()                     uart_read_bytes()
 * uart_write() + habilitar IRQ de TX a mano      uart_write_bytes() (usa el buffer del driver)
 * while(1) haciendo polling de uart_new_line()   Tarea FreeRTOS bloqueada en xQueueReceive()
 *
 * NOTA sobre echo local: se abandona UART_PATTERN_DET a propósito. El patrón
 * solo avisa cuando la línea ya está completa, y para hacer eco tenemos que
 * reaccionar a cada byte apenas llega (evento UART_DATA), no cuando termina
 * la línea. Por eso acá se vuelve a armar la línea a mano, como en la versión
 * original del LPC845, pero delegando en el driver del ESP-IDF los ring
 * buffers de hardware y el ISR.
 *
 * IMPORTANTE: si tu terminal (PuTTY, TeraTerm, minicom, etc.) tiene su propio
 * "local echo" activado, vas a ver cada carácter DUPLICADO, porque ahora el
 * ESP también lo está devolviendo. Con echo hecho en el firmware, el local
 * echo del terminal debe quedar DESACTIVADO.
 * ------------------------------------------------
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

// PINOUT
// UART2: UART0 queda libre para consola/debug/flasheo (pines 1/3, CP2102).
// En módulos WROVER (con PSRAM), evitar GPIO16/17; en WROOM están libres.
#define UART_PORT       UART_NUM_2
#define UART_TX_PIN     17
#define UART_RX_PIN     16

// BUFFERS (los administra el driver)
#define UART_RX_BUF_SIZE    1024
#define UART_TX_BUF_SIZE    1024
#define UART_QUEUE_SIZE     20
#define UART_LINE_BUF_SIZE  128

// Caracteres de control que manejamos a mano (idéntico criterio al LPC845)
#define CHAR_CR         '\r'
#define CHAR_LF         '\n'
#define CHAR_BS         0x08   // Backspace clásico (TeraTerm, etc.)
#define CHAR_DEL        0x7F   // Delete (algunos terminales lo mandan en vez de BS)

static const char *TAG = "uart_sdk";
static QueueHandle_t uart_queue;
static const char PROMPT[] = ">> ";

// Línea en construcción (reemplaza a tu rx_buffer + flag_new_line)
static char line_buf[UART_LINE_BUF_SIZE];
static uint8_t line_idx = 0;

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

    // Reemplaza tu bloque de ring buffers propios + uart_isr_register():
    // el driver reserva sus propios buffers internos y registra su propio ISR.
    ESP_ERROR_CHECK(uart_driver_install(uart, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
                                         UART_QUEUE_SIZE, &uart_queue, 0));
}

// ------------------------------------------------
// WRITE (bloqueante hasta que entra al buffer de TX del driver)
// ------------------------------------------------
static inline void uart_write(uart_port_t uart, const char *str)
{
    uart_write_bytes(uart, str, strlen(str));
}

static inline void uart_write_char(uart_port_t uart, char c)
{
    uart_write_bytes(uart, &c, 1);
}

// ------------------------------------------------
// Procesa un byte recién llegado: hace echo, maneja backspace,
// y detecta fin de línea. Equivalente directo a la lógica que tenías
// dentro del ISR_uart0 del LPC845, movida acá.
// ------------------------------------------------
static void process_incoming_byte(uart_port_t uart, char c)
{
    if (c == CHAR_CR || c == CHAR_LF) {
        // Fin de línea: eco de CRLF y proceso el comando acumulado.
        // Nota: si el terminal manda CRLF (dos bytes), el segundo byte
        // llega en el próximo evento y cae en este mismo "if" con
        // line_idx ya en 0 -> no rompe nada, solo emite un CRLF de más
        // en el peor caso. Si querés evitarlo, se puede filtrar con un
        // flag "ya until procesé un CR/LF", pero no suele hacer falta.
        uart_write(uart, "\r\n");

        line_buf[line_idx] = '\0';

        if (line_idx > 0) {
            if (strcmp(line_buf, "ping") == 0) {
                uart_write(uart, "< PONG\r\n");
            } else {
                uart_write(uart, "< NACK\r\n");
            }
        }

        line_idx = 0;
        uart_write(uart, PROMPT);
        return;
    }

    if (c == CHAR_BS || c == CHAR_DEL) {
        // Borrar el último carácter, si hay algo que borrar
        if (line_idx > 0) {
            line_idx--;
            // Secuencia visual de borrado: retrocede el cursor, pisa el
            // carácter con un espacio, y retrocede de nuevo.
            uart_write(uart, "\b \b");
        }
        // Si la línea está vacía no hago nada (no tiene sentido "borrar"
        // hacia el prompt).
        return;
    }

    // Carácter normal: lo guardo y hago echo
    if (line_idx < UART_LINE_BUF_SIZE - 1) {
        line_buf[line_idx++] = c;
        uart_write_char(uart, c);
    }
    // Si la línea local se llenó, simplemente descarto el carácter
    // (equivalente a tu chequeo de overflow en buffer_push).
}

// ------------------------------------------------
// TAREA: reemplaza tu while(1) + polling de uart_new_line()
// ------------------------------------------------
static void uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t rx_byte;

    uart_write(UART_PORT, PROMPT);

    for (;;) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (event.type) {

        case UART_DATA:
            // event.size = cantidad de bytes disponibles en el ring buffer
            // del driver. Los leo de a uno para poder hacer echo y manejar
            // backspace en el momento, igual que hacía el ISR original.
            for (int i = 0; i < event.size; i++) {
                int n = uart_read_bytes(UART_PORT, &rx_byte, 1, pdMS_TO_TICKS(20));
                if (n == 1) {
                    process_incoming_byte(UART_PORT, (char)rx_byte);
                }
            }
            break;

        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            ESP_LOGW(TAG, "Overflow de RX, reseteando buffers");
            uart_flush_input(UART_PORT);
            xQueueReset(uart_queue);
            line_idx = 0;
            break;

        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
            ESP_LOGW(TAG, "Error de trama/paridad");
            break;

        default:
            break;
        }
    }
}

// ------------------------------------------------
void app_main(void)
{
    uart_init(UART_PORT, 9600);

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
}
