#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"

// PINOUT
#define UART0_TX_PIN  17
#define UART0_RX_PIN  16

// ------------------------------------------------
// my_uart.h
// ------------------------------------------------
// BUFFER SIZE
#define UART_BUFFER_SIZE	64
#define RING_BF_MASK		(UART_BUFFER_SIZE - 1)

// UART HANDLE
// UART_NUM_0, UART_NUM_1, UART_NUM_2
typedef uart_port_t uart_handle_t;

// UARTS LIST
// typedef enum uarts {
// 	UART0,
// 	UART1,
// 	UART2
// } n_uart_t;

// RING BUFFER
typedef struct {
	char bf[UART_BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	//uint8_t count;
} ring_buffer_t;

// INIT

/// @brief Funcion para inicializar la UART con baudrate como parametro (los pines se definen por tags)
/// 
/// #define UARTn_TX_PIN, UARTn_RX_PIN, UARTn_RTS_PIN, UARTn_CTS_PIN
/// @param uart UART_NUM_0, UART_NUM_1, UART_NUM_2
/// @param baudrate Baudrate
void uart_init(uart_handle_t uart, uint32_t baudrate);

// void uart_enable_irq(uart_handle_t uart, n_uart_t n);
/// @brief Registro del callback como ISR
/// @param uart UART_NUM_0, UART_NUM_1, UART_NUM_2
void uart_enable_irq(uart_handle_t uart);

// READ
uint8_t uart_new_line(void);
uint8_t uart_getc(void);

// WRITE
void uart_write_blocking(uart_handle_t uart, const char *ptr);
void uart_write(uart_handle_t uart, char *bf);

// RING BUFFERS
void buffer_push(volatile ring_buffer_t *rb, char c);
char buffer_pop(volatile ring_buffer_t *rb);
// Teraterm manda 0x08 (ASCII BS).
// Algunos terminales mandan 0x7F (DEL).
// Conviene manejar los dos:
uint8_t buffer_unpush(volatile ring_buffer_t *rb);

// GET LL UART INSTANCE
static inline uart_dev_t *get_uart_instance(uart_handle_t uart);
// ------------------------------------------------

// ------------------------------------------------
// my_uart.c
// ------------------------------------------------
// VARIABLES DE LA LIBRERIA
volatile ring_buffer_t rx_buffer;
volatile ring_buffer_t tx_buffer;
volatile uint8_t flag_new_line = 0;

// Handle de las ISR
static intr_handle_t uart0_isr_handle;
static intr_handle_t uart1_isr_handle;
static intr_handle_t uart2_isr_handle;
static intr_handle_t uart_isr_handle[3];

void uart_init(uart_handle_t uart, uint32_t baudrate)
{
    // config de UART
    uart_config_t config = {
        .baud_rate  = baudrate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,   // análogo a CLOCK_GetFreq()
    };
    uart_param_config(uart, &config);
    // mapeo GPIO a signals de la UART
    uart_set_pin(uart, UART0_TX_PIN, UART0_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Registro del ISR (la parte más diferente)

// En el LPC845 el ISR es un símbolo débil que el linker resuelve automáticamente. 
// En ESP-IDF hay que registrarlo explícitamente, y hay una restricción importante: 
// uart_isr_register() no puede usarse si antes llamaste a uart_driver_install() 
// — son mutuamente excluyentes.
void uart_enable_irq(uart_handle_t uart)
{
    // Habilito RX interrupts a nivel de periférico
    // (RXFIFO_FULL + RXFIFO_TOUT, análogo a kUSART_RxReadyInterruptEnable)
    
    // El primer argumento de todas las funciones uart_ll_* es uart_dev_t *hw: 
    // en la práctica se usa &UART0, &UART1 o &UART2 (instancias globales del SDK).
    uart_dev_t *uart_n = get_uart_instance(uart);

    uart_ll_ena_intr_mask(uart_n,
        UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);

    // Registro ISR + habilito en el controlador de interrupciones del Xtensa
    // Análogo a NVIC_EnableIRQ(USART0_IRQn)
    // ESP_INTR_FLAG_IRAM → handler en IRAM, seguro ante cache miss de flash
    
    // UART0
    uart_isr_register(uart,
                      ISR_uart0,   // CALLBACK a asignar
                      NULL,
                      ESP_INTR_FLAG_IRAM,
                      &uart_isr_handle[uart]);
    /*
    // UART1
    uart_isr_register(uart,
                      ISR_uart1,   // CALLBACK a asignar
                      NULL,
                      ESP_INTR_FLAG_IRAM,
                      &uart_isr_handle[uart]);
    */
    /*
    // UART2
    uart_isr_register(uart,
                      ISR_uart2,   // CALLBACK a asignar
                      NULL,
                      ESP_INTR_FLAG_IRAM,
                      &uart_isr_handle[uart]);
    */                  
}

// IRAM_ATTR obligatorio si usás ESP_INTR_FLAG_IRAM

void IRAM_ATTR ISR_uart0(void *arg)
{
    uart_dev_t *n = &UART0;
    uint32_t status = uart_ll_get_intsts_mask(n);

    // --- RX: FIFO lleno O timeout ---
    if (status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)) {

        // Dreno TODO el FIFO (diferencia clave vs LPC845)
        uint32_t rx_len = uart_ll_get_rxfifo_len(n);
        while (rx_len--) {
            uint8_t c;
            uart_ll_read_rxfifo(n, &c, 1);   // análogo a USART_ReadByte()
            // ... misma lógica de CR/LF/BS/echo que tenés ...
            buffer_push(&rx_buffer, c);
        }
        uart_ll_clr_intsts_mask(n,
            UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    }

    // --- TX: FIFO vacío ---
    if (status & UART_INTR_TXFIFO_EMPTY) {
        uint32_t tx_free = uart_ll_get_txfifo_len(n);  // espacio libre

        while (tx_free--) {
            char c = buffer_pop(&tx_buffer);
            if (!c) {
                // No hay más datos: deshabilito IRQ de TX
                uart_disable_tx_intr(UART_NUM_0);       // análogo a USART_DisableInterrupts
                break;
            }
            uart_ll_write_txfifo(n, (uint8_t*)&c, 1); // análogo a USART_WriteByte()
        }
        uart_ll_clr_intsts_mask(n, UART_INTR_TXFIFO_EMPTY);
    }

    // --- Errores ---
    if (status & (UART_INTR_FRAM_ERR | UART_INTR_PARITY_ERR | UART_INTR_RXFIFO_OVF)) {
        uart_ll_clr_intsts_mask(n,
            UART_INTR_FRAM_ERR | UART_INTR_PARITY_ERR | UART_INTR_RXFIFO_OVF);
        // En OVF hay que hacer reset del FIFO:
        uart_ll_rxfifo_rst(n);
    }
}

uint8_t inline uart_new_line(void)
{
	if (flag_new_line) {
		flag_new_line = 0;
		return 1;
	}
	else return 0;
}

uint8_t uart_getc(void)
{
	return buffer_pop(&rx_buffer);
}

void uart_write_blocking(uart_handle_t uart, const char *ptr)
{
    uart_dev_t *n = get_uart_instance(uart);

    while (*ptr != '\0') {
        // Espera espacio libre en FIFO TX
        // Análogo a: while (!(flags & kUSART_TxReady))
        // SOC_UART_FIFO_LEN = 128 bytes (ESP32)
        while (uart_ll_get_txfifo_len(n) >= SOC_UART_FIFO_LEN);

        // Escribe 1 byte al FIFO
        // Análogo a: USART_WriteByte(uart, *ptr)
        uart_ll_write_txfifo(n, (const uint8_t *)ptr, 1);
        ptr++;
    }

    // Espera que el último byte salió completo del shift register
    // Análogo a: while (!(flags & kUSART_TxIdleFlag))
    while (!uart_ll_is_tx_idle(n));
    // Nota: uart_ll_get_txfifo_len() devuelve bytes usados en el FIFO 
    // (al revés de lo que el nombre sugiere). 
    // Por eso la condición es >= SOC_UART_FIFO_LEN y no == 0.
}

/* uart_write()
La diferencia conceptual está en kUSART_TxReadyInterruptEnable. 
En el LPC845, esa IRQ dispara cuando el registro de 1 byte está listo. 
En el ESP32 es UART_INTR_TXFIFO_EMPTY con un umbral (thresh):
    thresh = 0 → interrupt dispara cuando el FIFO tiene ≤ 0 bytes, o sea cuando está vacío
    Si el FIFO ya está vacío al momento de uart_enable_tx_intr(), 
    la IRQ dispara inmediatamente → mismo kick-start que en el LPC845
Y en el ISR, la contraparte uart_disable_tx_intr() reemplaza a USART_DisableInterrupts().
*/
void uart_write(uart_handle_t uart, char *ptr)
{
    // Copia al ring buffer — idéntico al LPC845
    while (*ptr != '\0') {
        buffer_push(&tx_buffer, *ptr);
        ptr++;
    }

    // Habilito TX IRQ
    // Análogo a: USART_EnableInterrupts(uart, kUSART_TxReadyInterruptEnable)
    // thresh=0 → dispara cuando FIFO vacío → si ya está vacío, dispara ahora
    uart_enable_tx_intr(uart, /*enable=*/1, /*thresh=*/0);
}

void buffer_push(volatile ring_buffer_t *rb, char c)
{
	// calculo la posicion del nuevo head
	uint8_t next_head = (rb->head + 1) & RING_BF_MASK;
	// uint8_t next_head = (rb->head + 1) % UART_BUFFER_SIZE;
	// si next apunta a tail, el buffer esta lleno
	if (next_head == rb->tail) {
		// overflow
		return;
	}
	// copio caracter
	rb->bf[rb->head] = c;
	// actualizo head
	rb->head = next_head;
	return;
}

char buffer_pop(volatile ring_buffer_t *rb)
{
	// si la queue esta vacia retorno caracter nulo
	if (rb->tail == rb->head) return '\0';
	// leo caracter
	char c = rb->bf[rb->tail];
	// calculo nueva posicion de tail
	rb->tail = (rb->tail + 1) & RING_BF_MASK;
	//rb->tail = (rb->tail + 1) % UART_BUFFER_SIZE;
	// retorno caracter leido
	return c;
}

// Retorna 1 si había algo para borrar, 0 si el buffer estaba vacío
uint8_t buffer_unpush(volatile ring_buffer_t *rb)
{
    if (rb->head == rb->tail) return 0;  // nada que borrar
    rb->head = (rb->head - 1) & RING_BF_MASK;
    // rb->head = (rb->head - 1) % UART_BUFFER_SIZE;
    return 1;
}

// Helper interno - devuelve la uart instance para las funciones uart_ll_()
static inline uart_dev_t *get_uart_instance(uart_handle_t uart)
{
    // Equivalente al puntero USART_Type* del LPC845
    static uart_dev_t *uart_map[] = {&UART0, &UART1, &UART2};
    return uart_map[uart];
}
// ------------------------------------------------

void app_main() 
{
    // Mapeo pines de UART y configuro
    uart_init(0, 9600);

    char c = 0;
	uint8_t i = 0;
	char bf[UART_BUFFER_SIZE];
	char prompt[] = ">> ";
	uart_write(0, prompt);

    // Habilito IRQ de RX
    uart_enable_irq(0);

    // LOOP DE EJECUCION
    while (1) {
        // Chequeo si termine de recibir una linea
		if (uart_new_line()) {
			// terminacion de linea CRLF
			uart_write(0, "\r\n");

			// Copio rx_buffer a buffer local
			i = 0;
			// lectura inicial
			c = uart_getc();
			while (c) {
				// copio caracter
				bf[i] = c;
				i++;
				// vuelvo a leer
				c = uart_getc();
			}
			// aseguro terminacion de string
			bf[i] = '\0';

			if (bf[0] != '\0') {
				// Comparo y escribo en uart
				if (strcmp(bf, "ping") == 0) {
					uart_write(0, "< PONG\r\n");
				}
				else uart_write(0, "< NACK\r\n");
			}
			// escribo prompt
			uart_write(0, prompt);
		}
    }
}
