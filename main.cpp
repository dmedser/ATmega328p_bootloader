#include <avr/io.h>
#include <avr/boot.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>

/* Имя и пароль от WiFi сети: AT+CWJAP_DEF="<SSID>","<passwrd>" */
const char *wifi_connection_str = "AT+CWJAP_DEF=\"<SSID>\",\"<password>\"\r\n";
 
/* Establish UDP Transmission
 * Multiple connections (AT+CIPMUX=1):
 *  AT+CIPSTART=<link  ID>,<type>,<remote IP>,<remote port>[,(<UDP  local port>),(<UDP mode>)]
 *  <link ID>: ID of network connection (0~4), used for multiple connections.
 * <type>: string parameter indicating the connection type: "TCP", "UDP" or "SSL".
 * <remote IP>: string parameter indicating the remote IP address.
 * <remote port>: remote port number.
 * [<UDP local port>]: optional; UDP port of ESP8266.
 * [<UDP mode>]: optional. In the UDP transparent transmission, the value of this parameter has to be 0.
 * 0: the destination peer entity of UDP will not change; this is the default setting.
 * 1: the destination peer entity of UDP can change once.
 * 2: the destination peer entity of UDP is allowed to change.
 */
const char *udp_pc_connection_str = "AT+CIPSTART=1,\"UDP\",\"192.168.1.40\",69,69,2\r\n";
     
#define CR_NL                           0x0A0D
#define CR_NL_LEN                       2
#define SHIFT_BUF_LEN                   9
#define TFTP_OPCODE_NONE                0x0000
#define TFTP_OPCODE_RDREQ               0x0100
#define TFTP_OPCODE_WRREQ               0x0200
#define TFTP_OPCODE_DATA                0x0300
#define TFTP_OPCODE_ACK                 0x0400
#define TFTP_OPCODE_ERROR               0x0500
#define TFTP_PCK_MAX_LEN                516
#define IHEX_START_CODE                 ':'
#define IHEX_RECORD_TYPE_DATA           0x00
#define IHEX_RECORD_TYPE_EOF            0x01
#define IHEX_SERVICE_CHAR_COUNT         13
#define IHEX_MAX_LEN			16
#define IHEX_MAX_C_LEN                  45
#define BOOTLOADER_START_ADDRESS        0x7000
#define APP_START_ADDRESS               0x0000
#define F_CPU				(16000000UL)
#define F_TIMER1			(F_CPU / 1024)
#define TICKS_IN_SEC			(F_TIMER1)
#define BOOT_TIMEOUT_SEC		10

void (*jmp_to_app)(void) = APP_START_ADDRESS;

/* Счетчик секунд таймаута выхода из программы бутлоадера */
uint8_t sec_cntr = 0;

/* Буфер страницы для записи в application область flash памяти программ */
uint8_t  pg[SPM_PAGESIZE];
uint8_t  pg_idx = 0;
uint32_t pg_addr = 0;

/* Резервный буфер для хранения строки длиной до 16-ти байт */
uint8_t  pg_str_reserve_buf[IHEX_MAX_LEN];
uint8_t  pg_str_reserve_buf_idx = 0;

typedef enum {
    BOOT_STATE_CTRL,
    BOOT_STATE_ACK,
    BOOT_STATE_DATA,
    BOOT_STATE_ERROR,
    BOOT_STATE_COMPLETE
} boot_state_t;

/* Hex файл состоит из ihex строк, см. https://ru.wikipedia.org/wiki/Intel_HEX */  
typedef struct {
  bool     is_correct;
  bool     len_is_known;
  uint8_t  len;           // Длина, указанная в ihex строке 
  bool     is_full;       // Все байты ihex строки находятся в текущем принятом tftp пакете 
  bool     is_last_in_pck;
  bool     is_eof;
  uint16_t addr;
  uint8_t  crc; 
} ihex_str_t;

/* Сдвиговый буфер для приема символов от ESP8266 */
class shift_buf {
public:
    char buf[SHIFT_BUF_LEN];
    void add_lsh(char val);
    bool is_updated();
    bool contains(char *bytes, uint8_t bytes_num);
    void clr();
};

shift_buf      sb;

boot_state_t   boot_state;
uint16_t       tftp_blk_num = 0;
uint8_t        tftp_pck[TFTP_PCK_MAX_LEN];
uint16_t       tftp_pck_len = 0;
uint8_t *      tftp_pck_ptr;

bool 	       tftp_transfer_is_started = false;

ihex_str_t     ihex_str = {/* is_correct */      false,
                           /* len_is_known */    false,
                           /* len */             0, 
                           /* is_full */         true, 
                           /* is_last_in_pck */  false, 
                           /* is_eof */          false, 
                           /* addr */            0,
                           /* crc */             0};

/* Количество непроверенных байт в текущем принятом tftp пакете */
uint16_t tftp_pck_unchecked = 0;
/* Количество символов ihex строки, которые отсутсвуют в текущем принятом tftp пакете */
uint8_t  tftp_ihex_remainder = 0;
/* Резервная ihex строка для дозаполнения из текущего принятого tftp пакета, 
   если последняя ihex строка предыдущего пакета была неполной */
uint8_t  tftp_ihex_reserve_buf[IHEX_MAX_C_LEN];


void uart_init() {
    /* 115200 bps 16 MHz, см. http://wormfood.net/avrbaudcalc.php */
    uint16_t baud_rate = 0x0008;
    UBRR0H = (baud_rate & 0x0F00) >> 8;
    UBRR0L = (baud_rate  & 0x00FF);
    UCSR0C = (0 << USBS0) | (3 << UCSZ00);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}


void uart_deinit() {
    UBRR0H = 0;
    UBRR0L = 0;
    UCSR0C = 0x06;
    UCSR0B = 0x00;
}


void uart_send_byte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}


uint8_t uart_receive() {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}


bool uart_byte_is_received() {
    return (UCSR0A & (1 << RXC0));
}


uint8_t uart_get_received_byte() {
    return UDR0;
}


void boot_timeout_timer_init() {
    TCCR1B |= (1 << WGM12);	 // Clear on compare match
    OCR1A = TICKS_IN_SEC;
    TCCR1B |= (1 << CS12) | (1 << CS10); // ps = 1024 
}


bool boot_timeout_is_expired() {
    if(TIFR1 & (1 << OCF1A)) {
	TIFR1 |= (1 << OCF1A); // Clear flag
	sec_cntr++;
    }
    return (sec_cntr == BOOT_TIMEOUT_SEC);
}


void boot_timeout_timer_deinit() {
    TCCR1B = 0;
    OCR1A = 0;
    TIFR1 |= (1 << OCF1A);
}


void shift_buf::add_lsh(char val) {
    char tmp[SHIFT_BUF_LEN] = {0};
    
    for(uint8_t t = 0, b = 1; t < (SHIFT_BUF_LEN - 1); t++, b++) {
        tmp[t] = buf[b];
    }
    
    for(uint8_t i = 0; i < SHIFT_BUF_LEN; i++) {
        buf[i] = tmp[i];
    }
    
    buf[SHIFT_BUF_LEN - 1] = val;
}


bool shift_buf::is_updated() {
    if(uart_byte_is_received()) {
        add_lsh(uart_get_received_byte());
        return true;
    }
    return false;
}


bool shift_buf::contains(char *bytes, uint8_t bytes_num) {
    for(uint8_t i = (SHIFT_BUF_LEN - bytes_num + 1), j = 0; i < SHIFT_BUF_LEN; i++, j++) {
        if(buf[i] != bytes[j]) {
            return false;
        }
    }
    return true;
}


void shift_buf::clr() {
    memset(buf, 0, SHIFT_BUF_LEN);
}


void esp_send_str(const char *str) {
    while (*((uint16_t *)(str - sizeof(uint16_t))) != CR_NL) {
        uart_send_byte(*str++);
    }
}


void esp_wait_for_str(const char *str, const char *cmd = NULL) {
    if(cmd != NULL) {
        esp_send_str(cmd);
    }
    uint8_t i = 0;
    uint8_t str_len = strlen(str);
    while(i < (str_len - 1)) {
        i = (uart_receive() == str[i]) ? (i + 1) : 0;
    }
}


void esp_init() {
    /* ESP8266 Instruction Set
      https://www.espressif.com/sites/default/files/documentation/4a-esp8266_at_instruction_set_en.pdf */
    
    /* Сброс */
    esp_wait_for_str("ready\r\n", "AT+RST\r\n");
            
    /* Установить режим Station */
    esp_wait_for_str("OK\r\n", "AT+CWMODE_CUR=1\r\n");
            
    /* Отключиться от текущей точки доступа WiFi */
    esp_wait_for_str("OK\r\n", "AT+CWQAP\r\n");
            
    /* Подключиться к точке доступа WiFi */
    esp_wait_for_str("OK\r\n", wifi_connection_str);
            
    /* Получить IP для ESP8266 Station */
    esp_wait_for_str("OK\r\n", "AT+CWDHCP_DEF=1,1\r\n");
      
    /* Установить режим множественных подключений */
    esp_wait_for_str("OK\r\n", "AT+CIPMUX=1\r\n");
            
    /* Deletes/Creates TCP Server
    * AT+CIPSERVER=<mode>[,<port>]
    * <mode>:
    * 0: deletes server.
    * 1: creates server.
    * <port>: port number; 333 by default */
    esp_wait_for_str("OK\r\n", "AT+CIPSERVER=1,69\r\n");
      
    /* Установить UDP соединение с ПК */   
    esp_wait_for_str("OK\r\n", udp_pc_connection_str);
    
    /* Запросить IP для данного ESP8266  */
    esp_wait_for_str("OK\r\n", "AT+CIPSTA?\r\n");
}

bool tftp_wrreq_is_accepted() {
    return sb.contains("octet", strlen("octet"));
}

uint16_t swap_endianess(uint16_t val) {
    return (val << 8) | (val >> 8);
}


void tftp_send_ack() {
    uint16_t tftp_ack_pck[3] = {TFTP_OPCODE_ACK, swap_endianess(tftp_blk_num), CR_NL};
    esp_wait_for_str("> ", "AT+CIPSEND=1,4\r\n");
    esp_send_str((char *)tftp_ack_pck);
    tftp_blk_num++;
}


uint8_t hb(uint16_t val) {
    return (uint8_t)(val >> 8);
}


uint8_t lb(uint16_t val) {
    return (uint8_t)val;
}


bool tftp_data_pck_is_accepted() {
    uint8_t tftp_data_pck_header[5] = {(uint8_t)':', lb(TFTP_OPCODE_DATA), hb(TFTP_OPCODE_DATA), hb(tftp_blk_num), lb(tftp_blk_num)};
    return sb.contains((char *)tftp_data_pck_header, sizeof(tftp_data_pck_header) + 1);
}


bool tftp_transfer_is_completed() {
    return (ihex_str.is_eof && sb.contains("OK", strlen("OK")));      
}


uint16_t get_tftp_data_pck_len() {
    uint16_t result = 0;
    uint8_t  sb_idx = 3;
    uint16_t numbers[3] = {0};
    uint16_t mul_factors[3] = {1, 10, 100};
    uint8_t  n_idx = 0;
    
    while(sb.buf[sb_idx] != ',') {
        numbers[n_idx++] = (uint8_t)(sb.buf[sb_idx--] & 0x0F);
    }
    for(uint8_t i = 0; i < 3; i++) {
       result += numbers[i] * mul_factors[i]; 
    }
    return result - 4; 
}


void atmega_pwr_on() {
    DDRC &= ~(1 << PORTC2);         // PC2 на вход для чтения PWR_GET_DI
    if(!(PORTC & (1 << PORTC2))) {  // Если 0 на PC2
        DDRC  |= (1 << PORTC1);     // PC1 на выход для подачи на него 1
        PORTC |= (1 << PORTC1);
    }
}


/* 2 chars to hex */
uint8_t ctox(uint16_t c) {
    return ((hb(c) > 0x3F) ? (hb(c) - 0x37) : (hb(c) & 0x0F)) | (((lb(c) > 0x3F) ? (lb(c) - 0x37) : (lb(c) & 0x0F)) << 4);
}


uint8_t get_hex_byte(uint8_t *ptr) {
    return ctox(*(uint16_t *)ptr);
}


void get_tftp_pck() {
    uint16_t tftp_pck_idx = 0;
    while(tftp_pck_idx < tftp_pck_len) {
        tftp_pck[tftp_pck_idx++] = uart_receive();
    }
}


bool ihex_str_is_correct() {
    uint8_t addr_h = get_hex_byte(tftp_pck_ptr);
    tftp_pck_ptr += sizeof(uint16_t);
    uint8_t addr_l = get_hex_byte(tftp_pck_ptr);
    uint16_t addr_hl = ((uint16_t)addr_h << 8) | (uint16_t)addr_l;
    bool mb_eof = ((addr_hl == 0) && ihex_str.is_last_in_pck);
    if(!mb_eof) {
	if(addr_hl != ihex_str.addr)  {
		return false;
	}
    }
    tftp_pck_ptr += sizeof(uint16_t);
    ihex_str.addr += ihex_str.len;
    ihex_str.crc += addr_h;
    ihex_str.crc += addr_l;
    uint8_t ihex_str_record_type = get_hex_byte(tftp_pck_ptr);

    if(mb_eof && (ihex_str_record_type == IHEX_RECORD_TYPE_EOF)) {
	ihex_str.is_eof = true;
    }
    else if(ihex_str_record_type != IHEX_RECORD_TYPE_DATA) {
	return false;
    }
    tftp_pck_ptr += sizeof(uint16_t);
    ihex_str.crc += ihex_str_record_type;

    for(uint8_t i = 0; i < ihex_str.len; i++) {
	uint8_t b = get_hex_byte(tftp_pck_ptr);
	pg_str_reserve_buf[i] = b;
	ihex_str.crc += b;
	tftp_pck_ptr += sizeof(uint16_t);
    }
    ihex_str.crc += get_hex_byte(tftp_pck_ptr);
    tftp_pck_ptr += 2 * sizeof(uint16_t);
    return (ihex_str.crc == 0);
}


void get_ihex_str_status() {
    ihex_str = {/* is_correct */      false, 
                /* len_is_known */    false, 
                /* len */             0, 
                /* is_full */         false, 
                /* is_last_in_pck */  false, 
                /* is_eof */          false, 
                /* addr */            ihex_str.addr, 
                /* crc */             0};
    if(*tftp_pck_ptr++ == IHEX_START_CODE) {
        if(tftp_pck_unchecked > 2) {
            ihex_str.len = get_hex_byte(tftp_pck_ptr);
            ihex_str.len_is_known = true;
            /* Количество символов в ihex строке */
            uint8_t ihex_str_c_len = ihex_str.len*(sizeof(uint16_t)) + IHEX_SERVICE_CHAR_COUNT;
            int delta = tftp_pck_unchecked - ihex_str_c_len; 
            if(delta < 0) {
                ihex_str.is_last_in_pck = true;
            }
            else {
                ihex_str.is_full = true;
                ihex_str.crc = ihex_str.len;  
                ihex_str.is_last_in_pck = (delta == 0);   
                tftp_pck_ptr += sizeof(uint16_t);   
                ihex_str.is_correct = ihex_str_is_correct();
                if(ihex_str.is_correct) {
                    tftp_pck_unchecked -= (tftp_ihex_remainder > 0) ? tftp_ihex_remainder : ihex_str_c_len;                  
                }
            }
        }
        else {
            ihex_str.is_last_in_pck = true;  
        }
    }
}


void erase_app_space() {
    while(pg_addr != BOOTLOADER_START_ADDRESS) {
        eeprom_busy_wait();
        boot_page_erase(pg_addr);
        boot_spm_busy_wait();
        pg_addr += SPM_PAGESIZE;
    }
    pg_addr = 0;
}


void boot_program_page() {
    eeprom_busy_wait();
    boot_page_erase(pg_addr);
    boot_spm_busy_wait();      // Wait until the memory is erased.

    uint8_t *pg_ptr = &pg[0];
    for (uint8_t i = 0; i < pg_idx; i += sizeof(uint16_t)) {
      
        // Set up little-endian word.
        uint16_t w = *pg_ptr++;
        w += (*pg_ptr++) << 8;
        
        boot_page_fill(pg_addr + i, w);
    }
    boot_page_write(pg_addr);  // Store buffer in flash page.
    boot_spm_busy_wait();      // Wait until the memory is written.
    
    // Reenable RWW-section again. We need this if we want to jump back
    // to the application after bootloading.
    boot_rww_enable();
}


void setup() {
    atmega_pwr_on();
    cli();
    uart_init();
    esp_init();
    boot_timeout_timer_init();
    boot_state = BOOT_STATE_CTRL;
}


void loop() {
    while(1) {
	switch(boot_state) {
	case BOOT_STATE_CTRL: 
		if(!tftp_transfer_is_started) {
			if(boot_timeout_is_expired()) {
				boot_state = BOOT_STATE_COMPLETE;
				break;
			}
		}	
		if(sb.is_updated()) {
			if(tftp_wrreq_is_accepted()) {
				tftp_transfer_is_started = true;
				boot_timeout_timer_deinit();
				sb.clr();
				erase_app_space();
				boot_state = BOOT_STATE_ACK;
			}
			else if(tftp_data_pck_is_accepted()) {
				tftp_pck_len = get_tftp_data_pck_len();
				sb.clr();
				boot_state = BOOT_STATE_DATA;
			}
			else if(tftp_transfer_is_completed()) {
				boot_state = BOOT_STATE_COMPLETE;
			}
		}
		break;
	case BOOT_STATE_ACK:            
		tftp_send_ack();
		boot_state = BOOT_STATE_CTRL;
		break;
	case BOOT_STATE_DATA:
		get_tftp_pck();
		if(ihex_str.is_full) {
			tftp_pck_ptr = &tftp_pck[0];  
		}
		else {
			/* Дозаполнить ihex строку в резервном буфере, 
			   если в предыдущем пакете она была принята не полностью */
			uint8_t i = tftp_pck_unchecked;
			/* Количество символов в ihex строке */
			uint8_t ihex_str_c_len = 0;
			if(ihex_str.len_is_known) {
				ihex_str_c_len = ihex_str.len*(sizeof(uint16_t)) + IHEX_SERVICE_CHAR_COUNT;                    
			}
			else {
				tftp_ihex_reserve_buf[i] = tftp_pck[0];
				tftp_ihex_reserve_buf[i + 1] = tftp_pck[1];
				ihex_str_c_len = get_hex_byte(&tftp_ihex_reserve_buf[1])*(sizeof(uint16_t)) + IHEX_SERVICE_CHAR_COUNT;
			}
			for(uint8_t j = 0; i < ihex_str_c_len; i++, j++) {
				tftp_ihex_reserve_buf[i] = tftp_pck[j];           
			}
			tftp_ihex_remainder = ihex_str_c_len - tftp_pck_unchecked; 
			tftp_pck_ptr = &tftp_ihex_reserve_buf[0];
		}

		tftp_pck_unchecked = tftp_pck_len;  

		while(1) {
			get_ihex_str_status();
			/* Если ihex строка была дозаполнена в резервнм буфере, 
			   перевести указатель tftp пакета на новую ihex строку */
			if(tftp_ihex_remainder > 0) {
				tftp_pck_ptr = &tftp_pck[0] + tftp_ihex_remainder;
				tftp_ihex_remainder = 0;
			}
			if(ihex_str.is_correct) {

				/* Добавить строку к странице, если страница заполнена, приостановить запонение */
				while((pg_str_reserve_buf_idx < ihex_str.len) && (pg_idx < SPM_PAGESIZE)) {
					pg[pg_idx++] = pg_str_reserve_buf[pg_str_reserve_buf_idx++];
				}

				/* Если страница заполнена или конец файла */
				if(ihex_str.is_eof || (pg_idx == SPM_PAGESIZE)) {
					boot_program_page(); 
					pg_addr += pg_idx; 
					pg_idx = 0;
				} 

				/* Если в предыдущую страницу строка поместилась неполностью, поместить оставшуюся часть 
				   строки в новую страницу */
				while(pg_str_reserve_buf_idx < ihex_str.len) {
					pg[pg_idx++] = pg_str_reserve_buf[pg_str_reserve_buf_idx++];
				} 

				pg_str_reserve_buf_idx = 0;

				if(ihex_str.is_last_in_pck) {
					boot_state = BOOT_STATE_ACK;
					break;
				}
			}
			else if(ihex_str.is_last_in_pck && !ihex_str.is_full) {
				/* Поместить принятую часть ihex строки в резервный буфер  */
				uint8_t *ptr = tftp_pck_ptr - 1;
				for(uint8_t i = 0; i < tftp_pck_unchecked; i++) {
					tftp_ihex_reserve_buf[i] = *ptr++;
				}
				boot_state = BOOT_STATE_ACK;
				break; 
			}
			else {
				boot_state = BOOT_STATE_ERROR; 
				break;
			}
		}
		break;
	case BOOT_STATE_COMPLETE:
		uart_deinit();	
		jmp_to_app();
		break;
	case BOOT_STATE_ERROR:
		erase_app_space(); 
		break;
	default:
		break;
   	}
    }
}


void main() {
    setup();
    loop();
}

