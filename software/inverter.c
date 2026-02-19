/*
    12V to 230V inverter auxiliary controller software written by MINT: https://www.youtube.com/@__MINT_
    
    Used to control the main controller via LIN bus and override some features. Allows for more energy
    saving and detailed error indication. The inverter itself comes from Audi A8 D5 and requires precise
    modifications to be compatible with this software. I would be actually surprised if anyone decided
    to replicate my tedious work. Code itself runs on AT89C2051 or similar 8051 uC. Check provided
    schematic for more details regarding the hardware.
    
    And as always, remember that this software comes with absolutely no warranty!
*/

#include <8051.h>
#include <stdbool.h>

typedef unsigned char byte;
typedef unsigned int word;

#define RCV_BUFF_SIZE_EXP 3
#define TR_BUFF_SIZE_EXP 3

#define RCV_BUFF_SIZE (1 << RCV_BUFF_SIZE_EXP)
#define TR_BUFF_SIZE (1 << TR_BUFF_SIZE_EXP)
#define RCV_BUFF_MASK (RCV_BUFF_SIZE - 1)
#define TR_BUFF_MASK (TR_BUFF_SIZE - 1)

#define sei() (EA = 1)
#define cli() (EA = 0)
#define UART_INT_EN() if(1) {TI = 0; RI = 0; PS = 1; ES = 1;}
#define UART_INT_DIS() (ES = 0)
#define PLUG_INT_EN() (EX0 = 1)
#define PLUG_INT_DIS() (EX0 = 0)
#define ENTER_IDLE() (PCON |= IDL)
#define ENTER_PD() (PCON |= PD)

#define RX P3_0
#define TX P3_1
#define PLUG !(P3_2)
#define POW_5V P3_3
#define EN_OV P3_4
#define LED_OV P3_5
#define P_GOOD P3_6

// errors indicated via red LED blinking
#define WAKEUP_ERROR 1  // short-short-long
#define RESP_ERROR 2    // short-long-short
#define STARTUP_ERROR 3 // short-long-long
#define PGOOD_ERROR 4   // long-short-short or rapid blinking <-- indication from original controller
#define LOW_BATT_ERR 5  // long-short-long

void show_error(byte);

byte rcv_buff[RCV_BUFF_SIZE];  // UART receive buffer
byte tr_buff[TR_BUFF_SIZE];    // UART transmit buffer

volatile byte buffered_rcv = 0;  // number of unread received bytes
byte rcv_read_pos = 0;   // pointer to first unread byte
byte rcv_write_pos = 0;  // pointer to first free slot for reception

volatile byte buffered_tr = 0;  // number of bytes pending for transmission
volatile bool tr_armed = false; // has transmission started
byte tr_read_pos = 0;  // pointer to first pending byte
byte tr_write_pos = 0; // pointer to first free slot for transmission

byte power_on_data[] = {0x02, 0x00, 0x00};  // LIN commands; {0x02, 0x00} for inverter startup, {0x00, 0x00} for stopping 
byte resp_buff[9];  // LIN response buffer

void PLUG_ISR(void) __interrupt(IE0_VECTOR) {
    return;  // just a wakeup source
}

void UART_ISR(void) __interrupt(SI0_VECTOR) {
    if(RI) {  // receive
        RI = 0;
        if(buffered_rcv < RCV_BUFF_SIZE) {  // buffer not full
            rcv_buff[rcv_write_pos] = SBUF;  // store received byte
            buffered_rcv++;  // increment buffered bytes counter
            rcv_write_pos = (rcv_write_pos + 1) & RCV_BUFF_MASK;  // increment write pointer with overlap
        }
    }
    if(TI) {  // transmit
        TI = 0;
        if(buffered_tr > 0) {  // data not fully sent
            SBUF = tr_buff[tr_read_pos];  // send next byte
            buffered_tr--;  // decrement buffered bytes counter
            tr_read_pos = (tr_read_pos + 1) & TR_BUFF_MASK;  // increment read pointer with overlap
        }
        else tr_armed = false;
    }
}

void delay(word time_ms) {
    for(word i=0; i<time_ms; i++) {
        byte wait = 100;  // produces correct delays with 7.37 MHz clock
        while(wait--);
    }
}

void UART_send(byte data) {
    cli();
    if(buffered_tr < TR_BUFF_SIZE) {
        tr_buff[tr_write_pos] = data;
        tr_write_pos = (tr_write_pos + 1) & TR_BUFF_MASK;
        buffered_tr++;
    }
    if(!tr_armed) {  // force transmit complete interrupt to let it take care of the rest
        TI = 1;
        tr_armed = true;
    }
    else if(buffered_tr == TR_BUFF_SIZE) {  // buffer full, must wait until at least one slot is empty
        byte iter_limit = 0xFF;  // always a good practice to limit the number of iterations for while loops
        while(!TI) {
            if(--iter_limit == 0) break;
        }
    }
    sei();
    delay(1);
}

byte UART_read() {
    cli();
    byte read = 0;
    if(buffered_rcv > 0) {  // read byte from buffer if any left
        read = rcv_buff[rcv_read_pos];
        rcv_read_pos = (rcv_read_pos + 1) & RCV_BUFF_MASK;
        buffered_rcv--;
    }
    sei();
    return read;
}

void LIN_wakeup() {
    TX = 1;
    delay(10);
    TX = 0;  // wakeup pulse
    delay(1);
    TX = 1;
    delay(105);  // wait until powered devices wake up
}

byte LIN_send_request(byte ID) {
    for(byte i=0; i<100; i++) {  // wait until all bytes are sent before changing the baud rate
        if(!tr_armed) break;  // no cli() needed, byte read is an atomic operation
        delay(1);
    }
    PCON &= ~SMOD;    // reset double baud rate bit
    UART_send(0x00);  // insert break
    PCON |= SMOD;     // back to normal baud rate (19200)
    byte parity_0 = (ID & 0x01) ^ ((ID >> 1) & 0x01) ^ ((ID >> 2) & 0x01) ^ ((ID >> 4) & 0x01);  // just LIN parity stuff
    byte parity_1 = (!(((ID >> 1) & 0x01) ^ ((ID >> 3) & 0x01) ^ ((ID >> 4) & 0x01) ^ ((ID >> 5) & 0x01))) << 1;
    byte ID_word = (ID & 0x3F) | ((parity_0 | parity_1) << 6);
    UART_send(0x55);     // sync word
    UART_send(ID_word);  // frame ID
    return ID_word;      // return what was sent, needed for checksum calculation
}

void LIN_send_data(byte* data, byte len, byte ID_word) {  // send data over LIN (master frame)
    word checksum = ID_word;
    for(byte i=0; i<len; i++) {
        UART_send(data[i]);
        checksum += data[i];
    }
    checksum = ((checksum & 0xFF) + (checksum >> 8)) ^ 0xFF;  // LIN enhanced checksum
    UART_send(checksum & 0xFF);
}

byte LIN_read_response(byte* dest) {  // read LIN response (slave frame)
    for(byte i=0; i<5; i++) {
        delay(2);
        if(buffered_rcv) break;
        if(i == 4) return 0;
    }
    byte read_bytes = 0;
    while(buffered_rcv) {
        byte received = UART_read();
        if(read_bytes < 9) dest[read_bytes++] = received;
        delay(1);  // don't read faster than new bytes are coming
    }
    return read_bytes;
}

bool is_power_good() {   // check for undervoltage
    byte undervoltages = 0;
    for(byte i=0; i<10; i++) {
        undervoltages += !P_GOOD;
        if(undervoltages >= 5) return false;
        delay(10);
    }
    return true;
}

bool anything_plugged() {  // check if anything plugged
    if(!PLUG) return false;
    delay(20);
    if(!PLUG) return false;
    return true;
}

byte start_inverter() {  // enable 230V output or keep it enabled
    for(byte i=0; i<3; i++) {  // wake up LIN transceiver
        if(!POW_5V) LIN_wakeup();
        else break;
        if(i == 2) return WAKEUP_ERROR;
    }
    for(byte i=0; i<10; i++) {  // 3 attempts to get inverter started
        byte ID_word = LIN_send_request(0x3A);
        LIN_send_data(power_on_data, 2, ID_word);
        bool no_resp = true;
        bool PGOOD_fail = false;
        for(byte j=0; j<10; j++) {  // 10 attempts to get valid response (starting takes time, read responses frequently)
            delay(100);
            LIN_send_request(0x3B);
            byte read = LIN_read_response(resp_buff);
            if(read > 0) no_resp = false;
            if(read < 3) continue;
            byte status = resp_buff[1];
            if(!(status & 0x01)) continue;
            if(!(status & 0x02)) {
                PGOOD_fail = true; continue;
            }
            return 0;
        }
        if(i == 2) {
            if(no_resp) return RESP_ERROR;
            return (PGOOD_fail) ? PGOOD_ERROR : STARTUP_ERROR;
        }
        delay(250);
    }
    return STARTUP_ERROR;
}

void stop_inverter(bool cut_power) {
    if(!POW_5V) return;  // inverter controller has no power, so it is definitely stopped
    for(byte i=0; i<3; i++) {  // 3 attempts to turn inverter off
        byte ID_word = LIN_send_request(0x3A);
        LIN_send_data(power_on_data + 1, 2, ID_word);
        for(byte j=0; j<10; j++) {  // 10 attempts to get valid response (turing off might take some time)
            delay(100);
            LIN_send_request(0x3B);
            byte read = LIN_read_response(resp_buff);
            if(read < 3) continue;
            if(resp_buff[3] != 0xFF) continue;  // might be a corrupted response
            if(resp_buff[1] & 0x01) continue;   // still operating
            if(!cut_power) return;
            for(byte k=0; k<10; k++) {
                EN_OV = 1;  // force-cut power to the controller
                delay(100);
                EN_OV = 0;
                if(!POW_5V) return;
            }
            i = 3; break;
        }
        delay(250);
    }
    for(byte i=0; i<10; i++) {  // power should be cut automatically after some time, avoid force-cutting when inverter is running
        delay(1000);
        if(!POW_5V) return;
    }
}

bool enough_power_drawn() {  // check if there is any load
    byte power_sum = 0;
    for(byte i=0; i<10; i++) {
        LIN_send_request(0x3B);
        delay(20);
        byte read = LIN_read_response(resp_buff);
        if(read < 3 || !(resp_buff[1] & 0x01) || (resp_buff[3] != 0xFF)) continue;
        // resp_buff[0] stores drawn power as 5W * x. Count x'es that are not zeros.
        power_sum += (resp_buff[0] > 0);
        if(power_sum >= 5) return true;  // at least half of the responses report load greater than 0
    }
    return false;
}

void wait_if_plugged(byte millis_100) {  // delay that ends upon device unplugging
    for(byte i=0; i<millis_100; i++) {
        if(anything_plugged()) delay(80);
        else break;
    }
}

void show_error(byte err_code) {  // show error code using red LED
    if(!POW_5V) LIN_wakeup();  // enables red LED power
    for(byte i=0; i<3; i++) {
        LED_OV = 1;
        delay((err_code & 0x04) ? 500 : 250);
        err_code <<= 1;
        LED_OV = 0;
        delay(350);
    }
}

// replace power-down with long delay, remove buffered UART (to free some flash), add software power limit (shutdown countdown above 165W)

void main(void) {
    LED_OV = 0;
    EN_OV = 0;
    SCON = 0x50;  // UART mode 1
    PCON = 0x80; // double baud rate set
    TMOD = 0x20;  // Timer 1 auto-reload
    TH1 = 0xFE;   // 9600 baud rate and 19200 after doubling
    TL1 = 0xFE;
    TCON = 0x41;  // start timer 1, set INT0 as edge triggered
    delay(500);
    byte no_load_counter = 0;    // number of no load indications in a row
    bool prev_was_load = false;  // was there a load during previous check
    byte low_batt_counter = 0;   // number of low battery indications in a row 
    bool drawn_power_detect = anything_plugged();  // does inverter stop only when load unplugged (false) or also when no load detected (true)
    UART_INT_EN();
    PLUG_INT_EN();
    sei();
    for(;;) {
        if(!is_power_good()) {  // low battery
            stop_inverter(true);
            delay(250);
            show_error(LOW_BATT_ERR);
            if(++low_batt_counter >= 5) {  // battery does not recover, disable inverter permanently
                ENTER_PD();
                while(1);
            }
            wait_if_plugged(30);
            continue;
        }
        else low_batt_counter = 0;
        
        if(anything_plugged()) {  // something plugged in
            byte status = start_inverter();  // try to enable 230V output
            if(status != 0) {  // something went wrong
                stop_inverter(true);
                show_error(status);
                wait_if_plugged((status == PGOOD_ERROR) ? 150 : 15);
            }
            else if(drawn_power_detect) {
                if(!prev_was_load) delay(200);  // filter out startup inrush when measuring power right after startup
                if(!enough_power_drawn()) {  // no load detected
                    // when load unplugged, set 3s load check interval for first minute, then 6s interval for 4 minutes, and 15s interval afterwards
                    if(no_load_counter >= 60) {  // 60
                        stop_inverter(true);
                        wait_if_plugged(133);  // ~15s check interval
                    }
                    else {
                        stop_inverter(false);
                        no_load_counter++;
                        wait_if_plugged(18);  // ~3s interval
                        if(no_load_counter >= 20) {  // 20
                            LIN_wakeup();  // prevent power from getting cut by timeout
                            wait_if_plugged(30);   // 6s in total
                        }
                    }
                    prev_was_load = false;
                }
                else if(no_load_counter > 0) {
                    if(prev_was_load) no_load_counter--;  // slowly reset no_load_counter to filter out false positives
                    else prev_was_load = true;
                }
            }
        }
        else {  // go to sleep and wake up when something plugged in
            stop_inverter(true);
            UART_INT_DIS();
            ENTER_IDLE();  // will be woken up by plugging something in
            UART_INT_EN(); 
        }
    }
}

// sdcc -mmcs51 -o [output file path] [input file path]
