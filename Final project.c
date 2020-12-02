#include "IRSendRev.h"
#include <Wire.h>

#define LED 5
#define IR_OUT_PIN 2 // IR 수신기를 D2 에 연결. D3 을 제외한 핀 사용 가능
#define QSIZE 100

volatile int msec;
uint8_t duty_cycle;
volatile int adValue;
volatile int ledStat;
uint8_t ch;
char reqstr[10];

unsigned char num_arr[10][5] = {{0,255,104,151},
{0,255,48,207},
{0,255,24,231},
{0,255,122,133},
{0,255,16,239},
{0,255,56,199},
{0,255,90,165},
{0,255,66,189},
{0,255,74,181},
{0,255,82,173}};

unsigned char dta[20];
int nflag=0;

class CQueue {
  uint8_t queue[QSIZE];
  uint8_t head; // 0~99
  uint8_t tail;
  public: CQueue()
    { head = tail = 0; }
  bool enqueue(uint8_t ch)
  {
    uint8_t h;
    bool flag;
    h = (head + 1) % QSIZE;
    if (h == tail) // QUEUE_FULL
    flag = false; // fail, do nothing
    else {
      queue[head] = ch;
      head = h;
      flag = true; //success
    }
    return flag;
  }
  uint8_t dequeue(void)
  {
    uint8_t ch;
    if (head == tail) //QUEUE_EMPTY
      ch = NULL;
    else {
      ch = queue[tail];
      tail = (tail+1) % QSIZE;
    }
    return ch;
  }
};

struct CQueue TxQueue;
struct CQueue RxQueue;

ISR(USART_UDRE_vect)
{
  uint8_t ch;
  ch = TxQueue.dequeue();
  if (ch != NULL) {
    UDR0 = ch;
  }
  else{
    UCSR0B &= 0b10011000; // Tx Data Ready Interrupt Disable
  }
}

ISR(USART_RX_vect)
{ 
  RxQueue.enqueue(UDR0);
}

void putch(uint8_t ch)
{
  cli();
  TxQueue.enqueue(ch);
  if ((UCSR0B & (1<<UDRIE0)) == 0) // Tx Data Ready Int. disable
  UCSR0B |= (1<<UDRIE0); // 0b00100000; Tx Data Ready Interrupt Enable
  sei();
}
void putstr(uint8_t *str)
{
  while (*str != NULL)
  {
    putch(*str++);
  }
}

uint8_t getch(void)
{ 
  cli();
  uint8_t ch = RxQueue.dequeue(); 
  sei(); 
  return ch;
}



void setup() {
  Wire.begin();
  duty_cycle = 255;
  // LED PD.5(because OCR0B output)
  DDRD |= (1 << LED); // output
  PORTD = 0; // Clear LED
  // initialize TC0
  TCCR0A = 0x23; // OC0A not used, OC2B = non-interving mode, xx, xx, FAST PWM(CTC[1:0])
  TCCR0B = 0x0B; // xx, xx, FAST PWM(CTC[2]), 1/64
  OCR0A = 250; // Compare Match on count 250 = 1 msec
  OCR0B = duty_cycle;
  TIMSK0 = 0x02; // TC0 Output Compare A Match Interrupt Enable
  //serial initalize
  UBRR0 = 103; // 9600 bps
  UCSR0B = 0b10011000; // RXCIE enable, Data Ready Interrut Disable, Tx Enable, RX Enable
  UCSR0C = 0b00000110; // Async, No parity, 1 stop bit, 8 data bits,
  //IR routin initalize
  IR.Init(IR_OUT_PIN); // IR 루틴 초기화
  sei();
}
ISR(TIMER0_COMPA_vect)
{
  msec = (msec + 1) % 500;
  if (msec == 0) {
    ledStat ^= 1;
    
  }
}


void loop() {

  ch = getch(); 

    if(ch=='t' || ch=='T'){
    Wire.requestFrom(1,4);
      while(Wire.available()){
      char c = Wire.read();
      if(c>='0'&&c<='9'){
      sprintf(reqstr,"%c",c);
        putstr(reqstr);
      }
     }
     putstr("\n");
  }
  
  if(IR.IsDta()) // 데이터를 수신하였다면
 {
   IR.Recv(dta);

   for(int j=0;j<10;j++){
     nflag=1;
     for(int i = 0; i<dta[D_DATALEN]; i++) // payload
      {
        if(dta[D_DATA+i] != num_arr[j][i]){
          nflag=0;
        }
      } 
      if(nflag){
        duty_cycle = (j+1)*25; // min:25, max:250
      }
    }        
  }                                                                              

  if(ledStat){
     OCR0B = duty_cycle; //on LED
  }
  else{
    OCR0B = 0; //off LED
  }
}