#include "mbed.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#define UINT14_MAX        16383
// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1
// FXOS8700CQ internal register addresses
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7


EventQueue blink_queue;
EventQueue led_blink_queue;

I2C i2c( PTD9,PTD8);
Serial pc(USBTX, USBRX);
int m_addr = FXOS8700CQ_SLAVE_ADDR1;


float t[3];

InterruptIn SW(SW3);
DigitalOut led(LED_RED);

float time_s[1000];
float x_s[1000];
float y_s[1000];
float z_s[1000];
int tilt[1000];
int counter = 0;

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);

void bank_angle();
void blink();
void accelerator();

int main()
{
    led = 0;
    Thread acc;
    acc.start(accelerator);      // let accelerator work independently
    
    SW.rise(blink_queue.event(bank_angle));
    blink_queue.dispatch();
    while(true){};
}



void accelerator() {

   pc.baud(115200);

   uint8_t who_am_i, data[2], res[6];
   int16_t acc16;
   // Enable the FXOS8700Q

   FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
   data[1] |= 0x01;
   data[0] = FXOS8700Q_CTRL_REG1;
   FXOS8700CQ_writeRegs(data, 2);

   // Get the slave address
   FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);
   while (true) {

      FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

      acc16 = (res[0] << 6) | (res[1] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      t[0] = ((float)acc16) / 4096.0f;

      acc16 = (res[2] << 6) | (res[3] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      t[1] = ((float)acc16) / 4096.0f;

      acc16 = (res[4] << 6) | (res[5] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      t[2] = ((float)acc16) / 4096.0f;

    wait(0.05);
   }
}

void blink() {led = !led;}


void bank_angle()
{
    Timer timer;
    timer.start();
    led = 1;
    Thread thread1;
    thread1.start(callback(&led_blink_queue, &EventQueue::dispatch_forever));
    int id;
    while (timer.read() <= 10) {
       time_s[counter] = timer.read();
       x_s[counter] = t[0];
       y_s[counter] = t[1];
       z_s[counter] = t[2];
/*       pc.printf("time = %f   ", timer.read());
       pc.printf("FXOS8700Q ACC: X=%1.4f Y=%1.4f Z=%1.4f\r\n", t[0], t[1], t[2]);*/
      

        if (x_s[counter] >= 0.6 || x_s[counter] <= -0.6 || y_s[counter] >= 0.6 || y_s[counter] <= -0.6)
        {
            id = led_blink_queue.call(&blink);
            tilt[counter] = 1;
        }
        else
        {
            led = 1;
            tilt[counter] = 0;
        } 
        counter++;
        wait(0.1);
    }
    timer.stop();
    timer.reset();
    led = 1;
    pc.printf("%d\r\n", counter);
    for (int i = 0; i < counter; i++)
    {
        pc.printf("%1.3f\r\n", time_s[i]);
        pc.printf("%1.3f\r\n", x_s[i]);
        pc.printf("%1.3f\r\n", y_s[i]);
        pc.printf("%1.3f\r\n", z_s[i]);
        pc.printf("%d\r\n", tilt[i]);
    }
    
}

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}

void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}