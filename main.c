#include "main.h"
#include "Time_Delays.h"
#include "Clk_Config.h"
#include "LCD_Display.h"
#include <stdio.h>
#include <string.h>
#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"
#include "stm32f4xx_ll_crc.h"
#define ACCELADR 0x98 // Accelerometer address
#define EEPROMADR 0xA0 // eeprom address
char data_bin[8] = {0};  //Buffer to convert decimal to binary to display it on lcd
char length[2]={0,0x36}; // length field of packet
char src[4]={0xDD,0xDD,0xDD,0xDD}; // src field of packet
char dest[4]={0xCC,0xCC,0xCC,0xCC}; //dst field of packet
char payload[54]={0}; // payload field of packet
char checksum[2]={0}; // checksum field of packet
char outputString[18];//Buffer to store text in for LCD
char packet2[66] = {0}; // packet that data is written to when read
int cs1=0; // uninverted checksum initialize
int cs2=0; // inverted checksum initialize    both will be used to calculate checksum
int combined =0;
int first = 0;
int second = 0;
int third = 0;
int fourth = 0;
int fifth = 0;
int second_third=0;
int st_fourth= 0; //THESE VARIABLES WILL BE USED TO CALCULATE CHECKSUM DESCRIPTIONS ARE BELOW
int final= 0;
int fc= 0;
int fc2= 0;
int fc3= 0;
int fc4= 0;
int checksum_1= 0;
int checksum_2= 0;
int final_cs= 0;
int chkbits1= 0;
int chkbits2= 0;
uint32_t j_c (void) {
// returns 1 if the joystick is pressed in the centre, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_5)); } //JOYSTIC PRESSING SETTINGS//
uint32_t j_u (void) {
// returns 1 if the joystick is pressed up, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_4)); }
uint32_t j_d (void) {
// returns 1 if the joystick is pressed down, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_0)); }
uint32_t j_r (void) {
// returns 1 if the joystick is pressed right, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_0)); }
uint32_t j_l (void) {
// returns 1 if the joystick is pressed left, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_1)); }

void enable_settings (void){
//Init
  SystemClock_Config();/* Configure the system clock to 84.0 MHz */
SysTick_Config_MCE2(us);
// configure the LCD
Configure_LCD_Pins ();
Configure_SPI1 ();
Activate_SPI1 ();
Clear_Screen ();
Initialise_LCD_Controller ();
set_font ((unsigned char*) Arial_12);
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
//Configure Joystick PINS
//C1 JOYSTICK LEFT
LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
//B5 JOYSTICK CENTRE
LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT);
LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
//B0 JOYSTICK DOWN //SETTINGS FOR EVERTHING INCLUDING ACCELEROMETER ACTIVE MODE//
LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
//C0 JOYSTICK RIGHT
LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
//A4 JOYSTICK UP
LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);
LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
// configure SCL as Alternate function, Open Drain, Pull Up:
LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
// configure SDA as: Alternate, Open Drain, Pull Up:
LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
}
void i2c_1_configure(void){
  //Configure SCL and SDA

  // Configure SCL as: Alternate function, High Speed, Open Drain, Pull Up
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);

  // Configure SDA as: Alternate, High Speed, Open Drain, Pull Up
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);

  //Enable I2C1 Clock
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  LL_I2C_Disable(I2C1);
  LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
  LL_I2C_ConfigSpeed(I2C1, 84000000, 100000, LL_I2C_DUTYCYCLE_2); //set speed to 100 kHz
  LL_I2C_Enable(I2C1);
}
void config_accelerometer(void){
  //Sets accelerometer to active mode
  LL_I2C_GenerateStartCondition(I2C1); //START
  while(!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, ACCELADR); //ADDRESS + WRITE
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, 0x07); //Transmit mode register address
  while(!LL_I2C_IsActiveFlag_TXE(I2C1));

  LL_I2C_TransmitData8(I2C1, 0x01); //Set device to active mode (writes a 1 to mode register bit 0)
  while(!LL_I2C_IsActiveFlag_TXE(I2C1));

  LL_I2C_GenerateStopCondition(I2C1);  //STOP
}


uint16_t read_tilt_sensor(){
//Reads the accelerometer titl sensor data
uint16_t data = 0; // to store the tilt register value
LL_I2C_GenerateStartCondition (I2C1); //START
while (!LL_I2C_IsActiveFlag_SB (I2C1));
LL_I2C_TransmitData8 (I2C1, ACCELADR); //ADDRESS + WRITE
while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
LL_I2C_ClearFlag_ADDR (I2C1);
LL_I2C_TransmitData8 (I2C1, 0x03); //Set pointer register to tilt register //FUNCTION FOR READING TILT SENSOR
while (!LL_I2C_IsActiveFlag_TXE (I2C1));
LL_I2C_GenerateStartCondition (I2C1); //RE-START
while (!LL_I2C_IsActiveFlag_SB (I2C1));
LL_I2C_TransmitData8 (I2C1, ACCELADR+1); //ADDRESS + READ
while (!LL_I2C_IsActiveFlag_ADDR (I2C1));
LL_I2C_ClearFlag_ADDR (I2C1);
LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); //ACK INCOMING DATA
while (!LL_I2C_IsActiveFlag_RXNE (I2C1));
data = LL_I2C_ReceiveData8 (I2C1); //DATA BYTE
LL_I2C_GenerateStopCondition (I2C1); //STOP
return data; // the functtion return the tilt register data
}

void eeprom_write_page_mode() {
LL_I2C_GenerateStartCondition(I2C1); //START
while(!LL_I2C_IsActiveFlag_SB(I2C1));
LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
LL_I2C_ClearFlag_ADDR(I2C1);
LL_I2C_TransmitData8(I2C1, 0x00); //POINTER HIGH
 while(!LL_I2C_IsActiveFlag_TXE(I2C1));
LL_I2C_TransmitData8(I2C1, 0x00); //POINTER LOW
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
for(int i=0; i<4; i++){ //WRITE dest field of packet
LL_I2C_TransmitData8(I2C1, dest[i]);
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
}
for(int i=0; i<4; i++){ //WRITE src field of packet
LL_I2C_TransmitData8(I2C1, src[i]);        
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
}
for(int i=0; i<2; i++){
LL_I2C_TransmitData8(I2C1, length[i]); //WRITE length field of packet
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
}
LL_I2C_TransmitData8(I2C1,payload[1]); //WRITE first bit of payload field of packet which is tilt register
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
for(int i=0; i<21; i++){
LL_I2C_TransmitData8(I2C1, payload[i+2]); // WRITE zeroes for the rest of the page since only first bit of payload is used
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
}
LL_I2C_GenerateStopCondition(I2C1); //STOP
																																																																																																																																																																																																																									            LL_mDelay(5000);
while(LL_I2C_IsActiveFlag_AF (I2C1)==1){       //ACKNOWLEDGE POLLING LOOP for transition between first page write and second page write
LL_I2C_ClearFlag_AF (I2C1);
LL_I2C_GenerateStartCondition(I2C1); //START
while(!LL_I2C_IsActiveFlag_SB(I2C1));
LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
LL_I2C_ClearFlag_ADDR(I2C1);
LL_mDelay(100);
}

LL_I2C_GenerateStartCondition(I2C1); //START
while(!LL_I2C_IsActiveFlag_SB(I2C1));
LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
LL_I2C_ClearFlag_ADDR(I2C1);
LL_I2C_TransmitData8(I2C1, 0x00); //POINTER HIGH
 while(!LL_I2C_IsActiveFlag_TXE(I2C1));
LL_I2C_TransmitData8(I2C1, 32); //POINTER LOW
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
for(int i=0; i<31; i++){               //WRITE the rest of the payload field of packet
LL_I2C_TransmitData8(I2C1, payload[24+i]);
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
}
LL_I2C_GenerateStopCondition(I2C1); //STOP
																																																																																																																																																																																																																														  LL_mDelay(5000);
LL_I2C_ClearFlag_AF (I2C1);
while(LL_I2C_IsActiveFlag_AF (I2C1)){ //ACKNOWLEDGE POLLING LOOP for transition between second page write and third page write
LL_I2C_ClearFlag_AF (I2C1);
LL_I2C_GenerateStartCondition(I2C1); //START
while(!LL_I2C_IsActiveFlag_SB(I2C1));
LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
LL_I2C_ClearFlag_ADDR(I2C1);
LL_mDelay(100);
}
   
LL_I2C_GenerateStartCondition(I2C1); //START
while(!LL_I2C_IsActiveFlag_SB(I2C1));
LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
LL_I2C_ClearFlag_ADDR(I2C1);
LL_I2C_TransmitData8(I2C1, 0x00); //POINTER HIGH
 while(!LL_I2C_IsActiveFlag_TXE(I2C1));
LL_I2C_TransmitData8(I2C1, 64); //POINTER LOW
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
for(int i=0; i<2; i++){
LL_I2C_TransmitData8(I2C1, checksum[i]); //WRITE the checksum field of packet into checksum field of packet
while(!LL_I2C_IsActiveFlag_TXE(I2C1));
}
LL_I2C_GenerateStopCondition(I2C1); //STOP
LL_mDelay(50000);
}


void read_eeporom_page(){
LL_I2C_GenerateStartCondition(I2C1); //START
while(!LL_I2C_IsActiveFlag_SB(I2C1));
LL_I2C_TransmitData8(I2C1, EEPROMADR); //ADDRESS + WRITE
while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
LL_I2C_ClearFlag_ADDR(I2C1);

LL_I2C_TransmitData8(I2C1, 0x00); //MEMORY POINTER HIGH BYTE
while(!LL_I2C_IsActiveFlag_TXE(I2C1));

LL_I2C_TransmitData8(I2C1, 0x00); //MEMORY POINTER LOW BYTE
while(!LL_I2C_IsActiveFlag_TXE(I2C1));

LL_I2C_GenerateStartCondition(I2C1); //RE-START
while(!LL_I2C_IsActiveFlag_SB(I2C1));

LL_I2C_TransmitData8(I2C1, EEPROMADR+1); //ADDRESS + READ
while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
LL_I2C_ClearFlag_ADDR(I2C1);

LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
for(int i=0; i<65; i++){
while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
packet2[i] = LL_I2C_ReceiveData8(I2C1);
}
LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
packet2[65] = LL_I2C_ReceiveData8(I2C1);//DATA BYTE
LL_I2C_GenerateStopCondition(I2C1);
}





void display_tilt(uint16_t sample){
for(int j = 0; j < 8; j++){ //DISPLAY TILT SENSOR in binary format
data_bin[j] = (sample & (0x80 >> j)) > 0;
}
put_string(0,0,"             ");
put_string(0,0,"Tilt Reg");
put_string(0,15,"             ");
sprintf(outputString,"%x%x%x%x%x%x%x%x", data_bin[0], data_bin[1], data_bin[2], data_bin[3], data_bin[4], data_bin[5], data_bin[6], data_bin[7]);
put_string(0,15,outputString);
}


int main(void){
int updown_counter=1;  //Counter that counts up and down buttons
int read_flag=0;     //read flag gets active when the read eeprom
int data_256; // data 256 is the tilt register value multipled by 256 to get 8 zeroes  on lhs to calc checksum
enable_settings();    //enable the settings for buttons
i2c_1_configure(); // configure the i2c bus
config_accelerometer(); // set up the accelerator
while (1){
if(j_c()){
updown_counter=2; // when centre is pressed updown counter is set to 2
read_flag=0;            // read flag remains zero since no reading
payload[1] =read_tilt_sensor();  //put the tilt sensor data into first byte of payload field
put_string(0,0,"             ");
put_string(0,0,"Sampled");        // report successfull read from accelometer
put_string(0,15,"             ");
LL_mDelay(500000);
display_tilt(payload[1]);      // show the value of first byte of payload which is tilt register value
}
else if(j_r()){ // activate when button is pressed right

eeprom_write_page_mode(); //eeprom write in page mode

put_string(0,0,"             "); //Report successful write
put_string(0,0,"Written");
put_string(0,15,"             ");
LL_mDelay(500000);
}
else if(j_l()){ // activate when button is pressed left
read_flag=1; // set read flag to 1 since eeprom is readed
updown_counter=2;
read_eeporom_page();
put_string(0,0,"             "); //Report successful read
put_string(0,0,"Retrieved");
put_string(0,15,"             ");
LL_mDelay(500000);
display_tilt(packet2[10]); //show the tilt register value readed from eepprom
  }
else if (j_u()||j_d()){ // activate when up or down is pressed
LL_mDelay(5000);
if(j_u()){updown_counter++;} // if up is pressed increase up down counter by 1
if(j_d()){updown_counter--;} // if down is pressed decrease up down counter by 1
if(updown_counter == 5){
put_string(0,0,"             "); //if up  down counter is 5 show dst field of packet
put_string(0,0,"dst");
put_string(0,15,"             ");
LL_mDelay(500000);
sprintf(outputString,"%x%x%x%x", dest[0],dest[1],dest[2],dest[3]);
put_string(0,15,outputString);
}
else if(updown_counter==4){put_string(0,0,"             "); //if up  down counter is 4 show src field of packet
put_string(0,0,"src");
put_string(0,15,"             ");
LL_mDelay(500000);
sprintf(outputString,"%x%x%x%x", src[0],src[1],src[2],src[3]);
put_string(0,15,outputString);}
 
else if(updown_counter== 3){put_string(0,0,"             "); //if up  down counter is 3 show length field of packet
put_string(0,0,"length");
put_string(0,15,"             ");
LL_mDelay(500000);
sprintf(outputString,"%x%x", length[0],length[1]);
put_string(0,15,outputString);
}
else if(updown_counter== 2){ //if up  down counter is 2 show first byte of payload field of packet
LL_mDelay(500000);
if(read_flag==0){
display_tilt(payload[1]);}
else if(read_flag==1){display_tilt(packet2[10]);}
put_string(0,15,outputString);
}
else if(updown_counter== 1){ //if up  down counter is 1 calculate the checksum and display it
LL_mDelay(500000);
if(read_flag==0){       //if last action is sampling (centre button) calc checksum by sampled data
data_256 =256*payload[1];}  
else if(read_flag==1){data_256 =256*packet2[10];}  //if last action is read calculate the checksum by using readed data
//Sum the packet fields
combined =((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256));
//extract each byte first second third fourth and fifth
first = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>16)&0x0000F;
second = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>12)&0x0000F;
third = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>8)&0x0000F;
fourth = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>4)&0x0000F;
fifth = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>0)&0x0000F;
//below used to apply 1s complement addition form first byte is added to last byte
second_third=second<<4 | third;
st_fourth= second_third<<4 |fourth;
final= st_fourth<<4|fifth+first;
//below substracting each value from 15(f) to find 1s complement
fc= 15-((final>>12)&0x000F);
fc2= 15-((final>>8)&0x000F);
fc3= 15-((final>>4)&0x000F);
fc4= 15-((final>>0)&0x000F);
// below is used to combine them together
checksum_1=fc<<4|fc2;
checksum_2=checksum_1<<4|fc3;
final_cs=checksum_2<<4|fc4;
//below is used to split the checksum value into 2
chkbits1=fc<<4 | fc2;
chkbits2=fc3<<4 | fc4;
cs1=final_cs;
cs2=final;
// put the values into checksum field of packet
checksum[0]= chkbits1;
checksum[1]=chkbits2;
put_string(0,0,"checksum");
put_string(0,15,"             ");
put_string(100,0,"             ");
sprintf(outputString,"%x%x", checksum[0],checksum[1]); /// display the checksum value
put_string(0,15,outputString);
}

else if(updown_counter == 0){ // check the checksum again when a further down is pressed
if(read_flag==0){
data_256 =256*payload[1];}  //if last action is sampling (centre button) calc checksum by sampled data
else if(read_flag==1){data_256 =256*packet2[10];} //if last action is read calculate the checksum by using readed data
//same calculation
combined =((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256));
first = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>16)&0x0000F;
second = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>12)&0x0000F;
third = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>8)&0x0000F;
fourth = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>4)&0x0000F;
fifth = (((dest[0]<<8 | dest[1])+(dest[2]<<8 | dest[3])+(src[0]<<8 | src[1])+(src[2]<<8 | src[3])+(length[0]<<8 | length[1])+(data_256))>>0)&0x0000F;
second_third=second<<4 | third;
st_fourth= second_third<<4 |fourth;
final= st_fourth<<4|fifth+first;
fc= 15-((final>>12)&0x000F);
fc2= 15-((final>>8)&0x000F);
fc3= 15-((final>>4)&0x000F);
fc4= 15-((final>>0)&0x000F);
checksum_1=fc<<4|fc2;
checksum_2=checksum_1<<4|fc3;
final_cs=checksum_2<<4|fc4;
//instead of putting them into checksum field of packet compare it with checksum field of packet
if(final_cs==checksum[0]<<4|checksum[1]){ // if the values are equal show OK
LL_mDelay(500000);

put_string(0,15,outputString);
put_string(100,0,"OK");
}
else { //if not display error
LL_mDelay(500000);
put_string(0,0,"cs error");
put_string(0,15,"             ");


}}
}
if(updown_counter<0){updown_counter=1;}
if(updown_counter>5){updown_counter=5;}}}

