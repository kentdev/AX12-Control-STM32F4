/*

Code to control multiple Dynamixel AX-12 servo motors over USART
on an STM32F4 chip.

Kent deVillafranca
April 2013

*/

#include "servo_control.h"


#define TEST_COMMANDS
__asm__(".global _printf_float");
__asm__(".global _scanf_float"); 



#ifndef true
#define true ((bool)1)
#endif

#ifndef false
#define false ((bool)0)
#endif

uint8_t servoErrorCode = 0;

ServoResponse response;

volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;

typedef enum ServoCommand
{
    PING = 1,
    READ = 2,
    WRITE = 3
} ServoCommand;

#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define MAX_SPEED           0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE       0x24

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params)
{
    sendServoByte (0xff);
    sendServoByte (0xff);  // command header
    
    sendServoByte (servoId);  // servo ID
    uint8_t checksum = servoId;
    
    sendServoByte (numParams + 2);  // number of following bytes
    sendServoByte ((uint8_t)commandByte);  // command
    
    checksum += numParams + 2 + commandByte;
    
    for (uint8_t i = 0; i < numParams; i++)
    {
        sendServoByte (params[i]);  // parameters
        checksum += params[i];
    }
    
    sendServoByte (~checksum);  // checksum
}

bool getServoResponse (void)
{
    uint8_t retries = 0;
    
    clearServoReceiveBuffer();
    
    while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries at start\n");
            #endif
            return false;
        }
        
        mWaitus (REC_WAIT_START_US);
    }
    retries = 0;
    
    getServoByte();  // servo header (two 0xff bytes)
    getServoByte();
    
    response.id = getServoByte();
    response.length = getServoByte();
    
    if (response.length > SERVO_MAX_PARAMS)
    {
        #ifdef SERVO_DEBUG
        printf ("Response length too big: %d\n", (int)response.length);
        #endif
        return false;
    }
    
    while (getServoBytesAvailable() < response.length)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries waiting for params, got %d of %d params\n", getServoBytesAvailable(), response.length);
            #endif
            return false;
        }
        
        mWaitus (REC_WAIT_PARAMS_US);
    }
    
    response.error = getServoByte();
    servoErrorCode = response.error;
    
    for (uint8_t i = 0; i < response.length - 2; i++)
        response.params[i] = getServoByte();
    
    
    uint8_t calcChecksum = response.id + response.length + response.error;
    for (uint8_t i = 0; i < response.length - 2; i++)
        calcChecksum += response.params[i];
    calcChecksum = ~calcChecksum;
    
    const uint8_t recChecksum = getServoByte();
    if (calcChecksum != recChecksum)
    {
        #ifdef SERVO_DEBUG
        printf ("Checksum mismatch: %x calculated, %x received\n", calcChecksum, recChecksum);
        #endif
        return false;
    }
    
    return true;
}

inline bool getAndCheckResponse (const uint8_t servoId)
{
    if (!getServoResponse())
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Servo %d did not respond correctly or at all\n", (int)servoId);
        #endif
        return false;
    }
    
    if (response.id != servoId)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response ID %d does not match command ID %d\n", (int)response.id);
        #endif
        return false;
    }
    
    if (response.error != 0)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response error code was nonzero (%d)\n", (int)response.error);
        #endif
        return false;
    }
    
    return true;
}

// ping a servo, returns true if we get back the expected values
bool pingServo (const uint8_t servoId)
{
    sendServoCommand (servoId, PING, 0, 0);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool setServoReturnDelayMicros (const uint8_t servoId,
                                const uint16_t micros)
{
    if (micros > 510)
        return false;
    
    const uint8_t params[2] = {RETURN_DELAY,
                               (uint8_t)((micros / 2) & 0xff)};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to blink its LED
bool setServoBlinkConditions (const uint8_t servoId,
                              const uint8_t flags)
{
    const uint8_t params[2] = {BLINK_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to shut off torque
bool setServoShutdownConditions (const uint8_t servoId,
                                 const uint8_t flags)
{
    const uint8_t params[2] = {SHUTDOWN_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}


// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue)
{
    const uint8_t highByte = (uint8_t)((torqueValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(torqueValue & 0xff);
    
    if (torqueValue > 1023)
        return false;
    
    const uint8_t params[3] = {TORQUE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue)
{
    const uint8_t params[2] = {TORQUE,
                               2};  // read two bytes, starting at address TORQUE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *torqueValue = response.params[1];
    *torqueValue <<= 8;
    *torqueValue |= response.params[0];
    
    return true;
}

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed (const uint8_t servoId,
                       const uint16_t speedValue)
{
    const uint8_t highByte = (uint8_t)((speedValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(speedValue & 0xff);
    
    if (speedValue > 1023)
        return false;
    
    const uint8_t params[3] = {MAX_SPEED,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoMaxSpeed (const uint8_t servoId,
                       uint16_t *speedValue)
{
    const uint8_t params[2] = {MAX_SPEED,
                               2};  // read two bytes, starting at address MAX_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *speedValue = response.params[1];
    *speedValue <<= 8;
    *speedValue |= response.params[0];
    
    return true;
}

bool getServoCurrentVelocity (const uint8_t servoId,
                              int16_t *velocityValue)
{
    const uint8_t params[2] = {CURRENT_SPEED,
                               2};  // read two bytes, starting at address CURRENT_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *velocityValue = response.params[1];
    *velocityValue <<= 8;
    *velocityValue |= response.params[0];
    
    return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle (const uint8_t servoId,
                    const float angle)
{
    if (angle < 0 || angle > 300)
        return false;
    
    // angle values go from 0 to 0x3ff (1023)
    const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));
    
    const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angleValue & 0xff);
    
    const uint8_t params[3] = {GOAL_ANGLE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoAngle (const uint8_t servoId,
                    float *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    uint16_t angleValue = response.params[1];
    angleValue <<= 8;
    angleValue |= response.params[0];
    
    *angle = (float)angleValue * 300.0 / 1023.0;
    
    return true;
}









void initServoUSART (void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStruct;
    
    clearServoReceiveBuffer();
    
    // set USART3 Tx (PB8) as alternate function open-drain
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
    
    GPIO_Init (GPIOB, &GPIO_InitStruct);
	
	// connect the output pin to the peripheral's alt function
    GPIO_PinAFConfig (GPIOB, GPIO_PinSource8, GPIO_AF_7);
    GPIO_PinAFConfig (GPIOB, GPIO_PinSource9, GPIO_AF_7);
    
    // enable the USART3 clock
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART3, ENABLE);
    
    // set up USART3
    USART_InitStructure.USART_BaudRate = 1000000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init (USART3, &USART_InitStructure);
    
    // configure the USART3 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init (&NVIC_InitStructure);
	
	// enable the USART3 receive interrupt
	USART_ITConfig (USART3, USART_IT_RXNE, ENABLE);
	
    // set USART3 to half-duplex
    USART_HalfDuplexCmd (USART3, ENABLE);
    
    // enable USART3
    USART_Cmd (USART3, ENABLE);
}

void sendServoByte (const uint8_t byte)
{
	  USART_SendData (USART3, (uint16_t)byte);
	  
	  //Loop until the end of transmission
	  while (USART_GetFlagStatus (USART3, USART_FLAG_TC) == RESET);
}

void clearServoReceiveBuffer (void)
{
    receiveBufferStart = receiveBufferEnd;
}

size_t getServoBytesAvailable (void)
{
    volatile uint8_t *start = receiveBufferStart;
    volatile uint8_t *end = receiveBufferEnd;
    
    if (end >= start)
        return (size_t)(end - start);
    else
        return (size_t)(REC_BUFFER_LEN - (start - end));
}

uint8_t getServoByte (void)
{
    receiveBufferStart++;
    if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
        receiveBufferStart = receiveBuffer;
    
    return *receiveBufferStart;
}

void USART3_IRQHandler (void)
{
	// check if the USART3 receive interrupt flag was set
	if (USART_GetITStatus (USART3, USART_IT_RXNE))
	{
		const uint8_t byte = (uint8_t)USART_ReceiveData (USART3); // grab the byte from the data register
        
        receiveBufferEnd++;
        if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
            receiveBufferEnd = receiveBuffer;
        
        *receiveBufferEnd = byte;
	}
}



#ifdef TEST_COMMANDS

void error (void)
{
    fflush (stdout);
    
    for (;;)
    {
        mRedTOGGLE;
        mWaitms (100);
    }
}

int main (void)
{
    mInit();
    mUSBInit();
    initServoUSART();
    
    mRedON;
    mWaitms (1500);
    mRedOFF;
    
    printf ("Init complete\n");
    
    const uint8_t id = 4;
    
    if (!pingServo (id))
    {
        printf ("Ping failed\n");
        error();
    }
    
    printf ("Ping OK\n");
    
    if (!setServoReturnDelayMicros (id, 0))
    {
        printf ("Set return delay failed\n");
        error();
    }
    
    printf ("Set return delay OK\n");
    
    if (!setServoBlinkConditions (id, SERVO_RANGE_ERROR | SERVO_ANGLE_LIMIT_ERROR))
    {
        fflush (stdout);
        printf ("Set blink conditions failed\n");
        error();
    }
    
    printf ("Set blink conditions OK\n");
    
    if (!setServoShutdownConditions (id, SERVO_OVERLOAD_ERROR | SERVO_OVERHEAT_ERROR))
    {
        fflush (stdout);
        printf ("Set shutdown conditions failed\n");
        error();
    }
    
    printf ("Set shutdown conditions OK\n");
    
    uint16_t torque = 0;
    if (!getServoTorque (id, &torque))
    {
        fflush (stdout);
        printf ("Get servo torque failed\n");
        error();
    }
    
    printf ("Get torque OK: servo torque = %u\n", torque);
    
    torque = 512;
    if (!setServoTorque (id, torque))
    {
        fflush (stdout);
        printf ("Set servo torque failed\n");
        error();
    }
    
    printf ("Set torque OK\n");
    
    torque = 0;
    if (!getServoTorque (id, &torque))
    {
        printf ("Get servo torque failed\n");
        error();
    }
    
    printf ("Get torque OK: servo torque = %u\n", torque);
    
    uint16_t speed = 0;
    if (!getServoMaxSpeed (id, &speed))
    {
        printf ("Get servo max speed failed\n");
        error();
    }
    
    printf ("Get max speed OK: max speed = %u\n", speed);
    
    speed = 1023;
    if (!setServoMaxSpeed (id, speed))
    {
        fflush (stdout);
        printf ("Set servo max speed failed\n");
        error();
    }
    
    printf ("Set max speed OK\n");
    
    speed = 0;
    if (!getServoMaxSpeed (id, &speed))
    {
        printf ("Get servo max speed failed\n");
        error();
    }
    
    printf ("Get max speed OK: max speed = %u\n", speed);
    
    int16_t currentSpeed = 0;
    if (!getServoMaxSpeed (id, &currentSpeed))
    {
        printf ("Get servo current speed failed\n");
        error();
    }
    
    printf ("Get current speed OK: current speed = %d\n", currentSpeed);
    
    
    for (int8_t i = 0; i < 5; i++)
    {
        if (!setServoMaxSpeed (id, 128 * (i + 1)))
        {
            fflush (stdout);
            printf ("Set servo max speed failed\n");
            error();
        }
        
        float angle = 0;
        if (!getServoAngle (id, &angle))
        {
            printf ("Get servo angle failed\n");
            error();
        }
        
        printf ("Angle = %f\n", angle);
        
        angle = 300.0 - angle;
        if (!setServoAngle (id, angle))
        {
            printf ("Set servo angle failed\n");
            error();
        }
        
        printf ("Set angle to %f\n", angle);
        
        for (uint8_t i = 0; i < 1 + (4 - i); i++)
            mWaitms (500);
    }
    
    printf ("Done\n");
    
    for (;;)
    {
        mGreenTOGGLE;
        mRedTOGGLE;
        
        mWaitms (250);
    }
    
    return 0;
}

#endif

