#include "amt203.hpp"
#include "utils.h"

SPI_HandleTypeDef spiSettings_ = {
    .Instance = SPI3,
    .Init = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_8BIT,
        .CLKPolarity = SPI_POLARITY_LOW,
        .CLKPhase = SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10}};

// See http://www.cui.com/product/resource/AMT203-v.pdf for protocol information
AMT203::AMT203(SPI_HandleTypeDef* spiHandle, GPIO_TypeDef* chipSelectHandle, uint16_t chipSelectPin)
    : spiHandle(spiHandle),
      chipSelectHandle(chipSelectHandle),
      chipSelectPin(chipSelectPin),
      position(0) {
    spiSettings_.Instance = spiHandle->Instance;
}

bool AMT203::init() {
    // Initialize the chip select pin so it behaves how the AMT203 expects
    HAL_GPIO_DeInit(chipSelectHandle, chipSelectPin);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = chipSelectPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(chipSelectHandle, &GPIO_InitStruct);

    // Set chipSelect high
    HAL_GPIO_WritePin(chipSelectHandle, chipSelectPin, GPIO_PIN_SET);

    // Reinitialize the SPI bus to the required settings,
    // but use the same Instance that was passed to the object
    HAL_SPI_DeInit(spiHandle);
    *spiHandle = spiSettings_;
    HAL_SPI_Init(spiHandle);

    //! WARNING! Potential for thread-lockup!
    while (sendNop() != AMT203_NOP_A5) {
        // Wait for the encoder to finish its boot sequence
        // Also, flushes the output buffer on the encoder
    }
    return true;
}
// Read the current position from the encoder.  Blocks until complete.
// TODO: Make non-blocking
uint32_t AMT203::readPosition() {
    // Send the rd_pos command, (0x10)
    uint8_t rxVal = sendCommand(AMT203_READ_POS_CMD);

    // Wait until the encoder echoes the command, which indicates it's ready
    //! WARNING! Potential for thread-lockup!
    do {
        rxVal = sendNop();
    } while (rxVal == AMT203_NOP_A5);

    // once the AMT echoes the read command,
    // The next two received bytes are the MSB and the LSB of the position, respectively
    // If the encoder sends anything but A5's, we've screwed something up so return a negative position
    if (rxVal == AMT203_READ_POS_CMD)
        this->position = (sendCommand(AMT203_NOP_CMD) << 8) | sendCommand(AMT203_NOP_CMD);
    else
        this->position = -1;
    return this->position;
}

// This no operation command is ignored by the encoder and simply causes the next byte of data to be read.
// The encoder will respond with 0xA5 if there is no remaining data to be sent.
// TODO: Make non-blocking
uint8_t AMT203::sendNop() {
    return sendCommand(AMT203_NOP_CMD);
}

/** This command sets the current position to zero and saves this setting in the AMT203x EEPROM. 
 *! The encoder must be power cycled after this is set.   If the encoder is not power cycled, 
 *!  the position values will not be calculated off the latest zero position. */
bool AMT203::setZeroPoint() {
    uint8_t response = sendCommand(AMT203_SET_ZERO_POINT_CMD);

    //! WARNING! Potential for thread-lockup!
    do {
        response = sendNop();
    } while (response == AMT203_NOP_A5);

    if (response == AMT203_SET_ZERO_POINT_SUCCESS)
        return true;
    else
        return false;
}

// Sends an 8-bit command to the AMT203, and returns the response.  Per the datasheet,
// Each byte transmitted must be followed by a release of the Chip Select line (CSB).
// TODO:  Make non-blocking
uint8_t AMT203::sendCommand(AMT203_Message_t command) {
    uint8_t rxData[8];
    HAL_GPIO_WritePin(this->chipSelectHandle, this->chipSelectPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(this->spiHandle, (uint8_t*)&command, rxData, 1, 1);
    HAL_GPIO_WritePin(this->chipSelectHandle, this->chipSelectPin, GPIO_PIN_SET);
    delay_us(20);

    return rxData[0];
}