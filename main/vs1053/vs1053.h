
#ifndef MAIN_VS1053_H_
#define MAIN_VS1053_H_

#include "driver/spi_master.h"

// SCI Register
#define VS_WRITE_COMMAND    0x02
#define VS_READ_COMMAND     0x03
#define SCI_MODE            0x00
#define SCI_STATUS          0x01
#define SCI_BASS            0x02
#define SCI_CLOCKF          0x03
#define SCI_DECODE_TIME     0x04
#define SCI_AUDATA          0x05
#define SCI_WRAM            0x06
#define SCI_WRAMADDR        0x07
#define SCI_HDAT0           0x08
#define SCI_HDAT1           0x09
#define SCI_AIADDR          0x0a
#define SCI_VOL             0x0b
#define SCI_AICTRL0         0x0c
#define SCI_AICTRL1         0x0d
#define SCI_AICTRL2         0x0e
#define SCI_AICTRL3         0x0f
#define SCI_num_registers   0x0f

// SCI_MODE bits
#define SM_SDINEW           11           // Bitnumber in SCI_MODE always on
#define SM_RESET            2            // Bitnumber in SCI_MODE soft reset
#define SM_CANCEL           3            // Bitnumber in SCI_MODE cancel song
#define SM_TESTS            5            // Bitnumber in SCI_MODE for tests
#define SM_LINE1            14           // Bitnumber in SCI_MODE for Line input

#define LOW                 0
#define HIGH                1
#define	VS1053_CHUNK_SIZE   32
#define _BV(bit) (1 << (bit)) 

typedef struct {
    int16_t cs_pin;
    int16_t dcs_pin;
    int16_t dreq_pin;
    int16_t reset_pin;
    uint8_t curvol;                         // Current volume setting 0..100%
    uint8_t endFillByte;                    // Byte to send when stopping song
    uint8_t chipVersion;                    // Version of hardware
    spi_device_handle_t SPIHandleLow;
    spi_device_handle_t SPIHandleFast;
} VS1053_t;

// Private
void delay(int ms);
void spi_master_init(VS1053_t * dev, int16_t GPIO_CS, int16_t GPIO_DCS, int16_t GPIO_DREQ, int16_t GPIO_RESET);
void await_data_request(VS1053_t * dev);
bool current_data_request(VS1053_t * dev);
void control_mode_on(VS1053_t * dev);
void control_mode_off(VS1053_t * dev);
void data_mode_on(VS1053_t * dev);
void data_mode_off(VS1053_t * dev);
uint16_t read_register(VS1053_t * dev, uint8_t _reg);
bool write_register(VS1053_t * dev, uint8_t _reg, uint16_t _value);
bool sdi_send_buffer(VS1053_t * dev, uint8_t *data, size_t len);
bool sdi_send_fillers(VS1053_t * dev, size_t length);
void wram_write(VS1053_t * dev, uint16_t address, uint16_t data);
uint16_t wram_read(VS1053_t * dev, uint16_t address);

// public
void startSong(VS1053_t * dev);                             // Prepare to start playing. Call this each
                                                            // time a new song starts.
void playChunk(VS1053_t * dev, uint8_t *data, size_t len);  // Play a chunk of data.  Copies the data to
                                                            // the chip.  Blocks until complete.
void stopSong(VS1053_t * dev);                              // Finish playing a song. Call this after
                                                            // the last playChunk call.
void setVolume(VS1053_t * dev, uint8_t vol);                // Set the player volume.Level from 0-100,
                                                            // higher is louder.
void setTone(VS1053_t * dev, uint8_t *rtone);               // Set the player baas/treble, 4 nibbles for
                                                            // treble gain/freq and bass gain/freq
uint8_t getVolume(VS1053_t * dev);                          // Get the currenet volume setting.
                                                            // higher is louder.
void printDetails(VS1053_t * dev, char *header);            // Print configuration details to serial output.
void softReset(VS1053_t * dev);                             // Do a soft reset
bool testComm(VS1053_t * dev, char *header);                // Test communication with module
void switchToMp3Mode(VS1053_t * dev);
bool isChipConnected(VS1053_t * dev);
uint16_t getDecodedTime(VS1053_t * dev);                    // Provides SCI_DECODE_TIME register value

void clearDecodedTime(VS1053_t * dev);                      // Clears SCI_DECODE_TIME register (sets 0x00)
uint8_t getHardwareVersion(VS1053_t * dev);

#endif /* MAIN_VS1053_H_ */

