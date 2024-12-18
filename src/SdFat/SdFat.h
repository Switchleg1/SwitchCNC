/**
 * Copyright (c) 20011-2017 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#ifndef SdFat_h
#define SdFat_h

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
/**
 * \file
 * \brief SdFat class
 */
#include "SysCall.h"
#include "BlockDriver.h"
#include "FatLib/FatLib.h"
#include "SdCard/SdioCard.h"
#if INCLUDE_SDIOS
#include "sdios.h"
#endif // INCLUDE_SDIOS
//------------------------------------------------------------------------------
/** SdFat version */
#define SD_FAT_VERSION "1.0.7"
//==============================================================================
/**
 * \class SdBaseFile
 * \brief Class for backward compatibility.
 */
class SdBaseFile : public FatFile {
public:
    SdBaseFile() {}
    /**  Create a file object and open it in the current working directory.
   *
   * \param[in] path A path for a file to be opened.
   *
   * \param[in] oflag Values for \a oflag are constructed by a
   * bitwise-inclusive OR of open flags. see
   * FatFile::open(FatFile*, const char*, uint8_t).
   */
    SdBaseFile(const char* path, uint8_t oflag)
        : FatFile(path, oflag) {}
};
//-----------------------------------------------------------------------------
#if ENABLE_ARDUINO_FEATURES
/**
 * \class SdFile
 * \brief Class for backward compatibility.
 */
class SdFile : public PrintFile {
public:
    SdFile() {}
    /**  Create a file object and open it in the current working directory.
   *
   * \param[in] path A path for a file to be opened.
   *
   * \param[in] oflag Values for \a oflag are constructed by a
   * bitwise-inclusive OR of open flags. see
   * FatFile::open(FatFile*, const char*, uint8_t).
   */
    SdFile(const char* path, uint8_t oflag)
        : PrintFile(path, oflag) {}
};
#endif // #if ENABLE_ARDUINO_FEATURES
//-----------------------------------------------------------------------------
/**
 * \class SdFileSystem
 * \brief Virtual base class for %SdFat library.
 */
template <class SdDriverClass>
class SdFileSystem : public FatFileSystem {
public:
    /** Initialize file system.
   * \return true for success else false.
   */
    bool begin() {
        return FatFileSystem::begin(&m_card);
    }
    /** \return Pointer to SD card object */
    SdDriverClass* card() {
        m_card.syncBlocks();
        return &m_card;
    }
    /** %Print any SD error code to Serial and halt. */
    void errorHalt() {
        errorPrint();
        SysCall::halt();
    }
    /** %Print msg, any SD error code and halt.
   *
   * \param[in] msg Message to print.
   */
    void errorHalt(char const* msg) {
        errorHalt(msg);
    }
    /** %Print any SD error code to Serial */
    void errorPrint() {
        if (!cardErrorCode()) {
            return;
        }
        Com::printF(PSTR("SD errorCode: 0X"));
        Com::print((int32_t)cardErrorCode());
        Com::printF(PSTR(",0X"));
        Com::print((int32_t)cardErrorData());
        Com::println();
    }
    /** %Print msg, any SD error code.
   *
   * \param[in] msg Message to print.
   */
    void errorPrint(const char* msg) {
        Com::printF(PSTR("error: "));
        Com::printFLN(msg);
        errorPrint();
    }
    /** %Print any SD error code and halt. */
    void initErrorHalt() {
        initErrorPrint();
        SysCall::halt();
    }
    /**Print message, error details, and halt after begin() fails.
   *
   * \param[in] msg Message to print.
   */
    void initErrorHalt(char const* msg) {
        initErrorHalt(msg);
    }

    /** Print error details after begin() fails. */
    void initErrorPrint() {
        if (cardErrorCode()) {
            Com::printFLN(PSTR("Can't access SD card. Do not reformat."));
            if (cardErrorCode() == SD_CARD_ERROR_CMD0) {
                Com::printFLN(PSTR("No card, wrong chip select pin, or SPI problem?"));
            }
            errorPrint();
        } else if (vol()->fatType() == 0) {
            Com::printFLN(PSTR("Invalid format, reformat SD."));
        } else if (!vwd()->isOpen()) {
            Com::printFLN(PSTR("Can't open root directory."));
        } else {
            Com::printFLN(PSTR("No error found."));
        }
    }
    /**Print message and error details and halt after begin() fails.
   *
   * \param[in] msg Message to print.
   */
    void initErrorPrint(char const* msg) {
        Com::printFLN(msg);
        initErrorPrint();
    }
#if defined(ARDUINO)
    /** %Print msg, any SD error code, and halt.
   *
   * \param[in] msg Message to print.
   */
    void errorHalt(const __FlashStringHelper* msg) {
        errorPrint(reinterpret_cast<const char*>(msg));
        SysCall::halt();
    }

    /** %Print msg, any SD error code.
   *
   * \param[in] msg Message to print.
   */
    void errorPrint(const __FlashStringHelper* msg) {
        Com::printF(PSTR("error: "));
        Com::printFLN(reinterpret_cast<const char*>(msg));
        errorPrint();
    }
    /**Print message, error details, and halt after begin() fails.
    *
    * \param[in] msg Message to print.
    */
    void initErrorHalt(const __FlashStringHelper* msg) {
        Com::printFLN(reinterpret_cast<const char*>(msg));
        initErrorHalt();
    }
    /**Print message and error details and halt after begin() fails.
   *
   * \param[in] msg Message to print.
   */
    void initErrorPrint(const __FlashStringHelper* msg) {
        Com::printFLN(reinterpret_cast<const char*>(msg));
        initErrorPrint();
    }
#endif // defined(ARDUINO)
    /** \return The card error code */
    uint8_t cardErrorCode() {
        return m_card.errorCode();
    }
    /** \return the card error data */
    uint32_t cardErrorData() {
        return m_card.errorData();
    }

protected:
    SdDriverClass m_card;
};
//==============================================================================
/**
 * \class SdFat
 * \brief Main file system class for %SdFat library.
 */
class SdFat : public SdFileSystem<SdSpiCard> {
public:
#if IMPLEMENT_SPI_PORT_SELECTION
    SdFat() {
        m_spi.setPort(0);
    }
    /** Constructor with SPI port selection.
   * \param[in] spiPort SPI port number.
   */
    explicit SdFat(uint8_t spiPort) {
        m_spi.setPort(spiPort);
    }
#endif // IMPLEMENT_SPI_PORT_SELECTION
    /** Initialize SD card and file system.
   *
   * \param[in] csPin SD card chip select pin.
   * \param[in] spiSettings SPI speed, mode, and bit order.
   * \return true for success else false.
   */
    bool begin(uint8_t csPin = SS, SPISettings spiSettings = SPI_FULL_SPEED) {
        return m_card.begin(&m_spi, csPin, spiSettings) && SdFileSystem::begin();
    }
    /** Initialize SD card for diagnostic use only.
   *
   * \param[in] csPin SD card chip select pin.
   * \param[in] settings SPI speed, mode, and bit order.
   * \return true for success else false.
   */
    bool cardBegin(uint8_t csPin = SS, SPISettings settings = SPI_FULL_SPEED) {
        return m_card.begin(&m_spi, csPin, settings);
    }
    /** Initialize file system for diagnostic use only.
   * \return true for success else false.
   */
    bool fsBegin() {
        return FatFileSystem::begin(card());
    }

private:
    SdFatSpiDriver m_spi;
};
//==============================================================================
#if ENABLE_SDIO_CLASS
/**
 * \class SdFatSdio
 * \brief SdFat class using SDIO.
 */
class SdFatSdio : public SdFileSystem<SdioCard> {
public:
    /** Initialize SD card and file system.
   * \return true for success else false.
   */
    bool begin() {
        return m_card.begin() && SdFileSystem::begin();
    }
    /** Initialize SD card for diagnostic use only.
   *
   * \return true for success else false.
   */
    bool cardBegin() {
        return m_card.begin();
    }
    /** Initialize file system for diagnostic use only.
   * \return true for success else false.
   */
    bool fsBegin() {
        return SdFileSystem::begin();
    }
};
#if ENABLE_SDIOEX_CLASS
//-----------------------------------------------------------------------------
/**
 * \class SdFatSdioEX
 * \brief SdFat class using SDIO.
 */
class SdFatSdioEX : public SdFileSystem<SdioCardEX> {
public:
    /** Initialize SD card and file system.
   * \return true for success else false.
   */
    bool begin() {
        return m_card.begin() && SdFileSystem::begin();
    }
    /** \return Pointer to SD card object */
    SdioCardEX* card() {
        return &m_card;
    }
    /** Initialize SD card for diagnostic use only.
   *
   * \return true for success else false.
   */
    bool cardBegin() {
        return m_card.begin();
    }
    /** Initialize file system for diagnostic use only.
   * \return true for success else false.
   */
    bool fsBegin() {
        return SdFileSystem::begin();
    }
};
#endif // ENABLE_SDIOEX_CLASS
#endif // ENABLE_SDIO_CLASS
//=============================================================================
#if ENABLE_SOFTWARE_SPI_CLASS
/**
 * \class SdFatSoftSpi
 * \brief SdFat class using software SPI.
 */
template <uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin>
class SdFatSoftSpi : public SdFileSystem<SdSpiCard> {
public:
    /** Initialize SD card and file system.
   *
   * \param[in] csPin SD card chip select pin.
   * \param[in] spiSettings ignored for software SPI..
   * \return true for success else false.
   */
    bool begin(uint8_t csPin = SS, SPISettings spiSettings = SPI_FULL_SPEED) {
        return m_card.begin(&m_spi, csPin, spiSettings) && SdFileSystem::begin();
    }

private:
    SdSpiSoftDriver<MisoPin, MosiPin, SckPin> m_spi;
};
#endif // #if ENABLE_SOFTWARE_SPI_CLASS
//==============================================================================
#if ENABLE_EXTENDED_TRANSFER_CLASS
/**
 * \class SdFatEX
 * \brief SdFat class with extended SD I/O.
 */
class SdFatEX : public SdFileSystem<SdSpiCardEX> {
public:
#if IMPLEMENT_SPI_PORT_SELECTION
    SdFatEX() {
        m_spi.setPort(0);
    }
    /** Constructor with SPI port selection.
   * \param[in] spiPort SPI port number.
   */
    explicit SdFatEX(uint8_t spiPort) {
        m_spi.setPort(spiPort);
    }
#endif // IMPLEMENT_SPI_PORT_SELECTION
    /** Initialize SD card and file system.
  *
  * \param[in] csPin SD card chip select pin.
  * \param[in] spiSettings SPI speed, mode, and bit order.
  * \return true for success else false.
  */
    bool begin(uint8_t csPin = SS, SPISettings spiSettings = SPI_FULL_SPEED) {
        return m_card.begin(&m_spi, csPin, spiSettings) && SdFileSystem::begin();
    }

private:
    SdFatSpiDriver m_spi;
};
//==============================================================================
#if ENABLE_SOFTWARE_SPI_CLASS
/**
 * \class SdFatSoftSpiEX
 * \brief SdFat class using software SPI and extended SD I/O.
 */
template <uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin>
class SdFatSoftSpiEX : public SdFileSystem<SdSpiCardEX> {
public:
    /** Initialize SD card and file system.
   *
   * \param[in] csPin SD card chip select pin.
   * \param[in] spiSettings ignored for software SPI.
   * \return true for success else false.
   */
    bool begin(uint8_t csPin = SS, SPISettings spiSettings = SPI_FULL_SPEED) {
        return m_card.begin(&m_spi, csPin, spiSettings) && SdFileSystem::begin();
    }

private:
    SdSpiSoftDriver<MisoPin, MosiPin, SckPin> m_spi;
};
#endif // #if ENABLE_SOFTWARE_SPI_CLASS
#endif // ENABLE_EXTENDED_TRANSFER_CLASS
//=============================================================================
/**
 * \class Sd2Card
 * \brief Raw access to SD and SDHC card using default SPI library.
 */
class Sd2Card : public SdSpiCard {
public:
    /** Initialize the SD card.
   * \param[in] csPin SD chip select pin.
   * \param[in] settings SPI speed, mode, and bit order.
   * \return true for success else false.
   */
    bool begin(uint8_t csPin = SS, SPISettings settings = SD_SCK_MHZ(50)) {
        return SdSpiCard::begin(&m_spi, csPin, settings);
    }

private:
    SdFatSpiDriver m_spi;
};
#pragma GCC diagnostic pop
#endif // SdFat_h
