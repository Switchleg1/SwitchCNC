#include "../../../SwitchCNC.h"
#include "SdSpiCard.h"
bool SdSpiCardEX::readBlock(uint32_t block, uint8_t* dst) {
    if (m_curState != READ_STATE || block != m_curBlock) {
        if (!syncBlocks()) {
            return false;
        }
        if (!SdSpiCard::readStart(block)) {
            return false;
        }
        m_curBlock = block;
        m_curState = READ_STATE;
    }
    if (!SdSpiCard::readData(dst)) {
        return false;
    }
    m_curBlock++;
    return true;
}
//-----------------------------------------------------------------------------
bool SdSpiCardEX::readBlocks(uint32_t block, uint8_t* dst, size_t nb) {
    for (size_t i = 0; i < nb; i++) {
        if (!readBlock(block + i, dst + i * 512UL)) {
            return false;
        }
    }
    return true;
}
//-----------------------------------------------------------------------------
bool SdSpiCardEX::syncBlocks() {
    if (m_curState == READ_STATE) {
        m_curState = IDLE_STATE;
        if (!SdSpiCard::readStop()) {
            return false;
        }
    } else if (m_curState == WRITE_STATE) {
        m_curState = IDLE_STATE;
        if (!SdSpiCard::writeStop()) {
            return false;
        }
    }
    return true;
}
//-----------------------------------------------------------------------------
bool SdSpiCardEX::writeBlock(uint32_t block, const uint8_t* src) {
    if (m_curState != WRITE_STATE || m_curBlock != block) {
        if (!syncBlocks()) {
            return false;
        }
        if (!SdSpiCard::writeStart(block)) {
            return false;
        }
        m_curBlock = block;
        m_curState = WRITE_STATE;
    }
    if (!SdSpiCard::writeData(src)) {
        return false;
    }
    m_curBlock++;
    return true;
}
//-----------------------------------------------------------------------------
bool SdSpiCardEX::writeBlocks(uint32_t block,
                              const uint8_t* src, size_t nb) {
    for (size_t i = 0; i < nb; i++) {
        if (!writeBlock(block + i, src + i * 512UL)) {
            return false;
        }
    }
    return true;
}
