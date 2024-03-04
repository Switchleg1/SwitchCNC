#ifndef INTERRUPTPROTECTEDBLOCK_H
#define INTERRUPTPROTECTEDBLOCK_H

class InterruptProtectedBlock
{
    uint8_t sreg;
public:
    inline void protect()
    {
        cli();
    }

    inline void unprotect()
    {
        SREG = sreg;
    }

    inline InterruptProtectedBlock(bool later = false)
    {
        sreg = SREG;
        if (!later)
            cli();
    }

    inline ~InterruptProtectedBlock()
    {
        SREG = sreg;
    }
};

#endif