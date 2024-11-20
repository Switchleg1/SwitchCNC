#ifndef RMATH_H
#define RMATH_H

#undef min
#undef max

class RMath
{
public:
    static inline float min(float a, float b)
    {
        if (a < b) return a;
        return b;
    }
    static inline float max(float a, float b)
    {
        if (a < b) return b;
        return a;
    }
    static inline int32_t min(int32_t a, int32_t b)
    {
        if (a < b) return a;
        return b;
    }
    static inline uint32_t min(uint32_t a, uint32_t b)
    {
        if (a < b) return a;
        return b;
    }
    static inline int32_t min(int32_t a, int32_t b, int32_t c)
    {
        if (a < b) return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline float min(float a, float b, float c)
    {
        if (a < b) return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline int32_t max(int32_t a, int32_t b)
    {
        if (a < b) return b;
        return a;
    }
    static inline int min(int a, int b)
    {
        if (a < b) return a;
        return b;
    }
    static inline uint16_t min(uint16_t a, uint16_t b)
    {
        if (a < b) return a;
        return b;
    }
    static inline int16_t max(int16_t a, int16_t b)
    {
        if (a < b) return b;
        return a;
    }
    static inline uint16_t max(uint16_t a, uint16_t b)
    {
        if (a < b) return b;
        return a;
    }
    static inline unsigned long absLong(long a)
    {
        return a >= 0 ? a : -a;
    }
    static inline int32_t sqr(int32_t a)
    {
        return a * a;
    }
    static inline uint32_t sqr(uint32_t a)
    {
        return a * a;
    }
#ifdef SUPPORT_64_BIT_MATH
    static inline int64_t sqr(int64_t a)
    {
        return a * a;
    }
    static inline uint64_t sqr(uint64_t a)
    {
        return a * a;
    }
#endif

    static inline float sqr(float a)
    {
        return a * a;
    }
};

class RVector3
{
public:
    float x, y, z;
    RVector3(float _x = 0, float _y = 0, float _z = 0) :x(_x), y(_y), z(_z) {};
    RVector3(const RVector3& a) :x(a.x), y(a.y), z(a.z) {};


    /*    const float &operator[](std::size_t idx) const
        {
            if(idx == 0) return x;
            if(idx == 1) return y;
            return z;
        };

        float &operator[](std::size_t idx)
        {
            switch(idx) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            }
            return 0;
        };*/

    inline bool operator==(const RVector3& rhs)
    {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
    inline bool operator!=(const RVector3& rhs)
    {
        return !(*this == rhs);
    }
    inline RVector3& operator=(const RVector3& rhs)
    {
        if (this != &rhs)
        {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        return *this;
    }

    inline RVector3& operator+=(const RVector3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    inline RVector3& operator-=(const RVector3& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    inline RVector3 operator-() const
    {
        return RVector3(-x, -y, -z);
    }

    inline float length() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    inline float lengthSquared() const
    {
        return (x * x + y * y + z * z);
    }

    inline RVector3 cross(const RVector3& b) const
    {
        return RVector3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }
    inline float scalar(const RVector3& b) const
    {
        return (x * b.x + y * b.y + z * b.z);
    }
    inline RVector3 scale(float factor) const
    {
        return RVector3(x * factor, y * factor, z * factor);
    }
    inline void scaleIntern(float factor)
    {
        x *= factor;
        y *= factor;
        z *= factor;
    }
    inline void setMinimum(const RVector3& b)
    {
        x = RMath::min(x, b.x);
        y = RMath::min(y, b.y);
        z = RMath::min(z, b.z);
    }
    inline void setMaximum(const RVector3& b)
    {
        x = RMath::max(x, b.x);
        y = RMath::max(y, b.y);
        z = RMath::max(z, b.z);
    }
    inline float distance(const RVector3& b) const
    {
        float dx = b.x - x, dy = b.y - y, dz = b.z - z;
        return sqrtf(dx * dx + dy * dy + dz * dz);
    }
    inline float angle(RVector3& direction)
    {
        return static_cast<float>(acos(scalar(direction) / (length() * direction.length())));
    }

    inline RVector3 normalize() const
    {
        float len = length();
        if (len != 0) len = static_cast<float>(1.0 / len);
        return RVector3(x * len, y * len, z * len);
    }

    inline RVector3 interpolatePosition(const RVector3& b, float pos) const
    {
        float pos2 = 1.0f - pos;
        return RVector3(x * pos2 + b.x * pos, y * pos2 + b.y * pos, z * pos2 + b.z * pos);
    }

    inline RVector3 interpolateDirection(const RVector3& b, float pos) const
    {
        //float pos2 = 1.0f - pos;

        float dot = scalar(b);
        if (dot > 0.9995 || dot < -0.9995)
            return interpolatePosition(b, pos); // cases cause trouble, use linear interpolation here

        float theta = acos(dot) * pos; // interpolated position
        float st = sin(theta);
        RVector3 t(b);
        t -= scale(dot);
        float lengthSq = t.lengthSquared();
        float dl = st * ((lengthSq < 0.0001f) ? 1.0f : 1.0f / sqrtf(lengthSq));
        t.scaleIntern(dl);
        t += scale(cos(theta));
        return t.normalize();
    }
};
inline RVector3 operator+(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

inline RVector3 operator-(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    return lhs;
}

inline RVector3 operator*(const RVector3& lhs, float rhs) {
    return lhs.scale(rhs);
}

inline RVector3 operator*(float lhs, const RVector3& rhs) {
    return rhs.scale(lhs);
}

#endif