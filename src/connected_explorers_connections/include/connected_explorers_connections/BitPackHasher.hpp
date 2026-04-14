
#include <array>
#include <cstdint>

struct BitpackHasher {
    std::size_t operator()(const std::array<int, 3>& arr) const {
        uint64_t x = static_cast<uint64_t>(arr[0]) & 0x1FFFFF;
        uint64_t y = static_cast<uint64_t>(arr[1]) & 0x1FFFFF;
        uint64_t z = static_cast<uint64_t>(arr[2]) & 0x1FFFFF;
        return x | (y << 21) | (z << 42);
    }
};