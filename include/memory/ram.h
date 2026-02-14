//
// Created by Tony on 2/13/2026.
//

#ifndef GB_EMULATOR_RAM_H
#define GB_EMULATOR_RAM_H
#include <cstdint>
#include <string>
#include <vector>


class RAM {
public:
  virtual ~RAM() = default;

  const uint16_t START;
  const uint16_t END;
  const uint16_t SIZE;

  RAM(const uint16_t start, const uint16_t end, const uint16_t size) : START(start), END(end), SIZE(size), ram_(SIZE) {}

  void Write(const uint8_t &value, const uint16_t &address);
  [[nodiscard]] uint8_t Read(const uint16_t &address) const;

  [[nodiscard]] const std::vector<uint8_t>& GetRAM() const { return ram_; }
protected:
  [[nodiscard]] virtual std::string get_class_name() const = 0;
  std::vector<uint8_t> ram_;
};


#endif //GB_EMULATOR_RAM_H