#ifndef __COPROCESSOR_BUILDER_HPP__
#define __COPROCESSOR_BUILDER_HPP__

#include "driftless/hal/Coprocessor.hpp"

namespace driftless {
  namespace hal {
    class CoprocessorBuilder {
      private:
        std::unique_ptr<io::ISerialDevice> m_serial_device{};

        std::unique_ptr<rtos::IClock> m_clock{};

        std::unique_ptr<rtos::IDelayer> m_delayer{};

        std::unique_ptr<rtos::IMutex> m_mutex{};

        std::unique_ptr<rtos::ITask> m_task{};
      
      public:
        /// @brief Adds a serial device to the builder
        /// @param serial_device __std::unique_ptr<io::ISerialDevice>&__ The serial
        /// device to add
        /// @return __CoprocessorBuilder*__ Pointer to the current builder
        CoprocessorBuilder* withSerialDevice(std::unique_ptr<io::ISerialDevice>& serial_device);

        CoprocessorBuilder* withClock(std::unique_ptr<rtos::IClock>& clock);

        CoprocessorBuilder* withDelayer(std::unique_ptr<rtos::IDelayer>& delayer);

        CoprocessorBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

        CoprocessorBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

        std::shared_ptr<Coprocessor> build();
    };
  }
}
#endif