#pragma once
#include "hardware.hpp"

class Macro {
public:
        virtual ~Macro() = default;
        virtual void Run(int timeout_millis) = 0;
};

class FlipToggleHeight : public Macro {
public:
        void Run(int timeout_millis) override;
};