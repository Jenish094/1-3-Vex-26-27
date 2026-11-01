#pragma once
#include "robot_hardware.hpp"

class Macro
{
public:
        virtual ~Macro() = default;
        virtual void Run(int timeout_millis) = 0;
};

class FlipToggleHeight final : public Macro
{
public:
        void Run(int timeout_millis) override;
};
