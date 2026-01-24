//
// Created by fish on 2025/1/3.
//

#pragma once

#include "controller_base.h"

namespace Controller {
	class ForwardFeed : public Base {
	public:
        ForwardFeed(float output_min, float output_max) : _output_min(output_min), _output_max(output_max) {}
		~ForwardFeed() override = default;

		void clear() override { };
		float update(float current, float target) override;
		float update(const MotorController *motor, float target) override { return 0; }
    private:
        float _output_min, _output_max;
	};
}
