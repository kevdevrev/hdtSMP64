#pragma once

#include <cstdint>

namespace hdt
{
	/** @brief World-space vector in Skyrim units. */
	struct WindVector
	{
		float x{}, y{}, z{};
	};

	/** @brief A movable bone; windFactor is its XML sensitivity, not shelter attenuation. */
	struct WindBody
	{
		WindVector position;
		WindVector velocity;
		float windFactor{};
		WindVector linearFactor;  // Bullet translation-axis factors, used when deriving off-center torque.
	};

	/** @brief One active physics system and its contiguous range in WindFrame::bodies. */
	struct WindSystem
	{
		std::uint64_t id{};
		WindVector position;
		float shelterFactor{};
		std::uint32_t firstBody{};
		std::uint32_t bodyCount{};
	};

	/** @brief Borrowed world-space input for one physics update; positions undo Bullet's origin shift. */
	struct WindFrame
	{
		const WindSystem* systems{};
		const WindBody* bodies{};
		std::uint32_t systemCount{};
		std::uint32_t bodyCount{};
		float timeStep{};
		float windStrength{};
		float unitsPerMeter{};
	};

	/** @brief Final central force and torque, in Bullet's mass/Skyrim-unit/second convention. */
	struct WindForce
	{
		WindVector force;
		WindVector torque;
	};

	/**
	 * @brief Optional external wind controller.
	 *
	 * FSMP copies this descriptor. Keep context and callbacks alive until unregister returns.
	 * Callbacks must not call FSMP APIs or retain borrowed arrays.
	 * isEnabled and sampleShelter run on the game thread; evaluate runs on the simulation worker.
	 * isEnabled is checked before shelter and simulation; false selects native wind for that frame.
	 * evaluate receives zeroed outputs, one per body. Apply strength, XML factors and shelter
	 * in the controller; FSMP adds the returned forces without further scaling or delta time.
	 * Returning false produces no wind for that update, not a fallback to native gusts.
	 */
	struct WindController
	{
		std::uint32_t version{ 2 };
		std::uint32_t size{ sizeof(WindController) };
		const char* name{};
		void* context{};
		bool (*sampleShelter)(void*, const WindVector& position, WindVector& direction) noexcept {};
		bool (*evaluate)(void*, const WindFrame&, WindForce* outputs) noexcept {};
		bool (*isEnabled)(void*) noexcept {};
	};
}
