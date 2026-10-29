#pragma once

#include "Events.h"
#include "WindAPI.h"
#include <string>

namespace hdt
{
	class SkinnedMeshWorld;

	class WindManager : public RE::BSTEventSink<Events::FrameEvent>
	{
	public:
		bool registerController(const WindController* controller);
		void unregisterController(const WindController* controller);
		bool isActive(const WindController* controller);
		/** @brief Whether actor updates should run FSMP's shelter checks. */
		bool usesNativeShelter() const { return !m_active || m_controller.useNativeShelter; }
		std::string controllerName();
		bool shelterDirection(const RE::NiPoint3& position, RE::NiPoint3& direction);
		void updateWeather(float delta);
		void applyWind(SkinnedMeshWorld& world, float timeStep);
		RE::BSEventNotifyControl ProcessEvent(const Events::FrameEvent*, RE::BSTEventSource<Events::FrameEvent>*) override;

	private:
		const WindController* m_owner{};
		WindController m_controller;
		std::string m_name;
		bool m_active{};
		bool m_refreshNative{};
	};

	extern WindManager g_windManager;
}
