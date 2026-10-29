#include "WindManager.h"
#include "WeatherManager.h"
#include "hdtSkyrimPhysicsWorld.h"
#include <cmath>

namespace hdt
{
	WindManager g_windManager;

	bool WindManager::registerController(const WindController* controller)
	{
		if (!controller || controller->version != 2 || controller->size < sizeof(WindController) ||
			!controller->name || !controller->name[0] || (controller->useNativeShelter && !controller->sampleShelter) || !controller->evaluate || !controller->isEnabled)
			return false;
		if (m_owner)
			return m_owner == controller;
		auto simulationLock = SkyrimPhysicsWorld::get()->lockSimulation();
		m_controller = *controller;
		m_name = controller->name;
		m_owner = controller;
		return true;
	}

	void WindManager::unregisterController(const WindController* controller)
	{
		if (!controller || controller != m_owner)
			return;
		// Finish the current simulation before the caller can release its callback data.
		auto simulationLock = SkyrimPhysicsWorld::get()->lockSimulation();
		m_owner = nullptr;
		m_active = false;
		m_controller = {};
		m_name.clear();
		m_refreshNative = true;
	}

	bool WindManager::isActive(const WindController* controller)
	{
		return controller && m_owner == controller && m_active;
	}

	std::string WindManager::controllerName()
	{
		return m_active ? m_name : std::string{};
	}

	RE::BSEventNotifyControl WindManager::ProcessEvent(const Events::FrameEvent*, RE::BSTEventSource<Events::FrameEvent>*)
	{
		const bool active = m_owner && m_controller.isEnabled(m_controller.context);
		if (m_active != active) {
			auto simulationLock = SkyrimPhysicsWorld::get()->lockSimulation();
			m_active = active;
			m_refreshNative = true;
		}
		return RE::BSEventNotifyControl::kContinue;
	}

	void WindManager::updateWeather(float delta)
	{
		if (m_active) {
			m_refreshNative = true;
			return;
		}
		if (m_refreshNative) {
			WeatherManager::resetCooldown();
			m_refreshNative = false;
		}
		WeatherManager::runWeatherTick(delta);
	}

	bool WindManager::shelterDirection(const RE::NiPoint3& position, RE::NiPoint3& direction)
	{
		if (m_active) {
			WindVector result{};
			if (!m_controller.sampleShelter(m_controller.context, { position.x, position.y, position.z }, result))
				return false;
			direction = { result.x, result.y, result.z };
		} else {
			direction = WeatherManager::getWindDirection();
		}
		return std::isfinite(direction.x) && std::isfinite(direction.y) && std::isfinite(direction.z) &&
		       !btFuzzyZero(direction.Length());
	}

	void WindManager::applyWind(SkinnedMeshWorld& world, float timeStep)
	{
		if (m_active)
			world.applyExternalWind(m_controller, timeStep);
		else
			world.applyWind(timeStep);
	}
}
