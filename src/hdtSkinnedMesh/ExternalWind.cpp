#include "hdtSkinnedMeshWorld.h"
#include "hdtSkyrimPhysicsWorld.h"
#include <cmath>

namespace hdt
{
	void SkinnedMeshWorld::applyExternalWind(const WindController& controller, btScalar timeStep)
	{
		BT_PROFILE("HDTSMP_externalWind");
		m_windSystems.clear();
		m_windBodies.clear();
		m_windRigidBodies.clear();
		for (const auto& system : m_systems) {
			const auto first = static_cast<std::uint32_t>(m_windBodies.size());
			btVector3 center(0.f, 0.f, 0.f);
			for (const auto& bone : system->m_bones) {
				auto& body = bone->m_rig;
				if (body.isStaticOrKinematicObject() || btFuzzyZero(bone->m_windFactor))
					continue;
				// The callback sees Skyrim coordinates even though Bullet has recentered the bodies.
				const auto position = body.getWorldTransform().getOrigin() + m_windOriginOffset;
				const auto velocity = body.getLinearVelocity();
				const auto linearFactor = body.getLinearFactor();
				center += position;
				m_windBodies.push_back({ { position.x(), position.y(), position.z() },
					{ velocity.x(), velocity.y(), velocity.z() }, bone->m_windFactor,
					{ linearFactor.x(), linearFactor.y(), linearFactor.z() } });
				m_windRigidBodies.push_back(&body);
			}
			const auto count = static_cast<std::uint32_t>(m_windBodies.size()) - first;
			if (!count)
				continue;
			center /= static_cast<btScalar>(count);
			m_windSystems.push_back({ m_windSystemIds.at(system.get()), { center.x(), center.y(), center.z() },
				controller.useNativeShelter ? static_cast<SkyrimSystem*>(system.get())->m_windFactor : 1.f, first, count });
		}

		// Include an empty frame so the controller can release state for removed systems.
		m_windForces.assign(m_windBodies.size(), WindForce{});
		const WindFrame frame{ m_windSystems.data(), m_windBodies.data(),
			static_cast<std::uint32_t>(m_windSystems.size()), static_cast<std::uint32_t>(m_windBodies.size()),
			timeStep, SkyrimPhysicsWorld::get()->m_windStrength, scaleSkyrim };
		if (!controller.evaluate(controller.context, frame, m_windForces.data()))
			return;
		for (std::size_t i = 0; i < m_windForces.size(); ++i) {
			const auto& result = m_windForces[i];
			if (!std::isfinite(result.force.x) || !std::isfinite(result.force.y) || !std::isfinite(result.force.z) ||
				!std::isfinite(result.torque.x) || !std::isfinite(result.torque.y) || !std::isfinite(result.torque.z))
				continue;
			// The controller has already applied its response model and XML sensitivity.
			m_windRigidBodies[i]->applyCentralForce({ result.force.x, result.force.y, result.force.z });
			m_windRigidBodies[i]->applyTorque({ result.torque.x, result.torque.y, result.torque.z });
		}
	}
}
