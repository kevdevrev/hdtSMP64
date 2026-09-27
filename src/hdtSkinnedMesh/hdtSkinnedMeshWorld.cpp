#include "hdtSkinnedMeshWorld.h"
#include "hdtBoneScaleConstraint.h"
#include "hdtDispatcher.h"
#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtSkyrimSystem.h"
#include <algorithm>
#include <cmath>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace hdt
{
	namespace
	{
		int initBulletTbbAndGetThreadCount()
		{
			auto* scheduler = btGetTBBTaskScheduler();
			btSetTaskScheduler(scheduler);

			int concurrency = std::max(1, scheduler->getMaxNumThreads());

			logger::info("Physics simulation is using {} threads", concurrency);

			return concurrency;
		}
	}

	SkinnedMeshWorld::SkinnedMeshWorld() :
		btDiscreteDynamicsWorldMt(
			nullptr,
			nullptr,
			// Pool of regular sequential solvers one per hardware thread.
			// Each island gets dispatched to a free solver on any thread.
			new btConstraintSolverPoolMt(
				initBulletTbbAndGetThreadCount()),
			nullptr,  // no Mt solver, avoids btBatchedConstraints entirely (we are not designed for that yet)
			nullptr)
	{
		m_windSpeed = _mm_setzero_ps();

		auto collisionConfiguration = new btDefaultCollisionConfiguration;
		auto collisionDispatcher = new CollisionDispatcher(collisionConfiguration);

		m_dispatcher1 = collisionDispatcher;
		m_broadphasePairCache = new btDbvtBroadphase();
	}

	SkinnedMeshWorld::~SkinnedMeshWorld()
	{
		for (auto system : m_systems) {
			for (int i = 0; i < system->m_meshes.size(); ++i)
				removeCollisionObject(system->m_meshes[i].get());

			for (int i = 0; i < system->m_constraints.size(); ++i)
				if (system->m_constraints[i]->m_constraint)
					removeConstraint(system->m_constraints[i]->m_constraint);

			for (int i = 0; i < system->m_bones.size(); ++i)
				removeRigidBody(&system->m_bones[i]->m_rig);

			for (auto i : system->m_constraintGroups)
				for (auto j : i->m_constraints)
					if (j->m_constraint)
						removeConstraint(j->m_constraint);
		}

		m_systems.clear();

		auto solver = m_constraintSolver;
		m_constraintSolver = nullptr;
		delete solver;
	}

	void SkinnedMeshWorld::addSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		if (std::find(m_systems.begin(), m_systems.end(), system) != m_systems.end()) {
			return;
		}

		m_systems.push_back(hdt::make_smart(system));
		for (int i = 0; i < system->m_meshes.size(); ++i) {
			addCollisionObject(system->m_meshes[i].get(), 1, 1);
		}

		for (int i = 0; i < system->m_bones.size(); ++i) {
			system->m_bones[i]->m_rig.setActivationState(DISABLE_DEACTIVATION);
			// 0,0 mask disables the collision of this object on Bullet.
			addRigidBody(&system->m_bones[i]->m_rig, 0, 0);
		}

		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				addConstraint(j->m_constraint, true);

		for (int i = 0; i < system->m_constraints.size(); ++i)
			addConstraint(system->m_constraints[i]->m_constraint, true);

		// -10 allows RESET_PHYSICS down the calls. But equality with a float?...
		system->readTransform(system->prepareForRead(RESET_PHYSICS));

		system->m_world = this;
	}

	void SkinnedMeshWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		auto idx = std::find(m_systems.begin(), m_systems.end(), system);
		if (idx == m_systems.end())
			return;

		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				if (j->m_constraint)
					removeConstraint(j->m_constraint);

		for (int i = 0; i < system->m_meshes.size(); ++i)
			removeCollisionObject(system->m_meshes[i].get());
		for (int i = 0; i < system->m_constraints.size(); ++i)
			if (system->m_constraints[i]->m_constraint)
				removeConstraint(system->m_constraints[i]->m_constraint);
		for (int i = 0; i < system->m_bones.size(); ++i)
			removeRigidBody(&system->m_bones[i]->m_rig);

		std::swap(*idx, m_systems.back());
		m_systems.pop_back();

		system->m_world = nullptr;
	}

	void SkinnedMeshWorld::updateConstraintsForBone(SkinnedMeshBone* bone)
	{
		if (!bone)
			return;

		int numConstraints = int(m_constraints.size());
		for (int i = 0; i < numConstraints; i++) {
			btTypedConstraint* constraint = m_constraints[i];

			if (&constraint->getRigidBodyA() == &bone->m_rig ||
				&constraint->getRigidBodyB() == &bone->m_rig) {
				bool bothKinematic = constraint->getRigidBodyA().isStaticOrKinematicObject() &&
				                     constraint->getRigidBodyB().isStaticOrKinematicObject();

				constraint->setEnabled(!bothKinematic);
			}
		}
	}

	int SkinnedMeshWorld::stepSimulation(btScalar remainingTimeStep, int, btScalar fixedTimeStep)
	{
		applyGravity();
		if (hdt::SkyrimPhysicsWorld::get()->m_enableWind)
			applyWind(remainingTimeStep);

		while (remainingTimeStep > fixedTimeStep) {
			internalSingleStepSimulation(fixedTimeStep);
			remainingTimeStep -= fixedTimeStep;
		}

		// For the sake of the bullet library, we don't manage a step that would be lower than a 300Hz frame.
		// Review this when (screens / Skyrim) will allow 300Hz+.
		// Note: We are taking a final variable-sized step for the remaining time.
		// Because Bullet's constraint solvers (ERP/CFM) are sensitive to delta-time,
		// this variable tick can cause constraints to behave a bit differently
		// (appearing more stiff or damping differently at various framerates).
		constexpr auto minPossiblePeriod = 1.0f / 300.0f;
		if (remainingTimeStep > minPossiblePeriod)
			internalSingleStepSimulation(remainingTimeStep);
		clearForces();

		_bodies.clear();
		_shapes.clear();

		return 0;
	}

	// --Todo: This, and the systems related to it, can be optimized a bit more. I WILL BE BACK...
	// This optimizes Bullet's broadphase by removing tons of redudent work. We don't use persistent manifolds,
	// we don't have static objects, etc..
	// HOWEVER: If we ever add non-skinned Bullet collision objects, or make (0,0) bodies participate in
	// broadphase queries/collision, this optimization must be revisited.
	void SkinnedMeshWorld::performDiscreteCollisionDetection()
	{
		BT_PROFILE("performDiscreteCollisionDetection");

		for (auto& system : m_systems) {
			system->internalUpdate();
		}

		btDispatcherInfo& dispatchInfo = getDispatchInfo();

		for (int i = 0; i < m_collisionObjects.size(); i++) {
			btCollisionObject* colObj = m_collisionObjects[i];
			btBroadphaseProxy* proxy = colObj->getBroadphaseHandle();

			if (proxy->m_collisionFilterGroup == 0 && proxy->m_collisionFilterMask == 0)
				continue;

			btVector3 minAabb, maxAabb;
			colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb, maxAabb);

			m_broadphasePairCache->setAabb(proxy, minAabb, maxAabb, m_dispatcher1);
		}

		m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);

		if (m_dispatcher1) {
			m_dispatcher1->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(), dispatchInfo, m_dispatcher1);
		}
	}

	void SkinnedMeshWorld::applyGravity()
	{
		for (auto& i : m_systems) {
			for (auto& j : i->m_bones) {
				auto body = &j->m_rig;
				if (!body->isStaticOrKinematicObject() && !(body->getFlags() & BT_DISABLE_WORLD_GRAVITY)) {
					body->setGravity(m_gravity * j->m_gravityFactor);
				}
			}
		}

		btDiscreteDynamicsWorldMt::applyGravity();
	}

	void SkinnedMeshWorld::updateWindField()
	{
		BT_PROFILE("HDTSMP_sampleWindField");
		// Drop last update's system pointers even when wind was just disabled.
		m_windFieldSystems.clear();
		m_windFieldPositions.clear();
		m_windFieldSamples.clear();
		if (!SkyrimPhysicsWorld::get()->m_enableWind)
			return;
		const auto provider = OpenShadersWind::GetActiveInterface();
		if (!provider)
			return;

		for (const auto& system : m_systems) {
			// Share one field sample across the system, centered on the bones that receive wind.
			btVector3 center(0.f, 0.f, 0.f);
			uint32_t count = 0;
			for (const auto& bone : system->m_bones) {
				if (bone->m_rig.isStaticOrKinematicObject() || btFuzzyZero(bone->m_windFactor))
					continue;
				center += bone->m_rig.getWorldTransform().getOrigin();
				++count;
			}
			if (count == 0)
				continue;
			// This runs before applyTranslationOffset, so the center is in Skyrim world coordinates.
			center /= static_cast<btScalar>(count);
			if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z()))
				continue;
			m_windFieldSystems.push_back(static_cast<SkyrimSystem*>(system.get()));
			m_windFieldPositions.push_back({ center.x(), center.y(), center.z() });
		}

		m_windFieldSamples.resize(m_windFieldPositions.size());
		// Keep each provider request within its batch limit; failed batches contribute no wind.
		for (size_t offset = 0; offset < m_windFieldPositions.size();) {
			const auto count = static_cast<uint32_t>(std::min(m_windFieldPositions.size() - offset,
				static_cast<size_t>(CSPluginAPI::CSWindMaximumBatchSize)));
			if (!provider->SampleWind(m_windFieldPositions.data() + offset, m_windFieldSamples.data() + offset, count))
				std::fill_n(m_windFieldSamples.data() + offset, count, CSPluginAPI::WindSample{});
			offset += count;
		}
	}

	void SkinnedMeshWorld::applyWindField(btScalar timeStep)
	{
		constexpr btScalar kMaxFrameStep = 0.1f;
		constexpr double kTwoPi = 6.283185307179586;
		constexpr double kFlutterFrequencyHz = 2.0;
		constexpr double kSecondaryFrequencyHz = 3.1;
		constexpr btScalar kMaxFrequencyIncrease = 0.75f;
		constexpr btScalar kSidewaysAmplitude = 0.06f;
		constexpr btScalar kVerticalAmplitude = 0.02f;
		const btVector3 spatialPhaseScale(0.012f, 0.017f, 0.02f);
		constexpr btScalar kMaxRelativeResponse = 1.4f;
		const auto world = SkyrimPhysicsWorld::get();
		// API vectors are normalized wind responses, not velocities in Skyrim units per second.
		const auto forceScale = world->m_windStrength * scaleSkyrim;
		// Read the menu settings once per update rather than once per bone.
		const auto response = world->m_fieldVelocityResponse.load(std::memory_order_relaxed);
		const auto amount = world->m_fieldGustFlutter.load(std::memory_order_relaxed);
		const auto rate = world->m_fieldMotionRate.load(std::memory_order_relaxed);
		const auto offset = world->m_fieldTorqueOffset.load(std::memory_order_relaxed);
		const auto flutterStep = clampScalar(timeStep, 0.0f, kMaxFrameStep) * rate;
		const btVector3 up(0.f, 0.f, 1.f);
		for (size_t index = 0; index < m_windFieldSystems.size(); ++index) {
			const auto system = m_windFieldSystems[index];
			const auto& sample = m_windFieldSamples[index];
			// Keep short-lived forces such as shockwaves separate from the ambient wind.
			const btVector3 ambient(sample.baseVelocity.x + sample.gustVelocity.x,
				sample.baseVelocity.y + sample.gustVelocity.y, sample.baseVelocity.z + sample.gustVelocity.z);
			const btVector3 transient(sample.transientVelocity.x, sample.transientVelocity.y, sample.transientVelocity.z);
			// The actor's shelter factor attenuates ambient wind only; a transient may originate on another side.
			const auto ambientForce = ambient * (system->m_windFactor * forceScale);
			const auto transientForce = transient * forceScale;
			// A bad API sample must not poison Bullet's body state.
			if (!std::isfinite(ambientForce.x()) || !std::isfinite(ambientForce.y()) || !std::isfinite(ambientForce.z()) ||
				!std::isfinite(transientForce.x()) || !std::isfinite(transientForce.y()) || !std::isfinite(transientForce.z()))
				continue;
			// Calm ambient wind skips normalization and flutter, while transients can still act on the bones.
			const auto magnitude = ambientForce.length();
			const bool hasAmbient = !btFuzzyZero(magnitude);
			const auto direction = hasAmbient ? ambientForce / magnitude : btVector3(0.f, 0.f, 0.f);
			// Vertical wind has no horizontal crosswind direction; use X as its lateral axis.
			auto side = direction.cross(up);
			if (btFuzzyZero(side.length2()))
				side = btVector3(1.f, 0.f, 0.f);
			else
				side.normalize();
			const auto lift = side.cross(direction);
			// Integrate the rate: multiplying elapsed time by changing wind strength would jump the phase.
			const auto shelteredStrength = (ambient * system->m_windFactor).length();
			const auto frequencyScale = 1.f + kMaxFrequencyIncrease * shelteredStrength / (1.f + shelteredStrength);
			system->m_windFieldFlutterTime += static_cast<double>(flutterStep) * frequencyScale;
			// Reduce the phase before converting to float so long sessions do not lose small time steps.
			const auto phase = static_cast<btScalar>(std::fmod(system->m_windFieldFlutterTime * kTwoPi * kFlutterFrequencyHz, kTwoPi));
			const auto secondaryPhase = static_cast<btScalar>(std::fmod(system->m_windFieldFlutterTime * kTwoPi * kSecondaryFrequencyHz, kTwoPi));
			const auto& position = m_windFieldPositions[index];
			const btVector3 center(position.x, position.y, position.z);
			for (const auto& bone : system->m_bones) {
				// A zero XML wind factor opts this bone out of both ambient wind and transients.
				if (bone->m_rig.isStaticOrKinematicObject() || btFuzzyZero(bone->m_windFactor))
					continue;
				auto windForce = ambientForce;
				btVector3 pressureOffset(0.f, 0.f, 0.f);
				if (hasAmbient) {
					// Preserve FSMP's velocity response: less push downwind, up to 40% more against the wind.
					const auto relativeScale = response == 0.f ? 1.f : clampScalar((ambientForce - bone->m_rig.getLinearVelocity() * response).dot(direction) / magnitude, 0.f, kMaxRelativeResponse);
					windForce = ambientForce * relativeScale;
					// Avoid waveform work when both flutter and offset-force torque are disabled.
					if (amount > 0.f || offset > 0.f) {
						// Reconstruct world coordinates so recentering cannot shift flutter phases.
						const auto origin = bone->m_rig.getWorldTransform().getOrigin() + m_windFieldOriginOffset;
						// Nearby bones stay in phase; using local offsets avoids jumps when the actor moves or wind turns.
						const auto spatialPhase = (origin - center).dot(spatialPhaseScale);
						const auto flutter = std::sin(phase + spatialPhase);
						const auto secondaryFlutter = std::sin(secondaryPhase - spatialPhase);
						// Both components are perpendicular to the wind; gust strength comes only from the sampled field.
						windForce += (side * (flutter * kSidewaysAmplitude) + lift * (secondaryFlutter * kVerticalAmplitude)) * (magnitude * amount);
						// Flutter amount also scales oscillating torque; zero leaves only the steady pressure offset.
						pressureOffset = (side * (flutter * amount) + lift * (0.35f + secondaryFlutter * 0.25f * amount)) * offset;
					}
				}
				// Bullet integrates these forces over the substeps; do not multiply them by delta time here.
				bone->m_rig.applyForce(windForce * bone->m_windFactor, pressureOffset);
				// Transients bypass shelter and flutter, and act at the center without added torque.
				bone->m_rig.applyCentralForce(transientForce * bone->m_windFactor);
			}
		}
	}

	void SkinnedMeshWorld::applyWind(btScalar timeStep)
	{
		constexpr btScalar kMaxFrameStep = 0.1f;
		constexpr btScalar kTimeWrap = 4096.0f;
		constexpr btScalar kResponseToBodyVelocity = 0.08f;
		constexpr btScalar kGustSpeedPerForce = 7.0f;
		constexpr btScalar kMinGustSpeed = 180.0f;
		constexpr btScalar kMaxGustSpeed = 1200.0f;
		constexpr btScalar kCrosswindTimeScale = 0.004f;
		constexpr btScalar kVerticalPhaseScale = 0.006f;
		constexpr btScalar kPressureCenterOffset = 0.8f;

		BT_PROFILE("HDTSMP_applyWind");
		if (OpenShadersWind::IsEnabled()) {
			applyWindField(timeStep);
			return;
		}

		const btScalar windMagnitude = m_windSpeed.length();
		if (btFuzzyZero(windMagnitude))
			return;

		m_windTime += clampScalar(timeStep, 0.0f, kMaxFrameStep);
		if (m_windTime > kTimeWrap)
			m_windTime -= kTimeWrap;

		const btVector3 windDirection = m_windSpeed / windMagnitude;
		const btVector3 up(0.0f, 0.0f, 1.0f);
		btVector3 side = windDirection.cross(up);
		if (btFuzzyZero(side.length2())) {
			side = btVector3(1.0f, 0.0f, 0.0f);
		} else {
			side.normalize();
		}

		const btScalar gustSpeed = clampScalar(windMagnitude * kGustSpeedPerForce, kMinGustSpeed, kMaxGustSpeed);

		for (auto& i : m_systems) {
			auto system = static_cast<SkyrimSystem*>(i.get());
			if (btFuzzyZero(system->m_windFactor))  // skip any systems that aren't affected by wind
				continue;
			for (auto& j : i->m_bones) {
				auto body = &j->m_rig;
				if (body->isStaticOrKinematicObject() || btFuzzyZero(j->m_windFactor))
					continue;

				const btScalar windFactor = j->m_windFactor * system->m_windFactor;
				const btVector3 origin = body->getWorldTransform().getOrigin();
				// Move gust phases downwind so stronger wind carries the same gust across bones faster
				const btScalar advectedTime = m_windTime - origin.dot(windDirection) / gustSpeed - origin.dot(side) * kCrosswindTimeScale;
				const btScalar verticalPhase = origin.getZ() * kVerticalPhaseScale;

				const btScalar longGust = std::sin(advectedTime * 0.55f + verticalPhase * 0.35f);
				const btScalar midGust = std::sin(advectedTime * 1.35f + verticalPhase + longGust * 0.5f);
				const btScalar flutter = std::sin(advectedTime * 4.15f + verticalPhase * 2.2f + midGust);
				const btScalar gustPulse = 0.5f + 0.5f * std::sin(advectedTime * 0.85f + verticalPhase * 0.6f);
				const btScalar gustBurst = gustPulse * gustPulse;
				const btScalar gustScale = clampScalar(0.68f + longGust * 0.18f + midGust * 0.12f + flutter * 0.06f + gustBurst * 0.45f, 0.35f, 1.55f);

				const btScalar relativeScale = clampScalar((m_windSpeed - body->getLinearVelocity() * kResponseToBodyVelocity).dot(windDirection) / windMagnitude,
					0.0f,
					1.4f);

				const btScalar baseMagnitude = windMagnitude * windFactor;
				const btScalar sidewaysFlutter = (midGust * 0.13f + flutter * 0.06f + gustBurst * 0.04f) * baseMagnitude;
				const btScalar verticalFlutter = (std::sin(advectedTime * 1.9f + verticalPhase * 1.4f) * 0.018f + flutter * 0.01f) * baseMagnitude;

				const btVector3 windForce =
					m_windSpeed * windFactor * gustScale * relativeScale +
					side * sidewaysFlutter +
					up * verticalFlutter;

				const btVector3 pressureOffset = (side * midGust + up * (0.35f + flutter * 0.25f)) * kPressureCenterOffset;
				body->applyForce(windForce, pressureOffset);
			}
		}
	}

	void SkinnedMeshWorld::predictUnconstraintMotion(btScalar timeStep)
	{
		struct UpdaterPredictUnconstraintMotion : public btIParallelForBody
		{
			btScalar timeStep;
			btRigidBody** rigidBodies;

			void forLoop(int iBegin, int iEnd) const BT_OVERRIDE
			{
				for (int i = iBegin; i < iEnd; ++i) {
					btRigidBody* body = rigidBodies[i];

					// not realistic, just an approximate
					if (!body->isStaticOrKinematicObject())
						body->applyDamping(timeStep);

					body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
				}
			}
		};

		if (m_nonStaticRigidBodies.size() > 0) {
			UpdaterPredictUnconstraintMotion update;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];

			const int grainSize = 100;
			btParallelFor(0, m_nonStaticRigidBodies.size(), grainSize, update);
		}
	}

	void SkinnedMeshWorld::integrateTransforms(btScalar timeStep)
	{
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto body = m_collisionObjects[i];
			if (body->isKinematicObject()) {
				btTransformUtil::integrateTransform(
					body->getWorldTransform(),
					body->getInterpolationLinearVelocity(),
					body->getInterpolationAngularVelocity(),
					timeStep,
					body->getInterpolationWorldTransform());
				body->setWorldTransform(body->getInterpolationWorldTransform());
			}
		}

		btVector3 limitMin(-1e+9f, -1e+9f, -1e+9f);
		btVector3 limitMax(1e+9f, 1e+9f, 1e+9f);
		for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
			btRigidBody* body = m_nonStaticRigidBodies[i];
			auto lv = body->getLinearVelocity();
			lv.setMax(limitMin);
			lv.setMin(limitMax);
			body->setLinearVelocity(lv);

			auto av = body->getAngularVelocity();
			av.setMax(limitMin);
			av.setMin(limitMax);
			body->setAngularVelocity(av);
		}

		btDiscreteDynamicsWorldMt::integrateTransforms(timeStep);
	}

	void SkinnedMeshWorld::calculateSimulationIslands()
	{
		BT_PROFILE("calculateSimulationIslands");
		getSimulationIslandManager()->updateActivationState(getCollisionWorld(), getCollisionWorld()->getDispatcher());

		auto unionFind = &getSimulationIslandManager()->getUnionFind();

		for (int i = 0; i < m_predictiveManifolds.size(); i++) {
			btPersistentManifold* manifold = m_predictiveManifolds[i];
			const btCollisionObject* colObj0 = manifold->getBody0();
			const btCollisionObject* colObj1 = manifold->getBody1();
			if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
				((colObj1) && (!(colObj1)->isStaticOrKinematicObject()))) {
				unionFind->unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
			}
		}

		int numConstraints = int(m_constraints.size());
		for (int i = 0; i < numConstraints; i++) {
			btTypedConstraint* constraint = m_constraints[i];
			if (constraint->isEnabled()) {
				const btRigidBody* colObj0 = &constraint->getRigidBodyA();
				const btRigidBody* colObj1 = &constraint->getRigidBodyB();
				if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
					((colObj1) && (!(colObj1)->isStaticOrKinematicObject()))) {
					unionFind->unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
				}
			}
		}

		// If two dynamic bones are colliding, they MUST be processed by the same island! Without this the solver will cause an
		// EXCEPTION_ACCESS_VIOLATION reading m_tmpSolverBodyPool :(
		btDispatcher* dispatcher = getCollisionWorld()->getDispatcher();
		int numManifolds = dispatcher->getNumManifolds();
		for (int i = 0; i < numManifolds; i++) {
			btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);

			// Only unite if they are actively generating contact points
			if (manifold->getNumContacts() > 0) {
				const btCollisionObject* colObj0 = static_cast<const btCollisionObject*>(manifold->getBody0());
				const btCollisionObject* colObj1 = static_cast<const btCollisionObject*>(manifold->getBody1());

				if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
					((colObj1) && (!(colObj1)->isStaticOrKinematicObject()))) {
					unionFind->unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
				}
			}
		}

		getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());
	}

	// Island-based constraint solving...
	// btDiscreteDynamicsWorldMt::solveConstraints decomposes the world into independent
	// simulation islands and dispatches each to a solver from the pool on separate threads.
	void SkinnedMeshWorld::solveConstraints(btContactSolverInfo& solverInfo)
	{
		BT_PROFILE("solveConstraints");
		if (!m_collisionObjects.size())
			return;

		// Kinematic objects should never be able to collide, or move. Because: They're kinematic?
		// This avoids broken configs, API misuse - etc. Better to check it every cycle for safety
		for (int i = 0; i < m_constraints.size(); i++) {
			btTypedConstraint* constraint = m_constraints[i];
			if (constraint->isEnabled()) {
				if (constraint->getRigidBodyA().isStaticOrKinematicObject() &&
					constraint->getRigidBodyB().isStaticOrKinematicObject()) {
					constraint->setEnabled(false);
				}
			}
		}

		btDiscreteDynamicsWorldMt::solveConstraints(solverInfo);

		// the HDT manifolds are still recreated every frame, clear to prevent stale data.
		static_cast<CollisionDispatcher*>(m_dispatcher1)->clearAllManifold();
	}
}
