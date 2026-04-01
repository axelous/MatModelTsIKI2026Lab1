#include "PhysicsEngine.h"

#define SAFE_RELEASE(obj) {	\
	if (obj) {				\
		obj->release();		\
		obj = nullptr;		\
	}						\
}

#define IS_FLOATS_EQUAL(f1, f2) (fabsf((f1) - (f2)) < 1e-5)

void CustomEventCallback::onTrigger(physx::PxTriggerPair* pairs, uint32_t count) {
	extern PhysicsEngine* physicsEngine;
	for (uint32_t i = 0; i < count; i++) {
		const physx::PxTriggerPair& pair = pairs[i];
		switch (pair.status) {
		case physx::PxPairFlag::eNOTIFY_TOUCH_FOUND:
			physicsEngine->MarkActor(pair.otherActor);
			break;
		default:
			break;
		}
	}
}

PhysicsEngine::PhysicsEngine() {
	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocatorCallback, errorCallback);

#ifdef USE_PVD
	pvd = physx::PxCreatePvd(*foundation);
	transport = physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10000);
	if (!pvd || !transport || !pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL)) {
		std::cout << "[WARNING] Could not initialize or connect pvd\n";
	}

	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, pvd);
	PxInitExtensions(*physics, pvd);
#else
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), false);
#endif

	physx::PxSceneDesc sceneDesc = physx::PxSceneDesc(physics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	dispatcher =
		physx::PxDefaultCpuDispatcherCreate(2);

	sceneDesc.cpuDispatcher =
		dispatcher;
	sceneDesc.filterShader = CustomFilterShader;
	sceneDesc.filterShaderData = &filterShaderData;
	sceneDesc.filterShaderDataSize = sizeof(filterShaderData);
	sceneDesc.simulationEventCallback = &eventCallback;

	scene = physics->createScene(sceneDesc);

#ifdef USE_PVD
	if (pvd && pvd->isConnected()) {
		physx::PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
		if (pvdClient) {
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
		}
	}
#endif

	filterShaderData.status = CustomFilterShaderData::Status::eCOLLISION;
}

PhysicsEngine::~PhysicsEngine()
{
	std::vector<physx::PxRigidActor*> actors =
		GetActors();

	for (auto* actor : actors)
	{
		SAFE_RELEASE(actor);
	}

	uint32_t materialsNum =
		physics->getNbMaterials();

	if (materialsNum > 0)
	{
		std::vector<physx::PxMaterial*> materials(materialsNum);

		physics->getMaterials(
			reinterpret_cast<physx::PxMaterial**>(materials.data()),
			materialsNum
		);

		for (auto* material : materials)
		{
			SAFE_RELEASE(material);
		}
	}

	SAFE_RELEASE(scene);
	SAFE_RELEASE(dispatcher);
	PxCloseExtensions();
	SAFE_RELEASE(physics);

#ifdef USE_PVD
	if (pvd && pvd->isConnected())
	{
		pvd->disconnect();
	}

	SAFE_RELEASE(pvd);
	SAFE_RELEASE(transport);

	SAFE_RELEASE(transport);
#endif

	SAFE_RELEASE(foundation);
}

void PhysicsEngine::Simulate(float elapsedTime) {
	scene->simulate(elapsedTime);
	scene->fetchResults(true);

	for (physx::PxActor* actor : markedActors) {
		RemoveActor(actor);
		SAFE_RELEASE(actor);
	}
	markedActors.clear();
}

physx::PxMaterial* PhysicsEngine::CreateMaterial(float staticFriction, float dynamicFriction, float restitution) {
	return physics->createMaterial(staticFriction, dynamicFriction, restitution);
}

physx::PxMaterial* PhysicsEngine::GetMaterial(float staticFriction, float dynamicFriction, float restitution) {
	uint32_t materialsNum = physics->getNbMaterials();
	if (materialsNum > 0) {
		std::vector<physx::PxMaterial*> materials(materialsNum);
		physics->getMaterials(reinterpret_cast<physx::PxMaterial**>(materials.data()), materialsNum);
		for (auto* material : materials) {
			if (IS_FLOATS_EQUAL(material->getStaticFriction(), staticFriction)
				&& IS_FLOATS_EQUAL(material->getDynamicFriction(), dynamicFriction)
				&& IS_FLOATS_EQUAL(material->getRestitution(), restitution)) {
				return material;
			}
		}
	}
	return CreateMaterial(staticFriction, dynamicFriction, restitution);
}

physx::PxShape* PhysicsEngine::CreateBoxShape(
	physx::PxVec3 size,
	physx::PxMaterial* material,
	CustomFilterData filterData,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxBoxGeometry geometry = physx::PxBoxGeometry(size / 2.0);
	physx::PxShape* shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);

	physx::PxFilterData data(filterData, 0, 0, 0);
	shape->setSimulationFilterData(data);

	return shape;
}

physx::PxShape* PhysicsEngine::CreateSphereShape(
	float radius,
	physx::PxMaterial* material,
	CustomFilterData filterData,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxSphereGeometry geometry = physx::PxSphereGeometry(radius);
	physx::PxShape* shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);

	physx::PxFilterData data(filterData, 0, 0, 0);
	shape->setSimulationFilterData(data);

	return shape;
}

physx::PxShape* PhysicsEngine::CreateCapsuleShape(
	float radius,
	float size,
	physx::PxMaterial* material,
	CustomFilterData filterData,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxCapsuleGeometry geometry = physx::PxCapsuleGeometry(radius, size / 2.0f);
	physx::PxShape* shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);

	physx::PxFilterData data(filterData, 0, 0, 0);
	shape->setSimulationFilterData(data);

	return shape;
}

physx::PxRigidStatic* PhysicsEngine::AddGround(physx::PxVec3 normal, float distance, physx::PxMaterial* material) {
	physx::PxPlane plane = physx::PxPlane(IS_FLOATS_EQUAL(normal.magnitudeSquared(), 1.0f) ? normal : normal.getNormalized(), distance);
	physx::PxRigidStatic* groundPlane = PxCreatePlane(*physics, plane, *material);
	scene->addActor(*groundPlane);
	return groundPlane;
}

physx::PxRigidStatic* PhysicsEngine::AddStaticActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation) {
	physx::PxRigidStatic* actor = physics->createRigidStatic(physx::PxTransform(position, rotation));
	actor->attachShape(*shape);
	scene->addActor(*actor);
	return actor;
}

physx::PxRigidDynamic* PhysicsEngine::AddDynamicActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation, float density) {
	physx::PxRigidDynamic* actor = physics->createRigidDynamic(physx::PxTransform(position, rotation));
	actor->attachShape(*shape);
	physx::PxRigidBodyExt::updateMassAndInertia(*actor, density);
	scene->addActor(*actor);
	return actor;
}

std::vector<physx::PxRigidActor*> PhysicsEngine::GetActors(physx::PxActorTypeFlags types) {
	uint32_t actorsNum = scene->getNbActors(types);
	std::vector<physx::PxRigidActor*> actors(actorsNum);
	scene->getActors(types, reinterpret_cast<physx::PxActor**>(actors.data()), actorsNum);
	return actors;
}

void PhysicsEngine::RemoveActor(physx::PxActor* actor) {
	scene->removeActor(*actor);
}

void PhysicsEngine::MarkActor(physx::PxActor* actor) {
	markedActors.push_back(actor);
}

void PhysicsEngine::SetFilterShaderConstantBlock(bool value) {
	filterShaderData.status = value ? CustomFilterShaderData::Status::eSKIP : CustomFilterShaderData::Status::eCOLLISION;
	scene->setFilterShaderData(&filterShaderData, sizeof(filterShaderData));
}

bool PhysicsEngine::GetFilterShaderConstantBlock() const {
	return filterShaderData.status == CustomFilterShaderData::Status::eSKIP;
}

physx::PxFilterFlags PhysicsEngine::CustomFilterShader(
	physx::PxFilterObjectAttributes attributes0,
	physx::PxFilterData filterData0,
	physx::PxFilterObjectAttributes attributes1,
	physx::PxFilterData filterData1,
	physx::PxPairFlags& pairFlags,
	const void* constantBlock,
	uint32_t constantBlockSize
) {
	if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1)) {
		pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
		return physx::PxFilterFlag::eDEFAULT;
	}

	pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;

	bool hasCueStick = (filterData0.word0 == CustomFilterData::eCUE_STICK
		|| filterData1.word0 == CustomFilterData::eCUE_STICK);

	if (hasCueStick) {
		bool hasCueBall = (filterData0.word0 == CustomFilterData::eCUE_BALL
			|| filterData1.word0 == CustomFilterData::eCUE_BALL);
		if (!hasCueBall) {
			return physx::PxFilterFlag::eKILL;
		}

		const CustomFilterShaderData* shaderData = static_cast<const CustomFilterShaderData*>(constantBlock);
		if (sizeof(*shaderData) == constantBlockSize && shaderData->status == CustomFilterShaderData::Status::eSKIP) {
			return physx::PxFilterFlag::eKILL;
		}
	}

	return physx::PxFilterFlag::eDEFAULT;
}