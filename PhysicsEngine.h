#pragma once

#include <iostream>
#include <vector>
#include "PxPhysicsAPI.h"

#ifdef _DEBUG
#define USE_PVD
#define PVD_HOST "127.0.0.1"
#endif

class CustomEventCallback : public physx::PxSimulationEventCallback {
public:
	virtual void onConstraintBreak(physx::PxConstraintInfo* constraints, uint32_t count) override {};
	virtual void onWake(physx::PxActor** actors, uint32_t count) override {};
	virtual void onSleep(physx::PxActor** actors, uint32_t count) override {};
	virtual void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, uint32_t nbPairs) override {};
	virtual void onTrigger(physx::PxTriggerPair* pairs, uint32_t count) override;
	virtual void onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer, const uint32_t count) override {};
};

enum CustomFilterData {
	eDYNAMIC = 1,
	eOBSTACLE,
	eTRIGGER,
	eCUE_BALL,
	eCUE_STICK
};

struct CustomFilterShaderData {
	enum Status {
		eCOLLISION = 0,
		eSKIP
	};
	Status status;
};

class PhysicsEngine {
public:
	PhysicsEngine();
	~PhysicsEngine();
	void Simulate(float elapsedTime);
	physx::PxMaterial* GetMaterial(float staticFriction, float dynamicFriction, float restitution);
	physx::PxShape* CreateBoxShape(
		physx::PxVec3 size,
		physx::PxMaterial* material,
		CustomFilterData filterData,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);
	physx::PxShape* CreateSphereShape(
		float radius,
		physx::PxMaterial* material,
		CustomFilterData filterData,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);
	physx::PxShape* CreateCapsuleShape(
		float radius,
		float size,
		physx::PxMaterial* material,
		CustomFilterData filterData,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);
	physx::PxRigidStatic* AddGround(physx::PxVec3 normal, float distance, physx::PxMaterial* material);
	physx::PxRigidStatic* AddStaticActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation);
	physx::PxRigidDynamic* AddDynamicActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation, float density);
	std::vector<physx::PxRigidActor*> GetActors(physx::PxActorTypeFlags types = physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC);
	void MarkActor(physx::PxActor* actor);
	void SetFilterShaderConstantBlock(bool value);
	bool GetFilterShaderConstantBlock() const;

private:
	physx::PxDefaultAllocator allocatorCallback;
	physx::PxDefaultErrorCallback errorCallback;
	physx::PxFoundation* foundation;
	physx::PxPhysics* physics;
	physx::PxDefaultCpuDispatcher* dispatcher;
#ifdef USE_PVD
	physx::PxPvd* pvd;
	physx::PxPvdTransport* transport;
#endif
	physx::PxScene* scene;

	std::vector<physx::PxActor*> markedActors;
	CustomEventCallback eventCallback;
	CustomFilterShaderData filterShaderData;

	physx::PxMaterial* CreateMaterial(float staticFriction, float dynamicFriction, float restitution);
	void RemoveActor(physx::PxActor* actor);

	static physx::PxFilterFlags CustomFilterShader(
		physx::PxFilterObjectAttributes attributes0,
		physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1,
		physx::PxFilterData filterData1,
		physx::PxPairFlags& pairFlags,
		const void* constantBlock,
		uint32_t constantBlockSize
	);
};