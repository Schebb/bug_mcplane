# include <unistd.h>
# include <map>
# include <vector>
# include <iostream>
# include <chrono>

# include "Graphics.hpp"
# include <PxPhysicsAPI.h>


using namespace physx;
using EntityID = int;


//// Globals ////
PxDefaultAllocator			gAllocator;
PxDefaultErrorCallback		gErrorCallback;
PxFoundation*				gFoundation = nullptr;
PxDefaultCpuDispatcher*		gDispatcher = nullptr;
PxCooking*					gCooking = nullptr;
PxPhysics*					gPhysics = nullptr;
PxMaterial*					gPhysicsMaterial = nullptr;
PxScene* 					gPhysicsScene = nullptr;

const vec3 VEC3_ZERO = vec3(0.f, 0.f, 0.f);

struct Entity
{
	vec3 			position 	= vec3(1.f, 1.f, 1.f);
	quat 			rotation 	= quat(0.f, 0.f, 0.f, 1.f);
	vec3 			scale 		= vec3(1.f, 1.f, 1.f);

	mat4 			getModelMatrix( void ) {
		mat4 model = mat4_cast(rotation);
		model = model*glm::scale(mat4(1.f), scale);
		model = glm::translate(mat4(1.f), position)*model;
		return model;
	};
};

struct DynamicEntity : public Entity
{
	PxRigidDynamic*	body = nullptr;
};

struct StaticEntity : public Entity
{
	PxRigidStatic*	body = nullptr;
};

std::map<int, DynamicEntity> dynamicEntities;

physx::PxVec3 	toPxVec3( vec3 v ) { return physx::PxVec3(v.x, v.y, v.z); }
physx::PxQuat 	toPxQuat( quat q ) { return physx::PxQuat(q.x, q.y, q.z, q.w); }
vec3 	toVec3( PxVec3 v ) { return vec3(v.x, v.y, v.z); }
quat 	toQuat( PxQuat q ) { return quat(q.w, q.x, q.y, q.z); }

bool 	initPhysics( void )
{
	if (gFoundation)
		return false; // already init

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = 
		&PxProfileZoneManager::createProfileZoneManager(gFoundation);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, 
			PxTolerancesScale(),true,profileZoneManager);

	gDispatcher = PxDefaultCpuDispatcherCreate(2);

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, 
			PxCookingParams(gPhysics->getTolerancesScale()));
	//PxCookingParams(toleranceScale));

	if (!gCooking)
		std::cout << "PxCreateCooking failed!" << std::endl;

	gPhysicsMaterial = 
		gPhysics->createMaterial(0.5f, 0.5f, 0.6f); //static friction, dynamic friction, restitution

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gPhysicsScene = gPhysics->createScene(sceneDesc);

	return true;
}

void 	deinitPhysics( void )
{
	if (gFoundation == nullptr)
		return;

	gPhysicsScene->release();

	gDispatcher->release();
	PxProfileZoneManager* profileZoneManager = gPhysics->getProfileZoneManager();

	gPhysics->release();	
	profileZoneManager->release();
	gCooking->release();
	gFoundation->release();

	gDispatcher = nullptr;
	gPhysics = nullptr;
	gCooking = nullptr;
	gFoundation = nullptr;
}

DynamicEntity& 	addEntityBox( EntityID eid, float mass, vec3 halfsize, vec3 position )
{
	DynamicEntity& e = dynamicEntities[eid];

	e.scale = halfsize * 2.f;
	e.position = position;

	PxTransform pxtr(PxVec3(position.x, position.y, position.z), PxQuat(PxIdentity));
	e.body = gPhysics->createRigidDynamic(pxtr);
	e.body->createShape( PxBoxGeometry(halfsize.x, halfsize.y, halfsize.z), *gPhysicsMaterial );
	e.body->userData = (void*)(uintptr_t)eid;

	PxRigidBodyExt::updateMassAndInertia(*e.body, 10.f);
	e.body->setMass(mass);

	gPhysicsScene->addActor(*e.body);

	return e;
}


void 	updateStates( void )
{
	//TODO: improve by directly calling getActors with a huge static buffer
	PxU32 nbActors = gPhysicsScene->getNbActors(
			PxActorTypeSelectionFlag::eRIGID_DYNAMIC);// | PxActorTypeSelectionFlag::eRIGID_STATIC);

	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		gPhysicsScene->getActors(
				PxActorTypeSelectionFlag::eRIGID_DYNAMIC/* | PxActorTypeSelectionFlag::eRIGID_STATIC*/, 
				(PxActor**)&actors[0], nbActors);

		for (PxRigidActor* actor : actors)
		{
			PxTransform localTm = actor->getGlobalPose();
			EntityID eid = (EntityID)(size_t)actor->userData;
			if (eid)
			{ // box
				DynamicEntity& e = dynamicEntities[eid];
				e.position = toVec3(localTm.p);
				e.rotation = toQuat(localTm.q);
			}
		}
	}
}


StaticEntity* 	ground = nullptr;
void 		initGround( vec3 halfsize, vec3 position )
{
	ground = new StaticEntity();
	StaticEntity& e = *ground;
	e.scale = halfsize * 2.f;
	e.position = position;

	PxTransform pxtr(PxVec3(position.x, position.y, position.z), PxQuat(PxIdentity));
	e.body = gPhysics->createRigidStatic(pxtr);
	e.body->createShape( PxBoxGeometry(halfsize.x, halfsize.y, halfsize.z), *gPhysicsMaterial );

	gPhysicsScene->addActor(*e.body);
}

void 	addFixedJoint( int eidA, vec3 posA, int eidB, vec3 posB )
{
	DynamicEntity& entityA = dynamicEntities[eidA];
	DynamicEntity& entityB = dynamicEntities[eidB];

	PxTransform otherPXTr = entityB.body->getGlobalPose();
	PxTransform meAnchor( toPxVec3(posA), PxQuat(PxIdentity) );
	PxTransform otherAnchor( toPxVec3(posB), PxQuat(PxIdentity) );

	PxTransform newMeTr = meAnchor.getInverse() * otherPXTr * otherAnchor;

	entityA.body->setGlobalPose(newMeTr);

	PxFixedJoint* joint = PxFixedJointCreate(
			*gPhysics, entityB.body, otherAnchor, entityA.body, meAnchor);
				
	joint->setConstraintFlag( PxConstraintFlag::eCOLLISION_ENABLED, false );
	entityA.body->setLinearVelocity(PxVec3(0, 0, 0));
	entityA.body->setAngularVelocity(PxVec3(0, 0, 0));
}

PxRevoluteJoint* 	addRevoluteJoint( int eidA, vec3 posA, int eidB, vec3 posB )
{
	DynamicEntity& entityA = dynamicEntities[eidA];
	DynamicEntity& entityB = dynamicEntities[eidB];

	PxTransform otherPXTr = entityB.body->getGlobalPose();
	PxTransform meAnchor( toPxVec3(posA), PxQuat(PxIdentity) );
	PxTransform otherAnchor( toPxVec3(posB), PxQuat(PxIdentity) );

	PxTransform newMeTr = meAnchor.getInverse() * otherPXTr * otherAnchor;

	entityA.body->setGlobalPose(newMeTr);

	PxRevoluteJoint* joint = PxRevoluteJointCreate(
			*gPhysics, entityB.body, otherAnchor, entityA.body, meAnchor);
				
	joint->setConstraintFlag( PxConstraintFlag::eCOLLISION_ENABLED, false );
	entityA.body->setLinearVelocity(PxVec3(0, 0, 0));
	entityA.body->setAngularVelocity(PxVec3(0, 0, 0));

	float _limit = 0.6f;
	joint->setLimit(PxJointAngularLimitPair(-_limit, _limit));//, 0.01f));
	joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
	joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
	joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_FREESPIN, false);
	joint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
	joint->setConstraintFlag(PxConstraintFlag::eREPORTING, true);
	//joint->setDriveGearRatio(0.f);
	joint->setDriveForceLimit(1000.f);
	joint->setDriveVelocity(-100.f);

	return joint;
}


void 	scriptPropulsor( DynamicEntity& e, float power )
{
	PxRigidDynamic& dyn = *e.body;

	vec3 force = e.rotation * vec3(0.f, 0.f, -power);
	dyn.addForce(toPxVec3(force), PxForceMode::eFORCE);
}

void 	scriptWing( DynamicEntity& e, float _lift, float _drag )
{
	PxVec3 liftDir(0, 1, 0);

	PxRigidDynamic& dyn = *e.body;

	// Parameters
	quat rotation = e.rotation;
	vec3 forwardDir = normalize(rotation * vec3(0, 0, -1));
	vec3 upDir = normalize(rotation * vec3(0, 1, 0));
	vec3 rightDir = normalize(rotation * vec3(1, 0, 0));

	vec3 linearVel = toVec3(dyn.getLinearVelocity());
	float velocity = length(linearVel);
	vec3 linearDir = normalize(linearVel);
	float atmosDensity = 1.225f; // kg/m3 (earth, sea level, 15°C)
	float staticAirPressure = 101.325f; // Pa, or N/m2 (earth, sea level, see US Standard Atmosphere)
	float cosAoa = dot(linearDir, forwardDir);
	float cosStupidAoa = dot(linearDir, upDir);


	// Equations:
	// Lift = CrossProduct(velocity, wingRight) * Cos(StupidAoA) * (1 - Abs(Cos(StupidAoA))) * Cos(AoA) * deflectionLiftCoeff * StaticAirPressure;
	// Drag = Abs(cos(StupidAoA))*NativeDragCoefficient;

	//// Drag Force ////
	// dragCoeff = Abs(cos(StupidAoA))*NativeDragCoefficient;
	// Fdrag = 0.5 * atmosDensity  * pow(velocity, 2) * dragCoeff

	float nativeDragCoeff = _drag;
	float dragCoeff = abs(cosStupidAoa) * nativeDragCoeff;

	float dragForce = 0.5f * atmosDensity * pow(velocity, 2.f) * dragCoeff;
	PxVec3 drag = toPxVec3(-linearDir) * dragForce;
	if (drag.magnitudeSquared() > 0.f)
		dyn.addForce(drag, PxForceMode::eFORCE);


	//// Lift Force ////
	// Lift = CrossProduct(velocity, wingRight) * Cos(StupidAoA) * (1 - Abs(Cos(StupidAoA))) * Cos(AoA) * deflectionLiftCoeff * StaticAirPressure;
	// see simplified 2d graph: cos(x + pi/2) * (1 - abs(cos(x + pi/2))) * cos(x)

	float liftCoeff = _lift;
	vec3 liftForce = cross(linearVel, rightDir) * cosStupidAoa * (1.f - abs(cosStupidAoa)) * cosAoa * liftCoeff * staticAirPressure;

	if (length(liftForce) > 0.f)
		dyn.addForce(toPxVec3(liftForce), PxForceMode::eFORCE);
}


int 	main ( void )
{
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
	{
		std::cerr << "failed to load SDL. (everything)";
		return 1;
	}

	Graphics graphics;

	if (graphics.init(1280, 720) == false)
		return 1;

	if (initPhysics() == false)
		return 0;

	initGround(vec3(90.f, 0.5f, 90.f), VEC3_ZERO);

	addEntityBox(316, 10.f, vec3(8.f, 0.25f, 1.5f), vec3(0.f, 3.f, 0.f));

	addEntityBox(315, 40.f, vec3(2.f, 1.f, 2.f), VEC3_ZERO);
	addFixedJoint(315, vec3(0.f, 0.f, 2.f), 316, VEC3_ZERO);

	addEntityBox(317, 20.f, vec3(1.f, 1.f, 1.5f), VEC3_ZERO);
	addFixedJoint(317, vec3(0.f, 0.f, -2.f), 316, VEC3_ZERO);

	addEntityBox(319, 2.f, vec3(2.5f, 0.25f, 0.25f), VEC3_ZERO);
	PxRevoluteJoint* revoA = addRevoluteJoint(319, VEC3_ZERO, 316, vec3(-4.5f, 0.f, 1.5f));

	addEntityBox(318, 2.f, vec3(2.5f, 0.25f, 0.25f), VEC3_ZERO);
	PxRevoluteJoint* revoB = addRevoluteJoint(318, VEC3_ZERO, 316, vec3(4.5f, 0.f, 1.5f));

	addEntityBox(320, 1.f, vec3(2.5f, 0.25f, 0.5f), VEC3_ZERO);
	addFixedJoint(320, vec3(0.f, 0.f, -0.8f), 318, vec3(0.f, 0.f, 0.25f));

	addEntityBox(321, 1.f, vec3(2.5f, 0.25f, 0.5f), VEC3_ZERO);
	addFixedJoint(321, vec3(0.f, 0.f, -0.8f), 319, vec3(0.f, 0.f, 0.25f));

	// If you comment this it works. But I don't think it is because THIS specific
	// box (more about the number of allocated joints/or shapes/ or dynamics)
	addEntityBox(112, 1.f, vec3(0.5f, 0.5f, 0.5f), VEC3_ZERO);
	addFixedJoint(112, vec3(0.f, -2.f, 0.f), 315, vec3(0.f, 0.f, 0.f));

	auto t0 = std::chrono::high_resolution_clock::now();
	while (true)
	{
		SDL_Event 	ev;
		SDL_PollEvent( &ev );
		if (ev.type == SDL_QUIT || (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE))
			break;

		auto t1 = std::chrono::high_resolution_clock::now();
		if (std::chrono::duration<float>(t1-t0).count() > 1.f)
		{
			revoA->setDriveVelocity(0.f);
			revoB->setDriveVelocity(0.f);
		}
		scriptWing(dynamicEntities[316], 10.f, 10.f);
		scriptWing(dynamicEntities[320], 0.5f, 0.5f);
		scriptWing(dynamicEntities[321], 0.5f, 0.5f);
		//scriptPropulsor(dynamicEntities[317], 1200.f);
		scriptPropulsor(dynamicEntities[317], 720.f);

		gPhysicsScene->simulate(1.f/60.f);
		gPhysicsScene->fetchResults(true);

		updateStates();

		graphics.clear();

		// Ground
		graphics.drawBox(ground->getModelMatrix(), Color(0.2f, 0.2f, 1.f));

		auto it = dynamicEntities.begin();
		while (it != dynamicEntities.end())
		{
			Entity& e = it->second;
			graphics.drawBox(e.getModelMatrix(), Color(1.f, 0.2f, 0.2f));

			++it;
		}

		graphics.refresh();
		usleep(1000);
	}

	graphics.deinit();
	deinitPhysics();

	SDL_Quit();

	return 0;
}

