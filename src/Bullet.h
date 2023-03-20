#pragma once

#include <iostream>
#include <unistd.h>

#include "Bullet/deps/bullet3/Extras/VHACD/inc/vhacdMesh.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btBulletDynamicsCommon.h"

#include <Core/Components/Component.h>

namespace PetrolEngine {
    class Collider: public Component {
    public:
        btCollisionShape* shape;
        btDefaultMotionState* motionState;
        btScalar mass;
        btVector3 inertia;
        btRigidBody* rigidBody;
        bool localInertia;
        
        Collider(int m, bool ci){
            mass = m;
            localInertia = ci;
            inertia = btVector3(1, 1, 1);
        }
        virtual btCollisionShape* getShape() = 0;
        
        void onStart();
        void onUpdate();

        ~Collider();
    };

    class PlaneCollider: public Collider {
    public:
        PlaneCollider(int m, bool ci) : Collider(m, ci) {};
        btCollisionShape* getShape();
    };

    class MeshCollider: public Collider {
    public:
        MeshCollider(int m, bool ci) : Collider(m, ci) {};
        btCollisionShape* getShape();
    };

    class BoxCollider: public Collider {
    public:
        BoxCollider(int m, bool ci) : Collider(m, ci) {};
        btCollisionShape* getShape();
    };
    
    class World {
    public:
        btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	
	World(){
		broadphase = new btDbvtBroadphase();
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);
		solver = new btSequentialImpulseConstraintSolver();
		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}

        void update(double t){
            dynamicsWorld->stepSimulation(t, 10);
        }

	~World(){
		delete dynamicsWorld;
                delete solver;
                delete dispatcher;
                delete collisionConfiguration;
                delete broadphase;
	}

    };
}
