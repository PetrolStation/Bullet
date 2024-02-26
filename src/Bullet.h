#pragma once

#include <iostream>
#include <Static/Window/Window.h>
#include "Bullet/deps/bullet3/Extras/VHACD/inc/vhacdMesh.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btBulletDynamicsCommon.h"
#include <Core/Physics/ColliderApi.h>

#include <Core/Components/Component.h>
#include <Core/Physics/ColliderApi.h>
#include <Core/Physics/PhysicsController.h>

namespace PetrolEngine {
    class BulletController;

    class BulletCollider: public Collider3DApi {
    public:
        btCollisionShape* shape;
        btDefaultMotionState* motionState;
        btVector3 inertia;
        btRigidBody* rigidBody;
        BulletController* controller;
    
        BulletCollider(int m, bool ci){
            mass = m;
            localInertia = ci;
            inertia = btVector3(1, 1, 1);
        }
        virtual btCollisionShape* getShape() = 0;
        void applyForce(glm::vec3 force) {rigidBody->applyCentralForce(btVector3(force.x, force.y, force.z));}
        void onStart();
        void onUpdate();

         ~BulletCollider();
    };

    class BulletPlaneCollider: public BulletCollider {
    public:
        BulletPlaneCollider(int m, bool ci): BulletCollider(m, ci) {};
        btCollisionShape* getShape();
    };

    class BulletMeshCollider: public BulletCollider {
    public:
        BulletMeshCollider(int m, bool ci): BulletCollider(m, ci) {};
        btCollisionShape* getShape();
    };

    class BulletBoxCollider: public BulletCollider {
    public:
        BulletBoxCollider(int m, bool ci): BulletCollider(m, ci) {};
        btCollisionShape* getShape();
    };
   
    
    class BulletController: public Component {
    public:
        Collider3DApi* newPlaneCollider(int mass, bool localInertia, glm::vec3 inertia) { return new BulletPlaneCollider(mass, localInertia); }
        Collider3DApi*  newMeshCollider(int mass, bool localInertia, glm::vec3 inertia) { return new BulletMeshCollider (mass, localInertia); }
        Collider3DApi*   newBoxCollider(int mass, bool localInertia, glm::vec3 inertia) { return new BulletBoxCollider  (mass, localInertia); }

        btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	
	BulletController(){
		broadphase = new btDbvtBroadphase();
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);
		solver = new btSequentialImpulseConstraintSolver();
		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}

        void onUpdate(){
            dynamicsWorld->stepSimulation(deltaTime, 10);
        }

	~BulletController(){
		delete dynamicsWorld;
                delete solver;
                delete dispatcher;
                delete collisionConfiguration;
                delete broadphase;
	}

    };

    class BulletCreator: public PhysicsCreator3D {
    public:
        Component* newPhysicsController() { return new BulletController(); }
        Collider3DApi* newPlaneCollider(int mass, bool localInertia, glm::vec3 inertia) { return new BulletPlaneCollider(mass, localInertia); }
        Collider3DApi*  newMeshCollider(int mass, bool localInertia, glm::vec3 inertia) { return new BulletMeshCollider (mass, localInertia); }
        Collider3DApi*   newBoxCollider(int mass, bool localInertia, glm::vec3 inertia) { return new BulletBoxCollider  (mass, localInertia); }
    };
}
