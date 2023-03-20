#include "Bullet.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "Core/Components/MeshRenderer.h"
#include "LinearMath/btTransform.h"

#include <Core/Components/Entity.h>
#include <Core/Components/Transform.h>
#include <Core/Scene.h>
#include <Core/Components/Mesh.h>

namespace PetrolEngine {
	void Collider::onStart(){
	    auto& q = transform->rotation;
            auto& p = transform->position;
            auto& s = transform->scale;

	    shape = getShape();
            motionState = new btDefaultMotionState(btTransform(btQuaternion(q.x, q.y, q.z, q.w), btVector3(p.x, p.y, p.z)));

	    if(localInertia) shape->calculateLocalInertia(mass, inertia);

	    auto rigidBodyCI = btRigidBody::btRigidBodyConstructionInfo(mass, motionState, shape, inertia);
            rigidBody = new btRigidBody(rigidBodyCI);
	    entity->getScene()->world->dynamicsWorld->addRigidBody(rigidBody);
	}
	
	void Collider::onUpdate(){
		btTransform t;
		rigidBody->getMotionState()->getWorldTransform(t);

		this->transform->position.x = t.getOrigin().getX();
	        this->transform->position.y = t.getOrigin().getY();
		this->transform->position.z = t.getOrigin().getZ();

		this->transform->rotation.x = t.getRotation().getX();
		this->transform->rotation.y = t.getRotation().getY();
		this->transform->rotation.z = t.getRotation().getZ();
		this->transform->rotation.w = t.getRotation().getW();
	}

	Collider::~Collider(){
	    entity->getScene()->world->dynamicsWorld->removeRigidBody(rigidBody);
	    delete rigidBody->getMotionState();
	    delete rigidBody;
	    delete shape;
	}

	btCollisionShape* MeshCollider::getShape(){
	    btConvexHullShape* hull = new btConvexHullShape();
	    
	    auto& mesh = entity->getComponent<Mesh>();

	    auto& verts = mesh.vertices;
	    auto& indis = mesh.indices;

	    auto f = [](glm::vec3 a) { return btVector3(a.x,a.y,a.z); };
	    int hi = verts.size();
	    for (int i = 0; i < verts.size(); i += 1) hull->addPoint(f(verts[i]));
	    //for (int i = 0; i < indis.size(); i += 3) shape->addTriangle(indis[i], indis[i+1], indis[i+2]);

	    return (btCollisionShape*) hull;
	}

	btCollisionShape* BoxCollider::getShape(){
	    return new btBoxShape(btVector3(
		this->transform->scale.x,
		this->transform->scale.y,
		this->transform->scale.z
	    ));
	}

	btCollisionShape* PlaneCollider::getShape(){
	    return new btStaticPlaneShape(btVector3(0, 1, 0), 1);
	}

}
