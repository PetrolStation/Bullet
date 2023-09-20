#pragma once

#include <glm/vec3.hpp>

namespace PetrolEngine {
    class Raycast {
    public:
        Raycast() = default;
        Raycast(const glm::vec3& origin, const glm::vec3& direction, float maxDistance = 1000.0f) {
            this->origin = origin;
            this->direction = direction;
            this->maxDistance = maxDistance;
        }

        glm::vec3 origin;
        glm::vec3 direction;
        float maxDistance;
    };
}
