#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }

        Vector2D dist(0,0);
        if(num_nodes > 1)  dist = (end - start) / (num_nodes - 1);
        Vector2D pos = start;
        for(int i=0; i < num_nodes; i++){
            Mass* m = new Mass(pos, node_mass, false);
            this->masses.push_back(m);
            if(i>0){
                Spring* s = new Spring(masses[i-1], masses[i], k);
                this->springs.push_back(s);
            }
            pos = pos + dist;
        }
        for(auto &i: pinned_nodes)
            masses[i]->pinned = true;

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            float k = s->k;
            Vector2D apos = s->m1->position, bpos = s->m2->position;
            Vector2D force, dir = bpos-apos;
            force = k * dir / dir.norm() * (dir.norm()-s->rest_length);
            s->m1->forces += force;
            s->m2->forces += (-1) * force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                m->velocity += a * delta_t;
                
                // TODO (Part 2): Add global damping
                float damping = 0.00005;
                m->velocity *= (1-damping);
                m->position += m->velocity * delta_t;
                m->start_position = m->position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            float k = s->k;
            Vector2D apos = s->m1->position, bpos = s->m2->position;
            Vector2D force, dir = bpos-apos;
            force = k * dir / dir.norm() * (dir.norm()-s->rest_length);
            s->m1->forces += force;
            s->m2->forces += (-1) * force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;

                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;

                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.00005;
                m->position = temp_position + (1-damping_factor) * (temp_position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
