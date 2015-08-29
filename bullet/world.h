#pragma once

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <cold-md/contract/cold-md.h>
#include <memory>

namespace cold {
    namespace bullet {
        class world : public ::cold::contract::world {
            class world_impl {
            public:
                std::unique_ptr<btDbvtBroadphase> m_col_broadphase;
                std::unique_ptr<btDefaultCollisionConfiguration> m_col_configuration;
                std::unique_ptr<btCollisionDispatcher> m_col_dispatcher;
                std::unique_ptr<btCollisionWorld> m_col_world;
            public:
                world_impl();
                ~world_impl();
            };

            world_impl _world;

            class handle : public ::cold::contract::handle {
                world* _bullet_world;
                std::unique_ptr<btCollisionObject> _obj;
                std::unique_ptr<btGImpactMeshShape> _shape;

                // Inherited via handle
                virtual void dispose() override;
                virtual void set_location(
                    double yaw_, double pitch_, double roll_, 
                    double x_, double y_, double z_) override;
                virtual void deactivate() override;
                virtual void activate() override;
                virtual bool is_active() const override;
            public:
                handle(
                    world* bullet_world_,
                    int triangle_count_, double* triangles_, int tri_stride_,
                    int indices_count, int* indices_, int ind_stride_);
                ~handle();
            };
        protected:
            btCollisionWorld* get_world()
            {
                return _world.m_col_world.get();
            }
        public:

            world();
            ~world();

            // Inherited via world
            virtual void dispose() override;
            virtual ::cold::contract::handle * add_mesh(
                int triangle_count_, double * triangles_, int tri_stride_, 
                int indices_count, int * indices_, int ind_stride_) override;
        };
    }
}