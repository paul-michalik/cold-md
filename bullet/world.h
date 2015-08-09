#pragma once

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <cold-md/contract/cold-md.h>
#include <memory>

namespace cold {
    namespace bullet {
        class world : public ::cold::contract::world {
        protected:
            std::unique_ptr<btDbvtBroadphase> m_col_broadphase;
            std::unique_ptr<btDefaultCollisionConfiguration> m_col_configuration;
            std::unique_ptr<btCollisionDispatcher> m_col_dispatcher;
            std::unique_ptr<btCollisionWorld> m_col_world;
        protected:
            btCollisionWorld* get_world()
            {
                return m_col_world.get();
            }

            world* add_object(btTriangleIndexVertexArray const& p_triangles)
            {
                
            }

            virtual void dispose() override
            {
                delete this;
            }
        public:
            virtual ::cold::contract::world* add_object() override
            {
                return this;
            }

            world()
            {
                m_col_broadphase =
                    std::make_unique<btDbvtBroadphase>();

                {
                    m_col_configuration =
                        std::make_unique<btDefaultCollisionConfiguration>();

                    {
                        m_col_dispatcher =
                            std::make_unique<btCollisionDispatcher>(m_col_configuration.get());

                        btGImpactCollisionAlgorithm::registerAlgorithm(m_col_dispatcher.get());

                        {
                            m_col_world =
                                std::make_unique<btCollisionWorld>(
                                m_col_dispatcher.get(),
                                m_col_broadphase.get(),
                                m_col_configuration.get());
                        }
                    }
                }
            }

            ~world() = default;
        };
    }
}