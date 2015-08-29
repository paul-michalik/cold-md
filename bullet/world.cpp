#include <cold-md/bullet/world.h>

// TODO: Add precompiled headers
// TODO: move everything into implementation file later...

namespace cold {
    namespace bullet {
        void cold::bullet::world::handle::dispose()
        {
        }

        void world::handle::set_location(double yaw_, double pitch_, double roll_, double x_, double y_, double z_)
        {
        }

        void world::handle::deactivate()
        {
        }

        void world::handle::activate()
        {
        }

        bool world::handle::is_active() const
        {
            return false;
        }

        cold::bullet::world::handle::handle(
            world* bullet_world,
            int triangle_count_, double * triangles_, int tri_stride_, 
            int indices_count, int * indices_, int ind_stride_)
        {
        }

        world::world_impl::world_impl()
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

        world::world_impl::~world_impl() = default;
        
        world::world()
        {
        }

        world::~world()
        {
            // won't be default...
        }

        void world::dispose()
        {
            delete this;
        }

        ::cold::contract::handle * world::add_mesh(
            int triangle_count_, double * triangles_, int tri_stride_, 
            int indices_count, int * indices_, int ind_stride_)
        {
            return nullptr;
        }
    }
}